#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Maze solver for TurtleBot3 waffle
알고리즘
1) 시작 시   : 좌하(225°)·우하(315°) 값 차이가 tol 이하가 될 때까지 회전(수평 정렬)
2) RUN_LOOP :
   ① 전방 0.3 m 이상 → 전진
   ② 0.3 m 미만     → 더 먼 쪽(좌·우) 선택 → 90° 정지-회전
   ③ 회전 완료 후   : 선택한 쪽의 대각선 두 값(좌회전→우상·우하, 우회전→좌상·좌하)을
                      tol 내로 맞출 때까지 미세 회전
"""
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum, auto


class Phase(Enum):
    ALIGN   = auto()   # 시작 수평 맞추기
    FORWARD = auto()   # 직진
    TURN    = auto()   # 90° 도는 중
    ADJUST  = auto()   # 대각선으로 자세 보정
    AVOID = auto()


class MazeSolver:
    ## ======= 하이퍼파라미터 ========
    LIN_SPD        = 0.1        # m/s
    ANG_SPD_ALIGN  = 0.2         # rad/s  (정렬용 저속)
    ANG_SPD_TURN   = 1.0         # rad/s  (90° 회전용)
    FRONT_THRESH   = 0.34        # m  (벽 감지 거리)
    DIAG_TOL       = 0.02        # m  (대각선 일치 허용값)
    ALIGN_TOL      = 0.03        # m  (초기 수평 맞춤 허용값)
    SIDE_NEAR  = 0.18
    SIDE_CLEAR = 0.4
    adjust_history = []
    adjust_count = 0
    ## =================================

    def __init__(self):
        rospy.init_node('maze_solver', anonymous=True)

        # pub/sub
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)

        # 센서 캐시
        self.ranges = None
        self.laser  = None

        # 상태
        self.phase      = Phase.ALIGN
        self.turn_left  = True          # 직전에 선택한 회전 방향 (ADJUST 단계에서 사용)
        self.turn_start = None          # 회전 시작 시간 (90° 타이머용)

        self.cmd  = Twist()
        self.rate = rospy.Rate(10)


    # ---------- LaserScan 처리 ----------
    def scan_cb(self, scan: LaserScan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges[np.isinf(ranges)] = scan.range_max
        self.ranges, self.laser = ranges, scan


    # ---------- 각도별 평균 레인지 ----------
    def min_deg(self, deg, width=10):
        """
        LaserScan에서 deg(도) 중심 ±width° 구간의 평균 거리.
        np.take + mod 연산으로 경계(−180↔+180) 래핑을 깔끔하게 처리.
        """
        if self.laser is None:
            return np.inf

        scan = self.laser
        n     = len(self.ranges)
        idx   = int((np.deg2rad(deg) - scan.angle_min) / scan.angle_increment) % n

        half  = max(1, int(np.deg2rad(width) / scan.angle_increment))   # 최소 1 샘플
        # 중심 idx 기준 −half‥+half 인덱스 집합을 mod n 로 래핑
        idxs  = (np.arange(-half, half + 1) + idx) % n
        slc   = np.take(self.ranges, idxs)     # 길이 2·half + 1

        return np.min(slc)

    def mean_deg(self, deg, width=10):
        """
        LaserScan에서 deg(도) 중심 ±width° 구간의 평균 거리.
        np.take + mod 연산으로 경계(−180↔+180) 래핑을 깔끔하게 처리.
        """
        if self.laser is None:
            return np.inf

        scan = self.laser
        n     = len(self.ranges)
        idx   = int((np.deg2rad(deg) - scan.angle_min) / scan.angle_increment) % n

        half  = max(1, int(np.deg2rad(width) / scan.angle_increment))   # 최소 1 샘플
        # 중심 idx 기준 −half‥+half 인덱스 집합을 mod n 로 래핑
        idxs  = (np.arange(-half, half + 1) + idx) % n
        slc   = np.take(self.ranges, idxs)     # 길이 2·half + 1

        return np.mean(slc)


    # ---------- 모션 API ----------
    def publish(self, lin, ang):
        self.cmd.linear.x  = lin
        self.cmd.angular.z = ang
        self.cmd_pub.publish(self.cmd)

    def stop(self):
        self.publish(0.0, 0.0)

    # ------------------------------------
    def run(self):
        # Laser 메시지 올 때까지 대기
        while not rospy.is_shutdown() and self.laser is None:
            rospy.loginfo_throttle(1.0, "Waiting for /scan …")
            self.rate.sleep()

        rospy.loginfo("Start maze run!")

        # self.do_align()

        # while self.min_deg(0,25) > self.FRONT_THRESH:
        #     self.publish(self.LIN_SPD, 0.0)
        #     self.phase = Phase.TURN
        
        # self.stop()
        # l = self.min_deg(90)
        # r = self.min_deg(270)
        # self.turn_left = l > r
        # direction = "LEFT" if self.turn_left else "RIGHT"
        # # rospy.loginfo(f"벽 감지 (front={front:.2f}) → {direction} 90° 회전")
        # self.turn_start = rospy.Time.now()
        # self.phase = Phase.TURN

        while not rospy.is_shutdown():
            # ===== 상태 머신 =====
            if self.phase == Phase.ALIGN:
                self.do_align()

            elif self.phase == Phase.FORWARD:
                self.do_forward()

            elif self.phase == Phase.TURN:
                self.do_turn()

            elif self.phase == Phase.ADJUST:
                self.do_adjust()

            elif self.phase == Phase.AVOID:
                self.do_avoid()
            self.rate.sleep()


    # --- Phase 동작 ---
    def do_align(self):
        dl = self.mean_deg(135,2)
        dr = self.mean_deg(225,2)
        diff = dr - dl                      # + → 오른쪽이 더 멀다
        if abs(diff) < self.ALIGN_TOL:
            rospy.loginfo("↔ 수평 정렬 완료 → FORWARD")
            self.phase = Phase.FORWARD
            self.stop()
            return
        ang = -self.ANG_SPD_ALIGN if diff > 0 else self.ANG_SPD_ALIGN
        self.publish(0.0, ang)

    def do_forward(self):
        front = self.min_deg(0,25)
        left  = self.min_deg(90,  4)
        right = self.min_deg(270, 4)
        if front > self.FRONT_THRESH:
            if (left < self.SIDE_NEAR) ^ (right < self.SIDE_NEAR):
                self.stop()
                self.turn_left = right < left        # 가까운 쪽 반대 방향
                self.phase = Phase.AVOID
                rospy.loginfo(f"벽이랑 가까움 → AVOID {self.turn_left}")
                return

            # (1-b) 정상 전진
            self.publish(self.LIN_SPD, 0.0)
            return

        # 벽 접근 → 정지 후 회전 방향 결정
        self.stop()
        l = self.min_deg(90)
        r = self.min_deg(270)
        self.turn_left = l > r
        direction = "LEFT" if self.turn_left else "RIGHT"
        rospy.loginfo(f"벽 감지 (front={front:.2f}) → {direction} 90° 회전")
        self.turn_start = rospy.Time.now()
        self.phase = Phase.TURN

    def do_turn(self):
        # 90° = π/2 rad
        elapsed = (rospy.Time.now() - self.turn_start).to_sec()
        need    = (np.pi / 2) / self.ANG_SPD_TURN
        if elapsed < need:
            ang =  self.ANG_SPD_TURN if self.turn_left else -self.ANG_SPD_TURN
            self.publish(0.0, ang)
            return

        self.stop()
        rospy.loginfo("90° 회전 완료 → ADJUST")
        self.phase = Phase.ADJUST
    
    def do_avoid(self):
        rospy.loginfo(f"벽에서 멀어지기 {self.turn_left}")
        left  = self.min_deg(90, 4)
        right = self.min_deg(270, 4)

        # 회피 회전 방향 (가까운 쪽 반대)
        ang =  self.ANG_SPD_ALIGN if self.turn_left else -self.ANG_SPD_ALIGN
        self.publish(0.05, ang)
        
        # 충분히 멀어졌으면 FORWARD 로 복귀
        if left >= self.SIDE_NEAR and right >= self.SIDE_NEAR:
            rospy.loginfo("측면 회피 완료 → FORWARD")
            self.stop()
            self.phase = Phase.ADJUST

    def do_adjust(self):
        # 좌회전했으면 우상(45)·우하(315), 우회전했으면 좌상(135)·좌하(225)
        if self.turn_left:        # 좌회전 → 오른쪽 대각선 일치
            upper = self.mean_deg(295,1)
            lower = self.mean_deg(245,1)
            ang   = -self.ANG_SPD_ALIGN
        else:                     # 우회전 → 왼쪽 대각선 일치
            upper = self.mean_deg(65,1)
            lower = self.mean_deg(115,1)
            ang   =  self.ANG_SPD_ALIGN
        diff = upper - lower
        if abs(diff) < self.DIAG_TOL or self.adjust_count >= 10:
            rospy.loginfo("대각선 보정 완료 → FORWARD")
            self.phase = Phase.FORWARD
            self.stop()
            self.adjust_count = 0
            return

        # diff > 0 이면 upper가 더 멀다 → 해당 방향으로 조금 더 회전
        sign = np.sign(diff)
        self.publish(0.0, sign * ang)
        self.adjust_count += 1


if __name__ == '__main__':
    try:
        MazeSolver().run()
    except rospy.ROSInterruptException:
        pass
