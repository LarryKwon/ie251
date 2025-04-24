#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from enum import Enum, auto

class Status(Enum):
    FORWARD = auto()
    LEFT = auto()
    RIGHT = auto()

class MazeSolver:
    DESIRED_WALL = 0.5   # 오른쪽 벽과의 목표 거리 [m]
    FRONT_CLEAR  = 0.5      # 정면 턴 임계 [m]
    LIN_SPD      = 0.16
    ANG_SPD      = 1.0

    def __init__(self):
        rospy.init_node('maze_solver', anonymous=True)

        self.pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.cmd  = Twist()
        self.rate = rospy.Rate(10)

        self.front_range = 0.0
        self.left_range  = 0.0
        self.right_range = 0.0
        self.front_left_range = 0.0
        self.front_right_range = 0.0
        self.state = Status.FORWARD
        self.front = 0.0
        
        self.last_log_time = rospy.Time.now()

    # ---------- LaserScan 처리 ----------
    def scan_callback(self, data: LaserScan):
        rospy.loginfo_throttle(1, "debug scancallback called")
        ranges = np.array(data.ranges, dtype=np.float32)
        # if len(ranges) < 350:
        #     return
        

        max_range    = data.range_max or 3.5      # inf 대체값
        ranges[np.isinf(ranges)] = max_range

        # 각도(도) → 인덱스 변환 헬퍼
        to_idx = lambda deg: int((np.deg2rad(deg) - data.angle_min) /
                                 data.angle_increment) % len(ranges)

        # ±5° 범위 평균
        # def mean_deg(deg, width=1):
        #     idx = to_idx(deg)
        #     # rospy.loginfo(f"idx: {idx}")
        #     half = int(np.deg2rad(width) / data.angle_increment)
        #     if idx == 0:
        #         slc = ranges[idx-half:] + ranges[0:idx+half]
        #     else:
        #         slc = ranges[idx-half : idx+half+1]
        #     # rospy.loginfo(f"[DEGREE] Degree: {deg}, Datas: {slc}")
        #     return np.mean(slc)

        def mean_deg(deg, width=5):
            idx = to_idx(deg)
            half = int(np.deg2rad(width) / data.angle_increment)

            if idx - half < 0:
                slc = np.concatenate((ranges[idx - half:], ranges[:idx + half + 1]))
            elif idx + half + 1 > len(ranges):
                slc = np.concatenate((ranges[idx - half:], ranges[:(idx + half + 1) % len(ranges)]))
            else:
                slc = ranges[idx - half : idx + half + 1]

            return np.mean(slc) if len(slc) > 0 else data.range_max


        def mean_dist(start, end):
            sidx = to_idx(start)
            eidx = to_idx(end)
            slc = ranges[sidx:eidx]
            if sidx < eidx:
                slc = ranges[sidx:eidx]
            else:
                slc = np.concatenate((ranges[sidx:1], ranges[:eidx]))
            return np.mean(slc) if len(slc) > 0 else data.range_max

        self.front_range = mean_deg(0)
        # self.front_left_range = mean_deg(45)
        # self.left_range  = mean_deg(90)
        # self.right_range = mean_deg(270)
        # self.front_right_range = mean_deg(315)

        self.front_right_range = mean_dist(315,359)
        self.right_range = mean_dist(225,315)
        self.back_range = mean_dist(135,225)
        self.left_range = mean_dist(45,135)
        self.front_left_range = mean_dist(0,45)
        self.front = (self.front_right_range + self.front_left_range) / 2

        # ① 1 초마다 상태·샘플 ranges 로그
        if (rospy.Time.now() - self.last_log_time).to_sec() >= 1.0:
            rospy.loginfo(
                f"[LIDAR] F: {self.front:.2f}m R: {self.right_range:.2f}, B: {self.back_range:.2f}, L: {self.left_range:.2f}, ang: {self.cmd.angular.z}, vel: {self.cmd.linear.x}")
            self.last_log_time = rospy.Time.now()

    # ranges 축약 출력용
    @staticmethod
    def sample_ranges(ranges: np.ndarray, step_deg: int = 30):
        step = len(ranges) // (360 // step_deg)
        return ["{:.2f}".format(r) for r in ranges[::step]]

    # ---------- 기본 동작 ----------
    def move_forward(self):
        self.cmd.linear.x  = self.LIN_SPD
        self.cmd.angular.z = 0.0

    def turn_left(self):
        self.cmd.linear.x  = 0.0
        self.cmd.angular.z =  self.ANG_SPD


    def turn_right(self):
        self.cmd.linear.x  = 0.0
        self.cmd.angular.z = -self.ANG_SPD

    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.pub.publish(self.cmd)

    # ---------- 메인 루프 ----------
    def solve_maze(self):
        # 첫 LIDAR 수신 대기
        while not rospy.is_shutdown() and np.isclose(self.front_range, 0.0):
            rospy.loginfo_once("Waiting for first /scan message…")

            self.rate.sleep()

        while not rospy.is_shutdown():
            
            # if self.state == Status.FORWARD:
                
            # if self.state == Status.LEFT:
            
            # if self.state == Status.RIGHT:

            if self.front <= self.FRONT_CLEAR:
                self.turn_left()
            # elif self.right_range > self.DESIRED_WALL + 0.2:
            #     self.turn_right()
            elif self.front > self.FRONT_CLEAR and self.right_range < self.DESIRED_WALL:
                self.move_forward()
            else:
                self.turn_right()

            self.pub.publish(self.cmd)
            # rospy.loginfo("cmd")
            self.rate.sleep()


if __name__ == '__main__':
    try:
        MazeSolver().solve_maze()
    except rospy.ROSInterruptException:
        pass
