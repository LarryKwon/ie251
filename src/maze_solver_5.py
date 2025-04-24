#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# ======= 튜닝 파라미터 =======
STEP_TIME   = 0.4      # s   한 번에 전진하는 시간
LINEAR_FWD  = 0.18     # m/s 전진 속도
ANG_LEFT    = 0.4      # rad/s 좌회전 속도
ANG_RIGHT   = 0.4      # rad/s 우회전 속도
TURN_TIME   = 0.5      # s   회전 지속 시간
SAFE_DIST   = 0.35     # m   왼쪽·앞 최소 허용 거리
# ============================

class MazeSolver:
    def __init__(self):
        rospy.init_node("maze_solver_step", anonymous=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        self.tw = Twist()
        self.reg = {"front": 1, "left": 1, "right": 1}
        self.state = "FORWARD"    # 초기 상태
        rospy.on_shutdown(self.stop_robot)

    # ---------- 센서 콜백 ----------
    def scan_cb(self, msg: LaserScan):
        r = msg.ranges
        self.reg = {
            "front": min(min(r[0:20]), min(r[-20:])),  # ±20°
            "left" : min(r[60:120]),                  # 90° ±30°
            "right": min(r[240:300])                  # -90° ±30°
        }

    # ---------- 모션 유틸 ----------
    def publish(self, lin, ang, dur):
        """lin/ang 속도를 dur초 동안 유지"""
        self.tw.linear.x  = lin
        self.tw.angular.z = ang
        end = rospy.Time.now() + rospy.Duration(dur)
        r = rospy.Rate(20)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.cmd_pub.publish(self.tw)
            r.sleep()

    def stop_robot(self):
        self.publish(0, 0, 0.1)

    # ---------- 메인 루프 ----------
    def run(self):
        rospy.loginfo("=== MazeSolver 시작 (left-hand rule) ===")
        while not rospy.is_shutdown():
            f = self.reg["front"]
            l = self.reg["left"]

            # 상태 결정 -------------------------------------------------
            if l > SAFE_DIST:               # 왼쪽이 비어 있다 → 왼쪽 벽 찾기
                self.state = "TURN_LEFT"
            elif f < SAFE_DIST:             # 앞이 막혔다 → 우회전
                self.state = "TURN_RIGHT"
            else:                           # 그대로 전진
                self.state = "FORWARD"

            # 로깅
            rospy.loginfo(f"[{self.state}]  front={f:.2f}  left={l:.2f}")

            # 상태별 액션 -----------------------------------------------
            if self.state == "FORWARD":
                self.publish(LINEAR_FWD, 0.0, STEP_TIME)

            elif self.state == "TURN_LEFT":
                self.publish(0.0, ANG_LEFT, TURN_TIME)

            elif self.state == "TURN_RIGHT":
                self.publish(0.0, -ANG_RIGHT, TURN_TIME)

            # 다음 루프에서 새 센서값으로 다시 판단 → 스텝-바이-스텝
        # while 끝
# ----------------------------------------------------------------------

if __name__ == "__main__":
    try:
        MazeSolver().run()
    except rospy.ROSInterruptException:
        pass
