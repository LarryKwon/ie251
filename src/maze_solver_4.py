#!/usr/bin/env python3
"""
maze_solver.py
==============

- RobotControl : /cmd_vel 퍼블리시 + /scan 구역 최소값 계산
- MazeSolver   : 왼쪽-벽 추종  ➜  스텝 전진-판단-회전 반복
                 + 충돌 완화(앞·뒤·옆 붙었을 때 살짝 전/후진)

로봇 크기 : 0.20 m × 0.20 m
통로 폭   : 0.30 m  (=> 실제 여유 ≈ 0.10 m)
"""
import rospy, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# ───────── 하드웨어/환경 파라미터 ─────────
LIN_SPEED   = 0.18         # 일반 전진 속도 (m/s)
ANG_SPEED   = 0.7          # 회전 속도 (rad/s)
STEP_TIME   = 0.40         # 직진 step 1회 시간  (s)
TURN_TIME   = 0.75         # 코너에서 회전 유지 (s)
BUMP_TIME   = 0.30         # 충돌 완화 전/후진 길이 (s)
SAFE_FWD    = 0.3        # 앞 안전 거리 (m) → 이보다 가까우면 회전
SAFE_LEFT   = 0.2         # 왼쪽 벽 붙임 판단
COLLISION   = 0.12         # 부딪힘 판정(어느 방향이든)
HZ          = 20           # 제어 루프 Hz
# ────────────────────────────────────────


# ═══════════ 저수준 제어 ═════════════════
class RobotControl:
    def __init__(self):
        rospy.init_node("maze_solver_node", anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self._scan_cb)

        self.tw   = Twist()
        self.reg  = {"front": 1, "left": 0.1, "right": 0.4, "back": 0.1}
        self.rate = rospy.Rate(HZ)
        rospy.on_shutdown(self.stop)

    # -- LiDAR 콜백 -------------------------------------------------
    @staticmethod
    def _min_valid(arr):
        vals = [d for d in arr if not math.isinf(d)]
        return min(vals) if vals else 3.5

    def _scan_cb(self, msg: LaserScan):
        r = msg.ranges  # 0°=앞, 반시계 +
        self.reg = {
            "front": self._min_valid(list(r[0:25]) + list(r[-25:])),
            "left" : self._min_valid(r[60:120]),
            "right": self._min_valid(r[240:300]),
            "back" : self._min_valid(r[170:190]),
        }

    # -- 속도 퍼블리시 ----------------------------------------------
    def publish_vel(self, lin, ang, dur):
        end = rospy.Time.now() + rospy.Duration.from_sec(dur)
        self.tw.linear.x  = lin
        self.tw.angular.z = ang
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.pub.publish(self.tw)
            self.rate.sleep()

    def stop(self):
        self.publish_vel(0, 0, 0.05)


# ═══════════ 미로 로직 ═══════════════════
class MazeSolver:
    def __init__(self, rc: RobotControl):
        self.rc    = rc
        self.state = "FORWARD"

    # -- 충돌 완화 primitive ----------------------------------------
    def _bump_back(self):
        self.rc.publish_vel(-LIN_SPEED, 0, BUMP_TIME)
        self.rc.stop()

    def _bump_forward(self):
        self.rc.publish_vel( LIN_SPEED, 0, BUMP_TIME)
        self.rc.stop()

    # -- 동작 primitive --------------------------------------------
    def _forward_step(self):
        self.rc.publish_vel(LIN_SPEED, 0, STEP_TIME)
        self.rc.stop()

    def _turn_left(self):
        self.rc.publish_vel(0,  ANG_SPEED, TURN_TIME)
        self.rc.stop()

    def _turn_right(self):
        self.rc.publish_vel(0, -ANG_SPEED, TURN_TIME)
        self.rc.stop()

    # -- 메인 FSM --------------------------------------------------
    def loop(self):
        rospy.loginfo("=== MazeSolver 시작 (left-hand, collision safe) ===")
        r = rospy.Rate(HZ)
        while not rospy.is_shutdown():
            f, l, rt, b = (self.rc.reg[k] for k in ("front", "left", "right", "back"))

            # ── ① 충돌 완화 단계 ─────────────────────────────────
            if f < COLLISION:        # 앞이 벽에 붙음
                rospy.loginfo("BUMP front %.2f --> 뒤로", f)
                self._bump_back()
                continue
            if b < COLLISION:        # 뒤에 끼임
                rospy.loginfo("BUMP back %.2f --> 앞으로", b)
                self._bump_forward()
                continue
            if l < COLLISION:        # 왼쪽이 너무 붙어 회전 어려움
                rospy.loginfo("BUMP left %.2f --> 살짝 앞으로", l)
                self._bump_forward()
                continue
            if rt < COLLISION:       # 오른쪽이 너무 붙음
                rospy.loginfo("BUMP right %.2f --> 살짝 앞으로", rt)
                self._bump_forward()
                continue

            # ── ② 정상 FSM ─────────────────────────────────────
            if self.state == "FORWARD":
                if l > SAFE_LEFT:              # 왼쪽이 비었음 → 벽 찾기
                    self.state = "TURN_LEFT"
                elif f < SAFE_FWD:             # 앞 막힘 → 우회전
                    self.state = "TURN_RIGHT"
                else:
                    self._forward_step()

            elif self.state == "TURN_LEFT":
                self._turn_left()
                self._forward_step()
                self.state = "FORWARD"

            elif self.state == "TURN_RIGHT":
                self._turn_right()
                self._forward_step()
                self.state = "FORWARD"

            # ── 로깅 (0.2 s) ───────────────────────────────────
            rospy.loginfo_throttle(
                0.2, f"[{self.state}] f={f:.2f} l={l:.2f} r={rt:.2f} b={b:.2f}"
            )
            r.sleep()


# ═════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    ctrl = RobotControl()
    try:
        MazeSolver(ctrl).loop()
    except rospy.ROSInterruptException:
        pass
