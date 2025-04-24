#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

### ───────────────── RobotControl ─────────────────
class RobotControl:
    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb)
        self.cmd = Twist(); self.scan = LaserScan()
        self.rate = rospy.Rate(10)                # 10 Hz
        self.ctrl_c = False
        rospy.on_shutdown(self._shutdown)
        rospy.wait_for_message('/scan', LaserScan)

    def _scan_cb(self, msg): self.scan = msg
    def _shutdown(self): self.ctrl_c = True; self.stop()
    def ranges(self):      return self.scan.ranges

    def _publish_for(self, sec):
        ticks = int(sec*10)
        for _ in range(ticks):
            self.pub.publish(self.cmd)
            self.rate.sleep()

    def stop(self):
        self.cmd = Twist(); self._publish_for(0.1)

    # ───── 이동 / 회전 ─────
    def move(self, forward=True, v=0.15, t=0.5):
        self.cmd.linear.x, self.cmd.angular.z = (v if forward else -v), 0
        self._publish_for(t); self.stop()

    def turn(self, dir_, w=0.3, t=1.5):
        cw = (dir_ == "right")      # dir_ ∈ {"left","right"}
        self.cmd.linear.x, self.cmd.angular.z = 0, (-w if cw else w)
        self._publish_for(t); self.stop()

### ───────────────── Robo ─────────────────
SECTOR = {"front":(-25,25), "left":(60,120), "back":(150,210), "right":(240,300)}
def smin(r, s, e): return min(r[s:]+r[:e]) if s<0 else min(r[s:e])

TH_BLOCK=0.30; TH_CLEAR=0.50

class Robo:
    def __init__(self, ctrl:RobotControl):
        self.c = ctrl
        self.turn_history = []    # 최근 3회 회전 방향 기억

    def log_turn(self, dir_:str):
        self.turn_history.append(dir_)
        if len(self.turn_history) > 3:
            self.turn_history.pop(0)

    def check_pingpong(self, new_dir:str)->bool:
        """L-R-L / R-L-R 패턴이면 True"""
        self.log_turn(new_dir)
        if len(self.turn_history)==3:
            tri = ''.join(self.turn_history)
            if tri in ("LRL","RLR"):
                self.turn_history.clear()
                return True
        return False

    def decide_escape(self):
        r = self.c.ranges()
        d = {k: smin(r,*SECTOR[k]) for k in SECTOR}

        # ① 사방 다 막힘 → 180° 회전
        if all(x < TH_BLOCK for x in d.values()):
            self.c.turn("right", w=0.4, t=4); return

        # ② 앞이 막힘
        if d["front"] < TH_BLOCK:
            # 좌·우 중 넓은 쪽 선택
            dir_ = "left" if d["left"] > d["right"] else "right"

        # 🔸 핑퐁 감지 → 먼저 0.5 s 후진
        if self.check_pingpong(dir_):
            self.c.move(forward=False, v=0.15, t=0.5)

        self.c.turn(dir_, w=0.3, t=1.5)
        # return

        # ③ 뒤가 막힘(앞은 여유) → 1 s 전진
        if d["back"] < TH_BLOCK and d["front"] > TH_CLEAR:
            self.c.move(forward=True, t=1.0); return

        # ④ 왼쪽만 막힘 → 시계방향
        if d["left"] < TH_BLOCK:
            self.c.turn("right", t=1.0); return
        # ⑤ 오른쪽만 막힘 → 반시계방향
        if d["right"] < TH_BLOCK:
            self.c.turn("left",  t=1.0); return

    def loop(self):
        while not self.c.ctrl_c:
            self.decide_escape()                      # 충돌·회피 판단
            self.c.move(forward=True, t=0.2)          # 평상시 0.2 s 전진
            self.c.rate.sleep()

### ───────────────── main ─────────────────
if __name__ == "__main__":
    ctl = RobotControl()
    Robo(ctl).loop()
