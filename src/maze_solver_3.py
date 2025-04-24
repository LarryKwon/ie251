#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time


class RobotControl():

    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.publish_once_in_cmd_vel()

    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_laser(self, pos):
        # time.sleep(1)
        return self.laser_msg.ranges[pos]

    def get_laser_all(self, pos1, pos2):
        # time.sleep(1)
        if pos1 < 0:
            laser_all_1 = self.laser_msg.ranges[360+pos1:360]
            laser_all_2 = self.laser_msg.ranges[0:pos2]
            return sum(laser_all_1 + laser_all_2) / len(laser_all_1 + laser_all_2)
        else:
            return sum(self.laser_msg.ranges[pos1:pos2])/len(self.laser_msg.ranges[pos1:pos2])
    
    def get_laser_range(self, pos1, pos2):
        # time.sleep(1)
        if pos1 < 0:
            laser_all_1 = self.laser_msg.ranges[360+pos1:360]
            laser_all_2 = self.laser_msg.ranges[0:pos2]
            return laser_all_1 + laser_all_2
        else:
            return self.laser_msg.ranges[pos1:pos2]

    def get_front_laser(self):
        # time.sleep(1)
        return self.laser_msg.ranges[0]

    def get_laser_full(self):
        # time.sleep(1)
        return self.laser_msg.ranges

    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.1
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
            self.cmd.linear.x = speed
        elif motion == "backward":
            self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s


    def turn(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0


        if clockwise == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
        return s


# if __name__ == '__main__':
#     #rospy.init_node('robot_control_node', anonymous=True)
#     robotcontrol_object = RobotControl()
#     try:
#         robotcontrol_object.move_straight()

#     except rospy.ROSInterruptException:
#         pass




import time
# from robot_control_class import RobotControl #importa uma classe
robotcontrol = RobotControl() #criando um objeto

class Robo:
    SECTOR = {
        "front": (-25,  25),   # 50 도
        "left" : ( 60, 120),   # 60 도
        "back" : (150, 210),   # 60 도
        "right": (240, 300)    # 60 도
    }
    TH = 0.18                # m, 충돌 판정 임계값

    def __init__(self):
        print("inicializando...")
        self.laser1 = robotcontrol.get_laser_all(-5,5)
        self.robotmove_speed = 0.1
        self.robotmove_time = 0.5
        self.robotturn_clockwise = "clockwise"
        self.robotturn_speed = 0.2
        self.robotturn_time = 2
        # self.laser2 = robotcontrol.get_laser_all(-30, 30)

    def decide_escape(self):
    r = robotcontrol.get_laser_full()        # 최신 레이저 (list[360])
    d = {k: sector_min(r, *SECTOR[k]) for k in SECTOR}

    if all(d[k] < TH for k in d):            # 완전 갇힘
        robotcontrol.turn("clockwise", 0.4, 4)   # 180° 턴
        return

    # 앞이 막힘
    if d["front"] < TH:
        if d["left"] > d["right"]:
            robotcontrol.turn("aclockwise", 0.3, 2)   # 왼쪽 틀기
        else:
            robotcontrol.turn("clockwise", 0.3, 2)    # 오른쪽 틀기
        return

    # 뒤가 막힘 → 살짝 전진
    if d["back"] < TH:
        robotcontrol.move_straight_time("forward", 0.1, 1)
        return

    # 좌·우 개별 처리
    if d["left"] < TH:
        robotcontrol.turn("clockwise", 0.3, 1)
        return
    if d["right"] < TH:
        robotcontrol.turn("aclockwise", 0.3, 1)
        return

    
    def check_crash(self):
        front = robotcontrol.get_laser_range(-45,45)
        front_min = min(front)
        while front_min < 0.2:
            # 후방 거리 확인
            laser_back = robotcontrol.get_laser_all(175,185)
            print("뒤 거리: ", laser_back)
            print("앞 거리", front_min)
            if laser_back > 0.1:
                print("후진합니다...")
                robotcontrol.move_straight_time("backward", 0.2, 1)  # 3초간 후진
            else:
                print("뒤도 막혔습니다... 못 움직입니다.")
            self.laser1 = robotcontrol.get_laser_all(-5,5)
            front = robotcontrol.get_laser_range(-45,45)
            front_min = min(front)

        back = robotcontrol.get_laser_range(135,215)
        back_min = min(back)
        while back_min < 0.2:
            # 후방 거리 확인
            laser_front = robotcontrol.get_laser_all(-5,5)
            print("앞 거리: ", laser_front)
            print("뒤 최소 거리", back_min)
            if laser_front > 0.1:
                print("전진합니다...")
                robotcontrol.move_straight_time("backward", 0.2, 1)  # 3초간 후진
            else:
                print("앞도 막혔습니다... 못 움직입니다.")
            self.laser1 = robotcontrol.get_laser_all(-5,5)
            back = robotcontrol.get_laser_range(135,215)
            back_min = min(back)
        # robotcontrol.stop_robot()

    def robotmove(self):
        # while self.laser1 > 0.7 and all(e >= 0.25 for e in self.laser2):
        while self.laser1 > 0.4:
            robotcontrol.move_straight_time("forward",0.1,1)
            self.laser1 = robotcontrol.get_laser_all(-5,5)
            # self.laser2 = robotcontrol.get_laser_all(-30, 30)
            print("distance: ", self.laser1)
            print("ang vel", robotcontrol.cmd.angular.z)
        self.check_crash()
        robotcontrol.stop_robot()
        

    def robotturn(self):
        distancia_right = robotcontrol.get_laser_all(265,275)
        distancia_left = robotcontrol.get_laser_all(85,95)
        print("right",distancia_right, "left", distancia_left)
        if distancia_right > distancia_left:
            while self.laser1 < 0.4:
                robotcontrol.turn(self.robotturn_clockwise, self.robotturn_speed, self.robotturn_time)
                self.laser1 = robotcontrol.get_laser_all(-5,5)
                print("right: distance: ", self.laser1)
            self.check_crash()
        else:
            while self.laser1 < 0.4:
                robotcontrol.turn("aclockwise", self.robotturn_speed, self.robotturn_time)
                self.laser1 = robotcontrol.get_laser_all(-5,5)
                print("left: dicstance: ", self.laser1)
            self.check_crash()
        robotcontrol.stop_robot()
        print("양쪽 공간이 충분합니다.")

if __name__ == '__main__':

    robo = Robo()

    while not robotcontrol.ctrl_c:

        robo.robotmove()
        print("now turn")
        robo.robotturn()