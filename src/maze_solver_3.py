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
        self.rate = rospy.Rate(1)
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

    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_laser(self, pos):
        time.sleep(1)
        return self.laser_msg.ranges[pos]

    # def get_laser_all(self, pos1, pos2):
    #     time.sleep(1)
    #     if pos1 < 0:
    #         laser_all_1 = self.laser_msg.ranges[360+pos1:360]
    #         laser_all_2 = self.laser_msg.ranges[0:pos2]
    #         return laser_all_1 + laser_all_2
    #     else:
    #         return self.laser_msg.ranges[pos1:pos2]

    def get_front_laser(self):
        time.sleep(1)
        return self.laser_msg.ranges[0]

    def get_laser_full(self):
        time.sleep(1)
        return self.laser_msg.ranges

    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.2
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

    def __init__(self):
        print("inicializando...")
        self.laser1 = robotcontrol.get_laser(0)
        self.robotmove_speed = 0.5
        self.robotmove_time = 1
        self.robotturn_clockwise = "clockwise"
        self.robotturn_speed = 0.5
        self.robotturn_time = 1
        # self.laser2 = robotcontrol.get_laser_all(-30, 30)

    def robotmove(self):
        # while self.laser1 > 0.7 and all(e >= 0.25 for e in self.laser2):
        while self.laser1 > 0.5:
            robotcontrol.move_straight()
            self.laser1 = robotcontrol.get_laser(0)
            # self.laser2 = robotcontrol.get_laser_all(-30, 30)
            print("distancia: ", self.laser1)

        robotcontrol.stop_robot()
        print("estou perto demais da parede... ")

    def robotturn(self):
        distancia_direita = robotcontrol.get_laser(270)
        distancia_esquerda = robotcontrol.get_laser(90)
        print("direita",distancia_direita, "esquerda", distancia_esquerda)
        if distancia_direita > distancia_esquerda:
            while self.laser1 < 0.5:
                robotcontrol.turn(self.robotturn_clockwise, self.robotturn_speed, self.robotturn_time)
                self.laser1 = robotcontrol.get_laser(0)
                # self.laser2 = robotcontrol.get_laser_all(-30, 30)
                print("direita: distancia: ", self.laser1)
                # print("distance_all : ", self.laser2)
        else:
            while self.laser1 < 0.5:
                robotcontrol.turn("aclockwise", self.robotturn_speed, self.robotturn_time)
                self.laser1 = robotcontrol.get_laser(0)
                # self.laser2 = robotcontrol.get_laser_all(-30, 30)
                print("esquerda: distancia: ", self.laser1)
                # print("distance_all : ", self.laser2)

        robotcontrol.stop_robot()
        print("ja virei, bora... ")

if __name__ == '__main__':

    robo = Robo()

    while not robotcontrol.ctrl_c:

        robo.robotmove()
        robo.robotturn()