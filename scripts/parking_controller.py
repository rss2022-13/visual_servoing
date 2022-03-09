#!/usr/bin/env python

from dis import dis
import rospy
import numpy as np
import math

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation, self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error", ParkingError, queue_size=10)

        self.parking_distance = .5 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.speed = 0.5
        self.angle = 0
        self.dist_P = 1
        self.dist_D = 1.1
        self.ang_P = rospy.get_param("~ang_P")
        self.ang_D = rospy.get_param("~ang_D")
        self.prev_dist_err = 0
        self.prev_ang_err = 0

    def relative_cone_callback(self, msg):
        #relative x and y wrt frame of racecar
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        target_angle = math.atan2(self.relative_y, self.relative_x)
        current_distance = (self.relative_x**2 + self.relative_y**2)**(0.5) + .5

        self.dist_P = rospy.get_param("~dist_P")
        self.dist_D = rospy.get_param("~dist_D")
        self.ang_P = rospy.get_param("~ang_P")
        self.ang_D = rospy.get_param("~ang_D")

        dist_err = current_distance - self.parking_distance
        ang_err = target_angle
        self.speed = self.dist_P*dist_err + self.dist_D*(dist_err - self.prev_dist_err)
        if abs(self.speed) > 1:
            self.speed = self.speed/abs(self.speed)
        self.steer_angle = self.ang_P*ang_err + self.ang_D*(ang_err - self.prev_ang_err)

        #TODO: change so that if youre far and angle is large, just go forward
        #or increase the threshold for a large angle
        if abs(ang_err) >= math.pi/4: #prioritize fixing large angle error, back up and rotate
            self.speed = -0.5
            self.steer_angle = -abs(ang_err)/ang_err * abs(ang_err)

        '''
        #stop if close enough
        if abs(dist_err) < 0.05 and abs(ang_err) < 0.1:
            self.speed = 0
            self.steer_angle = 0
        '''

        #print('speed:', self.speed, 'steer:', self.steer_angle, 'ang err:', ang_err)
        self.prev_ang_err = ang_err
        self.prev_dist_err = dist_err
        '''
        # CASE FOR SIMULATOR ONLY: Cone behind us:
        if self.relative_x <= 0:
            if self.relative_y <= 0:
                self.angle = -math.pi/2
            else:
                self.angle = math.pi/2
        
        # CASE: Cone in Front
        else:
            self.angle = target_angle

        # CASE: Cone too close
        if current_distance < self.parking_distance:
            #self.angle * -1
            self.speed = -self.speed

        # CASE: Cone to the side
        else:
            if target_angle > math.pi/8:
                self.angle = math.pi/2
            elif target_angle < math.pi/8:
                self.angle = -math.pi/2
            else:
                # CASE: Angle is fine, but we need to get closer
                if current_distance > self.parking_distance + 1: # We set the correctness tolerance as +1 Meter
                    self.speed = 0.5
                else:
                    self.speed = 0
        '''
        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = self.steer_angle
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()      
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = (self.relative_x**2 + self.relative_y**2)**(0.5) - self.parking_distance
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
