#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    pwm1 = rospy.get_param("~pwm1")
    pwm2 = rospy.get_param("~pwm2")
    pwm3 = rospy.get_param("~pwm3")
    pwm4 = rospy.get_param("~pwm4")


    rate = rospy.Rate(10) # 10 hz
    name = "cmd_position"
    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pwm1 = pwm1
    msg.pwm2 = pwm2
    msg.pwm3 = pwm3
    msg.pwm4 = pwm4

    pub = rospy.Publisher(name, Position, queue_size=1)

    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    # go to x: 0.2 y: 0.2
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.pwm1 = pwm1
        msg.pwm2 = pwm2
        msg.pwm3 = pwm3
        msg.pwm4 = pwm4
        now = rospy.get_time()
        if (now - start > 5.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo("sending...")
        rospy.loginfo(msg.pwm1)
        rospy.loginfo(msg.pwm2)
        rospy.loginfo(msg.pwm3)
        rospy.loginfo(msg.pwm4)
        pub.publish(msg)
        rate.sleep()

    stop_pub.publish(stop_msg)
