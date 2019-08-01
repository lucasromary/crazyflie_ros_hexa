#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('pwm', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    vx = rospy.get_param("~vx")
    vy = rospy.get_param("~vy")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")
    yaw = rospy.get_param("~yaw")

    rate = rospy.Rate(10) # 10 hz
    name = "cmd_position"
    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.x = x
    msg.y = y
    msg.z = z
    msg.yaw = yaw
    msg.vx = vx
    msg.vy = vy

    pub = rospy.Publisher(name, Position, queue_size=1)

    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)


    # go to x: 0.2 y: 0.2
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.x = x
        msg.vx = vx
        msg.vy = vy
        msg.y = y
        msg.yaw = yaw
        msg.z = z
        now = rospy.get_time()
        if (now - start > 10.0):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo("sending...")
        rospy.loginfo(msg.x)
        rospy.loginfo(msg.y)
        rospy.loginfo(msg.z)
        rospy.loginfo(msg.yaw)
        rospy.loginfo(msg.vx)
        rospy.loginfo(msg.vy)

        pub.publish(msg)
        rate.sleep()


    stop_pub.publish(stop_msg)
