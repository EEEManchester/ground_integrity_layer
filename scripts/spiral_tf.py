#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

class TF2PublisherSpiral(object):
    def __init__(self):
        rospy.init_node("TFSpiraliser")
                
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "base_link")
        self.speed = rospy.get_param("~speed", 1.0) # m/s

        rospy.loginfo("Starting TF broadcaster between %s -> %s", self.global_frame, self.child_frame)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.start_time = rospy.Time.now()

        
        self.distance_lim = 5.0 # m
        self.y_divisions = 11
        self.stepover = (2*self.distance_lim) / self.y_divisions

        self.x = self.y = -self.distance_lim
        self.z = 0.2 # m
        
        self.rate = rospy.Rate(10) # Hz

        while not rospy.is_shutdown():
            self.publishTF()
            self.rate.sleep()

    def publishTF(self):
        tfmsg = geometry_msgs.msg.TransformStamped()

        tfmsg.header.stamp = rospy.Time.now()
        tfmsg.header.frame_id = self.global_frame
        tfmsg.child_frame_id = self.child_frame

        self.calculateXY()
        tfmsg.transform.translation.x = self.x
        tfmsg.transform.translation.y = self.y
        tfmsg.transform.translation.z = self.z

        # Assume no rotation
        tfmsg.transform.rotation.x = 0.0
        tfmsg.transform.rotation.y = 0.0
        tfmsg.transform.rotation.z = 0.0
        tfmsg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tfmsg)


    def calculateXY(self):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.start_time).to_sec()

        distance = delta_time * self.speed

        delta_y, delta_x = divmod(distance, 2*self.distance_lim)
        delta_y = delta_y % self.y_divisions # makes the y steps repeat

        self.x = -self.distance_lim + delta_x
        self.y = -self.distance_lim + (delta_y * self.stepover)

if __name__ == "__main__":
    try:
        tfit = TF2PublisherSpiral()
        rospy.spin()
    except rospy.ROSInterruptException: pass
