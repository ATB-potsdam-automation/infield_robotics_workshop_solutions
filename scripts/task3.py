#!/usr/bin/env python3
# Copyright <2022> <Tjark Schuette (tschuette@atb-potsdam.de)>

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

#     Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#     Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#     Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from sensor_msgs.msg import RelativeHumidity, NavSatFix
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from nav_msgs.msg import Path

class RfidReader():
    
    def __init__(self):        

        # initialise storage space for 
        self.current_pos = NavSatFix()

        # hook the first subscriber to our rfid-callback
        rospy.Subscriber("/rfid_detections", RelativeHumidity, self.rfid_callback)
        
        # hook the first subscriber to the fix-callback
        rospy.Subscriber("/uav1/fix", NavSatFix, self.gps_callback)
        
        self.path_publisher = rospy.Publisher("/irus/path", Path, queue_size = 1)
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.path = Path()
        self.path.header.frame_id = "map"
        # TransformStamped.transform.translation
    
    def send_current_position_as_goal(self):
        transform = self.tfBuffer.lookup_transform('map', "uav/base_footprint", rospy.Time.now(), rospy.Duration(0.1))
        rospy.loginfo("\n received Rotation: (%f, %f, %f,%f),  \n", transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w,)
        rospy.loginfo("\n received Translation: (%f, %f, %f),  \n", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
        current_pose = PoseStamped()
        current_pose.header.frame_id = 'map'
        current_pose.header.stamp = rospy.Time.now()
        current_pose.pose.position = Point(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z )
        current_pose.pose.orientation = transform.transform.rotation
        self.path.poses.append(current_pose)
        self.path_publisher.publish(self.path)
        
        
        pass
   
    # RFID detection callback 
    def rfid_callback(self, message : RelativeHumidity):
        # check if the humidity we read out is below threshold
        if message.relative_humidity < 0.5:
            self.send_current_position_as_goal()      
    
    # GPS-position (fix) message callback 
    def gps_callback(self, message : NavSatFix):
        
        # print the current position every two seconds (not for every message)
        rospy.loginfo_throttle(2.0, "Read GPS Position. Lat: %f Long: %f \n", message.latitude, message.longitude)
        
        # store the position in a object attribute
        self.current_pos = message
        
        

    def run(self):

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
        print(self.path)

if __name__ == '__main__':
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    RFID_reader = RfidReader()
    RFID_reader.run()