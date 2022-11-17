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
import tf.transformations
from tf2_geometry_msgs import PoseStamped


class RfidReader():
    
    def __init__(self):        

        # initialise storage space for 
        self.current_pos = NavSatFix()

        # hook the first subscriber to our rfid-callback
        rospy.Subscriber("/rfid_detections", RelativeHumidity, self.rfid_callback)
        
        # hook the first subscriber to the fix-callback
        rospy.Subscriber("/uav1/fix", NavSatFix, self.gps_callback)
               
        # set up a tf2 Buffer this stores the incoming tf-messages       
        self.tfBuffer = tf2_ros.Buffer()
        # set up our TransformListener, this gives us access to transformations (even past ones through the buffer)
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
                
        # bool to avoid old latched message
        self.init = True
    
    def send_current_position_as_goal(self):
        
        # get the transform from map to the uav base_link
        transform = self.tfBuffer.lookup_transform("map", "uav/base_link", rospy.Time.now(), rospy.Duration(0.1))       
        
        # get data fields of tranformation
        q = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
        
        # get the inverse of the quaternion
        q_inv = tf.transformations.quaternion_inverse([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
        
        # define the point to be transformed (0, 0, 0) ( position of the uav in its own coordinate system)
        # we have to add a zero at the end in order to be able to multiply it with the quaternion (we can do that since it has no effect on the results)
        current_position_local = [0, 0, 0, 0]
        
        # compute the rotated position p_new = q*p*q_inv. Get the first 3 entries (x, y, z)
        current_position_rotated = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q, current_position_local), q_inv) [0:3]

        # compute the new rotation: p_new = q*p*q_inv
        # initialise as unit quaternion (no rotation in own frame)
        current_rotation_local = [0, 0, 0, 1]
        
        # rotate by transformation result
        current_rotation_map = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q, current_rotation_local), q_inv)
        
        # translate the rotated position
        current_position_map = [current_position_rotated[0] + transform.transform.translation.x, current_position_rotated[1] + transform.transform.translation.y, current_position_rotated[2] + transform.transform.translation.z] 
        
        # printout the results
        rospy.loginfo("transformed Position: (%f, %f, %f),  \n", current_position_map[0], current_position_map[1], current_position_map[2])
        rospy.loginfo("transformed Rotation: (%f, %f, %f, %f),  \n", current_rotation_map[0], current_rotation_map[1], current_rotation_map[2], current_rotation_map[3])
        
        
    # RFID detection callback 
    def rfid_callback(self, message : RelativeHumidity):
                
        # skip first message (old latched)
        if self.init:
            return
        
        # check if the humidity we read out is below threshold
        if message.relative_humidity < 0.5:
            rospy.loginfo(" Humidity too low - Send goal to UGV")
            # if it is below a certain threshold send the current UAV position as goal-point to the UGV
            self.send_current_position_as_goal()      
    
    # GPS-position (fix) message callback 
    def gps_callback(self, message : NavSatFix):
        
        # let other callbacks know that gps is available
        if self.init:
            self.init = False
        
        # print the current position every two seconds (not for every message)
        rospy.loginfo_throttle(2.0, "Read GPS Position. Lat: %f Long: %f \n", message.latitude, message.longitude)
        
        # store the position in a object attribute
        self.current_pos = message

    def run(self):

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        

if __name__ == '__main__':
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    RFID_reader = RfidReader()
    RFID_reader.run()