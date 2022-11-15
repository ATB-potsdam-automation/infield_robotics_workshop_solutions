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
import csv
import rospkg




class RfidReader():
    
    def __init__(self):        
        # get path of ros_pkg
        rospack = rospkg.RosPack()
        pkg_folder = rospack.get_path('infield_robotics_ws_solutions')
        
        # open a csv file in the results folder of the package for data saving
        self.fh = open(pkg_folder + "/results/humidity_sensors.csv", "w")
        
        # create a csv_writer object to write data the Dict-Writer uses a Dictionary Structure
        self.csv_writer = csv.DictWriter(self.fh, fieldnames=['Latitude', 'Longitude', 'Humidity'])

        self.csv_writer.writeheader()
        
        # initialise storage space for 
        self.current_pos = NavSatFix()

        # hook the first subscriber to our rfid-callback
        rospy.Subscriber("/rfid_detections", RelativeHumidity, self.rfid_callback)
        
        # hook the first subscriber to the fix-callback
        rospy.Subscriber("/uav1/fix", NavSatFix, self.gps_callback)
        

    # RFID detection callback
    def rfid_callback(self, message : RelativeHumidity):
        
        
        # we need to make sure the writer has been initialised:
        if self.csv_writer is not None:
                        
            """
            YOUR CODE GOES HERE:
            
            write the data to the csv file using the csv-writer
            
            the field names are: "Latitude", "Longitude", "Humidity"
            
            the writer expects an argument of dictionary type: {"field1" : value1, "field2" : value2}
                    
            documentation of the csv-DictWriter can be found here: https://docs.python.org/3/library/csv.html#csv.DictWriter 
                
            """
            rospy.loginfo("\n\n Read RFID-Sensor! Sensor: %s Humidity: %f \n", message.header.frame_id, message.relative_humidity)
            self.csv_writer.writerow({'Latitude': self.current_pos.latitude, 'Longitude': self.current_pos.longitude, 'Humidity': message.relative_humidity})

    # GPS-position (fix) message callback 
    def gps_callback(self, message : NavSatFix):
        # print the current position every two seconds (not for every message)
        rospy.loginfo_throttle(2.0, "Read GPS Position. Lat: %f Long: %f \n", message.latitude, message.longitude)
        
        # store the position in a object attribute
        self.current_pos = message

    def run(self):

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
        # close the file
        
        self.csv_writer = None
        
        self.fh.close()
        

if __name__ == '__main__':
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    RFID_reader = RfidReader()
    RFID_reader.run()