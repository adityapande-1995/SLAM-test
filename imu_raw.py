from __future__ import print_function
import FaBo9Axis_MPU9250, rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu , MagneticField
import numpy as np

mpu9250 = FaBo9Axis_MPU9250.MPU9250()

class IMU:
        def __init__(self):
                self.agm = rospy.Publisher('agm', String, queue_size=10)
                self.imu_raw = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
                self.maginfo = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
                
                rospy.init_node("imu_node")
                self.rate = rospy.Rate(1000)
                
                while not rospy.is_shutdown():
                        accel = mpu9250.readAccel() ; gyro = mpu9250.readGyro() ; mag = mpu9250.readMagnet()
                        self.publish_string(accel, gyro, mag)
                        self.publish_raw(accel, gyro, mag)
                        self.rate.sleep()

        def publish_string(self,accel, gyro, mag):
                agm_msg = str(accel["x"])+"|"+str(accel["y"])+"|"+str(accel["z"]) + "|"+str(gyro["x"])+"|"+str(gyro["y"])+"|"+str(gyro["z"]) + "|"+str(mag["x"])+"|"+str(mag["y"])+"|"+str(mag["z"])
                self.agm.publish(agm_msg)

        def publish_raw(self,accel, gyro, mag):
                temp = Imu()
                af = 10 ; gf = np.pi/180 ; mf = 1
                temp.angular_velocity.x = gyro["x"]*gf ; temp.angular_velocity.y = gyro["y"]*gf ; temp.angular_velocity.z = gyro["z"]*gf ;
                temp.linear_acceleration.x = accel["x"]*af ; temp.linear_acceleration.y = accel["y"]*af ; temp.linear_acceleration.z = accel["z"]*af ;
                temp.header.stamp = rospy.Time.now()
                temp.header.frame_id = "imu"
                self.imu_raw.publish(temp)

                temp2 = MagneticField()
                temp2.magnetic_field.x = mag["x"]*mf ; temp2.magnetic_field.y = mag["y"]*mf ; temp2.magnetic_field.z = mag["z"]*mf ;
                temp2.header.stamp = rospy.Time.now()
                temp2.header.frame_id = "imu"
                self.maginfo.publish(temp2)
                
        def euler(self,q):
                a1 = np.arctan2( 2*(q[0]*q[1] + q[2]*q[3]) , 1 - 2*(q[1]**2 + q[2]**2) )*180/np.pi
                a2 = np.arcsin( 2*(q[0]*q[2] - q[3]*q[1])  )*180/np.pi
                a3 = np.arctan2( 2*(q[0]*q[3] + q[1]*q[2]) , 1 - 2*(q[2]**2 + q[3]**2)    )*180/np.pi
                return [a1,a2,a3]
                
if __name__ == '__main__':
        print("Starting IMU node...")
        a = IMU()
        
