import rospy
import math
import serial
import time
import datetime
from std_msgs.msg import Float64
from imu_driver.msg import imu_msg
from sensor_msgs.msg import Imu as imu
from sensor_msgs.msg import MagneticField as mag



def EularToQuaternion(yaw, pitch, roll):

	cy = math.cos(0.5 * yaw)
	sy = math.sin(0.5 * yaw)
	cp = math.cos(0.5 * pitch)
	sp = math.sin(0.5 * pitch)
	cr = math.cos(0.5 * roll)
	sr = math.cos(0.5 * roll)
	
	quaternion_w = cr * cp * cy + sr * sp * sy
	quaternion_x = sr * cp * cy - cr * sp * sy
	quaternion_y = cr * sp * cy + sr * cp * sy
	quaternion_z = cr * cp * sy - sr * sp * cy
	
	quaternion = [quaternion_w, quaternion_x, quaternion_y, quaternion_z]
	
	return quaternion

def main():

	SENSOR_NAME = "VN-100"
	rospy.init_node("imu_driver")
	port_name = rospy.get_param("/imu_driver/port_name")
	baud_rate = 115200
	
	port = serial.Serial(port_name, baud_rate, timeout=3.)
	port.write(b'$VNWRG,07,40*XX\r\n')
	
	imu_publish = rospy.Publisher("imu", imu_msg, queue_size=5)
	#mag_publish = rospy.Publisher("magneticfield", mag_msg, queue_size=5)
	
	rospy.sleep(0.2)
	line = port.readline()
	
	final_message = imu_msg()
	final_message.Header.frame_id = "IMU1_Frame"
	final_message.Header.seq = 0
	final_message.Header.stamp = rospy.Time.now()
	
	imu_message = imu()
	imu_message.header.frame_id = "IMU"
	imu_message.header.seq = 0
	imu_message.header.stamp = rospy.Time.now()
	
	mag_message = mag()
	mag_message.header.frame_id = "MagneticField"
	mag_message.header.seq = 0
	mag_message.header.stamp = rospy.Time.now()
	
	final_message.IMU = imu_message
	final_message.MagField = mag_message
	
	#print()
	#print("Node Inilization Complete")
	#print("Node Starting Now...")
	#print()
	count = 0
	print(datetime.datetime.now())
	
	try:
		while not rospy.is_shutdown():
			line = port.readline()
			line = line.decode("utf-8")
			
			if "$VNWRG" in line:
				print(line)
				line = line.split(',')
				print(line)
				if len(line) != 13:
					print("Not able to parse data! Check your port and try again.")
				else:
					yaw = float(line[1])
					pitch = float(line[2])
					roll = float(line[3])
					
					quaternion = EularToQuaternion(yaw, pitch, roll)
					
					imu_message.orientation.w = quaternion[0]
					print("Quaternion orientation w:", quaternion[0])
					imu_message.orientation.x = quaternion[1]
					print("Quaternion orientation x:", quaternion[1])
					imu_message.orientation.y = quaternion[2]
					print("Quaternion orientation y:", quaternion[2])
					imu_message.orientation.z = quaternion[3]
					print("Quaternion orientation z:", quaternion[3])
					
					imu_message.angular_velocity.x = float(line[7])
					print("Angular velocity x:", float(line[7]))
					imu_message.angular_velocity.y = float(line[8])
					print("Angular velocity y:", float(line[8]))
					imu_message.angular_velocity.z = float(line[9])
					print("Angular velocity z:", float(line[9]))
					
					imu_message.linear_acceleration.x = float(line[10])
					print("Linear acceleration x:", float(line[10]))
					imu_message.linear_acceleration.y = float(line[11])
					print("Linear acceleration y:", float(line[11]))
					
					hex_list = line[12].split('*')
					left = float(hex_list[0])
					right = int(hex_list[-1], 16)
					right = str(right)
					right = float(right)
					result = left * right
					
					imu_message.linear_acceleration.z = result
					print("Linear acceleration z:", result)
					
					mag_message.magnetic_field.x = float(line[4])
					print("Magnetic field x:", float(line[4]))
					mag_message.magnetic_field.y = float(line[5])
					print("Magnetic field y:", float(line[5]))
					mag_message.magnetic_field.z = float(line[6])
					print("Magnetic field z:", float(line[6]))
			
			imu_publish.publish(final_message)
			#mag_publish.publish(mag_message)
	except rospy.ROSInterruptException:
		print("Abort")
		#print(datetime.datetime.now())
		print(count)
		port.close()
	


if __name__ == "__main__":
	main()
