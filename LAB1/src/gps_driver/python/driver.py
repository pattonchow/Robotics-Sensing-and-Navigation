import time
import datetime as dt
import utm
import string
import rospy
import serial
from math import sin, pi
from gps_driver.msg import gps_msg


def trans2Degree(DDmmm):
    mmData=str(DDmmm).split('.')
    ddData=0
    i=len(mmData[0])

    for i in range(i,i-2,-1):
        j=int(-(i-len(mmData[0])))
        if j:ddData = ddData + float(mmData[0][int(i)-1])*10
        else : ddData = ddData + float(mmData[0][int(i)-1])

    ddData=ddData+float(mmData[1])/pow(10,len(mmData[1]))
    ddData = ddData/60

    if len(mmData[0]) == 5: ddData=ddData + float(str(DDmmm)[:3])
    elif len(mmData[0]) == 4: ddData=ddData + float(str(DDmmm)[:2])
    else: print("invalid value")

    return ddData




if __name__ == '__main__':
    SENSOR_NAME = "GPS_1"
    rospy.init_node('gnss_reader')
    serial_port = rospy.get_param('~port','/dev/pts/2')
    serial_baud = rospy.get_param('~baudrate',4800) 
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    print("Using GPS sensor on port "+serial_port+" at "+str(serial_baud))
    gps_pub = rospy.Publisher('/gps', gps_msg, queue_size=5)
    
    rospy.sleep(0.2)        
    line = port.readline()
    
    GPS_msg = gps_msg()
    GPS_msg.Header.frame_id = "GPS1_Frame"
    GPS_msg.Header.seq=0
    
    
    
    #rospy.logdebug("Initialization complete!")
    print("Initialization complete!")
    try:
        #print("*********")
        while not rospy.is_shutdown():
            line = port.readline()
            line=line.decode("utf-8")
            
            #if line.startswith('$GPGGA'):
            if "$GPGGA" in line:
                print("Line: ")
                print(line)
                GPSdata=line.split(',')
                print(GPSdata) 
                if GPSdata[2] == '':
                    print("No Data")
                else:
                    utc_=float(GPSdata[1])
                    latitude_= trans2Degree(float(GPSdata[2]))
                    latitude_dir_= GPSdata[3]
                    print("latitude_dir:",latitude_dir_)
                    longitude_ = trans2Degree(float(GPSdata[4]))
                    longitude_dir_ = GPSdata[5]
                    print("longitude_dir:",longitude_dir_)
                    GPS_quality = GPSdata[6]
                    print("GPS_quality:",GPS_quality)
                    sats = GPSdata[7]
                    print("Number of satellites:",sats)
                    hdop = GPSdata[8]
                    print("Horizontal dilution of precision:",hdop)
                    altitude_=float(GPSdata[9])
                    print("Altitude:",altitude_)
                    if latitude_dir_ =="S": latitude_=-latitude_
                    if longitude_dir_ == "W": longitude_=-longitude_
                    

                    utm_=utm.from_latlon(latitude_, longitude_)
                    
                    utm_easting_=float(utm_[0])
                    utm_northing_=float(utm_[1])
                    zone_=float(utm_[2])
                    letter_=utm_[3]
                    
                    gps_time = dt.datetime.combine(
                        dt.datetime.utcnow().date(), 
                        dt.datetime.strptime(GPSdata[1], "%H%M%S.%f").time())
                    # gps_time = dt.datetime.strptime(GPSdata[1], "%H%M%S.%f")
   
                    GPS_msg.Header.stamp.secs = int(time.mktime(gps_time.timetuple()))
                    GPS_msg.Header.stamp.nsecs = gps_time.microsecond
    
		
			
		

                    GPS_msg.Latitude=latitude_
                    GPS_msg.Longitude=longitude_
                    
                    GPS_msg.Altitude=altitude_
                    GPS_msg.UTM_easting=utm_easting_
                    GPS_msg.UTM_northing=utm_northing_
                    GPS_msg.Zone=zone_
                    GPS_msg.Letter=letter_
                    
                    
                    gps_pub.publish(GPS_msg)
                    GPS_msg.Header.seq += 1
                
            
    except rospy.ROSInterruptException:
        port.close()
