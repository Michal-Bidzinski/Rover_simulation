import re
import rospy

# decoding gps messages
def gps_read_coordinates(new_data):
    received = False
    lat = 0.0
    lng = 0.0
    time = 0.0
    new_data_2 = str(new_data)

    if new_data_2[2:8] == "$GPGGA" or new_data_2[2:8] == "$GPRMC":
        array = re.findall('\d+.\d+', new_data_2)
        if len(array) > 3:
            time_b = float(array[0])
            lat_b = float(array[1])
            lng_b = float(array[2])
            received = True

    if new_data_2[2:8] == "$GPGLL":
        array = re.findall('\d+.\d+', new_data_2)
        if len(array) > 3:
            lat_b = float(array[0])
            lng_b = float(array[1])
            time_b = float(array[2])
            received = True


    if received:
        t_ros = rospy.Time.now().secs

        years = int(t_ros/3600/24/365)
        days = int((t_ros - years*365*24*3600)/3600/24)

        lat = float(int(lat_b/100) + (lat_b - int(lat_b/100)*100)/60)
        lng = float(int(lng_b/100) + (lng_b - int(lng_b/100)*100)/60)
         
        hour = int(time_b/10000)
        minute = int((time_b - hour*10000)/100)     
        second = int(time_b - hour*10000 - minute*100)

        print("date: ", hour, ":", minute, ":", second)
        print("ros time: ", rospy.Time.now().secs, " ", rospy.Time.now().nsecs)

        t_second = years*365*24*3600 + days*24*3600 +  hour * 3600 + minute * 60 + second

        if t_second > t_ros:
            t_second -= t_second - t_ros

        print("t_gps   : ", t_second)
        time = rospy.Time(secs=t_second, nsecs = 0)


    return received, lat, lng, time


