#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import os
import datetime
import time
import pexpect
import getpass
import rospy
from necst.msg import Status_weather_msg
from necst.msg import Float64_list_msg
from numpy.random import randint, random
node_name = "weather_status"

class weather_controller(object):
    host = "amigos@200.91.8.66"
    dir = "/home/weather/WeatherMonitor/Weather_Data/"
    #dir = "/home/necst/ros/src/necst/scripts/device/"
    copy_dir = "/home/amigos/data/monitor/"
    data = [0]*20
    passwd = ""

    humi24 = [0]*24
    wind24 = [0]*24
    
    def __init__(self):
        self.pub_humi = rospy.Publisher("web_humi24", Float64_list_msg, queue_size=1)
        self.pub_wind = rospy.Publisher("web_wind24", Float64_list_msg, queue_size=1)
        pass

    def pub_func(self):
        pub = rospy.Publisher("status_weather", Status_weather_msg, queue_size = 10, latch = True)
        msg = Status_weather_msg()
        wmsg = Float64_list_msg()
        while not rospy.is_shutdown():
            ret = self.get_weather()
            msg.in_temp = ret[6]+random()
            msg.out_temp = ret[7]+random()
            msg.in_humi = ret[8]+random()
            msg.out_humi = ret[9]+random()
            msg.wind_sp = ret[11]+random()
            msg.wind_dir = ret[10]+random()
            msg.press = ret[12]+random()
            msg.rain = ret[13]+random()
            msg.cabin_temp1 = ret[14]+random()
            msg.cabin_temp2 = ret[15]+random()
            msg.dome_temp1 = ret[16]+random()
            msg.dome_temp2 = ret[17]+random()
            msg.gen_temp1 = ret[18]+random()
            msg.gen_temp2 = ret[19]+random()
            msg.from_node = node_name
            msg.timestamp = time.time()
            pub.publish(msg)
            print(msg)

            now = time.gmtime()
            hour = now.tm_hour
            self.humi24[hour] = msg.out_humi
            self.wind24[hour] = msg.wind_sp
            wmsg.data = self.humi24
            self.pub_humi.publish(wmsg)
            wmsg.data = self.wind24
            self.pub_wind.publish(wmsg)
            time.sleep(1)
        return


    def copy_file(self, _dir, data):
        path = self.copy_dir + _dir
        if not os.path.exists(path):
            os.makedirs(path)
            print("make_dir")
        scp = pexpect.spawn('scp %s:%s %s' % (self.host, self.dir+data, self.copy_dir+data))
        scp.expect('.*ssword:')
        scp.sendline(self.passwd)
        scp.interact()
        time.sleep(0.1)
        return

    def get_weather(self):
        now = time.time()-17*3600.#-17h(nanmeteo time)
        d = datetime.datetime.utcfromtimestamp(now)
        
        if d.month < 10:
            month = '0'+str(d.month)
        else:
            month = str(d.month)
            
        if d.day < 10:
            day = '0'+str(d.day)
        else:
            day = str(d.day)

        data = str(d.year)+month+"/"+str(d.year)+month+day+".nwd"
        """
        self.copy_file(str(d.year)+month + "/", data)

        with open(self.copy_dir+data, "r") as f:
            last_data = f.readlines()[-1]
        """
        with open("/home/amigos/ros/src/necst/simulator/device/" + "20161219.nwd", "r") as f:
            last_data = f.readlines()[-1]

        data_list = last_data.strip()
        data_list = data_list.split(",")
        for i in range(len(data_list)):
            self.data[i] = float(data_list[i].strip())

        return self.data

if __name__ == "__main__":
    rospy.init_node(node_name)
    wc = weather_controller()
    wc.pub_func()
