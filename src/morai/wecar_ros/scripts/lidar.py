#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan,PointCloud
from math import cos,sin,pi,sqrt
from geometry_msgs.msg import Point32
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math
from time import sleep
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32, Float32


class lidarParser :

    def __init__(self):
        rospy.init_node('lidar_parser', anonymous=True)
        rospy.Subscriber("/lidar2D", LaserScan, self.laser_callback)
        
        self.pcd_pub = rospy.Publisher('/laser2pcd',PointCloud, queue_size=1)

        rospy.spin()

    # angle - 3 : 앞     1: 오른쪽      4: 왼쪽
    # Rviz - 한칸이 1
    # 0.15?같은 낮은 수가 뜨는 건, 센서의 위치 때문처럼 보여서 그걸 무시하도록 코드를 짜는게 관건일듯!
    def laser_callback(self,msg):
        pcd=PointCloud()
        motor_msg=Float64()
        pcd.header.frame_id=msg.header.frame_id
        angle=0
        angle_point=[]
        for r in msg.ranges :

            tmp_point=Point32()
            tmp_point.x=r*cos(angle)
            tmp_point.y=r*sin(angle)
            #print(angle,tmp_point.x,tmp_point.y)
            angle=angle+(1.0/180*pi)
            if r<2.5:
                tmp_point.z=angle
                pcd.points.append(tmp_point)
                angle_point.append(angle)
            
            
        self.pcd_pub.publish(pcd)
        for i in range (0,len(pcd.points)):
            distance = sqrt((pcd.points[i].x)**2+(pcd.points[i].y)**2)
            # if pcd.points[i].z>=2.0 and pcd.points[i].z<=4.0 and distance<1.8:
            #     print(distance, pcd.points[i].z)
               

if __name__ == '__main__':
    try:
        test=lidarParser()
    except rospy.ROSInterruptException:
        pass
