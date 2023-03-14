#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import math
import time
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight
from lib.utils import pathReader, findLocalPath,purePursuit,cruiseControl,vaildObject,pidController,velocityPlanning,latticePlanner
import tf
from sensor_msgs.msg import PointCloud
from math import cos,sin,sqrt,pow,atan2,pi

import rospy
import math
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from race.msg import drive_values

class ClusterLidar :

    def __init__(self) :
        self.angle = 0 
        self.is_rabacon_mission = False

    # select close rabacon
    def rabacon(self, seg)  :
        left_rabacons, right_rabacons = self.sel_close_rabacon(seg.circles)
        self.drive_angle = self.cal_angle(left_rabacons, right_rabacons)
        if seg.circles >= 6 :
            self.is_rabacon_mission = True
        else :
            self.is_rabacon_mission = False
        return self.is_rabacon_mission, self.drive_angle * 0.017

    def cal_angle(self, left_rabacons, right_rabacons) :
        left_angle = math.atan(left_rabacons[0].y/left_rabacons[0].x)
        right_angle =  math.atan(right_rabacons[0].y/right_rabacons[0].x)


        return left_angle - right_angle
        # left_angle = math.atan(left_angle[0].x/)
        

    def sel_close_rabacon(self, rabacons) :
        left_rabacon = []
        right_rabacon = []
        for circle in rabacons :
            if circle.x > 0 and circle.y > 0 :
                left_rabacon.append(circle)
            elif circle.x > 0 and circle.y < 0 :
                right_rabacon.append(circle)
        left_rabacon = sorted(left_rabacon, key = lambda x : x.x)[:2]
        right_rabacon = sorted(right_rabacon, key = lambda x : x.x)[:2]

        return left_rabacon, right_rabacon


class wecar_planner():
    def __init__(self):
        rospy.init_node('wecar_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]

        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        
        ########################  lattice  ########################
        for i in range(1,8):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  
        ########################  lattice  ########################
        
        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber 
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.objectInfoCB) ## Object information Subscriber
        rospy.Subscriber("/GetTrafficLightStatus",GetTrafficLightStatus,self.trafficInfoCB)
        rospy.Subscriber("/laser2pcd",PointCloud,self.lidarInfo) 
        rospy.Subscriber("/commands/motor/speed",Float64,self.speedInfo)
        
        
        #def
        self.is_status=False ## 차량 상태 점검
        self.is_obj=False ## 장애물 상태 점검
        self.steering_angle_to_servo_offset=0.5 ## servo moter offset
        self.rpm_gain = 4616
        self.motor_msg=Float64()
        self.servo_msg=Float64()
        self.status_traffic=False
        self.distance = 0
        self.angle = 0
        self.lotery_stop=False
        self.lotery_in = False
        self.stop=0
        self.now_speed = 0.0
        self.missioning = False
        self.missioning2 = False
        self.judgement = 0
        

        #class
        path_reader=pathReader('wecar_ros') ## 경로 파일의 위치
        pure_pursuit=purePursuit() ## purePursuit import
        self.cc=cruiseControl(0.5,1) ## cruiseControl import (object_vel_gain, object_dis_gain)
        self.vo=vaildObject() ## 장애물 유무 확인 ()
        pid=pidController() ## pidController import
        

        #read path
        self.global_path=path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름
        
        vel_planner=velocityPlanning(10,0.15) ## 속도 계획
        vel_profile=vel_planner.curveBasedVelocity(self.global_path,30)
        

        
        #time var
        count=0
        rate = rospy.Rate(30) # 30hz
        speedup=5 # lotery control speed - up
        speeddown=10 # lotery control speed - down
        lattice_current_lane=3

        while not rospy.is_shutdown():
            #print(self.is_status , self.is_obj)
                        
            if self.is_status==True:# and self.is_obj==True: ## 차량의 상태, 장애물 상태 점검
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg) 
                
                ## 장애물의 숫자와 Type 위치 속도 (object_num, object type, object pose_x, object pose_y, object velocity)
                # self.vo.get_object(self.object_num,self.object_info[0],self.object_info[1],self.object_info[2],self.object_info[3])
                # global_obj,local_obj=self.vo.calc_vaild_obj([self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi])
                #직접 코드 짜야 됨

                ########################  lattice  ########################
                vehicle_status=[self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi,self.status_msg.velocity.x/3.6]
                lattice_path,selected_lane=latticePlanner(local_path,vehicle_status,lattice_current_lane)
                lattice_current_lane=selected_lane
                                
                if selected_lane != -1: 
                    local_path=lattice_path[selected_lane]                
                
                if len(lattice_path)==7:                    
                    for i in range(1,8):
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ########################  lattice  ########################
            
                # self.cc.checkObject(local_path,global_obj,local_obj)

                
                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용

                self.steering=pure_pursuit.steering_angle()
                
                # self.cc_vel = self.cc.acc(local_obj,self.status_msg.velocity.x,vel_profile[self.current_waypoint],self.status_msg) ## advanced cruise control 적용한 속도 계획
                self.cc_vel = 10

                self.servo_msg = self.steering*0.021 + self.steering_angle_to_servo_offset
                self.motor_msg = self.speed_control() #self.cc_vel *self.rpm_gain /3.6 # per by steering
                #-----------mission 진행 판단----------------
                self.missioning = False
                self.stop=0
                #-----traffic detection---------------
                if self.status_msg.position.x >= 7.10 and self.status_msg.position.x <= 7.50 and self.status_msg.position.y >= -2.5 and self.status_msg.position.y <= -2.00:
                    self.missioning=True
                    if self.status_traffic == True:
                        self.motor_msg=0
                    else:
                        self.motor_msg=2500
                
                #----------lotery--------------------------
                if self.status_msg.position.x >= 12.0 and self.status_msg.position.x <= 12.5 and self.status_msg.position.y > 1.97 and self.status_msg.position.y <= 2.5:
                    self.missioning=True
                    print('lotery out--------------------------------------------------')
                    print(self.angle, self.lotery_stop, self.distance,self.now_speed)
                    if self.angle>3 and self.lotery_stop == False:
                        if self.distance<=1.6:
                            self.motor_msg=0
                            self.lotery_stop=True

                    if self.lotery_stop == True:
                        
                        if self.lotery_in == True:
                            self.motor_msg = 500
                        
                        elif self.lotery_in == False:
                            self.motor_msg = 0
                            if self.angle < 3.0 and self.distance>0.3:
                                self.lotery_in=True
                                self.motor_msg = 500
                
                #-------in lotery----------------------------
                if self.status_msg.position.x >= 11.2 and self.status_msg.position.x <= 14.0 and self.status_msg.position.y >= -1.45 and self.status_msg.position.y <= 1.97:
                    self.missioning=True
                    print('lotery in----------------------------------------------')
                    self.motor_msg=500
                    print(self.angle, self.lotery_stop, self.distance,self.now_speed)
                    #if self.angle<=4.5 and self.angle>=3.0:
                    if self.distance > 0.6 and self.motor_msg<=800: 
                        speeddown=100
                        self.motor_msg +=speedup
                        speedup+=5
                        print('------속도증가-------')
                        print(self.distance, self.angle, self.motor_msg)
                    else :
                        print('------속도감소-------')
                        print(self.distance, self.angle, self.motor_msg)
                        speedup=5
                        self.motor_msg -=speeddown
                        if self.motor_msg<0:
                            self.motor_msg=0
                            speeddown=0
                        speeddown+=100
                    #if self. distance < 1 :
                    #print(self.distance, self.angle, self.motor_msg)
                
                #----------------static object case-----------------------------
                # if self.missioning==False:
                #     print('----judgement----------',self.judgement)
                #     if self.judgement==0 or self.judgement==1:
                #         if self.distance <= 2.0 and self.distance>=0.7 and self.angle >=3.0 and self.angle<=3.3:
                #             self.judgement=1
                #             self.motor_msg=500
                #             self.missioning2 = True
                #         elif self.distance<=0.8 and self.missioning2==True:
                #             self.motor_msg=0
                #             start = time.time()
                #             if self.angle<3.6:
                #                 while(1):
                #                     end = time.time()
                #                     self.motor_msg=500
                #                     self.servo_msg=-19.5
                #                     self.servo_pub.publish(self.servo_msg)
                #                     self.motor_pub.publish(self.motor_msg)
                #                     if end-start>1.25:
                #                         self.missioning2 = False
                #                         self.judgement=0
                #                         break
                #             else:
                #                 self.judgement=0
                #                 self.missioning2 = False

                #--------------------------------------------------------------------------------------
                def __init__(self) :
                    self.angle = 0 
                    self.is_rabacon_mission = False

                # select close rabacon
                def rabacon(self, seg)  :
                    left_rabacons, right_rabacons = self.sel_close_rabacon(seg.circles)
                    self.drive_angle = self.cal_angle(left_rabacons, right_rabacons)
                    if seg.circles >= 6 :
                        self.is_rabacon_mission = True
                    else :
                        self.is_rabacon_mission = False
                    return self.is_rabacon_mission, self.drive_angle * 0.017

                def cal_angle(self, left_rabacons, right_rabacons) :
                    left_angle = math.atan(left_rabacons[0].y/left_rabacons[0].x)
                    right_angle =  math.atan(right_rabacons[0].y/right_rabacons[0].x)


                    return left_angle - right_angle
                    # left_angle = math.atan(left_angle[0].x/)
                    

                def sel_close_rabacon(self, rabacons) :
                    left_rabacon = []
                    right_rabacon = []
                    for circle in rabacons :
                        if circle.x > 0 and circle.y > 0 :
                            left_rabacon.append(circle)
                        elif circle.x > 0 and circle.y < 0 :
                            right_rabacon.append(circle)
                    left_rabacon = sorted(left_rabacon, key = lambda x : x.x)[:2]
                    right_rabacon = sorted(right_rabacon, key = lambda x : x.x)[:2]

                    return left_rabacon, right_rabacon


                if self.missioning==False:
                    
                    if (self.distance <= 1.8 and self.distance>=0.7) and (self.angle>=2.0 and self.angle<=4.0):
                        self.motor_msg=500
                        self.missioning2 = True
                    elif self.distance<=0.8 and self.missioning2==True:
                        self.first = self.angle
                        self.motor_msg=0
                        start_judge=time.time()
                        self.stop+=1
                        while(1):
                            self.servo_pub.publish(self.servo_msg)
                            self.motor_pub.publish(self.motor_msg)
                            end_judge=time.time()
                            if(end_judge-start_judge>1.0):
                                self.second = self.angle
                                if (self.first>3.0 and self.second<3.0) or (self.first<3.0 and self.second>3.0):
                                    if self.distance>0.7:
                                        self.stop=0
                                        break
                                elif (self.first>=3.0 and self.second>=3.0):
                                    self.stop=1
                                    break
                                
                        if self.stop!=0:
                            start = time.time()
                            if self.angle<3.6:
                                while(1):
                                    end = time.time()
                                    self.motor_msg=500
                                    self.servo_msg=-19.5
                                    self.servo_pub.publish(self.servo_msg)
                                    self.motor_pub.publish(self.motor_msg)
                                    if end-start>1.25:
                                        self.missioning2 = False
                                        break
                            else:
                                self.missioning2 = False
                        else:
                            self.missioning2 = False
                            self.stop=0
                            self.motor_msg=1000

               
                # #---------------- moving object case--------------------------------
                # if self.missioning==False:
                #     if self.judgement==0 or self.judgement==2:
                #         print('----judgement----------',self.judgement)
                #         if self.stop==0 and self.distance<=1.6 and (self.angle>=1.5 and self.angle<=4.5):
                #             self.judgement=2
                #             self.motor_msg=1000
                #             if self.distance < 0.9:
                #                 self.stop+=1
                #         elif self.stop!=0:
                #             self.motor_msg=0
                #             if self.distance > 0.8:
                #                 self.stop = -1
                #         elif self.stop==-1:
                #             self.motor_msg=1000
                        
                        


                #---------------- moving object case--------------------------------
                # else: 

                #     if self.missioning==False and self.distance <= 1.8 and (self.angle>=1.5 and self.angle<=4.5): 
                #         self.motor_msg=0

                #         print('moving object-',self.stop,self.now_speed)
                #         if self.now_speed == 0:
                #             self.stop= True

                #         if self.stop==True and self.distance>=0.95:
                #             self.motor_msg = 2000
                              
                    
                #     if self.angle<=1.3 or self.angle>=4.8:
                #         self.stop=False

                    
                local_path_pub.publish(local_path) ## Local Path 출력w

                # self.servo_msg = 0.0
                self.servo_pub.publish(self.servo_msg)
                self.motor_pub.publish(self.motor_msg)
                #self.print_info()
            
            if count==300 : ## global path 출력
                global_path_pub.publish(self.global_path)
                count=0
            count+=1

            
            rate.sleep()


    def print_info(self):

        os.system('clear')
        print('--------------------status-------------------------')
        print('position :{0} ,{1}, {2}'.format(self.status_msg.position.x,self.status_msg.position.y,self.status_msg.position.z))
        print('velocity :{} km/h'.format(self.status_msg.velocity.x))
        print('heading :{} deg'.format(self.status_msg.heading))

        print('--------------------object-------------------------')
        print('object num :{}'.format(self.object_num))
        for i in range(0,self.object_num) :
            print('{0} : type = {1}, x = {2}, y = {3}, z = {4} '.format(i,self.object_info[0],self.object_info[1],self.object_info[2],self.object_info[3]))

        print('--------------------controller-------------------------')
        print('target vel_planning :{} km/h'.format(self.cc_vel))
        print('target steering_angle :{} deg'.format(self.steering))

        print('--------------------localization-------------------------')
        print('all waypoint size: {} '.format(len(self.global_path.poses)))
        print('current waypoint : {} '.format(self.current_waypoint))


    def trafficInfoCB(self,data): ## Traffic Status Subscriber 
        # data.trafficLightStatus - traffic light information
        # 16 = green light  True = not green / False = green
        if data.trafficLightStatus != 16:
            self.status_traffic = True 
        else:
            self.status_traffic = False

    def lidarInfo(self,data):
        if data.points is not bool:
            #print('---------------------------------------')
            for i in range (0,len(data.points)):
                distance = sqrt((data.points[i].x)**2+(data.points[i].y)**2)
                if data.points[i].z>=1.5 and data.points[i].z<=4.5:
                    self.distance = distance
                    self.angle = data.points[i].z
    
    def speedInfo(self,data):
        self.now_speed = data.data
        
       
# x = 7.1, 7.5
# y = -2.0 -2.15
    def statusCB(self,data): ## Vehicl Status Subscriber 
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.is_status=True
                    
        # print(self.status_msg.yaw)
    
    #사용 금지
    def objectInfoCB(self,data): ## Object information Subscriber
        self.object_num=data.num_of_npcs+data.num_of_obstacle+data.num_of_pedestrian
        object_type=[]
        object_pose_x=[]
        object_pose_y=[]
        object_velocity=[]
        for num in range(data.num_of_npcs) :
            object_type.append(data.npc_list[num].type)
            object_pose_x.append(data.npc_list[num].position.x)
            object_pose_y.append(data.npc_list[num].position.y)
            object_velocity.append(data.npc_list[num].velocity.x)

        for num in range(data.num_of_obstacle) :
            object_type.append(data.obstacle_list[num].type)
            object_pose_x.append(data.obstacle_list[num].position.x)
            object_pose_y.append(data.obstacle_list[num].position.y)
            object_velocity.append(data.obstacle_list[num].velocity.x)

        for num in range(data.num_of_pedestrian) :
            object_type.append(data.pedestrian_list[num].type)
            object_pose_x.append(data.pedestrian_list[num].position.x)
            objecpure_pursuitct_info=[object_type,object_pose_x,object_pose_y,object_velocity]
        self.is_obj=True
    
    def speed_control(self):
        speed = 2500
        if abs(self.steering) > 5 :
            speed = 2000
        if abs(self.steering) > 7.5 :
            speed = 1250
        if abs(self.steering) > 10 :
            speed = 800
        return speed
       


if __name__ == '__main__':
    try:
        kcity_pathtracking=wecar_planner()
    except rospy.ROSInterruptException:
        pass