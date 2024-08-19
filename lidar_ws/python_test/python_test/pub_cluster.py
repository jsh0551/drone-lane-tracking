#!/usr/bin/env python3
# encoding: utf-8

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np

import math



class Lidar:
    def __init__(self):
        self.pub_range = rospy.Publisher('lidar_data',Point,queue_size=1)        
        rospy.Subscriber('scan',LaserScan,self.laser_callback,queue_size=1)
        self.xyz_data = Point()
        self.xyz_data.x = 0
        self.xyz_data.y = 0
        self.xyz_data.z = 0
        

                
    def laser_callback(self,scan_data):
        ## -60 ~ 60
        if not isinstance(scan_data,LaserScan):return
        # frontDistList = []
        # frontDistIDList = []
        ranges = np.array(scan_data.ranges)
        none = []   
        polar1 = np.array(none)
        cpoints = 0
        numb = 0
        # print("call_back")
        for i in range(len(ranges)): 
            if ranges[i] != np.inf:
            # if ranges[i] != np.inf and ranges[i] < 2.0 and (65<i and i <95):
                cpoints = cpoints +1
            # if ranges[i] <= 1.5:
            
        # print("cpoints ", cpoints)

        c_points = cpoints
        polar = np.zeros([c_points+1,2])

        for i in range(len(ranges)):   
            if ranges[i] != np.inf:
            # if ranges[i] != np.inf and ranges[i] < 2.0 and (65<i and i <95):

                polar[numb][0] = ranges[i]
                polar[numb][1] = scan_data.angle_min + scan_data.angle_increment * i
                # print(" range \t", polar[numb][0], " angle \t",polar[numb][1])
                # print("angle",polar[numb][1])
                numb=numb+1

        polar[c_points] = polar [0]
        # 오류없음

        # while(1): pass

        # clustered1 = np.array(c_points+1,False,dtype = bool)
        # clustered2 = np.array(c_points+1,True,dtype = bool)

        clustered1 = np.zeros(c_points+1, dtype = bool)
        clustered2 = np.zeros(c_points+1, dtype = bool)

        # print("first \t" ,clustered1 ,"second \t" , clustered2)
        d = np.zeros(1)
        for i in range(c_points):
            d = np.sqrt( np.power(polar[i][0],2) + np.power(polar[i+1][0],2)-2 * polar[i][0]*polar[i+1][0]*np.cos(polar[i+1][1] - polar[i][1]))

            if (d<0.32):
                clustered1[i] = True
                clustered2[i+1] = True
            # print("first \t" ,clustered1 ,"second \t" , clustered2)
        # temp = clustered2[c_points]
        # clustered2[0] = temp
        # clustered2[0] = clustered2[c_points]

        begin = np.zeros(c_points+1)
        nclus = np.zeros(c_points+1)

        # begin = np.array(none)
        # nclus = np.array(none)

        begin_idx = 0
        nclus_idx = 0
        nclus_cnt = 0

        flag = True

        i2 = 0

        while i2<c_points and flag == True:
            if clustered1[i2] == True and clustered2[i2] == False and flag == True:
                begin[begin_idx] = i2
                begin_idx+=1
                nclus_cnt = 1    #nclus[nclus_idx] = 1
                # nclus_idx+=1

                #print ("begin \t", begin,"nclus \t", nclus)

                while clustered2[i2+1] == True and clustered1[i2+1] == True:
                    i2+=1
                    nclus_cnt +=1   #nclus[nclus_idx]=nclus[nclus_idx]+1
                    if(i2 == c_points-1 and flag == True):
                        i2 = -1
                        flag = False
                nclus_cnt +=1       #nclus[nclus_idx]=nclus[nclus_idx]+1
                nclus[nclus_idx] = nclus_cnt
                nclus_idx+=1
            i2+=1
        # print ("begin \t", begin,"nclus \t", nclus)

        if(clustered1[cpoints-1]==True and clustered2[cpoints-1]==False):
            begin[begin_idx] = cpoints-1
            nclus[nclus_idx] = 1
            i2 = 0

            while(clustered2[i2] == True and clustered1[i2] == True):
                i2+=1
                nclus[nclus_idx]=nclus[nclus_idx]+1
                nclus_idx+=1 

        # leng = len(polar)  
        # print("begin_idx \t",len(begin))   
        # X_data = 0

        for i in range(begin_idx):
            print(begin[i])
            print("begin_idx \t")
            for j in range(nclus_idx):
                print(nclus[j])
        print("nclus_idx \t",len(nclus))   



        # RangeList = []
        # AngleDList = []
        
        # if(begin_idx >= 1):
        #     # print("len(begin) \t",len(begin))
        #     for i in range(begin_idx):
                
        #         Begin = begin[i]
                
        #         Nclus = nclus[i]
        #         middle = begin[i]+round((nclus[i]/2)-1,1)
        #         Range = polar[int(middle)][0]
        #         Angle = polar[int(middle)][1]*180/math.pi
        #         Radians = polar[int(middle)][1]   
                
        #         Range = float(Range*10)
        #         Angle = float(Angle*10)
                
        #         RangeList.append(Range)
        #         AngleDList.append(Angle) 
                

        #     minDist = min(RangeList)
                                
        #     numDist = RangeList.index(min(RangeList))
            
        #     angDist = AngleDList[numDist]
                           
        #     self.xyz_data.x = minDist
        #     self.xyz_data.y = angDist     
            
        #     # print(self.xyz_data.x)

        #     self.pub_range.publish(self.xyz_data)
            
  
                    

 
if __name__ =="__main__":
    rospy.init_node("lidar_mod")
    lidar = Lidar()
    rospy.spin()
