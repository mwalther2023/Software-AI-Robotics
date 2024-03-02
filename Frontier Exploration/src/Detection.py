#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from threading import Thread
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
from PIL import Image as im
from cv_bridge import CvBridge
import cv2 as cv
from std_msgs.msg import String
from skimage import io
from skimage import data
from sklearn import cluster
from sklearn.cluster import KMeans
from sklearn.cluster import AgglomerativeClustering
import matplotlib.pyplot as plt
from tf import TransformListener
import tf
import math

class Detectiom:
    outGrid = OccupancyGrid()
    def __init__(self):
        self.init_node()
        self.init_subscriber()
        self.init_publisher()
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    def init_node(self):
        self.node = rospy.init_node("frontier_detection", anonymous=True)
        pass
    def init_publisher(self):
        self.pub = rospy.Publisher('/frontiers_map', OccupancyGrid, queue_size=10)
        self.mark = rospy.Publisher('/frontiers_markers', MarkerArray, queue_size=10)
        self.cent = rospy.Publisher('/centroid_markers', MarkerArray, queue_size=10)
        pass
    def init_subscriber(self):
        self.grid = OccupancyGrid()    
        rospy.Subscriber('/map', OccupancyGrid,  self.subscription_callback)
        pass
    def subscription_callback(self, data):
        if(self.grid.data != data.data):
            self.grid = data
            print("Different")
            self.find_frontiers()
        else:
            print("Same")
        
    def find_frontiers(self):
        print("\tSearch")
        self.res = self.grid.info.resolution
        self.o_y = self.grid.info.origin.position.y
        self.o_x = self.grid.info.origin.position.x
        self.outGrid.header = self.grid.header
        self.outGrid.info = self.grid.info
        self.outGrid.data = np.copy(self.grid.data)
        # print(self.outGrid.data)
        x = 0
        temp = list(self.outGrid.data)
        
        while x < len(temp) and not rospy.is_shutdown():
            temp[x] = 0
            x += 1
        print("\t\tSet Empty outGrid")
        self.outGrid.data = tuple(temp)
        # print(self.outGrid.data)

        o = list(self.outGrid.data)
        i = 1
        g = list(self.grid.data)
        while i < len(g)-1 and not rospy.is_shutdown(): 
            # Search for possible frontiers
            if(g[i] == 0 and (g[i+1] == -1 or g[i-1] == -1)):
                # Set frontier
                # print("\t\t\tFound Frontier")
                o[i] = 100
            # Grow obstacles
            elif(g[i] == 100):
                o[i+1] = 0
                o[i] = 0
                o[i-1] = 0
            i += 1 
        print("\t\tSet Frontiers")
        self.outGrid.data = tuple(o)
        # print(self.outGrid.data)
        try:
            self.pub.publish(self.outGrid)
            
            # Contour and Segment the OccuGrid
            self.segment(self.outGrid)
            print("Grid Published")
            # Begin method calls for finding places to explore
            self.travel()
        except rospy.ROSInterruptException:
            pass
    def occupancygrid_to_numpy(self, msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        return data   
    
    def cell_to_coord(self, cell):
        (x,y) = cell
        return [(y+0.5) * self.res + self.o_y, (x+0.5) * self.res + self.o_x] # note flipped

    def coord_to_cell(self, coord):
        (x, y) = coord
        id_x = (y - self.o_y) / self.res
        id_y = (x - self.o_x) / self.res
        return (int(id_x), int(id_y))
    def travel(self):
        listener = tf.TransformListener()
        pos = None
        while pos is None and not rospy.is_shutdown():
            try:
                # Perform lookupTransform for turtle
                
                (pos,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                print("\tTravel")
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        self.goal_finder(pos, self.centroid)
     
    def segment(self, data):
        # cv.imwrite('temp.png', list(data.data))
        temp = self.occupancygrid_to_numpy(data)
        # print(np.uint8(temp*255).shape)
        cv_img = im.fromarray(temp, 'L')
        filename = "img.png"
        cv_img.save(filename)    

        img = cv.imread(filename)
        # Grayscale
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find Canny edges
        edged = cv.Canny(gray, 30, 200)
        # Finding Contours
        contours, hierarchy = cv.findContours(edged, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
        # cv.imshow('Canny Edges After Contouring', edged)
        cv.imwrite("edges.png", edged)
        # Draw all contours
        cv.drawContours(img, contours, -1, (0, 255, 0), 3)
        cv.imwrite('Contours.png', img)
        
        # image = io.imread(filename,as_gray=True)/255.0
        # h, w = image.shape
        # image_2d = image.reshape(h*w,1)
        # numcolors = 2
        # kmeans_cluster = cluster.KMeans(numcolors)
        # kmeans_cluster.fit(image_2d)
        # cluster_centers = kmeans_cluster.cluster_centers_
        # cluster_labels = kmeans_cluster.labels_
        # newimage = cluster_centers[cluster_labels].reshape(h, w)*255.0
        # newimage = newimage.astype('uint8')
        # lower = (100)
        # upper = (200)
        # thresh = cv.inRange(newimage, lower, upper)
        # cntrs_info = []
        # contours = cv.findContours(thresh, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        # if(len(contours) == 2):
        #     contours = contours[0]
        # else:
        #     contours = contours[1]
        # index=0
        # for cntr in contours:
        #     area = cv.contourArea(cntr)
        #     cntrs_info.append((index,area))
        #     index = index + 1
        # def takeSecond(elem):
        #     return elem[1]
        # cntrs_info.sort(key=takeSecond, reverse=True)
        # result = np.zeros_like(newimage)
        # index_first = cntrs_info[0][0]
        # cv.drawContours(result,[contours[index_first]],0,(255),-1)
        # index_second = cntrs_info[1][0]
        # cv.drawContours(result,[contours[index_second]],0,(255),-1)
        # cv.imwrite("kmeans_thresh.png",thresh)


        print("Number of Contours found = " + str(len(contours)))
        kmeans = KMeans(n_clusters=6, init='k-means++', random_state=0)
        hc_model = AgglomerativeClustering(n_clusters=6)
        count = 0
        i = 0
        self.arr = []
        # Adjust (x,y) position for contours to match map layout
        while count < len(contours) and not rospy.is_shutdown():
            pts = contours[count]
            
            for cell in pts:
                (x_coord, y_coord) = self.cell_to_coord(cell[0])
                self.arr.append([y_coord, x_coord])
                # arr.append(x_coord)
                
                i += 1

            count += 1  
        self.seg = hc_model.fit_predict(self.arr)
        self.centroid = kmeans.fit(self.arr).cluster_centers_
        # print(seg)
        print("Self.centroid: "+str(self.centroid))
        print("Segmented: "+str(self.seg))
        self.markerArray = MarkerArray()
        self.delArray = MarkerArray()
        self.delMarker = self.make_frontier_marker([0,0],[0,0,0,1],1000,.25,2,3)
        self.delArray.markers.append(self.delMarker)
        self.mark.publish(self.delArray)
        count = 0
        i = 0
        # Create and seperately color each segmented frontier
        while count < len(contours) and not rospy.is_shutdown():
            pts = contours[count]
            
            for cell in pts:
                (x_coord, y_coord) = self.cell_to_coord(cell[0])
                if(self.seg[i] == 0):
                    marker = self.make_frontier_marker([y_coord,x_coord],[1,0,0,1],i)
                elif(self.seg[i] == 1):
                    marker = self.make_frontier_marker([y_coord,x_coord],[0,1,0,1],i)
                elif(self.seg[i] == 2):
                    marker = self.make_frontier_marker([y_coord,x_coord],[0,0,1,1],i)
                elif(self.seg[i] == 3):
                    marker = self.make_frontier_marker([y_coord,x_coord],[1,0,1,1],i)
                elif(self.seg[i] == 4):
                    marker = self.make_frontier_marker([y_coord,x_coord],[1,1,0,1],i)
                elif(self.seg[i] == 5):
                    marker = self.make_frontier_marker([y_coord,x_coord],[0,1,1,1],i)
                else:
                    marker = self.make_frontier_marker([y_coord,x_coord],[0,0,0,1],i)   
                i += 1
                self.markerArray.markers.append(marker)
            count += 1

        
        self.markerArray.markers.append(marker)
        self.mark.publish(self.markerArray)
        print("Marker Published "+str(len(self.markerArray.markers)))
        self.centroidArray = MarkerArray()   
        self.cent.publish(self.delArray)
        i = 0
        # Create markers for each centroid found
        for cell in self.centroid:
            x_coord = cell[0]
            y_coord = cell[1]
            marker = self.make_frontier_marker([x_coord,y_coord],[1,1,1,1],i, .5)
            self.centroidArray.markers.append(marker)
            i += 1
        self.cent.publish(self.centroidArray)
        print("Centroids Published "+str(len(self.centroidArray.markers)))


    bad_goals = []
    
    def goal_finder(self, pos, centroids):
        self.client.wait_for_server()
        pose = Pose()
        farthest = 10000
        goal_point = []
        index = 0
        goal_index = 0
        goal_centroid = Marker()
        # Find centroid with closest distance from robot
        for cell in centroids:
            point1 = [pos[0],pos[1]]
            point2 = [cell[0],cell[1]]
            dist = math.dist(point1,point2)
            # Check if this point is closer than previous close point found
            if(dist < farthest and ([round(point2[0],3),round(point2[1],3)] not in self.bad_goals)):
                bad_point = False
                for x in self.bad_goals:
                    # Check if this point is too close to a previous unrachable spot
                    if(abs(math.dist(point2,x)) < .075):
                        bad_point = True
                # Check if this point was not unreachable before
                if(not bad_point):
                    farthest = dist
                    goal_point = point2
                    goal_point[0] = round(goal_point[0],3)
                    goal_point[1] = round(goal_point[1],3)
                    goal_index = index
            index += 1
        i = 0
        # Find centroid marker that is the goal to remove and replace with one of different color to display
        for marker in self.centroidArray.markers:
            if round(marker.pose.position.x,3) == goal_point[0] and round(marker.pose.position.y,3) == goal_point[1]:
                goal_centroid = marker
                goal_centroid.color.r = 0
                goal_centroid.color.g = 0
                goal_centroid.color.b = 0
                self.centroidArray.markers.pop(i)
            i+=1

        # Republish centroid markers to show goal marker as different from the rest
        self.centroidArray.markers.append(goal_centroid)
        self.cent.publish(self.delArray)
        self.cent.publish(self.centroidArray)
        print("Goal Marker Published")

        # Setup goal to send to client
        # temp = self.coord_to_cell(goal_point)
        # print("\t\tPoint: "+str(self.coord_to_cell(goal_point)))
        # print("\t\tCoord: "+str(self.cell_to_coord(self.coord_to_cell(goal_point))))

        # Create pose to be used in goal
        print("\t\tGoal Point: "+str(goal_point))
        pose.position.x = goal_point[0] - round(pos[0],3)
        pose.position.y = goal_point[1] - round(pos[1],3)
        print("\t\tGoal Pose: ("+str(pose.position.x)+", "+str(pose.position.y)+")")
        pose.position.z = 0.0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 1
        # pose.orientation.w = 1.0
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_footprint'
        goal.target_pose.pose = pose
        self.client.send_goal(goal)

        result = None
        listener = tf.TransformListener()
        try:
            while result is None and not rospy.is_shutdown():
                result = self.client.get_result()
                try:
                    # Perform lookupTransform for turtle
                    (pos,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                    pos = [pos[0],pos[1]]
                    close = round(math.dist(pos,goal_point),3)
                    # print("\t\t\t"+str(close))
                    # print("\t\t\t"+str(pos))

                    # Check if goal point is close enough to mark as reached
                    if(close <= .05):
                        self.client.cancel_goal()
                        print("\t\t\tGoal Close enough")
                        print("\t\t\t"+str(pos))
                        # Raise exception to exit loop
                        raise Exception("Aborted Goal")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
                
                # Check if goal has been aborted
                if(self.client.get_state() == GoalStatus.ABORTED):
                    # Mark frontiers for this segmet as obstsacles to remove potential future exploration there
                    self.unreachable_frontier(goal_index)
                    # Exit loop
                    raise Exception("Aborted Goal")
                # IMPORTANT: Sleep here or your program will hang
            print("\t\t\t"+str(pos))
            print("\t\t\tGoal Reached")
            self.bad_goals.append(goal_point)
        except Exception:
            
            self.bad_goals.append(goal_point)
            self.client.cancel_goal()
            print("\t\t\tGoal Aborted"+str(self.bad_goals))
            pass
        print("\t\tGoal Finder")
    def unreachable_frontier(self, goal_index):
        print(self.centroid[goal_index])
        seg_index = 0
        mark = []
        for s in self.seg:
            if(s == goal_index):
                mark.append(seg_index)
            seg_index += 1

        i = 0
        o = list(self.outGrid.data)
        # print("Grid: " +str(o))

        # Mark frontier spots as obstacles to remove possible exploration in future
        while i < len(o)-1 and not rospy.is_shutdown(): 
            if(i in mark):
                o[i] = 0
            i += 1 
        
        self.outGrid.data = tuple(o)
        try:
            self.pub.publish(self.outGrid)            
            # self.segment(self.outGrid)
            print("Abandoned Grid Published")
        except rospy.ROSInterruptException:
            pass

    def make_frontier_marker(self, loc, rgba=[1.,0.,0.,1.], k=0, scale=.25, t=2, a=0):
        f_marker = Marker()
        f_marker.header.frame_id = "map"
        f_marker.header.stamp    = rospy.get_rostime()
        f_marker.ns = "frontiers"
        f_marker.id = k
        f_marker.type = t
        f_marker.action = a
        f_marker.pose.position.x = loc[0]
        f_marker.pose.position.y = loc[1]
        f_marker.pose.position.z = 0
        f_marker.pose.orientation.x = 0
        f_marker.pose.orientation.y = 0
        f_marker.pose.orientation.z = 0
        f_marker.pose.orientation.w = 1.0
        f_marker.scale.x = scale
        f_marker.scale.y = scale
        f_marker.scale.z = scale
        f_marker.color.r = rgba[0]
        f_marker.color.g = rgba[1]
        f_marker.color.b = rgba[2]
        f_marker.color.a = rgba[3]
        f_marker.lifetime = rospy.Duration(0)
        return f_marker
if __name__ == "__main__":
    ros_interaction = Detectiom()
    
    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()