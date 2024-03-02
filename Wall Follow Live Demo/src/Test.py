#!/usr/bin/env python3

import numpy as np
import sys
import rospy
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
import matplotlib.pyplot as plt
import math
import random
import json
class Follow:
    # Right bordering following
    q_table = [
    #   0  1  2
    #   TL F TR
        [2,0,0], # Left Close  0
        [0,1,0], # Left Med    1
        [2,1,2], # Left Far    2
        [3,0,0], # Front Close 3
        [1,1,0], # Front Med   4
        [0,1,0], # Front Far   5
        [2,2,0], # Right Close 6
        [0,1,0], # Right Med   7
        [1,0,2]  # Right Far   8
            ]

    forward_move = Twist()
    forward_move.linear.x = .112

    left_turn = Twist()
    left_turn.angular.z = math.pi/4.5
    left_turn.linear.x = 0.01

    right_turn = Twist()
    right_turn.angular.z = -math.pi/3.75
    right_turn.linear.x = .05


            # L F R
    states = [0,0,0] # Distances index
    prev_state =  [0,0,0]
    # ex_states = [Far: 2, Far: 5, Med: 7]
    correct_actions = [0,0,0]
    total_actions = [0,0,0]
    
    correct_choice = {
        "Left": [[2,3,7], [1,3,7], [2,3,6], [2,3,8], [0,5,8], [0,3,6], [2,4,8], [2,4,7], [2,4,6]],
        "Forward": [[1,5,8], [2,5,6], [2,5,8]],
        "Right": [[0,3,8], [0,3,7], [0,4,8], [1,5,8], [1,5,7], [2,5,7]]
    }

    def __init__(self):
        # self.unpause()
        self.init_node()
        self.init_subscriber()
        self.init_publisher()

        n = len(sys.argv)
        # print("Total arguments passed:", sys.argv[0]) 
        # If marked to test from learned q-table file then load file and update q-table to learned one
        if(n > 1):
            with open(str(sys.argv[1]), 'r') as filehandle:
                # data = np.loadtxt(str(sys.argv[1]), dtype="float", delimiter='[')
                data = json.load(filehandle)
                self.q_table = list(data)
                # print(data)
        print("Start")
        self.scan = None
    
    def init_node(self):
        self.node = rospy.init_node("wall_follow", anonymous=True)
        pass
    def init_publisher(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pass
    def init_subscriber(self):
        rospy.Subscriber('/scan', LaserScan,  self.subscription_callback)
        pass
    episode = 0
    step = 0
    epsilon = .95
    lr = .1
    gamma = .99
    reward = .5
    action = None
    a = -1
    x_coords = []
    y_coords = []
    def subscription_callback(self, data):
        # rospy.wait_for_service('/gazebo/pause_physics')
        # self.pause()
        self.scan = data.header.stamp
        deg_inc = data.angle_increment * 180/math.pi
        print("Sub Scan Incr: "+str(deg_inc))
        print("Sub Scan Size: "+str(len(data.ranges)))
        # print("\tCallback: "+ str(data.ranges[270]))
        right_state = None
        left_state = None
        front_state = None

        # Right distance average
        avg = 0
        avg3 = 0
        avg2 = 0
        right_min = 10
        # for x in range(int(225/deg_inc),int(314/deg_inc)):
        #     avg += data.ranges[x]
        #     if right_min > data.ranges[x] and data.ranges[x] != 0:
        #         right_min = data.ranges[x]	
        for x in range(int(270/deg_inc),int(283/deg_inc)):
            avg2 += data.ranges[x]
            if right_min > data.ranges[x] and data.ranges[x] != 0:
                right_min = data.ranges[x]	
        # for x in range(int(200/deg_inc),int(224/deg_inc)):
        #     avg3 += data.ranges[x]
        # avg = avg/(314/deg_inc - 225/deg_inc)
        # avg2 = avg2/(279/deg_inc - 260/deg_inc)
        # avg3 = avg3/(200/deg_inc - 224/deg_inc)
        print("Right TOP Avg: "+str(right_min))
        if right_min < .55:# or data.ranges[int(240/deg_inc)] < .45 or data.ranges[int(270/deg_inc)] < .55:  # (avg < .5 and avg > .4 ) or data.ranges[315] < .2 or data.ranges[230] < .2 or 
            # data.ranges[int(300/deg_inc)] < .45 or avg2 < .5 or 
            print("6 Right Avg: "+str(right_min))
            right_state = 6
        elif right_min > .75: # (avg > .975) and avg3 > 2.12 and avg2 > 1.2 or 
            right_state = 8
            # print("8 Right Avg2: "+str(avg2))
            # print("8 Right Avg3: "+str(avg3))
        else:
            print("7 Right Avg: "+str(right_min))
            right_state = 7

        # Left distance average
        avg = 0
        left_min = 10
        for x in range(int(60/deg_inc),int(119/deg_inc)):
            avg += data.ranges[x]
            if left_min > data.ranges[x] and data.ranges[x] != 0:
                left_min = data.ranges[x]	
        avg = avg/(119/deg_inc-60/deg_inc)
        print("Left TOP Avg: "+str(left_min))
        if(left_min < .5 and left_min > .4):
            left_state = 1
        elif (left_min > .5):
            left_state = 2
        else:
            left_state = 0
        
        # Front distance average
        self.hit = False
        avg = 0
        avg2 = 0
        avg3 = 0
        front_min = 10
        # for x in range(0,int(44/deg_inc)):
        #     avg += data.ranges[x]
        for x in range(0,int(9/deg_inc)):
            if front_min > data.ranges[x] and data.ranges[x] != 0:
                front_min = data.ranges[x]	
        # for x in range(int(315/deg_inc),int(357/deg_inc)):
        #     avg2 += data.ranges[x]
        # for x in range(int(344/deg_inc), int(357/deg_inc)):
        #     avg3 += data.ranges[x]
        for x in range(int(325/deg_inc), len(data.ranges)):
            if front_min > data.ranges[x] and data.ranges[x] != 0:
                front_min = data.ranges[x]
        # avg += avg2
        # avg = avg/(359/deg_inc - 315/deg_inc + 44/deg_inc)
        # avg3 = avg3/(359/deg_inc - 344/deg_inc)
        print("Far TOP Avg: "+str(front_min))
        if front_min < .85 and front_min > .575: # (avg < .615 and avg > .3 and data.ranges[0] > .1 or avg2 < .85) or 
            print("4 Front distance: "+str(front_min))
            front_state = 4
            self.hit = False
        elif front_min > .85:# and data.ranges[0] > .1): (avg > 1.5 and avg2/45 > .25)  or 
            # print("Front Right distance: "+str(avg2/45))
            print("5 Far Front distance: "+str(front_min))
            front_state = 5
            self.hit = False
        elif (data.ranges[0] < .1 or avg3 < .12): # (avg < .36 and not (avg2/45 < .23 or 
            self.hit = False
            print("3 Front distance: "+str(front_min))
            front_state = 3
        elif data.ranges[0] < .05 or front_min <.1: # avg < .02 or avg2/45 < .23 or 
            front_state = 3
            self.hit = True
            print("HIT WALL: "+ str(front_min))
        else:
            front_state = 3
            self.hit = False

        self.states[0] = left_state
        self.states[1] = front_state
        self.states[2] = right_state
        # rospy.wait_for_service('/gazebo/unpause_physics')
        # self.unpause()
    

if __name__ == "__main__":
    print("Main")
    ros_interaction = Follow()
    
    #spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    '''
    # Replaces rospy.spin()
    while not rospy.shutdown():
        check if new scan
        for episodes
            set state
            for steps
                greedy or random
                state check
                learning check
                reward
                set new state
    '''
    scan_time = ros_interaction.scan
    epsilon = .90
    gamma = .99
    reward = .5
    correct_actions = [0,0,0]
    total_actions = [0,0,0]
    left_x_coords = []
    left_y_coords = []
    forward_x_coords = []
    forward_y_coords = []
    right_x_coords = []
    right_y_coords = []
    lr = .1
    count = 0
    fig, (left_turn, forward_move, right_turn) = plt.subplots(3)
    fig.suptitle("Percentage of Correct Moves")
    print("TEST BEGIN")
    while not rospy.is_shutdown():
        if (ros_interaction.scan != None):
            print("Start Scan: "+str(ros_interaction.scan))
            prev_state = ros_interaction.states
            for episode in range(1000):
                if rospy.is_shutdown():
                    break
                scan = ros_interaction.scan
                state = ros_interaction.states
                
                for l in range(250):
                    if rospy.is_shutdown():
                        break
                    # left_reward = ros_interaction.q_table[state[0]][0] + ros_interaction.q_table[state[1]][0] + ros_interaction.q_table[state[2]][0]
                    # forward_reward = ros_interaction.q_table[state[0]][1] + ros_interaction.q_table[state[1]][1] + ros_interaction.q_table[state[2]][1]
                    # right_reward = ros_interaction.q_table[state[0]][2] + ros_interaction.q_table[state[1]][2] + ros_interaction.q_table[state[2]][2]
                    left_reward = max([ros_interaction.q_table[state[0]][0], ros_interaction.q_table[state[1]][0],  ros_interaction.q_table[state[2]][0]])
                    forward_reward = max([ros_interaction.q_table[state[0]][1], ros_interaction.q_table[state[1]][1],  ros_interaction.q_table[state[2]][1]])
                    right_reward = max([ros_interaction.q_table[state[0]][2], ros_interaction.q_table[state[1]][2],  ros_interaction.q_table[state[2]][2]])
                    # left_reward = ros_interaction.q_table[state[0]][0]
                    # forward_reward = ros_interaction.q_table[state[1]][1]
                    # right_reward = ros_interaction.q_table[state[2]][2]
                    
                    

                    print("States:\t" + str(state[0]) +" | " + str(state[1]) + " | " + str(state[2]))
                    # print("Reward:\t" + str(left_reward) +" | " + str(forward_reward) + " | " + str(right_reward))
                    # self.prev_state = self.states
                    a = -1
                    action = None
                    # rospy.wait_for_service('/gazebo/pause_physics')
                    # ros_interaction.pause()
                    if random.uniform(0,1) > epsilon or len(sys.argv) > 1: # Greedy choice
                        if forward_reward >= left_reward and forward_reward >= right_reward:
                            ros_interaction.pub.publish(ros_interaction.forward_move)
                            print("Forward Publish")
                            action = "Forward"
                            a = 1
                        elif left_reward >= forward_reward and left_reward >= right_reward:
                            ros_interaction.pub.publish(ros_interaction.left_turn)
                            print("Left Publish")
                            action = "Left"
                            a = 0
                        elif right_reward > left_reward and right_reward > forward_reward:
                            ros_interaction.pub.publish(ros_interaction.right_turn)
                            print("Right Publish")
                            action = "Right"
                            a = 2
                        else:
                            ros_interaction.pub.publish(ros_interaction.forward_move)
                            print("Forward Publish")
                            action = "Forward"
                            a = 1
                        # print("\tGreedy: "+str(a))
                        # Observe new state by checking if scan is new
                        rospy.sleep(.25)
                        if(ros_interaction.scan != scan):
                            
                            scan = ros_interaction.scan
                            new_state = ros_interaction.states
                            if len(sys.argv) == 1: # Only learning check if not in test mode
                                # Learning Check to determine reward
                                print("\t\tLearn: "+ str(action))
                                if state in ros_interaction.correct_choice[action] and not ros_interaction.hit:
                                    print("\t\t\tCorrect")
                                    reward = .25
                                    correct_actions[a] += 1
                                elif ros_interaction.hit:
                                    reward = -20
                                    print("\t\t\tHit")
                                    # ros_interaction.random_spot()
                                    total_actions[a] += 1
                                    break # New episode due to collision
                                else:
                                    # print("\t\t\tElse")
                                    reward = -1
                                total_actions[a] += 1
                            # print("New scan: "+str(correct_actions[1])+", "+str(total_actions[1]))
                            
                                ros_interaction.q_table[state[0]][a] = (1-lr) * ros_interaction.q_table[state[0]][a] + lr * (reward + gamma) * max(ros_interaction.q_table[new_state[0]][:])
                                ros_interaction.q_table[state[1]][a] = (1-lr) * ros_interaction.q_table[state[1]][a] + lr * (reward + gamma) * max(ros_interaction.q_table[new_state[1]][:])
                                ros_interaction.q_table[state[2]][a] = (1-lr) * ros_interaction.q_table[state[2]][a] + lr * (reward + gamma) * max(ros_interaction.q_table[new_state[2]][:])
                            if new_state == state:
                                count += 1
                            else:
                                count = 0
                            # if count == 125 and len(sys.argv) > 1:
                            #     ros_interaction.random_spot()
                            #     print("\tLost")
                            state = new_state
                    else: # Random choice
                        a = random.randint(0,2)
                        # print("\tRandom: "+ str(a))
                        if(a == 0):
                            for i in range(5):
                                ros_interaction.pub.publish(ros_interaction.left_turn)
                            action = "Left"
                        elif a == 1:
                            ros_interaction.pub.publish(ros_interaction.forward_move)
                            action = "Forward"
                        elif a == 2:
                            for i in range(3):
                                ros_interaction.pub.publish(ros_interaction.right_turn)
                            action = "Right"

                        if(ros_interaction.scan != scan):
                            
                            scan = ros_interaction.scan
                            new_state = ros_interaction.states
                            state = new_state
                epsilon -= .0008
                # Calculate avg correct actions for episode
                for m in range(3):
                    if(total_actions[m] == 0):
                        # plt.plot(self.episode,0)
                        if m == 0:
                            left_x_coords.append(0)
                            left_y_coords.append(0)
                        elif m == 1:
                            forward_x_coords.append(0)
                            forward_y_coords.append(0)
                        elif m == 2:
                            right_x_coords.append(0)
                            right_y_coords.append(0)
                    else:
                        # x_coords.append(episode)
                        # y_coords.append((correct_actions[1]/total_actions[1]))
                        # print("Point: "+str(episode)+", "+str(correct_actions[1]/total_actions[1]))
                        if m == 0:
                            left_x_coords.append(episode)
                            left_y_coords.append(correct_actions[m]/total_actions[m])
                        elif m == 1:
                            forward_x_coords.append(episode)
                            forward_y_coords.append(correct_actions[m]/total_actions[m])
                        elif m == 2:
                            right_x_coords.append(episode)
                            right_y_coords.append(correct_actions[m]/total_actions[m])
                    left_turn.set_title("Left Turns")
                    left_turn.plot(left_x_coords,left_y_coords)
                    forward_move.set_title("Forward Moves")
                    forward_move.plot(forward_x_coords,forward_y_coords)
                    right_turn.set_title("Right Turns")
                    right_turn.plot(right_x_coords,right_y_coords)
                # Save Learning graph after completed steps
                plt.savefig('graph.png')
                with open('output.txt', 'w') as filehandle:
                    json.dump(ros_interaction.q_table, filehandle)
            break #Stop code from re-running after completing episodes
	#ros_interaction.pub.publish(Twist)
