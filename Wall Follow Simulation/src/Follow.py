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
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
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
    # q_table = [ # Zero Q-Table
    # #   0  1  2
    # #   TL F TR
    #     [0,0,0], # Left Close  0
    #     [0,0,0], # Left Med    1
    #     [0,0,0], # Left Far    2
    #     [0,0,0], # Front Close 3
    #     [0,0,0], # Front Med   4
    #     [0,0,0], # Front Far   5
    #     [0,0,0], # Right Close 6
    #     [0,0,0], # Right Med   7
    #     [0,0,0]  # Right Far   8
    #         ]
    forward_move = Twist()
    forward_move.linear.x = .112

    left_turn = Twist()
    left_turn.angular.z = math.pi/3
    left_turn.linear.x = -.065

    right_turn = Twist()
    right_turn.angular.z = -math.pi/3.5
    right_turn.linear.x = .065

    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
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
    '''
    E = .9 # get down to .1 towards end of episodes (1000)
    (Learning check)
        if greedy choice
            if {states} and A is the "Correct choice" = array of definite actions based on state
                correct{a} += 1
            total{a} += 1
        else
            random A
        correct{a}/total{a} (Per episode)
    '''
    '''
    Episode = 1000 : Epsilon = .9
        Steps (until crash or 250)
            random or greedy choice: random.uniform[0,1] > Epsilon (Greedy = higher Q-value action from Q-table)
            learning check
                correct
                total
        correct/total
        Epsilon -= some number

    '''
    

    def __init__(self):
        self.unpause()
        self.init_node()
        self.init_subscriber()
        self.init_publisher()
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_simulation()
        print("Reset")
        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_burger'
        state_msg.reference_frame = 'world'
        state_msg.pose.position.x = -2
        state_msg.pose.position.y = -2
        # state_msg.pose.position.z = 0.3
        # state_msg.pose.orientation.x = 0
        # state_msg.pose.orientation.y = 0
        # state_msg.pose.orientation.z = 1
        # state_msg.pose.orientation.w = 0
        n = len(sys.argv)
        print("Total arguments passed:", sys.argv[0]) 
        # If marked to test from learned q-table file then load file and update q-table to learned one
        if(n > 1):
            with open(str(sys.argv[1]), 'r') as filehandle:
                # data = np.loadtxt(str(sys.argv[1]), dtype="float", delimiter='[')
                data = json.load(filehandle)
                self.q_table = list(data)
                # print(data)
        rospy.wait_for_service('/gazebo/set_model_state')
        
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(state_msg)
            print(resp)
        except rospy.ServiceException:
            print ("Service call failed")
        '''
        
        '''
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
        # print("Sub Scan: "+str(self.scan))
        # print("\tCallback: "+ str(data.ranges[270]))
        right_state = None
        left_state = None
        front_state = None

        # Right distance average
        avg = 0
        avg3 = 0
        avg2 = 0
        for x in range(225,314):
            avg += data.ranges[x]
        for x in range(260,279):
            avg2 += data.ranges[x]
        for x in range(200,224):
            avg3 += data.ranges[x]
        avg = avg/90
        avg2 = avg2/20
        avg3 = avg3/25
        # print("Avg: "+str(avg))
        if data.ranges[270] < .45 or avg2 < .5: # (avg < .5 and avg > .4 ) or data.ranges[315] < .2 or data.ranges[230] < .2 or 
            print("6 Right Avg: "+str(avg2))
            right_state = 6
        elif (avg > .975) and avg3 > 2.12 and avg2 > 1.2:
            right_state = 8
            print("8 Right Avg2: "+str(avg2))
            print("8 Right Avg3: "+str(avg3))
        else:
            print("7 Right Avg: "+str(data.ranges[270]))
            right_state = 7

        # Left distance average
        avg = 0
        for x in range(60,119):
            avg += data.ranges[x]
        avg = avg/60
        # print("Avg: "+str(avg))
        if(avg < .5 and avg > .4):
            left_state = 1
        elif (avg > .5):
            left_state = 2
        else:
            left_state = 0
        
        # Front distance average
        self.hit = False
        avg = 0
        avg2 = 0
        avg3 = 0
        for x in range(0,44):
            avg += data.ranges[x]
        for x in range(315,359):
            avg2 += data.ranges[x]
        for x in range(344, 359):
            avg3 += data.ranges[x]
        avg += avg2
        avg = avg/90
        avg3 = avg3/15
        # print("Avg: "+str(avg))
        if(avg < .615 and avg > .3 and data.ranges[0] > .1):
            print("Front distance: "+str(avg))
            front_state = 4
            self.hit = False
        elif (avg > .9 and avg2/45 > .25 and data.ranges[0] > .1):
            # print("Front Right distance: "+str(avg2/45))
            print("Far Front distance: "+str(avg))
            front_state = 5
            self.hit = False
        elif (avg < .36 and not (avg2/45 < .23 or data.ranges[0] < .1) or avg3 < .12):
            self.hit = False
            front_state = 3
        elif (avg < .02 or avg2/45 < .23 or data.ranges[0] < .05):
            front_state = 3
            self.hit = True
            print("HIT WALL: "+ str(avg2/45))
        else:
            front_state = 3
            self.hit = False

        self.states[0] = left_state
        self.states[1] = front_state
        self.states[2] = right_state
        # rospy.wait_for_service('/gazebo/unpause_physics')
        # self.unpause()
    
    start_points = [[-2,-2], [2,-2], [2,0]]
    def random_spot(self):
        if len(sys.argv) == 1:
            print("Reset")
            state_msg = ModelState()
            state_msg.model_name = 'turtlebot3_burger'
            state_msg.reference_frame = 'world'
            r = random.randint(0,len(self.start_points)-1)
            state_msg.pose.position.x = self.start_points[r][0]
            state_msg.pose.position.y = self.start_points[r][1]
            
            if(r != 0):
                state_msg.pose.orientation.z = 0.7071068
                state_msg.pose.orientation.w = 0.7071068
            else:
                state_msg.pose.orientation.z = 0
            rospy.wait_for_service('/gazebo/set_model_state')
            
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state(state_msg)
                print(resp)
            except rospy.ServiceException:
                print ("Service call failed")
if __name__ == "__main__":
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
                    print("Reward:\t" + str(left_reward) +" | " + str(forward_reward) + " | " + str(right_reward))
                    # self.prev_state = self.states
                    a = -1
                    action = None
                    rospy.wait_for_service('/gazebo/pause_physics')
                    ros_interaction.pause()
                    if random.uniform(0,1) > epsilon or len(sys.argv) > 1: # Greedy choice
                        if forward_reward >= left_reward and forward_reward >= right_reward:
                            ros_interaction.pub.publish(ros_interaction.forward_move)
                            action = "Forward"
                            a = 1
                        elif left_reward >= forward_reward and left_reward >= right_reward:
                            for i in range(4):
                                ros_interaction.pub.publish(ros_interaction.left_turn)
                            action = "Left"
                            a = 0
                        elif right_reward > left_reward and right_reward > forward_reward:
                            for i in range(3):
                                ros_interaction.pub.publish(ros_interaction.right_turn)
                            action = "Right"
                            a = 2
                        else:
                            ros_interaction.pub.publish(ros_interaction.forward_move)
                            action = "Forward"
                            a = 1
                        print("\tGreedy: "+str(a))
                        # Observe new state by checking if scan is new
                        rospy.wait_for_service('/gazebo/unpause_physics')
                        ros_interaction.unpause()
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
                                    print("\t\t\tElse")
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
                        print("\tRandom: "+ str(a))
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
                        # rospy.wait_for_service('/gazebo/unpause_physics')
                        # ros_interaction.unpause()
                        # rospy.sleep(.25)
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
                '''
                # End condition for test mode
                if(len(sys.argv) != 1):
                    # If completed lap around exit
                    if episode > 200:
                        listener = tf.TransformListener()
                        try:
                            # Perform lookupTransform for turtle
                            (pos,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                            pos = [pos[0],pos[1]]
                            close = round(math.dist(pos,(-2,-2)),3)
                            # print("\t\t\t"+str(close))
                            # print("\t\t\t"+str(pos))

                            # Check if goal point is close enough to mark as reached
                            if(close <= .05):

                                print("\t\t\tGoal Close enough")
                                print("\t\t\t"+str(pos))
                                # Break episode loop
                                break
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            pass
                '''
                

            break #Stop code from re-running after completing episodes