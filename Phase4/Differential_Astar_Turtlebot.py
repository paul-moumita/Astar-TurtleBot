# Author : Moumita Paul
# Project3 Differential Astar Turtlebot 


import pygame
import numpy as np 
import sys
import os
import math
from time import time
from pygame.locals import *
import argparse

import rospy
from geometry_msgs.msg import Twist

# Colors
sky_blue = [135,206,235]
red = [255,0,0]
lime = [0,255,0]
white = [255,255,255]

# Robot Parameters
radius=0.076/2
l = 0.230
done = False

#-------------------------------------#
#           Helper Functions          #
#-------------------------------------#

def goal_achieved(node,goal,d):
    c=0
    if (node[0]-goal[0])**2+(node[1]-goal[1])**2<(d)**2:
        c=1
    return c

def movement(node,ul,ur,theta):
    n_nd=[0,0]
    x=node[0]
    y=node[1]
    for i in range(0,400):
        d_theta = (radius/l)*(ur-ul)*0.005
        dx = (radius/2)*(ul+ur)*(math.cos(theta))*0.005
        dy = (radius/2)*(ul+ur)*(math.sin(theta))*0.005
        x = x + dx
        y = y + dy
        theta = theta + d_theta

    n_nd[0]=(n_nd[0]+x)
    n_nd[1]=(n_nd[1]+y)
    n_nd = [ ((math.floor(n_nd[0] * 100)) / 100.0), ((math.floor(n_nd[1] * 100)) / 100.0) ]
    return n_nd,theta

def checkPoint(node, points):
    flag = 0
    for point in points:
        if (((node[0] - point[0])**2 + (node[1] - point[1])**2) - 0.1**2 < 0):
            return True
    return False

# Creating Map with obstacles
def Map(x,y, resolution, d):

    q = 0
    
    if (x< (d/resolution)) or (x >(100-d)/resolution) or (y<(d/resolution)) or (y>(100-d)/resolution):
        q= 1

        # Center circle
    if ((x-math.ceil(50/resolution))**2+(y-math.ceil(50/resolution))**2)<math.ceil((10+d)/resolution)**2:
        q = 1

        # Top right circle
    if ((x-math.ceil(70/resolution))**2+(y-math.ceil(80/resolution))**2)<math.ceil((10+d)/resolution)**2:
        q = 1

        # Bottom right circle
    if ((x-math.ceil(70/resolution))**2+(y-math.ceil(20/resolution))**2)<math.ceil((10+d)/resolution)**2:
        q = 1
    
        # Bottom left circle
    if ((x-math.ceil(30/resolution))**2+(y-math.ceil(20/resolution))**2)<math.ceil((10+d)/resolution)**2:
        q = 1

    if ((22.5-d/resolution) <= x <= (37.5+d/resolution) and (72.5-d/resolution) <= y <= (87.5+d/resolution)):
        q = 1

        
    if ((2.5-d/resolution) <= x <= (15+d/resolution) and (42.5-d/resolution) <= y <= (57.5+d/resolution)):
        q = 1

        
    if ((82.5-d/resolution) <= x <= (97.5+d/resolution) and (42.5-d/resolution) <= y <= (57.5+d/resolution)):
        q = 1
    
    return q

#  To display obstacles 
def obstacle_disp_pts():
    

    circle_pts1 = [50,50,10]
    circle_pts2 = [70,20,10]
    circle_pts3 = [70,80,10]
    circle_pts4 = [30,80,10]

    return circle_pts1,circle_pts2, circle_pts3, circle_pts4

# Heuristic
def heuristic(node,goal):
    h = math.sqrt ( (node[0] - goal[0])**2 +  (node[1] - goal[1])**2 )
    return h


def main(Args):

    print("Differential A-star Algorithm")

    #Inputs
    start = Args.start
    goal = Args.goal
    resolution = Args.resolution
    rpm1 = Args.rpm1
    rpm2 = Args.rpm2
    clearance = Args.clearance
    scale = Args.scale
    pygame_flag = Args.pygame
    gazebo_flag = Args.gazebo
    theta_i = Args.theta

    rows = 100/resolution
    coloums = 100/resolution 

    start = [10*m/resolution for m in start]
    goal = [10*n/resolution for n in goal]

    #-------------------------------------#
    #         Exploration of Robot        #
    #-------------------------------------#
    point_node = [start]
    c_nd = [start]
    heuristic_node = [round(heuristic(start,goal),2)]
    vp_nd=[]
    vc_nd=[]
    v_cst=[]
    vh_nd=[]
    v_theta=[]
    v_act=[]


    # Workspace  defined
    if (Map(goal[0],goal[1],resolution,radius+clearance) or Map(start[0],start[1],resolution,radius+clearance)):
        sys.exit(" Error: goal point or start point lies within the obstacles")
    if(start[0] not in range (0,100*scale+1) or goal[0] not in range (0,100*scale+1) or start[1] not in range(0,100*scale+1) or goal[1] not in range(0,100*scale+1)):
        sys.exit("Error: Entered point outside the workspace")

    print("Exploration Started")

    start_time = time()
    x=0 
    cst = [0]
    ndx = start
    flag = 0
    exit = 0 
    count =0
    theta = [theta_i]
    act=[[0,0]]
    start_time = time()
    while(flag!=1):
        
        nd, new_theta = movement(ndx,0,rpm1,theta[x])
        if(Map(nd[0],nd[1],resolution,radius+clearance)!=1):
            if not checkPoint(nd, vc_nd): 
                xl=range(0,len(c_nd))
                xl = xl[::-1]
                check = 0
                for p in xl:
                    if (nd == c_nd[p]):
                        check= 1
                        if (cst[p]>=(cst[x]+0.9)):
                            point_node[p]=ndx
                            cst[p]=round((cst[x]+0.9),3)
                            heuristic_node[p] = round((cst[x] + 0.9 + heuristic(nd,goal)),2)
                            act[p] = [0,rpm1]
                            theta[p] = new_theta
                            break
                if(check!=1):
                    point_node.append(ndx)
                    c_nd.append(nd)
                    theta.append(new_theta)
                    cst.append(round(0.9+cst[x],3))
                    heuristic_node.append(round((0.9+cst[x]+heuristic(nd,goal)),2))
                    act.append([0,rpm1])

        nd, new_theta = movement(ndx,0,rpm2,theta[x])
        if(Map(nd[0],nd[1],resolution,radius+clearance)!=1):
            if not checkPoint(nd, vc_nd): 
                xl=range(0,len(c_nd))
                xl = xl[::-1]
                check = 0
                for p in xl:
                    if (nd == c_nd[p]):
                        check= 1
                        if (cst[p]>=(cst[x]+0.8)):
                            point_node[p]=ndx
                            cst[p]=round((cst[x]+0.8),3)
                            heuristic_node[p] = round((cst[x] + 0.8 + heuristic(nd,goal)),2)
                            act[p] = [0,rpm2]
                            theta[p] = new_theta
                            break
                if(check!=1):
                    point_node.append(ndx)
                    c_nd.append(nd)
                    theta.append(new_theta)
                    cst.append(round(0.8+cst[x],3))
                    heuristic_node.append(round((0.8+cst[x]+heuristic(nd,goal)),2))
                    act.append([0,rpm2])
        
        nd, new_theta = movement(ndx,rpm1,rpm2,theta[x])
        if(Map(nd[0],nd[1],resolution,radius+clearance)!=1):
            if not checkPoint(nd, vc_nd): 
                xl=range(0,len(c_nd))
                xl = xl[::-1]
                check = 0
                for p in xl:
                    if (nd == c_nd[p]):
                        check= 1
                        if (cst[p]>=(cst[x]+0.6)):
                            point_node[p]=ndx
                            cst[p]=round((cst[x]+0.6),3)
                            heuristic_node[p] = round((cst[x] + 0.6 + heuristic(nd,goal)),2)
                            act[p] = [rpm1,rpm2]
                            theta[p] = new_theta
                            break
                if(check!=1):
                    point_node.append(ndx)
                    c_nd.append(nd)
                    theta.append(new_theta)
                    cst.append(round(0.6+cst[x],3))
                    heuristic_node.append(round((0.6+cst[x]+heuristic(nd,goal)),2))
                    act.append([rpm1,rpm2])


        
        nd, new_theta = movement(ndx,rpm1,rpm1,theta[x])
        if(Map(nd[0],nd[1],resolution,radius+clearance)!=1):
            if not checkPoint(nd, vc_nd): 
                xl=range(0,len(c_nd))
                xl = xl[::-1]
                check = 0
                for p in xl:
                    if (nd == c_nd[p]):
                        check= 1
                        if (cst[p]>=(cst[x]+0.7)):
                            point_node[p]=ndx
                            cst[p]=round((cst[x]+0.7),3)
                            heuristic_node[p] = round((cst[x] + 0.7 + heuristic(nd,goal)),2)
                            act[p] = [rpm1,rpm1]
                            theta[p] = new_theta
                            break
                if(check!=1):
                    point_node.append(ndx)
                    c_nd.append(nd)
                    theta.append(new_theta)
                    cst.append(round(0.7+cst[x],3))
                    heuristic_node.append(round((0.7+cst[x]+heuristic(nd,goal)),2))
                    act.append([rpm1,rpm1])

        nd, new_theta = movement(ndx,rpm2,rpm2,theta[x])
        if(Map(nd[0],nd[1],resolution,radius+clearance)!=1):
            if not checkPoint(nd, vc_nd): 
                xl=range(0,len(c_nd))
                xl = xl[::-1]
                check = 0
                for p in xl:
                    if (nd == c_nd[p]):
                        check= 1
                        if (cst[p]>=(cst[x]+0.5)):
                            point_node[p]=ndx
                            cst[p]=round((cst[x]+0.5),3)
                            heuristic_node[p] = round((cst[x] + 0.5 + heuristic(nd,goal)),2)
                            act[p] = [rpm2,rpm2]
                            theta[p] = new_theta
                            break
                if(check!=1):
                    point_node.append(ndx)
                    c_nd.append(nd)
                    theta.append(new_theta)
                    cst.append(round(0.5+cst[x],3))
                    heuristic_node.append(round((0.5+cst[x]+heuristic(nd,goal)),2))
                    act.append([rpm2,rpm2])

        nd, new_theta = movement(ndx,rpm1,rpm1,theta[x])
        if(Map(nd[0],nd[1],resolution,radius+clearance)!=1):
            if not checkPoint(nd, vc_nd): 
                xl=range(0,len(c_nd))
                xl = xl[::-1]
                check = 0
                for p in xl:
                    if (nd == c_nd[p]):
                        check= 1
                        if (cst[p]>=(cst[x]+0.6)):
                            point_node[p]=ndx
                            cst[p]=round((cst[x]+0.6),3)
                            heuristic_node[p] = round((cst[x] + 0.6 + heuristic(nd,goal)),2)
                            act[p] = [rpm1,rpm1]
                            theta[p] = new_theta
                            break
                if(check!=1):
                    point_node.append(ndx)
                    c_nd.append(nd)
                    theta.append(new_theta)
                    cst.append(round(0.6+cst[x],3))
                    heuristic_node.append(round((0.6+cst[x]+heuristic(nd,goal)),2))
                    act.append([rpm1,rpm1])

        nd, new_theta = movement(ndx,rpm2,0,theta[x])
        if(Map(nd[0],nd[1],resolution,radius+clearance)!=1):
            if not checkPoint(nd, vc_nd): 
                xl=range(0,len(c_nd))
                xl = xl[::-1]
                check = 0
                for p in xl:
                    if (nd == c_nd[p]):
                        check= 1
                        if (cst[p]>=(cst[x]+0.8)):
                            point_node[p]=ndx
                            cst[p]=round((cst[x]+0.8),3)
                            heuristic_node[p] = round((cst[x] + 0.8 + heuristic(nd,goal)),2)
                            act[p] = [rpm2,0]
                            theta[p] = new_theta
                            break
                if(check!=1):
                    point_node.append(ndx)
                    c_nd.append(nd)
                    theta.append(new_theta)
                    cst.append(round(0.8+cst[x],3))
                    heuristic_node.append(round((0.8+cst[x]+heuristic(nd,goal)),2))
                    act.append([rpm2,0])

        nd, new_theta = movement(ndx,rpm1,0,theta[x])
        if(Map(nd[0],nd[1],resolution,radius+clearance)!=1):
            if not checkPoint(nd, vc_nd): 
                xl=range(0,len(c_nd))
                xl = xl[::-1]
                check = 0
                for p in xl:
                    if (nd == c_nd[p]):
                        check= 1
                        if (cst[p]>=(cst[x]+0.9)):
                            point_node[p]=ndx
                            cst[p]=round((cst[x]+0.9),3)
                            heuristic_node[p] = round((cst[x] + 0.9 + heuristic(nd,goal)),2)
                            act[p] = [rpm1,0]
                            theta[p] = new_theta
                            break
                if(check!=1):
                    point_node.append(ndx)
                    c_nd.append(nd)
                    theta.append(new_theta)
                    cst.append(round(0.9+cst[x],3))
                    heuristic_node.append(round((0.9+cst[x]+heuristic(nd,goal)),2))
                    act.append([rpm1,0])

        vp_nd.append(point_node.pop(x))
        vc_nd.append(c_nd.pop(x))
        v_cst.append(cst.pop(x))
        vh_nd.append(heuristic_node.pop(x))
        v_theta.append(theta.pop(x))
        v_act.append(act.pop(x))

        if (goal_achieved(vc_nd[-1],goal,radius+clearance)==1):
            flag=1
        if (flag!=1 and c_nd!=[]):
            x = heuristic_node.index(min(heuristic_node))
            ndx = c_nd[x][:]
        # To check the desired path
        if(flag == 0 and c_nd == []):
            sys.exit("Path not available")

    seq=[v_act[-1]]
    x=vp_nd[-1]
    i=1
    while(x!=start):
        if(vc_nd[-i]==x):
            seq.append(v_act[-i])
            x=vp_nd[-i]
        i=i+1  

    sequence=[]
    seq_theta = []
    # seq_theta.append(theta_i)
    # seq_theta.append(v_theta[-1])
    sequence.append(vc_nd[-1])
    sequence.append(vp_nd[-1])
    x = vp_nd[-1]
    i = 1
    while(x!=start):
        if (vc_nd[-i]==x):
            sequence.append(vp_nd[-i])
            # seq_theta.append(v_theta[-i])
            x  = vp_nd[-i]
        i = i+1

    seq_theta = v_theta[::-1]
    my_list = np.array(vc_nd)
    vc_nd = my_list*resolution
    my_list_1 = np.array(sequence)
    sequence = my_list_1*resolution
    end_time = time()
    # To calculate the solving time for the algorithm 
    print("Time taken {} seconds to solve".format(end_time-start_time))
    
    if(pygame_flag):
        print("Pygame Display Output")
        pygame.init()
        start_time = time()
        #-------------Displaying Output----------#


        # Size of the screen
        size = [100*scale,100*scale]
        screen = pygame.display.set_mode(size)

        # Display Window
        pygame.display.set_caption("Output")
        clock = pygame.time.Clock()
        done = False

        while not done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
                
            screen.fill(sky_blue)

            # To display obstacles
            circle_pts1,circle_pts2, circle_pts3, circle_pts4 = obstacle_disp_pts()
            pygame.draw.circle(screen, lime, (circle_pts1[0]*scale,circle_pts1[1]*scale), circle_pts1[2]*scale)
            pygame.draw.circle(screen, lime, (circle_pts2[0]*scale,circle_pts2[1]*scale), circle_pts2[2]*scale)
            pygame.draw.circle(screen, lime, (circle_pts3[0]*scale,circle_pts3[1]*scale), circle_pts3[2]*scale)
            pygame.draw.circle(screen, lime, (circle_pts4[0]*scale,circle_pts4[1]*scale), circle_pts4[2]*scale)
            pygame.draw.rect(screen,lime,[22.5*scale,12.5*scale,15*scale,15*scale])
            pygame.draw.rect(screen,lime,[2.5*scale,42.5*scale,15*scale,15*scale])
            pygame.draw.rect(screen,lime,[82.5*scale,42.5*scale,15*scale,15*scale])

            # pygame.display.update()
            pygame.display.flip()
            
            # To display explored nodes
            for i in vc_nd:
                pygame.event.get()
                pygame.time.wait(1)
                pygame.draw.rect(screen,white,[i[0]*scale,100*scale-i[1]*scale,resolution*scale,resolution*scale])
                pygame.display.flip()

            # # To display Optimal path
            for j in sequence[::-1]:
                pygame.time.wait(1)
                pygame.draw.rect(screen,red,[j[0]*scale,100*scale-j[1]*scale,resolution*scale,resolution*scale])
                pygame.display.flip()
            
            pygame.display.flip()
            pygame.time.wait(10000)
            done = True

    if(gazebo_flag):
        done = False
        print("Gazebo Simulation started")
        rospy.init_node('Motion_command',anonymous=True)
        vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        vel_msg = Twist()

        
        r = rospy.Rate(0.5)
        
        prev_x=start[0]
        prev_y=start[1]
        prev_theta=theta_i
        for node,theta in zip(sequence,seq_theta):
            
            v = math.sqrt((node[0] - prev_x)**2 + (node[1]- prev_y)**2)
            w = theta - prev_theta
            w=math.radians(w)
            vel_msg.linear.x = v/10
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = w*5
            t0 = rospy.Time.now().to_sec()
            while not rospy.is_shutdown():
                t1 = rospy.Time.now().to_sec()
                elapsed = t1 - t0
                if elapsed >= 1.0:
                    break
                vel_pub.publish(vel_msg)
                print("published linear velocity: ",vel_msg.linear.x)
                print("published angular velocity: ",vel_msg.angular.z)
                r.sleep()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            vel_pub.publish(vel_msg)
            prev_x = node[0]
            prev_y = node[1]
            prev_theta = theta
        done = True

    if(done):
        pygame.quit()




if __name__ == "__main__":
    Parser = argparse.ArgumentParser()
    Parser.add_argument('--start', type=float, nargs="+", default= [1,3], help='Initial position, Default: (1,3)')
    Parser.add_argument('--goal', type=float, nargs="+", default= [5,3], help='Goal position, Default: (5,3)')
    Parser.add_argument('--scale',type = int,default=5,help="Display scale")
    Parser.add_argument('--pygame',default='True',help="Flag to show Pygame Simulation")
    Parser.add_argument('--gazebo',default='True',help="Flag to show Gazebo simulation")
    Parser.add_argument('--clearance',type = float,default=1,help="Clearance to maintain from obstacle(in meter")
    Parser.add_argument('--resolution',type = float,default=1,help="resolution of the map")
    Parser.add_argument('--rpm1',type = int,default=10,help="rpm1")
    Parser.add_argument('--rpm2',type = int,default=20,help="rpm2")
    Parser.add_argument('--theta',type = int,default=0,help="theta")
    Args = Parser.parse_args()

    main(Args)






