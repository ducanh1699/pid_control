#!/usr/bin/env python2

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandLong
from pid_move.srv import target_local_pos, target_local_posResponse, goto_pid, goto_pidResponse, goto_aruco, goto_arucoResponse
import sys
import signal
import numpy as np
import tf


class SimpleDrone():
    """
        Class that connects with a drone using mavros at the gazebo simulation
    """

    def __init__(self):
        rospy.init_node("client_node", anonymous=True)

        self.rate = rospy.Rate(20)
        mavros.set_namespace('mavros')

        signal.signal(signal.SIGINT, self.signal_handler)

        self.UAV_state = mavros_msgs.msg.State()

        # setup subscribers
        # /mavros/state
        state_sub = rospy.Subscriber(mavros.get_topic('state'),
                                     mavros_msgs.msg.State, self._state_callback)
        # /mavros/imu/data
        rospy.Subscriber('/mavros/imu/data', Imu, self.IMU_callback)

        # /mavros/setpoint_raw/target_local
        setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),
                                          mavros_msgs.msg.PositionTarget, self._setpoint_position_callback)


        # setup services
        # /mavros/cmd/arming
        self.set_arming = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        # mavros/cmd/command
        self.command_srv = rospy.ServiceProxy('mavros/cmd/command', mavros_msgs.srv.CommandLong)
        # /mavros/set_mode
        self.set_mode = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)      
        # goto_pos_services
        self.goto_loc_pos_serv = rospy.ServiceProxy("target_local_pos", target_local_pos)
        self.goto_pid_pos_serv = rospy.ServiceProxy("goto_pid_service", goto_pid)
        # aruco based services
        self.goto_aruco_serv = rospy.ServiceProxy('goto_aruco', goto_aruco)
        # self.land_aruco_serv = rospy.ServiceProxy('land_aruco', land_aruco)

        
        # wait for FCU connection
        while (not self.UAV_state.connected):
            self.rate.sleep()

        rospy.loginfo("Uav was successfully connected")

        self.InputHandler()

        rospy.spin()

    def signal_handler(self, signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)

    def IMU_callback(self, data):
        ''' IMU data subscriber that triggers the parachute in case of failure'''
        # Read the linear_accelaration on z-axis
        acc_z = data.linear_acceleration.z

        # if the acceleration is too low triger the parachute
        if acc_z < 6.0:
            self.command_srv(broadcast=False, command=185, param1=1.0)
    
    def _setpoint_position_callback(self, topic):
        pass

    def _state_callback(self, topic):
        self.UAV_state.armed = topic.armed
        self.UAV_state.connected = topic.connected
        self.UAV_state.mode = topic.mode
        self.UAV_state.guided = topic.guided
    

    def Hover(self):
        """
            Used to keep the altitude and position of the drone stable once activated
        """
        print("Hover", self.set_mode(0, 'AUTO.LOITER'))
    
    def GotoPos(self, pos):
        """
            Gets the drone to the defined pos,
            @pos_type = "local" for movement on the local coordinate system, pos[4] = [x, y, z, Yaw]
        """
        dist = -1.0
        # Deactivate the hover function if activated
        loc_pos = mavros_msgs.msg.PositionTarget(
                    header=mavros.setpoint.Header(
                        frame_id="att_pose",
                        stamp=rospy.Time.now()),
                    )
        # Position
        loc_pos.position.x = pos[0]
        loc_pos.position.y = pos[1]
        loc_pos.position.z = pos[2]

        # Orientation
        loc_pos.yaw = pos[3]

        try:
            dist = self.goto_loc_pos_serv(loc_pos)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            print("Service buisy")
            
        return dist
    
    def Land(self, pos = []):
        ''' Land the UAV at its current\given position '''
        if len(pos) > 0:
            self.GotoPos(pos)
        
        self.set_mode(0, 'AUTO.LAND')
        #mavros.command.arming(False)

    def Return(self):
        ''' Return to the home pos and land'''
        self.set_mode(0, 'AUTO.RTL')
        #mavros.command.arming(False)
    
    def Help(self):
        """ Print the instructions the swarm commander recognizes"""
        print("----- Swarm commander available inputs -----")
        print("exit -> to close the swarm commander")
        print("goto x y z yaw  -> uav will go to the  specified location location")
        print("gotopid x y z yaw  -> uav will go to the  specified location location by commanding velocity")
        print("goto aruco [timeout] -> the uav will got to the aruco marker, timeOut is optional")
        print("return -> uav will return to its home position and land")
        print("--------------------------------------------")
    
    def InputHandler(self):
        """ Used to handle the keyboard inputs from the terminal"""
        done = False
        while not done:
            inp = raw_input("Type your command > ")
            inp = inp.split()
            print(len(inp), (inp))
            
            if len(inp) == 1:
                if inp[0] == 'exit':
                    done = True
                elif inp[0] == 'return':
                    self.Return()
                elif inp[0] == 'land' and len(inp) == 1:
                    self.Land()
                else:
                    self.Help()
            elif len(inp) > 1:
                pos = []
                if (len(inp) == 5):
                    pos.append(float(inp[1]))
                    pos.append(float(inp[2]))
                    pos.append(float(inp[3]))
                    pos.append(float(inp[4]))

                if inp[0] == 'goto':
                    if len(inp) == 5:
                        dist = self.GotoPos(pos)
                        self.Hover()
                    elif inp[1] == 'aruco':
                        timeOut = 20
                        if len(inp) == 3:
                            timeOut = inp[2]
                            timeOut = float(timeOut)
                        dist = self.goto_aruco_serv(timeOut)
                        self.Hover()
                    else:             
                        self.Help()
                        continue   
                elif inp[0] == 'gotopid':
                    dist = self.goto_pid_pos_serv(pos[0], pos[1], pos[2], pos[3])
                    self.Hover()
                else:
                    self.Help()
            else:
                self.Help()

###################################################################################################        
if __name__ == "__main__":
    SD = SimpleDrone()
