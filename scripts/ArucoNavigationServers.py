#! /usr/bin/env python2

import rospy
import mavros
import numpy as np
from mavros_msgs.msg import PositionTarget as PT
from geometry_msgs.msg import TwistStamped
from pid_move.srv import goto_aruco, goto_arucoResponse
from tf import transformations as tr
import mavros_msgs.msg
import mavros_msgs.srv
import mavros.command
from mavros import setpoint as SP
from simple_pid import PID
import tf

class ArucoNavigationController():

    def __init__(self):
        ''' Class that acts as a server for the goto_aruco service and the land_aruco service '''

        # init node
        rospy.init_node('aruco_navigation_service')
        mavros.set_namespace('mavros')


        # Initialize variables
        self.pos = [0.0] * 4
        self.markerPos = [0.0] * 4
        self.UAV_state = mavros_msgs.msg.State()
        self.local_pos = [0.0] * 4

        # Setup Subscribers
        ## Marker pos
        aruco_pos = rospy.Subscriber('/aruco_marker_pos', PT, self._arucoCallback)
        ## mavros state
        state_sub = rospy.Subscriber(mavros.get_topic('state'),
                                 mavros_msgs.msg.State, self._state_callback)
        # /mavros/local_position/pose
        local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
             SP.PoseStamped, self._local_position_callback)

        # Setup publishers
        # /mavros/setpoint_velocity/cmd_vel
        self.cmd_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
        # setup services
        # /mavros/cmd/arming
        self.set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        # /mavros/set_mode
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)      
        
        # Initialize the service servers
        goto_aruco_serv = rospy.Service('goto_aruco', goto_aruco, self.GotoAruco)
        #land_aruco_serv = rospy.Service('land_aruco', land_aruco, self.LandAruco)

        # Setup rate
        self.rate = rospy.Rate(100)
        rospy.sleep(1)
        rospy.spin()

    def _local_position_callback(self, topic):
        # Position data
        self.local_pos[0] = topic.pose.position.x
        self.local_pos[1] = topic.pose.position.y
        self.local_pos[2] = topic.pose.position.z
        
        # Orientation data
        (r, p, y) = tf.transformations.euler_from_quaternion([topic.pose.orientation.x, topic.pose.orientation.y, topic.pose.orientation.z, topic.pose.orientation.w])
        self.local_pos[3] = y
    
    # def target_landing(self):
    #     setpose = np.zeros((3,1), dtype=np.float)
    #     setpose[0] = self.markerPos[0]
    #     setpose[1] = self.markerPos[1]
    #     setpose[2] = self.markerPos[2]
    #     # change form
    #     rotMat = tr.euler_matrix(0, 0, self.local_pos[3])
    #     rotMat = rotMat[0:3, 0:3]
    #     setpose = np.matmul(rotMat, setpose)
    #     self.pos[0] = setpose[0]
    #     self.pos[1] = setpose[1]
    #     self.pos[2] = setpose[2]

    def _arucoCallback(self, msg):
        ''' Callback for the aruco marker POS '''
        self.markerPos[0] = msg.position.x
        self.markerPos[1] = msg.position.y
        self.markerPos[2] = msg.position.z
        self.markerPos[3] = msg.yaw

        self.markerPos = np.array(self.markerPos)
    
    def _state_callback(self, topic):
        self.UAV_state.armed = topic.armed
        self.UAV_state.connected = topic.connected
        self.UAV_state.mode = topic.mode
        self.UAV_state.guided = topic.guided
    
    def GotoAruco(self, req):
        ''' Goto the aruco marker '''
        rospy.loginfo('Going to aruco marker')
        timeOut = req.timeOut
        
        rospy.loginfo('Wait to offboard')
        new_sp = TwistStamped()
        if self.UAV_state.mode == "OFFBOARD":
            self.cmd_vel_pub.publish(new_sp)
        new_sp = TwistStamped()
        # while self.UAV_state.mode != "OFFBOARD" :
        #     rospy.sleep(0.1)
        #     self.set_mode(0, 'OFFBOARD')
        #     # Publish something to activate the offboard mode
        #     self.cmd_vel_pub.publish(new_sp)
        
        # if not mavros.command.arming(True) :
        #     mavros.command.arming(True)
            
        ts = rospy.Time.now()

        xPID = PID(.2, 0.05, 0.1, output_limits=(-.5, 0.5), setpoint=0.0, sample_time=0.5)
        yPID = PID(.2, 0.05, 0.1, output_limits=(-.5, 0.5), setpoint=0.0, sample_time=0.5)
        zPID = PID(.2, 0.05, 0.1, output_limits=(-0.5, 0.5), setpoint=0.0, sample_time=0.5)
        yawPID = PID(.011, 0.005, 0.12, output_limits=(-1.0, 1.0), setpoint=0.0, sample_time=0.5)

        while (rospy.Time.now() - ts < rospy.Duration(timeOut)):
            
            new_sp = TwistStamped()
            if (self.local_pos[3] <0.1) and (self.local_pos[3]>-0.1):
                
                new_sp.twist.linear.x = xPID(-self.markerPos[0])
                new_sp.twist.linear.y = yPID(-self.markerPos[1])
                new_sp.twist.linear.z = zPID(0.0)
                new_sp.twist.angular.z = yawPID(0.0)
            else:
                setpose = np.zeros((3,1), dtype=np.float)
                setpose[0] = self.markerPos[0]
                setpose[1] = self.markerPos[1]
                setpose[2] = self.markerPos[2]
                # # change form
                rotMat = tr.euler_matrix(0, 0, self.local_pos[3])
                rotMat = rotMat[0:3, 0:3]
                setpose = np.matmul(rotMat, setpose)    
                new_sp.twist.linear.x = xPID(-setpose[0])
                new_sp.twist.linear.y = yPID(-setpose[1])
                new_sp.twist.linear.z = zPID(0.0)
                # new_sp.twist.angular.z = yawPID(self.markerPos[3])
                new_sp.twist.angular.z = yawPID(0.0)

            #print(np.linalg.norm(self.markerPos[0:3] - np.array([0.0, 0.0, -self.markerHeight])))

            self.cmd_vel_pub.publish(new_sp)

        return goto_arucoResponse(np.linalg.norm(self.markerPos[0:3]))


###################################################################################################
if __name__ == "__main__":
    ANC = ArucoNavigationController()
    