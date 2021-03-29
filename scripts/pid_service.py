#! /usr/bin/env python2

import rospy
import mavros
import numpy as np
from geometry_msgs.msg import TwistStamped
from pid_move.srv import goto_pid, goto_pidResponse
import tf 
import mavros_msgs.msg
import mavros.command
from mavros import setpoint 
from simple_pid import PID
from mavros import setpoint as SP

class PidNavigationController():

    def __init__(self):
        ''' Class that acts as a server for the goto_pid service '''

        # init node
        rospy.init_node('pid_navigation_service', anonymous=True)
        mavros.set_namespace('mavros')

        # Setup Subscribers
        ## mavros state
        state_sub = rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, self._state_callback)
        
        # Setup publishers
        # /mavros/setpoint_velocity/cmd_vel
        self.cmd_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
        # /mavros/local_position/pose
        local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self._local_position_callback)

        # setup services
        # /mavros/cmd/arming
        self.set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        # /mavros/set_mode
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)      
        
        # Initialize the service servers
        goto_pid_serv = rospy.Service('goto_pid_service', goto_pid, self.GotoLocPid)
        

        # Initialize variables
        self.local_pos = [0.0] * 4
        self.UAV_state = mavros_msgs.msg.State()

        # Setup rate
        self.rate = rospy.Rate(20)
        # rospy.sleep(1)
        # rospy.spin()

    def _local_position_callback(self, topic):
        # Position data
        self.local_pos[0] = topic.pose.position.x
        self.local_pos[1] = topic.pose.position.y
        self.local_pos[2] = topic.pose.position.z
        # Orientation data
        (r, p, y) = tf.transformations.euler_from_quaternion([topic.pose.orientation.x, topic.pose.orientation.y, topic.pose.orientation.z, topic.pose.orientation.w])
        self.local_pos[3] = y

    
    def _state_callback(self, topic):
        self.UAV_state.armed = topic.armed
        self.UAV_state.connected = topic.connected
        self.UAV_state.mode = topic.mode
        self.UAV_state.guided = topic.guided
    
    # def GotoLocPid(self, req):

    #     x = req.x
    #     y = req.y
    #     z = req.z
    #     Y = req.yaw
    #     print(x)
    #     rospy.loginfo("uav received local target")

    #     dist = self.GotoPid(x, y, z, Y)

    #     return goto_pidResponse(dist)


    def GotoPid(self, x, y, z, yaw):
        ''' Goto the position by pid '''
        rospy.loginfo('Going to the position by pid')
        rospy.loginfo('Wait to offboard')
        new_sp = TwistStamped()
        if self.UAV_state.mode == "OFFBOARD":
            self.cmd_vel_pub.publish(new_sp)
        # while self.UAV_state.mode != "OFFBOARD" :
        #     rospy.sleep(0.1)
        #     self.set_mode(0, 'OFFBOARD')
        #     # Publish something to activate the offboard mode
        #     self.cmd_vel_pub.publish(new_sp)
        
        # if not mavros.command.arming(True) :
        #     mavros.command.arming(True)
            
        xPID = PID(.4, 0.05, 0.1, output_limits=(-.5, 0.5), setpoint=x,sample_time=0.3)
        yPID = PID(.4, 0.05, 0.1, output_limits=(-.5, 0.5), setpoint=y,sample_time=0.3)
        zPID = PID(.2, 0.05, 0.1, output_limits=(-1.0, 1.0), setpoint=z,sample_time=0.3)
        yawPID = PID(.011, 0.005, 0.12, output_limits=(-.09, 0.09), setpoint=yaw, sample_time=0.3)
        
        while ((np.linalg.norm(np.array(self.local_pos[0:3]) - np.array([x, y, z])) > 0.2) or (abs(yaw - self.local_pos[3]) > 0.2)):
                
            new_sp = TwistStamped()
            new_sp.twist.linear.x = xPID(self.local_pos[0])
            new_sp.twist.linear.y = yPID(self.local_pos[1])
            new_sp.twist.linear.z = zPID(self.local_pos[2])
            new_sp.twist.angular.z = yawPID(self.local_pos[3])

            self.cmd_vel_pub.publish(new_sp)
            self.rate.sleep()
        return np.linalg.norm(np.array(self.local_pos[0:3]) - np.array([x, y, z]))
        
    def GotoLocPid (self, req):

        x = req.x
        y = req.y
        z = req.z
        Y = req.yaw
        rospy.loginfo("uav received local target")

        dist = self.GotoPid(x, y, z, Y)

        return goto_pidResponse(dist)


###################################################################################################
if __name__ == "__main__":
    pid = PidNavigationController()
    
    
