#!/usr/bin/env python2

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
from mavros import setpoint as SP
from pid_move.srv import target_local_pos, target_local_posResponse 
import numpy as np
import tf

class MoveToPos():
    """
        Class that is used to create a servers for the services that move 
        the drone to a specific position in the global and local coordinate system
    """
    def __init__(self):

        # Setup the nodes and the namespace
        # If the drone is a slave generate the master ID
        
        rospy.init_node("uav_pos_services", anonymous=True)        
        
        self.rate = rospy.Rate(20)
        mavros.set_namespace('mavros')

        # Initialize the parameters
        self.local_pos = [0.0] * 4
        self.UAV_state = mavros_msgs.msg.State()

        # setup subscribers
        # /mavros/state
        state_sub = rospy.Subscriber(mavros.get_topic('state'),
                                     mavros_msgs.msg.State, self._state_callback)

        # /mavros/local_position/pose
        local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
             SP.PoseStamped, self._local_position_callback)
    
        # setup publisher
        # /mavros/setpoint/position/local
        self.setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)

        # setup the services 
        # /mavros/cmd/arming
        self.set_arming = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        # /mavros/set_mode
        self.set_mode = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)      
        
        # Setup global and local service server
        sl = rospy.Service("target_local_pos", target_local_pos, self.GotoLocPos)
        
        self.setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
        )

        rospy.loginfo("The target services were successfully initiated")

        rospy.spin()

    def _state_callback(self, topic):
        self.UAV_state.armed = topic.armed
        self.UAV_state.connected = topic.connected
        self.UAV_state.mode = topic.mode
        self.UAV_state.guided = topic.guided
    
    def _local_position_callback(self, topic):
        # Position data
        self.local_pos[0] = topic.pose.position.x
        self.local_pos[1] = topic.pose.position.y
        self.local_pos[2] = topic.pose.position.z
        
        # Orientation data
        (r, p, y) = tf.transformations.euler_from_quaternion([topic.pose.orientation.x, topic.pose.orientation.y, topic.pose.orientation.z, topic.pose.orientation.w])
        self.local_pos[3] = y

    

    def GotoLocalPos(self, x, y, z, yaw):
        """ Enter the desired x, y, z coordinates and the yaw angle in the local system """
        
        dist = np.linalg.norm(np.array(self.local_pos[0:3]) - np.array([x, y, z]))
        rospy.loginfo("estimated distance : " + str(dist))

        self.set_mode(0, 'OFFBOARD')
        if (not self.UAV_state.armed):
            mavros.command.arming(True)

        last_request = rospy.Time.now()
        t0 = last_request

        # enter the main loop
        while ((np.linalg.norm(np.array(self.local_pos[0:3]) - np.array([x, y, z])) > 0.1) or
              (abs(yaw - self.local_pos[3]) > 0.01)):
            # print "Entered whiled loop"
            if (self.UAV_state.mode != "OFFBOARD" and
                    (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                self.set_mode(0, 'OFFBOARD')
                print("uav enabling offboard mode")
                last_request = rospy.Time.now()
            else:
                if (not self.UAV_state.armed and
                        (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                    if (mavros.command.arming(True)):
                        print("uav armed")
                    last_request = rospy.Time.now()

            # Position
            self.setpoint_msg.pose.position.x = x
            self.setpoint_msg.pose.position.y = y
            self.setpoint_msg.pose.position.z = z

            # Orientation
            (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(0, 0, yaw)
            self.setpoint_msg.pose.orientation.x = qx
            self.setpoint_msg.pose.orientation.y = qy
            self.setpoint_msg.pose.orientation.z = qz
            self.setpoint_msg.pose.orientation.w = qw

            #print("Height: %f" % self.setpoint_msg.pose.position.z)
            self.setpoint_local_pub.publish(self.setpoint_msg)
            #print(np.linalg.norm(np.array(self.local_pos) - np.array([x, y, z])))
            self.rate.sleep()

        return np.linalg.norm(np.array(self.local_pos[0:3]) - np.array([x, y, z])) 

    def GotoLocPos(self, req):

        x = req.goal_pos.position.x
        y = req.goal_pos.position.y
        z = req.goal_pos.position.z
        Y = req.goal_pos.yaw

        rospy.loginfo("uav received local target")

        dist = self.GotoLocalPos(x, y, z, Y)

        return target_local_posResponse(dist)


###################################################################################################
if __name__ == "__main__":
    MTP = MoveToPos()
