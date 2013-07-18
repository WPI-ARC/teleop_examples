#!/usr/bin/python

#############################################################
#                                                           #
#   Calder Phillips-Grafflin -- ARC Lab                     #
#                                                           #
#   Asynchronous cartesian end-effector teleoperation of    #
#   the DRCHubo robot using interactive markers in RVIZ.    #
#                                                           #
#############################################################

import rospy
import math
from copy import deepcopy

from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from hubo_robot_msgs.msg import *
import actionlib
from actionlib import *

class HuboMarkerTeleop:

    def __init__(self, marker_namespace, left_end_effector_mesh, right_end_effector_mesh, arm_action_prefix, gripper_action_prefix, controller_topic):
        self.marker_namespace = marker_namespace
        self.left_end_effector_mesh = left_end_effector_mesh
        self.right_end_effector_mesh = right_end_effector_mesh
        self.gripper_options = ["OPEN","CLOSE","EXEC"]
        self.left_gripper_handler = MenuHandler()
        self.right_gripper_handler = MenuHandler()
        # Populate menu options
        i = 1
        for menu_option in self.gripper_options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.left_gripper_handler.insert(menu_option, callback=self.l_gripper_feedback_cb)
            self.right_gripper_handler.insert(menu_option, callback=self.r_gripper_feedback_cb)
            i += 1
        # Set the default poses
        self.left_arm_pose = PoseStamped()
        self.left_arm_pose.header.frame_id = "/Body_TSY"
        self.left_arm_pose.pose.position.x = 0.20
        self.left_arm_pose.pose.position.y = 0.30
        self.left_arm_pose.pose.position.z = 0.0
        self.left_arm_pose.pose.orientation.x = 0.0
        self.left_arm_pose.pose.orientation.y = 0.0
        self.left_arm_pose.pose.orientation.z = 0.0
        self.left_arm_pose.pose.orientation.w = 1.0
        self.right_arm_pose = PoseStamped()
        self.right_arm_pose.header.frame_id = "/Body_TSY"
        self.right_arm_pose.pose.position.x = 0.20
        self.right_arm_pose.pose.position.y = -0.30
        self.right_arm_pose.pose.position.z = 0.0
        self.right_arm_pose.pose.orientation.x = 0.0
        self.right_arm_pose.pose.orientation.y = 0.0
        self.right_arm_pose.pose.orientation.z = 0.0
        self.right_arm_pose.pose.orientation.w = 1.0
        # Set up the control clients for arms and grippers
        
        #Setup the interactive marker server
        self.server = InteractiveMarkerServer(self.marker_namespace)
        self.update()
        rospy.spin()

    def update(self):
        # This bugfix should have made it into Groovy
        #for marker_name in self.server.marker_contexts.keys():
        #    self.server.erase(marker_name)
        self.server.clear()
        left_arm_name = "LEFT_ARM"
        left_gripper_name = "LEFT_HAND"
        right_arm_name = "RIGHT_ARM"
        right_gripper_name = "RIGHT_HAND"
        left_arm_marker = self.make_arm_imarker(self.left_arm_pose, left_arm_name)
        right_arm_marker = self.make_arm_imarker(self.right_arm_pose, right_arm_name)
        left_gripper_marker = self.make_gripper_imarker(self.left_arm_pose, left_gripper_name)
        right_gripper_marker = self.make_gripper_imarker(self.right_arm_pose, right_gripper_name)
        self.server.insert(left_arm_marker, self.l_arm_feedback_cb)
        self.server.insert(right_arm_marker, self.r_arm_feedback_cb)
        self.server.insert(left_gripper_marker, self.l_gripper_feedback_cb)
        self.server.insert(right_gripper_marker, self.r_gripper_feedback_cb)
        self.left_gripper_handler.apply(self.server, left_gripper_name)
        self.right_gripper_handler.apply(self.server, right_gripper_name)
        self.server.applyChanges()

    def l_gripper_feedback_cb(self, feedback):
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            pass
        elif (event_type == feedback.MENU_SELECT):
            if (self.gripper_options[feedback.menu_entry_id - 1] == "CLOSE"):
                #self.l_gripper_client.send_goal(self.close_goal)
                rospy.loginfo("Closing left hand")
            elif (self.gripper_options[feedback.menu_entry_id - 1] == "OPEN"):
                #self.l_gripper_client.send_goal(self.open_goal)
                rospy.loginfo("Opening left hand")
            elif (self.gripper_options[feedback.menu_entry_id - 1] == "EXEC"):
                #self.l_arm_client.send_goal(arm_goal)
                rospy.loginfo("Executing left arm")
            else:
                rospy.logerr("Unrecognized menu option")
        else:
            rospy.logerr("Left gripper - unrecognized feedback type - " + str(feedback.event_type))
        self.update()

    def r_gripper_feedback_cb(self, feedback):
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            pass
        elif (event_type == feedback.MENU_SELECT):
            if (self.gripper_options[feedback.menu_entry_id - 1] == "CLOSE"):
                #self.r_gripper_client.send_goal(self.close_goal)
                rospy.loginfo("Closing right hand")
            elif (self.gripper_options[feedback.menu_entry_id - 1] == "OPEN"):
                #self.r_gripper_client.send_goal(self.open_goal)
                rospy.loginfo("Opening right hand")
            elif (self.gripper_options[feedback.menu_entry_id - 1] == "EXEC"):
                #self.r_arm_client.send_goal(arm_goal)
                rospy.loginfo("Executing right arm")
            else:
                rospy.logerr("Unrecognized menu option")
        else:
            rospy.logerr("Right gripper - unrecognized feedback type - " + str(feedback.event_type))
        self.update()

    def l_arm_feedback_cb(self, feedback):
        event_type = feedback.event_type
        if (event_type == feedback.POSE_UPDATE):
            self.left_arm_pose.pose = self.clip_to_arm_bounds(feedback.pose, 'LEFT')
        elif (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        else:
            rospy.logerr("Left arm - unrecognized feedback type - " + str(feedback.event_type))
        self.update()

    def r_arm_feedback_cb(self, feedback):
        event_type = feedback.event_type
        if (event_type == feedback.POSE_UPDATE):
            self.right_arm_pose.pose = self.clip_to_arm_bounds(feedback.pose, 'RIGHT')
        elif (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        else:
            rospy.logerr("rightt arm - unrecognized feedback type - " + str(feedback.event_type))
        self.update()

    def clip_to_arm_bounds(self, arm_target, arm_code):
        max_up = 0.5
        max_down = -0.5
        max_left = 0.50
        max_right = -0.50
        max_backward = -0.5
        max_forward = 0.5
        if (arm_code == 'LEFT'):
            max_right = -0.40
        elif (arm_code == 'RIGHT'):
            max_left = 0.40
        if (arm_target.position.x > max_forward):
            arm_target.position.x = max_forward
        elif (arm_target.position.x < max_backward):
            arm_target.position.x = max_backward
        if (arm_target.position.y > max_left):
            arm_target.position.y = max_left
        elif (arm_target.position.y < max_right):
            arm_target.position.y = max_right
        if (arm_target.position.z > max_up):
            arm_target.position.z = max_up
        elif (arm_target.position.z < max_down):
            arm_target.position.z = max_down
        return arm_target

    def make_gripper_imarker(self, marker_pose, marker_name):
        control_pose = Pose()
        control_pose.position.x = marker_pose.pose.position.x
        control_pose.position.y = marker_pose.pose.position.y
        control_pose.position.z = marker_pose.pose.position.z + 1.0
        control_pose.orientation.w = 1.0
        control_pose_stamped = PoseStamped()
        control_pose_stamped.pose = control_pose
        control_pose_stamped.header.frame_id = marker_pose.header.frame_id
        new_marker = InteractiveMarker()
        new_marker.header.frame_id = marker_pose.header.frame_id
        new_marker.pose = control_pose
        new_marker.scale = 1.0
        new_marker.name = marker_name
        new_control = InteractiveMarkerControl()
        new_control.interaction_mode = InteractiveMarkerControl.MENU
        new_control.always_visible = True
        new_control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        display_marker = self.make_gripper_marker(control_pose_stamped, marker_name)
        new_control.markers.append(display_marker)
        new_marker.controls.append(new_control)
        return new_marker

    def make_arm_imarker(self, marker_pose, marker_name):
        new_marker = InteractiveMarker()
        new_marker.header.frame_id = marker_pose.header.frame_id
        new_marker.pose = marker_pose.pose
        new_marker.scale = 0.25
        new_marker.name = marker_name
        # Make the default control for the marker itself
        base_control = InteractiveMarkerControl()
        base_control.orientation_mode = InteractiveMarkerControl.FIXED
        base_control.always_visible = True
        display_marker = self.make_arm_marker(marker_pose, marker_name)
        base_control.markers.append(display_marker)
        new_marker.controls.append(base_control)
        # Make the x-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_x"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 1.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 0.0
        new_marker.controls.append(new_control)
        # Make the y-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_y"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 1.0
        new_control.orientation.z = 0.0
        new_marker.controls.append(new_control)
        # Make the z-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_z"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 1.0
        new_marker.controls.append(new_control)
        # Make the x-axis rotation control
        new_control = InteractiveMarkerControl()
        new_control.name = "rotate_x"
        new_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 1.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 0.0
        new_marker.controls.append(new_control)
        # Make the y-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "rotate_y"
        new_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 1.0
        new_control.orientation.z = 0.0
        new_marker.controls.append(new_control)
        # Make the z-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "rotate_z"
        new_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 1.0
        new_marker.controls.append(new_control)
        return new_marker

    def make_gripper_marker(self, marker_pose, marker_name):
        marker = Marker()
        marker.header.frame_id = marker_pose.header.frame_id
        marker.pose = marker_pose.pose
        #Give it a unique ID
        marker.ns = marker_name
        marker.id = 1
        #Give the marker a type
        marker.type = Marker.TEXT_VIEW_FACING
        if ("left" in marker_name.lower()):
            marker.text = "LEFT"
        elif ("right" in marker_name.lower()):
            marker.text = "RIGHT"
        marker.scale.z = 0.25
        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.a = 0.75
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def make_arm_marker(self, marker_pose, marker_name):
        marker = Marker()
        marker.header.frame_id = marker_pose.header.frame_id
        #Give it a unique ID
        marker.ns = marker_name
        marker.id = 1
        #Give the marker a type
        marker.type = Marker.MESH_RESOURCE
        if ("left" in marker_name.lower()):
            marker.mesh_resource = self.left_end_effector_mesh
        elif ("right" in marker_name.lower()):
            marker.mesh_resource = self.right_end_effector_mesh
        marker.pose = marker_pose.pose
        #Set the scale of the marker -- 1x1x1 here means native scale
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.a = 0.75
        marker.lifetime = rospy.Duration(0.5)
        return marker

if __name__ == '__main__':
    rospy.init_node('hubo_marker_teleop')
    marker_namespace = "hubo_marker_teleop"
    left_end_effector_mesh = "package://drchubo-v2/meshes/convhull_LWR_merged.stl"
    right_end_effector_mesh = "package://drchubo-v2/meshes/convhull_RWR_merged.stl"
    arm_action_prefix = "drchubo_fullbody_interface/"
    gripper_action_prefix = "drchubo_fullbody_interface/"
    controller_topic = "hydra_calib"
    HuboMarkerTeleop(marker_namespace, left_end_effector_mesh, right_end_effector_mesh, arm_action_prefix, gripper_action_prefix, controller_topic)
