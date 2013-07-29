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
from sensor_msgs.msg import *
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from hubo_robot_msgs.msg import *
import actionlib
import tf
from actionlib import *
from transformation_helper import *

class HuboMarkerTeleop:

    def __init__(self, marker_namespace, left_end_effector_mesh, right_end_effector_mesh, arm_action_prefix, gripper_action_prefix, controller_topic, trans_damping, rot_damping, enable_exec):
        self.enable_exec = enable_exec
        self.marker_namespace = marker_namespace
        self.left_end_effector_mesh = left_end_effector_mesh
        self.right_end_effector_mesh = right_end_effector_mesh
        self.trans_damping = trans_damping
        self.rot_damping = rot_damping
        self.controller_mode = "IDLE"
        self.last_buttons = [0,0]
        self.left_gripper_options = ["OPEN","CLOSE","EXEC"]
        self.right_gripper_options = ["OPEN","CLOSE","OPEN TRIGGER","CLOSE TRIGGER","EXEC"]
        self.left_gripper_handler = MenuHandler()
        self.right_gripper_handler = MenuHandler()
        # Populate menu options
        i = 1
        print "Left hand options:"
        for menu_option in self.left_gripper_options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.left_gripper_handler.insert(menu_option, callback=self.l_gripper_feedback_cb)
            i += 1
        i = 1
        print "Right hand options:"
        for menu_option in self.right_gripper_options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.right_gripper_handler.insert(menu_option, callback=self.r_gripper_feedback_cb)
            i += 1
        # Compute the start poses
        rospy.loginfo("Getting start poses of the end effectors...")
        self.listener = tf.TransformListener()
        self.base_frame = "/Body_Torso"
        t1 = self.listener.getLatestCommonTime(self.base_frame, "/Body_LWR")
        t2 = self.listener.getLatestCommonTime(self.base_frame, "/Body_RWR")
        [ltrans,lrot] = self.listener.lookupTransform(self.base_frame, "/Body_LWR", t1)
        [rtrans,rrot] = self.listener.lookupTransform(self.base_frame, "/Body_RWR", t2)
        start_left_pose = PoseFromTransform(TransformFromComponents(ltrans, lrot))
        start_right_pose = PoseFromTransform(TransformFromComponents(rtrans, rrot))
        # Set the default poses
        self.left_arm_pose = PoseStamped()
        self.left_arm_pose.header.frame_id = self.base_frame
        self.left_arm_pose.pose = start_left_pose
        self.right_arm_pose = PoseStamped()
        self.right_arm_pose.header.frame_id = self.base_frame
        self.right_arm_pose.pose = start_right_pose
        rospy.loginfo("...Set start end effector poses")
        # Set up the control clients for arms and grippers
        if (self.enable_exec):
            rospy.loginfo("Execution enabled - Setting up action clients...")
            self.left_arm_client = actionlib.SimpleActionClient(arm_action_prefix + "/left_arm/end_effector_pose_action", EndEffectorPoseAction)
            self.left_arm_client.wait_for_server()
            self.right_arm_client = actionlib.SimpleActionClient(arm_action_prefix + "/right_arm/end_effector_pose_action", EndEffectorPoseAction)
            self.right_arm_client.wait_for_server()
            rospy.loginfo("Arm clients loaded")
            self.left_gripper_client = actionlib.SimpleActionClient(gripper_action_prefix + "/left_gripper/gripper_action", HuboGripperCommandAction)
            self.left_gripper_client.wait_for_server()
            self.right_gripper_client = actionlib.SimpleActionClient(gripper_action_prefix + "/right_arm/gripper_action", HuboGripperCommandAction)
            self.right_gripper_client.wait_for_server()
            rospy.loginfo("Gripper clients loaded")
        else:
            rospy.logwarn("Execution disabled")
        # Setup the interactive marker server
        self.server = InteractiveMarkerServer(self.marker_namespace)
        # Subscribe to the spacenav
        if (controller_topic == ""):
            self.calibrated = True
            self.calibration_data = []
            self.calibrated_vals = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            rospy.logwarn("Ignoring Spacenav input as controller topic set to empty string")
        else:
            self.calibrated = False
            self.calibration_data = []
            self.calibrated_vals = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            rospy.loginfo("Enabling Spacenav input")
            self.spacenav_sub = rospy.Subscriber(controller_topic, Joy, self.spacenav_cb)
        # Calibrate the spacenav
        rospy.loginfo("Calibrating Spacenav - DO NOT TOUCH THE SPACENAV DURING CALIBRATION!")
        rate = rospy.Rate(20.0)
        while not self.calibrated:
            rate.sleep()
        rospy.loginfo("Spacenav calibrated")
        # Spin forever
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def attempt_calibration(self, new_axes):
        if (len(self.calibration_data) >= 50):
            elements = float(len(self.calibration_data))
            fxt = 0.0
            fyt = 0.0
            fzt = 0.0
            fxr = 0.0
            fyr = 0.0
            fzr = 0.0
            for axes in self.calibration_data:
                fxt += abs(axes[0])
                fyt += abs(axes[1])
                fzt += abs(axes[2])
                fxr += abs(axes[3])
                fyr += abs(axes[4])
                fzr += abs(axes[5])
            fxt = fxt / elements
            fyt = fyt / elements
            fzt = fzt / elements
            fxr = fxr / elements
            fyr = fyr / elements
            fzr = fzr / elements
            rospy.loginfo("Calibrated with floor values of " + str(fxt) + ", " + str(fyt) + ", " + str(fzt) + ", " + str(fxr) + ", " + str(fyr) + ", " + str(fzr))
            self.calibrated_vals = [fxt, fyt, fzt, fxr, fyr, fzr]
            self.calibrated = True
        else:
            self.calibration_data.append(new_axes)

    def spacenav_cb(self, msg):
        if not self.calibrated:
            self.attempt_calibration(msg.axes)
            return
        # Identify the primary axis, forget everything else
        self.controller_mode = self.mode_switch(msg.buttons, self.controller_mode)
        [principle_axis_value, principle_axis_num] = self.clean_spacenav(msg.axes)
        if (self.controller_mode == "BOTH"):
            self.left_arm_pose.pose = self.update_from_spacenav(principle_axis_value, principle_axis_num, self.left_arm_pose.pose, "LEFT")
            self.right_arm_pose.pose = self.update_from_spacenav(principle_axis_value, principle_axis_num, self.right_arm_pose.pose, "RIGHT")
        elif (self.controller_mode == "LEFT"):
            self.left_arm_pose.pose = self.update_from_spacenav(principle_axis_value, principle_axis_num, self.left_arm_pose.pose, "LEFT")
        elif (self.controller_mode == "RIGHT"):
            self.right_arm_pose.pose = self.update_from_spacenav(principle_axis_value, principle_axis_num, self.right_arm_pose.pose, "RIGHT")

    def mode_switch(self, buttons, current_mode):
        rising_edge = [False, False]
        if (self.last_buttons[0] == 0 and buttons[0] == 1):
            rising_edge[0] = True
        if (self.last_buttons[1] == 0 and buttons[1] == 1):
            rising_edge[1] = True
        self.last_buttons = buttons
        new_mode = current_mode
        if (current_mode == "IDLE"):
            if (rising_edge[0] and rising_edge[1]):
                new_mode = "BOTH"
            elif (rising_edge[0]):
                new_mode = "LEFT"
            elif (rising_edge[1]):
                new_mode = "RIGHT"
            else:
                new_mode = "IDLE"
        elif (current_mode == "BOTH"):
            if (rising_edge[0] and rising_edge[1]):
                new_mode = "IDLE"
            elif (rising_edge[0]):
                new_mode = "RIGHT"
            elif (rising_edge[1]):
                new_mode = "LEFT"
            else:
                new_mode = "BOTH"
        elif (current_mode == "LEFT"):
            if (rising_edge[0] and rising_edge[1]):
                new_mode = "RIGHT"
            elif (rising_edge[0]):
                new_mode = "IDLE"
            elif (rising_edge[1]):
                new_mode = "BOTH"
            else:
                new_mode = "LEFT"
        elif (current_mode == "RIGHT"):
            if (rising_edge[0] and rising_edge[1]):
                new_mode = "LEFT"
            elif (rising_edge[0]):
                new_mode = "BOTH"
            elif (rising_edge[1]):
                new_mode = "IDLE"
            else:
                new_mode = "RIGHT"
        if (new_mode != current_mode):
            rospy.loginfo("Switching to new mode: " + new_mode)
        return new_mode

    def clean_spacenav(self, axes):
        max_val = 0.0
        max_index = 0
        for index in range(len(axes)):
            if (abs(axes[index]) >= abs(max_val)):
                max_val = axes[index]
                max_index = index
        return [max_val, max_index]

    def update_from_spacenav(self, control_axis_value, control_axis_num, pose_to_update, arm):
        current_orientation = [pose_to_update.orientation.x, pose_to_update.orientation.y, pose_to_update.orientation.z, pose_to_update.orientation.w]
        # Filter out drift
        if (abs(control_axis_value) <= self.calibrated_vals[control_axis_num]):
            control_axis_value = 0.0
        else:
            if (control_axis_value < 0.0):
                control_axis_value = -(abs(control_axis_value) - self.calibrated_vals[control_axis_num])
            else:
                control_axis_value = abs(control_axis_value) - self.calibrated_vals[control_axis_num]
        # Update axes
        if (control_axis_num == 0):
            pose_to_update.position.x += (control_axis_value * self.trans_damping)
        elif (control_axis_num == 1):
            pose_to_update.position.y += (control_axis_value * self.trans_damping)
        elif (control_axis_num == 2):
            pose_to_update.position.z += (control_axis_value * self.trans_damping)
        elif (control_axis_num == 3):
            control_rotation = quaternion_about_axis((control_axis_value * self.rot_damping), (1,0,0))
            [x, y, z, w] = ComposeQuaternions(control_rotation, current_orientation)
            pose_to_update.orientation.x = x
            pose_to_update.orientation.y = y
            pose_to_update.orientation.z = z
            pose_to_update.orientation.w = w
        elif (control_axis_num == 4):
            control_rotation = quaternion_about_axis((control_axis_value * self.rot_damping), (0,1,0))
            [x, y, z, w] = ComposeQuaternions(control_rotation, current_orientation)
            pose_to_update.orientation.x = x
            pose_to_update.orientation.y = y
            pose_to_update.orientation.z = z
            pose_to_update.orientation.w = w
        elif (control_axis_num == 5):
            control_rotation = quaternion_about_axis((control_axis_value * self.rot_damping), (0,0,1))
            [x, y, z, w] = ComposeQuaternions(control_rotation, current_orientation)
            pose_to_update.orientation.x = x
            pose_to_update.orientation.y = y
            pose_to_update.orientation.z = z
            pose_to_update.orientation.w = w
        else:
            rospy.logerr("Invalid primary axis number")
        return self.clip_to_arm_bounds(pose_to_update, arm)

    def update(self):
        # This bugfix should have made it into Groovy
        #for marker_name in self.server.marker_contexts.keys():
        #    self.server.erase(marker_name)
        self.server.clear()
        left_gripper_name = "LEFT_HAND"
        right_gripper_name = "RIGHT_HAND"
        left_gripper_marker = self.make_gripper_imarker(self.left_arm_pose, left_gripper_name)
        right_gripper_marker = self.make_gripper_imarker(self.right_arm_pose, right_gripper_name)
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
            self.left_arm_pose.pose = self.clip_to_arm_bounds(feedback.pose, 'LEFT')
        elif (event_type == feedback.MENU_SELECT):
            if (self.gripper_options[feedback.menu_entry_id - 1] == "CLOSE"):
                close_goal = HuboGripperCommandGoal()
                close_goal.FingerPosition = 0.0
                close_goal.FingerEffort = -1.0
                if (self.enable_exec):
                    self.l_gripper_client.send_goal(close_goal)
                else:
                    rospy.logwarn("Execution is disabled, so hand will not close!")
                rospy.loginfo("Closing left hand")
            elif (self.gripper_options[feedback.menu_entry_id - 1] == "OPEN"):
                open_goal = HuboGripperCommandGoal()
                open_goal.FingerPosition = 1.0
                open_goal.FingerEffort = -1.0
                if (self.enable_exec):
                    self.l_gripper_client.send_goal(open_goal)
                else:
                    rospy.logwarn("Execution is disabled, so hand will not open!")
                rospy.loginfo("Opening left hand")
            elif (self.gripper_options[feedback.menu_entry_id - 1] == "EXEC"):
                arm_goal = EndEffectorPoseGoal()
                arm_goal.EndEffectorPose = self.left_arm_pose
                if (self.enable_exec):
                    self.l_arm_client.send_goal(arm_goal)
                else:
                    rospy.logwarn("Execution is disabled, so arm will not move!")
                rospy.loginfo("Executing left arm")
            else:
                rospy.logerr("Unrecognized menu option")
        else:
            rospy.logerr("Left gripper - unrecognized feedback type - " + str(feedback.event_type))

    def r_gripper_feedback_cb(self, feedback):
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            self.right_arm_pose.pose = self.clip_to_arm_bounds(feedback.pose, 'RIGHT')
        elif (event_type == feedback.MENU_SELECT):
            if (self.gripper_options[feedback.menu_entry_id - 1] == "CLOSE"):
                close_goal = HuboGripperCommandGoal()
                close_goal.FingerPosition = 0.0
                close_goal.FingerEffort = -1.0
                if (self.enable_exec):
                    self.r_gripper_client.send_goal(close_goal)
                else:
                    rospy.logwarn("Execution is disabled, so hand will not close!")
                rospy.loginfo("Closing right hand")
            elif (self.gripper_options[feedback.menu_entry_id - 1] == "OPEN"):
                open_goal = HuboGripperCommandGoal()
                open_goal.FingerPosition = 1.0
                open_goal.FingerEffort = -1.0
                if (self.enable_exec):
                    self.r_gripper_client.send_goal(open_goal)
                else:
                    rospy.logwarn("Execution is disabled, so hand will not open!")
                rospy.loginfo("Opening right hand")
            elif (self.gripper_options[feedback.menu_entry_id - 1] == "CLOSE TRIGGER"):
                close_goal = HuboGripperCommandGoal()
                close_goal.TriggerPosition = 0.0
                close_goal.TriggerEffort = -1.0
                if (self.enable_exec):
                    self.r_gripper_client.send_goal(close_goal)
                else:
                    rospy.logwarn("Execution is disabled, so trigger will not close!")
                rospy.loginfo("Closing right trigger")
            elif (self.gripper_options[feedback.menu_entry_id - 1] == "OPEN TRIGGER"):
                open_goal = HuboGripperCommandGoal()
                open_goal.TriggerPosition = 1.0
                open_goal.TriggerEffort = -1.0
                if (self.enable_exec):
                    self.r_gripper_client.send_goal(open_goal)
                else:
                    rospy.logwarn("Execution is disabled, so trigger will not open!")
                rospy.loginfo("Opening right trigger")
            elif (self.gripper_options[feedback.menu_entry_id - 1] == "EXEC"):
                arm_goal = EndEffectorPoseGoal()
                arm_goal.EndEffectorPose = self.right_arm_pose
                if (self.enable_exec):
                    self.r_arm_client.send_goal(arm_goal)
                else:
                    rospy.logwarn("Execution is disabled, so arm will not move!")
                rospy.loginfo("Executing right arm")
            else:
                rospy.logerr("Unrecognized menu option")
        else:
            rospy.logerr("Right gripper - unrecognized feedback type - " + str(feedback.event_type))

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
        new_control = InteractiveMarkerControl()
        new_control.interaction_mode = InteractiveMarkerControl.MENU
        new_control.always_visible = True
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        display_marker = self.make_arm_marker(marker_pose, marker_name)
        new_control.markers.append(display_marker)
        new_marker.controls.append(new_control)
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
            if (self.controller_mode == "BOTH" or self.controller_mode == "LEFT"):
                marker.color.r = 1.0
            else:
                marker.color.r = 0.0
        elif ("right" in marker_name.lower()):
            marker.mesh_resource = self.right_end_effector_mesh
            if (self.controller_mode == "BOTH" or self.controller_mode == "RIGHT"):
                marker.color.r = 1.0
            else:
                marker.color.r = 0.0
        marker.pose = marker_pose.pose
        #Set the scale of the marker -- 1x1x1 here means native scale
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        #Set the color -- be sure to set alpha to something non-zero!
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
    controller_topic = "spacenav/joy"
    trans_damping = 0.002
    rot_damping = 0.005
    enable_exec = rospy.get_param("~enable_exec", False)
    HuboMarkerTeleop(marker_namespace, left_end_effector_mesh, right_end_effector_mesh, arm_action_prefix, gripper_action_prefix, controller_topic, trans_damping, rot_damping, enable_exec)
