#!/usr/bin/python

#############################################################
#                                                           #
#   Calder Phillips-Grafflin - WPI/ARC Lab                  #
#                                                           #
#   Asynchronous cartesian end-effector teleoperation of    #
#   the DRCHubo robot using interactive markers in RVIZ.    #
#                                                           #
#############################################################

import rospy
import roslib
import math
from copy import deepcopy
import threading

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
from openravepy import *

from hubo_marker_teleop import *

class HuboMarkerTeleop:

    GRIPPER = 'GRIPPER'
    PEG = 'PEG'

    def __init__(self, marker_namespace, left_end_effector_mesh, right_end_effector_mesh, left_peg_mesh, right_peg_mesh, joint_action_server, active_joints, maximum_movement, enable_exec=True, use_viewer=False):
        self.listener = tf.TransformListener()
        self.enable_exec = enable_exec
        self.use_viewer = use_viewer
        self.marker_namespace = marker_namespace
        self.left_end_effector_mesh = left_end_effector_mesh
        self.right_end_effector_mesh = right_end_effector_mesh
        self.left_peg_mesh = left_peg_mesh
        self.right_peg_mesh = right_peg_mesh
        self.active_joints = active_joints
        self.joint_lock = threading.Lock()
        self.latest_joint_state = None
        self.execution_scaling_time = 1.0
        self.preview_scaling_factor = 0.25
        self.left_arm_joint_targets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.right_arm_joint_targets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.left_gripper_joint_targets = [0.0, 0.0]
        self.right_gripper_joint_targets = [0.0, 0.0]
        self.left_gripper_options = ["OPEN","CLOSE","OPEN TRIGGER","CLOSE TRIGGER","EXEC","SWITCH TO PEG"]
        self.right_gripper_options = ["OPEN","CLOSE","OPEN TRIGGER","CLOSE TRIGGER","EXEC","SWITCH TO PEG"]
        self.left_peg_options = ["EXEC","SWITCH TO GRIPPER"]
        self.right_peg_options = ["EXEC","SWITCH TO GRIPPER"]
        self.left_mode = self.GRIPPER
        self.right_mode = self.GRIPPER
        self.left_dragging = False
        self.right_dragging = False
        self.left_gripper_handler = MenuHandler()
        self.right_gripper_handler = MenuHandler()
        self.left_peg_handler = MenuHandler()
        self.right_peg_handler = MenuHandler()
        # Start the joint state subscriber
        self.js_sub = rospy.Subscriber("joint_states", JointState, self.joint_state_cb)
        # Populate menu options
        # Grippers
        i = 1
        print "Left GRIPPER options:"
        for menu_option in self.left_gripper_options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.left_gripper_handler.insert(menu_option, callback=self.l_gripper_feedback_cb)
            i += 1
        i = 1
        print "Right GRIPPER options:"
        for menu_option in self.right_gripper_options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.right_gripper_handler.insert(menu_option, callback=self.r_gripper_feedback_cb)
            i += 1
        # Pegs
        i = 1
        print "Left PEG options:"
        for menu_option in self.left_peg_options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.left_peg_handler.insert(menu_option, callback=self.l_peg_feedback_cb)
            i += 1
        i = 1
        print "Right PEG options:"
        for menu_option in self.right_peg_options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.right_peg_handler.insert(menu_option, callback=self.r_peg_feedback_cb)
            i += 1
        # Set up OpenRAVE and IKFAST
        drchubo_path = roslib.packages.get_pkg_dir("drchubo_v3")
        self.env = Environment()
        if (self.use_viewer):
            self.env.SetViewer('qtcoin')
        self.robot = self.env.ReadRobotURI(drchubo_path + "/robots/drchubo_v3.robot.xml")
        self.env.Add(self.robot, True)
        # Load the IK solvers for the grippers
        self.robot.SetActiveManipulator("leftArm")
        self.left_arm_ikfast = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterizationType.Transform6D)
        self.left_arm_ikfast.load()
        self.robot.SetActiveManipulator("rightArm")
        self.right_arm_ikfast = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterizationType.Transform6D)
        self.right_arm_ikfast.load()
        # Load the IK solvers for the pegs
        self.robot.SetActiveManipulator("leftPeg")
        self.left_peg_ikfast = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterizationType.Transform6D)
        self.left_peg_ikfast.load()
        self.robot.SetActiveManipulator("rightPeg")
        self.right_peg_ikfast = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterizationType.Transform6D)
        self.right_peg_ikfast.load()
        # Get the torso link (in OpenRAVE)
        self.torso_link = None
        self.torso_yaw_link = None
        self.left_wrist_link = None
        self.right_wrist_link = None
        links = self.robot.GetLinks()
        for link in links:
            if (link.GetName() == "Body_Torso"):
                self.torso_link = link
            elif (link.GetName() == "Body_TSY"):
                self.torso_yaw_link = link
            elif (link.GetName() == "Body_LWR"):
                self.left_wrist_link = link
            elif (link.GetName() == "Body_RWR"):
                self.right_wrist_link = link
        if (self.torso_link == None):
            rospy.logfatal("Could not find torso link in OpenRAVE")
            rospy.signal_shutdown("Could not find torso link in OpenRAVE")
            exit()
        if (self.torso_yaw_link == None):
            rospy.logfatal("Could not find torso yaw link in OpenRAVE")
            rospy.signal_shutdown("Could not find torso yaw link in OpenRAVE")
            exit()
        if (self.left_wrist_link == None):
            rospy.logfatal("Could not find left wrist link in OpenRAVE")
            rospy.signal_shutdown("Could not find left wrist link in OpenRAVE")
            exit()
        if (self.right_wrist_link == None):
            rospy.logfatal("Could not find right wrist link in OpenRAVE")
            rospy.signal_shutdown("Could not find right wrist link in OpenRAVE")
            exit()
        # Add padding to the waist to be (slightly) safer
        Tpadding = self.torso_yaw_link.GetTransform()
        # Front
        self.waist1 = RaveCreateKinBody(self.env,'')
        self.waist1.SetName('waist1')
        self.waist1.InitFromBoxes(numpy.array([[0.11,0,-0.05,0.005,0.18,.10]]), True) # False for not visible
        self.waist1.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0.3,0.3,0.3)))
        self.waist1.SetTransform(Tpadding)
        # Left side
        self.waist2 = RaveCreateKinBody(self.env,'')
        self.waist2.SetName('waist2')
        self.waist2.InitFromBoxes(numpy.array([[0,0.17,-0.05,0.11,0.005,.10]]), True) # False for not visible
        self.waist2.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0.3,0.3,0.3)))
        self.waist2.SetTransform(Tpadding)
        # Right side
        self.waist3 = RaveCreateKinBody(self.env,'')
        self.waist3.SetName('waist3')
        self.waist3.InitFromBoxes(numpy.array([[0,-0.17,-0.05,0.11,0.005,.10]]), True) # False for not visible
        self.waist3.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0.3,0.3,0.3)))
        self.waist3.SetTransform(Tpadding)
        # Add to the environment
        self.env.Add(self.waist1,True)
        self.env.Add(self.waist2,True)
        self.env.Add(self.waist3,True)
        # Set the offset pose between gripper and peg
        self.left_peg_offset = Pose()
        self.left_peg_offset.position.x = -0.027
        self.left_peg_offset.position.y = 0.067
        self.right_peg_offset = Pose()
        self.right_peg_offset.position.x = -0.027
        self.right_peg_offset.position.y = -0.067
        # Compute the start poses
        rospy.loginfo("Getting start poses of the end effectors...")
        rospy.sleep(0.5)
        self.base_frame = "/Body_Torso"
        [ltrans,lrot] = self.listener.lookupTransform(self.base_frame, "/Body_LWR", rospy.Time())
        [rtrans,rrot] = self.listener.lookupTransform(self.base_frame, "/Body_RWR", rospy.Time())
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
        # Set up the control clients for arms
        rospy.loginfo("Setting up actionlib clients to control arms and head")
        self.joint_traj_client = actionlib.SimpleActionClient(joint_action_server, JointTrajectoryAction)
        if (enable_exec):
            #self.joint_traj_client.wait_for_server()
            rospy.loginfo("Joint control client loaded")
        else:
            rospy.logwarn("Execution disabled")
        # Setup the interactive marker server
        self.server = InteractiveMarkerServer(self.marker_namespace)
        rate = rospy.Rate(20.0)
        # Make sure we're getting joint states from the robot
        rospy.loginfo("Waiting to receive state from the robot...")
        if (self.enable_exec):
            while (self.latest_joint_state == None):
                rate.sleep()
        rospy.loginfo("HuboMarkerTeleop loaded")
        # Spin forever
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def update(self):
        self.server.clear()
        left_gripper_name = "LEFT_HAND"
        right_gripper_name = "RIGHT_HAND"
        left_gripper_marker = self.make_gripper_imarker(self.left_arm_pose, left_gripper_name)
        right_gripper_marker = self.make_gripper_imarker(self.right_arm_pose, right_gripper_name)
        if (self.left_mode == self.GRIPPER):
            self.server.insert(left_gripper_marker, self.l_gripper_feedback_cb)
            self.left_gripper_handler.apply(self.server, left_gripper_name)
        elif (self.left_mode == self.PEG):
            self.server.insert(left_gripper_marker, self.l_peg_feedback_cb)
            self.left_peg_handler.apply(self.server, left_gripper_name)
        else:
            rospy.logerr("Invalid operating mode for left arm")
        if (self.right_mode == self.GRIPPER):
            self.server.insert(right_gripper_marker, self.r_gripper_feedback_cb)
            self.right_gripper_handler.apply(self.server, right_gripper_name)
        elif (self.right_mode == self.PEG):
            self.server.insert(right_gripper_marker, self.r_peg_feedback_cb)
            self.right_peg_handler.apply(self.server, right_gripper_name)
        else:
            rospy.logerr("Invalid operating mode for right arm")
        self.server.applyChanges()

    def l_gripper_feedback_cb(self, feedback):
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            if (self.left_dragging):
                rospy.loginfo("Mouse UP - finding best IK solution")
                best_ik_solution = self.compute_left_arm(self.left_arm_pose.pose, best=True)
                if (best_ik_solution != None):
                    self.left_arm_joint_targets = best_ik_solution
                self.left_dragging = False
            else:
                pass
        elif (event_type == feedback.POSE_UPDATE):
            self.left_dragging = True
            ik_solution = self.compute_left_arm(feedback.pose)
            if (ik_solution != None):
                self.left_arm_pose.pose = feedback.pose
                self.left_arm_joint_targets = ik_solution
        elif (event_type == feedback.MENU_SELECT):
            if (self.left_gripper_options[feedback.menu_entry_id - 1] == "CLOSE"):
                self.execute_left_gripper(gripper_target="CLOSE", trigger_target=None)
            elif (self.left_gripper_options[feedback.menu_entry_id - 1] == "OPEN"):
                self.execute_left_gripper(gripper_target="OPEN", trigger_target=None)
            elif (self.left_gripper_options[feedback.menu_entry_id - 1] == "CLOSE TRIGGER"):
                self.execute_left_gripper(gripper_target=None, trigger_target="CLOSE")
            elif (self.left_gripper_options[feedback.menu_entry_id - 1] == "OPEN TRIGGER"):
                self.execute_left_gripper(gripper_target=None, trigger_target="OPEN")
            elif (self.left_gripper_options[feedback.menu_entry_id - 1] == "EXEC"):
                self.execute_robot()
            elif (self.left_gripper_options[feedback.menu_entry_id - 1] == "SWITCH TO PEG"):
                self.left_mode = self.PEG
                self.stow_left()
            else:
                rospy.logerr("Unrecognized menu option")
        else:
            rospy.logerr("Left gripper - unrecognized feedback type - " + str(feedback.event_type))

    def l_peg_feedback_cb(self, feedback):
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            if (self.left_dragging):
                rospy.loginfo("Mouse UP - finding best IK solution")
                best_ik_solution = self.compute_left_peg(self.left_arm_pose.pose, best=True)
                if (best_ik_solution != None):
                    self.left_arm_joint_targets = best_ik_solution
                self.left_dragging = False
            else:
                pass
        elif (event_type == feedback.POSE_UPDATE):
            self.left_dragging = True
            ik_solution = self.compute_left_peg(feedback.pose)
            if (ik_solution != None):
                self.left_arm_pose.pose = feedback.pose
                self.left_arm_joint_targets = ik_solution
        elif (event_type == feedback.MENU_SELECT):
            if (self.left_peg_options[feedback.menu_entry_id - 1] == "EXEC"):
                self.execute_robot()
            elif (self.left_peg_options[feedback.menu_entry_id - 1] == "SWITCH TO GRIPPER"):
                self.left_mode = self.GRIPPER
                self.unstow_left()
            else:
                rospy.logerr("Unrecognized menu option")
        else:
            rospy.logerr("Left peg - unrecognized feedback type - " + str(feedback.event_type))

    def r_gripper_feedback_cb(self, feedback):
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            if (self.right_dragging):
                rospy.loginfo("Mouse UP - finding best IK solution")
                best_ik_solution = self.compute_right_arm(self.right_arm_pose.pose, best=True)
                if (best_ik_solution != None):
                    self.right_arm_joint_targets = best_ik_solution
                self.right_dragging = False
            else:
                pass
        elif (event_type == feedback.POSE_UPDATE):
            self.right_dragging = True
            ik_solution = self.compute_right_arm(feedback.pose)
            if (ik_solution != None):
                self.right_arm_pose.pose = feedback.pose
                self.right_arm_joint_targets = ik_solution
        elif (event_type == feedback.MENU_SELECT):
            if (self.right_gripper_options[feedback.menu_entry_id - 1] == "CLOSE"):
                self.execute_right_gripper(gripper_target="CLOSE", trigger_target=None)
            elif (self.right_gripper_options[feedback.menu_entry_id - 1] == "OPEN"):
                self.execute_right_gripper(gripper_target="OPEN", trigger_target=None)
            elif (self.right_gripper_options[feedback.menu_entry_id - 1] == "CLOSE TRIGGER"):
                self.execute_right_gripper(gripper_target=None, trigger_target="CLOSE")
            elif (self.right_gripper_options[feedback.menu_entry_id - 1] == "OPEN TRIGGER"):
                self.execute_right_gripper(gripper_target=None, trigger_target="OPEN")
            elif (self.right_gripper_options[feedback.menu_entry_id - 1] == "EXEC"):
                self.execute_robot()
            elif (self.right_gripper_options[feedback.menu_entry_id - 1] == "SWITCH TO PEG"):
                self.right_mode = self.PEG
                self.stow_right()
            else:
                rospy.logerr("Unrecognized menu option")
        else:
            rospy.logerr("Right gripper - unrecognized feedback type - " + str(feedback.event_type))

    def r_peg_feedback_cb(self, feedback):
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            if (self.right_dragging):
                rospy.loginfo("Mouse UP - finding best IK solution")
                best_ik_solution = self.compute_right_peg(self.right_arm_pose.pose, best=True)
                if (best_ik_solution != None):
                    self.right_arm_joint_targets = best_ik_solution
                self.right_dragging = False
            else:
                pass
        elif (event_type == feedback.POSE_UPDATE):
            self.right_dragging = True
            ik_solution = self.compute_right_peg(feedback.pose)
            if (ik_solution != None):
                self.right_arm_pose.pose = feedback.pose
                self.right_arm_joint_targets = ik_solution
        elif (event_type == feedback.MENU_SELECT):
            if (self.right_peg_options[feedback.menu_entry_id - 1] == "EXEC"):
                self.execute_robot()
            elif (self.right_peg_options[feedback.menu_entry_id - 1] == "SWITCH TO GRIPPER"):
                self.right_mode = self.GRIPPER
                self.unstow_right()
            else:
                rospy.logerr("Unrecognized menu option")
        else:
            rospy.logerr("Right peg - unrecognized feedback type - " + str(feedback.event_type))

    def joint_state_cb(self, msg):
        with self.joint_lock:
            self.latest_joint_state = msg

    def execute_robot(self, wait=False, execution_time=0.0):
        # Build trajectory
        current_joint_state = None
        with self.joint_lock:
            current_joint_state = deepcopy(self.latest_joint_state)
        #self.active_joints = current_joint_state.name
        target_joint_state = self.build_target_joint_state(current_joint_state, self.left_arm_joint_targets, self.right_arm_joint_targets, self.left_gripper_joint_targets, self.right_gripper_joint_targets)
        trajectory = self.build_full_trajectory(current_joint_state.name, current_joint_state.position, target_joint_state, execution_time)
        rospy.loginfo("Checking trajectory with " + str(len(trajectory.points)) + " states for self-collision")
        # Preview in OpenRAVE
        if (self.use_viewer):
            for index in range(len(trajectory.points)):
                point = trajectory.points[index]
                positions = point.positions
                left_arm = self.extract_left_arm(trajectory.joint_names, positions)
                right_arm = self.extract_right_arm(trajectory.joint_names, positions)
                # Set LEFT arm
                self.robot.SetDOFValues(left_arm, self.robot.GetManipulators()[0].GetArmIndices())
                # Set RIGHT arm
                self.robot.SetDOFValues(right_arm, self.robot.GetManipulators()[1].GetArmIndices())
                # Check self-collision
                if (self.robot.CheckSelfCollision()):
                    #rospy.logerr("Encountered self-collision - cutting trajectory short at the last safe state!")
                    #trajectory.points = trajectory.points[:index]
                    rospy.logerr("Encountered self-collision - aborting trajectory!")
                    trajectory.points = trajectory.points[:1]
                    # Set the robot in the last safe configuration
                    safe_point = trajectory.points[-1]
                    positions = safe_point.positions
                    left_arm = self.extract_left_arm(trajectory.joint_names, positions)
                    right_arm = self.extract_right_arm(trajectory.joint_names, positions)
                    # Set LEFT arm
                    self.robot.SetDOFValues(left_arm, self.robot.GetManipulators()[0].GetArmIndices())
                    # Set RIGHT arm
                    self.robot.SetDOFValues(right_arm, self.robot.GetManipulators()[1].GetArmIndices())
                    # Get the transforms for each end effector
                    Ttorso = self.torso_link.GetTransform()
                    Tleftwrist = self.left_wrist_link.GetTransform()
                    Trightwrist = self.right_wrist_link.GetTransform()
                    Ttorso_left_wrist = numpy.dot(numpy.linalg.inv(Ttorso), Tleftwrist)
                    Ttorso_right_wrist = numpy.dot(numpy.linalg.inv(Ttorso), Trightwrist)
                    left_wrist_pose = PoseFromMatrix(Ttorso_left_wrist)
                    right_wrist_pose = PoseFromMatrix(Ttorso_right_wrist)
                    # Convert the orientation of the OpenRAVE pose to ROS
                    left_rotation = quaternion_about_axis(-math.pi / 2.0, (0,0,1))
                    left_rotation = [0.0,0.0,0.0,0.0]
                    left_adjustment = PoseFromTransform(TransformFromComponents([0.0,0.0,0.0], left_rotation))
                    right_rotation = quaternion_about_axis(math.pi / 2.0, (0,0,1))
                    right_rotation = [0.0,0.0,0.0,0.0]
                    right_adjustment = PoseFromTransform(TransformFromComponents([0.0,0.0,0.0], right_rotation))
                    real_left_wrist_pose = ComposePoses(left_wrist_pose, left_adjustment)
                    real_right_wrist_pose = ComposePoses(right_wrist_pose, right_adjustment)
                    # Update the stored poses to the safe ones
                    self.left_arm_pose.pose = real_left_wrist_pose
                    self.left_arm_joint_targets = left_arm
                    self.right_arm_pose.pose = real_right_wrist_pose
                    self.right_arm_joint_targets = right_arm
                    # Wait for OpenRAVE to catch up
                    self.robot.WaitForController(0)
                    self.robot.GetController().Reset(0)
                    rospy.loginfo("Set markers to last safe state")
                    break
                # Wait for OpenRAVE to catch up
                self.robot.WaitForController(0)
                self.robot.GetController().Reset(0)
                # Wait a bit
                rospy.sleep(self.preview_scaling_factor * (1.0 / 30.0))
        rospy.loginfo("Safe trajectory with " + str(len(trajectory.points)) + " states")
        exec_time = trajectory.points[-1].time_from_start.to_sec()
        rospy.loginfo("Execution time: " + str(exec_time) + " seconds")
        # Ask the user to confirm execution
        confirm_exec = True
        confirm = raw_input("Confirm execution - Y/n: ")
        # Check value
        if (confirm.lower() == "n" or confirm.lower() == "no"):
            confirm_exec = False
        if (not confirm_exec):
            rospy.logwarn("Aborting execution")
            # Set the robot in the last safe configuration
            safe_point = trajectory.points[0]
            positions = safe_point.positions
            left_arm = self.extract_left_arm(trajectory.joint_names, positions)
            right_arm = self.extract_right_arm(trajectory.joint_names, positions)
            # Set LEFT arm
            self.robot.SetDOFValues(left_arm, self.robot.GetManipulators()[0].GetArmIndices())
            # Set RIGHT arm
            self.robot.SetDOFValues(right_arm, self.robot.GetManipulators()[1].GetArmIndices())
            # Get the transforms for each end effector
            Ttorso = self.torso_link.GetTransform()
            Tleftwrist = self.left_wrist_link.GetTransform()
            Trightwrist = self.right_wrist_link.GetTransform()
            Ttorso_left_wrist = numpy.dot(numpy.linalg.inv(Ttorso), Tleftwrist)
            Ttorso_right_wrist = numpy.dot(numpy.linalg.inv(Ttorso), Trightwrist)
            left_wrist_pose = PoseFromMatrix(Ttorso_left_wrist)
            right_wrist_pose = PoseFromMatrix(Ttorso_right_wrist)
            # Convert the orientation of the OpenRAVE pose to ROS
            #left_rotation = quaternion_about_axis(-math.pi / 2.0, (0,0,1))
            left_rotation = [0.0,0.0,0.0,0.0]
            left_adjustment = PoseFromTransform(TransformFromComponents([0.0,0.0,0.0], left_rotation))
            #right_rotation = quaternion_about_axis(math.pi / 2.0, (0,0,1))
            right_rotation = [0.0,0.0,0.0,0.0]
            right_adjustment = PoseFromTransform(TransformFromComponents([0.0,0.0,0.0], right_rotation))
            real_left_wrist_pose = ComposePoses(left_wrist_pose, left_adjustment)
            real_right_wrist_pose = ComposePoses(right_wrist_pose, right_adjustment)
            # Update the stored poses to the safe ones
            self.left_arm_pose.pose = real_left_wrist_pose
            self.left_arm_joint_targets = left_arm
            self.right_arm_pose.pose = real_right_wrist_pose
            self.right_arm_joint_targets = right_arm
            # Wait for OpenRAVE to catch up
            self.robot.WaitForController(0)
            self.robot.GetController().Reset(0)
            rospy.loginfo("Reset markers to initial state")
            return
        # Build the goal to execute
        if (self.enable_exec):
            trajectory_goal = JointTrajectoryGoal()
            trajectory_goal.trajectory = trajectory
            self.joint_traj_client.send_goal(trajectory_goal)
            if (wait):
                self.joint_traj_client.wait_for_result(rospy.Duration(exec_time + 2.0))
            rospy.loginfo("Execution complete")
        else:
            rospy.logwarn("Execution disabled")

    def stow_left(self):
        rospy.loginfo("Switching LEFT arm to PEG mode")
        # Stow the wrist back to zero
        self.left_arm_joint_targets[6] = -math.pi / 2.0
        self.execute_robot(wait=True)
        # Close the fingers & trigger
        self.execute_left_gripper(gripper_target="CLOSE", trigger_target="CLOSE")
        #'''
        # Point the peg outwards (flip the wrist back)
        self.left_arm_joint_targets[5] = -math.pi
        # Set LEFT arm
        self.robot.SetDOFValues(self.left_arm_joint_targets, self.robot.GetManipulators()[0].GetArmIndices())
        # Get the transform for the left end effector
        Ttorso = self.torso_link.GetTransform()
        Tleftwrist = self.left_wrist_link.GetTransform()
        Ttorso_left_wrist = numpy.dot(numpy.linalg.inv(Ttorso), Tleftwrist)
        left_wrist_pose = PoseFromMatrix(Ttorso_left_wrist)
        #left_rotation = quaternion_about_axis(-math.pi / 2.0, (0,0,1))
        left_rotation = [0.0,0.0,0.0,0.0]
        left_adjustment = PoseFromTransform(TransformFromComponents([0.0,0.0,0.0], left_rotation))
        real_left_wrist_pose = ComposePoses(left_wrist_pose, left_adjustment)
        self.left_arm_pose.pose = real_left_wrist_pose
        self.execute_robot(wait=True)
        #'''

    def unstow_left(self):
        rospy.loginfo("Switching LEFT arm to GRIPPER mode")
        #'''
        # Point the peg inwards (flip the wrist back)
        self.left_arm_joint_targets[5] = 0.0
        # Set LEFT arm
        self.robot.SetDOFValues(self.left_arm_joint_targets, self.robot.GetManipulators()[0].GetArmIndices())
        # Get the transform for the left end effector
        Ttorso = self.torso_link.GetTransform()
        Tleftwrist = self.left_wrist_link.GetTransform()
        Ttorso_left_wrist = numpy.dot(numpy.linalg.inv(Ttorso), Tleftwrist)
        left_wrist_pose = PoseFromMatrix(Ttorso_left_wrist)
        #left_rotation = quaternion_about_axis(-math.pi / 2.0, (0,0,1))
        left_rotation = [0.0,0.0,0.0,0.0]
        left_adjustment = PoseFromTransform(TransformFromComponents([0.0,0.0,0.0], left_rotation))
        real_left_wrist_pose = ComposePoses(left_wrist_pose, left_adjustment)
        self.left_arm_pose.pose = real_left_wrist_pose
        self.execute_robot(wait=True)
        #'''

    def stow_right(self):
        rospy.loginfo("Switching RIGHT arm to PEG mode")
        # Stow the wrist back to zero
        self.right_arm_joint_targets[6] = math.pi / 2.0
        self.execute_robot(wait=True)
        # Close the fingers & trigger
        self.execute_right_gripper(gripper_target="CLOSE", trigger_target="CLOSE")
        #'''
        # Point the peg outwards (flip the wrist back)
        self.right_arm_joint_targets[5] = -math.pi
        # Set RIGHT arm
        self.robot.SetDOFValues(self.right_arm_joint_targets, self.robot.GetManipulators()[1].GetArmIndices())
        # Get the transform for the right end effector
        Ttorso = self.torso_link.GetTransform()
        Trightwrist = self.right_wrist_link.GetTransform()
        Ttorso_right_wrist = numpy.dot(numpy.linalg.inv(Ttorso), Trightwrist)
        right_wrist_pose = PoseFromMatrix(Ttorso_right_wrist)
        #right_rotation = quaternion_about_axis(math.pi / 2.0, (0,0,1))
        right_rotation = [0.0,0.0,0.0,0.0]
        right_adjustment = PoseFromTransform(TransformFromComponents([0.0,0.0,0.0], right_rotation))
        real_right_wrist_pose = ComposePoses(right_wrist_pose, right_adjustment)
        self.right_arm_pose.pose = real_right_wrist_pose
        self.execute_robot(wait=True)
        #'''

    def unstow_right(self):
        rospy.loginfo("Switching RIGHT arm to GRIPPER mode")
        #'''
        # Point the peg inwards (flip the wrist back)
        self.right_arm_joint_targets[5] = 0.0
        # Set RIGHT arm
        self.robot.SetDOFValues(self.right_arm_joint_targets, self.robot.GetManipulators()[1].GetArmIndices())
        # Get the transform for the right end effector
        Ttorso = self.torso_link.GetTransform()
        Trightwrist = self.right_wrist_link.GetTransform()
        Ttorso_right_wrist = numpy.dot(numpy.linalg.inv(Ttorso), Trightwrist)
        right_wrist_pose = PoseFromMatrix(Ttorso_right_wrist)
        #right_rotation = quaternion_about_axis(math.pi / 2.0, (0,0,1))
        right_rotation = [0.0,0.0,0.0,0.0]
        right_adjustment = PoseFromTransform(TransformFromComponents([0.0,0.0,0.0], right_rotation))
        real_right_wrist_pose = ComposePoses(right_wrist_pose, right_adjustment)
        self.right_arm_pose.pose = real_right_wrist_pose
        self.execute_robot(wait=True)
        #'''

    def execute_left_gripper(self, gripper_target=None, trigger_target=None):
        if (gripper_target != None):
            self.left_gripper_joint_targets[0] = self.get_gripper_target(gripper_target)
        elif (trigger_target != None):
            self.left_gripper_joint_targets[1] = self.get_gripper_target(trigger_target)
        else:
            self.left_gripper_joint_targets = [0.0, 0.0]
            return
        # We open and close grippers for 6 seconds
        self.execute_robot(wait=True, execution_time=6.0)
        # Clear the targets
        self.left_gripper_joint_targets = [0.0, 0.0]

    def execute_right_gripper(self, gripper_target=None, trigger_target=None):
        if (gripper_target != None):
            self.right_gripper_joint_targets[0] = self.get_gripper_target(gripper_target)
        elif (trigger_target != None):
            self.right_gripper_joint_targets[1] = self.get_gripper_target(trigger_target)
        else:
            self.right_gripper_joint_targets = [0.0, 0.0]
            return
        # We open and close grippers for 6 seconds
        self.execute_robot(wait=True, execution_time=6.0)
        # Clear the targets
        self.right_gripper_joint_targets = [0.0, 0.0]

    def get_gripper_target(self, command_value):
        if (command_value.lower() == "close"):
            return 1.0
        elif (command_value.lower() == "open"):
            return -1.0
        else:
            return 0.0

    def extract_left_arm(self, joint_names, positions):
        values = []
        values.append(positions[joint_names.index("LSP")])
        values.append(positions[joint_names.index("LSR")])
        values.append(positions[joint_names.index("LSY")])
        values.append(positions[joint_names.index("LEP")])
        values.append(positions[joint_names.index("LWY")])
        values.append(positions[joint_names.index("LWP")])
        values.append(positions[joint_names.index("LWR")])
        return values

    def extract_right_arm(self, joint_names, positions):
        values = []
        values.append(positions[joint_names.index("RSP")])
        values.append(positions[joint_names.index("RSR")])
        values.append(positions[joint_names.index("RSY")])
        values.append(positions[joint_names.index("REP")])
        values.append(positions[joint_names.index("RWY")])
        values.append(positions[joint_names.index("RWP")])
        values.append(positions[joint_names.index("RWR")])
        return values

    def build_target_joint_state(self, current_joints, left_arm_targets, right_arm_targets, left_gripper_targets, right_gripper_targets):
        target_state = list(current_joints.position)
        # Replace the values in the current state with the new arm values
        target_state[current_joints.name.index("LSP")] = left_arm_targets[0]
        target_state[current_joints.name.index("LSR")] = left_arm_targets[1]
        target_state[current_joints.name.index("LSY")] = left_arm_targets[2]
        target_state[current_joints.name.index("LEP")] = left_arm_targets[3]
        target_state[current_joints.name.index("LWY")] = left_arm_targets[4]
        target_state[current_joints.name.index("LWP")] = left_arm_targets[5]
        if (self.left_mode == self.GRIPPER):
            target_state[current_joints.name.index("LWR")] = left_arm_targets[6]
        else:
            target_state[current_joints.name.index("LWR")] = -math.pi / 2.0
        target_state[current_joints.name.index("RSP")] = right_arm_targets[0]
        target_state[current_joints.name.index("RSR")] = right_arm_targets[1]
        target_state[current_joints.name.index("RSY")] = right_arm_targets[2]
        target_state[current_joints.name.index("REP")] = right_arm_targets[3]
        target_state[current_joints.name.index("RWY")] = right_arm_targets[4]
        target_state[current_joints.name.index("RWP")] = right_arm_targets[5]
        if (self.right_mode == self.GRIPPER):
            target_state[current_joints.name.index("RWR")] = right_arm_targets[6]
        else:
            target_state[current_joints.name.index("RWR")] = math.pi / 2.0
        # Replace gripper values
        try:
            target_state[current_joints.name.index("LF1")] = left_gripper_targets[0]
        except:
            rospy.logwarn("No LF1 joint")
        try:
            target_state[current_joints.name.index("LF2")] = left_gripper_targets[1]
        except:
            rospy.logwarn("No LF2 joint")
        try:
            target_state[current_joints.name.index("RF1")] = right_gripper_targets[0]
        except:
            rospy.logwarn("No RF1 joint")
        try:
            target_state[current_joints.name.index("RF2")] = right_gripper_targets[1]
        except:
            rospy.logwarn("No RF2 joint")
        return target_state

    def get_finger_indices(self, current_names):
        finger_indices = []
        for index in range(len(current_names)):
            if ("F" in current_names[index] or "f" in current_names[index]):
                finger_indices.append(index)
        return finger_indices

    def interpolate_trajectory(self, current_names, start_config, end_config, num_intermediate_points):
        assert(len(start_config) == len(end_config))
        finger_indices = self.get_finger_indices(current_names)
        points = []
        for i in range(num_intermediate_points):
            point = []
            for j in range(len(start_config)):
                # Check to see if the joint value belongs to a finger
                if (j in finger_indices):
                    # For fingers, we don't interpolate - we just set the final value at every step
                    point.append(end_config[j])
                else:
                    # For other joints, we do a linear interpolation
                    point.append(start_config[j] + ((end_config[j] - start_config[j]) * (float(i) / float(num_intermediate_points))))
            points.append(point)
        points.append(end_config)
        return points

    def clean_point(self, current_joint_names, active_joint_names, point):
        cleaned_points = []
        for name in active_joint_names:
            setpoint = point[current_joint_names.index(name)]
            cleaned_points.append(setpoint)
        assert(len(cleaned_points) == len(active_joint_names))
        return cleaned_points

    def build_full_trajectory(self, current_names, current_joints, target_joint_state, execution_time):
        exec_time = self.execution_scaling_time * self.euclidean_distance(current_joints, target_joint_state)
        if (execution_time > exec_time):
            exec_time = execution_time
        num_intermediate_points = int(exec_time * 30.0)
        full_points = self.interpolate_trajectory(current_names, current_joints, target_joint_state, num_intermediate_points)
        new_traj = JointTrajectory()
        new_traj.joint_names = self.active_joints
        step_time = 1.0 / 30.0
        time = step_time
        for point in full_points:
            target_point = JointTrajectoryPoint()
            target_point.positions = self.clean_point(current_names, self.active_joints, point)
            target_point.velocities = [0.0 for i in range(len(point))]
            target_point.time_from_start = rospy.Duration(time)
            time += step_time
            new_traj.points.append(target_point)
        new_traj.header.stamp = rospy.Time.now()
        return new_traj

    '''
    def build_trajectory(self, current_joints, target_joint_state):
        new_traj = JointTrajectory()
        new_traj.joint_names = self.active_joints
        execution_time = self.execution_scaling_time * self.euclidean_distance(current_joints, target_joint_state)
        target_point = JointTrajectoryPoint()
        target_point.positions = target_joint_state
        target_point.velocities = [0.0 for i in range(len(target_joint_state))]
        target_point.time_from_start = rospy.Duration(execution_time)
        new_traj.points = [target_point]
        new_traj.header.stamp = rospy.Time.now()
        return new_traj
    '''

    def euclidean_distance(self, state1, state2, dimensions=None):
        assert(len(state1) == len(state2))
        if (dimensions == None):
            total = 0.0
            for index in range(len(state1)):
                temp = (state1[index] - state2[index]) ** 2
                total = total + temp
            return math.sqrt(total)
        else:
            assert(dimensions <= len(state1))
            total = 0.0
            for index in range(dimensions):
                temp = (state1[index] - state2[index]) ** 2
                total = total + temp
            return math.sqrt(total)

    def get_closest_solution(self, current_config, solutions, dimensions=None):
        print "Trying to find the closest of " + str(len(solutions)) + " IKfast solutions"
        # Compute the distance for each IK solution
        solution_dists = []
        for solution in solutions:
            dist = self.euclidean_distance(current_config, solution, dimensions)
            solution_dists.append([solution, dist])
        # Pick the closest IK solution
        closest = None
        min_distance = float('inf')
        for [solution, dist] in solution_dists:
            if (dist < min_distance):
                min_distance = dist
                closest = solution
        rospy.loginfo("Picked the closest IKfast solution with a distance of " + str(min_distance))
        return closest

    def get_left_arm_state(self):
        current_joint_state = None
        with self.joint_lock:
            current_joint_state = deepcopy(self.latest_joint_state)
        left_arm = self.extract_left_arm(current_joint_state.name, current_joint_state.position)
        return left_arm

    def get_right_arm_state(self):
        current_joint_state = None
        with self.joint_lock:
            current_joint_state = deepcopy(self.latest_joint_state)
        right_arm = self.extract_right_arm(current_joint_state.name, current_joint_state.position)
        return right_arm

    def compute_left_arm(self, target_pose, best=False):
        #print "Attempting to compute an IKfast solution for the left arm"
        ik_solution = None
        # Rotate ROS pose to match OpenRAVE orientation
        rotation = quaternion_about_axis(math.pi / 2.0, (0,0,1))
        adjustment = PoseFromTransform(TransformFromComponents([0.1,0.0,0.0], rotation))
        real_target_pose = ComposePoses(target_pose, adjustment)
        # Convert the ROS pose to an OpenRAVE transform
        Ttorso = self.torso_link.GetTransform()
        Tgripper = numpy.dot(Ttorso, PoseToMatrix(real_target_pose))
        # Query IKFAST
        if (best):
            ik_solutions = self.left_arm_ikfast.manip.FindIKSolutions(array(Tgripper),IkFilterOptions.CheckEnvCollisions)
            ik_solution = self.get_closest_solution(self.get_left_arm_state(), ik_solutions)
        else:
            ik_solution = self.left_arm_ikfast.manip.FindIKSolution(array(Tgripper),IkFilterOptions.CheckEnvCollisions)
        #print "IK solution for the left arm: " + str(ik_solution)
        return ik_solution

    def compute_right_arm(self, target_pose, best=False):
        #print "Attempting to compute an IKfast solution for the right arm"
        ik_solution = None
        # Rotate ROS pose to match OpenRAVE orientation
        rotation = quaternion_about_axis(-math.pi / 2.0, (0,0,1))
        adjustment = PoseFromTransform(TransformFromComponents([0.1,0.0,0.0], rotation))
        real_target_pose = ComposePoses(target_pose, adjustment)
        # Convert the ROS pose to an OpenRAVE transform
        Ttorso = self.torso_link.GetTransform()
        Tgripper = numpy.dot(Ttorso, PoseToMatrix(real_target_pose))
        # Query IKFAST
        if (best):
            ik_solutions = self.right_arm_ikfast.manip.FindIKSolutions(array(Tgripper),IkFilterOptions.CheckEnvCollisions)
            ik_solution = self.get_closest_solution(self.get_right_arm_state(), ik_solutions)
        else:
            ik_solution = self.right_arm_ikfast.manip.FindIKSolution(array(Tgripper),IkFilterOptions.CheckEnvCollisions)
        #print "IK solution for the right arm: " + str(ik_solution)
        return ik_solution

    def compute_left_peg(self, target_pose, best=False):
        #print "Attempting to compute an IKfast solution for the left arm"
        ik_solution = None
        # Rotate ROS pose to match OpenRAVE orientation
        rotation = quaternion_about_axis(math.pi / 2.0, (0,0,1))
        adjustment = PoseFromTransform(TransformFromComponents([0.0,0.0,0.0], rotation))
        real_target_pose = ComposePoses(target_pose, adjustment)
        # Convert the ROS pose to an OpenRAVE transform
        Ttorso = self.torso_link.GetTransform()
        Tgripper = numpy.dot(Ttorso, PoseToMatrix(real_target_pose))
        # Query IKFAST
        if (best):
            ik_solutions = self.left_peg_ikfast.manip.FindIKSolutions(array(Tgripper),IkFilterOptions.CheckEnvCollisions)
            ik_solution = self.get_closest_solution(self.get_left_arm_state(), ik_solutions, dimensions=6)
        else:
            ik_solution = self.left_peg_ikfast.manip.FindIKSolution(array(Tgripper),IkFilterOptions.CheckEnvCollisions)
        # Zero the last joint in the IK solution (since it doesn't actually exist)
        try:
            ik_solution[6] = -math.pi / 2.0
        except:
            pass
        #print "IK solution for the left arm: " + str(ik_solution)
        return ik_solution

    def compute_right_peg(self, target_pose, best=False):
        #print "Attempting to compute an IKfast solution for the right arm"
        ik_solution = None
        # Rotate ROS pose to match OpenRAVE orientation
        rotation = quaternion_about_axis(-math.pi / 2.0, (0,0,1))
        adjustment = PoseFromTransform(TransformFromComponents([0.0,0.0,0.0], rotation))
        real_target_pose = ComposePoses(target_pose, adjustment)
        # Convert the ROS pose to an OpenRAVE transform
        Ttorso = self.torso_link.GetTransform()
        Tgripper = numpy.dot(Ttorso, PoseToMatrix(real_target_pose))
        # Query IKFAST
        if (best):
            ik_solutions = self.right_peg_ikfast.manip.FindIKSolutions(array(Tgripper),IkFilterOptions.CheckEnvCollisions)
            ik_solution = self.get_closest_solution(self.get_right_arm_state(), ik_solutions, dimensions=6)
        else:
            ik_solution = self.right_peg_ikfast.manip.FindIKSolution(array(Tgripper),IkFilterOptions.CheckEnvCollisions)
        # Zero the last joint in the IK solution (since it doesn't actually exist)
        try:
            ik_solution[6] = math.pi / 2.0
        except:
            pass
        #print "IK solution for the right arm: " + str(ik_solution)
        return ik_solution

    def make_gripper_imarker(self, marker_pose, marker_name):
        new_marker = InteractiveMarker()
        new_marker.header.frame_id = marker_pose.header.frame_id
        new_marker.pose = marker_pose.pose
        new_marker.scale = 0.25
        new_marker.name = marker_name
        # Make the marker menu control (this includes the visual display marker)
        new_control = InteractiveMarkerControl()
        new_control.interaction_mode = InteractiveMarkerControl.MENU
        new_control.always_visible = True
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        gripper_marker = self.make_gripper_marker(marker_pose, marker_name)
        peg_marker = self.make_peg_marker(marker_pose, marker_name)
        new_control.markers.append(gripper_marker)
        new_control.markers.append(peg_marker)
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

    def make_gripper_marker(self, marker_pose, marker_name):
        marker = Marker()
        marker.header.frame_id = marker_pose.header.frame_id
        #Give it a unique ID
        marker.ns = marker_name + "G"
        marker.id = 1
        #Give the marker a type
        marker.type = Marker.MESH_RESOURCE
        if ("left" in marker_name.lower()):
            marker.mesh_resource = self.left_end_effector_mesh
            marker.color.r = 1.0
        elif ("right" in marker_name.lower()):
            marker.mesh_resource = self.right_end_effector_mesh
            marker.color.g = 1.0
        marker.pose = marker_pose.pose
        #Set the scale of the marker -- 1x1x1 here means native scale
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.a = 0.75
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def make_peg_marker(self, marker_pose, marker_name):
        marker = Marker()
        marker.header.frame_id = marker_pose.header.frame_id
        #Give it a unique ID
        marker.ns = marker_name + "P"
        marker.id = 2
        #Give the marker a type
        marker.type = Marker.MESH_RESOURCE
        if ("left" in marker_name.lower()):
            marker.mesh_resource = self.left_peg_mesh
            marker.color.r = 1.0
            marker.pose = ComposePoses(marker_pose.pose, self.left_peg_offset)
        elif ("right" in marker_name.lower()):
            marker.mesh_resource = self.right_peg_mesh
            marker.color.g = 1.0
            marker.pose = ComposePoses(marker_pose.pose, self.right_peg_offset)
        #Set the scale of the marker -- 1x1x1 here means native scale
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.a = 0.75
        marker.lifetime = rospy.Duration(0.5)
        return marker

if __name__ == '__main__':
    rospy.init_node('hubo_marker_teleop')
    marker_namespace = rospy.get_param("~marker_namespace", "hubo_marker_teleop")
    left_end_effector_mesh = rospy.get_param("~left_end_effector_mesh", "package://drchubo_v3/meshes/convhull_LWR_merged.stl")
    right_end_effector_mesh = rospy.get_param("~right_end_effector_mesh", "package://drchubo_v3/meshes/convhull_RWR_merged.stl")
    left_peg_mesh = rospy.get_param("~left_peg_mesh", "package://drchubo_v3/meshes/convhull_LWP_merged.stl")
    right_peg_mesh = rospy.get_param("~right_peg_mesh", "package://drchubo_v3/meshes/convhull_RWP_merged.stl")
    joint_action_server = rospy.get_param("~joint_action_server" , "/drchubo_fullbody_controller/joint_trajectory_action")
    enable_exec = rospy.get_param("~enable_exec", True)
    enable_viewer = rospy.get_param("~enable_viewer", True)
    maximum_movement = rospy.get_param("~max_movement", 6.0)
    active_joints = ["TSY", "LSP", "LSR", "LSY", "LEP", "LWY", "LWP", "LWR", "RSP", "RSR", "RSY", "REP", "RWY", "RWP", "RWR", "LHY", "LHR", "LHP", "LKP", "LAP", "LAR", "RHY", "RHR", "RHP", "RKP", "RAP", "RAR", "LF1", "RF1", "RF2"]
    HuboMarkerTeleop(marker_namespace, left_end_effector_mesh, right_end_effector_mesh, left_peg_mesh, right_peg_mesh, joint_action_server, active_joints, maximum_movement, enable_exec, enable_viewer)
