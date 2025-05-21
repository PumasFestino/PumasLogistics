#!/usr/bin/env python

"""
tug_joy_own_config is used to define all needed callbacks, which are needed for the current environment. There are also
some predefined callbacks.
"""

import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommand
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from geometry_msgs.msg import TwistStamped
import moveit_commander
import rospy
from tug_joy.joy_constants import *
from tug_joy.joy_base import Manager
from tug_joy.joy_base import Callback
from tug_joy.joy_std_callbacks import *

########################################################################################################################
#                                                    OWN CALLBACKS                                                     #
# def example_cb_fct(values_dict):                                                                                     #
#     # values_dict[ACTUATORNAME] = value_of_actuator                                                                  #
#     pass                                                                                                             #
#                                                                                                                      #
# example_cb_object = Callback('Example', [...used actuator...], example_cb_fct, ...filter option...)                  #
#                                                                                                                      #
########################################################################################################################
"""
All callback functions are defined in this section. Do not forget to create the Callback-object. The callback fct can
also be used to add or remove callbacks of the manager.
"""

CONTROLLER_MANAGER_NS = "/controller_manager"
SWITCH_CONTROLLERS_SERVICE = CONTROLLER_MANAGER_NS + "/switch_controller"
MOVEIT_CONTROLLERS = ["arm_controller"]
CARTESIAN_CONTROLLERS = ["cartesian_controller"]
JOINT_VELOCITY_CONTROLLERS = ["arm_%d_joint_velocity_controller" % (i + 1) for i in range(6)]

print "loading forbot config"

rospy.wait_for_service(SWITCH_CONTROLLERS_SERVICE, 10.0)
switch_controllers_service = rospy.ServiceProxy(SWITCH_CONTROLLERS_SERVICE, SwitchController)


def switch_controllers(start_controllers, stop_controllers):
    global switch_controllers_service
    try:
        resp = switch_controllers_service(start_controllers=start_controllers,
                                          stop_controllers=stop_controllers,
                                          strictness=SwitchControllerRequest.STRICT)
    except rospy.ServiceException as e:
        rospy.logerr("Error calling switch controllers service: %s", e)
    else:
        if not resp.ok:
            rospy.logerr("Switch controllers service failed")

# Cmd vel
cmd_vel = CmdVel(actuator_linear_x=AXIS.STICK_AXIS_LEFT_VERTICAL,
                 actuator_linear_y="",
                 actuator_angular_z=AXIS.STICK_AXIS_LEFT_HORIZONTAL,
                 namespace='cmd_vel_1/',
                 publishing_topic='/cmd_vel')

cmd_vel_cb = Callback(name='cmd_vel_1', actuators=cmd_vel.used_actuators, callback_fct=cmd_vel.callback)


def enable_disable_cmd_vel_cb(values_dict):
    if values_dict[BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT]:
        Manager().add_callback(cmd_vel_cb)
    else:
        stop_cmd_vel(values_dict)


def stop_cmd_vel(values_dict):
    Manager().remove_callback(cmd_vel_cb)
    cmd_vel.stop_cb(values_dict)


enable_cmd_vel = Callback('Cmd_Vel_On', [BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT], enable_disable_cmd_vel_cb,
                          CB_FILTERING_PRESS)
disable_cmd_vel = Callback('Cmd_Vel_Off', [BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT], enable_disable_cmd_vel_cb,
                           CB_FILTERING_RELEASE)

# arm cmds
arm_1_joint = AngularCommand(actuator=AXIS.STICK_AXIS_LEFT_HORIZONTAL, namespace="",
                             publishing_topic="/arm_1_joint_velocity_controller/command", inverse=True,
                             absolute_mode=False)
arm_2_joint = AngularCommand(actuator=AXIS.STICK_AXIS_LEFT_VERTICAL, namespace="",
                             publishing_topic="/arm_2_joint_velocity_controller/command", inverse=True,
                             absolute_mode=False)
arm_3_joint = AngularCommand(actuator=AXIS.STICK_AXIS_RIGHT_VERTICAL, namespace="",
                             publishing_topic="/arm_3_joint_velocity_controller/command", inverse=False,
                             absolute_mode=False)
arm_4_joint = AngularCommand(actuator=AXIS.STICK_AXIS_RIGHT_HORIZONTAL, namespace="",
                             publishing_topic="/arm_4_joint_velocity_controller/command", inverse=True,
                             absolute_mode=False)
arm_5_joint = AngularCommand(actuator=AXIS.CROSS_1_AXIS_VERTICAL, namespace="",
                             publishing_topic="/arm_5_joint_velocity_controller/command", inverse=False,
                             absolute_mode=False)
arm_6_joint = AngularCommand(actuator=AXIS.CROSS_1_AXIS_HORIZONTAL, namespace="",
                             publishing_topic="/arm_6_joint_velocity_controller/command", inverse=True,
                             absolute_mode=False)

gripper_action_client = None


def call_gripper_action(position):
    global gripper_action_client
    if not gripper_action_client:
        gripper_action_client = actionlib.SimpleActionClient("/gripper_controller/gripper_cmd", GripperCommandAction)
        gripper_action_client.wait_for_server(rospy.Duration(1))
    goal = GripperCommandGoal(command=GripperCommand(position=position))
    gripper_action_client.send_goal(goal)


def close_gripper(values_dict):
    call_gripper_action(0.0)


def open_gripper(values_dict):
    call_gripper_action(0.11)


arm_joints = [Callback(name='arm_1_joint', actuators=arm_1_joint.actuator, callback_fct=arm_1_joint.callback),
              Callback(name='arm_2_joint', actuators=arm_2_joint.actuator, callback_fct=arm_2_joint.callback),
              Callback(name='arm_3_joint', actuators=arm_3_joint.actuator, callback_fct=arm_3_joint.callback),
              Callback(name='arm_4_joint', actuators=arm_4_joint.actuator, callback_fct=arm_4_joint.callback),
              Callback(name='arm_5_joint', actuators=arm_5_joint.actuator, callback_fct=arm_5_joint.callback),
              Callback(name='arm_6_joint', actuators=arm_6_joint.actuator, callback_fct=arm_6_joint.callback),
              Callback(name='gripper_close', actuators=BUTTONS.CROSS_2_BUTTON_UP, callback_fct=close_gripper,
                       cb_filtering=CB_FILTERING_PRESS),
              Callback(name='gripper_open', actuators=BUTTONS.CROSS_2_BUTTON_DOWN, callback_fct=open_gripper,
                       cb_filtering=CB_FILTERING_PRESS),
              ]

arm_cartesian_publisher = rospy.Publisher("/cartesian_controller/arm_cmd_vel", TwistStamped, queue_size=1)


def move_arm_cartesian(values_dict):
    global arm_cartesian_publisher
    msg = TwistStamped()
    msg.twist.linear.x = 0.1 * values_dict[AXIS.STICK_AXIS_LEFT_VERTICAL]
    msg.twist.linear.y = -0.1 * values_dict[AXIS.STICK_AXIS_LEFT_HORIZONTAL]
    msg.twist.linear.z = 0.1 * values_dict[AXIS.STICK_AXIS_RIGHT_VERTICAL]
    arm_cartesian_publisher.publish(msg)

arm_cartesian = [Callback(name='cartesian', actuators=[AXIS.STICK_AXIS_LEFT_HORIZONTAL,
                                                       AXIS.STICK_AXIS_LEFT_VERTICAL,
                                                       AXIS.STICK_AXIS_RIGHT_VERTICAL],
                          callback_fct=move_arm_cartesian)]


move_group = moveit_commander.MoveGroupCommander("lwa4p_arm")


def arm_go_home_cb(values_dict):
    global move_group
    if values_dict[BUTTONS.SHOULDER_BUTTON_LOWER_LEFT] and values_dict[BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT]:
        move_group.clear_pose_targets()
        move_group.set_named_target("home")
        plan = move_group.plan()
        if plan:
            move_group.execute(plan)

arm_go_home = Callback(name='arm_go_home',
                       actuators=[BUTTONS.SHOULDER_BUTTON_LOWER_LEFT, BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT],
                       callback_fct=arm_go_home_cb, cb_filtering=CB_FILTERING_PRESS)


def enable_disable_arm_cb(values_dict):
    left_shoulder = values_dict[BUTTONS.SHOULDER_BUTTON_UPPER_LEFT]
    right_shoulder = values_dict[BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT]
    if left_shoulder and not right_shoulder:
        switch_controllers(JOINT_VELOCITY_CONTROLLERS, MOVEIT_CONTROLLERS + CARTESIAN_CONTROLLERS)
        Manager().add_callback_list(arm_joints)
    elif right_shoulder and not left_shoulder:
        switch_controllers(CARTESIAN_CONTROLLERS, MOVEIT_CONTROLLERS + JOINT_VELOCITY_CONTROLLERS)
        Manager().add_callback_list(arm_cartesian)
    else:
        stop_arm(values_dict)


def stop_arm(values_dict):
    Manager().remove_callback_list(arm_joints)
    Manager().remove_callback_list(arm_cartesian)
    arm_1_joint.stop_cb(values_dict)
    arm_2_joint.stop_cb(values_dict)
    arm_3_joint.stop_cb(values_dict)
    arm_4_joint.stop_cb(values_dict)
    arm_5_joint.stop_cb(values_dict)
    arm_6_joint.stop_cb(values_dict)
    switch_controllers(MOVEIT_CONTROLLERS, CARTESIAN_CONTROLLERS + JOINT_VELOCITY_CONTROLLERS)

enable_arm = Callback('Arm_On', [BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT],
                      enable_disable_arm_cb, CB_FILTERING_PRESS)
disable_arm = Callback('Arm_Off', [BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT],
                       enable_disable_arm_cb, CB_FILTERING_RELEASE)


# switch between arm and robot
def select_robot_cb(values_dict):
    print "Selected base"
    Manager().remove_callback(enable_arm)
    Manager().remove_callback(disable_arm)
    Manager().remove_callback(arm_go_home)
    stop_arm(values_dict)

    Manager().add_callback(enable_cmd_vel)
    Manager().add_callback(disable_cmd_vel)


def select_arm_cb(values_dict):
    print "Selected arm"
    Manager().remove_callback(enable_cmd_vel)
    Manager().remove_callback(disable_cmd_vel)
    stop_cmd_vel(values_dict)

    Manager().add_callback(enable_arm)
    Manager().add_callback(disable_arm)
    Manager().add_callback(arm_go_home)


select_robot = Callback('Select_Robot', [BUTTONS.FUNCTION_RIGHT], select_robot_cb, CB_FILTERING_PRESS)
select_arm = Callback('Select_ARM', [BUTTONS.FUNCTION_LEFT], select_arm_cb, CB_FILTERING_PRESS)


########################################################################################################################
#                                                 PREDEFINED CALLBACKS                                                 #
########################################################################################################################

def manager_start_cb():
    """
    This callback is called once at start time. It is used to load the initial callbacks. It can also be used to set
    the system, robot, or whatever to a defined state.
    USE THIS FUNCTION TO REGISTER ALL CALLBACKS, WHICH ARE USED AFTER STARTUP.
    """
    rospy.logdebug('manager_start_cb')
    Manager().add_callback(select_robot)
    Manager().add_callback(select_arm)


def manager_break_once_cb():
    """
    This callback is called once if no joy-node-publisher-connection is
    available any more. If connection is lost, it is called once before
    'manager_break_continuous_cb()'. It can be used to set the system, robot,
    or whatever to a defined state.
    """
    rospy.logdebug('manager_break_once_cb')


def manager_break_continuous_cb():
    """
    This callback is called continuous if no joy-node-publisher-connection
    is available any more. If connection is lost, it is called after
    'manager_break_once_cb()'. It can be used to set the system, robot, or
    whatever to a defined state.
    """
    rospy.logdebug('manager_break_continuous_cb')


def manager_exit_cb():
    """ This callback is called before this node will be killed. It can be used
    to set the system, robot, or whatever to a defined state. """
    rospy.logdebug('manager_exit_cb')
