#!/usr/bin/env python

"""
tug_joy_own_config is used to define all needed callbacks, which are needed for the current environment. There are also
some predefined callbacks.
"""

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
# Cmd vel
cmd_vel = CmdVel(actuator_linear_x=AXIS.STICK_AXIS_LEFT_VERTICAL,
                 actuator_linear_y="",
                 actuator_angular_z=AXIS.STICK_AXIS_LEFT_HORIZONTAL,
                 namespace='cmd_vel_1/',
                 publishing_topic='/cmd_vel',
                 trans_vel=0.1, rot_vel=0.3)

cmd_vel_cbs = [Callback('cmd_vel_1', cmd_vel.used_actuators, cmd_vel.callback),
               Callback('cmd_vel_1_lin+', BUTTONS.CROSS_1_BUTTON_UP, cmd_vel.increase_linear_speed_cb,
                        CB_FILTERING_PRESS),
               Callback('cmd_vel_1_lin-', BUTTONS.CROSS_1_BUTTON_DOWN, cmd_vel.decrease_linear_speed_cb,
                        CB_FILTERING_PRESS),
               Callback('cmd_vel_1_ang+', BUTTONS.CROSS_1_BUTTON_LEFT, cmd_vel.increase_angular_speed_cb,
                        CB_FILTERING_PRESS),
               Callback('cmd_vel_1_ang-', BUTTONS.CROSS_1_BUTTON_RIGHT, cmd_vel.decrease_angular_speed_cb,
                        CB_FILTERING_PRESS)]


def enable_disable_cmd_vel_cb(values_dict):
    if values_dict[BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT]:
        Manager().add_callback_list(cmd_vel_cbs)
    else:
        Manager().remove_callback_list(cmd_vel_cbs)
        cmd_vel.stop_cb(values_dict)


enable_cmd_vel = Callback('Cmd_Vel_On', [BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT], enable_disable_cmd_vel_cb,
                          CB_FILTERING_PRESS)
disable_cmd_vel = Callback('Cmd_Vel_Off', [BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT], enable_disable_cmd_vel_cb,
                           CB_FILTERING_RELEASE)


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
    # Manager().add_callback(stick_left_on)
    # Manager().add_callback(stick_left_off)
    # Manager().add_callback(stick_right_on)
    # Manager().add_callback(stick_right_off)
    # Manager().add_callback_list(cmd_vel)
    Manager().add_callback(enable_cmd_vel)
    Manager().add_callback(disable_cmd_vel)


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
