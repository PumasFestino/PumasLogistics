#!/usr/bin/env python

"""
This file contains all used constants of tug_joy.
"""


class CONTROLLER:
    """
    All known controllers are listed here.
    IF A NEW CONTROLLER IS ADDED, DON'T FORGET TO ADD IT HERE!!
    """
    # Controller names
    CONTROLLER_NAME_PS3 = 'ps3'
    CONTROLLER_NAME_XBOX360 = 'xbox360'
    CONTROLLER_NAME_LOGITECH_RUMBLE_PAD_2 = 'logitech_rumble_pad_2'
    CONTROLLER_NAME_DEFAULT = 'default'

# CB filter types
CB_FILTERING_DISABLED = 'Disabled'
CB_FILTERING_NONE = 'None'
CB_FILTERING_PRESS = 'Press'
CB_FILTERING_RELEASE = 'Release'


class BUTTONS:
    """
    All names that can be used for buttons.
    """
    # General button names
    CROSS_1_BUTTON_LEFT = 'cross_1_button_left'
    CROSS_1_BUTTON_UP = 'cross_1_button_up'
    CROSS_1_BUTTON_DOWN = 'cross_1_button_down'
    CROSS_1_BUTTON_RIGHT = 'cross_1_button_right'

    CROSS_2_BUTTON_LEFT = 'cross_2_button_left'
    CROSS_2_BUTTON_UP = 'cross_2_button_up'
    CROSS_2_BUTTON_DOWN = 'cross_2_button_down'
    CROSS_2_BUTTON_RIGHT = 'cross_2_button_right'

    FUNCTION_LEFT = 'function_left'
    FUNCTION_RIGHT = 'function_right'
    FUNCTION_CENTER = 'function_center'

    STICK_BUTTON_LEFT = 'stick_button_left'
    STICK_BUTTON_RIGHT = 'stick_button_right'

    SHOULDER_BUTTON_LOWER_LEFT = 'shoulder_button_lower_left'
    SHOULDER_BUTTON_UPPER_LEFT = 'shoulder_button_upper_left'
    SHOULDER_BUTTON_MIDDLE_LEFT = 'shoulder_button_middle_left'
    SHOULDER_BUTTON_LOWER_RIGHT = 'shoulder_button_lower_right'
    SHOULDER_BUTTON_UPPER_RIGHT = 'shoulder_button_upper_right'
    SHOULDER_BUTTON_MIDDLE_RIGHT = 'shoulder_button_middle_right'


class AXIS:
    """
    All names that can be used for axes.
    """
    # General axis names
    CROSS_1_AXIS_LEFT = 'cross_1_axis_left'
    CROSS_1_AXIS_UP = 'cross_1_axis_up'
    CROSS_1_AXIS_DOWN = 'cross_1_axis_down'
    CROSS_1_AXIS_RIGHT = 'cross_1_axis_right'
    CROSS_1_AXIS_HORIZONTAL = 'cross_1_axis_horizontal'
    CROSS_1_AXIS_VERTICAL = 'cross_1_axis_vertical'

    CROSS_2_AXIS_LEFT = 'cross_2_axis_left'
    CROSS_2_AXIS_UP = 'cross_2_axis_up'
    CROSS_2_AXIS_DOWN = 'cross_2_axis_down'
    CROSS_2_AXIS_RIGHT = 'cross_2_axis_right'
    CROSS_2_AXIS_HORIZONTAL = 'cross_2_axis_horizontal'
    CROSS_2_AXIS_VERTICAL = 'cross_2_axis_vertical'

    STICK_AXIS_LEFT_HORIZONTAL = 'stick_axis_left_horizontal'
    STICK_AXIS_LEFT_VERTICAL = 'stick_axis_left_vertical'
    STICK_AXIS_RIGHT_HORIZONTAL = 'stick_axis_right_horizontal'
    STICK_AXIS_RIGHT_VERTICAL = 'stick_axis_right_vertical'

    SHOULDER_AXIS_LOWER_LEFT = 'shoulder_axis_lower_left'
    SHOULDER_AXIS_UPPER_LEFT = 'shoulder_axis_upper_left'
    SHOULDER_AXIS_MIDDLE_LEFT = 'shoulder_axis_middle_left'
    SHOULDER_AXIS_LOWER_RIGHT = 'shoulder_axis_lower_right'
    SHOULDER_AXIS_UPPER_RIGHT = 'shoulder_axis_upper_right'
    SHOULDER_AXIS_MIDDLE_RIGHT = 'shoulder_axis_middle_right'

    ACCELERATION_X = 'acceleration_x'
    ACCELERATION_Y = 'acceleration_y'
    ACCELERATION_Z = 'acceleration_z'

    GYROSCOPE_ROLL = 'gyroscope_roll'
    GYROSCOPE_PITCH = 'gyroscope_pitch'
    GYROSCOPE_YAW = 'gyroscope_yaw'
