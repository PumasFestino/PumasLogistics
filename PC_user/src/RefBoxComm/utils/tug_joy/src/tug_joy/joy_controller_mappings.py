#!/usr/bin/env python

"""
This file contains all mappings between different controllers and general
actuator names. If a controller does not support some functionality natively,
it is calculated, but with some inaccuracy, e.g. if a button has no analog
values between 1 (pressed) and 0 (not pressed) the axis also has only 1 and 0.
"""

from joy_actuators import Button
from joy_actuators import VirtualButton
from joy_actuators import VirtualAxis
from joy_actuators import Axis

from joy_constants import *


class PS3Mapping:
    """
    Mapping for the PS3 controller
    """

    mapping = {
        # Button mapping general
        BUTTONS.CROSS_1_BUTTON_LEFT: Button(7, 'cross_1_button_left'),
        BUTTONS.CROSS_1_BUTTON_UP: Button(4, 'cross_1_button_up'),
        BUTTONS.CROSS_1_BUTTON_DOWN: Button(6, 'cross_1_button_down'),
        BUTTONS.CROSS_1_BUTTON_RIGHT: Button(5, 'cross_1_button_right'),

        BUTTONS.CROSS_2_BUTTON_LEFT: Button(15, 'cross_2_button_left'),
        BUTTONS.CROSS_2_BUTTON_UP: Button(12, 'cross_2_button_up'),
        BUTTONS.CROSS_2_BUTTON_DOWN: Button(14, 'cross_2_button_down'),
        BUTTONS.CROSS_2_BUTTON_RIGHT: Button(13, 'cross_2_button_right'),

        BUTTONS.FUNCTION_LEFT: Button(0, 'function_left'),
        BUTTONS.FUNCTION_RIGHT: Button(3, 'function_right'),
        BUTTONS.FUNCTION_CENTER: Button(16, 'function_center'),

        BUTTONS.STICK_BUTTON_LEFT: Button(1, 'stick_button_left'),
        BUTTONS.STICK_BUTTON_RIGHT: Button(2, 'stick_button_right'),

        BUTTONS.SHOULDER_BUTTON_LOWER_LEFT: Button(8, 'shoulder_button_lower_left'),
        BUTTONS.SHOULDER_BUTTON_UPPER_LEFT: Button(10, 'shoulder_button_upper_left'),
        BUTTONS.SHOULDER_BUTTON_MIDDLE_LEFT: None,
        BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT: Button(9, 'shoulder_button_lower_right'),
        BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT: Button(11, 'shoulder_button_upper_right'),
        BUTTONS.SHOULDER_BUTTON_MIDDLE_RIGHT: None,

        # Axis mapping
        AXIS.CROSS_1_AXIS_LEFT: Axis(11, 'cross_1_axis_left'),
        AXIS.CROSS_1_AXIS_UP: Axis(8, 'cross_1_axis_up'),
        AXIS.CROSS_1_AXIS_DOWN: Axis(10, 'cross_1_axis_down'),
        AXIS.CROSS_1_AXIS_RIGHT: Axis(9, 'cross_1_axis_right'),
        AXIS.CROSS_1_AXIS_HORIZONTAL: VirtualAxis(11, 9, True, 'cross_1_axis_horizontal'),
        AXIS.CROSS_1_AXIS_VERTICAL: VirtualAxis(8, 10, True, 'cross_1_axis_vertical'),

        AXIS.CROSS_2_AXIS_LEFT: Axis(19, 'cross_2_axis_left'),
        AXIS.CROSS_2_AXIS_UP: Axis(16, 'cross_2_axis_up'),
        AXIS.CROSS_2_AXIS_DOWN: Axis(18, 'cross_2_axis_down'),
        AXIS.CROSS_2_AXIS_RIGHT: Axis(17, 'cross_2_axis_right'),
        AXIS.CROSS_2_AXIS_HORIZONTAL: VirtualAxis(19, 17, True, 'cross_2_axis_horizontal'),
        AXIS.CROSS_2_AXIS_VERTICAL: VirtualAxis(16, 18, True, 'cross_2_axis_vertical'),

        AXIS.STICK_AXIS_LEFT_HORIZONTAL: Axis(0, 'stick_axis_left_horizontal'),
        AXIS.STICK_AXIS_LEFT_VERTICAL: Axis(1, 'stick_axis_left_vertical'),
        AXIS.STICK_AXIS_RIGHT_HORIZONTAL: Axis(2, 'stick_axis_right_horizontal'),
        AXIS.STICK_AXIS_RIGHT_VERTICAL: Axis(3, 'stick_axis_right_vertical'),

        AXIS.SHOULDER_AXIS_LOWER_LEFT: Axis(12, 'shoulder_axis_lower_left'),
        AXIS.SHOULDER_AXIS_UPPER_LEFT: Axis(14, 'shoulder_axis_upper_left'),
        AXIS.SHOULDER_AXIS_MIDDLE_LEFT: None,
        AXIS.SHOULDER_AXIS_LOWER_RIGHT: Axis(13, 'shoulder_axis_lower_right'),
        AXIS.SHOULDER_AXIS_UPPER_RIGHT: Axis(15, 'shoulder_axis_upper_right'),
        AXIS.SHOULDER_AXIS_MIDDLE_RIGHT: None,

        AXIS.ACCELERATION_X: Axis(24, 'acceleration_x'),
        AXIS.ACCELERATION_Y: Axis(23, 'acceleration_y'),
        AXIS.ACCELERATION_Z: Axis(25, 'acceleration_z'),

        AXIS.GYROSCOPE_ROLL: None,
        AXIS.GYROSCOPE_PITCH: None,
        AXIS.GYROSCOPE_YAW: Axis(26, 'gyroscope_yaw')

    }


class LogitechRumblePad2Mapping:
    """
    Mapping for the Logitech Rumble Pad 2 controller
    """
    mapping = {
        # Button mapping general
        BUTTONS.CROSS_1_BUTTON_LEFT: VirtualButton(4, True, 'cross_1_button_left'),
        BUTTONS.CROSS_1_BUTTON_UP: VirtualButton(5, True, 'cross_1_button_up'),
        BUTTONS.CROSS_1_BUTTON_DOWN: VirtualButton(5, False, 'cross_1_button_down'),
        BUTTONS.CROSS_1_BUTTON_RIGHT: VirtualButton(4, False, 'cross_1_button_right'),

        BUTTONS.CROSS_2_BUTTON_LEFT: Button(0, 'cross_2_button_left'),
        BUTTONS.CROSS_2_BUTTON_UP: Button(3, 'cross_2_button_up'),
        BUTTONS.CROSS_2_BUTTON_DOWN: Button(1, 'cross_2_button_down'),
        BUTTONS.CROSS_2_BUTTON_RIGHT: Button(2, 'cross_2_button_right'),

        BUTTONS.FUNCTION_LEFT: Button(8, 'function_left'),
        BUTTONS.FUNCTION_RIGHT: Button(9, 'function_right'),
        BUTTONS.FUNCTION_CENTER: None,

        BUTTONS.STICK_BUTTON_LEFT: Button(10, 'stick_button_left'),
        BUTTONS.STICK_BUTTON_RIGHT: Button(11, 'stick_button_right'),

        BUTTONS.SHOULDER_BUTTON_LOWER_LEFT: Button(6, 'shoulder_button_lower_left'),
        BUTTONS.SHOULDER_BUTTON_UPPER_LEFT: Button(4, 'shoulder_button_upper_left'),
        BUTTONS.SHOULDER_BUTTON_MIDDLE_LEFT: None,
        BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT: Button(7, 'shoulder_button_lower_right'),
        BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT: Button(5, 'shoulder_button_upper_right'),
        BUTTONS.SHOULDER_BUTTON_MIDDLE_RIGHT: None,

        # Axis mapping
        AXIS.CROSS_1_AXIS_LEFT: VirtualAxis(4, None, True, 'cross_1_axis_left'),
        AXIS.CROSS_1_AXIS_UP: VirtualAxis(5, None, True, 'cross_1_axis_up'),
        AXIS.CROSS_1_AXIS_DOWN: VirtualAxis(None, 5, True, 'cross_1_axis_down'),
        AXIS.CROSS_1_AXIS_RIGHT: VirtualAxis(None, 4, True, 'cross_1_axis_right'),
        AXIS.CROSS_1_AXIS_HORIZONTAL: Axis(4, 'cross_1_axis_horizontal'),
        AXIS.CROSS_1_AXIS_VERTICAL: Axis(5, 'cross_1_axis_vertical'),

        AXIS.CROSS_2_AXIS_LEFT: VirtualAxis(0, None, False, 'cross_2_axis_left'),
        AXIS.CROSS_2_AXIS_UP: VirtualAxis(3, None, False, 'cross_2_axis_up'),
        AXIS.CROSS_2_AXIS_DOWN: VirtualAxis(1, None, False, 'cross_2_axis_down'),
        AXIS.CROSS_2_AXIS_RIGHT: VirtualAxis(2, None, False, 'cross_2_axis_right'),
        AXIS.CROSS_2_AXIS_HORIZONTAL: VirtualAxis(0, 2, False, 'cross_2_axis_horizontal'),
        AXIS.CROSS_2_AXIS_VERTICAL: VirtualAxis(3, 1, False, 'cross_2_axis_vertical'),

        AXIS.STICK_AXIS_LEFT_HORIZONTAL: Axis(0, 'stick_axis_left_horizontal'),
        AXIS.STICK_AXIS_LEFT_VERTICAL: Axis(1, 'stick_axis_left_vertical'),
        AXIS.STICK_AXIS_RIGHT_HORIZONTAL: Axis(2, 'stick_axis_right_horizontal'),
        AXIS.STICK_AXIS_RIGHT_VERTICAL: Axis(3, 'stick_axis_right_vertical'),

        AXIS.SHOULDER_AXIS_LOWER_LEFT: VirtualAxis(6, None, False, 'shoulder_axis_lower_left'),
        AXIS.SHOULDER_AXIS_UPPER_LEFT: VirtualAxis(4, None, False, 'shoulder_axis_upper_left'),
        AXIS.SHOULDER_AXIS_MIDDLE_LEFT: None,
        AXIS.SHOULDER_AXIS_LOWER_RIGHT: VirtualAxis(7, None, False, 'shoulder_axis_lower_right'),
        AXIS.SHOULDER_AXIS_UPPER_RIGHT: VirtualAxis(5, None, False, 'shoulder_axis_upper_right'),
        AXIS.SHOULDER_AXIS_MIDDLE_RIGHT: None,

        AXIS.ACCELERATION_X: None,  # Axis(24, 'acceleration_x'),
        AXIS.ACCELERATION_Y: None,  # Axis(23, 'acceleration_y'),
        AXIS.ACCELERATION_Z: None,  # Axis(25, 'acceleration_z'),

        AXIS.GYROSCOPE_ROLL: None,
        AXIS.GYROSCOPE_PITCH: None,
        AXIS.GYROSCOPE_YAW: None  # Axis(26, 'gyroscope_yaw')
    }


class XBox360Mapping:
    """
    Mapping for the XBox 360 controller
    """
    mapping = {
        # Button mapping general
        BUTTONS.CROSS_1_BUTTON_LEFT: Button(11, 'cross_1_button_left'),
        BUTTONS.CROSS_1_BUTTON_UP: Button(13, 'cross_1_button_up'),
        BUTTONS.CROSS_1_BUTTON_DOWN: Button(14, 'cross_1_button_down'),
        BUTTONS.CROSS_1_BUTTON_RIGHT: Button(12, 'cross_1_button_right'),

        BUTTONS.CROSS_2_BUTTON_LEFT: Button(2, 'cross_2_button_left'),
        BUTTONS.CROSS_2_BUTTON_UP: Button(3, 'cross_2_button_up'),
        BUTTONS.CROSS_2_BUTTON_DOWN: Button(0, 'cross_2_button_down'),
        BUTTONS.CROSS_2_BUTTON_RIGHT: Button(1, 'cross_2_button_right'),

        BUTTONS.FUNCTION_LEFT: Button(6, 'function_left'),
        BUTTONS.FUNCTION_RIGHT: Button(7, 'function_right'),
        BUTTONS.FUNCTION_CENTER: Button(8, 'function_center'),

        BUTTONS.STICK_BUTTON_LEFT: Button(9, 'stick_button_left'),
        BUTTONS.STICK_BUTTON_RIGHT: Button(10, 'stick_button_right'),

        BUTTONS.SHOULDER_BUTTON_LOWER_LEFT: VirtualButton(2, False, 'shoulder_button_lower_left'),
        BUTTONS.SHOULDER_BUTTON_UPPER_LEFT: Button(4, 'shoulder_button_upper_left'),
        BUTTONS.SHOULDER_BUTTON_MIDDLE_LEFT: None,
        BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT: VirtualButton(5, False, 'shoulder_button_lower_right'),
        BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT: Button(5, 'shoulder_button_upper_right'),
        BUTTONS.SHOULDER_BUTTON_MIDDLE_RIGHT: None,

        # Axis mapping
        AXIS.CROSS_1_AXIS_LEFT: VirtualAxis(11, None, False, 'cross_1_axis_left'),
        AXIS.CROSS_1_AXIS_UP: VirtualAxis(13, None, False, 'cross_1_axis_up'),
        AXIS.CROSS_1_AXIS_DOWN: VirtualAxis(14, None, False, 'cross_1_axis_down'),
        AXIS.CROSS_1_AXIS_RIGHT: VirtualAxis(12, None, False, 'cross_1_axis_right'),
        AXIS.CROSS_1_AXIS_HORIZONTAL: VirtualAxis(11, 12, False, 'cross_1_axis_horizontal'),
        AXIS.CROSS_1_AXIS_VERTICAL: VirtualAxis(13, 14, False, 'cross_1_axis_vertical'),

        AXIS.CROSS_2_AXIS_LEFT: VirtualAxis(2, None, False, 'cross_2_axis_left'),
        AXIS.CROSS_2_AXIS_UP: VirtualAxis(3, None, False, 'cross_2_axis_up'),
        AXIS.CROSS_2_AXIS_DOWN: VirtualAxis(0, None, False, 'cross_2_axis_down'),
        AXIS.CROSS_2_AXIS_RIGHT: VirtualAxis(1, None, False, 'cross_2_axis_right'),
        AXIS.CROSS_2_AXIS_HORIZONTAL: VirtualAxis(2, 1, False, 'cross_2_axis_horizontal'),
        AXIS.CROSS_2_AXIS_VERTICAL: VirtualAxis(3, 0, False, 'cross_2_axis_vertical'),

        AXIS.STICK_AXIS_LEFT_HORIZONTAL: Axis(0, 'stick_axis_left_horizontal'),
        AXIS.STICK_AXIS_LEFT_VERTICAL: Axis(1, 'stick_axis_left_vertical'),
        AXIS.STICK_AXIS_RIGHT_HORIZONTAL: Axis(3, 'stick_axis_right_horizontal'),
        AXIS.STICK_AXIS_RIGHT_VERTICAL: Axis(4, 'stick_axis_right_vertical'),

        AXIS.SHOULDER_AXIS_LOWER_LEFT: Axis(2, 'shoulder_axis_lower_left'),
        AXIS.SHOULDER_AXIS_UPPER_LEFT: VirtualAxis(4, None, False, 'shoulder_axis_upper_left'),
        AXIS.SHOULDER_AXIS_MIDDLE_LEFT: None,
        AXIS.SHOULDER_AXIS_LOWER_RIGHT: Axis(5, 'shoulder_axis_lower_right'),
        AXIS.SHOULDER_AXIS_UPPER_RIGHT: VirtualAxis(5, None, False, 'shoulder_axis_upper_right'),
        AXIS.SHOULDER_AXIS_MIDDLE_RIGHT: None,

        AXIS.ACCELERATION_X: None,
        AXIS.ACCELERATION_Y: None,
        AXIS.ACCELERATION_Z: None,

        AXIS.GYROSCOPE_ROLL: None,
        AXIS.GYROSCOPE_PITCH: None,
        AXIS.GYROSCOPE_YAW: None
    }


class DefaultMapping:
    """
    Mapping for the default controller of our laboratory
    """
    mapping = {
        # Button mapping general
        BUTTONS.CROSS_1_BUTTON_LEFT: VirtualButton(4, True, 'cross_1_button_left'),
        BUTTONS.CROSS_1_BUTTON_UP: VirtualButton(5, True, 'cross_1_button_up'),
        BUTTONS.CROSS_1_BUTTON_DOWN: VirtualButton(5, False, 'cross_1_button_down'),
        BUTTONS.CROSS_1_BUTTON_RIGHT: VirtualButton(4, False, 'cross_1_button_right'),

        BUTTONS.CROSS_2_BUTTON_LEFT: Button(0, 'cross_2_button_left'),
        BUTTONS.CROSS_2_BUTTON_UP: Button(1, 'cross_2_button_up'),
        BUTTONS.CROSS_2_BUTTON_DOWN: Button(2, 'cross_2_button_down'),
        BUTTONS.CROSS_2_BUTTON_RIGHT: Button(3, 'cross_2_button_right'),

        BUTTONS.FUNCTION_LEFT: Button(8, 'function_left'),
        BUTTONS.FUNCTION_RIGHT: Button(9, 'function_right'),
        BUTTONS.FUNCTION_CENTER: None,

        BUTTONS.STICK_BUTTON_LEFT: Button(10, 'stick_button_left'),
        BUTTONS.STICK_BUTTON_RIGHT: Button(11, 'stick_button_right'),

        BUTTONS.SHOULDER_BUTTON_LOWER_LEFT: Button(5, 'shoulder_button_lower_left'),
        BUTTONS.SHOULDER_BUTTON_UPPER_LEFT: Button(4, 'shoulder_button_upper_left'),
        BUTTONS.SHOULDER_BUTTON_MIDDLE_LEFT: None,
        BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT: Button(7, 'shoulder_button_lower_right'),
        BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT: Button(6, 'shoulder_button_upper_right'),
        BUTTONS.SHOULDER_BUTTON_MIDDLE_RIGHT: None,

        # Axis mapping
        AXIS.CROSS_1_AXIS_LEFT: VirtualAxis(4, None, True, 'cross_1_axis_left'),
        AXIS.CROSS_1_AXIS_UP: VirtualAxis(5, None, True, 'cross_1_axis_up'),
        AXIS.CROSS_1_AXIS_DOWN: VirtualAxis(None, 5, True, 'cross_1_axis_down'),
        AXIS.CROSS_1_AXIS_RIGHT: VirtualAxis(None, 4, True, 'cross_1_axis_right'),
        AXIS.CROSS_1_AXIS_HORIZONTAL: Axis(4, 'cross_1_axis_horizontal'),
        AXIS.CROSS_1_AXIS_VERTICAL: Axis(5, 'cross_1_axis_vertical'),

        AXIS.CROSS_2_AXIS_LEFT: VirtualAxis(0, None, False, 'cross_2_axis_left'),
        AXIS.CROSS_2_AXIS_UP: VirtualAxis(1, None, False, 'cross_2_axis_up'),
        AXIS.CROSS_2_AXIS_DOWN: VirtualAxis(2, None, False, 'cross_2_axis_down'),
        AXIS.CROSS_2_AXIS_RIGHT: VirtualAxis(3, None, False, 'cross_2_axis_right'),
        AXIS.CROSS_2_AXIS_HORIZONTAL: VirtualAxis(0, 3, False, 'cross_2_axis_horizontal'),
        AXIS.CROSS_2_AXIS_VERTICAL: VirtualAxis(1, 2, False, 'cross_2_axis_vertical'),

        AXIS.STICK_AXIS_LEFT_HORIZONTAL: Axis(0, 'stick_axis_left_horizontal'),
        AXIS.STICK_AXIS_LEFT_VERTICAL: Axis(1, 'stick_axis_left_vertical'),
        AXIS.STICK_AXIS_RIGHT_HORIZONTAL: Axis(2, 'stick_axis_right_horizontal'),
        AXIS.STICK_AXIS_RIGHT_VERTICAL: Axis(3, 'stick_axis_right_vertical'),

        AXIS.SHOULDER_AXIS_LOWER_LEFT: VirtualAxis(5, None, False, 'shoulder_axis_lower_left'),
        AXIS.SHOULDER_AXIS_UPPER_LEFT: VirtualAxis(4, None, False, 'shoulder_axis_upper_left'),
        AXIS.SHOULDER_AXIS_MIDDLE_LEFT: None,
        AXIS.SHOULDER_AXIS_LOWER_RIGHT: VirtualAxis(7, None, False, 'shoulder_axis_lower_right'),
        AXIS.SHOULDER_AXIS_UPPER_RIGHT: VirtualAxis(6, None, False, 'shoulder_axis_upper_right'),
        AXIS.SHOULDER_AXIS_MIDDLE_RIGHT: None,

        AXIS.ACCELERATION_X: None,  # Axis(24, 'acceleration_x'),
        AXIS.ACCELERATION_Y: None,  # Axis(23, 'acceleration_y'),
        AXIS.ACCELERATION_Z: None,  # Axis(25, 'acceleration_z'),

        AXIS.GYROSCOPE_ROLL: None,
        AXIS.GYROSCOPE_PITCH: None,
        AXIS.GYROSCOPE_YAW: None  # Axis(26, 'gyroscope_yaw')
    }
