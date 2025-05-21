#!/usr/bin/env python

from joy_constants import *
from rospy import loginfo, logwarn, logerr, logdebug


class Actuator:
    """
    Base of all actuators.
    """

    def __init__(self, name, filtering=CB_FILTERING_DISABLED):
        """
        Constructor of Actuator
        :param name: name of this actuator
        :param filtering:
                CB_FILTERING_NONE     -> callback is enabled for this actuator. This parameter saves
                                         the information if last change (press -> release  or release -> press)
                CB_FILTERING_DISABLED -> callback is disabled for this actuator.
        """
        self.name = name
        self.value = 0.0
        self.__used = 0
        self.filtering = filtering

    def set_used(self, in_use):
        """
        Mark this actuator used or unused by one loaded callback
        :param in_use: True if actuator is now used
                       False if actuator is now used
        """

        if self.__used > 0 and in_use:
            logwarn("[" + Actuator.__str__(self) + "] is used more than once!")

        if in_use:
            self.__used += 1
        else:
            self.__used -= 1

    def is_used(self):
        """
        Check if actuator is already used
        :return: True if actuator is in use by at least on callback
                 False if actuator is not used
        """
        return True if self.__used > 0 else False

    def __str__(self):
        return str(self.name)


class Button(Actuator):
    """
    Derivation from actuator. This class is for buttons, which only can be
    1 (pressed) or 0 (not pressed). It extracts its data from the 'buttons'-
    array of the joy-msg.
    """

    def __init__(self, button_id, name):
        """
        Constructor of Button derived from Actuator
        :param button_id: index of this button in the 'buttons'-array of the joy-msg.
        :param name: name of this actuator
        """
        Actuator.__init__(self, name, CB_FILTERING_NONE)
        self.button_id = button_id

    def set_value(self, msg):
        """
        Is called from the subscriber callback and updates the value of the
        actuator. It reads the value of the 'Buttons'-array of the joy-msg at
        its corresponding index. Additionally information for the filter are defined.
        :param msg: ROS subscriber message
        """
        try:
            if self.value != msg.buttons[self.button_id]:
                self.value = msg.buttons[self.button_id]
                self.filtering = (CB_FILTERING_PRESS if self.value == 1 else CB_FILTERING_RELEASE)
        except:
            logerr("[" + Actuator.__str__(self) + "] button id '" + str(self.button_id) + "' not found")

    def __str__(self):
        return ' B ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value)

    def __repr__(self):
        return str(self)


class VirtualButton(Actuator):
    """
    Derivation from actuator. This class is for buttons, which only can be
    1 (pressed) or 0 (not pressed). It extracts its data from the 'axis'-
    array of the joy-msg and rounds the float values to simulate buttons.
    """

    def __init__(self, axis_id, response_at_positive, name):
        """
        Constructor of Button derived from Actuator
        :param axis_id: index of this button in the 'axes'-array of the joy-msg.
        :param response_at_positive: True  if >  0.5 represents a pressed button
                                     False if < -0.5 represents a pressed button
        :param name: name of this actuator
        """
        Actuator.__init__(self, name, CB_FILTERING_NONE)
        self.axis_id = axis_id
        self.response_at_positive = response_at_positive

    def set_value(self, msg):
        """
        Is called from the subscriber callback and updates the value of the
        actuator. It reads the value of the 'Axes'-array of the joy-msg at
        its corresponding index, rounds the floating value and checks if the
        virtual button is for positive or negative values. Additionally
        information for the filter are defined.
        :param msg: ROS subscriber message
        """
        try:
            new_value = round(msg.axes[self.axis_id])
            if self.response_at_positive:
                new_value = max(new_value, 0)
            else:
                new_value = max(new_value * -1, 0)

            if self.value != new_value:
                self.value = new_value
                self.filtering = (CB_FILTERING_PRESS if self.value == 1 else CB_FILTERING_RELEASE)
        except:
            logerr("[" + Actuator.__str__(self) + "] virtual button id '" + str(self.axis_id) + "' not found")

    def __str__(self):
        return 'VB ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value)

    def __repr__(self):
        return str(self)


class Axis(Actuator):
    """
    Derivation from actuator. This class is for axis, which can be floating
    values between -1.0 and 1.0. It extracts its data from the 'axis'-array
    of the joy-msg.
    """

    def __init__(self, axis_id, name):
        """
        Constructor of Axis derived from Actuator
        :param axis_id: index of this button in the 'axes'-array of the joy-msg.
        :param name: name of this actuator
        """
        Actuator.__init__(self, name)
        self.axis_id = axis_id

    def set_value(self, msg):
        """
        Is called from the subscriber callback and updates the value of the
        actuator. It reads the value of the 'Axes'-array of the joy-msg at
        its corresponding index.
        :param msg: ROS subscriber message
        """
        try:
            self.value = msg.axes[self.axis_id]
        except:
            logerr("[" + Actuator.__str__(self) + "] axis id '" + str(self.axis_id) + "' not found")

    def __str__(self):
        return ' A ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value)

    def __repr__(self):
        return str(self)


class VirtualAxis(Actuator):
    """
    Derivation from actuator. This class is for axis, which can be floating
    values between -1.0 and 1.0. It extracts its data from two inputs, which
    can be buttons or axes.
    """

    def __init__(self, positive_id, negative_id, use_axis, name):
        """
        Constructor of Axis derived from Actuator
        :param positive_id: Index of the actuator in the 'buttons'-array or 'axes'-array of the joy-msg.
                            This actuator is used to generate the positive values of the virtual axis.
                            If id is None only the negative id is used.
        :param negative_id: Index of the actuator in the 'buttons'-array or 'axes'-array of the joy-msg.
                            This actuator is used to generate the negative values of the virtual axis.
                            If id is None only the positive id is used.
        :param use_axis: Defines if the ids are for the 'buttons'-array or the 'axes'-array of the joy-msg.
        :param name: name of this actuator
        """
        Actuator.__init__(self, name)
        self.positive_id = positive_id
        self.negative_id = negative_id

        if self.positive_id is not None and self.negative_id is None:
            self.set_value = self.set_value_with_axes_pos if use_axis else self.set_value_with_buttons_pos
        elif self.positive_id is None and self.negative_id is not None:
            self.set_value = self.set_value_with_axes_neg if use_axis else self.set_value_with_buttons_neg
        elif self.positive_id is not None and self.negative_id is not None:
            self.set_value = self.set_value_with_axes_pos_neg if use_axis else self.set_value_with_buttons_pos_neg

    def set_value_with_axes_pos(self, msg):
        """
        Set value if only positive id is set and axes are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(max(msg.axes[self.positive_id], 0.0))
        except:
            logerr("[" + Actuator.__str__(self) + "] axis id '" + str(self.positive_id) + "' not found")

    def set_value_with_axes_neg(self, msg):
        """
        Set value if only negative id is set and axes are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(min(msg.axes[self.negative_id], 0.0))
        except:
            logerr("[" + Actuator.__str__(self) + "] axis id '" + str(self.negative_id) + "' not found")

    def set_value_with_axes_pos_neg(self, msg):
        """
        Set value if positive and negative id are set and axes are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(msg.axes[self.positive_id]) - abs(msg.axes[self.negative_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] one or more axis ids not found")

    def set_value_with_buttons_pos(self, msg):
        """
        Set value if only positive id is set and buttons are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(msg.buttons[self.positive_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] button id '" + str(self.positive_id) + "' not found")

    def set_value_with_buttons_neg(self, msg):
        """
        Set value if only negative id is set and buttons are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(msg.buttons[self.negative_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] button id '" + str(self.negative_id) + "' not found")

    def set_value_with_buttons_pos_neg(self, msg):
        """
        Set value if positive and negative ids are set and buttons are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(msg.buttons[self.positive_id]) - abs(msg.buttons[self.negative_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] one or more button ids not found")

    def __str__(self):
        return 'VA ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value)

    def __repr__(self):
        return str(self)
