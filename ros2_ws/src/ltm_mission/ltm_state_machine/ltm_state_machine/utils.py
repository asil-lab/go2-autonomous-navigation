""" This is the main node for the ltm_state_machine package.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-23
"""

import re
import numpy as np

from geometry_msgs.msg import Quaternion

def reduce_consecutive_duplicates(lst) -> list:
    """ Reduces consecutive duplicates that only appear at the end of a list."""
    if len(lst) == 0:
        return lst
    if lst[-1] == lst[-2]:
        lst.pop()
    return lst

def get_snake_case(name: str) -> str:
    """ Converts a camel case string to snake case."""
    return re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower()

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Converts a yaw angle to a quaternion.
    Args:
        yaw (float): The yaw angle to convert.

    Returns:
        Quaternion: The quaternion representation of the yaw angle.
    """
    assert isinstance(yaw, float), "Input to argument yaw is not of type float."

    quaternion = Quaternion()
    quaternion.z = np.sin(yaw / 2)
    quaternion.w = np.cos(yaw / 2)
    return quaternion