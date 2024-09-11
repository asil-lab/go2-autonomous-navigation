""" This is the main node for the scanning procedure of the LTM project.
Author: Alexander James Becoy
Revision: 1.0
Date: 11-09-2024
"""

import numpy as np
from geometry_msgs.msg import Quaternion

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

def quaternion_to_yaw(quaternion: Quaternion) -> float:
    """Converts a quaternion to a yaw angle.
    Args:
        quaternion (Quaternion): The quaternion to convert.

    Returns:
        float: The yaw angle representation of the quaternion.
    """
    assert isinstance(quaternion, Quaternion), "Input to argument quaternion is not of type Quaternion."

    return np.arctan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 
        1 - 2 * (quaternion.y**2 + quaternion.z**2)
    )