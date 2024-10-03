""" This is the main node for the ltm_navigation_planner package.
Author: Alexander James Becoy
Revision: 1.0
Date: 30-09-2024
"""

import numpy as np
from geometry_msgs.msg import Quaternion

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