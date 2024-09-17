""" This is the main node for the scanning procedure of the LTM project.
Author: Alexander James Becoy
Revision: 1.0
Date: 11-09-2024
"""

import numpy as np
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image

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

def normalize_yaw(yaw: float) -> float:
    """Normalizes a yaw angle to be within the range [-pi, pi].
    Args:
        yaw (float): The yaw angle to normalize.

    Returns:
        float: The normalized yaw angle.
    """
    return yaw % (2 * np.pi)

def image_msg_to_numpy(image_msg: Image) -> np.ndarray:
    """Converts an Image message to a numpy array.
    Args:
        image_msg (Image): The Image message to convert.

    Returns:
        np.ndarray: The numpy array representation of the Image message.
    """
    assert isinstance(image_msg, Image), "Input to argument image_msg is not of type Image."

    return np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)