""" This is the main node for the scanning procedure of the LTM project.
Author: Alexander James Becoy
Revision: 1.0
Date: 11-09-2024
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
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

    quat_R = R.from_euler('zyx', [yaw, 0, 0]).as_quat()

    quaternion_msg = Quaternion()
    quaternion_msg.x = quat_R[0]
    quaternion_msg.y = quat_R[1]
    quaternion_msg.z = quat_R[2]
    quaternion_msg.w = quat_R[3]

    return quaternion_msg

def quaternion_to_yaw(quaternion: Quaternion) -> float:
    """Converts a quaternion to a yaw angle.
    Args:
        quaternion (Quaternion): The quaternion to convert.

    Returns:
        float: The yaw angle representation of the quaternion.
    """
    assert isinstance(quaternion, Quaternion), "Input to argument quaternion is not of type Quaternion."

    return R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_euler('zyx')[0]

def normalize_yaw(yaw: float) -> float:
    """Normalizes a yaw angle to be within the range [-pi, pi].
    Args:
        yaw (float): The yaw angle to normalize.

    Returns:
        float: The normalized yaw angle.
    """
    assert isinstance(yaw, float), "Input to argument yaw is not of type float."

    return (yaw + np.pi) % (2 * np.pi) - np.pi

def image_msg_to_numpy(image_msg: Image) -> np.ndarray:
    """Converts an Image message to a numpy array.
    Args:
        image_msg (Image): The Image message to convert.

    Returns:
        np.ndarray: The numpy array representation of the Image message.
    """
    assert isinstance(image_msg, Image), "Input to argument image_msg is not of type Image."

    return np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)