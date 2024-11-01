"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import rclpy
from rclpy.node import Node

from unitree_go.msg import SportModeState
from geometry_msgs.msg import TwistStamped

from collections import deque
from scipy import stats

class OdometryBiasEstimatorNode(Node):

    def __init__(self):
        super().__init__('odometry_bias_estimator_node')
        self.initialize_queues()
        self.initialize_bias_publisher()
        self.initialize_sport_mode_state_subscriber()
        self.get_logger().info('Odometry Bias Estimator Node has been initialized.')

    def sport_mode_state_callback(self, msg):
        # Update the time stamps
        dt = (self.get_clock().now() - self.current_time).nanoseconds / 1e9
        self.time_stamps.append(dt)
        self.current_time = self.get_clock().now()

        # Update the linear and angular velocities
        for i in range(3):
            self.linear_velocities[i].append(msg.position[i])
            # self.angular_velocities[i].append(msg.angular_velocity[i])

        self.publish_bias()

    def publish_bias(self):
        # Check if the queues are full
        if len(self.time_stamps) < self.time_stamps.maxlen:
            return
        
        # Calculate the bias
        time_stamps = list(self.time_stamps)
        linear_velocities = [list(v) for v in self.linear_velocities]
        # angular_velocities = [list(v) for v in self.angular_velocities]

        # Calculate the bias for x
        slope, intercept, r_value, p_value, std_err = stats.linregress(time_stamps, linear_velocities[0])
        # self.get_logger().info(f'Linear Velocity (x) Slope: {slope}, Intercept: {intercept}, R-Value: {r_value}, P-Value: {p_value}, Std-Error: {std_err}')

        # Create the message
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = slope

        # Publish the message
        self.bias_publisher.publish(msg)

    def initialize_queues(self, queue_size=1000):
        self.time_stamps = deque(maxlen=queue_size)
        self.linear_velocities = [deque(maxlen=queue_size) for _ in range(3)]
        # self.angular_velocities = [deque(maxlen=queue_size) for _ in range(3)]

    def initialize_bias_publisher(self):
        self.bias_publisher = self.create_publisher(TwistStamped, 'odometry_bias', 10)

    def initialize_sport_mode_state_subscriber(self):
        self.sport_mode_state_subscription = self.create_subscription(
            SportModeState, 'sportmodestate', self.sport_mode_state_callback, 10)
        self.current_time = self.get_clock().now()

def main():
    rclpy.init()
    node = OdometryBiasEstimatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
