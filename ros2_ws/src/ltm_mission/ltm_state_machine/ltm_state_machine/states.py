""" Here contains the states.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-05
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ltm_shared_msgs.srv import LoadMap, GenerateWaypoints

from ltm_state_machine.utils import get_snake_case

class State(Node):

    def __init__(self, name, id, is_terminal=False) -> None:
        super().__init__('state_' + get_snake_case(name) + '_node')
        self.name = name
        self.id = id
        self.terminal_flag = is_terminal
        self.error_flag = None

        # Initialize the state
        self.get_logger().info(f'Current state: {self.name}')
        if self.configure():
            self.run()

    def __del__(self) -> None:
        self.destroy_node()

    def configure(self) -> bool:
        self.state_pub = self.create_publisher(String, 'state', 1)
        return True

    def run(self) -> None:
        self.publish()

    def transition(self):
        if self.error_flag is not None:
            return ErrorState(self.error_flag)

    def publish(self) -> None:
        msg = String()
        msg.data = self.name
        self.state_pub.publish(msg)

    def __str__(self) -> str:
        return self.name
    
    def __repr__(self) -> str:
        return self.name + " " + str(self.id)
    
    def __eq__(self, other) -> bool:
        return self.id == other.id
    
    def __ne__(self, other) -> bool:
        return self.id != other.id
    
    def __hash__(self) -> int:
        return self.id

    def is_client_ready(self, client) -> bool:
        self.wait_for_service(client)
        return self.is_service_ready(client)

    def wait_for_service(self, client, timeout_sec=5) -> bool:
        return client.wait_for_service(timeout_sec)
    
    def is_service_ready(self, client) -> bool:
        return client.service_is_ready()
    
    def is_terminal(self) -> bool:
        return self.terminal_flag
    
    def is_error(self) -> bool:
        return self.error_flag is not None
    
    def flag_error(self, message) -> None:
        self.error_flag = message

    def print_error(self) -> None:
        self.get_logger().error(self.error_flag)


class UndefinedState(State):
    """ Undefined class is the state that is used to represent
    an undefined state.
    """

    def __init__(self):
        super().__init__("Undefined", 0)


class LoadMapState(State):
    """ LoadMap class is the state that loads the map.
    """
    UNDEFINED_MAP_NAME = 'undefined'

    def __init__(self) -> None:
        super().__init__("LoadMap", 1)

    def configure(self) -> bool:
        super().configure()

        # Create the client for the service
        self.load_map_client = self.create_client(LoadMap, 'state_machine/load_map')
        self.generate_waypoints_client = self.create_client(GenerateWaypoints, 'state_machine/generate_waypoints')

        # Throw an error if the client cannot find the service /state_machine/load_map
        if not self.is_client_ready(self.load_map_client):
            self.flag_error('Service state_machine/load_map is not available.')
            return False
        self.get_logger().info('Service state_machine/load_map found.')

        # Throw an error if the client cannot find the service /state_machine/generate_waypoints
        if not self.is_client_ready(self.generate_waypoints_client):
            self.flag_error('Service state_machine/generate_waypoints is not available.')
            return False
        self.get_logger().info('Service state_machine/generate_waypoints found.')

        self.get_logger().info('State LoadMap configured.')
        return True

    def run(self) -> None:
        super().run()

        # Load the map from the map server to the map reader
        self.get_logger().info('Loading map...')
        request = LoadMap.Request()
        request.filename = 'lab'
        future = self.load_map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Map loaded.')
        else:
            self.flag_error('Failed to load map.')
            return
        
        # # Generate waypoints from the map, and plan a route
        # self.get_logger().info('Generating waypoints...')
        # request = GenerateWaypoints.Request()
        # future = self.generate_waypoints_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)

        # if future.result() is not None:
        #     self.get_logger().info('Waypoints generated.')
        # else:
        #     self.flag_error('Failed to generate waypoints.')
        #     return

    def transition(self):
        super().transition()
        return CheckWaypointsState()


class CheckWaypointsState(State):
    """ CheckWaypoints class is the state that checks if there are
    waypoints remaining.
    """

    def __init__(self):
        super().__init__("CheckWaypoints", 2)
        self.waypoints_remaining = False

    def configure(self) -> bool:
        super().configure()

        self.get_logger().info('State CheckWaypoints configured.')
        return True

    def run(self) -> None:
        super().run()

        self.get_logger().info('Checking waypoints...')

    def transition(self):
        super().transition()
        if self.waypoints_remaining:
            return CheckDestinationState()
        else:
            return HomeState()
        

class CheckDestinationState(State):
    """ CheckDestination class is the state that checks if the robot
    has reached the destination.
    """

    def __init__(self):
        super().__init__("CheckDestination", 3)
        self.destination_reached = False

    def configure(self) -> bool:
        super().configure()

        self.get_logger().info('State CheckDestination configured.')
        return True

    def run(self) -> None:
        super().run()

        self.get_logger().info('Checking destination...')

    def transition(self):
        super().transition()
        if self.destination_reached:
            return ScanEnvironmentState()
        else:
            return MoveToDestinationState()
        

class MoveToDestinationState(State):
    """ MoveToDestination class is the state that moves the robot to
    the destination.
    """

    def __init__(self):
        super().__init__("MoveToDestination", 4)
        self.is_interrupted = False

    def configure(self) -> bool:
        super().configure()

        self.get_logger().info('State MoveToDestination configured.')
        return True

    def run(self) -> None:
        super().run()

        self.get_logger().info('Moving to destination...')

    def transition(self):
        super().transition()
        if self.is_interrupted:
            return ManualControlState()
        else:
            return CheckDestinationState()


class ScanEnvironmentState(State):
    """ ScanEnvironment class is the state that scans the environment.
    """

    def __init__(self):
        super().__init__("ScanEnvironment", 5)

    def configure(self) -> bool:
        super().configure()

        self.get_logger().info('State ScanEnvironment configured.')
        return True

    def run(self) -> None:
        super().run()

        self.get_logger().info('Scanning environment...')

    def transition(self):
        super().transition()
        return CheckWaypointsState()


class ManualControlState(State):
    """ ManualControl class is the state that allows manual control
    of the robot.
    """

    def __init__(self):
        super().__init__("ManualControl", 6)

    def configure(self) -> bool:
        super().configure()

        self.get_logger().info('State ManualControl configured.')
        return True

    def run(self) -> None:
        super().run()

        self.get_logger().info('Manual control...')

    def transition(self):
        super().transition()
        return ScanEnvironmentState()


class HomeState(State):
    """ Home class is the state that sends the robot back to the
    home location.
    """

    def __init__(self):
        super().__init__("Home", 7)

    def configure(self) -> bool:
        super().configure()

        self.get_logger().info('State Home configured.')
        return True

    def run(self) -> None:
        super().run()

        self.get_logger().info('Returning home...')

    def transition(self):
        super().transition()
        return ShutdownState()


class ShutdownState(State):
    """ Shutdown class is the state that shuts down the robot.
    """

    def __init__(self):
        super().__init__("Shutdown", 11, is_terminal=True)

    def transition(self):
        return ShutdownState()


class ErrorState(State):
    """ Error class is the state that handles errors.
    """

    def __init__(self, error_message) -> None:
        super().__init__("Error", 12, is_terminal=True)
        self.error_message = error_message

    def transition(self):
        return ShutdownState()
    

def get_all_service_names() -> list:
    return [state.get_service_name() for state in State.__subclasses__()]

def get_all_states() -> list:
    return [state() for state in State.__subclasses__()]

def get_state_by_id(id: int) -> State:
    for state in State.__subclasses__():
        if state().id == id:
            return state()
    return None