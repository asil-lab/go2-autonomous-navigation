""" Here contains the states.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-05
"""

import rclpy
from rclpy.node import Node

from ltm_shared_msgs.srv import LoadMap

from ltm_state_machine.utils import get_snake_case

class State(Node):

    def __init__(self, name, id, is_terminal=False):
        super().__init__('state_' + get_snake_case(name) + '_node')
        self.name = name
        self.id = id
        self.is_terminal = is_terminal

    def __del__(self):
        self.destroy_node()

    def configure(self):
        pass

    def run(self):
        pass

    def transition(self):
        pass

    def __str__(self):
        return self.name
    
    def __repr__(self):
        return self.name + " " + str(self.id)
    
    def __eq__(self, other):
        return self.id == other.id
    
    def __ne__(self, other):
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

    # def emergency_stop(self):
    #     return EmergencyStop()
    
    def error(self):
        return ErrorState()


class UndefinedState(State):
    """ Undefined class is the state that is used to represent
    an undefined state.
    """

    def __init__(self):
        super().__init__("Undefined", 0)


class LoadMapState(State):
    """ LoadMap class is the state that loads the map.
    """

    def __init__(self):
        super().__init__("LoadMap", 2)

    def configure(self):
        self.load_map_client = self.create_client(LoadMap, 'state_machine/load_map')

        if not self.is_client_ready(self.load_map_client):
            self.get_logger().info('Service state_machine/load_map not available.')
            return False

        self.get_logger().info('Service state_machine/load_map found.')
        return True

    def run(self):
        self.get_logger().info('Loading map...')
        request = LoadMap.Request()
        future = self.load_map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Map loaded.')
        else:
            self.get_logger().info('Failed to load map.')

    def transition(self):
        return ShutdownState()


# class PlanPath(State):
#     """ PlanPath class is the state that creates a list of waypoints
#     to follow, and creates a path to follow.
#     """

#     def __init__(self):
#         super().__init__("PlanPath", 6)

#     def transition(self):
#         return Navigate()
    

# class Navigate(State):
#     """ Navigate class is the state that moves the robot to the
#     next waypoint.
#     """

#     def __init__(self):
#         super().__init__("Navigate", 7)

#     def transition(self):
#         if self.input == "reached":
#             return Scan()
#         elif self.input == "failed":
#             return ManualControl()
#         else:
#             return Navigate()


# class ManualControl(State):
#     """ ManualControl class is the state that allows manual control
#     of the robot.
#     """

#     def __init__(self):
#         super().__init__("ManualControl", 8)

#     def transition(self):
#         if self.input == "stop":
#             return Scan()
#         else:
#             return ManualControl()

# class Scan(State):
#     """ Scan class is the state that scans the environment at the
#     current waypoint in 360 degrees.
#     TODO: Implement smaller scanning state machine.
#     """

#     def __init__(self):
#         super().__init__("Scan", 9)

#     def transition(self):
#         if self.input == "stop":
#             return Home()
#         else:
#             return Navigate()
        

# class HomeState(State):
#     """ Home class is the state that returns the robot to the
#     starting position.
#     """

#     def __init__(self):
#         super().__init__("Home", 10)

#     def transition(self):
#         return ShutdownState()
    

class ShutdownState(State):
    """ Shutdown class is the state that shuts down the robot.
    """

    def __init__(self):
        super().__init__("Shutdown", 11, is_terminal=True)

    def transition(self):
        return ShutdownState()

# class EmergencyStopState(State):
#     """ EmergencyStop class is the state that stops the robot
#     immediately.
#     """

#     def __init__(self):
#         super().__init__("EmergencyStop", 12)

#     def transition(self):
#         return Shutdown()
    

class ErrorState(State):
    """ Error class is the state that handles errors.
    """

    def __init__(self):
        super().__init__("Error", 13)

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