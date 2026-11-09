import copy
import threading

import yaml

from mrs_msgs.msg import GazeboSpawnerStatus, GazeboSpawnerVehicle


# #{ hw_api_outputs(robot_params)
def hw_api_outputs(robot_params: dict) -> dict:
    '''
    HW API outputs provided by the spawned vehicle. A keyword is in robot_params only if the spawner accepted it,
    i.e. the vehicle's template calls the component.
    '''
    return {
        'distance_sensor': 'enable-rangefinder' in robot_params,
        'ground_truth': 'enable-ground-truth' in robot_params,
    }


# #}


# #{ platform_config_yaml(robot_params)
def platform_config_yaml(robot_params: dict) -> str:
    '''Platform config with the vehicle's HW API outputs, for the MRS HW API manager'''
    return yaml.safe_dump({'mrs_uav_hw_api': {'outputs': hw_api_outputs(robot_params)}})


# #}


class SpawnerStatus:
    '''
    Per-vehicle state of the spawner, published as GazeboSpawnerStatus by the spawner node.
    spawning(), spawned() and failed() require queued() first (KeyError otherwise, a spawner bug).
    The spawner refuses a name that is_active(), so queued() replaces only a FAILED entry.
    '''

    # #{ __init__(self)
    def __init__(self):
        self._mutex = threading.Lock()
        self._vehicles = {}  # name -> GazeboSpawnerVehicle, in insertion order

    # #}

    # #{ queued(self, name, fcu_url, system_id)
    def queued(self, name: str, fcu_url: str, system_id: int) -> None:
        '''Adds the vehicle, or replaces the entry of a vehicle with the same name (respawn)'''
        vehicle = GazeboSpawnerVehicle()
        vehicle.name = name
        vehicle.state = GazeboSpawnerVehicle.STATE_QUEUED
        vehicle.mavlink_fcu_url = fcu_url
        vehicle.mavlink_system_id = system_id
        with self._mutex:
            self._vehicles.pop(name, None)
            self._vehicles[name] = vehicle

    # #}

    # #{ is_active(self, name)
    def is_active(self, name: str) -> bool:
        '''True if a vehicle with this name is queued, being spawned or spawned (a FAILED one can be respawned)'''
        with self._mutex:
            return name in self._vehicles and self._vehicles[name].state != GazeboSpawnerVehicle.STATE_FAILED

    # #}

    # #{ spawning(self, name)
    def spawning(self, name: str) -> None:
        with self._mutex:
            self._vehicles[name].state = GazeboSpawnerVehicle.STATE_SPAWNING

    # #}

    # #{ spawned(self, name, platform_config)
    def spawned(self, name: str, platform_config: str) -> None:
        with self._mutex:
            self._vehicles[name].state = GazeboSpawnerVehicle.STATE_SPAWNED
            self._vehicles[name].platform_config = platform_config

    # #}

    # #{ failed(self, name, message)
    def failed(self, name: str, message: str) -> None:
        with self._mutex:
            self._vehicles[name].state = GazeboSpawnerVehicle.STATE_FAILED
            self._vehicles[name].message = message

    # #}

    # #{ to_msg(self)
    def to_msg(self) -> GazeboSpawnerStatus:
        msg = GazeboSpawnerStatus()
        with self._mutex:
            # copies: the entries change under the lock, publish() serializes outside it
            msg.vehicles = [copy.copy(v) for v in self._vehicles.values()]
        return msg

    # #}
