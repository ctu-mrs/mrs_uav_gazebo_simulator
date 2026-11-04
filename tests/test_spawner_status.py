#!/usr/bin/python3
import pytest
import yaml

from mrs_msgs.msg import GazeboSpawnerVehicle
from mrs_uav_gazebo_simulator.core.spawner_status import SpawnerStatus, hw_api_outputs, platform_config_yaml

FCU_URL = 'udp://127.0.0.1:14005@127.0.0.1:14004'


# #{ outputs
def test_outputs_from_keywords():
    assert hw_api_outputs({'enable-rangefinder': None}) == {'distance_sensor': True, 'ground_truth': False}
    assert hw_api_outputs({'enable-ground-truth': {'topic_name': 'x'}})['ground_truth'] is True
    assert hw_api_outputs({
        'enable-rangefinder-up': None,
        'enable-rangefinder-external': None
    }) == {
        'distance_sensor': False,
        'ground_truth': False
    }


def test_platform_config_yaml():
    assert yaml.safe_load(platform_config_yaml({'enable-rangefinder': None})) == {
        'mrs_uav_hw_api': {
            'outputs': {
                'distance_sensor': True,
                'ground_truth': False
            }
        }
    }


# #}


# #{ status
def test_state_sequence():
    status = SpawnerStatus()
    status.queued('uav1', FCU_URL, 2)
    status.spawning('uav1')
    status.spawned('uav1', 'cfg')

    vehicles = status.to_msg().vehicles
    assert len(vehicles) == 1
    assert vehicles[0].name == 'uav1'
    assert vehicles[0].state == GazeboSpawnerVehicle.STATE_SPAWNED
    assert vehicles[0].platform_config == 'cfg'
    assert vehicles[0].mavlink_fcu_url == FCU_URL
    assert vehicles[0].mavlink_system_id == 2


def test_multiple_vehicles():
    status = SpawnerStatus()
    status.queued('uav1', FCU_URL, 2)
    status.spawned('uav1', 'cfg')
    status.queued('uav2', 'udp://127.0.0.1:14009@127.0.0.1:14008', 3)

    vehicles = status.to_msg().vehicles
    assert [v.name for v in vehicles] == ['uav1', 'uav2']
    assert [v.state for v in vehicles] == [GazeboSpawnerVehicle.STATE_SPAWNED, GazeboSpawnerVehicle.STATE_QUEUED]


def test_respawn_replaces_failed():
    status = SpawnerStatus()
    status.queued('uav1', FCU_URL, 2)
    status.failed('uav1', 'boom')
    status.queued('uav1', FCU_URL, 2)

    vehicles = status.to_msg().vehicles
    assert len(vehicles) == 1
    assert vehicles[0].state == GazeboSpawnerVehicle.STATE_QUEUED
    assert vehicles[0].message == ''


def test_to_msg_is_a_copy():
    status = SpawnerStatus()
    status.queued('uav1', FCU_URL, 2)
    msg = status.to_msg()
    status.failed('uav1', 'boom')

    assert msg.vehicles[0].state == GazeboSpawnerVehicle.STATE_QUEUED
    assert msg.vehicles[0].message == ''


def test_is_active():
    # a name in use blocks a second spawn under the same name, a FAILED one can be respawned
    status = SpawnerStatus()
    assert not status.is_active('uav1')
    status.queued('uav1', FCU_URL, 2)
    assert status.is_active('uav1')
    status.spawned('uav1', 'cfg')
    assert status.is_active('uav1')
    status.failed('uav1', 'boom')
    assert not status.is_active('uav1')


def test_unknown_vehicle_raises():
    # spawning/spawned/failed require queued() first, a miss is a spawner bug
    with pytest.raises(KeyError):
        SpawnerStatus().spawning('uav1')


# #}
