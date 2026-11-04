#!/usr/bin/python3
import pytest

from mrs_uav_gazebo_simulator.core.user_input_manager import UserInputManager
from mrs_uav_gazebo_simulator.utils.spawner_exceptions import NoFreeIDAvailable, NoValidIDGiven


class FakeLogger:

    def info(self, msg):
        pass

    def warn(self, msg):
        pass

    def error(self, msg):
        pass


class FakeNode:

    def get_logger(self):
        return FakeLogger()


def make_manager():
    return UserInputManager(ros_node=FakeNode(), jinja_templates={'x500': None}, model_spacing=5.0, default_robot_name='uav')


# #{ IDs: MAV_SYS_ID = ID + 1 must fit 1..255
def test_id_255_ignored():
    with pytest.raises(NoValidIDGiven):
        make_manager().parse_user_input('255 --x500')
    assert make_manager().parse_user_input('254 --x500')['ids'] == [254]


def test_assign_free_id_up_to_254():
    manager = make_manager()
    manager.assigned_ids = set(range(0, 254))
    assert manager.assign_free_id() == 254
    manager.assigned_ids = set(range(0, 255))
    with pytest.raises(NoFreeIDAvailable):
        manager.assign_free_id()


# #}


# #{ names: <default_robot_name><ID>, or the whole name with --name
def test_default_names():
    assert make_manager().parse_user_input('1 2 --x500')['names'] == ['uav1', 'uav2']


def test_name_is_the_whole_name():
    params = make_manager().parse_user_input('3 --x500 --name pes')
    assert params['names'] == ['pes']
    assert params['ids'] == [3]


def test_names_one_per_id():
    params = make_manager().parse_user_input('3 5 --x500 --name pes kocka')
    assert params['names'] == ['pes', 'kocka']
    assert params['ids'] == [3, 5]


def test_names_count_must_match_ids():
    for args in ['1 --x500 --name', '1 --x500 --name pes kocka', '1 2 --x500 --name pes']:
        with pytest.raises(AssertionError, match='one name per vehicle ID'):
            make_manager().parse_user_input(args)


def test_duplicate_names_refused():
    with pytest.raises(AssertionError, match='more than once'):
        make_manager().parse_user_input('1 2 --x500 --name pes pes')


def test_name_with_component_args_refused():
    with pytest.raises(AssertionError, match='names separated by spaces'):
        make_manager().parse_user_input('1 --x500 --name a:=b')


def test_invalid_name_refused():
    for name in ['1uav', 'uav-a']:
        with pytest.raises(AssertionError, match='Invalid vehicle name'):
            make_manager().parse_user_input(f'1 --x500 --name {name}')


# #}
