import ast
import re
import os
import csv
import yaml
import random
import math

from rclpy.node import Node
from mrs_uav_gazebo_simulator.utils.spawner_exceptions import *


class UserInputManager():

    def __init__(self, ros_node: Node, jinja_templates: dict, model_spacing: float, default_robot_name: str):
        self._ros_node = ros_node
        self._jinja_templates = jinja_templates
        self._model_spacing = model_spacing
        self._default_robot_name = default_robot_name

        self.assigned_ids = set()

    # #{ get_model_help_text(self, model_name)
    def get_model_help_text(self, model_name):
        '''
        Create a help string by loading all callable components from a given template in the following format
        Component name
            Description:
            Default args:
        '''
        self._ros_node.get_logger().info(f'Getting help for model {model_name}')
        try:
            template_wrapper = self._jinja_templates[model_name]
            response = f'Components used in template "{template_wrapper.jinja_template.filename}":\n'
        except ValueError:
            return f'Template for model {model_name} not found'

        for name, component in template_wrapper.components.items():
            response += f'{component.keyword}\n\tDescription: {component.description}\n\tDefault args: {component.default_args}\n\n'

        return response

    # #}

    # #{ get_spawner_help_text(self)
    def get_spawner_help_text(self):
        '''Create a generic help string for the spawner basic use'''

        self._ros_node.get_logger().info(f'Getting generic spawner help')
        response = 'The spawn service expects the following input (as a string):\n'
        response += '\tdevice ids (integers separated by spaces, auto-assigned if no ID is specified),\n'
        response += '\tmodel (use \'--\' with a model name to select a specific model),\n'
        response += '\tkeywords (specified inside jinja macros as "spawner_keyword". Add \'--\' before each keyword when calling spawn),\n'
        response += '\tcomponent args following a keyword (values separated by spaces or a python dict, overrides "spawner_default_args" in jinja macros),\n'
        response += '\n'
        response += '\tModels available: '

        for model_name in sorted(self._jinja_templates.keys()):
            response += f'{model_name}, '

        return response

    # #}

    # #{ get_help_text(self, input_dict):
    def get_help_text(self, input_dict):
        '''
        Used to construct the help text (string) for a given dict of input args
        Returns:
            generic spawner help
            or
            help for a specific model
            or
            None (if the input does not contain "help")
        '''
        if not input_dict['help']:
            return None

        if input_dict['model'] is None:
            display_text = self.get_spawner_help_text()
        else:
            display_text = self.get_model_help_text(input_dict['model'])

        return display_text

    # #}

    # #{ parse_string_to_objects(self, input_str)
    def parse_string_to_objects(self, input_str):
        '''
        Attempt to convert input_str into a dictionary or a list
        Convert numerals into number datatypes whenever possible
        Returns None if the input cannot be interpreted as dict or list
        '''
        input_str = input_str.strip()

        params = []
        for s in input_str.split():
            if len(s) > 0:
                try:
                    # try to convert input_str to numbers
                    params.append(ast.literal_eval(s))
                except (SyntaxError, ValueError):
                    # leave non-numbers as string
                    params.append(s)

        params_dict = {}
        if isinstance(params, list):
            # try to convert named args into a dict
            for p in params:
                try:
                    if ':=' in p:
                        kw, arg = p.split(':=')
                        try:
                            # try to convert arg to number
                            params_dict[kw] = ast.literal_eval(arg)
                        except (SyntaxError, ValueError):
                            # leave non-numbers as string
                            params_dict[kw] = arg
                except TypeError:
                    pass

        if len(params_dict.keys()) > 0 and len(params_dict.keys()) == len(params):
            # whole input converted to a dict
            return params_dict
        else:
            return params

        return None

    # #}

    # #{ parse_user_input(self, input_str)
    def parse_user_input(self, input_str):
        '''
        Extract params from an input string, create spawner args
        expected input:
            device ids (integers separated by spaces)
            keywords (specified in jinja components starting with '--')
            component args following a keyword (values separated by spaces)
        :param input_str: string containing all args in the format specified above
        :return: a dict in format {keyword: component_args}, always contains keys "help", "model", "ids", "names", "spawn_poses"
        NOTE: arguments of a component/keyword will always be parsed as a list/dict, even for a single value

        Raises:
        AssertionError in case of unexpected data in mandatory values under keys "model", "ids", "names", "spawn_poses"
        '''

        input_dict = {'help': False, 'model': None, 'ids': [], 'names': [], 'spawn_poses': {}}

        # parse out the keywords starting with '--'
        pattern = re.compile(r'(--\S*)')
        substrings = [m.strip() for m in re.split(pattern, input_str) if len(m.strip()) > 0]

        if len(substrings) < 1:
            input_dict['help'] = True
            return input_dict

        # before the first keyword, there should only be device IDs
        first_keyword_index = 0
        if '--' not in substrings[0]:
            input_dict['ids'] = self.parse_string_to_objects(substrings[0])
            first_keyword_index = 1
        else:
            input_dict['ids'].append(self.assign_free_id())

        # pair up keywords with args
        for i in range(first_keyword_index, len(substrings)):

            if substrings[i].startswith('--'):
                input_dict[substrings[i][2:]] = None
                continue
            else:
                input_keys = [*input_dict.keys()]
                if len(input_keys) > 1:
                    input_dict[input_keys[-1]] = self.parse_string_to_objects(substrings[i])

        # attempt to match model to available templates
        for k in input_dict.keys():
            if k in self._jinja_templates.keys():
                input_dict['model'] = str(k)
                del input_dict[k]
                break

        valid_ids = []

        for ID in input_dict['ids']:
            if not isinstance(ID, int):
                if ID in self._jinja_templates.keys() and input_dict['model'] is None:
                    self._ros_node.get_logger().info(f'Using {ID} as model template')
                    input_dict['model'] = ID
                else:
                    self._ros_node.get_logger().warn(f'Ignored ID {ID}: Not an integer')
                continue
            if ID < 0 or ID > 255:
                self._ros_node.get_logger().warn(f'Ignored ID {ID}: Must be in range(0, 256)')
                continue
            if ID in self.assigned_ids:
                self._ros_node.get_logger().warn(f'Ignored ID {ID}: Already assigned')
                continue
            valid_ids.append(ID)

        input_dict['ids'].clear()

        if '--help' in substrings:
            input_dict['help'] = True
            return input_dict

        if len(valid_ids) > 0:
            self._ros_node.get_logger().info(f'Valid robot IDs: {valid_ids}')
            input_dict['ids'] = valid_ids
            self.assigned_ids.update(input_dict['ids'])
        else:
            raise NoValidIDGiven('No valid ID given. Check your input')

        if 'pos' in input_dict.keys():
            try:
                input_dict['spawn_poses'] = self.get_spawn_poses_from_args(input_dict['pos'], input_dict['ids'])
            except (WrongNumberOfArguments, ValueError) as err:
                self._ros_node.get_logger().error(f'While parsing args for "--pos": {err}')
                self._ros_node.get_logger().warn(f'Assigning random spawn poses instead')
                input_dict['spawn_poses'] = self.get_randomized_spawn_poses(input_dict['ids'])
            finally:
                del input_dict['pos']

        elif 'pos-file' in input_dict.keys():
            try:
                input_dict['spawn_poses'] = self.get_spawn_poses_from_file(input_dict['pos-file'][0], input_dict['ids'])
            except (FileNotFoundError, SuffixError, FormattingError, WrongNumberOfArguments, ValueError) as err:
                self._ros_node.get_logger().error(f'While parsing args for "--pos-file": {err}')
                self._ros_node.get_logger().warn(f'Assigning random spawn poses instead')
                input_dict['spawn_poses'] = self.get_randomized_spawn_poses(input_dict['ids'])
            finally:
                del input_dict['pos-file']

        else:
            input_dict['spawn_poses'] = self.get_randomized_spawn_poses(input_dict['ids'])

        if 'name' in input_dict.keys():
            for ID in input_dict['ids']:
                input_dict['names'].append(str(input_dict['name'][0]) + str(ID))
            del input_dict['name']
        else:
            for ID in input_dict['ids']:
                input_dict['names'].append(str(self._default_robot_name) + str(ID))

        assert isinstance(input_dict['ids'], list) and len(input_dict['ids']) > 0, 'No vehicle ID assigned'
        assert input_dict['model'] is not None, 'Model not specified'
        assert isinstance(input_dict['names'], list) and len(input_dict['names']) == len(
            input_dict['ids']), f'Invalid vehicle names {input_dict["names"]}'
        assert isinstance(input_dict['spawn_poses'], dict) and len(input_dict['spawn_poses'].keys()) == len(
            input_dict['ids']), f'Invalid spawn poses {input_dict["spawn_poses"]}'

        return input_dict

    # #}

    # #{ assign_free_id(self)
    def assign_free_id(self):
        '''
        Assign an unused ID in range <0, 255>
        :return: unused ID for a robot (int)
        :raise NoFreeIDAvailable: if max vehicle count has been reached
        '''
        for i in range(0, 256):  # 255 is a hard limit of px4 sitl
            if i not in self.assigned_ids:
                self._ros_node.get_logger().info(f'Assigned free ID "{i}" to a robot')
                return i
        raise NoFreeIDAvailable('Cannot assign a free ID')

    # #}

    # #{ get_spawn_poses_from_file(self, filename, ids)
    def get_spawn_poses_from_file(self, filename, ids):
        '''
        Parses an input file and extracts spawn poses for vehicles. The file must be either ".csv" or ".yaml"

        CSV files have to include one line per robot, formatting: X, Y, Z, HEADING
        YAML files have to include one block per robot, formatting:
        block_header: # not used
            id: int
            x: float
            y: float
            z: float
            heading: float


        The file must contain spawn poses for all vehicles
        :param fileame: full path to a file
        :param ids: a list of ints containing unique vehicle IDs
        :return: a dict in format {id: {'x': pos_x, 'y', pos_y, 'z': pos_z, 'heading': pos_heading}}

        Raises:
        FileNotFoundError - if filename does not exist
        FormattingError - if the csv or yaml file does not match the expected structure
        SuffixError - filename has other suffix than ".csv" or ".yaml"
        WrongNumberOfArguments - number of poses defined in the file does not match the number of ids
        ValueError - spawn poses are not numbers
        '''

        self._ros_node.get_logger().info(f'Loading spawn poses from file "{filename}"')
        if not os.path.isfile(filename):
            raise FileNotFoundError(f'File "{filename}" does not exist!')

        spawn_poses = {}

        # #{ csv
        if filename.endswith('.csv'):
            array_string = list(csv.reader(open(filename)))
            for row in array_string:
                if (len(row) != 5):
                    raise FormattingError(
                        f'Incorrect data in file "{filename}"! Data in ".csv" file type should be in format [id, x, y, z, heading] (types: int, float, float, float, float)'
                    )
                if int(row[0]) in ids:
                    spawn_poses[int(row[0])] = {
                        'x': float(row[1]),
                        'y': float(row[2]),
                        'z': float(row[3]),
                        'heading': float(row[4])
                    }
        # #}

        # #{ yaml
        elif filename.endswith('.yaml'):
            dict_vehicle_info = yaml.safe_load(open(filename, 'r'))
            for item, data in dict_vehicle_info.items():
                if (len(data.keys()) != 5):
                    raise FormattingError(
                        f'Incorrect data in file "{filename}"! Data  in ".yaml" file type should be in format \n uav_name: \n\t id: (int) \n\t x: (float) \n\t y: (float) \n\t z: (float) \n\t heading: (float)'
                    )

                if int(data['id']) in ids:
                    spawn_poses[data['id']] = {
                        'x': float(data['x']),
                        'y': float(data['y']),
                        'z': float(data['z']),
                        'heading': float(data['heading'])
                    }
        # #}

        else:
            raise SuffixError(f'Incorrect file type! Suffix must be either ".csv" or ".yaml"')

        if len(spawn_poses.keys()) != len(ids) or set(spawn_poses.keys()) != set(ids):
            raise WrongNumberOfArguments(f'File "{filename}" does not specify poses for all robots!')

        self._ros_node.get_logger().info(f'Spawn poses returned: {spawn_poses}')
        return spawn_poses

    # #}

    # #{ get_spawn_poses_from_args(self, pos_args, ids)
    def get_spawn_poses_from_args(self, pos_args, ids):
        '''
        Parses the input args extracts spawn poses for vehicles.
        If more vehicles are spawned at the same time, the given pose is used for the first vehicle.
        Additional vehicles are spawned with an offset of {config param: gazebo_models/spacing} meters in X

        :param pos_args: a list of 4 numbers [x,y,z,heading]
        :param ids: a list of ints containing unique vehicle IDs
        :return: a dict in format {id: {'x': pos_x, 'y', pos_y, 'z': pos_z, 'heading': pos_heading}}

        Raises:
        WrongNumberOfArguments - pos_args does not contain exactly 4 values
        ValueError - input cannot be converted into numbers
        '''
        spawn_poses = {}
        if len(pos_args) != 4:
            raise WrongNumberOfArguments(f'Expected exactly 4 args after keyword "--pos", got {len(pos_args)}')

        x = float(pos_args[0])
        y = float(pos_args[1])
        z = float(pos_args[2])
        heading = float(pos_args[3])

        spawn_poses[ids[0]] = {'x': x, 'y': y, 'z': z, 'heading': heading}

        if len(ids) > 1:
            self._ros_node.get_logger().warn(
                f'Spawning more than one vehicle with "--pos". Each additional vehicle will be offset by {self._model_spacing} meters in X'
            )
            for i in range(len(ids)):
                x += self._model_spacing
                spawn_poses[ids[i]] = {'x': x, 'y': y, 'z': z, 'heading': heading}

        self._ros_node.get_logger().info(f'Spawn poses returned: {spawn_poses}')
        return spawn_poses

    # #}

    # #{ get_randomized_spawn_poses(self, ids)
    def get_randomized_spawn_poses(self, ids):
        '''
        Creates randomized spawn poses for all vehicles.
        The poses are generated with spacing defined by config param: gazebo_models/spacing
        Height is always set to 0.3

        :param ids: a list of ints containing unique vehicle IDs
        :return: a dict in format {id: {'x': pos_x, 'y', pos_y, 'z': pos_z, 'heading': pos_heading}}
        '''
        spawn_poses = {}

        circle_diameter = 0.0
        total_positions_in_current_circle = 0
        angle_increment = 0
        remaining_positions_in_current_circle = 1
        circle_perimeter = math.pi * circle_diameter
        random_angle_offset = 0
        random_x_offset = round(random.uniform(-self._model_spacing, self._model_spacing), 2)
        random_y_offset = round(random.uniform(-self._model_spacing, self._model_spacing), 2)

        for ID in ids:
            if remaining_positions_in_current_circle == 0:
                circle_diameter = circle_diameter + self._model_spacing
                circle_perimeter = math.pi * circle_diameter
                total_positions_in_current_circle = math.floor(circle_perimeter / self._model_spacing)
                remaining_positions_in_current_circle = total_positions_in_current_circle
                angle_increment = (math.pi * 2) / total_positions_in_current_circle
                random_angle_offset = round(random.uniform(-math.pi, math.pi), 2)

            x = round(
                math.sin(angle_increment * remaining_positions_in_current_circle + random_angle_offset) *
                circle_diameter, 2) + random_x_offset
            y = round(
                math.cos(angle_increment * remaining_positions_in_current_circle + random_angle_offset) *
                circle_diameter, 2) + random_y_offset
            z = 0.3
            heading = round(random.uniform(-math.pi, math.pi), 2)
            remaining_positions_in_current_circle = remaining_positions_in_current_circle - 1
            spawn_poses[ID] = {'x': x, 'y': y, 'z': z, 'heading': heading}

        self._ros_node.get_logger().info(f'Spawn poses returned: {spawn_poses}')
        return spawn_poses

    # #}

    # #{ check_user_request(self, params_dict)
    def check_user_request(self, params_dict) -> bool:
        core_keys = ["help", "model", "ids", "names", "spawn_poses"]
        user_cmds = []
        for key, _ in params_dict.items():
            if key in core_keys:
                continue
            user_cmds.append(key)

        valid_cmds = []
        try:
            template_wrapper = self._jinja_templates[params_dict["model"]]
            for name, component in template_wrapper.components.items():
                valid_cmds.append(component.keyword)
        except ValueError:
            return False

        for cmd in user_cmds:
            if cmd not in valid_cmds:
                return False

        return True

    # #}
