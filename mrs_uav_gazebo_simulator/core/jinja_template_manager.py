import jinja2
import math
import os
import sys
import re
import xml.dom.minidom
import tempfile
import datetime

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from mrs_uav_gazebo_simulator.utils.component_wrapper import ComponentWrapper
from mrs_uav_gazebo_simulator.utils.template_wrapper import TemplateWrapper


# #{ filter_templates(template_name)
def filter_templates(template_name, suffix):
    '''Comparator used to load files with given suffix'''
    return template_name.endswith(suffix)
# #}


class JinjaTemplateManager():

    # #{ __init__(self, ros_node, resource_paths, template_suffix)
    def __init__(
        self,
        ros_node: Node,
        resource_paths: list[str],
        template_suffix: str,
    ):
        self._ros_node = ros_node
        self._jinja_env = self._configure_jinja2_environment(resource_paths)
        self._template_suffix = template_suffix

    # #}

    # #{ get_jinja_templates(self)
    def get_jinja_templates(self):
        try:
            self._jinja_templates = self._build_template_database()
            return self._jinja_templates
        except RecursionError as err:
            self._ros_node.get_logger().error(f'{err}')
            raise RuntimeError(f'{err}')

    # #}

    # #{ get_spawner_components_from_template(self, template)
    def _get_spawner_components_from_template(self, template):
        '''
        Builds a dict of spawner-compatible macros in a given template and their corresponding ComponentWrapper objects
        Does NOT check for macros imported from other templates
        :return a dict in format {macro name: component_wrapper.ComponentWrapper}
        '''
        with open(template.filename, 'r') as f:
            template_source = f.read()
            preprocessed_template = template_source.replace('\n', '')
            parsed_template = self._jinja_env.parse(preprocessed_template)
            macro_nodes = [node for node in parsed_template.find_all(jinja2.nodes.Macro)]
            spawner_components = {}
            for node in macro_nodes:
                spawner_keyword = None
                spawner_description = None
                spawner_default_args = None
                for elem in node.body:
                    if isinstance(elem, jinja2.nodes.Assign) and elem.target.name == 'spawner_description':
                        spawner_description = elem.node.value
                    if isinstance(elem, jinja2.nodes.Assign) and elem.target.name == 'spawner_default_args':
                        if isinstance(elem.node, jinja2.nodes.Const):
                            spawner_default_args = elem.node.value
                        elif isinstance(elem.node, jinja2.nodes.List):
                            spawner_default_args = []
                            for e in elem.node.items:
                                spawner_default_args.append(e.value)
                        elif isinstance(elem.node, jinja2.nodes.Dict):
                            spawner_default_args = {}
                            for pair in elem.node.items:
                                spawner_default_args[pair.key.value] = pair.value.value
                        else:
                            self._ros_node.get_logger().warn(
                                f'Unsupported param type "{type(elem.node)}" in template {template.filename}')
                    if isinstance(elem, jinja2.nodes.Assign) and elem.target.name == 'spawner_keyword':
                        spawner_keyword = elem.node.value
                if spawner_keyword is not None:
                    spawner_components[node.name] = ComponentWrapper(spawner_keyword, spawner_description,
                                                                     spawner_default_args)
            return spawner_components

    # #}

    # #{ render_sdf(self, spawner_args)
    def render_sdf(self, spawner_args):
        '''
        Renders a jinja template into a sdf, creates a formatted xml
        Input has to specify the template name in spawner_args['model']
        :param spawner_args: a dict to be passed into the template as variables, format {component_name (string): args (list or dict)}
        :return: content of the xml file as a string or None
        '''

        params = {'spawner_args': spawner_args}

        try:
            model_name = spawner_args['model']
        except KeyError:
            self._ros_node.get_logger().error(f'Cannot render template, model not specified')
            return

        try:
            template_wrapper = self._jinja_templates[model_name]
        except KeyError:
            self._ros_node.get_logger().error(f'Cannot render model "{model_name}". Template not found!')
            return

        self._ros_node.get_logger().info(
            f'Rendering model "{model_name}" using template {template_wrapper.jinja_template.filename}')

        context = template_wrapper.jinja_template.new_context(params)
        rendered_template = template_wrapper.jinja_template.render(context)
        try:
            root = xml.dom.minidom.parseString(rendered_template)
        except Exception as e:
            self._ros_node.get_logger().error(f'XML error: "{e}"')
            fd, filepath = tempfile.mkstemp(prefix='mrs_drone_spawner_' +
                                            datetime.datetime.now().strftime('%Y_%m_%d__%H_%M_%S_'),
                                            suffix='_DUMP_' + str(model_name) + '.sdf')
            with os.fdopen(fd, 'w') as output_file:
                output_file.write(rendered_template)
                self._ros_node.get_logger().info(f'Malformed XML for model {model_name} dumped to {filepath}')
            return

        ugly_xml = root.toprettyxml(indent='  ')

        # Remove empty lines
        pretty_xml = '\n'.join(line for line in ugly_xml.split('\n') if line.strip())

        return pretty_xml

    # #}

    # #{ _configure_jinja2_environment(self, resource_paths)
    def _configure_jinja2_environment(self, resource_paths: list[str]):
        '''Create a jinja2 environment and setup its variables'''
        env = jinja2.Environment(loader=jinja2.FileSystemLoader(resource_paths), autoescape=False)
        # Allows use of math module directly in the templates
        env.globals['math'] = math

        return env

    # #}

    # #{ _build_template_database(self)
    def _build_template_database(self):
        '''
        Generate a database of jinja2 templates available to the spawner
        Scans through all folders provided into the jinja2 environment for files with matching target suffix
        Recursively checks templates imported by templates, prevents recursion loops
        Returns a dictionary of template_wrapper.TemplateWrapper objects in format {template_name: template_wrapper.TemplateWrapper}
        '''

        template_wrappers = {}
        all_templates = self._get_all_templates()

        self._load_all_templates(all_templates, template_wrappers)

        self._reindex_imported_templates(template_wrappers)

        self._adding_components_from_depend(template_wrappers)

        callable_components = self._prune_components(all_templates, template_wrappers)

        for name, wrapper in template_wrappers.items():
            wrapper.components = callable_components[name]

        self._ros_node.get_logger().info('Template database built')
        self._ros_node.get_logger().info('Jinja templates loaded.')

        return template_wrappers

    # #}

    # #{ _load_all_templates(self, all_templates, template_wrappers):
    def _load_all_templates(self, all_templates, template_wrappers):
        self._ros_node.get_logger().info('Loading all templates')
        for name, template in all_templates:
            imports = self._get_template_imports(template)
            components = self._get_spawner_components_from_template(template)
            package_name = self._get_ros_package_name(template.filename)
            wrapper = TemplateWrapper(template, imports, components, package_name)
            template_wrappers[name] = wrapper

    # #}

    # #{ _get_all_templates(self)
    def _get_all_templates(self):
        '''
        Get all templates loaded by the given jinja environment
        :returns a list of tuples, consisting of (str_name, jinja2.Template)
        '''
        template_names = self._jinja_env.list_templates(
            filter_func=lambda template_name: filter_templates(template_name, self._template_suffix))

        templates = []
        for i, full_name in enumerate(template_names):
            self._ros_node.get_logger().info(f'\t({i+1}/{len(template_names)}): {full_name}')
            template_name = full_name.split(os.path.sep)[-1][:-(len(self._template_suffix))]
            templates.append((template_name, self._jinja_env.get_template(full_name)))
        return templates

    # #}

    # #{ _get_template_imports(self, jinja_template)
    def _get_template_imports(self, jinja_template):
        '''Returns a list of sub-templates imported by a given jinja_template'''
        with open(jinja_template.filename, 'r') as f:
            template_source = f.read()
            preprocessed_template = template_source.replace('\n', '')
            parsed_template = self._jinja_env.parse(preprocessed_template)
            import_names = [node.template.value for node in parsed_template.find_all(jinja2.nodes.Import)]
            imported_templates = []
            for i in import_names:
                template = self._jinja_env.get_template(i)
                imported_templates.append(template)
            return imported_templates

    # #}

    # #{ _get_ros_package_name(self, filepath)
    def _get_ros_package_name(self, filepath):
        '''Return the name of a ros package that contains a given filepath'''

        package_share_pattern = r'^(.*?share/[^/]+)'
        match_result = re.match(package_share_pattern, filepath)
        if match_result is not None:
            package_share_path = match_result.group(0)
        else:
            package_share_path = None

        package_name_pattern = r'share/([^/]+)'
        search_result = re.search(package_name_pattern, filepath)
        if search_result is not None:
            package_name = search_result.group(1)
        else:
            package_name = None

        if package_share_path is None or package_name is None:
            self._ros_node.get_logger().error(
                f'Package name or share path could not be determined from filepath "{filepath}"')
            return None

        share_path_from_ament_index = get_package_share_directory(package_name)

        # sanity check
        if share_path_from_ament_index != package_share_path:
            self._ros_node.get_logger().error(
                f'Share path for package "{package_name}" not registered in ament index. Is the resource package installed and sourced?'
            )
            return None

        return package_name

    # #}

    # #{ _reindex_imported_templates(self, template_wrappers)
    def _reindex_imported_templates(self, template_wrappers):
        self._ros_node.get_logger().info('Reindexing imported templates')
        for name, wrapper in template_wrappers.items():
            for i, it in enumerate(wrapper.imported_templates):
                if isinstance(it, TemplateWrapper):
                    continue
                for ww in template_wrappers.values():
                    if ww.jinja_template == it:
                        wrapper.imported_templates[i] = ww

    # #}

    # #{ _adding_components_from_depend(self, template_wrappers)
    def _adding_components_from_depend(self, template_wrappers):
        self._ros_node.get_logger().info('Adding available components from dependencies')
        for _, wrapper in template_wrappers.items():
            prev_limit = sys.getrecursionlimit()
            sys.setrecursionlimit(int(math.pow(len(template_wrappers), 2)))
            wrapper.components = self._get_accessible_components(wrapper, {})
            sys.setrecursionlimit(prev_limit)

    # #}

    # #{ _prune_components(self, all_templates, template_wrappers)
    def _prune_components(self, all_templates, template_wrappers):
        self._ros_node.get_logger().info('Pruning components to only include callables')
        callable_components = {}
        for name, template in all_templates:
            callable_components[name] = self._get_callable_components(template, template_wrappers[name].components)
        return callable_components

    # #}

    # #{ _get_callable_components(self, template)
    def _get_callable_components(self, template, accessible_components):
        '''
        Get all components that are actually called from a template
        :param template: a jinja template file
        :param accessible_components: a dict of macros accessible from this template (including imported modules)
        :returns a dictionary of callable components {macro_name: component_wrapper.ComponentWrapper}
        sorted alphabetically by keywords
        '''
        callable_components = {}
        with open(template.filename, 'r') as f:
            template_source = f.read()
            preprocessed_template = template_source.replace('\n', '')
            parsed_template = self._jinja_env.parse(preprocessed_template)
            call_nodes = [node for node in parsed_template.find_all(jinja2.nodes.Call)]
            callable_components = {}
            for node in call_nodes:
                if isinstance(node.node, jinja2.nodes.Getattr):
                    if node.node.attr in accessible_components.keys():
                        callable_components[node.node.attr] = accessible_components[node.node.attr]
                elif isinstance(node.node, jinja2.nodes.Name):
                    if node.node.name in accessible_components.keys():
                        callable_components[node.node.name] = accessible_components[node.node.name]
        return dict(sorted(callable_components.items(), key=lambda item: item[1].keyword))
    # #}

    # #{ _get_accessible_components(self, template_wrapper, all_components)
    def _get_accessible_components(self, template_wrapper, all_components):
        '''
        Recursive function to get all spawner-compatible components accessible from template_wrapper
        Includes components in imported sub-templates
        :param template_wrapper: template_wrapper.TemplateWrapper for which we want to load components
        :param all_components: a dict to which all found ComponentWrappers will be added
        :returns a dict of objects {macro name: component_wrapper.ComponentWrapper}
        '''
        all_components.update(template_wrapper.components)
        for i in template_wrapper.imported_templates:
            try:
                all_components.update(self._get_accessible_components(i, all_components))
            except RecursionError as err:
                raise RecursionError(
                    f'Cyclic import detected in file {template_wrapper.jinja_template.filename}. Fix your templates')
        return all_components
    # #}
