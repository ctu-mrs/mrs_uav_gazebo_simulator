import pathlib
import jinja2
import math


class MacroCollector(jinja2.visitor.NodeVisitor):

    def __init__(self):
        self.names = []

    def visit_Macro(self, node: jinja2.nodes.Macro):
        self.names.append(node.name)


class JinjaLoader():

    def __init__(self, resource_paths):
        self._resource_paths = resource_paths

    def _make_env(self):
        env = jinja2.Environment(loader=jinja2.FileSystemLoader(self._resource_paths), autoescape=False)
        env.globals['math'] = math
        return env

    def render_macro_file(self, template_path: pathlib.Path, macro_name: str, **ctx) -> str:
        env = self._make_env()
        try:
            tmpl = env.get_template(template_path)
            spawner_keyword = self._get_spawner_keyword(env, template_path)
            ctx["spawner_args"][spawner_keyword] = {"update_rate": 100, "noise": 0.0}
        except jinja2.TemplateNotFound as e:
            raise AssertionError(f"Template not found: {template_path}") from e
        # create a module object where macros are callables
        module = tmpl.make_module(ctx)
        macro = getattr(module, macro_name)
        if macro is None:
            raise AssertionError(f"Macro '{macro_name}' not found in {template_path}")
        return macro(**ctx)

    def render_drone_file(self, template_path: pathlib.Path, **ctx):
        env = self._make_env()
        try:
            tmpl = env.get_template(template_path)
        except jinja2.TemplateNotFound as e:
            raise AssertionError(f"Template not found: {template_path}") from e

        try:
            tmpl.render(ctx)
        except jinja2.TemplateRuntimeError as e:
            raise AssertionError(f"Template cannot be rendered") from e

    def _get_spawner_keyword(self, env: jinja2.Environment, template_name: str) -> str | None:
        """Return the string literal assigned to `spawner_keyword` in a template, or None."""
        src, _filename, _ = env.loader.get_source(env, template_name)
        ast = env.parse(src)

        class KWVisitor(jinja2.visitor.NodeVisitor):

            def __init__(self):
                self.value = None

            def visit_Assign(self, node: jinja2.nodes.Assign):
                if (isinstance(node.target, jinja2.nodes.Name) and node.target.name == "spawner_keyword"
                        and isinstance(node.node, jinja2.nodes.Const) and isinstance(node.node.value, str)):
                    self.value = node.node.value

        v = KWVisitor()
        v.visit(ast)
        return v.value

    def _macros_in_template(self, env: jinja2.Environment, template_name: str) -> list[str]:
        src, _filename, _uptodate = env.loader.get_source(env, template_name)
        ast = env.parse(src)
        collector = MacroCollector()
        collector.visit(ast)
        return collector.names

    def get_template_to_macros(self, TEMPLATES_GROUP_PATH) -> dict:
        env = self._make_env()
        camera_templates = [name for name in env.list_templates() if name.startswith(TEMPLATES_GROUP_PATH)]
        return {name: self._macros_in_template(env, name) for name in camera_templates}

    def extract_macros_and_keyword_from_template(self, template: str) -> list[str]:
        env = self._make_env()
        spawner_key = self._get_spawner_keyword(env, template)
        macro = self._macros_in_template(env, template)
        return [macro, spawner_key]

    def get_templates_from_group(self, group: str) -> list[str]:
        env = self._make_env()
        return [name for name in env.list_templates() if name.startswith(group)]
