import ast
import importlib.util
import math
from pathlib import Path
import re
import subprocess
import sys
import textwrap
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parent.parent
ROBOT = ROOT / 'src/robot_description'
WORLD = ROOT / 'src/smart_community_gazebo'


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Record:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class LaunchConfiguration(Record):
    def perform(self, context):
        return context.get(self.args[0], self.kwargs.get('default'))


class LaunchDescription:
    def __init__(self, actions):
        self.actions = actions


class FakeLaunch:
    def __enter__(self):
        modules = {}
        for name in ('ament_index_python', 'ament_index_python.packages', 'launch',
                     'launch.actions', 'launch.conditions', 'launch.substitutions',
                     'launch.launch_description_sources', 'launch_ros',
                     'launch_ros.actions', 'launch_ros.parameter_descriptions', 'xacro'):
            modules[name] = types.ModuleType(name)
        modules['ament_index_python.packages'].get_package_share_directory = lambda name: str(
            ROBOT if name == 'robot_description' else WORLD if name == 'smart_community_gazebo'
            else ROOT / 'external' / name)
        modules['launch'].LaunchDescription = LaunchDescription
        for group, names in {
            'launch.actions': ['DeclareLaunchArgument', 'IncludeLaunchDescription',
                               'SetEnvironmentVariable', 'TimerAction', 'OpaqueFunction'],
            'launch.conditions': ['IfCondition', 'UnlessCondition'],
            'launch.substitutions': ['PythonExpression'],
            'launch.launch_description_sources': ['PythonLaunchDescriptionSource'],
            'launch_ros.actions': ['Node'],
            'launch_ros.parameter_descriptions': ['ParameterValue'],
        }.items():
            for name in names:
                setattr(modules[group], name, type(name, (Record,), {}))
        modules['launch.substitutions'].LaunchConfiguration = LaunchConfiguration
        modules['xacro'].process_file = lambda *a, **kw: types.SimpleNamespace(
            toprettyxml=lambda **kwargs: '<robot name="test"/>')
        self.patcher = patch.dict(sys.modules, modules)
        self.patcher.start()
        return self

    def __exit__(self, *args):
        self.patcher.stop()


def description(name):
    return load(ROBOT / 'launch' / name).generate_launch_description().actions


def includes(actions, filename):
    return [a for a in actions if a.__class__.__name__ == 'IncludeLaunchDescription'
            and Path(a.args[0].args[0]).name == filename]


class CandidateTests(unittest.TestCase):
    def test_python_and_xml_syntax(self):
        for path in ROOT.rglob('*.py'):
            if '.git' not in path.parts:
                with self.subTest(path=path.relative_to(ROOT)):
                    ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
        for path in [*ROOT.glob('src/*/package.xml'), ROBOT / 'urdf/robot.xacro',
                     *WORLD.glob('worlds/*.sdf')]:
            ET.parse(path)

    def test_map_pair_exactly_matches_user_branch(self):
        for name in ('community.yaml', 'community.pgm'):
            path = f'src/smart_community_gazebo/maps/{name}'
            upstream = subprocess.check_output(['git', '-C', str(ROOT), 'show',
                f'a5656441c90827ef22ad64c7363dee55ba987e48:{path}'])
            self.assertEqual((ROOT / path).read_bytes(), upstream)
        self.assertEqual(yaml.safe_load((WORLD / 'maps/community.yaml').read_text(encoding='utf-8'))['origin'],
                         [-0.344, -0.360, 0])

    def test_all_yaml_files_parse(self):
        for path in ROOT.rglob('*.yaml'):
            with self.subTest(path=path.relative_to(ROOT)):
                yaml.safe_load(path.read_text(encoding='utf-8-sig'))

    def test_install_union_and_unique_dependencies(self):
        cmake = (ROBOT / 'CMakeLists.txt').read_text(encoding='utf-8')
        scripts = re.findall(r'^\s+(scripts/[^\s)]+)', cmake, re.M)
        self.assertEqual(len(scripts), len(set(scripts)))
        for script in scripts:
            self.assertTrue((ROBOT / script).is_file(), script)
        for script in ('scripts/waypoint_patrol.py', 'scripts/vision_detector.py',
                       'scripts/aic_v5_patrol.py', 'scripts/aic_coverage_mapping.py'):
            self.assertIn(script, scripts)
        root = ET.parse(ROBOT / 'package.xml').getroot()
        deps = [e.text for e in root if e.tag in ('depend', 'exec_depend')]
        self.assertEqual(len(deps), len(set(deps)))
        self.assertTrue({'cv_bridge', 'nav2_msgs', 'python3-yaml',
                         'nav2_simple_commander', 'python3-opencv'} <= set(deps))

    def test_teammate_waypoints_are_transformed_and_keep_metadata(self):
        source = ast.parse((ROBOT / 'scripts/waypoint_patrol.py').read_text(encoding='utf-8'))
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_load_route')
        route_module = load(ROBOT / 'scripts/aic_v5_route.py')
        import os
        namespace = {'yaml': yaml, 'os': os, 'to_local': route_module.to_local}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<isolated loader>', 'exec'), namespace)
        route_path = WORLD / 'config/official_route.yaml'
        fake = types.SimpleNamespace(route_file=str(route_path),
             get_logger=lambda: types.SimpleNamespace(info=lambda *a: None))
        with FakeLaunch():
            converted = namespace['_load_route'](fake)
        raw = yaml.safe_load(route_path.read_text(encoding='utf-8'))['route']
        self.assertEqual(len(converted), 22)
        self.assertAlmostEqual(converted[0]['x'], 0)
        self.assertAlmostEqual(converted[0]['y'], 0)
        self.assertAlmostEqual(converted[0]['yaw'], 0)
        anchor = yaml.safe_load((WORLD / 'config/official_route.yaml').read_text(encoding='utf-8'))['route'][0]
        for actual, original in zip(converted, raw):
            self.assertEqual(actual['name'], original['name'])
            self.assertEqual(actual['action'], original['action'])
            expected = route_module.to_local(original, anchor)
            for key in ('x', 'y', 'yaw'):
                self.assertAlmostEqual(actual[key], expected[key])
        self.assertAlmostEqual(converted[1]['x'], 1.264, places=3)
        self.assertAlmostEqual(converted[1]['y'], 0, places=3)
        v5_raw = yaml.safe_load((WORLD / 'config/official_route.yaml').read_text(encoding='utf-8'))['route']
        self.assertEqual(len(v5_raw), 22)
        self.assertTrue({'A_BOTTOM_VIEW', 'B_TOP_VIEW', 'TURN_UP_2'} <= {p['name'] for p in v5_raw})
        stops = route_module.planned_stops(route_module.load_points(route_path))
        self.assertEqual(len(stops), 23)
        self.assertEqual([p['name'] for p in stops if p['name'] not in ('TOP_STAGING', 'MID_STAGING')],
                         [p['name'] for p in raw[1:]])
        self.assertEqual({p['name'] for p in stops if 'observation_world_yaw' in p},
                         {p['name'] for p in raw if p['action'].startswith(('view', 'read_plate'))})
        left_view = next(p for p in stops if p['name'] == 'A_LEFT_VIEW')
        self.assertAlmostEqual(left_view['yaw'], math.pi / 2)
        self.assertAlmostEqual(left_view['observation_world_yaw'], 0)

    def test_bringup_preserves_team_defaults_and_conditions(self):
        with FakeLaunch():
            actions = description('bringup.launch.py')
            defaults = {a.args[0]: a.kwargs.get('default_value') for a in actions
                        if a.__class__.__name__ == 'DeclareLaunchArgument'}
            self.assertEqual(defaults['start_navigation'], 'true')
            self.assertEqual(defaults['robot_scale'], '0.5')
            for name in ('localization_launch.py', 'navigation_launch.py'):
                condition = includes(actions, name)[0].kwargs['condition']
                self.assertEqual(condition.args[0].args[0], 'start_navigation')
            spawn = includes(actions, 'spawn_robot.launch.py')[0]
            self.assertEqual(dict(spawn.kwargs['launch_arguments'])['scale'].args[0], 'robot_scale')

    def test_v5_v3_and_slam_wrappers_disable_embedded_nav(self):
        with FakeLaunch():
            for name in ('aic_v5_navigation.launch.py', 'navigation.launch.py',
                         'aic_mapping_demo.launch.py', 'aic_full_mission.launch.py'):
                with self.subTest(name=name):
                    actions = description(name)
                    embed = includes(actions, 'bringup.launch.py')
                    self.assertEqual(len(embed), 1)
                    args = dict(embed[0].kwargs['launch_arguments'])
                    self.assertEqual(args['start_navigation'], 'false')
                    self.assertEqual(args['start_traffic_detector'], 'false')
                    self.assertEqual(args['robot_scale'], '1.0')
                    if 'navigation' in name:
                        self.assertEqual(len(includes(actions, 'localization_launch.py')), 1)
                        self.assertEqual(len(includes(actions, 'nav_core_v3_1.launch.py')), 1)

    def test_spawn_scale_reaches_xacro(self):
        with FakeLaunch():
            module = load(ROBOT / 'launch/spawn_robot.launch.py')
            with patch('xacro.process_file', return_value=types.SimpleNamespace(
                    toprettyxml=lambda **kw: '<robot/>')) as process, \
                 patch.object(module.subprocess, 'run', return_value=types.SimpleNamespace(stdout='<sdf/>')):
                for scale in ('0.5', '1.0'):
                    actions = module.spawn_actions({'scale': scale})
                    self.assertEqual(process.call_args.kwargs['mappings'], {'scale': scale})
                    self.assertEqual(len(actions), 2)
                with self.assertRaises(ValueError):
                    module.spawn_actions({'scale': '2.0'})

    def test_real_xacro_geometry_when_available(self):
        if importlib.util.find_spec('xacro') is None:
            self.skipTest('Real xacro expansion runs in the Ubuntu ROS environment')
        import xacro
        for scale in (0.5, 1.0):
            document = xacro.process_file(str(ROBOT / 'urdf/robot.xacro'), mappings={'scale': str(scale)})
            root = ET.fromstring(document.toxml())
            box = root.find("./link[@name='base_link']/collision/geometry/box")
            size = [float(v) for v in box.attrib['size'].split()]
            for actual, base in zip(size, (0.32, 0.24, 0.10)):
                self.assertAlmostEqual(actual, base * scale)

    def test_green_requires_four_distinct_recent_frames(self):
        with patch.object(sys, 'path', [str(ROBOT / 'scripts'), *sys.path]):
            module = load(ROBOT / 'scripts/aic_v5_patrol.py')
        floor = 10_000_000_000
        guard = module.PreparedGreenEvidence('top', floor)
        def observe(stamp, instant='GREEN'):
            value = dict(source='camera', gate='top', sim_stamp_ns=stamp, state='GREEN',
                         instant_state=instant, confidence=.92, instant_confidence=.92)
            return guard.observe(value, 100., 100.1, max(stamp, floor))
        self.assertFalse(observe(floor))
        self.assertFalse(observe(floor + 100_000_000))
        for _ in range(10):
            self.assertFalse(observe(floor + 100_000_000))
        self.assertEqual(guard.samples, 1)
        self.assertFalse(observe(floor + 200_000_000, 'RED'))
        self.assertEqual(guard.samples, 0)
        self.assertEqual([observe(floor + i * 100_000_000) for i in range(3, 7)],
                         [False, False, False, True])

    def dispatch_events(self, deny):
        tree = ast.parse((ROBOT / 'scripts/aic_v5_patrol.py').read_text(encoding='utf-8'))
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        travel = next(n for n in main.body if isinstance(n, ast.FunctionDef) and n.name == 'travel')
        code = 'def factory():\n    moving = False\n' + textwrap.indent(ast.unparse(travel), '    ') + '\n    return travel\n'
        events = []
        class DispatchReached(Exception):
            pass
        class GreenDenied(Exception):
            pass
        class Nav:
            def followPath(self, path):
                events.append('dispatch')
                raise DispatchReached()
        def prepare(a, sequence):
            events.append('prepare')
            return types.SimpleNamespace(poses=[1]), None
        def reconfirm(gate):
            events.append('confirm')
            if deny:
                raise GreenDenied()
        scope = dict(use_unified_follow=lambda a, seq: True,
                     segment_check=lambda *a: ({}, []), lateral_margin_check=lambda *a, **k: ({}, []),
                     grid=None, health=lambda **k: None, log=lambda *a, **k: None,
                     args=types.SimpleNamespace(mode='patrol'), centerline_follow_path=prepare,
                     reconfirm_green_after_preparation=reconfirm, nav=Nav())
        exec(code, scope)
        run = scope['factory']()
        with self.assertRaises(GreenDenied if deny else DispatchReached):
            run([{'name': 'TOP_AFTER_LIGHT'}], {'name': 'TOP_STAGING'}, gate='top')
        return events

    def test_dispatch_follows_preparation_and_fresh_green_confirmation(self):
        self.assertEqual(self.dispatch_events(False), ['prepare', 'confirm', 'dispatch'])

    def test_no_motion_dispatch_if_green_confirmation_fails(self):
        self.assertEqual(self.dispatch_events(True), ['prepare', 'confirm'])

    def test_matching_costmap_sizes_and_safe_mapping_default(self):
        small = yaml.safe_load((ROBOT / 'config/nav2_params.yaml').read_text(encoding='utf-8'))
        full = yaml.safe_load((ROBOT / 'config/nav2_params_aic_v5.yaml').read_text(encoding='utf-8'))
        for kind in ('local_costmap', 'global_costmap'):
            self.assertEqual(small[kind][kind]['ros__parameters']['robot_radius'], 0.11)
            footprint = yaml.safe_load(full[kind][kind]['ros__parameters']['footprint'])
            self.assertGreaterEqual(max(abs(y) for x, y in footprint), 0.19)
        with FakeLaunch():
            actions = description('mapping.launch.py')
            explorer = next(a for a in actions if a.__class__.__name__ == 'DeclareLaunchArgument'
                            and a.args[0] == 'explorer')
            self.assertEqual(explorer.kwargs['default_value'], 'explore_mapping.py')
        init = (ROOT / 'nav_start.sh').read_text(encoding='utf-8')
        self.assertNotIn('/home/biao', init)
        self.assertIn('initial_pose.py', init)


if __name__ == '__main__':
    unittest.main(verbosity=2)
