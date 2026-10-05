"""실행기 정적 구조 검사만 수행한다. import·실행·ROS·TCP·모의 실기 없음."""
import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / 'src/can_powerpack/scripts/ph_run_profile.py'


def test_runner_compiles_without_executing():
    compile(SOURCE.read_text(), str(SOURCE), 'exec')


def test_no_ros_publisher_or_hardware_import_at_module_scope():
    tree = ast.parse(SOURCE.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr not in ('create_publisher', 'publish', 'set_parameters')
    imports = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    for node in imports:
        names = [n.name for n in node.names] if isinstance(node, ast.Import) else [node.module]
        assert not any(n in ('rclpy', 'actuator_map', 'valve_deadzone', 'socket') for n in names)


def test_dry_run_returns_before_hardware_entry():
    tree = ast.parse(SOURCE.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    branch = next(n for n in main.body if isinstance(n, ast.If)
                  and isinstance(n.test, ast.Attribute) and n.test.attr == 'dry_run')
    assert isinstance(branch.body[0], ast.Return)
    call = next(n for n in ast.walk(main) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name) and n.func.id == 'run')
    assert call.lineno > branch.lineno
