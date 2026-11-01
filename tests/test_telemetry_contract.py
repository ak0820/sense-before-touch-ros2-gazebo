"""Lightweight contract checks for telemetry fields and dashboard source files."""
from pathlib import Path
import ast

ROOT = Path(__file__).resolve().parents[1]

def test_dashboard_files():
    html = (ROOT / 'scripts/dashboard/index.html').read_text()
    controller = (ROOT / 'sense_before_touch_ros2/sense_before_touch_ros2/controller.py').read_text()
    assert '/api/telemetry' in html
    assert 'synthetic multispectral' in html
    assert '/rescue_ai/telemetry' in controller
    assert 'publish_telemetry' in controller
    assert 'last_confidence' in controller
    ast.parse(controller)
    ast.parse((ROOT / 'scripts/ai_dashboard.py').read_text())
