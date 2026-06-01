# pylint: disable=protected-access,redefined-outer-name
import importlib.util
import sys
import types
from pathlib import Path

import pytest


class _DummyNotification:
    def __init__(self, message=""):
        self.message = message

    def update(self, _delta):
        return None

    def draw(self, _ctx):
        return None


class _DummyMenu:
    def __init__(self, *_args, **_kwargs):
        self.is_animating = "none"

    def update(self, _delta):
        return None

    def draw(self, _ctx):
        return None

    def _cleanup(self):
        return None


class _DummyTextDialog:
    def __init__(self, *_args, **_kwargs):
        self.text = ""

    def _cleanup(self):
        return None

    def draw(self, _ctx):
        return None


class _DummyButtons:
    def __init__(self, *_args):
        self._pressed = set()

    def get(self, button):
        return button in self._pressed

    def clear(self):
        self._pressed.clear()


class _DummyPin:
    IN = 0
    OUT = 1
    PULL_UP = 2
    IRQ_FALLING = 3

    def __init__(self, *_args, **_kwargs):
        self._value = 1

    def init(self, **_kwargs):
        return None

    def on(self):
        self._value = 1

    def off(self):
        self._value = 0

    def value(self, v=None):
        if v is None:
            return self._value
        self._value = v
        return self._value

    def irq(self, **_kwargs):
        return None


class _DummyPWM:
    def __init__(self, _pin, freq=0, duty_ns=0):
        self._freq = freq
        self._duty_ns = duty_ns

    def freq(self, value=None):
        if value is None:
            return self._freq
        self._freq = value

    def duty_ns(self, value):
        self._duty_ns = value


class _DummyEventBus:
    def on_async(self, *_args, **_kwargs):
        return None

    def remove(self, *_args, **_kwargs):
        return None

    def emit(self, *_args, **_kwargs):
        return None


class _DummyLeds(dict):
    def __setitem__(self, key, value):
        super().__setitem__(key, value)

    def write(self):
        return None


class _DummyTildagon:
    def __init__(self):
        self.leds = _DummyLeds()

    def set_led_power(self, _enabled):
        return None


def _load_xystage_module(monkeypatch):
    root = Path(__file__).resolve().parents[1]

    def _install(name, module):
        monkeypatch.setitem(sys.modules, name, module)
        return module

    ota_mod = types.ModuleType("ota")
    ota_mod.get_version = lambda: "v1.0.0"
    _install("ota", ota_mod)

    settings_mod = types.ModuleType("settings")
    store = {}
    settings_mod.get = lambda key, default=None: store.get(key, default)
    settings_mod.set = lambda key, value: (store.pop(key, None) if value is None else store.__setitem__(key, value))
    settings_mod.save = lambda: None
    _install("settings", settings_mod)

    _install("vfs", types.ModuleType("vfs"))

    display_mod = types.ModuleType("display")
    display_mod.hexagon = lambda *_args, **_kwargs: None
    _install("display", display_mod)

    app_components_mod = types.ModuleType("app_components")
    app_components_mod.Menu = _DummyMenu
    app_components_mod.TextDialog = _DummyTextDialog
    _install("app_components", app_components_mod)

    notification_mod = types.ModuleType("app_components.notification")
    notification_mod.Notification = _DummyNotification
    _install("app_components.notification", notification_mod)

    tokens_mod = types.ModuleType("app_components.tokens")
    tokens_mod.label_font_size = 16
    tokens_mod.twentyfour_pt = 24
    tokens_mod.clear_background = lambda _ctx: None
    tokens_mod.button_labels = lambda *_args, **_kwargs: None
    _install("app_components.tokens", tokens_mod)

    events_input_mod = types.ModuleType("events.input")
    events_input_mod.BUTTON_TYPES = {
        "CANCEL": "cancel",
        "CONFIRM": "confirm",
        "RIGHT": "right",
        "LEFT": "left",
        "UP": "up",
        "DOWN": "down",
    }
    events_input_mod.Button = str
    events_input_mod.Buttons = _DummyButtons
    events_input_mod.ButtonUpEvent = type("ButtonUpEvent", (), {})
    _install("events.input", events_input_mod)

    frontboards_twentyfour_mod = types.ModuleType("frontboards.twentyfour")
    frontboards_twentyfour_mod.BUTTONS = {}
    _install("frontboards.twentyfour", frontboards_twentyfour_mod)

    machine_mod = types.ModuleType("machine")
    machine_mod.PWM = _DummyPWM
    machine_mod.Timer = type("Timer", (), {})
    machine_mod.Pin = _DummyPin
    _install("machine", machine_mod)

    eventbus_mod = types.ModuleType("system.eventbus")
    eventbus_mod.eventbus = _DummyEventBus()
    _install("system.eventbus", eventbus_mod)

    hex_events_mod = types.ModuleType("system.hexpansion.events")
    hex_events_mod.HexpansionInsertionEvent = type("HexpansionInsertionEvent", (), {"__init__": lambda self, port=1: setattr(self, "port", port)})
    hex_events_mod.HexpansionRemovalEvent = type("HexpansionRemovalEvent", (), {"__init__": lambda self, port=1: setattr(self, "port", port)})
    _install("system.hexpansion.events", hex_events_mod)

    hex_header_mod = types.ModuleType("system.hexpansion.header")
    hex_header_mod.read_header = lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("no eeprom"))
    _install("system.hexpansion.header", hex_header_mod)

    class _DummyHexpansionConfig:
        def __init__(self, _port):
            self.pin = [_DummyPin(), _DummyPin(), _DummyPin(), _DummyPin()]
            self.ls_pin = [_DummyPin(), _DummyPin(), _DummyPin(), _DummyPin()]

    hex_config_mod = types.ModuleType("system.hexpansion.config")
    hex_config_mod.HexpansionConfig = _DummyHexpansionConfig
    _install("system.hexpansion.config", hex_config_mod)

    scheduler_mod = types.ModuleType("system.scheduler")
    scheduler_mod.scheduler = type("Scheduler", (), {"apps": []})()
    _install("system.scheduler", scheduler_mod)

    scheduler_events_mod = types.ModuleType("system.scheduler.events")
    scheduler_events_mod.RequestStopAppEvent = type("RequestStopAppEvent", (), {"__init__": lambda self, app: setattr(self, "app", app)})
    _install("system.scheduler.events", scheduler_events_mod)

    tildagon_mod = types.ModuleType("tildagonos")
    tildagon_mod.tildagonos = _DummyTildagon()
    _install("tildagonos", tildagon_mod)

    app_base_mod = types.ModuleType("app")

    class _BaseApp:
        def __init__(self):
            return None

        def minimise(self):
            return None

    app_base_mod.App = _BaseApp
    _install("app", app_base_mod)

    pkg_name = "xystage_testpkg"
    pkg_mod = types.ModuleType(pkg_name)
    pkg_mod.__path__ = [str(root)]
    _install(pkg_name, pkg_mod)

    utils_spec = importlib.util.spec_from_file_location(f"{pkg_name}.utils", root / "utils.py")
    utils_mod = importlib.util.module_from_spec(utils_spec)
    _install(f"{pkg_name}.utils", utils_mod)
    utils_spec.loader.exec_module(utils_mod)

    app_spec = importlib.util.spec_from_file_location(f"{pkg_name}.app", root / "app.py")
    app_mod = importlib.util.module_from_spec(app_spec)
    app_mod.__package__ = pkg_name
    _install(f"{pkg_name}.app", app_mod)
    app_spec.loader.exec_module(app_mod)
    return app_mod


@pytest.fixture
def xystage_module(monkeypatch):
    return _load_xystage_module(monkeypatch)


@pytest.fixture
def app_obj(xystage_module):
    obj = xystage_module.XYStageApp.__new__(xystage_module.XYStageApp)
    obj._settings = {
        "x_usteps_per_mm": types.SimpleNamespace(v=1280),
        "y_usteps_per_mm": types.SimpleNamespace(v=1280),
        "gcode_feed": types.SimpleNamespace(v=300),
        "logging": types.SimpleNamespace(v=False),
        "min_speed": types.SimpleNamespace(v=320),
        "max_speed": types.SimpleNamespace(v=32000),
    }
    obj._gcode_speed_limit = {"x": 300.0, "y": 300.0}
    obj._record_waypoints_mm = []
    obj.notification = None
    obj.current_state = None
    obj._led_mode = xystage_module.LED_MODE_IDLE
    obj._log_gcode = lambda _message: None
    obj._set_led_mode = lambda mode: setattr(obj, "_led_mode", mode)
    obj.set_menu = lambda name="main": setattr(obj, "_last_menu", name)
    return obj


def test_parse_move_line_with_feed(app_obj):
    entry = app_obj._parse_gcode_line("g01 x10.5 y-2.25 f600 ; comment", 7)
    assert entry["cmd"] == "G1"
    assert entry["line"] == 7
    assert entry["params"]["X"] == 10.5
    assert entry["params"]["Y"] == -2.25
    assert entry["params"]["F"] == 600.0


def test_parse_rejects_unsupported_argument(app_obj):
    with pytest.raises(ValueError, match=r"G1 bad arg Z"):
        app_obj._parse_gcode_line("G1 X1 Z2", 4)


def test_parse_rejects_duplicate_argument(app_obj):
    with pytest.raises(ValueError, match=r"duplicate X"):
        app_obj._parse_gcode_line("G1 X1 X2", 5)


def test_parse_allows_axis_only_homing(app_obj):
    entry = app_obj._parse_gcode_line("G28 X Y", 2)
    assert entry["cmd"] == "G28"
    assert entry["params"] == {"X": True, "Y": True}


def test_parse_rejects_non_finite_numbers(app_obj):
    with pytest.raises(ValueError, match=r"non-finite value"):
        app_obj._parse_gcode_line("G1 X1e309", 9)


class _StepperStub:
    def __init__(self, pos=0):
        self._pos = pos
        self.max_sps = []

    def get_pos(self, _delta):
        return self._pos

    def set_max_sps(self, value):
        self.max_sps.append(int(value))


def test_feed_based_speed_limits_are_clamped(app_obj):
    app_obj._stepperX = _StepperStub(pos=0)
    app_obj._stepperY = _StepperStub(pos=0)
    app_obj._gcode_speed_limit = {"x": 9999999.0, "y": 9999999.0}

    app_obj._set_axis_speed_limits_from_feed(9999999.0, {"x": 500000, "y": 250000})

    assert app_obj._stepperX.max_sps[-1] <= app_obj._settings["max_speed"].v
    assert app_obj._stepperY.max_sps[-1] <= app_obj._settings["max_speed"].v


def test_save_recording_writes_expected_nc(tmp_path, monkeypatch, xystage_module, app_obj):
    output_dir = tmp_path / "gcode"
    monkeypatch.setattr(xystage_module, "_GCODE_DIR", str(output_dir))
    app_obj._record_waypoints_mm = [(1.0, 2.0), (3.25, -4.5)]

    app_obj._save_recording_file("demo file")

    saved = output_dir / "demo_file.nc"
    assert saved.exists()
    lines = saved.read_text().splitlines()
    assert lines[1] == "G28"
    assert lines[2] == "G90"
    assert "G1 X1.000 Y2.000 F300" in lines
    assert "G1 X3.250 Y-4.500 F300" in lines
    assert lines.count("M0") == 2


def test_save_recording_requires_waypoints(tmp_path, monkeypatch, xystage_module, app_obj):
    output_dir = tmp_path / "gcode"
    monkeypatch.setattr(xystage_module, "_GCODE_DIR", str(output_dir))
    app_obj._record_waypoints_mm = []

    app_obj._save_recording_file("empty")

    assert not output_dir.exists()
    assert app_obj._led_mode == xystage_module.LED_MODE_INPUT
