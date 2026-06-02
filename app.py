import asyncio
import os
import time
import machine
from math import isfinite
import ota
import settings
import vfs
from app_components.notification import Notification
from app_components.tokens import label_font_size, small_font_size, sixteen_pt, clear_background, button_labels
from app_components import Menu, TextDialog
from events.input import BUTTON_TYPES, Button, Buttons
from frontboards.twentyfour import BUTTONS
from machine import PWM, Pin
from system.eventbus import eventbus
from system.hexpansion.events import (HexpansionInsertionEvent,
                                      HexpansionRemovalEvent)
from system.hexpansion.config import HexpansionConfig
from system.hexpansion.util   import get_slots_by_vid_pid
from system.scheduler.events import (RequestStopAppEvent)
try:
    from system.patterndisplay.events import PatternDisable, PatternEnable
except ImportError:
    PatternDisable = None
    PatternEnable = None

from tildagonos import tildagonos

import app


_APP_VERSION = "1.0" # XYStage App Version Number


# Stepper Tester - Defaults
_STEPPER_MAX_SPEED          = 500*32     # steps per second
_STEPPER_MIN_SPEED          = 10*32      # steps per second
_STEPPER_MAX_ACCELERATION   = 100*32     # steps per second per update
_STEPPER_MAX_POSITION       = 3100       # steps from h/w endstop to s/w endstop at the other end
_STEPPER_STEP_PULSE_NS      = 2000       # minimum 1.9uS STEP pulse width for DRV8825
_PWM_SAFE_MODE_DEFAULT      = False      # prefer stable hardware retune path; fallback enables safe mode dynamically

# ESP32-S3 LEDC + GPIO matrix register layout (ESP-IDF soc/reg headers)
_ESP32S3_DR_REG_GPIO_BASE = 0x60004000
_ESP32S3_DR_REG_LEDC_BASE = 0x60019000

_ESP32S3_GPIO_FUNC0_OUT_SEL_CFG_REG = _ESP32S3_DR_REG_GPIO_BASE + 0x554
_ESP32S3_GPIO_FUNC_OUT_SEL_STRIDE = 4
_ESP32S3_GPIO_FUNC_OUT_SEL_MASK = 0x1FF
_ESP32S3_GPIO_NUM_MAX = 53

_ESP32S3_LEDC_LS_SIG_OUT0_IDX = 73
_ESP32S3_LEDC_LS_SIG_OUT7_IDX = 80

_ESP32S3_LEDC_LSCH0_CONF0_REG = _ESP32S3_DR_REG_LEDC_BASE + 0x0000
_ESP32S3_LEDC_LSCH_CONF_STRIDE = 0x14
_ESP32S3_LEDC_TIMER_SEL_MASK = 0x3
_ESP32S3_LEDC_LSCH_DUTY_REG_OFFSET = 0x08
_ESP32S3_LEDC_LSCH_CONF1_REG_OFFSET = 0x0C
_ESP32S3_LEDC_DUTY_RAW_MASK = 0x7FFFF
_ESP32S3_LEDC_DUTY_INT_PART_SHIFT = 4
_ESP32S3_LEDC_DUTY_START_BIT = 1 << 31
_ESP32S3_LEDC_DUTY_INC_BIT = 1 << 30
_ESP32S3_LEDC_DUTY_NUM_MASK = 0x3FF
_ESP32S3_LEDC_DUTY_NUM_SHIFT = 20
_ESP32S3_LEDC_DUTY_CYCLE_MASK = 0x3FF
_ESP32S3_LEDC_DUTY_CYCLE_SHIFT = 10
_ESP32S3_LEDC_DUTY_SCALE_MASK = 0x3FF
_ESP32S3_LEDC_DUTY_SCALE_SHIFT = 0

_ESP32S3_LEDC_LSTIMER0_CONF_REG = _ESP32S3_DR_REG_LEDC_BASE + 0x00A0
_ESP32S3_LEDC_LSTIMER_CONF_STRIDE = 0x08
_ESP32S3_LEDC_LSTIMER_DUTY_RES_MASK = 0xF
_ESP32S3_LEDC_LSTIMER_CLK_DIV_MASK = 0x3FFFF
_ESP32S3_LEDC_LSTIMER_CLK_DIV_SHIFT = 4
_ESP32S3_LEDC_LSTIMER_PARA_UP_BIT = 1 << 25
_ESP32S3_LEDC_LSTIMER_TICK_SEL_BIT = 1 << 24

_ESP32S3_LEDC_REF_TICK_HZ = 1_000_000
_ESP32S3_LEDC_APB_HZ = 80_000_000
_ESP32S3_LEDC_DIV_FRAC_BITS = 8
_ESP32S3_LEDC_DIV_MIN = 1 << _ESP32S3_LEDC_DIV_FRAC_BITS
_ESP32S3_LEDC_DIV_MAX = _ESP32S3_LEDC_LSTIMER_CLK_DIV_MASK
_ESP32S3_LEDC_DUTY_RES_MIN = 1
_ESP32S3_LEDC_DUTY_RES_MAX = 14

# Timings
_AUTO_REPEAT_MS = 200       # Time between auto-repeats, in ms
_AUTO_REPEAT_COUNT_THRES = 10 # Number of auto-repeats before increasing level
_AUTO_REPEAT_SPEED_LEVEL_MAX = 4  # Maximum level of auto-repeat speed increases
_AUTO_REPEAT_LEVEL_MAX = 3  # Maximum level of auto-repeat digit increases


# App states
STATE_INIT = -1
STATE_WARNING = 0
STATE_MENU = 1
STATE_XYSTAGE = 2
STATE_ERROR = 3          # Hexpansion error
STATE_MESSAGE = 4        # Message display
STATE_SETTINGS = 5       # Edit Settings
STATE_GCODE_FILES = 6    # List .nc files from /gcode
STATE_GCODE_REPLAY = 7   # Execute parsed GCODE commands
STATE_GCODE_RECORD = 8   # Manual movement + waypoint capture

# App states where user can minimise app
_MINIMISE_VALID_STATES = [STATE_WARNING, STATE_MENU, STATE_MESSAGE]

# Hexpansion constants
_XYSTAGE_HEXPANSION_SLOT = None  # Hexpansion slot for XYStage - if it does not have an EEPROM to be detected automatically
_XYSTAGE_VID = 0xCBCB
_XYSTAGE_PID = 0x6000
_JOYSTICK_HEXPANSION_SLOT = None # Hexpansion slot for Joystick - if it does not have an EEPROM to be detected automatically
_JOYSTICK_VID = 0xCBCB
_JOYSTICK_PID = 0x6001


# Dedicated Pins - to drive an external stepper driver
X_DIR = 1   # ls pin (LSB)
X_ENABLE = 0  # ls pin (LSA) - active low
X_ENDSTOP = 3  # hs pin (HSI) - switch to ground
X_STEP = 0  # hs pin (HSF)
Y_DIR = 3   # ls pin (LSD)
Y_ENABLE = 2  # ls pin (LSC) - active low
Y_ENDSTOP = 1  # hs pin (HSG) - switch to ground
Y_STEP = 2  # hs pin (HSH)

# Joystick switch inputs on second hexpansion HS pins (active low)
JOY_POS_X = 0  # HSF
JOY_NEG_X = 1  # HSG
JOY_POS_Y = 2  # HSH
JOY_NEG_Y = 3  # HSI

_MICRO_STEPS = 32  # We are using DRV8825 configured for 1/32 microstepping

_USABLE_X_PIXELS =  200
_USABLE_Y_PIXELS =  140
_WIDTH_DEFAULT   = (2000*_MICRO_STEPS)  # Driver configured for 1/32 steps
_HEIGHT_DEFAULT  = (2000*_MICRO_STEPS)  # Driver configured for 1/32 steps
_XRANGE_DEFAULT  = (2200*_MICRO_STEPS) # Driver configured for 1/32 steps
_YRANGE_DEFAULT  = (2000*_MICRO_STEPS) # Driver configured for 1/32 steps
POSITION_MATCH_TOLERANCE = 20
_GCODE_DIR = "/gcode"
_GCODE_EXTENSION = ".nc"
# GCODE replay UI settings
_GCODE_HISTORY_MAX = 4  # includes currently executing command
_GCODE_FUTURE_MAX = 2

_GCODE_HOMING_TIMEOUT_MIN_MS = 5000
_GCODE_HOMING_TIMEOUT_MAX_MS = 120000
_GCODE_HOMING_TIMEOUT_MARGIN_MS = 4000
_GCODE_HOMING_BACKOFF_MM = 5.0
_GCODE_HOMING_FINE_FEED_RATIO = 0.25
_GCODE_HOMING_FINE_FEED_MIN_MM_MIN = 20

_X_STEPS_PER_MM_DEFAULT = int(0.5 + (32.8 * _MICRO_STEPS))  # Driver configured for 1/32 steps, 20 steps per rotation, 0.6096 pitch leadscrew => 20/0.6096 = ~32.8 steps per mm
_Y_STEPS_PER_MM_DEFAULT = int(0.5 + (40.0 * _MICRO_STEPS))  # Driver configured for 1/32 steps, 20 steps per rotation, 0.5 pitch leadscrew => 20/0.5 = 40 steps per mm
_GCODE_DEFAULT_FEED_MM_MIN = 300

_LED_GREEN = (0, 255, 0)
_LED_RED = (255, 0, 0)
_LED_YELLOW = (255, 180, 0)
_LED_PURPLE = (180, 0, 255)
_LED_BLUE = (0, 0, 255)
_LED_OFF = (0, 0, 0)

LED_MODE_IDLE = "idle"
LED_MODE_ERROR = "error"
LED_MODE_BUSY = "busy"
LED_MODE_INPUT = "input"
LED_MODE_MOVING = "moving"

#Misceallaneous Settings
_LOGGING = True

# Menu Items
_main_menu_items = ["XYStage", "Home XY", "Run NC", "Record NC", "Settings", "About", "Exit"]

class XYStageApp(app.App):
    def __init__(self):
        super().__init__()
        # UI Button Controls
        self.button_states = Buttons(self)
        self.last_press: Button = BUTTON_TYPES["CANCEL"]
        self._auto_repeat_intervals = [ _AUTO_REPEAT_MS, _AUTO_REPEAT_MS//2, _AUTO_REPEAT_MS//4, _AUTO_REPEAT_MS//8, _AUTO_REPEAT_MS//16] # at the top end the loop is unlikley to cycle this fast
        self._auto_repeat: int = 0
        self._auto_repeat_count: int = 0
        self._auto_repeat_level: int = 0

        # UI Feature Controls
        self._refresh: bool = True
        self.notification: Notification = None
        self.error_message = []
        self.current_menu: str = None
        self.menu: Menu = None

        # Settings
        self._settings = {}
        self._settings['logging']       = MySetting(self._settings, _LOGGING, False, True)
        self._settings['width']         = MySetting(self._settings, _WIDTH_DEFAULT,  10, 100000)
        self._settings['height']        = MySetting(self._settings, _HEIGHT_DEFAULT, 10, 100000)
        self._settings['XRange']        = MySetting(self._settings, _XRANGE_DEFAULT, 10, 100000)
        self._settings['YRange']        = MySetting(self._settings, _YRANGE_DEFAULT, 10, 100000)
        self._settings['min_speed']     = MySetting(self._settings, _STEPPER_MIN_SPEED, 10, 10000)
        self._settings['max_speed']     = MySetting(self._settings, _STEPPER_MAX_SPEED, 10, 100000)
        self._settings['acceleration']  = MySetting(self._settings, _STEPPER_MAX_ACCELERATION,  1, 1000)
        self._settings['x_steps_per_mm'] = MySetting(self._settings, _X_STEPS_PER_MM_DEFAULT, 1, 100000)
        self._settings['y_steps_per_mm'] = MySetting(self._settings, _Y_STEPS_PER_MM_DEFAULT, 1, 100000)
        self._settings['mm_per_min']    = MySetting(self._settings, _GCODE_DEFAULT_FEED_MM_MIN, 1, 60000)

        self._edit_setting: int  = None
        self._edit_setting_value = None
        self.update_settings()   

        # Check what version of the Badge s/w we are running on
        try:
            ver = parse_version(ota.get_version())
            if ver is not None:
                if self._settings['logging'].v:
                    print(f"XYStage V{ver}")
                # Potential to do things differently based on badge s/w version
                # e.g. if ver < [1, 9, 0]:
        except:
            pass

        # Hexpansion related
        self._xystage_port: int | None  = _XYSTAGE_HEXPANSION_SLOT
        self._joystick_port: int | None = _JOYSTICK_HEXPANSION_SLOT

        self._xystage_config: HexpansionConfig | None = None
        self._joystick_config: HexpansionConfig | None = None

        self._joystick_pins = {}
        eventbus.on_async(HexpansionInsertionEvent, self._handle_hexpansion_insertion, self)
        eventbus.on_async(HexpansionRemovalEvent, self._handle_hexpansion_removal, self)

        # Motor Driver
        self._stepperX: Stepper | None = None
        self._stepperY: Stepper | None = None
        self.xystage = {}
        self.xystage['x'] = 0
        self.xystage['y'] = 0
        self._keep_alive_period: int = 500                     # ms (half the value used in hexdrive.py)  
        self._timeout_period: int = 60*60000                   # ms (60 minutes)        
        self._time_since_last_input = 0

        # GCODE runtime state
        self._gcode_files = []
        self._gcode_file_names = []
        self._gcode_commands = []
        self._gcode_current_index = 0
        self._gcode_current_command = None
        self._gcode_history = []
        self._gcode_wait_remaining_ms = None
        self._gcode_wait_button = False
        self._gcode_mode_absolute = True
        self._gcode_abort_requested = False
        self._gcode_move_active = False
        self._gcode_move_target = {'x': 0, 'y': 0}
        self._gcode_move_feed_mm_min = self._settings['mm_per_min'].v
        self._gcode_speed_limit = self._default_gcode_speed_limits_mm_min()
        self._gcode_selected_file = None
        self._gcode_error = None
        self._gcode_homing_axes = []
        self._gcode_homing_remaining_ms = None

        # GCODE recording runtime
        self._record_waypoints_mm = []
        self._record_capture_debounce = False
        self._filename_dialog: TextDialog = None

        # LED runtime state
        self._led_mode = LED_MODE_IDLE
        self._led_last_mode = None
        self._led_anim_phase = 0
        self._led_anim_elapsed = 0
        self._led_control_taken = False

        # Overall app state (controls what is displayed and what user inputs are accepted)
        self.current_state = STATE_INIT
        self.previous_state = self.current_state
        self._target = {}
        self._target['x'] = 0
        self._target['y'] = 0   
        self._take_led_control()
        if self._settings['logging'].v:
            print("XYStageApp:Init")


    ### ASYNC EVENT HANDLERS ###

    async def _handle_hexpansion_removal(self, event: HexpansionRemovalEvent):
        if event.port == self._xystage_port:
            self._xystage_port = None
            self._xystage_config = None
            self.current_state = STATE_WARNING
            self.notification = Notification("XY Stage Removed")
            if self._settings['logging'].v:
                print("XY Stage:Removed")            
        if event.port == self._joystick_port:
            self._joystick_port = None
            self._joystick_config = None
            self._joystick_pins = {}
            self.notification = Notification("Joystick Removed")
            if self._settings['logging'].v:
                print("Joystick:Removed")

    async def _handle_hexpansion_insertion(self, event: HexpansionInsertionEvent):
        self.check_port_for_hexpansions(event.port)


    ### HEXPANSION FUNCTIONS ###

    def check_port_for_hexpansions(self, port: int):
        # we currently ignore the requested port and use the new get_slots_by_vid_pid function to check for the presence of the hexpansions we are interested in
        if self._joystick_port is None:
            slots = get_slots_by_vid_pid(_JOYSTICK_VID, _JOYSTICK_PID)
            if slots:
                port = slots[0]
                if self._settings['logging'].v:
                    print(f"Joy:Found on port {port}")
                self._joystick_port = port

        if self._joystick_port is not None:
            self._configure_joystick_inputs()
        
        if self._xystage_port is None:
            slots = get_slots_by_vid_pid(_XYSTAGE_VID, _XYSTAGE_PID)
            if slots:
                port = slots[0]
                if self._settings['logging'].v:
                    print(f"XYStage:Found on port {port}")
                self._xystage_port = port

        if self._xystage_port is not None:
            self._ensure_steppers_ready()


    def update_settings(self):
        for s in self._settings:
            self._settings[s].v = settings.get(f"xystage.{s}", self._settings[s].d)


    ### MAIN APP CONTROL FUNCTIONS ###

    def update(self, delta: int):
        if self.notification:
            self.notification.update(delta)

        if self.current_state == STATE_INIT:
            # One Time initialisation
            self.check_port_for_hexpansions(None)
            if self._xystage_port is None:
                self.current_state = STATE_WARNING
            else:
                self.current_state = STATE_MENU
        
        self._update_main_application(delta)
        self._update_leds(delta)

        if self.current_state != self.previous_state:
            if self._settings['logging'].v:
                print(f"State: {self.previous_state} -> {self.current_state}")
            self.previous_state = self.current_state
            # something has changed - so worth redrawing
            self._refresh = True


    def _update_main_application(self, delta: int):
        if self.current_state == STATE_MENU or self.current_state == STATE_GCODE_FILES:
            if self.current_menu is None:
                self.set_menu("main" if self.current_state == STATE_MENU else "GCODE Files")
                self._refresh = True
            else:
                self.menu.update(delta)    
                if self.menu.is_animating != "none":
                    if self._settings['logging'].v:
                        print("Menu is animating")
                    self._refresh = True
                if self.current_state == STATE_GCODE_FILES:
                    self._set_led_mode(LED_MODE_INPUT)
        elif self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in _MINIMISE_VALID_STATES:
            self.button_states.clear()
            self._minimise_with_cleanup()

    ### XY Stage Application ###
        elif self.current_state == STATE_XYSTAGE:
            self._update_state_xystage(delta)
        elif self.current_state == STATE_GCODE_REPLAY:
            self._update_state_gcode_replay(delta)
        elif self.current_state == STATE_GCODE_RECORD:
            self._update_state_gcode_record(delta)
        elif self.current_state == STATE_ERROR:
            self._update_state_error(delta)

    ### Settings Capability ###
        elif self.current_state == STATE_SETTINGS:
            self._update_state_settings(delta)
    ### End of Update ###

    # XY Stage Control
    def _update_state_xystage(self, delta: int):
        if not self._ensure_steppers_ready(silent=True):
            self.current_state = STATE_MENU
            self.set_menu("main")
            self.notification = Notification("No Free Timers")
            self._set_led_mode(LED_MODE_ERROR)
            return

        self._update_live_position(delta)
        # Left/Right to adjust position
        pressed = False
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.button_states.clear()

            # if CONFIRM pressed then go to position 0,0
            pressed = True
            # if current position is not close to 0,0 then go to 0,0
            # check each of X & Y independently
            # if value is too high then apply -ve speed
            # if value is too low then apply +ve speed
            # subject to the min and max speed limits
            if self.xystage['x'] > (self._target['x'] + POSITION_MATCH_TOLERANCE):
                self._stepperX.speed(-self._stepperX.get_speed_from_disance((self._target['x']-self.xystage['x'])))
            elif self.xystage['x'] < (self._target['x'] - POSITION_MATCH_TOLERANCE):
                self._stepperX.speed(self._stepperX.get_speed_from_disance((self._target['x']-self.xystage['x'])))
            else:
                self._stepperX.speed(0)
            if self.xystage['y'] > (self._target['y'] + POSITION_MATCH_TOLERANCE):
                self._stepperY.speed(-self._stepperY.get_speed_from_disance((self._target['y']-self.xystage['y'])))
            elif self.xystage['y'] < (self._target['y'] - POSITION_MATCH_TOLERANCE):
                self._stepperY.speed(self._stepperY.get_speed_from_disance((self._target['y']-self.xystage['y'])))
            else:
                self._stepperY.speed(0)
            self._refresh = True
        else:
            pressed = self._apply_manual_movement(delta)

        if pressed:
            self._time_since_last_input = 0
        else:
            self._auto_repeat_clear()
            # non auto-repeating buttons
            if self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.button_states.clear()
                self._stepperX.enable(False)
                self._stepperY.enable(False)
                self.current_state = STATE_MENU
                self.set_menu("main")
                return            
            if self._refresh or self._time_since_last_input == 0:
                # still decelerating or first time through since buttons released
                self._refresh = True            
            else:
                self._time_since_last_input += delta                
                if self._time_since_last_input > self._timeout_period:
                    self._stepperX.stop()
                    self._stepperX.speed(0)
                    self._stepperX.enable(False)
                    self._stepperY.stop()
                    self._stepperY.speed(0)
                    self._stepperY.enable(False)                
                    self.current_state = STATE_MENU
                    self.set_menu("main")
                    self.notification = Notification("Stepper Timeout")
                    if self._settings['logging'].v:
                        print("Stepper:Timeout")          

        if self._refresh and self._settings['logging'].v and (self._stepperX.get_speed() != 0 or self._stepperY.get_speed() != 0):
            print(f"X:{self.xystage['x']} Y:{self.xystage['y']}")
        self._set_led_mode(LED_MODE_MOVING if self._is_stage_moving() else LED_MODE_IDLE)


    def _update_live_position(self, delta: int):
        self.xystage['x'] = self._stepperX.get_pos(delta) - self._settings['XRange'].v//2
        self.xystage['y'] = self._stepperY.get_pos(delta) - self._settings['YRange'].v//2


    def _read_joystick_inputs(self):
        pressed = {'+x': False, '-x': False, '+y': False, '-y': False}
        if not self._joystick_pins:
            return pressed

        try:
            pressed['+x'] = self._joystick_pins[JOY_POS_X].value() == 0
            pressed['-x'] = self._joystick_pins[JOY_NEG_X].value() == 0
            pressed['+y'] = self._joystick_pins[JOY_POS_Y].value() == 0
            pressed['-y'] = self._joystick_pins[JOY_NEG_Y].value() == 0
        except Exception as e:
            if self._settings['logging'].v:
                print(f"Joy:Read failed {e}")
        return pressed


    def _apply_manual_movement(self, delta: int) -> bool:
        joystick = self._read_joystick_inputs()
        x_pos = self.button_states.get(BUTTON_TYPES["RIGHT"]) or joystick['+x']
        x_neg = self.button_states.get(BUTTON_TYPES["LEFT"]) or joystick['-x']
        y_pos = self.button_states.get(BUTTON_TYPES["UP"]) or joystick['+y']
        y_neg = self.button_states.get(BUTTON_TYPES["DOWN"]) or joystick['-y']

        pressed = x_pos or x_neg or y_pos or y_neg

        if x_pos and not x_neg:
            if self._auto_repeat_check(delta, False):
                speed = abs(self._stepperX.get_speed())
                speed = max(self._settings['min_speed'].v, self._inc(speed, 1 + self._auto_repeat_level))
                self._stepperX.speed(speed)
                self._refresh = True
        elif x_neg and not x_pos:
            if self._auto_repeat_check(delta, False):
                speed = abs(self._stepperX.get_speed())
                speed = max(self._settings['min_speed'].v, self._inc(speed, 1 + self._auto_repeat_level))
                self._stepperX.speed(-speed)
                self._refresh = True
        elif self._stepperX.speed(0):
            self._refresh = True

        if y_pos and not y_neg:
            if self._auto_repeat_check(delta, False):
                speed = abs(self._stepperY.get_speed())
                speed = max(self._settings['min_speed'].v, self._inc(speed, 1 + self._auto_repeat_level))
                self._stepperY.speed(speed)
                self._refresh = True
        elif y_neg and not y_pos:
            if self._auto_repeat_check(delta, False):
                speed = abs(self._stepperY.get_speed())
                speed = max(self._settings['min_speed'].v, self._inc(speed, 1 + self._auto_repeat_level))
                self._stepperY.speed(-speed)
                self._refresh = True
        elif self._stepperY.speed(0):
            self._refresh = True

        return pressed


    def _update_state_settings(self, delta: int):    
        self._set_led_mode(LED_MODE_INPUT)
        if self.button_states.get(BUTTON_TYPES["UP"]):
            if self._auto_repeat_check(delta, False):
                self._edit_setting_value = self._settings[self._edit_setting].inc(self._edit_setting_value, self._auto_repeat_level)
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} (+) Value: {self._edit_setting_value}")
                self._refresh = True
        elif self.button_states.get(BUTTON_TYPES["DOWN"]):
            if self._auto_repeat_check(delta, False):
                self._edit_setting_value = self._settings[self._edit_setting].dec(self._edit_setting_value, self._auto_repeat_level)  
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} (-) Value: {self._edit_setting_value}")
                self._refresh = True            
        else:
            # non auto-repeating buttons
            self._auto_repeat_clear()                           
            if self.button_states.get(BUTTON_TYPES["RIGHT"]) or self.button_states.get(BUTTON_TYPES["LEFT"]):
                self.button_states.clear() 
                # Force default value    
                self._edit_setting_value = self._settings[self._edit_setting].d
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} Default: {self._edit_setting_value}")
                self._refresh = True
                self.notification = Notification("Default")
            elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.button_states.clear()
                # leave setting unchanged
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} Cancelled")
                self.set_menu("Settings")
                self.current_state = STATE_MENU
            elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.button_states.clear()
                # set setting
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} = {self._edit_setting_value}")
                self._settings[self._edit_setting].v = self._edit_setting_value
                self._settings[self._edit_setting].persist()
                self.notification = Notification(f"Setting: {self._edit_setting}={self._edit_setting_value}")
                self.set_menu("Settings")
                self.current_state = STATE_MENU


    def _mm_to_steps(self, axis: str, mm: float) -> int:
        scale = self._settings['x_steps_per_mm'].v if axis == 'x' else self._settings['y_steps_per_mm'].v
        return int(round(mm * scale))


    def _steps_to_mm(self, axis: str, steps: int) -> float:
        scale = self._settings['x_steps_per_mm'].v if axis == 'x' else self._settings['y_steps_per_mm'].v
        if scale <= 0:
            return 0.0
        return float(steps) / float(scale)


    def _mm_min_to_sps_x(self, mm_min: float) -> int:
        return max(0, int((mm_min * self._settings['x_steps_per_mm'].v) / 60.0))


    def _mm_min_to_sps_y(self, mm_min: float) -> int:
        return max(0, int((mm_min * self._settings['y_steps_per_mm'].v) / 60.0))


    def _sps_to_mm_min_x(self, sps: int) -> float:
        scale = self._settings['x_steps_per_mm'].v
        if scale <= 0:
            return 0.0
        return (float(sps) * 60.0) / float(scale)


    def _sps_to_mm_min_y(self, sps: int) -> float:
        scale = self._settings['y_steps_per_mm'].v
        if scale <= 0:
            return 0.0
        return (float(sps) * 60.0) / float(scale)


    def _default_gcode_speed_limits_mm_min(self):
        feed = max(1.0, float(self._settings['mm_per_min'].v))
        return {
            'x': feed,
            'y': feed,
        }


    def _get_homing_speed_sps(self, axis: str) -> int:
        min_sps = max(1, int(self._settings['min_speed'].v))
        max_sps = max(min_sps, int(self._settings['max_speed'].v))
        limit_mm_min = self._gcode_speed_limit['x'] if axis == 'x' else self._gcode_speed_limit['y']
        limit_sps = self._mm_min_to_sps_x(limit_mm_min) if axis == 'x' else self._mm_min_to_sps_y(limit_mm_min)
        if limit_sps <= 0:
            limit_sps = max_sps
        return max(min_sps, min(max_sps, int(limit_sps)))


    def _trim_filename(self, filename: str) -> str:
        if filename is None:
            return ""
        cleaned = []
        for ch in str(filename):
            if (('a' <= ch <= 'z') or ('A' <= ch <= 'Z') or ('0' <= ch <= '9') or ch in ('_', '-', '.')):
                cleaned.append(ch)
            elif ch == ' ':
                cleaned.append('_')
        return "".join(cleaned).strip('._')


    def _ensure_gcode_dir(self):
        try:
            os.stat(_GCODE_DIR)
        except OSError:
            os.mkdir(_GCODE_DIR)


    def _load_gcode_files(self):
        self._gcode_files = []
        self._gcode_file_names = []
        try:
            files = os.listdir(_GCODE_DIR)
        except OSError as e:
            if self._settings['logging'].v:
                print(f"GCODE:List failed {_GCODE_DIR} {e}")
            return

        labels_seen = {}
        for filename in sorted(files):
            if not filename.lower().endswith(_GCODE_EXTENSION):
                continue
            label = filename[:-len(_GCODE_EXTENSION)]
            if label == "":
                label = "unnamed"
            if label in labels_seen:
                labels_seen[label] += 1
                shown = f"{label}~{labels_seen[label]}"
            else:
                labels_seen[label] = 1
                shown = label
            self._gcode_files.append(shown)
            self._gcode_file_names.append(filename)


    def _enter_gcode_file_browser(self):
        self._ensure_gcode_dir()
        self._load_gcode_files()
        if len(self._gcode_files) == 0:
            self.notification = Notification("No .nc files")
            self.current_state = STATE_MENU
            self.set_menu("main")
            self._set_led_mode(LED_MODE_IDLE)
            return

        self.current_state = STATE_GCODE_FILES
        self.set_menu("GCODE Files")
        self.button_states.clear()
        self._set_led_mode(LED_MODE_INPUT)
        self._refresh = True


    def _gcode_file_select_handler(self, item: str, idx: int):
        del item
        if idx < 0 or idx >= len(self._gcode_file_names):
            return

        filename = self._gcode_file_names[idx]
        full_path = f"{_GCODE_DIR}/{filename}"
        self._set_led_mode(LED_MODE_BUSY)
        self._log_gcode(f"Load:start {full_path}")
        try:
            commands = self._parse_gcode_file(full_path)
        except Exception as e:
            self._set_error_state(["GCODE load", "failed", str(e)[:18]])
            self._log_gcode(f"Load:error {full_path} {e}")
            return

        if len(commands) == 0:
            self.notification = Notification("No GCODE")
            self.current_state = STATE_MENU
            self.set_menu("main")
            self._set_led_mode(LED_MODE_IDLE)
            return

        self._log_gcode(f"Load:end {full_path} cmds={len(commands)}")
        self._start_gcode_replay(commands, filename)


    def _gcode_file_back_handler(self):
        self.current_state = STATE_MENU
        self.set_menu("main")
        self._set_led_mode(LED_MODE_IDLE)


    def _strip_gcode_comments(self, text: str) -> str:
        out = []
        in_paren = False
        for ch in text:
            if ch == ';' and not in_paren:
                break
            if ch == '(' and not in_paren:
                in_paren = True
                continue
            if ch == ')' and in_paren:
                in_paren = False
                continue
            if not in_paren:
                out.append(ch)
        return "".join(out).strip()


    def _scan_gcode_words(self, text: str):
        words = []
        i = 0
        while i < len(text):
            if text[i].isspace() or text[i] == ',':
                i += 1
                continue
            if not text[i].isalpha():
                raise ValueError(f"bad token '{text[i]}'")
            letter = text[i].upper()
            i += 1
            start = i
            while i < len(text):
                ch = text[i]
                if ch.isspace() or ch == ',':
                    break
                if ch.isalpha():
                    # Allow scientific notation like 1e-3 as part of numeric values.
                    if ch in ('e', 'E') and i > start:
                        i += 1
                        if i < len(text) and text[i] in ('+', '-'):
                            i += 1
                        continue
                    break
                i += 1
            value = text[start:i].strip()
            words.append((letter, value))
        return words


    def _normalize_gcode_cmd(self, cmd: str) -> str:
        mapping = {
            "G00": "G0",
            "G01": "G1",
            "M00": "M0",
            "M01": "M1",
        }
        return mapping.get(cmd, cmd)


    def _parse_gcode_line(self, line: str, line_num: int):
        clean = self._strip_gcode_comments(line)
        if clean == "":
            return None

        words = self._scan_gcode_words(clean)
        if len(words) == 0:
            return None

        cmd_letter, cmd_value = words[0]
        if cmd_letter not in ('G', 'M') or cmd_value == "":
            raise ValueError(f"L{line_num}: bad command")
        cmd = self._normalize_gcode_cmd(f"{cmd_letter}{cmd_value}")

        valid = {
            "G0", "G1", "G4", "G28", "G90", "G91", "G92",
            "M0", "M1", "M17", "M18", "M112", "M114", "M203", "M204", "M400"
        }
        if cmd not in valid:
            raise ValueError(f"L{line_num}: unsupported {cmd}")

        allowed_params = {
            "G0": {'X', 'Y', 'F'},
            "G1": {'X', 'Y', 'F'},
            "G4": {'P', 'S'},
            "G28": {'X', 'Y', 'O'},
            "G90": set(),
            "G91": set(),
            "G92": {'X', 'Y'},
            "M0": set(),
            "M1": set(),
            "M17": set(),
            "M18": set(),
            "M112": set(),
            "M114": set(),
            "M203": {'X', 'Y'},
            "M204": {'S'},
            "M400": set(),
        }

        params = {}
        for letter, value in words[1:]:
            if letter in params:
                raise ValueError(f"L{line_num}: duplicate {letter}")
            if value == "":
                if cmd == "G28" and letter in ('X', 'Y', 'O'):
                    params[letter] = True
                    continue
                raise ValueError(f"L{line_num}: missing value for {letter}")
            try:
                numeric_value = float(value)
            except ValueError:
                raise ValueError(f"L{line_num}: invalid value '{letter}{value}'")
            if not isfinite(numeric_value):
                raise ValueError(f"L{line_num}: non-finite value '{letter}{value}'")
            params[letter] = numeric_value

        for letter in params:
            if letter not in allowed_params[cmd]:
                raise ValueError(f"L{line_num}: {cmd} bad arg {letter}")

        if cmd in ("G0", "G1") and 'X' not in params and 'Y' not in params:
            raise ValueError(f"L{line_num}: {cmd} needs X and/or Y")
        if cmd == "G4" and 'P' not in params and 'S' not in params:
            raise ValueError(f"L{line_num}: G4 needs P or S")
        if cmd == "G4":
            if 'P' in params and params['P'] < 0:
                raise ValueError(f"L{line_num}: G4 P must be >= 0")
            if 'S' in params and params['S'] < 0:
                raise ValueError(f"L{line_num}: G4 S must be >= 0")
        if cmd == "M203" and 'X' not in params and 'Y' not in params:
            raise ValueError(f"L{line_num}: M203 needs X and/or Y")
        if cmd == "M203":
            if 'X' in params and params['X'] <= 0:
                raise ValueError(f"L{line_num}: M203 X must be > 0")
            if 'Y' in params and params['Y'] <= 0:
                raise ValueError(f"L{line_num}: M203 Y must be > 0")
        if cmd == "M204" and 'S' not in params:
            raise ValueError(f"L{line_num}: M204 needs S")
        if cmd == "M204" and params['S'] <= 0:
            raise ValueError(f"L{line_num}: M204 S must be > 0")

        return {
            'cmd': cmd,
            'params': params,
            'line': line_num,
            'text': clean,
        }


    def _parse_gcode_file(self, full_path: str):
        parsed = []
        with open(full_path, "r") as handle:
            for line_num, line in enumerate(handle, 1):
                entry = self._parse_gcode_line(line, line_num)
                if entry is not None:
                    parsed.append(entry)
                    self._log_gcode(f"parse L{line_num} {entry['cmd']}")
        return parsed


    def _build_gcode_commands_from_lines(self, lines):
        parsed = []
        for line_num, line in enumerate(lines, 1):
            entry = self._parse_gcode_line(line, line_num)
            if entry is not None:
                parsed.append(entry)
        return parsed


    def _get_default_homing_preamble_lines(self):
        fast_feed = max(1.0, float(self._settings['mm_per_min'].v))
        fine_feed = max(_GCODE_HOMING_FINE_FEED_MIN_MM_MIN, fast_feed * _GCODE_HOMING_FINE_FEED_RATIO)
        backoff_mm = max(0.1, float(_GCODE_HOMING_BACKOFF_MM))
        # convert from stage coordinates (0,0 in centre) in usteps to GCODE coordinates (0,0 at min endstop) in mm
        centre_x_mm = self._steps_to_mm('x', self._settings['XRange'].v // 2)
        centre_y_mm = self._steps_to_mm('y', self._settings['YRange'].v // 2)

        return [
            f"M203 X{fast_feed:.2f} Y{fast_feed:.2f}",
            "G28",
            "G91",
            f"G1 X{backoff_mm:.2f} Y{backoff_mm:.2f} F{fast_feed:.2f}",
            "G90",
            f"M203 X{fine_feed:.2f} Y{fine_feed:.2f}",
            "G28",
            f"M203 X{fast_feed:.2f} Y{fast_feed:.2f}",
            "G90",
            f"G1 X{centre_x_mm:.2f} Y{centre_y_mm:.2f} F{fast_feed:.2f}",
        ]


    def _start_default_homing_sequence(self):
        if not self._ensure_steppers_ready(silent=False):
            return

        try:
            commands = self._build_gcode_commands_from_lines(self._get_default_homing_preamble_lines())
        except Exception as e:
            self._set_error_state(["Home script", "invalid", str(e)[:18]], f"Home script parse failed {e}")
            return

        self._start_gcode_replay(commands, "HOME")
        self.notification = Notification("Home XY")


    def _clear_gcode_runtime(self):
        self._gcode_commands = []
        self._gcode_current_index = 0
        self._gcode_current_command = None
        self._gcode_history = []
        self._gcode_wait_remaining_ms = None
        self._gcode_wait_button = False
        self._gcode_abort_requested = False
        self._gcode_move_active = False
        self._gcode_homing_axes = []
        self._gcode_homing_remaining_ms = None


    def _start_gcode_replay(self, commands, filename: str):
        if not self._ensure_steppers_ready(silent=False):
            return

        self._clear_gcode_runtime()
        self._apply_stepper_limits_from_settings()
        self._stepperX.enable(True)
        self._stepperY.enable(True)
        self._gcode_commands = commands
        self._gcode_selected_file = filename
        self._gcode_mode_absolute = True
        self._gcode_move_feed_mm_min = self._settings['mm_per_min'].v
        self._gcode_speed_limit = self._default_gcode_speed_limits_mm_min()
        self.current_state = STATE_GCODE_REPLAY
        self.set_menu(None)
        self.button_states.clear()
        self._refresh = True
        self._set_led_mode(LED_MODE_IDLE)
        self.notification = Notification(f"Run:{filename[:10]}")


    def _push_gcode_history(self, text: str):
        self._gcode_history.append(text)
        if len(self._gcode_history) > _GCODE_HISTORY_MAX:
            self._gcode_history.pop(0)


    def _log_gcode(self, message: str):
        if self._settings['logging'].v:
            print(f"GCODE:{message}")


    def _complete_current_gcode_step(self):
        if self._gcode_current_command is not None:
            self._log_gcode(
                f"step end L{self._gcode_current_command['line']} {self._gcode_current_command['cmd']}"
            )
            self._gcode_current_command = None
        self._gcode_current_index += 1
        self._refresh = True


    def _set_error_state(self, message_lines, log_line: str = None):
        if self._stepperX is not None:
            self._stepperX.stop()
            self._stepperX.enable(False)
        if self._stepperY is not None:
            self._stepperY.stop()
            self._stepperY.enable(False)
        self.error_message = message_lines[:6]
        self.current_state = STATE_ERROR
        self._refresh = True
        self._set_led_mode(LED_MODE_ERROR)
        if log_line is not None:
            self._log_gcode(log_line)


    def _update_state_error(self, delta: int):
        del delta
        self._set_led_mode(LED_MODE_ERROR)
        if self.button_states.get(BUTTON_TYPES["CANCEL"]) or self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.button_states.clear()
            self._clear_gcode_runtime()
            self.current_state = STATE_MENU
            self.set_menu("main")
            self._set_led_mode(LED_MODE_IDLE)
            self._refresh = True


    def _set_axis_speed_limits_from_feed(self, feed_mm_min: float, target_steps):
        min_sps = max(1, int(self._settings['min_speed'].v))
        max_sps = max(min_sps, int(self._settings['max_speed'].v))

        current_x = self._stepperX.get_pos(0)
        current_y = self._stepperY.get_pos(0)
        dx = abs(target_steps['x'] - current_x)
        dy = abs(target_steps['y'] - current_y)

        feed_x = max(1.0, min(feed_mm_min, self._gcode_speed_limit['x']))
        feed_y = max(1.0, min(feed_mm_min, self._gcode_speed_limit['y']))
        sps_x = min(max_sps, max(min_sps, self._mm_min_to_sps_x(feed_x)))
        sps_y = min(max_sps, max(min_sps, self._mm_min_to_sps_y(feed_y)))

        if dx == 0 and dy == 0:
            self._stepperX.set_max_sps(max_sps)
            self._stepperY.set_max_sps(max_sps)
            return

        if dx == 0:
            self._stepperX.set_max_sps(min_sps)
            self._stepperY.set_max_sps(sps_y)
            return
        if dy == 0:
            self._stepperX.set_max_sps(sps_x)
            self._stepperY.set_max_sps(min_sps)
            return

        time_x = dx / float(max(1, sps_x))
        time_y = dy / float(max(1, sps_y))
        travel_time = max(time_x, time_y)
        sync_x = max(min_sps, int(dx / travel_time))
        sync_y = max(min_sps, int(dy / travel_time))
        self._stepperX.set_max_sps(min(max_sps, sync_x))
        self._stepperY.set_max_sps(min(max_sps, sync_y))


    def _start_move_command(self, cmd):
        params = cmd['params']
        current_steps = {
            'x': self._stepperX.get_pos(0),
            'y': self._stepperY.get_pos(0),
        }
        target = current_steps.copy()

        if self._gcode_mode_absolute:
            if 'X' in params:
                target['x'] = self._mm_to_steps('x', params['X'])
            if 'Y' in params:
                target['y'] = self._mm_to_steps('y', params['Y'])
        else:
            if 'X' in params:
                target['x'] = current_steps['x'] + self._mm_to_steps('x', params['X'])
            if 'Y' in params:
                target['y'] = current_steps['y'] + self._mm_to_steps('y', params['Y'])

        target['x'] = min(max(target['x'], 0), self._settings['XRange'].v)
        target['y'] = min(max(target['y'], 0), self._settings['YRange'].v)
        self._gcode_move_target = target

        if 'F' in params:
            self._gcode_move_feed_mm_min = max(1.0, params['F'])

        self._set_axis_speed_limits_from_feed(self._gcode_move_feed_mm_min, target)

        # Duplicate/near-duplicate targets should not enter motion wait state.
        # This prevents replay from stalling on consecutive G1 commands that
        # resolve to the same coordinate after rounding/tolerance.
        if (
            abs(current_steps['x'] - target['x']) <= POSITION_MATCH_TOLERANCE
            and abs(current_steps['y'] - target['y']) <= POSITION_MATCH_TOLERANCE
        ):
            self._stepperX.speed(0)
            self._stepperY.speed(0)
            self._gcode_move_active = False
            self._complete_current_gcode_step()
            self._apply_stepper_limits_from_settings()
            self._set_led_mode(LED_MODE_IDLE)
            return

        self._gcode_move_active = True
        self._set_led_mode(LED_MODE_MOVING)


    def _update_gcode_motion(self):
        x_done = True
        y_done = True
        tx = self._gcode_move_target['x']
        ty = self._gcode_move_target['y']

        x_pos = self._stepperX.get_pos(0)
        y_pos = self._stepperY.get_pos(0)

        if x_pos > (tx + POSITION_MATCH_TOLERANCE):
            self._stepperX.speed(-self._stepperX.get_speed_from_disance(tx - x_pos))
            x_done = False
        elif x_pos < (tx - POSITION_MATCH_TOLERANCE):
            self._stepperX.speed(self._stepperX.get_speed_from_disance(tx - x_pos))
            x_done = False
        else:
            self._stepperX.speed(0)

        if y_pos > (ty + POSITION_MATCH_TOLERANCE):
            self._stepperY.speed(-self._stepperY.get_speed_from_disance(ty - y_pos))
            y_done = False
        elif y_pos < (ty - POSITION_MATCH_TOLERANCE):
            self._stepperY.speed(self._stepperY.get_speed_from_disance(ty - y_pos))
            y_done = False
        else:
            self._stepperY.speed(0)

        self._refresh = True
        if x_done and y_done:
            self._gcode_move_active = False
            self._complete_current_gcode_step()
            self._apply_stepper_limits_from_settings()
            self._set_led_mode(LED_MODE_IDLE)


    def _start_homing(self, cmd):
        params = cmd['params']
        only_if_needed = False
        if 'O' in params:
            o_val = params.get('O')
            if o_val is True:
                only_if_needed = True
            else:
                try:
                    only_if_needed = float(o_val) != 0.0
                except Exception:
                    only_if_needed = bool(o_val)

        axes = []
        if len(params) == 0 or (len(params) == 1 and 'O' in params):
            axes = ['x', 'y']
        else:
            if 'X' in params:
                axes.append('x')
            if 'Y' in params:
                axes.append('y')
        if len(axes) == 0:
            axes = ['x', 'y']

        if only_if_needed:
            pending_axes = []
            for axis in axes:
                stepper = self._stepperX if axis == 'x' else self._stepperY
                if not stepper.is_calibrated():
                    pending_axes.append(axis)

            if len(pending_axes) == 0:
                self._log_gcode(f"G28 skip already homed axes={axes}")
                self._complete_current_gcode_step()
                self._set_led_mode(LED_MODE_IDLE)
                return

            axes = pending_axes

        self._gcode_homing_axes = axes
        if 'x' in axes:
            self._stepperX.set_calibrated(False)
        if 'y' in axes:
            self._stepperY.set_calibrated(False)

        # Build a timeout from configured travel and active homing speed.
        timeout_ms = _GCODE_HOMING_TIMEOUT_MIN_MS
        for axis in axes:
            axis_range = self._settings['XRange'].v if axis == 'x' else self._settings['YRange'].v
            home_speed = self._get_homing_speed_sps(axis)
            axis_timeout = int((axis_range * 1000) / home_speed) + _GCODE_HOMING_TIMEOUT_MARGIN_MS
            if axis_timeout > timeout_ms:
                timeout_ms = axis_timeout
        timeout_ms = min(_GCODE_HOMING_TIMEOUT_MAX_MS, max(_GCODE_HOMING_TIMEOUT_MIN_MS, timeout_ms))
        self._gcode_homing_remaining_ms = timeout_ms
        self._log_gcode(f"G28 timeout {timeout_ms}ms axes={axes}")
        self._set_led_mode(LED_MODE_MOVING)


    def _update_gcode_homing(self, delta: int):
        if self._gcode_homing_remaining_ms is not None:
            self._gcode_homing_remaining_ms -= delta
            if self._gcode_homing_remaining_ms <= 0:
                for axis in self._gcode_homing_axes:
                    stepper = self._stepperX if axis == 'x' else self._stepperY
                    stepper.speed(0)
                timed_out_axes = "".join([axis.upper() for axis in self._gcode_homing_axes])
                if timed_out_axes == "":
                    timed_out_axes = "XY"
                self._gcode_homing_axes = []
                self._gcode_homing_remaining_ms = None
                self._set_error_state(
                    ["Homing timeout", f"G28 {timed_out_axes}", "Check endstops"],
                    f"G28 timeout axes={timed_out_axes}",
                )
                return

        remaining = []
        for axis in self._gcode_homing_axes:
            stepper = self._stepperX if axis == 'x' else self._stepperY
            home_speed = self._get_homing_speed_sps(axis)
            if stepper.is_endstop_pressed() or stepper.is_calibrated():
                stepper.set_pos(0)
                stepper.set_calibrated(True)
                stepper.speed(0)
                continue
            stepper.speed(-home_speed)
            remaining.append(axis)

        self._gcode_homing_axes = remaining
        self._refresh = True
        if len(self._gcode_homing_axes) == 0:
            self._gcode_homing_remaining_ms = None
            self._complete_current_gcode_step()
            self._set_led_mode(LED_MODE_IDLE)


    def _execute_next_gcode_command(self):
        if self._gcode_current_index >= len(self._gcode_commands):
            return

        cmd = self._gcode_commands[self._gcode_current_index]
        self._gcode_current_command = cmd
        self._push_gcode_history(cmd['text'])
        self._log_gcode(f"step start L{cmd['line']} {cmd['text']}")

        c = cmd['cmd']
        p = cmd['params']

        if c == 'G90':
            self._gcode_mode_absolute = True
            self._complete_current_gcode_step()
        elif c == 'G91':
            self._gcode_mode_absolute = False
            self._complete_current_gcode_step()
        elif c == 'G92':
            if 'X' in p:
                self._stepperX.set_pos(self._mm_to_steps('x', p['X']))
                self._stepperX.set_calibrated(True)
            if 'Y' in p:
                self._stepperY.set_pos(self._mm_to_steps('y', p['Y']))
                self._stepperY.set_calibrated(True)
            self._complete_current_gcode_step()
        elif c in ('G0', 'G1'):
            self._start_move_command(cmd)
        elif c == 'G28':
            self._start_homing(cmd)
        elif c == 'G4':
            wait_ms = int(p.get('P', 0.0) + (p.get('S', 0.0) * 1000.0))
            if wait_ms <= 0:
                self._complete_current_gcode_step()
            else:
                self._gcode_wait_remaining_ms = wait_ms
                self._set_led_mode(LED_MODE_IDLE)
        elif c in ('M0', 'M1'):
            self._gcode_wait_button = True
            self._set_led_mode(LED_MODE_INPUT)
        elif c == 'M17':
            self._stepperX.enable(True)
            self._stepperY.enable(True)
            self._complete_current_gcode_step()
        elif c == 'M18':        # Disable steppers
            self._stepperX.stop()
            self._stepperY.stop()
            self._stepperX.enable(False)
            self._stepperY.enable(False)
            self._complete_current_gcode_step()
        elif c == 'M112':       # Emergency stop
            self._stepperX.stop()
            self._stepperY.stop()
            self._stepperX.enable(False)
            self._stepperY.enable(False)
            self._set_error_state(["Emergency stop", "M112", "Re-home req"], "M112 emergency stop")
        elif c == 'M114':       # Report position
            x_mm = self._steps_to_mm('x', self._stepperX.get_pos(0))
            y_mm = self._steps_to_mm('y', self._stepperY.get_pos(0))
            self._log_gcode(f"M114 X{x_mm:.2f} Y{y_mm:.2f}")
            self._complete_current_gcode_step()
        elif c == 'M203':       # Speed limits
            if 'X' in p:
                self._gcode_speed_limit['x'] = max(1.0, p['X'])
            if 'Y' in p:
                self._gcode_speed_limit['y'] = max(1.0, p['Y'])
            self._complete_current_gcode_step()
        elif c == 'M204':       # Acceleration
            accel_mm_s2 = max(1.0, p['S'])
            updates_x = max(1, int(getattr(self._stepperX, "_updates_per_sec", 1)))
            updates_y = max(1, int(getattr(self._stepperY, "_updates_per_sec", 1)))
            x_change = int((accel_mm_s2 * self._settings['x_steps_per_mm'].v) / updates_x)
            y_change = int((accel_mm_s2 * self._settings['y_steps_per_mm'].v) / updates_y)
            self._stepperX.set_max_sps_change(max(1, x_change))
            self._stepperY.set_max_sps_change(max(1, y_change))
            self._complete_current_gcode_step()
        elif c == 'M400':       # Wait for moves to finish
            self._complete_current_gcode_step()
        else:
            self._set_error_state(["Unsupported", c, f"Line {cmd['line']}"], f"unsupported L{cmd['line']} {c}")


    def _abort_gcode_replay(self, reason: str):
        self._log_gcode(f"abort {reason}")
        self._stepperX.stop()
        self._stepperY.stop()
        self._stepperX.enable(False)
        self._stepperY.enable(False)
        self._clear_gcode_runtime()
        self.current_state = STATE_MENU
        self.set_menu("main")
        self.notification = Notification("Replay aborted")
        self._set_led_mode(LED_MODE_IDLE)


    def _update_state_gcode_replay(self, delta: int):
        if not self._ensure_steppers_ready(silent=True):
            self._set_error_state(["No steppers", "GCODE abort"])
            return

        self._update_live_position(delta)

        if self.button_states.get(BUTTON_TYPES["CANCEL"]):
            self.button_states.clear()
            self._abort_gcode_replay("cancel")
            return

        if self._gcode_homing_axes:
            self._update_gcode_homing(delta)
            return

        if self._gcode_move_active:
            self._update_gcode_motion()
            return

        if self._gcode_wait_button:
            # Keep decelerating to zero while waiting for user input.
            self._stepperX.speed(0)
            self._stepperY.speed(0)
            self._set_led_mode(LED_MODE_INPUT)
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.button_states.clear()
                self._gcode_wait_button = False
                self._complete_current_gcode_step()
            return

        if self._gcode_wait_remaining_ms is not None:
            # Keep decelerating to zero during dwell.
            self._stepperX.speed(0)
            self._stepperY.speed(0)
            self._set_led_mode(LED_MODE_IDLE)
            self._gcode_wait_remaining_ms -= delta
            if self._gcode_wait_remaining_ms <= 0:
                self._gcode_wait_remaining_ms = None
                self._complete_current_gcode_step()
            return

        if self._gcode_current_index >= len(self._gcode_commands):
            self._log_gcode("replay complete")
            self.notification = Notification("Replay done")
            self.current_state = STATE_MENU
            self.set_menu("main")
            self._stepperX.enable(False)
            self._stepperY.enable(False)
            self._set_led_mode(LED_MODE_IDLE)
            return

        # Run non-blocking commands immediately in sequence.
        guard = 0
        while (
            self.current_state == STATE_GCODE_REPLAY
            and self._gcode_current_index < len(self._gcode_commands)
            and not self._gcode_move_active
            and not self._gcode_homing_axes
            and not self._gcode_wait_button
            and self._gcode_wait_remaining_ms is None
        ):
            self._execute_next_gcode_command()
            guard += 1
            if guard > 12:
                break


    def _capture_record_waypoint(self):
        x_mm = self._steps_to_mm('x', self._stepperX.get_pos(0))
        y_mm = self._steps_to_mm('y', self._stepperY.get_pos(0))
        self._record_waypoints_mm.append((x_mm, y_mm))
        self._log_gcode(f"record capture idx={len(self._record_waypoints_mm)} X{x_mm:.2f} Y{y_mm:.2f}")
        self.notification = Notification(f"Pt {len(self._record_waypoints_mm)}")
        self._refresh = True


    def _close_filename_dialog(self):
        if self._filename_dialog is not None:
            try:
                self._filename_dialog._cleanup()
            except Exception:
                pass
        self._filename_dialog = None


    def _record_save_complete(self):
        filename = self._filename_dialog.text if self._filename_dialog is not None else ""
        self._close_filename_dialog()
        self._save_recording_file(filename)


    def _record_save_cancel(self):
        self._close_filename_dialog()
        self.notification = Notification("Save cancelled")
        self._set_led_mode(LED_MODE_IDLE)


    def _open_filename_dialog(self):
        self._set_led_mode(LED_MODE_INPUT)
        self._filename_dialog = TextDialog(
            "Save as:",
            self,
            masked=False,
            on_complete=self._record_save_complete,
            on_cancel=self._record_save_cancel,
        )
        self._refresh = True


    def _save_recording_file(self, filename: str):
        if len(self._record_waypoints_mm) == 0:
            self.notification = Notification("No points")
            self._set_led_mode(LED_MODE_INPUT)
            return

        trimmed = self._trim_filename(filename)
        if trimmed == "":
            self.notification = Notification("Bad filename")
            self._set_led_mode(LED_MODE_INPUT)
            return

        if not trimmed.lower().endswith(_GCODE_EXTENSION):
            trimmed += _GCODE_EXTENSION

        self._set_led_mode(LED_MODE_BUSY)
        self._ensure_gcode_dir()
        full_path = f"{_GCODE_DIR}/{trimmed}"

        try:
            with open(full_path, "w") as handle:
                handle.write(f"; XYStage {_APP_VERSION}\n")
                for line in self._get_default_homing_preamble_lines():
                    handle.write(line + "\n")
                for x_mm, y_mm in self._record_waypoints_mm:
                    handle.write(f"G1 X{x_mm:.2f} Y{y_mm:.2f} F{self._settings['mm_per_min'].v}\n")
                    handle.write("M0\n")
            self.notification = Notification(f"Saved {trimmed}")
            self._log_gcode(f"record saved {full_path}")
        except Exception as e:
            self._set_error_state(["Save failed", str(e)[:18]], f"save failed {e}")
            return

        self._record_waypoints_mm = []
        self.current_state = STATE_MENU
        self.set_menu("main")
        self._set_led_mode(LED_MODE_IDLE)


    def _update_state_gcode_record(self, delta: int):
        if not self._ensure_steppers_ready(silent=False):
            self.current_state = STATE_MENU
            self.set_menu("main")
            return

        if self._filename_dialog is not None:
            self._set_led_mode(LED_MODE_INPUT)
            return

        self._update_live_position(delta)
        pressed = self._apply_manual_movement(delta)

        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.button_states.clear()
            if not self._record_capture_debounce:
                self._record_capture_debounce = True
                self.notification = Notification("Point captured")
                self._capture_record_waypoint()
            return
        else:
            self._record_capture_debounce = False

        if self.button_states.get(BUTTON_TYPES["CANCEL"]):
            self.button_states.clear()
            if len(self._record_waypoints_mm) == 0:
                self.notification = Notification("No points")
                #self._record_waypoints_mm = []
                self._close_filename_dialog()
                self._stepperX.stop()
                self._stepperY.stop()
                self._stepperX.enable(False)
                self._stepperY.enable(False)
                self.current_state = STATE_MENU
                self.set_menu("main")
                self._set_led_mode(LED_MODE_IDLE)
            else:
                self._open_filename_dialog()
            return

        if pressed:
            self._time_since_last_input = 0
            self._set_led_mode(LED_MODE_MOVING)
        else:
            self._set_led_mode(LED_MODE_IDLE)


    def _ensure_steppers_ready(self, silent: bool = False) -> bool:
        if self._xystage_port is None:
            if not silent:
                self.notification = Notification("No XYStage")
                self._set_led_mode(LED_MODE_ERROR)
            return False
        self._xystage_config: HexpansionConfig  = HexpansionConfig(self._xystage_port)

        if self._stepperX is None or self._stepperY is None:
            for i in range(4):
                if self._stepperX is None:
                    try:
                        pins = {}
                        pins["dir"] = self._xystage_config.ls_pin[X_DIR]
                        pins["en"] = self._xystage_config.ls_pin[X_ENABLE]
                        pins["step"] = self._xystage_config.pin[X_STEP]
                        pins["stop"] = self._xystage_config.pin[X_ENDSTOP]
                        self._stepperX = Stepper(
                            self,
                            pins,
                            initial_pos=self._settings['XRange'].v//2,
                            reverse=True,
                            name="X",
                            max_sps=self._settings['max_speed'].v,
                            max_pos=self._settings['XRange'].v,
                            max_sps_change=self._settings['acceleration'].v,
                        )
                        if self._settings['logging'].v:
                            print(f"StepperX:Init {i}")
                        continue
                    except Exception as e:
                        if self._settings['logging'].v:
                            print(f"StepperX:Init {i} Failed {e}")
                elif self._stepperY is None:
                    try:
                        pins = {}
                        pins["dir"] = self._xystage_config.ls_pin[Y_DIR]
                        pins["en"] = self._xystage_config.ls_pin[Y_ENABLE]
                        pins["step"] = self._xystage_config.pin[Y_STEP]
                        pins["stop"] = self._xystage_config.pin[Y_ENDSTOP]
                        self._stepperY = Stepper(
                            self,
                            pins,
                            initial_pos=self._settings['YRange'].v//2,
                            name="Y",
                            max_sps=self._settings['max_speed'].v,
                            max_pos=self._settings['YRange'].v,
                            max_sps_change=self._settings['acceleration'].v,
                        )
                        if self._settings['logging'].v:
                            print(f"StepperY:Init {i}")
                        continue
                    except Exception as e:
                        if self._settings['logging'].v:
                            print(f"StepperY:Init {i} Failed {e}")

        if self._stepperX is None or self._stepperY is None:
            if not silent:
                self.notification = Notification("No Free Timers")
                self._set_led_mode(LED_MODE_ERROR)
            return False

        self._apply_stepper_limits_from_settings()
        return True


    def _apply_stepper_limits_from_settings(self):
        if self._stepperX is None or self._stepperY is None:
            return
        self._stepperX.set_max_sps(self._settings['max_speed'].v)
        self._stepperY.set_max_sps(self._settings['max_speed'].v)
        self._stepperX.set_max_sps_change(self._settings['acceleration'].v)
        self._stepperY.set_max_sps_change(self._settings['acceleration'].v)


    def _configure_joystick_inputs(self):
        if self._joystick_port is None:
            return
        try:
            self._joystick_config = HexpansionConfig(self._joystick_port)
            self._joystick_pins = {
                JOY_POS_X: self._joystick_config.pin[JOY_POS_X],
                JOY_NEG_X: self._joystick_config.pin[JOY_NEG_X],
                JOY_POS_Y: self._joystick_config.pin[JOY_POS_Y],
                JOY_NEG_Y: self._joystick_config.pin[JOY_NEG_Y],
            }
            for _, joy_pin in self._joystick_pins.items():
                joy_pin.init(mode=Pin.IN, pull=Pin.PULL_UP)
            if self._settings['logging'].v:
                print(f"Joy:Configured port {self._joystick_port}")
        except Exception as e:
            self._joystick_pins = {}
            if self._settings['logging'].v:
                print(f"Joy:Config failed {e}")


    def _is_stage_moving(self) -> bool:
        if self._stepperX is None or self._stepperY is None:
            return False
        return abs(self._stepperX.get_speed()) > 0 or abs(self._stepperY.get_speed()) > 0


    def _take_led_control(self):
        try:
            tildagonos.set_led_power(True)
            if PatternDisable is not None and not self._led_control_taken:
                eventbus.emit(PatternDisable())
                self._led_control_taken = True
        except Exception as e:
            if self._settings['logging'].v:
                print(f"LED:take failed {e}")


    def _release_led_control(self):
        try:
            for i in range(1, 13):
                tildagonos.leds[i] = _LED_OFF
            tildagonos.leds.write()
            if PatternEnable is not None and self._led_control_taken:
                eventbus.emit(PatternEnable())
                self._led_control_taken = False
        except Exception as e:
            if self._settings['logging'].v:
                print(f"LED:release failed {e}")


    def _set_led_mode(self, mode: str):
        self._led_mode = mode


    def _write_leds_color(self, color):
        for i in range(1, 13):
            tildagonos.leds[i] = color
        tildagonos.leds.write()


    def _update_leds(self, delta: int):
        mode = self._led_mode
        if self.current_state == STATE_ERROR:
            mode = LED_MODE_ERROR

        try:
            if mode == LED_MODE_MOVING:
                self._led_anim_elapsed += delta
                if self._led_anim_elapsed > 120:
                    self._led_anim_elapsed = 0
                    self._led_anim_phase = (self._led_anim_phase + 1) % 12
                for i in range(1, 13):
                    tildagonos.leds[i] = (0, 0, 24)
                tildagonos.leds[self._led_anim_phase + 1] = _LED_BLUE
                tildagonos.leds.write()
            elif mode != self._led_last_mode:
                if mode == LED_MODE_ERROR:
                    self._write_leds_color(_LED_RED)
                elif mode == LED_MODE_BUSY:
                    self._write_leds_color(_LED_YELLOW)
                elif mode == LED_MODE_INPUT:
                    self._write_leds_color(_LED_PURPLE)
                else:
                    self._write_leds_color(_LED_GREEN)
            self._led_last_mode = mode
        except Exception as e:
            if self._settings['logging'].v:
                print(f"LED:update failed {e}")


    def _minimise_with_cleanup(self):
        self._close_filename_dialog()
        if self._stepperX is not None:
            self._stepperX.stop()
            self._stepperX.enable(False)
        if self._stepperY is not None:
            self._stepperY.stop()
            self._stepperY.enable(False)
        self._release_led_control()
        self.minimise()


    def draw(self, ctx):
        if self._refresh or self.notification is not None:
            self._refresh = False
            clear_background(ctx)   
            ctx.save()
            ctx.font_size = label_font_size
            if ctx.text_align != ctx.LEFT:
                # See https://github.com/emfcamp/badge-2024-software/issues/181             
                ctx.text_align = ctx.LEFT
            ctx.text_baseline = ctx.BOTTOM            
            ctx.rgb(0,0,0).rectangle(-120,-120,240,240).fill()
            # Main screen content 
            if   self.current_state == STATE_WARNING:
                self.draw_message(ctx, ["XYStage requires","a custom","Motor Driver","hexpansion"], [(1,1,1),(1,1,0),(1,1,0),(1,1,0)], label_font_size)
            elif self.current_state == STATE_ERROR:
                self.draw_message(ctx, self.error_message, [(1,0,0)]*len(self.error_message), label_font_size)
            elif self.current_state == STATE_MESSAGE:
                self.draw_message(ctx, self.error_message, [(0,1,0)]*len(self.error_message), label_font_size)            
            elif self.current_state == STATE_XYSTAGE:
                self._draw_state_xystage(ctx)                
            elif self.current_state == STATE_GCODE_REPLAY:
                self._draw_state_gcode_replay(ctx)
            elif self.current_state == STATE_GCODE_RECORD:
                self._draw_state_gcode_record(ctx)
            elif self.current_state == STATE_SETTINGS:
                self.draw_message(ctx, ["Edit Setting",f"{self._edit_setting}:",f"{self._edit_setting_value}"], [(1,1,1),(0,0,1),(0,1,0)], label_font_size)
                button_labels(ctx, up_label="+", down_label="-", confirm_label="Set", cancel_label="Cancel", right_label="Default")
            ctx.restore()

        # These need to be drawn every frame as they contain animations
        if self.current_state == STATE_MENU or self.current_state == STATE_GCODE_FILES:
            clear_background(ctx)               
            if self.menu is not None:
                self.menu.draw(ctx)

        if self.notification:
            self.notification.draw(ctx)

        if self._filename_dialog is not None:
            self._filename_dialog.draw(ctx)

    def _draw_centered_title(self, ctx, title: str, y: int=-90, colour=(1, 1, 1)):
        # Use explicit width-based centering so titles remain correctly positioned
        # even if text alignment state drifts elsewhere in the render pipeline.
        ctx.text_align = ctx.LEFT
        ctx.font_size = label_font_size
        x = int(-(ctx.text_width(title) / 2))
        ctx.rgb(*colour).move_to(x, y).text(title)

    def _draw_state_xystage(self, ctx):
        # Draw outer rectangle for the XYStage based on the largest that can fit on the screen
        # top left of the rectangle is at -100,-100 i.e. Y is inverted
        ctx.rgb(0.3,0.3,0.3).rectangle(-_USABLE_X_PIXELS//2,-_USABLE_Y_PIXELS//2,_USABLE_X_PIXELS,_USABLE_Y_PIXELS).stroke()
        x,y   = self._scale_xystage(self.xystage['x'],-self.xystage['y'])
        sx,sy = self._scale_xystage(self._settings['width'].v,self._settings['height'].v)
        ctx.rgb(0.0,1.0,0.2).rectangle(x-(sx//2),y-(sy//2),sx,sy).fill()        
        # Draw a small black cross hair at the 'x','y' position
        ctx.rgb(0,0,0).move_to(x-10,y).line_to(x+10,y).stroke()
        ctx.rgb(0,0,0).move_to(x,y-10).line_to(x,y+10).stroke()
        # Display the x,y position of the stage in text underneath the stage
        if self.current_state == STATE_XYSTAGE:
            self._draw_centered_title(ctx, "XY Stage")
            ctx.rgb(1,1,1).move_to(-70, 100).text(f"{self.xystage['x']//32:5d}, {self.xystage['y']//32:5d}")
        #button_labels(ctx, confirm_label="Stop", cancel_label="Exit", left_label="<--", right_label="-->")


    def _draw_state_gcode_replay(self, ctx):
        self._draw_centered_title(ctx, "NC Replay")
        #if self._gcode_selected_file:
        #    short_name = self._gcode_selected_file[:-len(_GCODE_EXTENSION)]
        #    ctx.rgb(0.5, 0.8, 0.5).move_to(-95, -85).text(short_name[:14])

        history = self._gcode_history[-_GCODE_HISTORY_MAX:]
        next_start = self._gcode_current_index
        if self._gcode_current_command is not None:
            next_start += 1

        future = []
        if self._gcode_commands is not None and next_start < len(self._gcode_commands):
            for i in range(next_start, min(len(self._gcode_commands), next_start + _GCODE_FUTURE_MAX)):
                cmd = self._gcode_commands[i]
                future.append(cmd.get('text', cmd.get('cmd', '')))

        if len(history) == 0 and len(future) == 0:
            ctx.rgb(0.7, 0.7, 0.7).move_to(-60, -10).text("No commands")
        else:
            if len(history) > 1:
                base_y = 13 - label_font_size - (len(history)-2)*(small_font_size+2)
                for idx, line in enumerate(history[:-1]):
                    ctx.font_size = small_font_size
                    ctx.rgb(0.6, 0.6, 0.6).move_to(-100, base_y + idx*(small_font_size+2)).text(line[:20])
            ctx.font_size = label_font_size
            if len(history) > 0:        
                ctx.rgb(0.2, 1.0, 0.2).move_to(-105, 15).text(history[-1][:20])
            else:
                ctx.rgb(0.2, 1.0, 0.2).move_to(-105, 15).text("(waiting)")

            ctx.font_size = small_font_size
            future_y = 15
            for idx, line in enumerate(future):
                ctx.rgb(0.9, 0.9, 0.3).move_to(-100, future_y + (idx+1)*(small_font_size+2)).text(line[:18])
            

        if self._gcode_wait_button:
            button_labels(ctx, confirm_label="Go", cancel_label="Stop")
        else:
            button_labels(ctx, cancel_label="Stop")


    def _draw_state_gcode_record(self, ctx):
        self._draw_state_xystage(ctx)
        self._draw_centered_title(ctx, f"Record:{len(self._record_waypoints_mm)}", colour=(0, 1, 0))
        button_labels(ctx, confirm_label="Mark", cancel_label="Done")

    def _scale_xystage(self, x: int, y: int) -> (int, int):
        # scale x,y to the canvas range:
        # x,y are in the range -'XRange'/2 to 'XRange'/2 and -'YRange'/2 to 'YRange'/2
        x = int(_USABLE_X_PIXELS*x/(self._settings['XRange'].v + self._settings['width'].v))
        y = int(_USABLE_Y_PIXELS*y/(self._settings['YRange'].v + self._settings['height'].v))
        return x, y

    # Value increment/decrement functions for positive integers only
    def _inc(self, v: int, l: int):
        if l==0:
            return v+1
        else:
            d = 10**l
            v = ((v // d) + 1) * d   # round up to the next multiple of 10^l
            return v
    
    def _dec(self, v: int, l: int):
        if l==0:
            return v-1
        else:
            d = 10**l
            v = (((v+(9*(10**(l-1)))) // d) - 1) * d   # round down to the next multiple of 10^l
            return v


    def draw_message(self, ctx, message, colours, size=label_font_size):
        ctx.font_size = size
        num_lines = len(message)
        for i_num, instr in enumerate(message):
            text_line = str(instr)
            width = ctx.text_width(text_line)
            try:
                colour = colours[i_num]
            except IndexError:
                colour = None
            if colour is None:
                colour = (1,1,1)
            # Font is not central in the height allocated to it due to space for descenders etc...
            # this is most obvious when there is only one line of text
            # # position fine tuned to fit around button labels when showing 5 lines of text        
            y_position = int(0.35 * ctx.font_size) if num_lines == 1 else int((i_num-((num_lines-2)/2)) * ctx.font_size - 2)
            ctx.rgb(*colour).move_to(-width//2, y_position).text(text_line)

### MENU FUNCTIONALITY ###


    def set_menu(self, menu_name = "main"):  #: Literal["main"]): does it work without the type hint?
        if self._settings['logging'].v:
            print(f"H:Set Menu {menu_name}")
        if self.menu is not None:
            try:
                self.menu._cleanup()
            except:
                # See badge-2024-software PR#168
                # in case badge s/w changes and this is done within the menu s/w
                # and then access to this function is removed
                pass
            self.menu = None
        self.current_menu = menu_name
        # Prevent held buttons from triggering immediate actions in the next menu.
        if menu_name is not None:
            self.button_states.clear()
            self._auto_repeat_clear()
        if menu_name == "main":
            # construct the main menu based on template
            menu_items = _main_menu_items.copy()
            self.menu = Menu(
                    self,
                    menu_items,
                    select_handler=self._main_menu_select_handler,
                    back_handler=self._menu_back_handler,
                )            
        elif menu_name == "Settings":
            # construct the settings menu
            _settings_menu_items = ["SAVE ALL", "DEFAULT ALL"]
            for _, setting in enumerate(self._settings):
                _settings_menu_items.append(f"{setting}")
            self.menu = Menu(
                self,
                _settings_menu_items,
                select_handler=self._settings_menu_select_handler,
                back_handler=self._menu_back_handler,
                )
        elif menu_name == "GCODE Files":
            self.menu = Menu(
                self,
                self._gcode_files,
                select_handler=self._gcode_file_select_handler,
                back_handler=self._gcode_file_back_handler,
            )


    # this appears to be able to be called at any time
    def _main_menu_select_handler(self, item: str, idx: int):
        if self._settings['logging'].v:
            print(f"H:Main Menu {item} at index {idx}")
        if item == _main_menu_items[0]: # XYStage
            if self._ensure_steppers_ready(silent=False):
                self.set_menu(None)
                self.button_states.clear()
                self.current_state = STATE_XYSTAGE
                self._refresh = True
                self._auto_repeat_clear()
                self._stepperX.enable(True)
                self._stepperY.enable(True)
                self._time_since_last_input = 0
                self._set_led_mode(LED_MODE_IDLE)
        elif item == _main_menu_items[1]: # Home XY
            self._start_default_homing_sequence()
        elif item == _main_menu_items[2]: # Run NC
            self._enter_gcode_file_browser()
        elif item == _main_menu_items[3]: # Record NC
            if self._ensure_steppers_ready(silent=False):
                self.set_menu(None)
                self.button_states.clear()
                self.current_state = STATE_GCODE_RECORD
                self._refresh = True
                self._auto_repeat_clear()
                self._record_waypoints_mm = []
                self._record_capture_debounce = False
                self.long_press_delta = 0
                self._stepperX.enable(True)
                self._stepperY.enable(True)
                self._time_since_last_input = 0
        elif item == _main_menu_items[4]: # Settings
            self.notification = None
            self.button_states.clear()
            self.set_menu("Settings")
            self.current_state = STATE_MENU
            self._refresh = True
        elif item == _main_menu_items[5]: # About
            self.set_menu(None)
            self.button_states.clear()
            self.notification = None
            self.error_message = ["XYStage",f"Version: {_APP_VERSION}"]
            self.current_state = STATE_MESSAGE
            self._refresh = True   
        elif item == _main_menu_items[6]: # Exit
            self._release_led_control()
            eventbus.remove(HexpansionInsertionEvent, self._handle_hexpansion_insertion, self)
            eventbus.remove(HexpansionRemovalEvent, self._handle_hexpansion_removal, self)
            eventbus.emit(RequestStopAppEvent(self))

    def _settings_menu_select_handler(self, item: str, idx: int):
        if self._settings['logging'].v:
            print(f"H:Setting {item} @ {idx}")
        if idx == 0: #Save
            if self._settings['logging'].v:
                print("H:Settings Save All")
            settings.save()
            self.notification = Notification("Settings Saved")
            self.set_menu("main")
            self.button_states.clear()
        elif idx == 1: #Default
            if self._settings['logging'].v:
                print("H:Settings Default All")
            for s in self._settings:
                self._settings[s].v = self._settings[s].d
                self._settings[s].persist()
            self.notification = Notification("Settings Defaulted")

            self.set_menu("main")
            self.button_states.clear()
        else:
            self.set_menu(None)
            self.button_states.clear()
            self.current_state = STATE_SETTINGS
            self._refresh = True
            self._auto_repeat_clear()
            self._edit_setting = item
            self._edit_setting_value = self._settings[item].v


    def _menu_back_handler(self):
        self.button_states.clear()
        self._auto_repeat_clear()
        if self.current_menu == "main":
            self._minimise_with_cleanup()
            return
        # There are only two menus so this is the only other option    
        self.current_state = STATE_MENU
        self.set_menu("main")
        self._refresh = True


    # multi level auto repeat
    # if speed_up is True, the auto repeat gets faster the longer the button is held
    # otherwise it is a fixed rate, but the level is used to determine the scale of the increase in the setttings inc() and dec() functions
    def _auto_repeat_check(self, delta: int, speed_up: bool = True) -> bool:                
        self._auto_repeat += delta
        # multi stage auto repeat - the repeat gets faster the longer the button is held
        if self._auto_repeat > self._auto_repeat_intervals[self._auto_repeat_level if speed_up else 0]:
            self._auto_repeat = 0
            self._auto_repeat_count += 1
            # variable threshold to count to increase level so that it is not too easy to get to the highest level as the auto repeat period is reduced
            if self._auto_repeat_count > ((_AUTO_REPEAT_COUNT_THRES*_AUTO_REPEAT_MS) // self._auto_repeat_intervals[self._auto_repeat_level if speed_up else 0]):
                self._auto_repeat_count = 0
                if self._auto_repeat_level < (_AUTO_REPEAT_SPEED_LEVEL_MAX if speed_up else _AUTO_REPEAT_LEVEL_MAX):
                    self._auto_repeat_level += 1
                    if self._settings['logging'].v:
                        print(f"Auto Repeat Level: {self._auto_repeat_level}")

            return True
        return False


    def _auto_repeat_clear(self):                
        self._auto_repeat = 1+ self._auto_repeat_intervals[0] # so that we trigger immediately on next press 

        self._auto_repeat_count = 0 
        self._auto_repeat_level = 0









######## STEPPER MOTOR CLASS ########

class Stepper:  # External Driver DRV8825
    def __init__(self, container, pins, initial_pos: int = 0, reverse = False, name: str = "", max_sps_change: int = _STEPPER_MAX_ACCELERATION, max_sps: int = _STEPPER_MAX_SPEED, max_pos: int = _STEPPER_MAX_POSITION):
        try:
            self._container = container
            self._name: str = name
            self._reverse: bool = reverse                    # reverse direction of stepper at hardware level
            self._max_sps_change: int = int(max_sps_change)  # max change in speed in steps per second per update
            self._max_sps: int = int(max_sps)                # max speed in steps per second
            self._max_pos: int = int(max_pos)                # max position stored in steps        
            self._steps_per_sec: int = 0                     # current speed in steps per second
            self._calibrated: bool = False
            self._timer_mode: int = 0
            self._pos: int = initial_pos                     # current position in steps
            self._free_run_mode: int = 1                     # direction of free run mode
            self._enabled: bool = False
            self._freq: int = 0
            self._updates_per_sec: int = 10
            self._pwm: PWM = None
            self._pwm_safe_mode: bool = _PWM_SAFE_MODE_DEFAULT
            self._pwm_error_count: int = 0
            self._pwm_reinit_count: int = 0
            self._pwm_hw_retune_count: int = 0
            self._pwm_hw_fallback_count: int = 0
            self._pwm_hw_channel = None
            self._pwm_hw_timer = None

            # Pins for external stepper driver
            self._pins = pins
            self._pins["en"].init(mode=Pin.OUT)
            self._pins["en"].on()   # active low
            self._pins["dir"].init(mode=Pin.OUT)
            self._pins["dir"].off()
            self._pins["step"].init(mode=Pin.OUT)
            self._pins["step"].off()
            self._pins["stop"].init(mode=Pin.IN, pull=Pin.PULL_UP)
            self._pins["stop"].irq(trigger=Pin.IRQ_FALLING, handler=self._hit_endstop)
        except Exception as e:
            print(f"{self._name} Init failed:{e}")
 
  
    def get_speed_from_disance(self, distance: int) -> int:
        # function to calculate the speed that we need to reach the target as quickly as possible
        # subject to the max speed (max_sps) and acceleration (max_sps_change) settings, and the distance to the target and the current speed (sps)
        # s = ut + 0.5*at^2
        # v^2 = u^2 + 2as
        # calculate stopping distance at current speed
        # s = 0.5 * (u^2) / a
        current_sps = abs(int(self._steps_per_sec))
        min_sps = min(int(self._max_sps), int(self._container._settings['min_speed'].v))
        steps_to_stop = int((current_sps ** 2) / (2 * (self._max_sps_change * self._updates_per_sec)))
        print(f"{self._name}:{time.ticks_ms():6d}:Steps to stop:{steps_to_stop}/{distance}")
        if abs(distance) <= (steps_to_stop + (current_sps // self._updates_per_sec)):
            # if we are already close to the target, then stop
            # we can return min target speed as the speed control will enforce the max acceleration
            return min_sps
        elif abs(distance) <= (steps_to_stop + ((current_sps + 2*self._max_sps_change)//self._updates_per_sec)):
            # we can't affored to increase the speed as we will overshoot the target
            # If currently stationary, we still need to start moving toward target.
            if current_sps == 0:
                return min_sps
            return min(int(self._max_sps), current_sps)
        else:
            # we can increase the speed (if not already at max speed)
            return self._max_sps
    
    def speed(self,sps) -> bool:    # speed in FULL steps per second
        if self._free_run_mode == 1 and sps < 0:
            self._free_run_mode = -1
        elif self._free_run_mode == -1 and sps > 0:
            self._free_run_mode = 1
        speed_change_limited = False                
        if sps > 0:
            if self._calibrated and self._pos >= self._max_pos:
                # endstop reached
                sps = 0    
            else:
                if sps > self._max_sps:
                    # limit speed
                    sps = self._max_sps
                # limit acceleration by comparing the change in speed to the max acceleration
                # if the change is greater than the max acceleration, limit the change to the max acceleration
                if sps - self._steps_per_sec > self._max_sps_change:
                    sps = self._steps_per_sec + self._max_sps_change
                    speed_change_limited = True
                elif sps - self._steps_per_sec < -self._max_sps_change:
                    sps = self._steps_per_sec - self._max_sps_change
                    speed_change_limited = True
        else:
            if self._pins["stop"].value() == 0 or (self._calibrated and self._pos <= 0):
                # endstop reached
                sps = 0        
            else:
                if sps < -self._max_sps:
                    # limit speed
                    sps = -self._max_sps
                # limit acceleration by comparing the change in speed to the max acceleration
                # if the change is greater than the max acceleration, limit the change to the max acceleration
                if sps - self._steps_per_sec > self._max_sps_change:
                    sps = self._steps_per_sec + self._max_sps_change
                    speed_change_limited = True
                elif sps - self._steps_per_sec < -self._max_sps_change:
                    sps = self._steps_per_sec - self._max_sps_change
                    speed_change_limited = True
        self._steps_per_sec = int(sps)
        self._update_timer(abs(self._steps_per_sec))    # steps per second
        return speed_change_limited

    def get_speed(self) -> int:
        return self._steps_per_sec

    def set_max_sps(self, max_sps: int):
        self._max_sps = int(max_sps)

    def set_max_sps_change(self, max_sps_change: int):
        self._max_sps_change = int(max_sps_change)

    def set_pos(self, pos: int):
        self._pos = int(pos)

    def set_calibrated(self, calibrated: bool):
        self._calibrated = calibrated

    def is_endstop_pressed(self) -> bool:
        return self._pins["stop"].value() == 0

    def is_calibrated(self) -> bool:
        return self._calibrated

    # function to estimate the current position based on the speed and time since last update
    def get_pos(self, delta) -> int:
        steps = (self._steps_per_sec * delta) // 1000
        self._pos += steps
        # Check if we have hit the end stop
        if self._calibrated:
            if self._pos < 0 and self._steps_per_sec < 0:
                if self._container._settings['logging'].v:
                    print(f"{self._name} s/w min endstop")
                self.speed(0)
            elif self._pos > self._max_pos and self._steps_per_sec > 0:
                if self._container._settings['logging'].v:
                    print(f"{self._name} s/w max endstop")
                self.speed(0)        
        return self._pos 
        
    def _hit_endstop(self, pin: Pin):           
        # double check the endstop is hit
        # if not, ignore the interrupt
        if pin.value() == 0:  
            if self._container._settings['logging'].v:
                print(f"{self._name} Endstop - hit")
            self._calibrated = True
            self._pos = 0
            # if we are still trying to move TOWARDS the endstop 
            if self._steps_per_sec < 0:
                self.speed(0)
        else:
            print(f"{self._name} Endstop - false alarm")


    def _release_pwm(self):
        if self._pwm is None:
            return
        try:
            if hasattr(self._pwm, "deinit"):
                self._pwm.deinit()
            else:
                self._pwm.duty_ns(0)
        except Exception as e:
            if self._container._settings['logging'].v:
                print(f"{self._name} PWM release failed:{e}")
        self._pwm = None
        self._pwm_hw_channel = None
        self._pwm_hw_timer = None


    def _create_pwm(self, freq: int) -> bool:
        try:
            requested_freq = int(freq)
            create_freq = requested_freq
            sibling = self._get_sibling_stepper()
            if sibling is not None and sibling._pwm is not None and sibling._pwm_hw_timer is not None:
                if int(getattr(sibling, "_freq", 0)) == requested_freq:
                    create_freq = requested_freq + 1
                    if self._container._settings['logging'].v:
                        print(f"{self._name} PWM init offset:{requested_freq}->{create_freq}Hz to avoid shared timer")

            ledc_before = self._snapshot_ledc_output_map()
            pin = self._pins["step"]
            pin.init(mode=Pin.IN)
            self._pwm = PWM(pin, freq=create_freq)
            self._cache_pwm_hw_resource(ledc_before)
            if create_freq != requested_freq:
                # Return to requested frequency while keeping the newly allocated timer resource.
                self._retune_pwm_hw(requested_freq)
            self._pwm_reinit_count += 1
            if self._container._settings['logging'].v and (self._pwm_reinit_count <= 3 or (self._pwm_reinit_count % 50) == 0):
                mode = "safe" if self._pwm_safe_mode else "normal"
                print(f"{self._name} PWM init:{requested_freq}Hz mode={mode} count={self._pwm_reinit_count}")
            return True
        except Exception as e:
            self._pwm_error_count += 1
            print(f"{self._name} PWM failed:{e} freq={int(freq)}Hz errors={self._pwm_error_count}")
            self._pwm = None
            return False


    def _get_sibling_stepper(self):
        stepper_x = getattr(self._container, "_stepperX", None)
        stepper_y = getattr(self._container, "_stepperY", None)
        if stepper_x is self:
            return stepper_y
        if stepper_y is self:
            return stepper_x
        return None


    def _is_timer_shared_with_sibling(self):
        sibling = self._get_sibling_stepper()
        if sibling is None:
            return False
        if self._pwm_hw_timer is None or sibling._pwm_hw_timer is None:
            return False
        if self._pwm is None or sibling._pwm is None:
            return False
        return self._pwm_hw_timer == sibling._pwm_hw_timer


    def _get_step_pin_index(self):
        pin = self._pins.get("step")
        if pin is None:
            return None

        try:
            return int(pin)
        except Exception:
            pass

        for attr in ("id", "pin", "gpio", "_id", "_pin", "_gpio", "number", "idx", "index", "io_num", "gpio_num"):
            try:
                value = getattr(pin, attr)
                value = value() if callable(value) else value
                if isinstance(value, int):
                    return value
                if isinstance(value, str) and value.isdigit():
                    return int(value)
            except Exception:
                pass

        for text in (repr(pin), str(pin)):
            idx = self._extract_pin_index_from_text(text)
            if idx is not None:
                return idx

        return None


    def _extract_pin_index_from_text(self, text):
        if text is None:
            return None

        text = str(text)
        candidates = []
        run = ""
        start = 0

        for i, ch in enumerate(text):
            if '0' <= ch <= '9':
                if run == "":
                    start = i
                run += ch
            elif run != "":
                try:
                    value = int(run)
                    if 0 <= value <= _ESP32S3_GPIO_NUM_MAX:
                        candidates.append((value, start))
                except Exception:
                    pass
                run = ""
        if run != "":
            try:
                value = int(run)
                if 0 <= value <= _ESP32S3_GPIO_NUM_MAX:
                    candidates.append((value, start))
            except Exception:
                pass

        if not candidates:
            return None
        if len(candidates) == 1:
            return candidates[0][0]

        for marker in ("Pin(", "pin=", "pin:", "gpio=", "GPIO", "io="):
            marker_pos = text.find(marker)
            if marker_pos < 0:
                continue
            lower = marker_pos
            upper = marker_pos + len(marker) + 4
            for value, pos in candidates:
                if lower <= pos <= upper:
                    return value

        return None


    def _snapshot_ledc_output_map(self):
        mem32 = getattr(machine, "mem32", None)
        if mem32 is None:
            return None

        channel_to_pin = {}
        for pin_index in range(_ESP32S3_GPIO_NUM_MAX + 1):
            out_sel_reg = _ESP32S3_GPIO_FUNC0_OUT_SEL_CFG_REG + (pin_index * _ESP32S3_GPIO_FUNC_OUT_SEL_STRIDE)
            out_sel = mem32[out_sel_reg] & _ESP32S3_GPIO_FUNC_OUT_SEL_MASK
            if _ESP32S3_LEDC_LS_SIG_OUT0_IDX <= out_sel <= _ESP32S3_LEDC_LS_SIG_OUT7_IDX:
                channel_to_pin[out_sel - _ESP32S3_LEDC_LS_SIG_OUT0_IDX] = pin_index
        return channel_to_pin


    def _cache_pwm_hw_resource(self, ledc_before=None):
        mem32 = getattr(machine, "mem32", None)
        if mem32 is None:
            return

        ledc_after = self._snapshot_ledc_output_map()
        if ledc_after is None:
            return

        channel = None

        if ledc_before is not None:
            new_channels = [ch for ch in ledc_after if ch not in ledc_before]
            if len(new_channels) == 1:
                channel = new_channels[0]
            else:
                moved_channels = [ch for ch, pin_index in ledc_after.items() if ledc_before.get(ch) != pin_index]
                if len(moved_channels) == 1:
                    channel = moved_channels[0]

        if channel is None:
            pin_index = self._get_step_pin_index()
            if pin_index is not None:
                for ch, mapped_pin in ledc_after.items():
                    if mapped_pin == pin_index:
                        channel = ch
                        break

        if channel is None:
            return

        ch_conf0_reg = _ESP32S3_LEDC_LSCH0_CONF0_REG + (channel * _ESP32S3_LEDC_LSCH_CONF_STRIDE)
        timer = mem32[ch_conf0_reg] & _ESP32S3_LEDC_TIMER_SEL_MASK
        if timer > 3:
            return

        previous_channel = self._pwm_hw_channel
        previous_timer = self._pwm_hw_timer
        self._pwm_hw_channel = channel
        self._pwm_hw_timer = timer
        if self._container._settings['logging'].v:
            if previous_channel != channel or previous_timer != timer:
                print(f"{self._name} PWM hw cache:ch={channel} timer={timer}")
            if self._is_timer_shared_with_sibling():
                sibling = self._get_sibling_stepper()
                sibling_name = sibling._name if sibling is not None else "?"
                print(f"{self._name} PWM shared timer:{timer} with {sibling_name}")


    def _log_pwm_hw_fallback(self, detail: str):
        self._pwm_hw_fallback_count += 1
        if self._container._settings['logging'].v:
            if self._pwm_hw_fallback_count <= 3 or (self._pwm_hw_fallback_count % 50) == 0:
                print(f"{self._name} PWM hw fallback:{detail} count={self._pwm_hw_fallback_count}")


    def _resolve_pwm_hw_resource(self):
        mem32 = getattr(machine, "mem32", None)
        if mem32 is None:
            return None, None, None, "machine.mem32 unavailable"

        if self._pwm_hw_channel is not None and self._pwm_hw_timer is not None:
            if 0 <= self._pwm_hw_channel <= 7 and 0 <= self._pwm_hw_timer <= 3:
                timer_conf_reg = _ESP32S3_LEDC_LSTIMER0_CONF_REG + (self._pwm_hw_timer * _ESP32S3_LEDC_LSTIMER_CONF_STRIDE)
                return self._pwm_hw_channel, self._pwm_hw_timer, timer_conf_reg, None

        self._cache_pwm_hw_resource()
        if self._pwm_hw_channel is not None and self._pwm_hw_timer is not None:
            if 0 <= self._pwm_hw_channel <= 7 and 0 <= self._pwm_hw_timer <= 3:
                timer_conf_reg = _ESP32S3_LEDC_LSTIMER0_CONF_REG + (self._pwm_hw_timer * _ESP32S3_LEDC_LSTIMER_CONF_STRIDE)
                return self._pwm_hw_channel, self._pwm_hw_timer, timer_conf_reg, None

        pin_index = self._get_step_pin_index()
        if pin_index is None:
            return None, None, None, "step pin index unavailable"
        if pin_index < 0 or pin_index > _ESP32S3_GPIO_NUM_MAX:
            return None, None, None, f"invalid step pin:{pin_index}"

        out_sel_reg = _ESP32S3_GPIO_FUNC0_OUT_SEL_CFG_REG + (pin_index * _ESP32S3_GPIO_FUNC_OUT_SEL_STRIDE)
        out_sel = mem32[out_sel_reg] & _ESP32S3_GPIO_FUNC_OUT_SEL_MASK
        if out_sel < _ESP32S3_LEDC_LS_SIG_OUT0_IDX or out_sel > _ESP32S3_LEDC_LS_SIG_OUT7_IDX:
            return None, None, None, f"pin{pin_index} out_sel={out_sel} not LEDC"

        channel = out_sel - _ESP32S3_LEDC_LS_SIG_OUT0_IDX
        ch_conf0_reg = _ESP32S3_LEDC_LSCH0_CONF0_REG + (channel * _ESP32S3_LEDC_LSCH_CONF_STRIDE)
        timer = mem32[ch_conf0_reg] & _ESP32S3_LEDC_TIMER_SEL_MASK
        if timer > 3:
            return None, None, None, f"invalid timer sel:{timer}"

        timer_conf_reg = _ESP32S3_LEDC_LSTIMER0_CONF_REG + (timer * _ESP32S3_LEDC_LSTIMER_CONF_STRIDE)
        return channel, timer, timer_conf_reg, None


    def _apply_pwm_duty_hw(self, channel: int, duty_res: int, div_raw: int, src_hz: int, pulse_ns: int) -> bool:
        mem32 = getattr(machine, "mem32", None)
        if mem32 is None:
            return False
        if channel < 0 or channel > 7:
            return False
        if div_raw <= 0:
            return False

        if duty_res < _ESP32S3_LEDC_DUTY_RES_MIN or duty_res > _ESP32S3_LEDC_DUTY_RES_MAX:
            duty_res = _ESP32S3_LEDC_DUTY_RES_MAX

        precision = 1 << duty_res
        max_duty_int = max(0, precision - 1)

        pulse_ns = int(pulse_ns)
        if pulse_ns <= 0:
            duty_int = 0
        else:
            ticks_per_second = (int(src_hz) << _ESP32S3_LEDC_DIV_FRAC_BITS) // int(div_raw)
            duty_int = (pulse_ns * ticks_per_second + 999_999_999) // 1_000_000_000
            if duty_int < 1:
                duty_int = 1
            if duty_int > max_duty_int:
                duty_int = max_duty_int

        duty_raw = (int(duty_int) << _ESP32S3_LEDC_DUTY_INT_PART_SHIFT) & _ESP32S3_LEDC_DUTY_RAW_MASK
        ch_conf0_reg = _ESP32S3_LEDC_LSCH0_CONF0_REG + (channel * _ESP32S3_LEDC_LSCH_CONF_STRIDE)
        ch_duty_reg = ch_conf0_reg + _ESP32S3_LEDC_LSCH_DUTY_REG_OFFSET
        ch_conf1_reg = ch_conf0_reg + _ESP32S3_LEDC_LSCH_CONF1_REG_OFFSET

        mem32[ch_duty_reg] = (mem32[ch_duty_reg] & ~_ESP32S3_LEDC_DUTY_RAW_MASK) | duty_raw

        conf1 = mem32[ch_conf1_reg]
        conf1 = conf1 & ~(
            (_ESP32S3_LEDC_DUTY_NUM_MASK << _ESP32S3_LEDC_DUTY_NUM_SHIFT)
            | (_ESP32S3_LEDC_DUTY_CYCLE_MASK << _ESP32S3_LEDC_DUTY_CYCLE_SHIFT)
            | (_ESP32S3_LEDC_DUTY_SCALE_MASK << _ESP32S3_LEDC_DUTY_SCALE_SHIFT)
        )
        conf1 = conf1 | _ESP32S3_LEDC_DUTY_INC_BIT | _ESP32S3_LEDC_DUTY_START_BIT
        mem32[ch_conf1_reg] = conf1

        return True


    def _update_pwm_duty_hw(self, pulse_ns: int) -> bool:
        mem32 = getattr(machine, "mem32", None)
        if mem32 is None:
            return False

        channel, _, timer_conf_reg, err = self._resolve_pwm_hw_resource()
        if err is not None:
            return False

        timer_conf = mem32[timer_conf_reg]
        duty_res = timer_conf & _ESP32S3_LEDC_LSTIMER_DUTY_RES_MASK
        src_hz = _ESP32S3_LEDC_REF_TICK_HZ if (timer_conf & _ESP32S3_LEDC_LSTIMER_TICK_SEL_BIT) else _ESP32S3_LEDC_APB_HZ
        div_raw = (timer_conf >> _ESP32S3_LEDC_LSTIMER_CLK_DIV_SHIFT) & _ESP32S3_LEDC_LSTIMER_CLK_DIV_MASK
        if div_raw < _ESP32S3_LEDC_DIV_MIN:
            div_raw = _ESP32S3_LEDC_DIV_MIN
        elif div_raw > _ESP32S3_LEDC_DIV_MAX:
            div_raw = _ESP32S3_LEDC_DIV_MAX

        return self._apply_pwm_duty_hw(channel, duty_res, div_raw, src_hz, pulse_ns)


    def _retune_pwm_hw(self, freq: int) -> bool:
        target_hz = int(freq)
        if target_hz <= 0:
            return False

        mem32 = getattr(machine, "mem32", None)
        if mem32 is None:
            self._log_pwm_hw_fallback("mem32 unavailable")
            return False

        try:
            channel, timer, timer_conf_reg, err = self._resolve_pwm_hw_resource()
            if err is not None:
                self._log_pwm_hw_fallback(err)
                return False

            sibling = self._get_sibling_stepper()
            if sibling is not None and sibling._pwm is not None and sibling._pwm_hw_timer == timer:
                sibling_freq = int(getattr(sibling, "_freq", 0))
                if sibling_freq > 0 and sibling_freq != target_hz:
                    self._log_pwm_hw_fallback(
                        f"shared timer{timer} sibling={sibling._name} freq={sibling_freq} target={target_hz}"
                    )
                    return False

            timer_conf = mem32[timer_conf_reg]
            duty_res = timer_conf & _ESP32S3_LEDC_LSTIMER_DUTY_RES_MASK
            if duty_res < _ESP32S3_LEDC_DUTY_RES_MIN or duty_res > _ESP32S3_LEDC_DUTY_RES_MAX:
                duty_res = _ESP32S3_LEDC_DUTY_RES_MAX

            src_hz = _ESP32S3_LEDC_REF_TICK_HZ if (timer_conf & _ESP32S3_LEDC_LSTIMER_TICK_SEL_BIT) else _ESP32S3_LEDC_APB_HZ
            precision = 1 << duty_res
            denom = target_hz * precision
            if denom <= 0:
                self._log_pwm_hw_fallback(f"invalid divisor target={target_hz} duty={duty_res}")
                return False

            div_raw = (src_hz << _ESP32S3_LEDC_DIV_FRAC_BITS) // denom
            if div_raw < _ESP32S3_LEDC_DIV_MIN:
                div_raw = _ESP32S3_LEDC_DIV_MIN
            elif div_raw > _ESP32S3_LEDC_DIV_MAX:
                div_raw = _ESP32S3_LEDC_DIV_MAX

            timer_conf = timer_conf & ~(_ESP32S3_LEDC_LSTIMER_CLK_DIV_MASK << _ESP32S3_LEDC_LSTIMER_CLK_DIV_SHIFT)
            timer_conf = timer_conf | (div_raw << _ESP32S3_LEDC_LSTIMER_CLK_DIV_SHIFT)
            timer_conf = (timer_conf & ~_ESP32S3_LEDC_LSTIMER_DUTY_RES_MASK) | duty_res

            mem32[timer_conf_reg] = timer_conf
            mem32[timer_conf_reg] = timer_conf | _ESP32S3_LEDC_LSTIMER_PARA_UP_BIT

            if not self._apply_pwm_duty_hw(channel, duty_res, div_raw, src_hz, _STEPPER_STEP_PULSE_NS):
                self._log_pwm_hw_fallback("duty update failed")
                return False

            self._pwm_hw_retune_count += 1
            if self._pwm_hw_channel != channel or self._pwm_hw_timer != timer:
                self._pwm_hw_channel = channel
                self._pwm_hw_timer = timer
                if self._container._settings['logging'].v:
                    print(f"{self._name} PWM hw resource:ch={channel} timer={timer}")

            return True
        except Exception as e:
            self._log_pwm_hw_fallback(str(e))
            return False


    def _retune_pwm(self, freq: int) -> bool:
        if self._pwm is None:
            return self._create_pwm(freq)

        if self._retune_pwm_hw(freq):
            return True

        # Fall back to recreate path if hardware mapping is unavailable.
        self._pwm_safe_mode = True
        self._release_pwm()
        if self._create_pwm(freq):
            return True

        self._pwm_error_count += 1
        print(f"{self._name} PWM retune failed target={int(freq)}Hz prev={int(self._freq)}Hz mode={self._free_run_mode}/{self._timer_mode} errors={self._pwm_error_count}")
        return False

    def _update_timer(self,freq):
        freq = int(freq)
        if freq == 0:
            self._pins["en"].on()        # disable the stepper
            if self._pwm is not None:
                try:
                    if not self._update_pwm_duty_hw(0):
                        self._pwm.duty_ns(0)        # stop the PWM (frequency of 0 is not allowed)
                except Exception as e:
                    if self._container._settings['logging'].v:
                        print(f"{self._name} PWM stop failed:{e}")
                if self._pwm_safe_mode:
                    self._release_pwm()
            self._freq = 0   
        elif freq != self._freq or self._free_run_mode != self._timer_mode:
            try:                
                if self._container._settings['logging'].v:
                    print(f"{self._name} Timer:{self._free_run_mode} {freq}Hz")

                if self._pwm is None or freq != self._freq:
                    if not self._retune_pwm(freq):
                        return

                if self._free_run_mode>0:
                    self._pins["dir"].value(1 if self._reverse else 0)
                    if not self._update_pwm_duty_hw(_STEPPER_STEP_PULSE_NS):
                        self._pwm.duty_ns(_STEPPER_STEP_PULSE_NS)
                    self._pins["en"].off()    # enable active low
                elif self._free_run_mode<0:
                    self._pins["dir"].value(0 if self._reverse else 1)
                    if not self._update_pwm_duty_hw(_STEPPER_STEP_PULSE_NS):
                        self._pwm.duty_ns(_STEPPER_STEP_PULSE_NS)
                    self._pins["en"].off()    # enable active low
                else:
                    self._pins["en"].on()
                    if not self._update_pwm_duty_hw(0):
                        self._pwm.duty_ns(0)        # stop the PWM (frequency of 0 is not allowed)
                self._freq = freq
                self._timer_mode = self._free_run_mode
            except Exception as e:
                self._pwm_error_count += 1
                print(f"{self._name} update_timer failed:{e} target={int(freq)}Hz prev={int(self._freq)}Hz mode={self._free_run_mode}/{self._timer_mode} safe={self._pwm_safe_mode} errors={self._pwm_error_count}")


    def stop(self):
        self._update_timer(0)

    def enable(self,e = True):
        self._enabled=e
        self._pins["en"].value(not e)
        try:
            if e:
                if self._free_run_mode!=0:
                    self._update_timer(abs(self._steps_per_sec))   # steps per second                
            else:
                self._update_timer(0)
        except Exception as e:
            print(f"{self._name} enable failed:{e}")

    def is_enabled(self) -> bool:
        return self._enabled
    
########## END OF STEPPER CLASS ##########

class MySetting:
    def __init__(self, container, default, minimum, maximum):
        self._container = container
        self.d = default
        self.v = default
        self._min = minimum
        self._max = maximum


    def __str__(self):
        return str(self.v)


    def _index(self):
        for k,v in self._container.items():
            if v == self:
                return k
        return None

        
    # This returns an increase in the value passed in - subject to max and with scale of increase depending on level
    # based on the type of the setting
    # it does not affect the current value of the setting
    def inc(self, v, l=0):            
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l==0:
                v += 1
            else:
                d = 10**l
                v = ((v // d) + 1) * d   # round up to the next multiple of 10^l, being very careful not to cause big jumps when value was nearly at the next multiple 

            if v > self._max:
                v = self._max
        elif isinstance(self.v, float):
            # only float at present is brightness from 0.0 to 1.0
            v += 0.1            
            if v > self._max:
                v = self._max  
        elif self._container['logging'].v:
            print(f"H:inc type: {type(self.v)}")                               
        return v

    # This returns a decrease in the value passed in - subject to min and with scale of increase depending on level
    # based on the type of the setting
    # it does not affect the current value of the setting
    def dec(self, v, l=0):            
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l==0:
                v -= 1
            else:
                d = 10**l
                v = (((v+(9*(10**(l-1)))) // d) - 1) * d   # round down to the next multiple of 10^l

            if v < self._min:
                v = self._min       
        elif isinstance(self.v, float):
            # only float at present is brightness from 0.0 to 1.0
            v -= 0.1            
            if v < self._min:
                v = self._min
        elif self._container['logging'].v:
            print(f"H: dec type: {type(self.v)}") 
        return v
    

    def persist(self):
        # only save non-default settings to the settings store
        try:
            if self.v != self.d:
                settings.set(f"xystage.{self._index()}", self.v)
            else:
                settings.set(f"xystage.{self._index()}", None)
        except Exception as e:
            print(f"H:Failed to persist setting {self._index()}: {e}")


def parse_version(version):
    #pre_components = ["final"]
    #build_components = ["0", "000000z"]
    #build = ""
    components = []
    if "+" in version:
        version, build = version.split("+", 1)
        #build_components = build.split(".")
    if "-" in version:
        version, pre_release = version.split("-", 1)
        #if pre_release.startswith("rc"):
        #    # Re-write rc as c, to support a1, b1, rc1, final ordering
        #    pre_release = pre_release[1:]
        #pre_components = pre_release.split(".")
    version = version.strip("v").split(".")
    components = [int(item) if item.isdigit() else item for item in version]
    #components.append([int(item) if item.isdigit() else item for item in pre_components])
    #components.append([int(item) if item.isdigit() else item for item in build_components])
    return components

__app_export__ = XYStageApp
