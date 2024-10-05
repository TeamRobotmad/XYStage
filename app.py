import asyncio
import os
import time
from math import cos, pi
import ota
import settings
import vfs
from app_components.notification import Notification
from app_components.tokens import label_font_size, twentyfour_pt, clear_background, button_labels
from app_components import Menu
from events.input import BUTTON_TYPES, Button, Buttons, ButtonUpEvent
from frontboards.twentyfour import BUTTONS
from machine import PWM, Timer, Pin
from system.eventbus import eventbus
from system.hexpansion.events import (HexpansionInsertionEvent,
                                      HexpansionRemovalEvent)
from system.hexpansion.header import read_header
from system.hexpansion.config import HexpansionConfig
from system.scheduler import scheduler
from system.scheduler.events import (RequestStopAppEvent)

from tildagonos import tildagonos

import app

from .utils import chain, draw_logo_animated, parse_version

_APP_VERSION = "1.0" # XYStage App Version Number


# Stepper Tester - Defaults
_STEPPER_MAX_SPEED     = 1100*32    # full steps per second
_STEPPER_MIN_SPEED     = 20*32      # full steps per second
_STEPPER_MAX_POSITION  = 3100       # full steps from h/w endstop to s/w endstop at the other end

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

# App states where user can minimise app
_MINIMISE_VALID_STATES = [0, 1, 3, 4, 5]

# Hexpansion constants
_EEPROM_NUM_ADDRESS_BYTES = 2

XYSTAGE_HEXPANSION = 1  # Hexpansion slot for XYStage - as it does not have an EEPROM to be detected automatically
# Dedicated Pins - to drive an external stepper driver
X_DIR = 1   # ls pin (LSB)
X_ENABLE = 0  # ls pin (LSA) - active low
X_ENDSTOP = 3  # hs pin (HSG) - switch to ground
X_STEP = 0  # hs pin (HSF)
Y_DIR = 3   # ls pin (LSD)
Y_ENABLE = 2  # ls pin (LSC) - active low
Y_ENDSTOP = 1  # hs pin (HSI) - switch to ground
Y_STEP = 2  # hs pin (HSH)

_USABLE_X_PIXELS =  200
_USABLE_Y_PIXELS =  140
_WIDTH_DEFAULT   = (2000*32)
_HEIGHT_DEFAULT  = (2000*32)
_XRANGE_DEFAULT  = (2200*32) # Driver configured for 1/32 steps
_YRANGE_DEFAULT  = (2000*32) # Driver configured for 1/32 steps
POSITION_MATCH_TOLERANCE = 20

#Misceallaneous Settings
_LOGGING = True

# Menu Items
_main_menu_items = ["XYStage", "Settings", "About","Exit"]

class XYStageApp(app.App):
    def __init__(self):
        super().__init__()
        # UI Button Controls
        self.button_states = Buttons(self)
        self.last_press: Button = BUTTON_TYPES["CANCEL"]
        self.long_press_delta: int = 0
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
        self._HEXDRIVE_TYPES = [HexDriveType(0xCBCB, motors=2, servos=4), 
                                HexDriveType(0xCBCA, motors=2, name="2 Motor"), 
                                HexDriveType(0xCBCC, servos=4, name="4 Servo"), 
                                HexDriveType(0xCBCD, motors=1, servos=2, name="1 Mot 2 Srvo"),
                                HexDriveType(0xCBCE, steppers=1, name="Stepper")]  
        self.hexpansion_slot_type = [None]*6
        self.hexdrive_port: int = None
        self.ports_with_hexdrive = set()
        self.hexdrive_app = None
        eventbus.on_async(HexpansionInsertionEvent, self._handle_hexpansion_insertion, self)
        eventbus.on_async(HexpansionRemovalEvent, self._handle_hexpansion_removal, self)

        # Motor Driver
        self._hexpansion_config = HexpansionConfig(XYSTAGE_HEXPANSION)  # There is no EEPROM on the XYStage Hexpansion
        self.num_steppers: int = 2       # Default assumed for dedicated hardware
        self._stepperX: Stepper = None
        self._stepperY: Stepper = None
        self.xystage = {}
        self.xystage['x'] = 0
        self.xystage['y'] = 0
        self._keep_alive_period: int = 500                     # ms (half the value used in hexdrive.py)  
        self._timeout_period: int = 60*60000                   # ms (60 minutes)        

        # Overall app state (controls what is displayed and what user inputs are accepted)
        self.current_state = STATE_INIT
        self.previous_state = self.current_state
        if self._settings['logging'].v:
            print("XYStageApp:Init")


    ### ASYNC EVENT HANDLERS ###

    async def _handle_hexpansion_removal(self, event: HexpansionRemovalEvent):
        self.hexpansion_slot_type[event.port-1] = None
        if event.port in self.ports_with_hexdrive:
            self.ports_with_hexdrive.remove(event.port)
        if event.port == self.hexdrive_port:
            self.hexdrive_port = None
            self.hexdrive_app = None
            self.current_state = STATE_WARNING
            self.notification = Notification("HexDrive Removed")

    async def _handle_hexpansion_insertion(self, event: HexpansionInsertionEvent):
        if self.check_port_for_hexdrive(event.port):
            pass
    ### HEXPANSION FUNCTIONS ###

    # Scan the Hexpansion ports for EEPROMs and HexDrives in case they are already plugged in when we start
    def scan_ports(self):
        for port in range(1, 7):
            self.check_port_for_hexdrive(port)


    def check_port_for_hexdrive(self, port: int) -> bool:
        # we know the EEPROM address so we can just read the header directly
        if port not in range(1, 7):
            return False
        # We want to do this in two parts so that we detect if there is a valid EEPROM or not
        try:
            hexpansion_header = read_header(port, addr_len=_EEPROM_NUM_ADDRESS_BYTES)
        except OSError:
            # no EEPROM on this port
            return False
        except RuntimeError:
            # not a valid header
            if self._settings['logging'].v:
                print(f"H:Found EEPROM on port {port}")
            return True
        # check is this is a HexDrive header by scanning the _HEXDRIVE_TYPES list
        for index, hexpansion_type in enumerate(self._HEXDRIVE_TYPES):
            if hexpansion_header.vid == hexpansion_type.vid and hexpansion_header.pid == hexpansion_type.pid:
                if self._settings['logging'].v:
                    print(f"H:Found '{hexpansion_type.name}' HexDrive on port {port}")
                if port not in self.ports_with_hexdrive:
                    self.ports_with_hexdrive.add(port)
                self.hexpansion_slot_type[port-1] = index
                return True
        # we are not interested in this type of hexpansion
        return False


    def find_hexdrive_app(self, port: int) -> app:                    
        for an_app in scheduler.apps:
            if type(an_app).__name__ is 'HexDriveApp':
                if hasattr(an_app, "config") and hasattr(an_app.config, "port") and  an_app.config.port == port:
                    return an_app
        return None


    def update_settings(self):
        for s in self._settings:
            self._settings[s].v = settings.get(f"xystage.{s}", self._settings[s].d)


    ### MAIN APP CONTROL FUNCTIONS ###

    def update(self, delta: int):
        if self.notification:
            self.notification.update(delta)

        if self.current_state == STATE_INIT:
            # One Time initialisation
            self.scan_ports()
            if (len(self.ports_with_hexdrive) == 0):
                # There are currently no possible HexDrives plugged in
                self.current_state = STATE_WARNING
            else:
                # We have a HexDrive so we can start the main app
                # remember which port it is on
                self.hexdrive_port = list(self.ports_with_hexdrive)[0]
                self.hexdrive_app = self.find_hexdrive_app(self.hexdrive_port)
                self.current_state = STATE_MENU
            self.current_state = STATE_MENU # NO HEXDRIVE REQUIRED FOR XYSTAGE AT PRESENT
        
        self._update_main_application(delta)

        if self.current_state != self.previous_state:
            if self._settings['logging'].v:
                print(f"State: {self.previous_state} -> {self.current_state}")
            self.previous_state = self.current_state
            # something has changed - so worth redrawing
            self._refresh = True


    def _update_main_application(self, delta: int):
        if self.current_state == STATE_MENU:
            if self.current_menu is None:
                self.set_menu("main")
                self._refresh = True
            else:
                self.menu.update(delta)    
                if self.menu.is_animating != "none":
                    if self._settings['logging'].v:
                        print("Menu is animating")
                    self._refresh = True
        elif self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in _MINIMISE_VALID_STATES:
            self.button_states.clear()
            self.minimise()

    ### XY Stage Application ###
        elif self.current_state == STATE_XYSTAGE:
            self._update_state_xystage(delta)

    ### Settings Capability ###
        elif self.current_state == STATE_SETTINGS:
            self._update_state_settings(delta)
    ### End of Update ###

    def _get_speed_from_disance(self, distance: int) -> int:
        # calculate the speed required to move the distance in 2 second
        # subject to obeying the min and max speed limits
        speed = int((distance) // 2)
        return max(self._settings['min_speed'].v, min(speed, self._settings['max_speed'].v))


    # Stepper Tester:
    def _update_state_xystage(self, delta: int):
        self.xystage['x'] = self._stepperX.get_pos(delta) - self._settings['XRange'].v//2        
        self.xystage['y'] = self._stepperY.get_pos(delta) - self._settings['YRange'].v//2        
        # Left/Right to adjust position
        pressed = False
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            # if CONFIRM pressed then go to position 0,0
            pressed = True
            # if current position is not close to 0,0 then go to 0,0
            # check each of X & Y independently
            # if value is too high then apply -ve speed
            # if value is too low then apply +ve speed
            # subject to the min and max speed limits
            if self.xystage['x'] > (0 + POSITION_MATCH_TOLERANCE):
                self._stepperX.speed(-self._get_speed_from_disance(abs(self.xystage['x'])))
            elif self.xystage['x'] < (0 - POSITION_MATCH_TOLERANCE):
                self._stepperX.speed(self._get_speed_from_disance(abs(self.xystage['x'])))
            else:
                self._stepperX.speed(0)
            if self.xystage['y'] > (0 + POSITION_MATCH_TOLERANCE):
                self._stepperY.speed(-self._get_speed_from_disance(abs(self.xystage['y'])))
            elif self.xystage['y'] < (0 - POSITION_MATCH_TOLERANCE):
                self._stepperY.speed(self._get_speed_from_disance(abs(self.xystage['y'])))
            else:
                self._stepperY.speed(0)
            self._refresh = True
        else:
            if self.button_states.get(BUTTON_TYPES["RIGHT"]):
                pressed = True
                if self._auto_repeat_check(delta, False):
                    speed = abs(self._stepperX.get_speed())
                    # estimate the amount of movement based on the speed and time since last update         
                    speed = max(self._settings['min_speed'].v, self._inc(speed, 1 + self._auto_repeat_level))
                    self._stepperX.speed(speed)              
                    self._refresh = True
            elif self.button_states.get(BUTTON_TYPES["LEFT"]):
                pressed = True
                if self._auto_repeat_check(delta, False):
                    speed = abs(self._stepperX.get_speed())            
                    speed = max(self._settings['min_speed'].v, self._inc(speed, 1 + self._auto_repeat_level))
                    self._stepperX.speed(-speed)
                    self._refresh = True
            elif self._stepperX.speed(0):
                self._refresh = True
            if self.button_states.get(BUTTON_TYPES["UP"]):
                pressed = True
                if self._auto_repeat_check(delta, False):
                    speed = abs(self._stepperY.get_speed())            
                    speed = max(self._settings['min_speed'].v, self._inc(speed, 1 + self._auto_repeat_level))
                    self._stepperY.speed(speed)
                    self._refresh = True
            elif self.button_states.get(BUTTON_TYPES["DOWN"]):
                pressed = True
                if self._auto_repeat_check(delta, False):
                    speed = abs(self._stepperY.get_speed())            
                    speed = max(self._settings['min_speed'].v, self._inc(speed, 1 + self._auto_repeat_level))
                    self._stepperY.speed(-speed)
                    self._refresh = True
            elif self._stepperY.speed(0):
                self._refresh = True    
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
                    self.notification = Notification("  Stepper:\n Timeout")
                    if self._settings['logging'].v:
                        print("Stepper:Timeout")          

        if self._refresh and self._settings['logging'].v:               
            print(f"X:{self.xystage['x']} Y:{self.xystage['y']}")


    def _update_state_settings(self, delta: int):    
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
                self.set_menu(_main_menu_items[3])
                self.current_state = STATE_MENU
            elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.button_states.clear()
                # set setting
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} = {self._edit_setting_value}")
                self._settings[self._edit_setting].v = self._edit_setting_value
                self._settings[self._edit_setting].persist()
                self.notification = Notification(f"  Setting:   {self._edit_setting}={self._edit_setting_value}")
                self.set_menu(_main_menu_items[3])
                self.current_state = STATE_MENU


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
                self.draw_message(ctx, ["XYStage requires","HexDrive hexpansion","from RobotMad","github.com","/TeamRobotmad","/XYStage"], [(1,1,1),(1,1,0),(1,1,0),(1,1,1),(1,1,1),(1,1,1)], label_font_size)
            elif self.current_state == STATE_ERROR:
                self.draw_message(ctx, self.error_message, [(1,0,0)]*len(self.error_message), label_font_size)
            elif self.current_state == STATE_MESSAGE:
                self.draw_message(ctx, self.error_message, [(0,1,0)]*len(self.error_message), label_font_size)            
            elif self.current_state == STATE_XYSTAGE:
                self._draw_state_xystage(ctx)                
            elif self.current_state == STATE_SETTINGS:
                self.draw_message(ctx, ["Edit Setting",f"{self._edit_setting}:",f"{self._edit_setting_value}"], [(1,1,1),(0,0,1),(0,1,0)], label_font_size)
                button_labels(ctx, up_label="+", down_label="-", confirm_label="Set", cancel_label="Cancel", right_label="Default")
            ctx.restore()

        # These need to be drawn every frame as they contain animations
        if self.current_state == STATE_MENU:
            clear_background(ctx)               
            self.menu.draw(ctx)

        if self.notification:
            self.notification.draw(ctx)

    def _draw_state_xystage(self, ctx):
        ctx.rgb(1,1,1).move_to(-80, -100).text("XY Stage")
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
        ctx.rgb(1,1,1).move_to(-70, 100).text(f"{self.xystage['x']//32:5d}, {self.xystage['y']//32:5d}")
        #button_labels(ctx, confirm_label="Stop", cancel_label="Exit", left_label="<--", right_label="-->")

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
        self.current_menu = menu_name
        if menu_name == "main":
            # construct the main menu based on template
            menu_items = _main_menu_items.copy()
            if self.num_steppers == 0:
                menu_items.remove(_main_menu_items[0])   
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


    # this appears to be able to be called at any time
    def _main_menu_select_handler(self, item: str, idx: int):
        if self._settings['logging'].v:
            print(f"H:Main Menu {item} at index {idx}")
        if item == _main_menu_items[0]: # XYStage
            if self.num_steppers == 0:
                self.notification = Notification("Hexpansion Missing")
                if self._settings['logging'].v:
                    print("No Hexpansion")
            else:
                if self._stepperX is None or self._stepperY is None:
                    # try timer IDs 0-3 until one is free
                    for i in range(4):
                        if self._stepperX is None:
                            try:
                                # Pins                                
                                pins = {}
                                pins["dir"]  = self._hexpansion_config.ls_pin[X_DIR]
                                pins["en"]   = self._hexpansion_config.ls_pin[X_ENABLE]
                                pins["step"] = self._hexpansion_config.pin[X_STEP]
                                pins["stop"] = self._hexpansion_config.pin[X_ENDSTOP]
                                self._stepperX = Stepper(self, pins, reverse = True, name = "X", max_sps = self._settings['max_speed'].v, max_pos=self._settings['XRange'].v)
                                if self._settings['logging'].v:
                                    print(f"StepperX:Init {i}")
                                continue
                            except:
                                print(f"StepperX:Init {i} Failed")
                                pass
                        elif self._stepperY is None:
                            try:
                                # Pins                                
                                pins = {}
                                pins["dir"]  = self._hexpansion_config.ls_pin[Y_DIR]
                                pins["en"]   = self._hexpansion_config.ls_pin[Y_ENABLE]
                                pins["step"] = self._hexpansion_config.pin[Y_STEP]
                                pins["stop"] = self._hexpansion_config.pin[Y_ENDSTOP]                                
                                self._stepperY = Stepper(self, pins, name = "Y", max_sps = self._settings['max_speed'].v, max_pos=self._settings['YRange'].v)
                                if self._settings['logging'].v:
                                    print(f"StepperY:Init {i}")
                                # Start off assuming stage is in last known position
                                continue
                            except:
                                print(f"StepperY:Init {i} Failed")
                                pass
                        else:
                            break
                if self._stepperX is None or self._stepperY is None:
                    self.notification = Notification("No Free Timers")
                else:
                    self.set_menu(None)
                    self.button_states.clear()                    
                    self.current_state = STATE_XYSTAGE 
                    self._refresh = True
                    self._auto_repeat_clear()
                    self._stepperX.enable(True)
                    self._stepperY.enable(True)
                    self._time_since_last_input = 0                                       
        elif item == _main_menu_items[1]: # Settings
            self.set_menu(_main_menu_items[3])
        elif item == _main_menu_items[2]: # About
            self.set_menu(None)
            self.button_states.clear()
            self.error_message = ["XYStage","Version: 1.0"]
            self.current_state = STATE_MESSAGE
            self._refresh = True   
        elif item == _main_menu_items[3]: # Exit
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
            self.notification = Notification("  Settings  Saved")
            self.set_menu("main")
        elif idx == 1: #Default
            if self._settings['logging'].v:
                print("H:Settings Default All")
            for s in self._settings:
                self._settings[s].v = self._settings[s].d
                self._settings[s].persist()
            self.notification = Notification("  Settings Defaulted")

            self.set_menu("main")
        else:
            self.set_menu(None)
            self.button_states.clear()
            self.current_state = STATE_SETTINGS
            self._refresh = True
            self._auto_repeat_clear()
            self._edit_setting = item
            self._edit_setting_value = self._settings[item].v


    def _menu_back_handler(self):
        if self.current_menu == "main":
            self.minimise()
        # There are only two menus so this is the only other option    
        self.set_menu("main")


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
    def __init__(self, container, pins, reverse = False, name: str = "", max_sps: int = _STEPPER_MAX_SPEED, max_pos: int = _STEPPER_MAX_POSITION):
        self._container = container
        self._name = name 
        self._calibrated = False
        #self._timer = Timer(timer_id)
        self._timer_is_running = False
        self._timer_mode = 0
        self._pos = 0                               # current position in steps
        self._free_run_mode = 1                     # direction of free run mode
        self._enabled = False
        self._reverse = reverse
        self._max_sps_change = int(max_sps/10)      # max change in speed in steps per second per update
        self._max_sps = int(max_sps)                # max speed in steps per second
        self._steps_per_sec = 0                     # current speed in steps per second
        self._max_pos = int(max_pos)                # max position stored in half steps
        self._freq = 0

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

        # Setup PWM output on the step pin
        try:
            self._pwm = PWM(self._pins["step"], freq=10, duty_ns=2000)    # 0 Hz is invalid but 0 duty is allowed
        except Exception as e:
            print(f"{self._name} PWM failed:{e}")
  
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

    # function to estimate the current position based on the speed and time since last update
    def get_pos(self, delta) -> int:
        steps = (self._steps_per_sec * delta) // 1000
        self._pos += steps
        # Check if we have hit the end stop
        if self._calibrated:
            if self._pos < 0 and self._steps_per_sec < 0:
                if self._settings['logging'].v:
                    print(f"{self._name} s/w min endstop")
                self.speed(0)
            elif self._pos > self._max_pos and self._steps_per_sec > 0:
                if self._settings['logging'].v:
                    print(f"{self._name} s/w max endstop")
                self.speed(0)        
        return self._pos 
        
    def _hit_endstop(self, pin: Pin):           
        # double check the endstop is hit
        # if not, ignore the interrupt
        if pin.value() == 0:  
            if self._settings['logging'].v:
                print(f"{self._name} Endstop - hit")
            if not self._calibrated:
                self._calibrated = True
                self._pos = 0
            # if we are still trying to move TOWARDS the endstop 
            if self._steps_per_sec < 0:
                self.speed(0)
        else:
            print(f"{self._name} Endstop - false alarm")

    def _update_timer(self,freq):
        if freq == 0:
            self._pins["en"].on()        # disable the stepper
            self._pwm.duty_ns(0)        # stop the PWM (frequency of 0 is not allowed)
            self._freq = 0   
        elif freq != self._freq or self._free_run_mode != self._timer_mode:
            try:                
                if self._settings['logging'].v:
                    print(f"{self._name} Timer:{self._free_run_mode} {freq}Hz")
                if self._free_run_mode>0:
                    self._pins["dir"].value(1 if self._reverse else 0)
                    self._pwm.freq(int(freq))
                    self._pwm.duty_ns(2000)     # minimum 1.9uS STEP pulse width for DRV8825                     
                    self._pins["en"].off()    # enable active low
                elif self._free_run_mode<0:
                    self._pins["dir"].value(0 if self._reverse else 1)
                    self._pwm.freq(int(freq))
                    self._pwm.duty_ns(2000)     # minimum 1.9uS STEP pulse width for DRV8825                      
                    self._pins["en"].off()    # enable active low
                else:
                    self._pins["en"].on()
                    self._pwm.duty_ns(0)        # stop the PWM (frequency of 0 is not allowed)   
                self._freq = freq
                self._timer_is_running=True
                self._timer_mode = self._free_run_mode
            except Exception as e:
                print(f"{self._name} update_timer failed:{e}")


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

class HexDriveType:
    def __init__(self, pid, vid = 0xCAFE, motors = 0, steppers = 0, servos = 0, name ="Unknown"):
        self.vid = vid
        self.pid = pid
        self.name = name
        self.motors = motors
        self.servos = servos
        self.steppers = steppers


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

__app_export__ = XYStageApp
