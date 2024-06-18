import asyncio
import os
import time

import settings
import vfs
from app_components.notification import Notification
from app_components.tokens import label_font_size, twentyfour_pt
from events.input import BUTTON_TYPES, Button, Buttons, ButtonUpEvent
from frontboards.twentyfour import BUTTONS
from machine import I2C
from system.eventbus import eventbus
from system.hexpansion.events import (HexpansionInsertionEvent,
                                      HexpansionMountedEvent,
                                      HexpansionRemovalEvent)
from system.hexpansion.header import HexpansionHeader
from system.hexpansion.util import get_hexpansion_block_devices
from system.patterndisplay.events import PatternDisable, PatternEnable
from system.scheduler import scheduler
from system.scheduler.events import (RequestForegroundPopEvent,
                                     RequestForegroundPushEvent)
from tildagonos import tildagonos
from math import pi, cos
import app

from .utils import chain, draw_logo_animated
from .uQR import QRCode

# Hard coded to talk to 16bit address EEPROM on address 0x50 - because we know that is what is on the HexDrive Hexpansion
# makes it a lot more efficient than scanning the I2C bus for devices and working out what they are

CURRENT_APP_VERSION = 2648 # Integer Version Number - checked against the EEPROM app.py version to determine if it needs updating

_URL = "https://robotmad.odoo.com" # URL for QR code

# Screen positioning for movement sequence text
VERTICAL_OFFSET = label_font_size
H_START = -78
V_START = -58
_BRIGHTNESS = 1.0

# Motor Driver - Defaults
_MAX_POWER = 65535
_POWER_STEP_PER_TICK = 7500  # effectively the acceleration

# Timings
_TICK_MS       =  10 # Smallest unit of change for power, in ms
_USER_DRIVE_MS =  50 # User specifed drive durations, in ms
_USER_TURN_MS  =  20 # User specifed turn durations, in ms
_LONG_PRESS_MS = 750 # Time for long button press to register, in ms
_RUN_COUNTDOWN_MS = 5000 # Time after running program until drive starts, in ms

# App states
STATE_INIT = -1
STATE_WARNING = 0
STATE_MENU = 1
STATE_RECEIVE_INSTR = 2
STATE_COUNTDOWN = 3
STATE_RUN = 4
STATE_DONE = 5
STATE_WAIT = 6            # Between Hexpansion initialisation and upgrade steps  
STATE_DETECTED = 7        # Hexpansion ready for EEPROM initialisation
STATE_UPGRADE = 8         # Hexpansion ready for EEPROM upgrade
STATE_PROGRAMMING = 9     # Hexpansion EEPROM programming
STATE_REMOVED = 10        # Hexpansion removed
STATE_ERROR = 11          # Hexpansion error
STATE_LOGO = 12           # Logo display

# App states where user can minimise app
MINIMISE_VALID_STATES = [0, 1, 2, 5, 6, 7, 8, 10, 11, 12]

# HexDrive Hexpansion constants
_EEPROM_ADDR  = 0x50
_EEPROM_NUM_ADDRESS_BYTES = 2
_EEPROM_PAGE_SIZE = 32
_HEXDRIVE_VID = 0xCAFE
_HEXDRIVE_PID = 0xCBCB

hexdrive_header = HexpansionHeader(
    manifest_version="2024",
    fs_offset=32,
    eeprom_page_size=_EEPROM_PAGE_SIZE,
    eeprom_total_size=64 * 1024 // 8,
    vid=_HEXDRIVE_VID,
    pid=_HEXDRIVE_PID,
    unique_id=0x0,
    friendly_name="HexDrive",
)

class BadgeBotApp(app.App):
    def __init__(self):
        super().__init__()
        self.button_states = Buttons(self)
        self.last_press: Button = BUTTON_TYPES["CANCEL"]
        self.long_press_delta = 0

        # UI Featrue Controls
        self.rpm = 5                    # logo rotation speed in RPM
        self.animation_counter = 0
        qr = QRCode(error_correction=1, box_size=10, border=4)
        qr.add_data(_URL)
        self.qr_code = qr.get_matrix()
        self.b_msg = "BadgeBot"
        self.t_msg = "RobotMad"
        self.is_scroll = False
        self.scroll_offset = 0
        self.notification = None
        self.error_message = []
        self.we_have_focus = False

        # BadgeBot Control Sequence Variables
        self.run_countdown_elapsed_ms = 0
        self.instructions = []
        self.current_instruction = None
        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])

        # Settings
        self._settings = {}
        self._settings['acceleration']  = _POWER_STEP_PER_TICK
        self._settings['max_power']     = _MAX_POWER
        self._settings['drive_step_ms'] = _USER_DRIVE_MS
        self._settings['turn_step_ms']  = _USER_TURN_MS
        self._settings['brightness']    = _BRIGHTNESS
        self.update_settings()   

        # Hexpansion related
        self.hexdrive_seen = False
        self.detected_port = None
        self.upgrade_port = None
        self.ports_with_blank_eeprom = set()
        self.ports_with_hexdrive = set()
        self.ports_with_upgraded_hexdrive = set()
        self.hexdrive_app = None
        self.hexdrive_port = None
        eventbus.on_async(HexpansionInsertionEvent, self.handle_hexpansion_insertion, self)
        eventbus.on_async(HexpansionRemovalEvent, self.handle_hexpansion_removal, self)
        eventbus.on_async(HexpansionMountedEvent, self.handle_hexpansion_mounted, self)

        # Overall app state (controls what is displayed and what user inputs are accepted)
        self.current_state = STATE_INIT

        eventbus.on_async(RequestForegroundPushEvent, self.gain_focus, self)
        eventbus.on_async(RequestForegroundPopEvent, self.lose_focus, self)
        eventbus.on_async(ButtonUpEvent, self.handle_button_up, self)

        # We start with focus on launch, without an event emmited
        self.gain_focus(RequestForegroundPushEvent(self))
   

    ### ASYNC EVENT HANDLERS ###

    async def handle_hexpansion_mounted(self, event: HexpansionMountedEvent):
        print(f"H:Mounted {event.port} at {event.mountpoint}")

    async def handle_hexpansion_removal(self, event: HexpansionRemovalEvent):
        if event.port in self.ports_with_blank_eeprom:
            print(f"H:EEPROM removed from port {event.port}")
            self.ports_with_blank_eeprom.remove(event.port)
        if event.port in self.ports_with_hexdrive:
            print(f"H:HexDrive removed from port {event.port}")
            self.ports_with_hexdrive.remove(event.port)
        if event.port in self.ports_with_upgraded_hexdrive:
            print(f"H:HexDrive removed from port {event.port}")
            self.ports_with_upgraded_hexdrive.remove(event.port)
        if self.current_state == STATE_DETECTED and event.port == self.detected_port:
            self.current_state = STATE_WAIT
        elif self.current_state == STATE_UPGRADE and event.port == self.upgrade_port:
            self.current_state = STATE_WAIT
        elif self.hexdrive_port is not None and event.port == self.hexdrive_port:
            self.current_state = STATE_WAIT

    async def handle_hexpansion_insertion(self, event: HexpansionInsertionEvent):
        if self.check_port_for_hexdrive(event.port):
            self.current_state = STATE_WAIT

    async def gain_focus(self, event: RequestForegroundPushEvent):
        if event.app is self:
            self.we_have_focus = True
            eventbus.emit(PatternDisable())
            self.clear_leds()

    async def lose_focus(self, event: RequestForegroundPopEvent):
        if event.app is self:
            self.we_have_focus = False
            eventbus.emit(PatternEnable())

    async def background_task(self):
        # Modiifed background task loop for shorter sleep time
        last_time = time.ticks_ms()
        while True:
            cur_time = time.ticks_ms()
            delta_ticks = time.ticks_diff(cur_time, last_time)
            self.background_update(delta_ticks)
             # If we want to be kind we could make this variable depending on app state
             # i.e. on transition into run set this lower
            await asyncio.sleep(0.01)
            last_time = cur_time

    async def handle_button_up(self, event: ButtonUpEvent):
        if self.current_state == STATE_RECEIVE_INSTR and event.button == BUTTONS["C"]:
            self.is_scroll = not self.is_scroll
            state = "yes" if self.is_scroll else "no"
            self.notification = Notification(f"Scroll {state}")


    ### NON-ASYNC FUCNTIONS ###

    def background_update(self, delta):
        if self.current_state == STATE_RUN:
            power = self.get_current_power_level(delta)
            if power is None:
                self.current_state = STATE_DONE
            else:
                self.hexdrive_app.set_pwm(power)


    ### HEXPANSION FUNCTIONS ###

    # Scan the Hexpansion ports for EEPROMs and HexDrives in case they are already plugged in when we start
    def scan_ports(self):
        for port in range(1, 7):
            self.check_port_for_hexdrive(port)

    def check_port_for_hexdrive(self, port) -> bool:
        # avoiding use of read_hexpansion_header as this triggers a full i2c scan each time
        # we know the EEPROM address so we can just read the header directly
        if port not in range(1, 7):
            return False
        try:
            i2c = I2C(port)
            i2c.writeto(_EEPROM_ADDR, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES))  # Read header @ address 0                
            header_bytes = i2c.readfrom(_EEPROM_ADDR, 32)
        except OSError:
            # no EEPROM on this port
            #print(f"H:No compatible EEPROM on port {port}")
            return False
        try:
            read_header = HexpansionHeader.from_bytes(header_bytes)
        except Exception:
            # not a valid header
            print(f"H:Found EEPROM on port {port}")
            self.ports_with_blank_eeprom.add(port)
            return True
        if read_header.vid == _HEXDRIVE_VID and read_header.pid == _HEXDRIVE_PID:
            print(f"H:Found HexDrive on port {port}")
            self.ports_with_hexdrive.add(port)
            return True
        # we are not interested in this type of hexpansion
        return False

    def get_app_version_in_eeprom(self, port, header, i2c, addr) -> int:
        try:
            _, partition = get_hexpansion_block_devices(i2c, header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return 0
        version = 0
        already_mounted = False # if hexpansion file system was already mounted then we will not unmount it
        mountpoint = '/hexpansion_' + str(port)
        try:
            vfs.mount(partition, mountpoint, readonly=True)
            print(f"H:Mounted {partition} at {mountpoint}")
        except OSError as e:
            if e.args[0] == 1:
                already_mounted = True
            else:
                print(f"H:Error mounting: {e}")
        except Exception as e:
            print(f"H:Error mounting: {e}")
        print("H:Reading app.mpy")
        try:
            appfile = open(f"{mountpoint}/app.mpy", "rb")
            app_mpy = appfile.read()
            appfile.close()
        except OSError as e:
            if e.args[0] == 2:
                # file does not exist 
                print("H:No app.mpy found")
            else:    
                print(f"H:Error reading HexDrive app.mpy: {e}")
        except Exception as e:
            print(f"H:Error reading HexDrive app.mpy: {e}")            
        try:
            #version = app.split("APP_VERSION = ")[1].split("\n")[0]
            # TODO - means of identifying the version number in the app.mpy file 
            # quick hack - lets use the length of the file as a version number
            version = len(app_mpy)
        except Exception as e:
            version = 0
        if not already_mounted:
            print(f"H:Unmounting {mountpoint}")                    
            try:
                vfs.umount(mountpoint)
            except Exception as e:
                print(f"H:Error unmounting {mountpoint}: {e}")
        print(f"H:HexDrive app.mpy version:{version}")
        return int(version)

    def update_app_in_eeprom(self, port, header, i2c, addr) -> bool:
        # Copy hexdreive.py to EEPROM as app.mpy
        print(f"H:Updating HexDrive app.mpy on port {port}")
        try:
            _, partition = get_hexpansion_block_devices(i2c, header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False              
        mountpoint = '/hexpansion_' + str(port)
        already_mounted = False
        if not already_mounted:
            print(f"H:Mounting {partition} at {mountpoint}")
            try:
                vfs.mount(partition, mountpoint, readonly=False)
            except OSError as e:
                if e.args[0] == 1:
                    already_mounted = True
                else:
                    print(f"H:Error mounting: {e}")
            except Exception as e:
                print(f"H:Error mounting: {e}")
        source_path = "/" + __file__.rsplit("/", 1)[0] + "/hexdrive.mpy"
        dest_path   = f"{mountpoint}/app.mpy"
        try:
            # delete the existing app.mpy file
            print(f"H:Deleting {dest_path}")
            os.remove(f"{mountpoint}/app.py")
            os.remove(dest_path)
        except Exception:
            # ignore errors which will happen if the file does not exist
            pass
        print(f"H:Copying {source_path} to {dest_path}")
        try:
            appfile = open(dest_path, "wb")
        except Exception as e:
            print(f"H:Error opening {dest_path}: {e}")
            return False   
        try:        
            template = open(source_path, "rb")
        except Exception as e:
            print(f"H:Error opening {source_path}: {e}")
            return False   
        try:    
            appfile.write(template.read())                           
        except Exception as e:
            print(f"H:Error updating HexDrive: {e}")
            return False   
        try:
            appfile.close()
            template.close()     
        except Exception as e:
            print(f"H:Error closing files: {e}")
            return False
        if not already_mounted:
            try:
                vfs.umount(mountpoint)
                print(f"H:Unmounted {mountpoint}")                    
            except Exception as e:
                print(f"H:Error unmounting {mountpoint}: {e}")
                return False 
        print(f"H:HexDrive app.mpy updated to version {CURRENT_APP_VERSION}")            
        return True
    
    def prepare_eeprom(self, port, i2c) -> bool:
        print(f"H:Initialising EEPROM on port {port}")
        # Write and read back header efficiently
        try:
            i2c.writeto(_EEPROM_ADDR, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES) + hexdrive_header.to_bytes())
        except Exception as e:
            print(f"H:Error writing header: {e}")
            return False
        try:
            i2c.writeto(_EEPROM_ADDR, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES))  # Read header @ address 0                
            header_bytes = i2c.readfrom(_EEPROM_ADDR, 32)
        except Exception as e:
            print(f"H:Error reading header back: {e}")
            return False
        try:
            read_header = HexpansionHeader.from_bytes(header_bytes)
        except Exception as e:
            print(f"H:Error parsing header: {e}")
            return False
        try:
            # Get block devices
            _, partition = get_hexpansion_block_devices(i2c, read_header, _EEPROM_ADDR)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False           
        try:
            # Format
            vfs.VfsLfs2.mkfs(partition)
            print("H:EEPROM formatted")
        except Exception as e:
            print(f"H:Error formatting: {e}")
            return False
        try:
            # And mount!
            mountpoint = '/hexpansion_' + str(port)
            vfs.mount(partition, mountpoint, readonly=False)
            print("H:EEPROM initialised")
        except Exception as e:
            print(f"H:Error mounting: {e}")
            return False
        return True 



    def update_settings(self):
        self._settings['acceleration']  = settings.get("badgebot_acceleration",  _POWER_STEP_PER_TICK)
        self._settings['max_power']     = settings.get("badgebot_max_power",     _MAX_POWER)
        self._settings['drive_step_ms'] = settings.get("badgebot_drive_step_ms", _USER_DRIVE_MS)
        self._settings['turn_step_ms']  = settings.get("badgebot_turn_step_ms",  _USER_TURN_MS)
        self._settings['brightness']    = settings.get("pattern_brightness",     _BRIGHTNESS)
        if (self._settings['acceleration'] != _POWER_STEP_PER_TICK):
            print(f"Power step per tick: {self._settings['acceleration']}")
        if (self._settings['max_power'] != _MAX_POWER):
            print(f"Max power: {self._settings['max_power']}")
        if (self._settings['drive_step_ms'] != _USER_DRIVE_MS):  
            print(f"Drive step ms: {self._settings['drive_step_ms']}")
        if (self._settings['turn_step_ms'] != _USER_TURN_MS):
            print(f"Turn step ms: {self._settings['turn_step_ms']}")    
        if (self._settings['brightness'] != _BRIGHTNESS):
            print(f"Brightness: {self._settings['brightness']}")

    ### MAIN APP CONTROL FUNCTIONS ###

    def update(self, delta):
        self.clear_leds()
        if self.notification:
            self.notification.update(delta)

### START UI FOR HEXPANSION INITIALISATION AND UPGRADE ###
        if self.current_state == STATE_INIT:
            # One Time initialisation
            eventbus.emit(PatternDisable())
            self.scan_ports()
            if (len(self.ports_with_hexdrive) == 0) and (len(self.ports_with_blank_eeprom) == 0):
                # There are currently no possible HexDrives plugged in
                self.animation_counter = 0
                self.current_state = STATE_WARNING
            else:
                self.current_state = STATE_WAIT
            return
        elif self.current_state == STATE_WARNING or self.current_state == STATE_LOGO:
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                # Warning has been acknowledged by the user
                self.button_states.clear()
                if self.current_state == STATE_WARNING:
                    self.animation_counter = 0
                    self.current_state = STATE_LOGO
                else:
                    self.current_state = STATE_WARNING    
            else:
                # "CANCEL" button is handled below in common for all MINIMISE_VALID_STATES 
                # Show the warning screen for 10 seconds
                self.animation_counter += delta/1000
                if self.current_state == STATE_WARNING and self.animation_counter > 10:
                    # after 10 seconds show the logo
                    self.animation_counter = 0
                    self.current_state = STATE_LOGO
                elif self.current_state == STATE_LOGO:
                    # LED management - to match rotating logo:
                    for i in range(1,13):
                        colour = (255, 241, 0)      # custom Robotmad shade of yellow                                
                        # raised cosine cubed wave
                        wave = self._settings['brightness'] * pow((1.0 + cos(((i) *  pi / 1.5) - (self.rpm * self.animation_counter * pi / 7.5)))/2.0, 3)    
                        # 4 sides each projecting a pattern of 3 LEDs (12 LEDs in total)
                        tildagonos.leds[i] = tuple(int(wave * j) for j in colour)                                                     
                else: # STATE_WARNING
                    for i in range(1,13):
                        tildagonos.leds[i] = (255,0,0)                       
        elif self.current_state == STATE_ERROR or self.current_state == STATE_REMOVED: 
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                # Logo/Error has been acknowledged by the user
                self.button_states.clear()
                self.current_state = STATE_WAIT
                self.error_message = []
            else:
                for i in range(1,13):
                    tildagonos.leds[i] = (255,0,0)                   
        if self.current_state in MINIMISE_VALID_STATES:
            if self.current_state == STATE_DETECTED:
                # We are currently asking the user if they want hexpansion EEPROM initialising
                if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                    # Yes
                    self.button_states.clear()
                    self.current_state = STATE_PROGRAMMING
                    if self.prepare_eeprom(self.detected_port, I2C(self.detected_port)):
                        self.notification = Notification("Initialised", port = self.detected_port)
                        self.ports_with_hexdrive.add(self.detected_port)
                        self.current_state = STATE_WAIT
                    else:
                        self.notification = Notification("Failed", port = self.detected_port)
                        self.error_message = ["EEPROM","initialisation","failed"]
                        self.current_state = STATE_ERROR          
                elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                    # No
                    print("H:Cancelled")
                    self.button_states.clear()
                    self.current_state = STATE_WAIT
                return           
            elif self.current_state == STATE_UPGRADE:
                # We are currently asking the user if they want hexpansion App upgradingwith latest App.mpy                
                if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                    # Yes
                    self.button_states.clear()
                    self.current_state = STATE_PROGRAMMING
                    try:
                        i2c = I2C(self.upgrade_port)
                        i2c.writeto(_EEPROM_ADDR, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES))  # Read header @ address 0                
                        header_bytes = i2c.readfrom(_EEPROM_ADDR, 32)
                        read_header = HexpansionHeader.from_bytes(header_bytes)
                    except OSError:           
                        read_header = None                       
                    if read_header is not None and read_header.vid == _HEXDRIVE_VID and read_header.pid == _HEXDRIVE_PID:
                        if self.update_app_in_eeprom(self.upgrade_port, read_header, i2c, _EEPROM_ADDR):
                            self.notification = Notification("Upgraded", port = self.upgrade_port)
                            self.ports_with_upgraded_hexdrive.add(self.upgrade_port)
                            self.error_message = ["Upgraded:","Please","reboop"]
                            self.current_state = STATE_ERROR                                     
                        else:
                            self.notification = Notification("Failed", port = self.upgrade_port)
                            self.error_message = ["HexDrive","programming","failed"]
                            self.current_state = STATE_ERROR
                    else:
                        self.error_message = ["HexDrive","read","failed"]
                        self.current_state = STATE_ERROR
                elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                    print("H:Cancelled")
                    self.button_states.clear()
                    self.current_state = STATE_WAIT
                return
            elif 0 < len(self.ports_with_blank_eeprom):
                # if there are any ports with blank eeproms
                # Show the UI prompt and wait for button press
                self.detected_port = self.ports_with_blank_eeprom.pop()
                self.notification = Notification("Initialise?", port = self.detected_port)
                self.current_state = STATE_DETECTED          
            elif 0 < len(self.ports_with_hexdrive):
                # if there are any ports with HexDrives - check if they need upgrading
                port = self.ports_with_hexdrive.pop()
                try:
                    i2c = I2C(port)
                    i2c.writeto(_EEPROM_ADDR, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES))  # Read header @ address 0                
                    header_bytes = i2c.readfrom(_EEPROM_ADDR, 32)
                    read_header = HexpansionHeader.from_bytes(header_bytes)
                except OSError:     
                    read_header = None                  
                if read_header is not None and read_header.vid == _HEXDRIVE_VID and read_header.pid == _HEXDRIVE_PID:
                    print(f"H:HexDrive on port {port}")
                    if self.get_app_version_in_eeprom(port, read_header, i2c, _EEPROM_ADDR) == CURRENT_APP_VERSION:
                        print(f"H:HexDrive on port {port} has latest App")
                        self.ports_with_upgraded_hexdrive.add(port)
                        self.current_state = STATE_WAIT
                    else:    
                        # Show the UI prompt and wait for button press
                        self.upgrade_port = port
                        self.notification = Notification("Upgrade?", port = self.upgrade_port)
                        self.current_state = STATE_UPGRADE
                else:
                    print("H:Error reading Hexpansion header")
                    self.notification = Notification("Error", port = port)
                    self.error_message = ["Hexpansion","read","failed"]
                    self.current_state = STATE_ERROR        
            elif self.current_state == STATE_WAIT: 
                if 0 < len(self.ports_with_upgraded_hexdrive):
                    valid_port = next(iter(self.ports_with_upgraded_hexdrive))
                    # We have at least one HexDrive with the latest App.mpy
                    self.hexdrive_seen = True
                    # Find our running hexdrive app
                    for an_app in scheduler.apps:
                        if hasattr(an_app, "config") and an_app.config.port == valid_port:
                            self.hexdrive_app = an_app
                            self.hexdrive_port = valid_port # only inteneded for use with a single active HexDrive at once at present
                            print(f"H:Found app on port {valid_port}")
                            if self.hexdrive_app.get_status():
                                print(f"H:HexDrive [{valid_port}] OK")
                                self.current_state = STATE_MENU
                                self.animation_counter = 0
                            else:
                                print(f"H:HexDrive {valid_port}: Failed to initialise PWM resources")
                                self.error_message = ["HexDrive {valid_port}","PWM Init","Failed","Please","Reboop"]
                                self.current_state = STATE_ERROR
                            break
                    else:
                        print(f"H:HexDrive {valid_port}: App not found, please reboop")
                        self.error_message = [f"HexDrive {valid_port}","App not found.","Please","reboop"]
                        self.current_state = STATE_ERROR                           
                elif self.hexdrive_seen:
                    self.hexdrive_seen = False
                    self.current_state = STATE_REMOVED
                else:
                    self.animation_counter = 0                   
                    self.current_state = STATE_WARNING
### END OF UI FOR HEXPANSION INITIALISATION AND UPGRADE ###

        if self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in MINIMISE_VALID_STATES and self.current_state != STATE_DONE:
            self.button_states.clear()
            self.minimise()
        elif self.current_state == STATE_MENU:
            # Exit start menu
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.is_scroll = True   # so that release of this button will CLEAR Scroll mode
                self.current_state = STATE_RECEIVE_INSTR
                self.button_states.clear()
            # Show the instructions screen for 10 seconds
            self.animation_counter += delta/1000
            if self.animation_counter > 10:
                # after 10 seconds show the logo
                self.animation_counter = 0
                self.current_state = STATE_LOGO
        elif self.current_state == STATE_RECEIVE_INSTR:
            # Enable/disable scrolling and check for long press
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.long_press_delta += delta
                if self.long_press_delta >= _LONG_PRESS_MS:
                    self.finalize_instruction()
                    self.current_state = STATE_COUNTDOWN
            else:
                # Confirm is not pressed. Reset long_press state
                self.long_press_delta = 0
                # Manage scrolling
                if self.is_scroll:
                    if self.button_states.get(BUTTON_TYPES["DOWN"]):
                        self.scroll_offset -= 1
                    elif self.button_states.get(BUTTON_TYPES["UP"]):
                        self.scroll_offset += 1
                    self.button_states.clear()
                # Instruction button presses
                elif self.button_states.get(BUTTON_TYPES["RIGHT"]):
                    self._handle_instruction_press(BUTTON_TYPES["RIGHT"])
                    self.button_states.clear()
                elif self.button_states.get(BUTTON_TYPES["LEFT"]):
                    self._handle_instruction_press(BUTTON_TYPES["LEFT"])
                    self.button_states.clear()
                elif self.button_states.get(BUTTON_TYPES["UP"]):
                    self._handle_instruction_press(BUTTON_TYPES["UP"])
                    self.button_states.clear()
                elif self.button_states.get(BUTTON_TYPES["DOWN"]):
                    self._handle_instruction_press(BUTTON_TYPES["DOWN"])
                    self.button_states.clear()
            # LED management
            if self.last_press == BUTTON_TYPES["RIGHT"]:
                # Green = Starboard = Right
                tildagonos.leds[2]  = (0, 255, 0)
                tildagonos.leds[3]  = (0, 255, 0)
            elif self.last_press == BUTTON_TYPES["LEFT"]:
                # Red = Port = Left
                tildagonos.leds[8]  = (255, 0, 0)
                tildagonos.leds[9]  = (255, 0, 0)
            elif self.last_press == BUTTON_TYPES["UP"]:
                # Cyan
                tildagonos.leds[12] = (0, 255, 255)
                tildagonos.leds[1]  = (0, 255, 255)
            elif self.last_press == BUTTON_TYPES["DOWN"]:
                # Magenta
                tildagonos.leds[6]  = (255, 0, 255)
                tildagonos.leds[7]  = (255, 0, 255)
        elif self.current_state == STATE_COUNTDOWN:
            self.run_countdown_elapsed_ms += delta
            if self.run_countdown_elapsed_ms >= _RUN_COUNTDOWN_MS:
                self.power_plan_iter = chain(*(instr.power_plan for instr in self.instructions))
                self.hexdrive_app.set_power(True)
                self.current_state = STATE_RUN
        elif self.current_state == STATE_RUN:
            # Run is primarily managed in the background update
            pass
        elif self.current_state == STATE_DONE:
            if self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.hexdrive_app.set_power(False)
                self.reset()
                self.button_states.clear()
            elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.hexdrive_app.set_power(False)
                self.run_countdown_elapsed_ms = 0
                self.current_power_duration = ((0,0,0,0), 0)
                self.current_state = STATE_COUNTDOWN                       
                self.button_states.clear()
        if self._settings['brightness'] < 1.0:
            # Scale brightness
            for i in range(1,13):
                tildagonos.leds[i] = tuple(int(j * self._settings['brightness']) for j in tildagonos.leds[i])                            
        tildagonos.leds.write()


    def draw(self, ctx):
        ctx.save()
        ctx.font_size = label_font_size
        if self.current_state == STATE_LOGO:
            draw_logo_animated(ctx, self.rpm, self.animation_counter, [self.b_msg, self.t_msg], self.qr_code)
        # Scroll mode indicator
        elif self.is_scroll:
            ctx.rgb(0,0.2,0).rectangle(-120,-120,240,240).fill()
        else:
            ctx.rgb(0,0,0.2).rectangle(-120,-120,240,240).fill()

        if   self.current_state == STATE_WARNING:
            self.draw_message(ctx, ["BadgeBot requires","HexDrive hexpansion","from RobotMad","github.com","/TeamRobotmad","/BadgeBot"], [(1,1,1),(1,1,0),(1,1,0),(1,1,1),(1,1,1),(1,1,1)], label_font_size)
        elif self.current_state == STATE_REMOVED:
            self.draw_message(ctx, ["HexDrive","removed","Please reinsert"], [(1,1,0),(1,1,1),(1,1,1)], label_font_size)      
        elif self.current_state == STATE_DETECTED:
            self.draw_message(ctx, ["Hexpansion","detected in",f"Slot {self.detected_port}","Init EEPROM","as HexDrive?"], [(1,1,1),(1,1,1),(0,0,1),(1,1,1),(1,1,0)], label_font_size)
        elif self.current_state == STATE_UPGRADE:
            self.draw_message(ctx, ["HexDrive","detected in",f"Slot {self.upgrade_port}","Upgrade","HexDrive app?"], [(1,1,0),(1,1,1),(0,0,1),(1,1,1),(1,1,1)], label_font_size)             
        elif self.current_state == STATE_PROGRAMMING:
            self.draw_message(ctx, ["HexDrive:","Programming","EEPROM","Please wait..."], [(1,1,0),(1,1,1),(1,1,1),(1,1,1)], label_font_size)            
        elif self.current_state == STATE_MENU:
            self.draw_message(ctx, ["BadgeBot","To Program:","Press C","When finished:","Long press C"], [(1,1,0),(1,1,1),(1,1,1),(1,1,1),(1,1,1)], label_font_size)
        elif self.current_state == STATE_ERROR:
            self.draw_message(ctx, self.error_message, [(1,0,0)]*len(self.error_message), label_font_size)
        elif self.current_state == STATE_RECEIVE_INSTR:
            # Display list of movements
            for i_num, instr in enumerate(["START"] + self.instructions + [self.current_instruction, "END"]):
                ctx.rgb(1,1,0).move_to(H_START, V_START + VERTICAL_OFFSET * (self.scroll_offset + i_num)).text(str(instr))
        elif self.current_state == STATE_COUNTDOWN:
            countdown_val = 1 + ((_RUN_COUNTDOWN_MS - self.run_countdown_elapsed_ms) // 1000)
            self.draw_message(ctx, [str(countdown_val)], [(1,1,0)], twentyfour_pt)
        elif self.current_state == STATE_RUN:
            # convert current_power_duration to string, dividing all four values down by 655 (to get a value from 0-100)
            current_power, _ = self.current_power_duration
            power_str = str(tuple([x//655 for x in current_power]))
            #TODO - remember the directon to be shown: direction_str = str(self.current_instruction.press_type)
            self.draw_message(ctx, ["Running...",power_str], [(1,1,1),(1,1,0)], label_font_size)
        elif self.current_state == STATE_DONE:
            self.draw_message(ctx, ["Program","complete!","Replay:Press C","Restart:Press F"], [(0,1,0),(0,1,0),(1,1,0),(0,1,1)], label_font_size)
        if self.notification:
            self.notification.draw(ctx)
        ctx.restore()


    def clear_leds(self):
        for i in range(1,13):
            tildagonos.leds[i] = (0, 0, 0)


    def draw_message(self, ctx, message, colours, size):
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
            y_position = int(0.35 * ctx.font_size) if num_lines == 1 else int((i_num-((num_lines-2)/2)) * ctx.font_size)
            ctx.rgb(*colour).move_to(-width//2, y_position).text(text_line)


### BADGEBOT DEMO FUNCTIONALITY ###
    def _handle_instruction_press(self, press_type: Button):
        if self.last_press == press_type:
            self.current_instruction.inc()
        else:
            self.finalize_instruction()
            self.current_instruction = Instruction(press_type)
        self.last_press = press_type

    def reset(self):
        self.current_state = STATE_MENU
        self.last_press = BUTTON_TYPES["CONFIRM"]
        self.animation_counter = 0
        self.long_press_delta = 0
        self.is_scroll = False
        self.scroll_offset = 0
        self.run_countdown_elapsed_ms = 0
        self.instructions = []
        self.current_instruction = None
        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])


    def get_current_power_level(self, delta) -> int:
        # takes in delta as ms since last call
        # if delta was > 10... what to do
        if delta >= _TICK_MS:
            delta = _TICK_MS-1

        current_power, current_duration = self.current_power_duration

        updated_duration = current_duration - delta
        if updated_duration <= 0:
            try:
                next_power, next_duration = next(self.power_plan_iter)
            except StopIteration:
                # returns None when complete
                return None
            next_duration += updated_duration
            self.current_power_duration = next_power, next_duration
            return next_power
        else:
            self.current_power_duration = current_power, updated_duration
            return current_power


    def finalize_instruction(self):
        if self.current_instruction is not None:
            self.current_instruction.make_power_plan(self._settings)
            self.instructions.append(self.current_instruction)
            if len(self.instructions) >= 5:
                self.scroll_offset -= 1
            self.current_instruction = None








class Instruction:
    def __init__(self, press_type: Button) -> None:
        self._press_type = press_type
        self._duration: int = 1
        self.power_plan = []


    @property
    def press_type(self) -> Button:
        return self._press_type


    def inc(self):
        self._duration += 1


    def __str__(self):
        return f"{self.press_type.name} {self._duration}"


    def directional_power_tuple(self, power):
        if self._press_type == BUTTON_TYPES["UP"]:
            return (0, power, 0, power)
        elif self._press_type == BUTTON_TYPES["DOWN"]:
            return (power, 0, power, 0)
        elif self._press_type == BUTTON_TYPES["LEFT"]:
            return (power, 0, 0, power)
        elif self._press_type == BUTTON_TYPES["RIGHT"]:
            return (0, power, power, 0)


    def directional_duration(self, mysettings):
        if self._press_type == BUTTON_TYPES["UP"]:
            return (mysettings['drive_step_ms'])
        elif self._press_type == BUTTON_TYPES["DOWN"]:
            return (mysettings['drive_step_ms'])            
        elif self._press_type == BUTTON_TYPES["LEFT"]:
            return (mysettings['turn_step_ms'])
        elif self._press_type == BUTTON_TYPES["RIGHT"]:
            return (mysettings['turn_step_ms'])
        

    def make_power_plan(self, mysettings):
        # return collection of tuples of power and their duration
        curr_power = 0
        ramp_up = []
        for i in range(1*(self._duration+3)):
            ramp_up.append((self.directional_power_tuple(curr_power), _TICK_MS))
            curr_power += mysettings['acceleration']
            if curr_power >= mysettings['max_power']:
                ramp_up.append((self.directional_power_tuple(mysettings['max_power']), _TICK_MS))
                break
        user_power_duration = (self.directional_duration(mysettings) * self._duration)-(2*(i+1)*_TICK_MS)
        power_durations = ramp_up.copy()
        if user_power_duration > 0:
            power_durations.append((self.directional_power_tuple(mysettings['max_power']), user_power_duration))
        ramp_down = ramp_up.copy()
        ramp_down.reverse()
        power_durations.extend(ramp_down)
        print("Power durations:")
        print(power_durations)
        self.power_plan = power_durations


__app_export__ = BadgeBotApp
