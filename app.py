import app
import asyncio
from app_components.tokens import label_font_size
from app_components.notification import Notification
from events.input import Buttons, BUTTON_TYPES
from tildagonos import tildagonos
from system.eventbus import eventbus
from system.patterndisplay.events import (PatternDisable, PatternEnable)

# Hexpansion related imports
from system.hexpansion.header import HexpansionHeader
from system.hexpansion.util import (read_hexpansion_header, get_hexpansion_block_devices, detect_eeprom_addr)
from system.hexpansion.events import (HexpansionInsertionEvent, HexpansionRemovalEvent)
from system.hexpansion.config import _pin_mapping

import vfs
import os
import asyncio
from machine import (Pin, I2C)
from tildagon import Pin as ePin

CURRENT_APP_VERSION = 1 # Integer Version Number - checked against the EEPROM app.py version to determine if it needs updating

# Motor Driver
PWM_FREQ = 5000
MAX_DUTY = 1023

VERTICAL_OFFSET = label_font_size
H_START = -78
V_START = -58

TICK_MS = 20
POWER_STEP_PER_TICK = 20
LONG_PRESS_MS = 750

USER_TICK_MULTIPLIER = 4

MAX_POWER = 100

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

MINIMISE_VALID_STATES = [0, 1, 2, 5, 7, 8, 10, 11]

DEFAULT_HEX_DRIVE_SLOT = 3
POWER_ENABLE_PIN_INDEX = 0	# First LS pin

HEXDRIVE_PID = 0xCBCB

class Instruction:
    def __init__(self, press_type: BUTTON_TYPES) -> None:
        self._press_type = press_type
        self._duration = 1
        self.power_plan_iterator = iter([])

    @property
    def press_type(self) -> BUTTON_TYPES:
        return self._press_type

    def inc(self):
        self._duration += 1

    def __str__(self):
        return f"{self.press_type.name} {self._duration}"

    def directional_power_tuple(self, power):
        if self._press_type == BUTTON_TYPES["UP"]:
            return (power, 0, power, 0)
        elif self._press_type == BUTTON_TYPES["DOWN"]:
            return (0, power, 0, power)
        elif self._press_type == BUTTON_TYPES["LEFT"]:
            return (power, 0, 0, power)
        elif self._press_type == BUTTON_TYPES["RIGHT"]:
            return (0, power, power, 0)

    def make_power_plan(self):
        # return collection of tuples of power and their duration
        ramp_up = [(self.directional_power_tuple(p), TICK_MS)
                   for p in range(0, MAX_POWER, POWER_STEP_PER_TICK)]
        power_durations = ramp_up.copy()
        user_power_duration = TICK_MS * USER_TICK_MULTIPLIER * (self._duration-1)
        power_durations.append((self.directional_power_tuple(MAX_POWER), user_power_duration))
        ramp_down = ramp_up.copy()
        ramp_down.reverse()
        power_durations.extend(ramp_down)
        self.power_plan_iterator = iter(power_durations)


class BadgeBotApp(app.App):
    def __init__(self):
        super().__init__()
        self.button_states = Buttons(self)
        self.last_press: BUTTON_TYPES = BUTTON_TYPES["CANCEL"]
        self.long_press_delta = 0

        self.is_scroll = False
        self.scroll_offset = 0

        self.run_countdown_target_ms = 3000
        self.run_countdown_ms = 0

        self.instructions = []
        self.current_instruction = None

        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])

        self.notification = None

        # Hexpansion related
        self.hexdrive_has_been_seen = False
        self.ports_with_blank_eeprom = []
        self.ports_with_hexdrive = []
        self.ports_with_upgraded_hexdrive = []
        eventbus.on_async(HexpansionInsertionEvent, self.handle_hexpansion_insertion, self)
        eventbus.on_async(HexpansionRemovalEvent,   self.handle_hexpansion_removal,   self)

        # Overall app state (controls what is displayed)
        self.current_state = STATE_INIT

    async def handle_hexpansion_removal(self, event):
        await asyncio.sleep(1)
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
        if self.current_state == STATE_UPGRADE and event.port == self.upgrade_port:
            self.current_state = STATE_WAIT

    async def handle_hexpansion_insertion(self, event):
        await asyncio.sleep(1)    
        i2c = I2C(event.port)
        # Autodetect eeprom addr
        addr = detect_eeprom_addr(i2c)
        if addr is None:
            print(f"H:Scan found no EEPROMs")
            return
        # Do we have a header?
        header = read_hexpansion_header(i2c, addr)
        if (header is None):
            print(f"H:Detected EEPROM at {hex(addr)}")
            self.ports_with_blank_eeprom.append(event.port)
        elif header.vid == 0xCAFE and header.pid == HEXDRIVE_PID:
            # We have a HexDrive Hexpansion
            print(f"H:Found {header.friendly_name} #{header.unique_id}")
            if event.port not in self.ports_with_hexdrive:
                self.ports_with_hexdrive.append(event.port)


    def get_app_version_in_eeprom(self, port, header, i2c, addr):
        try:
            eep, partition = get_hexpansion_block_devices(i2c, header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return 0
        version = 0
        mountpoint = '/hexpansion_' + str(port)
        already_mounted = False # if there is a way to find out if it is already mounted then use it here
        if not already_mounted:
            print(f"H:Mounting {partition} at {mountpoint}")
            try:
                vfs.mount(partition, mountpoint, readonly=True)
            except Exception as e:
                #if e.args[0] == 1:
                already_mounted = True
                #else:
                print(f"H:Error mounting: {e}")
        print("H:Reading app.py")
        try:
            with open(f"{mountpoint}/app.py", "rt") as appfile:
                app = appfile.read()
                version = app.split("APP_VERSION = ")[1].split("\n")[0]
            if not already_mounted:
                print(f"H:Unmounting {mountpoint}")                    
                vfs.umount(mountpoint)
            print(f"H:HexDrive app.py version is {version}")
            return int(version)
        except Exception as e:
            print(f"H:Error reading HexDrive app.py: {e}")
            return 0

    def update_app_in_eeprom(self, port, header, i2c, addr):
        # Copy hexdreive.py to EEPROM as app.py
        print(f"H:Updating HexDrive app.py on port {port}")
        try:
            eep, partition = get_hexpansion_block_devices(i2c, header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False              
        mountpoint = '/hexpansion_' + str(port)
        already_mounted = False
        if not already_mounted:
            print(f"H:Mounting {partition} at {mountpoint}")
            try:
                vfs.mount(partition, mountpoint, readonly=False)
            except Exception as e:
                if e.args[0] == 1:
                    already_mounted = True
                else:
                    print(f"H:Error mounting: {e}")
        try:
            print(os.listdir(mountpoint))
            path = "/" + __file__.rsplit("/", 1)[0] + "/hexdrive.py"
            print(f"H:Copying {path} to {mountpoint}/app.py")
            with open(f"{mountpoint}/app.py", "wt") as appfile:
                with open(path, "rt") as template:
                    appfile.write(template.read())
            if not already_mounted:
                print(f"H:Unmounting {mountpoint}")                    
                vfs.umount(mountpoint)
            print(f"H:HexDrive app.py updated to version {CURRENT_APP_VERSION}")
            return True
        except Exception as e:
            print(f"H:Error updating HexDrive app.py: {e}")
            return False   

    def prepare_eeprom(self, port, i2c, addr):
        print(f"H:Initialising EEPROM @ {hex(addr)} on port {port}")
        header = HexpansionHeader(
            manifest_version="2024",
            fs_offset=32,
            eeprom_page_size=32,
            eeprom_total_size=64 * 1024 // 8,
            vid=0xCAFE,
            pid=HEXDRIVE_PID,
            unique_id=0x0,
            friendly_name="HexDrive",
        )
        # Write and read back header
        # write_header is broken for our type of EEPROM so we do it manually
        #write_header(port, header, addr=addr, addr_len=addr_len, page_size=header.eeprom_page_size)
        i2c.writeto(addr, bytes([0, 0]) + header.to_bytes())
        header = read_hexpansion_header(i2c, addr, set_read_addr=True, addr_len=2)
        if header is None:
            raise RuntimeError("H:EEPROM Init failed")
            return False
        try:
            # Get block devices
            eep, partition = get_hexpansion_block_devices(i2c, header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False           
        try:
            # Format
            vfs.VfsLfs2.mkfs(partition)
            print(f"H:EEPROM formatted")
        except Exception as e:
            print(f"H:Error formatting: {e}")
            return False
        try:
            # And mount!
            vfs.mount(partition, "/eeprom")
            print(f"H:EEPROM initialised")
        except Exception as e:
            print(f"H:Error mounting: {e}")
            return False
        return True 

    def set_pin_out(self, pin):
        # Tildagon(s) (version 1.6) is missing code to set the eGPIO direction to output
        # so we need to update this directly
        try:
            # Use a Try in case access to i2C(7) is blocked for apps in future
            # presumably if this happens then the code will have been updated to
            # handle the GPIO direction correctly anyway.
            i2c = I2C(7)
            config_reg = int.from_bytes(i2c.readfrom_mem(pin[0], 0x04+pin[1], 1), 'little')
            config_reg &= ~(pin[2])
            i2c.writeto_mem(pin[0], 0x04+pin[1], bytes([config_reg]))
        except:
            print(f"H:access to I2C(7) blocked")

    def set_hexdrive_power(self, state):
        port = self.ports_with_upgraded_hexdrive[0]
        power_enable_pin_number = _pin_mapping[port]["ls"][POWER_ENABLE_PIN_INDEX]
        HexDrivePowerEnable = ePin(power_enable_pin_number,Pin.OUT)
        #Issue that the Badge Code does not set the IO expander to output mode
        self.set_pin_out(HexDrivePowerEnable.pin)
        HexDrivePowerEnable.value(state)

    # Scan the Hexpansion ports for EEPROMs and HexDrives in case they are already plugged in when we start
    def scan_ports(self):
        for port in range(1, 5):
            i2c = I2C(port)
            addr = detect_eeprom_addr(i2c)
            if addr is not None:
                header = read_hexpansion_header(i2c, addr)
                if header is None:
                    print(f"H:Found EEPROM on port {port}")
                    self.ports_with_blank_eeprom.append(port)
                elif header.vid == 0xCAFE and header.pid == 0xCBCB:
                    print(f"H:Found HexDrive on port {port}")
                    self.ports_with_hexdrive.append(port)

    def update(self, delta):
        if self.notification:
            self.notification.update(delta)

        if self.current_state == STATE_INIT:
            # One Time initialisation
            self.scan_ports()
            if (len(self.ports_with_hexdrive) == 0) and (len(self.ports_with_blank_eeprom) == 0):
                # There are currently no possible HexDrives plugged in
                self.current_state = STATE_WARNING
        
        if self.current_state == STATE_ERROR:
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]) or self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.button_states.clear()
                self.current_state = STATE_WAIT
                self.error_message = []
        elif 0 < len(self.ports_with_blank_eeprom):
            # if there are any ports with blank eeproms
            if self.current_state == STATE_DETECTED:
                if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                    self.button_states.clear()
                    # EEPROM initialisation        
                    port = self.ports_with_blank_eeprom.pop(0)
                    self.current_state = STATE_PROGRAMMING
                    if self.prepare_eeprom(port, I2C(port), 0x50):
                        self.ports_with_hexdrive.append(port)
                        self.current_state = STATE_WAIT
                    else:
                        self.error_message = ["EEPROM","Initialisation","Failed"]
                        self.current_state = STATE_ERROR          
                elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                    print(f"H:Cancelled")
                    self.button_states.clear()
                    self.ports_with_blank_eeprom.pop(0)
                    self.current_state = STATE_WAIT
                return
            else:
                # Show the UI prompt and wait for button press
                self.detected_port = self.ports_with_blank_eeprom[0]
                self.current_state = STATE_DETECTED          
        elif 0 < len(self.ports_with_hexdrive):
            # if there are any ports with HexDrives
            if self.current_state == STATE_UPGRADE:
                if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                    self.button_states.clear()
                    # EEPROM programming with latest App.py                
                    self.current_state = STATE_PROGRAMMING
                    port = self.ports_with_hexdrive.pop(0)
                    i2c = I2C(port)
                    header = read_hexpansion_header(i2c, 0x50)
                    if header:
                        if self.update_app_in_eeprom(port, header, i2c, 0x50):
                            self.ports_with_upgraded_hexdrive.append(port)
                            self.current_state = STATE_WAIT
                        else:
                            self.error_message = ["HexDrive","Programming","Failed"]
                            self.current_state = STATE_ERROR
                    else:
                        self.error_message = ["HexDrive","Read","Failed"]
                        self.current_state = STATE_ERROR
                elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                    print(f"H:Cancelled")
                    self.button_states.clear()
                    self.ports_with_hexdrive.pop(0)
                    self.current_state = STATE_WAIT
                return
            else:
                i2c = I2C(self.ports_with_hexdrive[0])
                header = read_hexpansion_header(i2c, 0x50)
                if header and header.vid == 0xCAFE and header.pid == HEXDRIVE_PID:
                    print(f"H:HexDrive on port {self.ports_with_hexdrive[0]}")
                    if self.get_app_version_in_eeprom(self.ports_with_hexdrive[0], header, i2c, 0x50) == CURRENT_APP_VERSION:
                        print(f"H:HexDrive on port {self.ports_with_hexdrive[0]} has latest App")
                        port = self.ports_with_hexdrive.pop(0)
                        self.ports_with_upgraded_hexdrive.append(port)
                        self.current_state = STATE_WAIT
                    else:    
                        self.current_state = STATE_UPGRADE
                        # Show the UI prompt and wait for button press
                        self.upgrade_port = self.ports_with_hexdrive[0]
                        self.current_state = STATE_UPGRADE
                else:
                    print(f"H:Error reading Hexpansion header")
                    self.error_message = ["Hexpansion","Read","Failed"]
                    self.current_state = STATE_ERROR        
        elif self.current_state == STATE_WAIT: 
            if 0 < len(self.ports_with_upgraded_hexdrive):
                # We have at least one HexDrive with the latest App.py
                self.hexdrive_has_been_seen = True
                self.current_state = STATE_MENU
            elif self.hexdrive_has_been_seen:
                self.current_state = STATE_REMOVED
            else:                   
                self.current_state = STATE_WARNING






        if self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in MINIMISE_VALID_STATES:
            self.button_states.clear()
            eventbus.emit(PatternEnable()) # TODO replace with on lose focus on gain focus
            self.minimise()
        elif self.current_state == STATE_MENU:
            # Exit start menu
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.current_state = STATE_RECEIVE_INSTR
                self.button_states.clear()
        elif self.current_state == STATE_WARNING:
            # Exit warning screen
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.current_state = STATE_MENU
                self.button_states.clear()
        elif self.current_state == STATE_RECEIVE_INSTR:
            eventbus.emit(PatternDisable())
            self.clear_leds()
            # Enable/disable scrolling and check for long press
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                if self.long_press_delta == 0:
                    # TODO Move to button up event
                    self.is_scroll = not self.is_scroll
                    self.notification = Notification(f"Scroll {self.is_scroll}")
                self.long_press_delta += delta
                if self.long_press_delta >= LONG_PRESS_MS:
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
                    tildagonos.leds[2] = (255, 0, 0)
                    tildagonos.leds[3] = (255, 0, 0)
                elif self.last_press == BUTTON_TYPES["LEFT"]:
                    tildagonos.leds[8] = (0, 255, 0)
                    tildagonos.leds[9] = (0, 255, 0)
                elif self.last_press == BUTTON_TYPES["UP"]:
                    tildagonos.leds[12] = (0, 0, 255)
                    tildagonos.leds[1] = (0, 0, 255)
                elif self.last_press == BUTTON_TYPES["DOWN"]:
                    tildagonos.leds[6] = (255, 255, 0)
                    tildagonos.leds[7] = (255, 255, 0)
            tildagonos.leds.write()

        elif self.current_state == STATE_COUNTDOWN:
            self.run_countdown_ms += delta
            if self.run_countdown_ms >= self.run_countdown_target_ms:
                self.power_plan_iter = chain(*(instr.power_plan_iterator for instr in self.instructions))
                self.set_hexdrive_power(True)
                self.current_state = STATE_RUN

        elif self.current_state == STATE_RUN:
            print(delta)
            power = self.get_current_power_level(delta)
            if power is None:
                self.current_state = STATE_DONE
            print(f"Using power: {power}")

        elif self.current_state == STATE_DONE:
            self.set_hexdrive_power(False)
            eventbus.emit(PatternEnable())
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.reset()

    def _handle_instruction_press(self, press_type: BUTTON_TYPES):
        if self.last_press == press_type:
            self.current_instruction.inc()
        else:
            self.finalize_instruction()
            self.current_instruction = Instruction(press_type)
        self.last_press = press_type

    def reset(self):
        self.current_state = STATE_MENU
        self.button_states.clear()
        self.last_press: BUTTON_TYPES = BUTTON_TYPES["CANCEL"]
        self.long_press_delta = 0

        self.is_scroll = False
        self.scroll_offset = 0

        self.run_countdown_target_ms = 3000
        self.run_countdown_ms = 0

        self.instructions = []
        self.current_instruction = None

        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])


    def draw(self, ctx):
        ctx.save()
        ctx.font_size = label_font_size
        # Scroll mode indicator
        if self.is_scroll:
            ctx.rgb(0.1,0,0).rectangle(-120,-120,240,240).fill()
        else:
            ctx.rgb(0,0,0.1).rectangle(-120,-120,240,240).fill()

        if self.current_state == STATE_WARNING:
            ctx.rgb(1,1,1).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 20).text(f"Please buy")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 20).text(f"HexDrive")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 20).text(f"Hexpansion")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 20).text(f"from")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 4*VERTICAL_OFFSET + 20).text(f"RobotMad")
        elif self.current_state == STATE_REMOVED:
            ctx.rgb(1,1,0).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 20).text(f"HexDrive")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 20).text(f"removed")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 20).text(f"Please reinsert")            
        elif self.current_state == STATE_DETECTED:  
            ctx.rgb(1,1,1).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 20).text(f"Hexpansion")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 20).text(f"Detected in")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 20).text(f"Slot {self.detected_port}")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 20).text(f"Initialise EEPROM")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 4*VERTICAL_OFFSET + 20).text(f"as HexDrive?")
        elif self.current_state == STATE_UPGRADE:  
            ctx.rgb(1,1,0).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 20).text(f"HexDrive")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 20).text(f"Detected in")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 20).text(f"Slot {self.upgrade_port}")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 20).text(f"Program EEPROM")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 4*VERTICAL_OFFSET + 20).text(f"with App?")            
        elif self.current_state == STATE_PROGRAMMING:
            ctx.rgb(1,1,0).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 20).text(f"HexDrive")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 20).text(f"Programming")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 20).text(f"EEPROM")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 20).text(f"...")             
        elif self.current_state == STATE_MENU:
            ctx.rgb(1,1,1).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 00).text(f"To Program:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 00).text(f"Press C")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 10).text(f"When finished:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 10).text(f"Long press C")
        elif self.current_state == STATE_ERROR:
            for i_num, instr in enumerate(self.error_message):
                ctx.rgb(1,0,0).move_to(H_START, V_START + VERTICAL_OFFSET * i_num).text(str(instr))
        elif self.current_state == STATE_RECEIVE_INSTR:
            for i_num, instr in enumerate(["START"] + self.instructions + [self.current_instruction, "END"]):
                ctx.rgb(1,1,0).move_to(H_START, V_START + VERTICAL_OFFSET * (self.scroll_offset + i_num)).text(str(instr))
        elif self.current_state == STATE_COUNTDOWN:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text("Running in:")
            countdown_val = (self.run_countdown_target_ms - self.run_countdown_ms) / 1000
            ctx.rgb(1,1,0).move_to(H_START, V_START+VERTICAL_OFFSET).text(str(countdown_val))
        elif self.current_state == STATE_RUN:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text("Running power")
            ctx.rgb(1,0,0).move_to(H_START-30, V_START + 2*VERTICAL_OFFSET).text(str(self.current_power_duration))
        elif self.current_state == STATE_DONE:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text(f"Complete!")
            ctx.rgb(1,1,1).move_to(H_START, V_START + VERTICAL_OFFSET).text("To restart:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 2*VERTICAL_OFFSET).text("Press C")
        if self.notification:
            self.notification.draw(ctx)
        ctx.restore()


    def get_current_power_level(self, delta) -> int:
        # takes in delta as ms since last call
        # if delta was > 20... what to do
        if delta >= TICK_MS:
            delta = TICK_MS-1

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
            self.current_instruction.make_power_plan()
            self.instructions.append(self.current_instruction)
            if len(self.instructions) >= 5:
                self.scroll_offset -= 1
            self.current_instruction = None

    def clear_leds(self):
        for i in range(0,12):
            tildagonos.leds[i+1] = (0, 0, 0)

def chain(*iterables):
    for iterable in iterables:
        yield from iterable

__app_export__ = BadgeBotApp
