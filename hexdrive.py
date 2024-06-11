# This is the app to be installed from the HexDrive Hexpansion EEPROM.
# it is copied onto the EEPROM and renamed as app.py/mpy
# It is then run from the EEPROM by the BadgeOS.

import asyncio
import app
from machine import I2C, PWM
from system.eventbus import eventbus
from system.scheduler.events import RequestStopAppEvent

# HexDrive.py App Version - parsed by app.py to check if upgrade is required
APP_VERSION = 2 

_ENABLE_PIN = const(0)	# First LS pin used to enable the SMPSU
_DETECT_PIN = const(1)   # Second LS pin used to sense if the SMPSU has a source of power

_DEFAULT_PWM_FREQ = const(20000)    
_DEFAULT_KEEP_ALIVE_PERIOD = const(1000)  # 1 second
class HexDriveApp(app.App):

    def __init__(self, config=None):
        self.config = config
        self.keep_alive_period = _DEFAULT_KEEP_ALIVE_PERIOD
        self.power_state = None
        self.pwm_setup_failed = True
        self.last_update_time = 0
        self.outputs_energised = False
        self.PWMOutput = [None] * len(self.config.pin)
        self.power_detect = self.config.ls_pin[_DETECT_PIN]
        self.power_control = self.config.ls_pin[_ENABLE_PIN]       

        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)

        self.initialise()

    def initialise(self) -> bool:
        if self.config is None:
            return False        
        # report app starting and which port it is running on
        print(f"HexDrive App Init on port {self.config.port}")
        # Set Power Detect Pin to Input and Power Enable Pin to Output
        self._set_pin_direction(self.power_detect.pin,  1)
        self._set_pin_direction(self.power_control.pin, 0)  
        self.set_power(False)
        # Set all HexDrive Hexpansion HS pins to low level outputs
        for hs_pin in self.config.pin:
            hs_pin.value(0)    
        # Allocate PWM generation to pins
        for i_num, hs_pin in enumerate(self.config.pin):
            try:
                self.PWMOutput[i_num] = PWM(hs_pin, freq = _DEFAULT_PWM_FREQ, duty_u16 = 0)
                print(f"H:{self.config.port}:PWM[{i_num}]:{self.PWMOutput[i_num]}")
            except:
                # There are a finite number of PWM resources so it is possible that we run out
                print(f"H:{self.config.port}:PWM[{i_num}]:PWM allocation failed")
                return False
        self.pwm_setup_failed = False
        return not self.pwm_setup_failed


    def deinitialise(self) -> bool:
        # Turn off all PWM outputs & release resources
        for i, pwm in enumerate(self.PWMOutput):
            pwm.deinit()
            self.PWMOutput[i] = None
        self.set_power(False)
        for hs_pin in self.config.pin:
            hs_pin.value(0)          
        return True

    async def _handle_stop_app(self, event):
        if event.app == self:
            print(f"H:{self.config.port}:Stopping HexDrive App")
            self.deinitialise()

    # Check keep alive period and turn off PWM outputs if exceeded
    def background_update(self, delta):
        if (self.config is None) or self.pwm_setup_failed:
            return
        self.time_since_last_update += delta
        if self.time_since_last_update > self.keep_alive_period:
            self.set_pwm([0, 0, 0, 0])
            self.time_since_last_update = 0
            if self.outputs_energised:
                # First time the keep alive period has expired so report it
                print(f"H:{self.config.port}:Keep Alive Timeout")            
                self.outputs_energised = False
            # we keep retriggering in case anything else has corrupted the PWM outputs


    def get_status(self) -> bool:
        return not self.pwm_setup_failed


    # Turn the SPMPSU on or off
    # Just because the SPMSU is turned off does not mean that the outputs are NOT energised
    # as there could be external battery power
    def set_power(self, state) -> bool:
        if (self.config is None) or (state == self.power_state):
            return False
        print(f"H:{self.config.port}:Power={'On' if state else 'Off'}")
        if (self._get_pin_state(self.power_detect_pin.pin)):
            # if the power detect pin is high then the SMPSU has a power source so enable it
            self._set_pin_state(self.power_control.pin, state)
            self._set_pin_direction(self.power_control.pin, 0)  # in case it gets corrupted by other code
        self.power_state = state
        return self.power_state    


    # Set the keep alive period - this is the time in milli-seconds that the PWM outputs will be kept on
    def set_keep_alive(self, period):
        self.keep_alive_period = period

    
    # Only one PWM frequency (in Hz) is supported for all outputs due to timer limitations
    def set_freq(self, freq) -> bool:
        if self.pwm_setup_failed:
            return False
        for i, pwm in enumerate(self.PWMOutput):
            try:
                pwm.freq(freq)
                print(f"H:{self.config.port}:PWM[{i}] freq: {freq}Hz")
            except:
                print(f"H:{self.config.port}:PWM[{i}] freq: {freq}Hz set failed")
                return False
        return True
    

    # Set all 4 PWM duty cycles in one go (0-65535)
    def set_pwm(self, pwms) -> bool:
        if self.pwm_setup_failed:
            return False
        self.time_since_last_update = 0
        self.outputs_energised = any(pwms)
        for i, pwm in enumerate(pwms):
            if pwm != self.PWMOutput[i].duty_u16():
                # pwm duty cycle has changed so update it
                try:
                    self.PWMOutput[i].duty_u16(pwm)
                    print(f"H:{self.config.port}:PWM[{i}]:{pwm}")
                except:
                    print(f"H:{self.config.port}:PWM[{i}]:{pwm} set failed")
                    return False
        return True
    

    def _set_pin_state(self, pin, state):
        try:
            i2c = I2C(7)
            output_reg = i2c.readfrom_mem(pin[0], 0x02+pin[1], 1)[0]
            output_reg = (output_reg | pin[2]) if state else (output_reg & ~pin[2])
            i2c.writeto_mem(pin[0], 0x02+pin[1], bytes([output_reg]))
            print(f"H:Write to {hex(pin[0])} address {hex(0x02+pin[1])} value {hex(output_reg)}")
        except Exception as e:
            print(f"H:{self.config.port}:access to I2C(7) failed: {e}")


    def _get_pin_state(self, pin) -> bool:
        try:
            i2c = I2C(7)
            input_reg = i2c.readfrom_mem(pin[0], 0x00+pin[1], 1)[0]
            return (input_reg & pin[2]) != 0
        except Exception as e:
            print(f"H:{self.config.port}:access to I2C(7) failed: {e}")


    def _set_pin_direction(self, pin, direction):
        try:
            # Use a Try in case access to i2C(7) is blocked for apps in future
            # presumably if this happens then the code will have been updated to
            # handle the GPIO direction correctly anyway.
            i2c = I2C(7)
            config_reg = i2c.readfrom_mem(pin[0], 0x04+pin[1], 1)[0]
            config_reg = (config_reg | pin[2]) if (1 == direction) else (config_reg & ~pin[2])
            i2c.writeto_mem(pin[0], 0x04+pin[1], bytes([config_reg]))
        except Exception as e:
            print(f"H:{self.config.port}:access to I2C(7) failed: {e}")
    
__app_export__ = HexDriveApp
