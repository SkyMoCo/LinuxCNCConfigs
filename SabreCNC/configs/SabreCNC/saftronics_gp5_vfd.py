#!/usr/bin/env python3

# Saftronics GP5 VFD LinuxCNC userspace component
#
# Copyright (C) 2024 Forest Darling <fdarling@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import time
import minimalmodbus
import serial
import hal
import ctypes
import argparse
import traceback

MODBUS_DELAY = 0.01

# Modbus register addresses
VFD_REG_COMMAND_OPERATIONAL_SIGNALS = 0x01
VFD_REG_COMMAND_FREQUENCY_REFERENCE = 0x02
VFD_REG_MONITOR_STATUS_SIGNAL = 0x20
VFD_REG_MONITOR_DRIVE_FAULT   = 0x21
VFD_REG_MONITOR_FREQUENCY_REFERENCE = 0x23
VFD_REG_MONITOR_OUTPUT_FREQUENCY = 0x24
VFD_REG_MONITOR_OUTPUT_CURRENT = 0x27
VFD_REG_MONITOR_OUTPUT_VOLTAGE = 0x28
VFD_REG_MONITOR_DRIVE_STATUS = 0x2C
VFD_REG_MONITOR_OUTPUT_POWER   = 0x37

# status bit masks
VFD_REG_MONITOR_STATUS_SIGNALS_BIT_RUN_COMMAND              = 1 << 0
VFD_REG_MONITOR_STATUS_SIGNALS_BIT_REVERSE_CMD              = 1 << 1
VFD_REG_MONITOR_STATUS_SIGNALS_BIT_DRIVE_READY              = 1 << 2
VFD_REG_MONITOR_STATUS_SIGNALS_BIT_DRIVE_FAULT              = 1 << 3
VFD_REG_MONITOR_STATUS_SIGNALS_BIT_DATA_SETTING_ERROR       = 1 << 4
VFD_REG_MONITOR_STATUS_SIGNALS_BIT_MULTI_FUNC_OUTPUT_SET_1  = 1 << 5
VFD_REG_MONITOR_STATUS_SIGNALS_BIT_MULTI_FUNC_OUTPUT_SET_2  = 1 << 6

# fault bit masks
VFD_REG_MONITOR_DRIVE_FAULT_BIT_OVERCURRENT    = 1 <<  0 # oC, GF, SC
VFD_REG_MONITOR_DRIVE_FAULT_BIT_OVERVOLTAGE    = 1 <<  1 # uV
VFD_REG_MONITOR_DRIVE_FAULT_BIT_DRIVE_OVERLOAD = 1 <<  2 # oL2
VFD_REG_MONITOR_DRIVE_FAULT_BIT_DRIVE_OVERHEAT = 1 <<  3 # oH1, oH2
#= 1 << 4 # UNUSED
VFD_REG_MONITOR_DRIVE_FAULT_BIT_MAIN_CIRCUIT   = 1 <<  5
VFD_REG_MONITOR_DRIVE_FAULT_BIT_BRAKING_TRANS  = 1 <<  6 # rr, rH
VFD_REG_MONITOR_DRIVE_FAULT_BIT_EXTERNAL       = 1 <<  7 # EF0, EF2-EF6
VFD_REG_MONITOR_DRIVE_FAULT_BIT_HARDWARE       = 1 <<  8 # CPFx
VFD_REG_MONITOR_DRIVE_FAULT_BIT_MOTOR_OVERLOAD = 1 <<  9 # oL1, oL3
#= 1 << 10 # UNUSED
VFD_REG_MONITOR_DRIVE_FAULT_BIT_UNDERVOLTAGE   = 1 << 11
VFD_REG_MONITOR_DRIVE_FAULT_BIT_POWER_LOSS     = 1 << 12 # Uu1, Uu2, Uu3
VFD_REG_MONITOR_DRIVE_FAULT_BIT_PHASE_LOSS     = 1 << 13 # SPi/SPo

# drive status bit masks
VFD_REG_MONITOR_DRIVE_STATUS_BIT_RUN_COMMAND_RECEIVED            = 1 << 0
VFD_REG_MONITOR_DRIVE_STATUS_BIT_DURING_ZERO_SPEED               = 1 << 1
VFD_REG_MONITOR_DRIVE_STATUS_BIT_DURING_FREQUENCY_COINCIDENCE    = 1 << 2
VFD_REG_MONITOR_DRIVE_STATUS_BIT_ARBITRARY_FREQUENCY_COINCIDENCE = 1 << 3
VFD_REG_MONITOR_DRIVE_STATUS_BIT_OUTPUT_FREQUENCY_LE_N075        = 1 << 4
VFD_REG_MONITOR_DRIVE_STATUS_BIT_OUTPUT_FREQUENCY_GE_N075        = 1 << 5
VFD_REG_MONITOR_DRIVE_STATUS_BIT_DRIVE_READY                     = 1 << 6
# ...TODO the rest of these...
VFD_REG_MONITOR_DRIVE_STATUS_BIT_COMMUNICATION_ERROR             = 1 << 15

# command bit masks
VFD_REG_COMMAND_OPERATIONAL_SIGNALS_BIT_RUN            = 1 << 0
VFD_REG_COMMAND_OPERATIONAL_SIGNALS_BIT_REVERSE_RUN    = 1 << 1
VFD_REG_COMMAND_OPERATIONAL_SIGNALS_BIT_EXTERNAL_FAULT = 1 << 2
VFD_REG_COMMAND_OPERATIONAL_SIGNALS_BIT_FAULT_RESET    = 1 << 3

# a helper lookup table for command line argument parsing
PARITY_FROM_COMMAND_LINE_OPTION = {
    'none': serial.PARITY_NONE,
    'even': serial.PARITY_EVEN,
    'odd':  serial.PARITY_ODD
}

class Saftronics_GP5_Modbus_Driver:
    def __init__(self, port, baudrate, parity, slave_addr, h):
        self.instrument = minimalmodbus.Instrument(port, slave_addr)
        self.instrument.serial.baudrate = baudrate
        self.instrument.serial.bytesize = 8 # fixed
        self.instrument.serial.parity   = parity
        self.instrument.serial.stopbits = 1 # fixed
        self.instrument.serial.timeout  = 0.05 # seconds

        self.h = h

    def read_register(self, *args, **kwargs):
        value = self.instrument.read_register(*args, **kwargs)
        time.sleep(MODBUS_DELAY)
        return value

    def read_registers(self, *args, **kwargs):
        value = self.instrument.read_registers(*args, **kwargs)
        time.sleep(MODBUS_DELAY)
        return value

    def write_register(self, *args, **kwargs):
        value = self.instrument.write_register(*args, **kwargs)
        time.sleep(MODBUS_DELAY)

    def run(self):
        # parameter needed to convert electrical frequency to RPM
        pole_pair_count = self.h['pole-pair-count']
        if pole_pair_count == 0:
            pole_pair_count = 1

        # read status word
        status_signal_word = self.read_register(VFD_REG_MONITOR_STATUS_SIGNAL)
        running = bool(status_signal_word & VFD_REG_MONITOR_STATUS_SIGNALS_BIT_RUN_COMMAND)
        running_rev = bool(status_signal_word & VFD_REG_MONITOR_STATUS_SIGNALS_BIT_REVERSE_CMD)
        ready = bool(status_signal_word & VFD_REG_MONITOR_STATUS_SIGNALS_BIT_DRIVE_READY)
        faulted = bool(status_signal_word & VFD_REG_MONITOR_STATUS_SIGNALS_BIT_DRIVE_FAULT)

        # read the fault code
        fault_code = self.read_register(VFD_REG_MONITOR_DRIVE_FAULT)
        time.sleep(MODBUS_DELAY)

        # read extended status word
        drive_status_word = self.read_register(VFD_REG_MONITOR_DRIVE_STATUS)
        time.sleep(MODBUS_DELAY)
        at_zero_speed = bool(drive_status_word & VFD_REG_MONITOR_DRIVE_STATUS_BIT_DURING_ZERO_SPEED)
        at_speed = bool(drive_status_word & VFD_REG_MONITOR_DRIVE_STATUS_BIT_DURING_FREQUENCY_COINCIDENCE)

        # TODO read the fixed point decimal count from the VFD's parameters (n103, etc.), assuming defaults for now

        # read everything else
        frequency_int, \
        dummy_int1, \
        dummy_int2, \
        current_int, \
        voltage_int = self.read_registers(VFD_REG_MONITOR_OUTPUT_FREQUENCY, 5) # frequency, (2x padding), current, voltage
        power_int = self.read_register(VFD_REG_MONITOR_OUTPUT_POWER) # power
        # frequency_out = float(ctypes.c_short(frequency_int).value) / 10.0
        frequency_out = float(frequency_int) / 10.0
        rpm_abs_out = frequency_out * 60.0 / pole_pair_count
        rpm_out = -rpm_abs_out if running_rev else rpm_abs_out

        # update HAL pins (read from the VFD)
        self.h['connected-out'] = True
        self.h['running-out'] = running
        self.h['rev-out'] = running_rev
        self.h['ready-out'] = ready
        self.h['fault-out'] = faulted
        self.h['fault-code'] = fault_code
        self.h['freq-out'] = frequency_out
        self.h['voltage-out'] = float(voltage_int)
        self.h['current-out'] = float(current_int) / 10.0
        self.h['power-out'] = float(power_int) / 10.0
        self.h['rpm-abs-out'] = rpm_abs_out
        self.h['rpm-out'] = rpm_out
        self.h['at-zero-speed-out'] = at_zero_speed
        self.h['at-speed-out'] = at_speed

        # craft the RPM value going out to the VFD
        commanded_rpm = self.h['rpm-in']
        if commanded_rpm > self.h['maximum-rpm']:
            commanded_rpm = self.h['maximum-rpm']
        if commanded_rpm < self.h['minimum-rpm'] and commanded_rpm != 0.0:
            commanded_rpm = self.h['minimum-rpm']
        commanded_frequency = abs(commanded_rpm) / 60.0 * pole_pair_count
        commanded_frequency_int = int(commanded_frequency*10.0)
        if (commanded_frequency_int > 0xFFFF):
            commanded_frequency_int = 0xFFFF
        is_reversed = (bool(self.h['reverse-in']) != bool(commanded_rpm < 0))
        enabled = (self.h['enable-in'] and commanded_frequency != 0)

        # craft the control word going out to the VFD
        operational_signals = 0

        if self.h['fake-in']:
            operational_signals |= VFD_REG_COMMAND_OPERATIONAL_SIGNALS_BIT_EXTERNAL_FAULT
        elif self.h['reset-in'] or (faulted and fault_code == 0):
            operational_signals |= VFD_REG_COMMAND_OPERATIONAL_SIGNALS_BIT_FAULT_RESET
        else:
            if enabled:
                operational_signals |= VFD_REG_COMMAND_OPERATIONAL_SIGNALS_BIT_RUN
            if is_reversed:
                operational_signals |= VFD_REG_COMMAND_OPERATIONAL_SIGNALS_BIT_REVERSE_RUN

        # actually write the frequency / command
        self.write_register(VFD_REG_COMMAND_FREQUENCY_REFERENCE, commanded_frequency_int)
        self.write_register(VFD_REG_COMMAND_OPERATIONAL_SIGNALS, operational_signals)

    def main_loop(self):
        # first try and shut off the VFD before doing anything else
        self.write_register(VFD_REG_COMMAND_OPERATIONAL_SIGNALS, 0x0)

        # keep looping when there are no exceptions
        while True:
            self.run()

def resetInputPinsAndParams(h):
    h['minimum-rpm'] = 0
    h['maximum-rpm'] = 24000
    h['pole-pair-count'] = 1
    h['rpm-in'] = 0.0
    h['enable-in'] = False
    h['reverse-in'] = False
    h['fake-in'] = False
    h['reset-in'] = False

def resetOutputPins(h):
    h['running-out'] = False
    h['rev-out'] = False
    h['ready-out'] = False
    h['fault-out'] = False
    h['connected-out'] = False
    h['fault-code'] = 0
    h['freq-out'] = 0.0
    h['voltage-out'] = 0.0
    h['current-out'] = 0.0
    h['power-out'] = 0.0
    h['rpm-abs-out'] = 0.0
    h['rpm-out'] = 0.0
    h['at-zero-speed-out'] = False
    h['at-speed-out'] = False

def main():
    # parse command line arguments
    parser = argparse.ArgumentParser(
        prog='Saftronics GP5 VFD LinuxCNC driver',
        description='userspace LinuxCNC HAL component to control a Saftronics GP5 VFD using Modbus RTU over RS-232')
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="serial port name or device path")
    parser.add_argument("--baudrate", type=int, choices=[2400, 4800, 9600], default=9600, help="serial port baudrate")
    parser.add_argument("--parity", type=str, choices=PARITY_FROM_COMMAND_LINE_OPTION.keys(), default="none", help="serial port parity")
    parser.add_argument("--slave-address", type=int, choices=range(1,248), metavar="[1-247]", default=1, help="Modbus slave address")
    args = parser.parse_args()
    print("Using serial port settings:", args.port, args.baudrate, PARITY_FROM_COMMAND_LINE_OPTION[args.parity], "Modbus slave:", args.slave_address)

    # create HAL component
    h = hal.component("saftronics-gp5-vfd")

    # create HAL parameters
    h.newparam("minimum-rpm", hal.HAL_FLOAT, hal.HAL_RW)
    h.newparam("maximum-rpm", hal.HAL_FLOAT, hal.HAL_RW)
    h.newparam("pole-pair-count", hal.HAL_U32, hal.HAL_RW)

    # create HAL input pins
    h.newpin("rpm-in", hal.HAL_FLOAT, hal.HAL_IN)
    h.newpin("enable-in", hal.HAL_BIT, hal.HAL_IN)
    h.newpin("reverse-in", hal.HAL_BIT, hal.HAL_IN)
    h.newpin("fake-in", hal.HAL_BIT, hal.HAL_IN)
    h.newpin("reset-in", hal.HAL_BIT, hal.HAL_IN)

    # create HAL output pins
    h.newpin("running-out", hal.HAL_BIT, hal.HAL_OUT)
    h.newpin("rev-out", hal.HAL_BIT, hal.HAL_OUT)
    h.newpin("ready-out", hal.HAL_BIT, hal.HAL_OUT)
    h.newpin("fault-out", hal.HAL_BIT, hal.HAL_OUT)
    h.newpin("connected-out", hal.HAL_BIT, hal.HAL_OUT)
    h.newpin("fault-code", hal.HAL_U32, hal.HAL_OUT)
    h.newpin("freq-out", hal.HAL_FLOAT, hal.HAL_OUT)
    h.newpin("voltage-out", hal.HAL_FLOAT, hal.HAL_OUT)
    h.newpin("current-out", hal.HAL_FLOAT, hal.HAL_OUT)
    h.newpin("power-out", hal.HAL_FLOAT, hal.HAL_OUT)
    #h.newpin("torque-out", hal.HAL_FLOAT, hal.HAL_OUT)
    h.newpin("rpm-abs-out", hal.HAL_FLOAT, hal.HAL_OUT)
    h.newpin("rpm-out", hal.HAL_FLOAT, hal.HAL_OUT)
    h.newpin("at-zero-speed-out", hal.HAL_BIT, hal.HAL_OUT)
    h.newpin("at-speed-out", hal.HAL_BIT, hal.HAL_OUT)

    # initialize the output pins
    resetInputPinsAndParams(h)
    resetOutputPins(h)

    # signal that we are done setting up the pins
    h.ready()

    # keep looping, even with exceptions (except for CTRL+C)
    while True:
        # run a main loop that continuously read/writes HAL pins
        try:
            driver = Saftronics_GP5_Modbus_Driver(args.port, args.baudrate, PARITY_FROM_COMMAND_LINE_OPTION[args.parity], args.slave_address, h)
            driver.main_loop()
        except KeyboardInterrupt:
            raise SystemExit
        except Exception as e:
            print("Modbus error:", e)
            print(traceback.format_exc())
            pass

        # if we got here, an exception occurred (or main_loop() returned) and we should cool off for a bit
        resetOutputPins(h)
        print("Recreating Modbus context...")
        time.sleep(2.0)

if __name__ == "__main__":
    main()
