"""
AeroHawk hardware layer: PCA9685 board, steering servo, and IBT-2 drive motor.

This file has NO ROS in it. drive_node.py imports it, and you can also test
the hardware directly on the Jetson:

    python3 hardware.py servo     # sweep the steering left / center / right
    python3 hardware.py motor     # WHEELS OFF THE GROUND: slow forward, stop, slow reverse

ALL settings (I2C bus, channel numbers, limits) live in ONE place:
    aerohawk_bringup/config/drive_params.yaml
Nothing in this file hard-codes them. build_hardware() reads them from there.

Wiring (all signal wires go to the PCA9685; channel numbers are in the YAML):
    Steering servo signal, IBT-2 RPWM, LPWM, R_EN, L_EN -> PCA9685 channels
    IBT-2 VCC / GND -> 5 V / GND shared with the PCA9685 and Jetson

Requires: python3-smbus (sudo apt install python3-smbus) or smbus2.
Your user must be allowed to use I2C: sudo usermod -aG i2c $USER  (then log out/in).
"""

import os
import time

try:
    from smbus2 import SMBus
except ImportError:  # fall back to the apt package
    from smbus import SMBus


def _clamp(value, low, high):
    return max(low, min(high, value))


def _sign(value):
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


# ==========================================================================
# 1. The board
# ==========================================================================
class PCA9685:
    """Talks to the PCA9685 over I2C. Knows channels, not servos or motors.

    Every channel shares one PWM frequency (50 Hz here, because the servo needs it).
    """

    # Register addresses from the PCA9685 datasheet
    _MODE1 = 0x00
    _MODE2 = 0x01
    _LED0_ON_L = 0x06      # each channel uses 4 registers starting here
    _ALL_LED_ON_L = 0xFA
    _PRESCALE = 0xFE

    # MODE1 / MODE2 bits
    _RESTART = 0x80
    _AUTO_INCREMENT = 0x20
    _SLEEP = 0x10
    _OUTDRV = 0x04         # totem-pole outputs (right for servos and the IBT-2 inputs)

    _FULL_BIT = 0x10       # "fully on" / "fully off" flag in the ON_H / OFF_H registers

    def __init__(self, bus, address, frequency, oscillator_hz=25_000_000):
        self.address = address
        self.oscillator_hz = oscillator_hz
        self.frequency = None
        self._bus = SMBus(bus)

        self.all_off()
        self._bus.write_byte_data(self.address, self._MODE2, self._OUTDRV)
        self._bus.write_byte_data(self.address, self._MODE1, self._AUTO_INCREMENT)
        time.sleep(0.005)  # oscillator start-up
        self.set_frequency(frequency)

    def set_frequency(self, frequency):
        """Set the PWM frequency for ALL channels."""
        prescale = round(self.oscillator_hz / (4096.0 * frequency)) - 1
        prescale = int(_clamp(prescale, 3, 255))

        old_mode = self._bus.read_byte_data(self.address, self._MODE1)
        sleep_mode = (old_mode & ~self._RESTART) | self._SLEEP
        self._bus.write_byte_data(self.address, self._MODE1, sleep_mode)   # must sleep to change prescale
        self._bus.write_byte_data(self.address, self._PRESCALE, prescale)
        self._bus.write_byte_data(self.address, self._MODE1, old_mode & ~self._SLEEP)
        time.sleep(0.005)
        self._bus.write_byte_data(self.address, self._MODE1,
                                  (old_mode & ~self._SLEEP) | self._RESTART | self._AUTO_INCREMENT)

        # The real frequency is slightly off from the request because prescale is a whole number
        self.frequency = self.oscillator_hz / (4096.0 * (prescale + 1))

    def _write_channel(self, channel, on_l, on_h, off_l, off_h):
        if not 0 <= channel <= 15:
            raise ValueError(f"PCA9685 channel must be 0-15, got {channel}")
        register = self._LED0_ON_L + 4 * channel
        self._bus.write_i2c_block_data(self.address, register, [on_l, on_h, off_l, off_h])

    def set_ticks(self, channel, off_tick):
        """Output goes high at tick 0 and low at off_tick (0-4095 out of 4096)."""
        off_tick = int(_clamp(off_tick, 0, 4095))
        self._write_channel(channel, 0, 0, off_tick & 0xFF, off_tick >> 8)

    def set_pulse_us(self, channel, microseconds):
        """Send a pulse of this many microseconds each cycle (used for servos)."""
        period_us = 1_000_000.0 / self.frequency
        self.set_ticks(channel, round(microseconds / period_us * 4096))

    def set_duty(self, channel, fraction):
        """Set duty cycle from 0.0 (always low) to 1.0 (always high)."""
        fraction = _clamp(fraction, 0.0, 1.0)
        if fraction <= 0.0:
            self.full_off(channel)
        elif fraction >= 1.0:
            self.full_on(channel)
        else:
            self.set_ticks(channel, round(fraction * 4096))

    def full_on(self, channel):
        self._write_channel(channel, 0, self._FULL_BIT, 0, 0)

    def full_off(self, channel):
        self._write_channel(channel, 0, 0, 0, self._FULL_BIT)

    def all_off(self):
        self._bus.write_i2c_block_data(self.address, self._ALL_LED_ON_L, [0, 0, 0, self._FULL_BIT])

    def close(self):
        try:
            self.all_off()
        finally:
            self._bus.close()


# ==========================================================================
# 2. Steering
# ==========================================================================
class SteeringServo:
    """Steering servo on one PCA9685 channel.

    set_steering(-1.0) = full left, 0.0 = straight, +1.0 = full right.
    Pulse widths are clamped so the servo can never be driven past its limits.
    """

    def __init__(self, pca, channel, left_us, center_us, right_us, invert):
        self.pca = pca
        self.channel = channel
        self.left_us = left_us
        self.center_us = center_us
        self.right_us = right_us
        self.invert = invert
        self.value = 0.0

    def set_steering(self, value):
        value = _clamp(float(value), -1.0, 1.0)
        self.value = value
        if self.invert:
            value = -value
        # Left and right are scaled separately so an off-center trim still reaches both limits
        if value < 0:
            pulse = self.center_us + value * (self.center_us - self.left_us)
        else:
            pulse = self.center_us + value * (self.right_us - self.center_us)
        low, high = sorted((self.left_us, self.right_us))
        self.pca.set_pulse_us(self.channel, _clamp(pulse, low, high))

    def center(self):
        self.set_steering(0.0)

    def release(self):
        """Stop sending pulses (servo goes limp)."""
        self.pca.full_off(self.channel)


# ==========================================================================
# 3. Drive motor through the IBT-2
# ==========================================================================
class MotorIBT2:
    """35T motor through an IBT-2 (BTS7960), all inputs driven by PCA9685 channels.

    set_speed(-1.0 .. 1.0) sets the TARGET speed. update() must be called
    regularly (drive_node does it on a timer) to ramp toward that target.
    stop() is the immediate safety stop: no ramp, enable pins off.
    """

    def __init__(self, pca, rpwm_channel, lpwm_channel, r_en_channel, l_en_channel,
                 max_duty, ramp_per_second, reverse_pause_s, invert,
                 deadband=0.05):         # ignore tiny commands that wouldn't move the car anyway
        self.pca = pca
        self.rpwm = rpwm_channel
        self.lpwm = lpwm_channel
        self.r_en = r_en_channel
        self.l_en = l_en_channel
        self.max_duty = _clamp(max_duty, 0.0, 1.0)
        self.ramp_per_second = ramp_per_second
        self.reverse_pause_s = reverse_pause_s
        self.deadband = deadband
        self.invert = invert

        self.enabled = False
        self._target = 0.0
        self._current = 0.0
        self._last_direction = 0
        self._zero_since = time.monotonic()
        self._last_update = time.monotonic()

        self.stop()

    # ---- enable pins: the hardware kill switch ----
    def enable(self):
        self.pca.full_on(self.r_en)
        self.pca.full_on(self.l_en)
        self.enabled = True

    def disable(self):
        self.pca.full_off(self.r_en)
        self.pca.full_off(self.l_en)
        self.enabled = False

    # ---- commands ----
    def set_speed(self, value):
        """Set the target speed, -1.0 (full reverse) to 1.0 (full forward)."""
        value = _clamp(float(value), -1.0, 1.0)
        if abs(value) < self.deadband:
            value = 0.0
        self._target = value
        if value != 0.0 and not self.enabled:
            self._last_update = time.monotonic()  # don't count time spent stopped as ramp time
            self.enable()

    def stop(self):
        """Immediate stop: both PWM inputs off and the IBT-2 disabled."""
        self._target = 0.0
        self._current = 0.0
        self._zero_since = time.monotonic()
        self.pca.full_off(self.rpwm)
        self.pca.full_off(self.lpwm)
        self.disable()

    @property
    def speed(self):
        """The speed actually being output right now (after ramping)."""
        return self._current

    def update(self):
        """Move the output one step toward the target. Call ~20+ times per second."""
        now = time.monotonic()
        dt = min(now - self._last_update, 0.1)  # cap so a long gap can't cause a jump
        self._last_update = now

        target = self._target
        current = self._current

        # Never jump straight from forward to reverse: head for zero first
        if current * target < 0:
            target = 0.0

        # After stopping, wait reverse_pause_s before going the other direction
        if (current == 0.0 and target != 0.0 and self._last_direction != 0
                and _sign(target) != self._last_direction
                and now - self._zero_since < self.reverse_pause_s):
            target = 0.0

        step = self.ramp_per_second * dt
        if target > current:
            new = min(current + step, target)
        else:
            new = max(current - step, target)

        if current != 0.0 and new == 0.0:
            self._zero_since = now
        if new != 0.0:
            self._last_direction = _sign(new)

        self._current = new
        self._apply(new)

    def _apply(self, value):
        if self.invert:
            value = -value
        duty = abs(value) * self.max_duty
        if value > 0:
            self.pca.full_off(self.lpwm)          # turn the other side off FIRST
            self.pca.set_duty(self.rpwm, duty)
        elif value < 0:
            self.pca.full_off(self.rpwm)
            self.pca.set_duty(self.lpwm, duty)
        else:
            self.pca.full_off(self.rpwm)
            self.pca.full_off(self.lpwm)


# ==========================================================================
# Build everything from drive_params.yaml settings
# ==========================================================================
def build_hardware(settings):
    """Create (board, steering, motor) from a dict of drive_params.yaml settings.

    drive_node passes in the settings ROS loaded; the test below loads the YAML itself.
    This is the ONLY place that maps setting names to hardware.
    """
    try:
        pca = PCA9685(settings['i2c_bus'], settings['pca9685_address'],
                      settings['pwm_frequency'])
        steering = SteeringServo(
            pca, settings['steering_channel'],
            settings['steering_left_us'], settings['steering_center_us'],
            settings['steering_right_us'], settings['steering_invert'])
        motor = MotorIBT2(
            pca, settings['rpwm_channel'], settings['lpwm_channel'],
            settings['r_en_channel'], settings['l_en_channel'],
            settings['max_duty'], settings['ramp_per_second'],
            settings['reverse_pause_s'], settings['motor_invert'])
    except KeyError as missing:
        raise RuntimeError(f"setting {missing} is missing from drive_params.yaml") from None
    return pca, steering, motor


def load_settings(path):
    """Read drive_node's settings straight from the YAML file (for testing without ROS)."""
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)['drive_node']['ros__parameters']


# Where drive_params.yaml sits relative to this file in the repo
DEFAULT_SETTINGS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', 'aerohawk_bringup', 'config', 'drive_params.yaml')


# ==========================================================================
# Stand-alone hardware test (no ROS needed)
# ==========================================================================
def _test_servo(servo):
    for label, value in [("center", 0.0), ("left", -1.0), ("center", 0.0),
                         ("right", 1.0), ("center", 0.0)]:
        print(f"steering {label} ({value:+.1f})")
        servo.set_steering(value)
        time.sleep(1.0)


def _run_motor(motor, target, seconds):
    motor.set_speed(target)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        motor.update()
        time.sleep(0.02)


def _test_motor(motor):
    input("WHEELS OFF THE GROUND? Press Enter to continue, Ctrl+C to cancel... ")
    print("forward (slow)")
    _run_motor(motor, 0.5, 2.0)
    print("ramping to stop")
    _run_motor(motor, 0.0, 1.0)
    print("reverse (slow)")
    _run_motor(motor, -0.5, 2.0)
    print("stop")
    motor.stop()


if __name__ == "__main__":
    import sys

    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode not in ("servo", "motor"):
        print("usage: python3 hardware.py servo|motor [path/to/drive_params.yaml]")
        sys.exit(1)

    settings_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_SETTINGS_PATH
    settings = load_settings(settings_path)
    board, steering, motor = build_hardware(settings)
    print(f"PCA9685 on bus {settings['i2c_bus']} at {hex(settings['pca9685_address'])}, "
          f"running at {board.frequency:.1f} Hz")
    try:
        if mode == "servo":
            _test_servo(steering)
        else:
            _test_motor(motor)
    except KeyboardInterrupt:
        print("\ncancelled")
    finally:
        board.close()  # every channel off, including the IBT-2 enables
        print("all outputs off")