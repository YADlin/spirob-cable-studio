"""Verified RMCS-2303 register protocol plus an explicit offline simulator.

One worker owns one drive. No auto-resume after communication failure.
"""
import math
import time
from .model import COUNTS_PER_REV, MAX_MOTOR_RPM, Rejected, signed_count

CONTROL, ACCEL, SPEED = 2, 12, 14
DISABLE, HOLD = 0x0700, 0x0701
POSITION_DISABLED, POSITION_ENABLED = 0x0200, 0x0201


class DriveFault(RuntimeError):
    pass


def validate_rpm(rpm):
    if type(rpm) is not int or not 1 <= rpm <= MAX_MOTOR_RPM:
        raise Rejected(f"Motor RPM must be an integer from 1 to {MAX_MOTOR_RPM}")


class SerialDrive:
    demo = False

    def __init__(self, settings, journal, client=None):
        from pymodbus import FramerType
        from pymodbus.client import ModbusSerialClient
        self.settings, self.journal = settings, journal
        self.owns_client = client is None
        self.client = client if client is not None else ModbusSerialClient(
            port=settings.port, framer=FramerType.ASCII, baudrate=9600,
            bytesize=8, parity="N", stopbits=1, timeout=0.3, retries=0)

    def open(self):
        if not self.client.connected and not self.client.connect():
            raise DriveFault("Cannot open serial port. Check path and dialout permissions")

    def close(self):
        if self.owns_client:
            self.client.close()

    def read(self, address):
        # Prevent PyModbus silently reopening after a known disconnect.
        if not self.client.connected:
            raise DriveFault("Serial port disconnected")
        r = self.client.read_holding_registers(address=address, count=1,
                                             device_id=self.settings.device_id)
        if r is None or r.isError() or len(r.registers) != 1:
            raise DriveFault(f"Invalid read at {address}: {r}")
        return r.registers[0]

    def write(self, address, value, emergency=False):
        if not self.client.connected:
            raise DriveFault("Serial port disconnected; command not sent")
        def record(event):
            try:
                self.journal.event(event, address=address, value=value)
            except Exception:
                if not emergency:
                    raise
        record("write_requested")
        r = self.client.write_register(address=address, value=value,
                                      device_id=self.settings.device_id)
        if r is None or r.isError() or r.address != address or r.registers != [value]:
            raise DriveFault(f"Write not acknowledged at {address}: {r}")
        record("write_acknowledged")

    def position(self):
        # High/low/high minimizes split-word rollover errors. Atomic snapshots
        # are undocumented by the manufacturer; no stale value is substituted.
        for _ in range(2):
            h1, low, h2 = self.read(22), self.read(20), self.read(22)
            if h1 == h2:
                value = (h2 << 16) | low
                return value - (1 << 32) if value & 0x80000000 else value
        raise DriveFault("Inconsistent position words")

    def target(self, count, cancelled=lambda: False):
        signed_count(count)
        if cancelled():
            return False
        unsigned = count & 0xFFFFFFFF
        self.write(16, unsigned & 0xFFFF)
        if cancelled():  # Do not commit if STOP arrived during the low-word write.
            return False
        self.write(18, unsigned >> 16)
        return True

    def disable(self):
        self.write(CONTROL, DISABLE, emergency=True)

    def hold(self):
        self.write(CONTROL, HOLD, emergency=True)

    def configure(self):
        if self.read(10) != 334:
            raise DriveFault("LPR is not 334. Check motor/configuration")
        self.write(CONTROL, POSITION_DISABLED)
        self.write(ACCEL, self.settings.acceleration_raw)
        self.write(SPEED, self.settings.motor_rpm)
        if self.read(ACCEL) != self.settings.acceleration_raw or self.read(SPEED) != self.settings.motor_rpm:
            raise DriveFault("Speed/acceleration readback mismatch")

    def enable(self):
        self.write(CONTROL, POSITION_ENABLED)

    def set_speed(self, rpm, cancelled=lambda: False):
        validate_rpm(rpm)
        if cancelled():
            return False
        self.write(SPEED, rpm)
        if self.read(SPEED) != rpm:
            raise DriveFault("Speed readback mismatch; new target not sent")
        return not cancelled()


class DemoDrive:
    """Constant-speed simulated encoder, not a motor or cable dynamics model."""
    demo = True

    def __init__(self, settings, journal):
        self.settings, self.journal = settings, journal
        # More than two turns deliberately proves that positions are not wrapped.
        self.count = float(COUNTS_PER_REV * 2 + 12345)
        self.goal = self.count
        self.enabled = False
        self.last = time.monotonic()
        self.failed = False
        self.motor_rpm = settings.motor_rpm

    def open(self):
        pass

    def close(self):
        self.enabled = False

    def configure(self):
        self.motor_rpm = self.settings.motor_rpm

    def set_speed(self, rpm, cancelled=lambda: False):
        validate_rpm(rpm)
        if cancelled():
            return False
        self.position()  # Integrate elapsed time using the previous speed.
        self.motor_rpm = rpm
        self.journal.event("demo_speed", motor_rpm=rpm)
        return not cancelled()

    def position(self):
        if self.failed:
            raise DriveFault("Injected demo communication loss")
        now = time.monotonic()
        if self.enabled:
            step = self.settings.speed_for_rpm(self.motor_rpm) / self.settings.mm_per_count * (now - self.last)
            delta = self.goal - self.count
            self.count += math.copysign(min(abs(delta), step), delta)
        self.last = now
        return round(self.count)

    def target(self, count, cancelled=lambda: False):
        signed_count(count)
        if cancelled():
            return False
        self.position()
        self.goal = float(count)
        self.journal.event("demo_target", count=count)
        return True

    def enable(self):
        self.enabled = True

    def disable(self):
        self.position()
        self.enabled = False

    def hold(self):
        self.goal = self.position()
        self.enabled = True
