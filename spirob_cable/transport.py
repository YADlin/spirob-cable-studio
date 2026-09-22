"""Bound serial request time while retaining PyModbus's ASCII parser and checks.

PyModbus 3.11.3 sync_get_response loops until a valid PDU or an empty receive.
A stream of invalid bytes can therefore outlive its per-receive timeout. This
client gives the whole request one monotonic budget, including the write, and
does not wait for an idle receive buffer. Use from the single serial owner only.
OS-level open/ioctl calls and scheduler delays are not hard-real-time bounded.
"""
import time
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ConnectionException, ModbusIOException


class SerialDeadlineError(ModbusIOException):
    pass


class BoundedSerialClient(ModbusSerialClient):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._deadline = None
        self.last_io = {}

    def execute(self, no_response_expected, request):
        # An explicit open is required; do not silently reconnect during motion.
        if not self.connected:
            raise ConnectionException('Serial port is closed; reconnect explicitly')
        start = time.monotonic()
        self._deadline = start + self.comm_params.timeout_connect
        self.last_io = {'device_id': request.dev_id, 'function': request.function_code,
                        'address': getattr(request, 'address', None), 'tx_hex': '',
                        'rx_bytes': 0, 'rx_hex_prefix': '', 'discarded_before_tx': 0}
        socket = self.socket
        old_timeout, old_write_timeout = socket.timeout, socket.write_timeout
        try:
            result = super().execute(no_response_expected, request)
            self._remaining()  # Also reject a reply completed after the budget.
            return result
        finally:
            self.last_io['elapsed_s'] = time.monotonic() - start
            self._deadline = None
            if self.socket is socket and socket.is_open:
                socket.timeout, socket.write_timeout = old_timeout, old_write_timeout

    def _remaining(self):
        if self._deadline is None:
            raise RuntimeError('Serial I/O outside a bounded request')
        remaining = self._deadline - time.monotonic()
        if remaining <= 0:
            raise SerialDeadlineError(
                f'ID {self.last_io["device_id"]}: no complete valid response within '
                f'{self.comm_params.timeout_connect:.3f} s; '
                f'RX bytes={self.last_io["rx_bytes"]}, '
                f'RX prefix={self.last_io["rx_hex_prefix"] or "<none>"}')
        return remaining

    def send(self, request, addr=None):
        if not self.socket:
            raise ConnectionException('Serial port disconnected')
        self._remaining()
        if not request:
            return 0
        # Discard only the finite backlog already present, never wait for quiet.
        waiting = self.socket.in_waiting
        if waiting:
            self.socket.timeout = 0
            discarded = self.socket.read(min(waiting, 4096))
            self.last_io['discarded_before_tx'] = len(discarded)
        self.socket.write_timeout = self._remaining()
        self.last_io['tx_hex'] = request.hex(' ')
        size = self.socket.write(request)
        self._remaining()
        if size != len(request):
            raise ModbusIOException(f'Incomplete serial write: {size}/{len(request)} bytes')
        return size

    def recv(self, size):
        if not self.socket:
            raise ConnectionException('Serial port disconnected')
        self.socket.timeout = self._remaining()
        # Read available bytes, or wait for one byte. The parser assembles frames.
        count = min(max(1, self.socket.in_waiting), 4096)
        if size is not None:
            count = min(count, size)
        data = self.socket.read(count)
        self.last_io['rx_bytes'] += len(data)
        prefix = bytes.fromhex(self.last_io['rx_hex_prefix'])
        self.last_io['rx_hex_prefix'] = (prefix + data)[:64].hex(' ')
        self._remaining()
        self.last_frame_end = round(time.time(), 6)
        return data
