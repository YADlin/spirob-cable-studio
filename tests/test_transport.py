"""Real PyModbus framing/transactions over an adversarial serial test double."""
import unittest
from unittest.mock import patch
from pymodbus import FramerType
from pymodbus.exceptions import ConnectionException, ModbusIOException
from spirob_cable.transport import BoundedSerialClient, SerialDeadlineError


def ascii_frame(payload):
    return b':'+(payload+bytes([(-sum(payload)) & 255])).hex().upper().encode()+b'\r\n'


class SerialStub:
    def __init__(self, clock, mode):
        self.clock, self.mode = clock, mode
        self.timeout = .3; self.write_timeout = None; self.is_open = True
        self.writes = []; self.data = b''; self.sent = False

    @property
    def in_waiting(self):
        return 1 if self.sent and (self.data or self.mode in ('noise','wrong_id','bad_checksum')) else 0

    def write(self, request):
        self.sent = True; self.writes.append(request)
        if self.mode == 'short_write': return 2
        if self.mode == 'blocked_write':
            self.clock[0] += self.write_timeout
            from serial import SerialTimeoutException
            raise SerialTimeoutException('Write timeout')
        payload = bytes.fromhex(request[1:-2].decode())[:-1]
        if self.mode == 'exception': self.data = ascii_frame(bytes([payload[0],0x83,2]))
        elif self.mode == 'valid_write': self.data = ascii_frame(payload)
        elif self.mode in ('valid','wrong_id','bad_checksum'):
            self.data = ascii_frame(bytes([7 if self.mode=='wrong_id' else payload[0],3,2,1,78]))
            if self.mode == 'bad_checksum': self.data = self.data[:-4]+b'00\r\n'
        return len(request)

    def read(self, count):
        if self.data:
            self.clock[0] += .005
            result, self.data = self.data[:count], self.data[count:]
            return result
        if self.sent and self.mode in ('noise','wrong_id','bad_checksum'):
            self.clock[0] += .02
            return b'x'
        self.clock[0] += self.timeout
        return b''


class TransportTests(unittest.TestCase):
    def make(self, mode):
        clock = [100.0]
        self.addCleanup(patch.stopall)
        patch('spirob_cable.transport.time.monotonic',side_effect=lambda:clock[0]).start()
        client = BoundedSerialClient('/unused',framer=FramerType.ASCII,baudrate=9600,timeout=.3,retries=0)
        client.socket = SerialStub(clock,mode)
        return client

    def read(self, client):
        return client.read_holding_registers(address=10,count=1,device_id=5)

    def test_fragmented_valid_ascii_is_decoded_and_port_settings_restored(self):
        c=self.make('valid'); response=self.read(c)
        self.assertEqual(response.registers,[334]); self.assertEqual(response.dev_id,5)
        self.assertEqual(c.socket.timeout,.3); self.assertIsNone(c.socket.write_timeout)
        self.assertEqual(len(c.socket.writes),1)

    def test_continuous_noise_has_one_request_deadline_and_no_resend(self):
        c=self.make('noise')
        with self.assertRaises(SerialDeadlineError): self.read(c)
        self.assertGreater(c.last_io['rx_bytes'],0)
        self.assertLess(c.last_io['elapsed_s'],.33); self.assertEqual(len(c.socket.writes),1)

    def test_wrong_id_plus_noise_does_not_satisfy_request_or_extend_deadline(self):
        c=self.make('wrong_id')
        with self.assertRaises(SerialDeadlineError): self.read(c)
        self.assertLess(c.last_io['elapsed_s'],.33)

    def test_bad_checksum_plus_noise_cannot_keep_request_alive(self):
        c=self.make('bad_checksum')
        with self.assertRaises(SerialDeadlineError): self.read(c)
        self.assertLess(c.last_io['elapsed_s'],.33)

    def test_silent_motor_is_bounded_and_next_id_can_be_tested(self):
        c=self.make('silent')
        with self.assertRaises(SerialDeadlineError): self.read(c)
        self.assertEqual(c.last_io['rx_bytes'],0)
        c.socket.mode='valid'
        r=c.read_holding_registers(address=10,count=1,device_id=7)
        self.assertEqual(r.dev_id,7); self.assertEqual(r.registers,[334])

    def test_modbus_exception_response_is_preserved(self):
        c=self.make('exception'); self.assertTrue(self.read(c).isError())

    def test_write_acknowledgement_keeps_standard_parser_checks(self):
        c=self.make('valid_write')
        r=c.write_register(address=2,value=0x0700,device_id=5)
        self.assertFalse(r.isError()); self.assertEqual(r.address,2); self.assertEqual(r.registers,[0x0700])

    def test_partial_write_is_an_error_not_a_success(self):
        c=self.make('short_write')
        with self.assertRaises(ModbusIOException): self.read(c)

    def test_write_has_a_finite_remaining_timeout(self):
        from serial import SerialTimeoutException
        c=self.make('blocked_write')
        with self.assertRaises(SerialTimeoutException): self.read(c)
        self.assertAlmostEqual(c.last_io['elapsed_s'],.3)

    def test_closed_port_never_automatically_reconnects(self):
        c=self.make('valid'); c.socket=None
        with patch.object(c,'connect') as connect:
            with self.assertRaises(ConnectionException): self.read(c)
            connect.assert_not_called()
