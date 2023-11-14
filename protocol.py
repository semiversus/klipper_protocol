from serial import Serial
from struct import Struct
from typing import Tuple
import zlib
import json

import crc


_msg_header_struct = Struct('BB')
_crc_struct = Struct('>H')


_crc_config = crc.Configuration(width=16, polynomial=0x1021, init_value=0xFFFF,
                                reverse_input=True, reverse_output=True)
crc_ccitt = crc.Calculator(_crc_config, optimized=True)


def msg_build(data: bytes, sequence_id: int = 0) -> bytes:
    msg = _msg_header_struct.pack(len(data) + 5, 0x10 | (sequence_id & 0x0f)) + data
    msg += _crc_struct.pack(crc_ccitt.checksum(msg)) + b'\x7e'
    return msg


def msg_parse(msg: bytes, sequence_id: int = 0) -> bytes:
    crc_value = crc_ccitt.checksum(msg[:-3])
    if _crc_struct.pack(crc_value) != msg[-3:-1]:
        raise ValueError(f'Wrong CRC checksum in message {msg!r}')
    assert msg[0] == len(msg), 'Wrong length'
    assert msg[1] & 0xF0 == 0x10, 'upper nibble in sequence byte is wrong'
    assert msg[-1] == 0x7e, 'sync byte is wrong'
    if msg[1] & 0x0F != sequence_id & 0x0f:
        raise ValueError('Wrong sequence id (means NAK response)')
    return msg[2:-3]


def vlq_pack(value: int) -> bytes:
    vlq_bstr = (value & 0x7F).to_bytes(1, 'little')
    bits = ((value + 0x20).bit_length() - 1) if value >= 0 else ((value + 1).bit_length() + 1)
    for i in range(bits // 7):
        value = value >> 7
        vlq_bstr = (0x80 + (value & 0x7F)).to_bytes(1, 'little') + vlq_bstr
    return vlq_bstr


def vlq_unpack(data: bytes) -> Tuple[int, bytes]:
    pos, value = 0, 0
    for pos in range(len(data)):
        value = (value << 7) + (data[pos] & 0x7F)
        if not data[pos] & 0x80:
            break
    else:
        raise ValueError('last byte has MSB set')

    if data[0] & 0x60 == 0x60:
        value = value - (0x80 << (pos * 7))

    return value, data[pos+1:]


class Device:
    def __init__(self, serial: Serial):
        self._serial = serial
        self._sequence_id = 0
    
    def receive(self):
        msg = self._serial.read(1)
        return msg + self._serial.read(msg[0] - 1)
    
    def identify(self):
        address = 0
        data = b''
        while True:
            self._serial.reset_input_buffer()
            req_msg = msg_build(b'\x01' + vlq_pack(address) + b'\x28', self._sequence_id)
            self._sequence_id += 1
            address += 0x28
            self._serial.write(req_msg)
            msg_resp = msg_parse(self.receive(), self._sequence_id)
            data += msg_resp[len(req_msg) - 5:]
            if len(msg_resp) + 5 - len(req_msg) != 0x28:
                break
        data = zlib.decompress(data)
        return json.loads(data)


if __name__ == '__main__':
    s = Serial('/dev/ttyACM0')
    d = Device(s)
    print(d.identify())