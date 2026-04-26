from bafang.protocol import BafangUART
import threading
import time


class ChunkedSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.writes = []
        self.is_open = True
        self.lock = threading.Lock()

    @property
    def in_waiting(self):
        return 1

    def write(self, data):
        self.writes.append(bytes(data))

    def flush(self):
        pass

    def read(self, size):
        time.sleep(0.01)
        with self.lock:
            if not self.chunks:
                return b''
            return self.chunks.pop(0)

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


def _uart_with_response(response_bytes: bytes) -> BafangUART:
    uart = BafangUART(port="COM_TEST")
    uart._send_with_retry = lambda cmd, expected_response_id: response_bytes
    return uart


def test_read_basic_without_optional_temp_sensor_byte():
    payload = bytes(range(25))
    response = bytes([0x52, 0x00]) + payload
    uart = _uart_with_response(response)

    data = uart.read_basic(use_cache=False)

    assert data is not None
    assert data.temp_sensor_type_code is None
    assert data.temp_sensor_type == "Unavailable"
    assert len(data.raw_bytes) == 25


def test_read_basic_with_temp_sensor_byte_present():
    payload = bytes(list(range(25)) + [2])
    response = bytes([0x52, 0x00]) + payload
    uart = _uart_with_response(response)

    data = uart.read_basic(use_cache=False)

    assert data is not None
    assert data.temp_sensor_type_code == 2
    assert data.temp_sensor_type == "Motor only"
    assert len(data.raw_bytes) == 26


def test_read_basic_incomplete_native_payload_falls_back_without_index_error():
    payload = bytes([
        10,
        16,
        25,
        4,
        0,
        0,
        10,
        20,
        30,
        40,
        50,
        60,
        70,
        80,
        90,
        101,
        100,
        1,
        20,
        10,
        20,
        15,
        1,
        11,
    ])
    response = bytes([0x52, 0x00]) + payload
    uart = _uart_with_response(response)

    data = uart.read_basic(use_cache=False)

    assert data is not None
    assert data.protocol_variant == "fallback"
    assert data.temp_sensor_type_code is None
    assert data.throttle_start_voltage == 1100
    assert data.throttle_end_voltage == 4200
    assert uart._section_meta["basic"]["safe_to_write"] is False


def test_read_throttle_with_optional_start_percent():
    payload = bytes([11, 42, 7, 0x23, 0x01, 88])
    response = bytes([0x54, 0x00]) + payload
    uart = _uart_with_response(response)

    data = uart.read_throttle(use_cache=False)

    assert data is not None
    assert data.start_voltage == 1100
    assert data.end_voltage == 4200
    assert data.enabled is True
    assert data.start_percent == 88


def test_read_live_data_uses_framed_payload_without_checksum():
    payload = bytes([
        0x00, 0x7B,
        0x00, 0x00,
        0x04, 0xD2,
        0x01, 0xE0,
        0x00, 0x64,
        0x00, 0x32,
        25,
        250,
        0x01, 0x2C,
        80,
        3,
    ])
    frame = bytes([0x19, len(payload)]) + payload
    response = frame + bytes([sum(frame) & 0xFF])
    uart = BafangUART(port="COM_TEST")
    uart._send_command = lambda cmd: response

    data = uart.read_live_data()

    assert data is not None
    assert data.wheel_speed == 123
    assert data.motor_rpm == 1234
    assert data.battery_voltage == 48.0
    assert data.battery_current == 10.0
    assert data.motor_current == 5.0
    assert data.controller_temp == 25
    assert data.motor_temp == -6
    assert data.torque_sensor == 300
    assert data.cadence == 80
    assert data.assistant_level == 3


def test_read_errors_uses_framed_payload_without_checksum():
    payload = bytes([0x21, 2, 3])
    frame = bytes([0x15, len(payload)]) + payload
    response = frame + bytes([sum(frame) & 0xFF])
    uart = BafangUART(port="COM_TEST")
    sent = []
    uart._send_command = lambda cmd: sent.append(cmd) or response

    data = uart.read_errors()

    assert data is not None
    assert sent == [bytes([0x14, 0x15])]
    assert data.error_code == 0x21
    assert data.system_status == "Rychlostní sensor (E21)"
    assert data.controller_fw == 2
    assert data.motor_fw == 3


def test_extract_frame_skips_noise_and_validates_checksum():
    payload = bytes([41, 12, 0, 23])
    frame = bytes([0x52, len(payload)]) + payload
    response = frame + bytes([sum(frame) & 0xFF])
    uart = BafangUART(port="COM_TEST")

    assert uart._extract_frame(bytes([0x99, 0x88]) + response) == response


def test_event_reader_reassembles_chunks_and_waits_for_matching_frame():
    wrong_payload = bytes([1])
    wrong_frame = bytes([0x53, len(wrong_payload)]) + wrong_payload
    wrong_frame += bytes([sum(wrong_frame) & 0xFF])
    payload = bytes([41, 12, 0, 23])
    frame = bytes([0x52, len(payload)]) + payload
    response = frame + bytes([sum(frame) & 0xFF])
    serial = ChunkedSerial([wrong_frame[:2], wrong_frame[2:], response[:2], response[2:]])
    uart = BafangUART(port="COM_TEST", serial_transport=serial)
    uart._start_reader()

    try:
        assert uart._send_command(bytes([0x11, 0x52]), timeout=1.0) == response
        assert serial.writes == [bytes([0x11, 0x52])]
    finally:
        uart.disconnect()
