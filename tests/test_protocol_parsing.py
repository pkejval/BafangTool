from bafang.protocol import BafangUART


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
    assert data["temp_sensor_type_code"] is None
    assert data["temp_sensor_type"] == "Unavailable"
    assert len(data["raw_bytes"]) == 25


def test_read_basic_with_temp_sensor_byte_present():
    payload = bytes(list(range(25)) + [2])
    response = bytes([0x52, 0x00]) + payload
    uart = _uart_with_response(response)

    data = uart.read_basic(use_cache=False)

    assert data is not None
    assert data["temp_sensor_type_code"] == 2
    assert data["temp_sensor_type"] == "Motor only"
    assert len(data["raw_bytes"]) == 26


def test_read_throttle_with_optional_start_percent():
    payload = bytes([11, 42, 7, 0x23, 0x01, 88])
    response = bytes([0x54, 0x00]) + payload
    uart = _uart_with_response(response)

    data = uart.read_throttle(use_cache=False)

    assert data is not None
    assert data["start_voltage"] == 1100
    assert data["end_voltage"] == 4200
    assert data["enabled"] is True
    assert data["start_percent"] == 88
