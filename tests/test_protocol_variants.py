from bafang.protocol import BafangUART


def _uart_for(cmd_to_response):
    uart = BafangUART(port="COM_TEST")

    def _send_with_retry(cmd, expected_response_id, max_retries=2):
        code = cmd[1] if len(cmd) > 1 else None
        return cmd_to_response.get(code)

    uart._send_with_retry = _send_with_retry
    return uart


def test_read_all_reports_section_meta_and_warnings_on_fallback():
    uart = _uart_for({
        0x52: bytes([0x52, 0x00, 28, 16, 25]),
        0x53: bytes([0x53, 0x00, 3, 0xFF, 0xFF]),
        0x54: bytes([0x54, 0x00, 11, 42]),
    })

    data = uart.read_all_known_params()

    assert data["basic"] is not None
    assert data["pedal"] is not None
    assert data["throttle"] is not None
    assert data["section_meta"]["basic"]["safe_to_write"] is False
    assert data["section_meta"]["pedal"]["safe_to_write"] is False
    assert data["section_meta"]["throttle"]["safe_to_write"] is False
    assert "basic" in data["read_warnings"]
    assert "pedal" in data["read_warnings"]
    assert "throttle" in data["read_warnings"]


def test_read_bafang_variants_detected():
    basic_payload = bytes([
        41,
        18,
        0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
        100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
        56,
        0x41,
    ])
    pedal_payload = bytes([3, 0xFF, 0xFF, 20, 5, 6, 10, 25, 4, 0, 30])
    throttle_payload = bytes([11, 42, 1, 0xFF, 32, 10])

    uart = _uart_for({
        0x52: bytes([0x52, 0x00]) + basic_payload,
        0x53: bytes([0x53, 0x00]) + pedal_payload,
        0x54: bytes([0x54, 0x00]) + throttle_payload,
    })

    data = uart.read_all_known_params()

    assert data["basic"].protocol_variant == "bafang"
    assert data["pedal"].protocol_variant == "bafang"
    assert data["throttle"].protocol_variant == "bafang"
    assert data["section_meta"]["basic"]["safe_to_write"] is True
    assert data["section_meta"]["pedal"]["safe_to_write"] is True
    assert data["section_meta"]["throttle"]["safe_to_write"] is True
    assert data["pedal"].work_mode == 10
    assert data["pedal"].pedal_time_to_stop == 250
    assert data["throttle"].throttle_assist_level == 0xFF


def test_bafang_write_packets_match_reference_checksum():
    uart = BafangUART(port="COM_TEST")
    uart._section_meta = {
        "basic": {"variant": "bafang", "safe_to_write": True},
        "pedal": {"variant": "bafang", "safe_to_write": True},
        "throttle": {"variant": "bafang", "safe_to_write": True},
    }

    basic_cmd = uart._build_basic_command({
        "low_battery_voltage": 41,
        "max_current": 18,
        "wheel_size_code": 4,
        "speedometer_type_code": 1,
        "speedometer_signals": 1,
        **{f"assist_current_{i}": i * 10 for i in range(10)},
        **{f"assist_speed_{i}": 100 for i in range(10)},
    })
    pedal_cmd = uart._build_pedal_command({
        "pedal_type": "DoubleSignal-24",
        "designated_assist": 0xFF,
        "speed_limit": 0xFF,
        "pedal_start_current": 20,
        "pedal_slow_start_mode": 5,
        "pedal_signals_before_start": 6,
        "work_mode": 10,
        "pedal_time_to_stop": 250,
        "pedal_current_decay": 4,
        "pedal_stop_decay": 0,
        "pedal_keep_current": 30,
    })
    throttle_cmd = uart._build_throttle_command({
        "start_voltage": 1100,
        "end_voltage": 4200,
        "throttle_mode": 1,
        "throttle_assist_level": 0xFF,
        "throttle_speed_limit": 32,
        "throttle_start_current": 10,
    })

    assert basic_cmd[-1] == sum(basic_cmd[1:-1]) & 0xFF
    assert pedal_cmd[-1] == sum(pedal_cmd[1:-1]) & 0xFF
    assert throttle_cmd[-1] == sum(throttle_cmd[1:-1]) & 0xFF
    assert basic_cmd[:3] == bytes([0x16, 0x52, 0x18])
    assert pedal_cmd[:3] == bytes([0x16, 0x53, 0x0B])
    assert throttle_cmd[:3] == bytes([0x16, 0x54, 0x06])


def test_bafang_connect_packet_parses_ascii_info():
    uart = BafangUART(port="COM_TEST")
    payload = bytes([*b"SZBF", *b"SW06", ord("2"), ord("2"), 0, 0, 0, 0, 2, 18])
    response_without_checksum = bytes([0x51, len(payload)]) + payload
    response = response_without_checksum + bytes([sum(response_without_checksum) & 0xFF])
    uart._send_command = lambda cmd, timeout=0.3: response

    assert uart._connect_cmd() is True
    assert uart.device_info["manufacturer"] == "SZBF"
    assert uart.device_info["model"] == "SW06"
    assert uart.device_info["hw_version"] == "2.2"
    assert uart.device_info["voltage"] == "48V"
    assert uart.device_info["max_current"] == 18
