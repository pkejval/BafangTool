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


def test_read_openbafang_variants_detected():
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

    assert data["basic"]["protocol_variant"] == "openbafang"
    assert data["pedal"]["protocol_variant"] == "openbafang"
    assert data["throttle"]["protocol_variant"] == "openbafang"
    assert data["section_meta"]["basic"]["safe_to_write"] is True
    assert data["section_meta"]["pedal"]["safe_to_write"] is True
    assert data["section_meta"]["throttle"]["safe_to_write"] is True
