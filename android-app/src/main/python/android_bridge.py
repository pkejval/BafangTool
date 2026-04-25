import json
from dataclasses import asdict, is_dataclass

from protocol import BafangUART


class AndroidSerialTransport:
    def __init__(self, java_port):
        self.java_port = java_port
        self.is_open = True

    def write(self, data):
        self.java_port.writeBytes(bytes(data))

    def flush(self):
        pass

    def read(self, size):
        data = self.java_port.readBytes(size)
        return bytes((int(value) & 0xFF for value in data))

    def close(self):
        self.is_open = False
        self.java_port.closePort()


def _plain(value):
    if is_dataclass(value):
        return {key: _plain(val) for key, val in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): _plain(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, bytes):
        return list(value)
    return value


def _json(value):
    return json.dumps(_plain(value), ensure_ascii=False, indent=2)


class AndroidBafangController:
    def __init__(self, java_port):
        self.transport = AndroidSerialTransport(java_port)
        self.controller = BafangUART("android-usb-serial", self.transport)

    def connect(self):
        return self.controller.connect()

    def disconnect(self):
        self.controller.disconnect()

    def status_json(self):
        return _json({
            "connected": self.controller.connected,
            "port": self.controller.port,
            "device_info": self.controller.device_info,
        })

    def read_all_json(self):
        return _json(self.controller.read_all_known_params())

    def read_basic_json(self):
        return _json(self.controller.read_basic(use_cache=False))

    def read_pedal_json(self):
        return _json(self.controller.read_pedal(use_cache=False))

    def read_throttle_json(self):
        return _json(self.controller.read_throttle(use_cache=False))

    def write_all_json(self, basic_json, pedal_json, throttle_json):
        return _json(self.controller.write_all(_loads(basic_json), _loads(pedal_json), _loads(throttle_json)))

    def write_basic_json(self, params_json):
        return _json(self.controller.write_basic(_loads(params_json)))

    def write_pedal_json(self, params_json):
        return _json(self.controller.write_pedal(_loads(params_json)))

    def write_throttle_json(self, params_json):
        return _json(self.controller.write_throttle(_loads(params_json)))

    def live_data_json(self):
        return _json(self.controller.read_live_data() or {})

    def errors_json(self):
        return _json(self.controller.read_errors() or {})

    def config_version_json(self):
        return _json(self.controller.read_config_version() or {})

    def read_experimental_json(self):
        return _json(self.controller.read_experimental())

    def read_raw_basic_json(self):
        return _json(self.controller.read_raw_basic() or {})

    def read_raw_pedal_json(self):
        return _json(self.controller.read_raw_pedal() or {})

    def read_raw_throttle_json(self):
        return _json(self.controller.read_raw_throttle() or {})

    def send_raw_command_json(self, command_hex):
        return _json(self.controller.send_raw_command(command_hex) or {"error": "No response"})

    def write_custom_raw_json(self, hex_data):
        return _json(self.controller.write_custom_raw(hex_data) or {"error": "No response"})

    def scan_commands_json(self):
        return _json(self.controller.scan_all_commands())

    def torque_calibration_json(self):
        return _json({"success": self.controller.torque_calibration()})

    def reset_to_defaults_json(self):
        return _json({"success": self.controller.reset_to_defaults()})

    def set_wheel_circumference_json(self, circumference):
        return _json({"success": self.controller.set_wheel_circumference(int(circumference))})


def _loads(value):
    if not value or not str(value).strip():
        return {}
    return json.loads(value)
