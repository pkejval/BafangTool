try:
    import serial
except ImportError:
    serial = None
import time
import logging
from typing import Optional, Dict, Any, Tuple, List
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

BAUDRATE = 1200
DATA_BITS = 8
PARITY = 'N'
STOP_BITS = 1

@dataclass
class CommandDef:
    """Definition of a UART command"""
    read_code: int
    write_code: int = 0x16
    response_id: int = 0
    timeout: float = 0.15

class BafangUART:
    COMMANDS = {
        'connect': CommandDef(0x51, 0x11, 0x51),
        'basic': CommandDef(0x52, 0x52, 0x52),
        'pedal': CommandDef(0x53, 0x53, 0x53),
        'throttle': CommandDef(0x54, 0x54, 0x54),
        'config_version': CommandDef(0x55, 0x55, 0x55),
        'live_data': CommandDef(0x19, 0x19, 0x19),
        'errors': CommandDef(0x1A, 0x1A, 0x1A),
        'torque_calib': CommandDef(0xA0, 0xA0, 0xA0),
    }
    
    _CACHE_TTL = 2.0

    @dataclass
    class BasicParameters:
        low_battery_voltage: int = 28
        max_current: int = 16
        speed_limit: int = 25
        wheel_size_code: int = 4
        wheel_size: str = "28\""
        wheel_circumference: int = 0
        speedometer_type: str = "Unknown"
        speedometer_type_code: int = 0
        speedometer_signals: int = 0
        assist_levels: List[Dict[str, Any]] = field(default_factory=lambda: [{'level': i, 'current_percent': 100, 'speed_percent': 100} for i in range(10)])
        assist_level: int = 1
        start_current: int = 20
        start_current_decay: int = 10
        stop_delay: int = 20
        current_ramp: int = 15
        throttle_enabled: bool = True
        throttle_start_voltage: int = 1100
        throttle_end_voltage: int = 4200
        temp_sensor_type_code: Optional[int] = None
        temp_sensor_supported: bool = False
        temp_sensor_type: str = "Unavailable"
        raw_bytes: List[int] = field(default_factory=list)
        protocol_variant: str = "native"
        parse_warning: Optional[str] = None

    @dataclass
    class PedalParameters:
        pedal_type: str = "DoubleSignal-24"
        designated_assist: int = 0xFF
        speed_limit: int = 0xFF
        circumference: int = 0
        signal_number: int = 6
        start_pulse: int = 0x14
        torque_gain: int = 0x0A
        torque_offset: int = 0x19
        torque_step: int = 0x08
        cadence_gain: int = 0x14
        cadence_min: int = 0x14
        cadence_max: int = 0x14
        pedal_start_current: int = 0x14
        pedal_slow_start_mode: int = 0x0A
        pedal_signals_before_start: int = 6
        pedal_time_to_stop: int = 250
        pedal_current_decay: int = 0x08
        pedal_stop_decay: int = 0
        pedal_keep_current: int = 0x14
        work_mode: int = 0x0A
        raw_bytes: List[int] = field(default_factory=list)
        protocol_variant: str = "native"
        parse_warning: Optional[str] = None

    @dataclass
    class ThrottleParameters:
        start_voltage: int = 1100
        end_voltage: int = 4200
        start_current: int = 0x0B
        mode: int = 0x23
        enabled: bool = True
        assist_level: int = 0xFF
        speed_limit: int = 0xFF
        start_percent: Optional[int] = None
        throttle_mode: int = 0
        throttle_assist_level: int = 0xFF
        throttle_speed_limit: int = 0xFF
        throttle_start_current: int = 0x0B
        raw_bytes: List[int] = field(default_factory=list)
        protocol_variant: str = "native"
        parse_warning: Optional[str] = None

    @dataclass
    class LiveData:
        wheel_speed: int = 0
        motor_rpm: int = 0
        battery_voltage: float = 0.0
        battery_current: float = 0.0
        motor_current: float = 0.0
        controller_temp: int = 0
        motor_temp: int = 0
        torque_sensor: int = 0
        cadence: int = 0
        assistant_level: int = 0
        raw_bytes: List[int] = field(default_factory=list)

    @dataclass
    class Errors:
        system_status: str = "Neznámý"
        error_code: int = 0
        controller_fw: int = 0
        motor_fw: int = 0
        raw_bytes: List[int] = field(default_factory=list)

    def __init__(self, port: str, serial_transport=None):
        self.port = port
        self.serial = serial_transport
        self._external_serial = serial_transport is not None
        self.connected = False
        self.device_info: Dict[str, Any] = {}
        self._cache: Dict[str, Tuple[float, Any]] = {}
        self._last_read: Dict[str, bytes] = {}
        self._initial_snapshot: Dict[str, Dict[str, Any]] = {}
        self._allowed_keys: Dict[str, set] = {}
        self._section_meta: Dict[str, Dict[str, Any]] = {}

    def connect(self) -> bool:
        try:
            if self.serial is None:
                if serial is None:
                    raise RuntimeError('pyserial is not available and no serial transport was provided')
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=BAUDRATE,
                    bytesize=DATA_BITS,
                    parity=PARITY,
                    stopbits=STOP_BITS,
                    timeout=2.0
                )
            time.sleep(0.3)
            
            if self._connect_cmd():
                self.connected = True
                logger.info(f"Connected to {self.port}")
                return True
            return False
        except Exception as e:
            logger.error(f"Connection error: {e}")
            return False

    def disconnect(self):
        if self.serial and self._serial_is_open():
            self.serial.close()
        if self._external_serial:
            self.serial = None
        self.connected = False
        self._cache.clear()
        self._last_read.clear()
        self._initial_snapshot.clear()
        self._allowed_keys.clear()
        self._section_meta.clear()

    def _calculate_checksum(self, data: bytes) -> int:
        return sum(data) % 256

    def _calculate_bafang_write_checksum(self, data: bytes) -> int:
        # Bafang UART write checksum is computed from code + length + payload,
        # excluding the leading write marker 0x16.
        return sum(data[1:]) % 256

    def _send_command(self, cmd: bytes, wait_response: bool = True, timeout: float = 0.15) -> Optional[bytes]:
        if not self.serial or not self._serial_is_open():
            return None
        
        try:
            self.serial.write(cmd)
            self.serial.flush()
            
            if not wait_response:
                return b'\x01'
            
            time.sleep(timeout)
            response = self.serial.read(2048)
            return response if response else None
        except Exception as e:
            logger.error(f"Send command error: {e}")
            return None

    def _serial_is_open(self) -> bool:
        is_open = getattr(self.serial, 'is_open', None)
        if callable(is_open):
            return bool(is_open())
        if is_open is None:
            return True
        return bool(is_open)

    def _send_with_retry(self, cmd: bytes, expected_response_id: int, max_retries: int = 2) -> Optional[bytes]:
        """Send command with retry on failure"""
        for attempt in range(max_retries):
            response = self._send_command(cmd)
            if response and len(response) > 0 and response[0] == expected_response_id:
                return response
            if attempt < max_retries - 1:
                time.sleep(0.1)
        return None

    def _connect_cmd(self) -> bool:
        cmd = bytes([0x11, 0x51, 0x04, 0xB0, 0x05])
        response = self._send_command(cmd, timeout=0.3)
        
        if not response or response[0] != 0x51 or len(response) < 19:
            return False
            
        payload = self._payload(response)
        self.device_info = {
            'manufacturer': bytes(payload[0:4]).decode('ascii', errors='ignore'),
            'model': bytes(payload[4:8]).decode('ascii', errors='ignore'),
            'hw_version': f"{chr(payload[8])}.{chr(payload[9])}" if len(payload) > 9 else '',
            'fw_version': '',
            'voltage': {0: "24V", 1: "36V", 2: "48V", 3: "43V", 4: "24V-48V"}.get(payload[14] if len(payload) > 14 else None, "Unknown"),
            'max_current': payload[15] if len(payload) > 15 else None,
            'raw_connect_response': response.hex()
        }
        return True

    def _get_cache(self, key: str) -> Optional[Any]:
        if key in self._cache:
            timestamp, data = self._cache[key]
            if time.time() - timestamp < self._CACHE_TTL:
                return data
        return None

    def _set_cache(self, key: str, data: Any):
        self._cache[key] = (time.time(), data)

    def _invalidate_cache(self, *keys: str):
        for key in keys:
            self._cache.pop(key, None)

    def _capture_initial(self, section: str, values: Dict[str, Any]):
        if section not in self._initial_snapshot:
            self._initial_snapshot[section] = dict(values)
            self._allowed_keys[section] = set(values.keys())

    def _set_section_meta(self, section: str, variant: str, warning: Optional[str] = None):
        self._section_meta[section] = {
            'variant': variant,
            'warning': warning,
            'safe_to_write': variant not in ('fallback',),
        }

    def _section_write_guard(self, section: str) -> Optional[Dict[str, Any]]:
        meta = self._section_meta.get(section, {})
        if not meta:
            return {
                'success': False,
                'error': 'Missing protocol metadata. Please read controller first.',
                'code': None,
                'exception_type': 'ProtocolStateError',
            }
        if not bool(meta.get('safe_to_write', False)):
            return {
                'success': False,
                'error': f"Unsafe write blocked for {section} (variant: {meta.get('variant', 'unknown')}).",
                'code': None,
                'exception_type': 'UnsafeVariantError',
                'protocol_variant': meta.get('variant'),
                'warning': meta.get('warning'),
            }
        return None

    def _clamp(self, value: Any, lower: int, upper: int, default: int) -> int:
        try:
            ivalue = int(value)
        except Exception:
            ivalue = default
        if ivalue < lower:
            return lower
        if ivalue > upper:
            return upper
        return ivalue

    def _debug_hex_for_section(self, section: str) -> Optional[str]:
        response = self._last_read.get(section)
        if response is None:
            return None
        return response.hex()

    def _log_parse_issue(self, section: str, message: str):
        logger.warning(
            "Protocol parse issue section=%s message=%s variant=%s device=%s raw=%s",
            section,
            message,
            self._section_meta.get(section, {}).get('variant', 'unknown'),
            {
                'manufacturer': self.device_info.get('manufacturer'),
                'model': self.device_info.get('model'),
                'fw_version': self.device_info.get('fw_version'),
                'hw_version': self.device_info.get('hw_version'),
            },
            self._debug_hex_for_section(section),
        )

    def _changed_only(self, section: str, incoming: Dict[str, Any]) -> Dict[str, Any]:
        if section not in self._initial_snapshot:
            return {}
        base = self._initial_snapshot[section]
        allowed = self._allowed_keys.get(section, set())
        changed: Dict[str, Any] = {}
        for k, v in incoming.items():
            if k in allowed and k in base and base[k] != v:
                changed[k] = v
        return changed

    def _basic_to_writable(self, data: 'BafangUART.BasicParameters') -> Dict[str, Any]:
        out: Dict[str, Any] = {
            'low_battery_voltage': data.low_battery_voltage,
            'max_current': data.max_current,
            'speed_limit': data.speed_limit,
            'wheel_size_code': data.wheel_size_code,
            'wheel_circumference': data.wheel_circumference,
            'speedometer_type_code': data.speedometer_type_code,
            'speedometer_signals': data.speedometer_signals,
            'assist_level': data.assist_level,
            'start_current': data.start_current,
            'start_current_decay': data.start_current_decay,
            'stop_delay': data.stop_delay,
            'current_ramp': data.current_ramp,
            'throttle_enabled': data.throttle_enabled,
            'throttle_start_voltage': data.throttle_start_voltage,
            'throttle_end_voltage': data.throttle_end_voltage,
            'temp_sensor_type': data.temp_sensor_type_code if data.temp_sensor_type_code is not None else 3,
        }
        levels = data.assist_levels or []
        for i in range(10):
            if i < len(levels):
                out[f'assist_current_{i}'] = levels[i]['current_percent']
                out[f'assist_speed_{i}'] = levels[i]['speed_percent']
            else:
                out[f'assist_current_{i}'] = 100
                out[f'assist_speed_{i}'] = 100
        return out

    def _pedal_to_writable(self, data: 'BafangUART.PedalParameters') -> Dict[str, Any]:
        writable = {
            'pedal_type': data.pedal_type,
            'designated_assist': data.designated_assist,
            'speed_limit': data.speed_limit,
            'circumference': data.circumference,
            'signal_number': data.signal_number,
            'start_pulse': data.start_pulse,
            'torque_gain': data.torque_gain,
            'torque_offset': data.torque_offset,
            'torque_step': data.torque_step,
            'cadence_gain': data.cadence_gain,
            'cadence_min': data.cadence_min,
            'cadence_max': data.cadence_max,
            'pedal_start_current': data.pedal_start_current,
            'pedal_slow_start_mode': data.pedal_slow_start_mode,
            'pedal_signals_before_start': data.pedal_signals_before_start,
            'pedal_time_to_stop': data.pedal_time_to_stop,
            'pedal_current_decay': data.pedal_current_decay,
            'pedal_stop_decay': data.pedal_stop_decay,
            'pedal_keep_current': data.pedal_keep_current,
            'work_mode': data.work_mode,
            'protocol_variant': data.protocol_variant,
        }
        return writable

    def _throttle_to_writable(self, data: 'BafangUART.ThrottleParameters') -> Dict[str, Any]:
        writable = {
            'start_voltage': data.start_voltage,
            'end_voltage': data.end_voltage,
            'start_current': data.start_current,
            'mode': data.mode,
            'enabled': 1 if data.enabled else 0,
            'assist_level': data.assist_level,
            'speed_limit': data.speed_limit,
            'throttle_mode': data.throttle_mode,
            'throttle_assist_level': data.throttle_assist_level,
            'throttle_speed_limit': data.throttle_speed_limit,
            'throttle_start_current': data.throttle_start_current,
        }
        return writable

    def _pick(self, values: list, idx: int, default: Any = "Unknown") -> Any:
        if 0 <= idx < len(values):
            return values[idx]
        return default

    def _get_byte(self, data: bytes, idx: int, default: Any = None) -> Any:
        if 0 <= idx < len(data):
            return data[idx]
        return default

    def _payload(self, response: bytes) -> bytes:
        if len(response) >= 3 and response[1] == len(response) - 3:
            return response[2:-1]
        return response[2:]

    def _looks_like_bafang_basic(self, payload: bytes) -> bool:
        if len(payload) < 24:
            return False
        if len(payload) > 24 and payload[24] > 5:
            return False
        speedmeter_raw = payload[23]
        speedmeter_type = (speedmeter_raw & 0xC0) >> 6
        magnets = speedmeter_raw & 0x3F
        if speedmeter_type > 2:
            return False
        if magnets < 1 or magnets > 32:
            return False
        for i in range(10):
            if payload[2 + i] > 100 or payload[12 + i] > 100:
                return False
        return True

    def _wheel_size_code_from_diameter(self, wheel_diameter: float) -> int:
        mapping = {
            12.0: 9,
            16.0: 6,
            20.0: 7,
            24.0: 8,
            26.0: 1,
            27.5: 3,
            28.0: 4,
            29.0: 5,
        }
        return mapping.get(round(wheel_diameter * 2) / 2, 4)

    def _wheel_diameter_from_code(self, wheel_size_code: int) -> float:
        mapping = {
            0: 28.0,
            1: 26.0,
            2: 26.0,
            3: 27.5,
            4: 28.0,
            5: 29.0,
            6: 16.0,
            7: 20.0,
            8: 24.0,
            9: 12.0,
        }
        return mapping.get(self._clamp(wheel_size_code, 0, 9, 4), 28.0)

    def send_raw_command(self, command_hex: str) -> Optional[Dict[str, Any]]:
        try:
            cmd_bytes = bytes.fromhex(command_hex.replace(' ', ''))
            response = self._send_command(cmd_bytes)
            if response:
                return {
                    'raw_hex': response.hex(),
                    'raw_bytes': list(response),
                    'length': len(response),
                    'first_byte': response[0] if len(response) > 0 else None,
                    'data': [response[i] for i in range(min(len(response), 64))]
                }
            return None
        except Exception as e:
            logger.error(f"Raw command error: {e}")
            return None

    def read_all_known_params(self) -> Dict[str, Any]:
        cached = self._get_cache('all_params')
        if cached:
            return cached

        read_errors: Dict[str, Dict[str, str]] = {}
        read_warnings: Dict[str, str] = {}

        def safe_read(name: str, reader, required: bool = True):
            try:
                parsed = reader()
                if parsed is None:
                    if required:
                        read_errors[name] = {
                            'message': 'No response or unsupported payload format',
                            'exception_type': 'ParseError',
                        }
                        self._log_parse_issue(name, read_errors[name]['message'])
                elif hasattr(parsed, 'parse_warning') and parsed.parse_warning:
                    read_warnings[name] = str(parsed.parse_warning)
                    self._log_parse_issue(name, read_warnings[name])
                return parsed
            except Exception as e:
                logger.error(f"Failed to read {name}: {e}")
                read_errors[name] = {
                    'message': str(e),
                    'exception_type': e.__class__.__name__,
                }
                self._log_parse_issue(name, read_errors[name]['message'])
                return None
            
        result = {
            'device_info': self.device_info,
            'basic': safe_read('basic', self.read_basic),
            'pedal': safe_read('pedal', self.read_pedal),
            'throttle': safe_read('throttle', self.read_throttle),
            'live_data': safe_read('live_data', self.read_live_data, required=False),
            'errors': safe_read('errors', self.read_errors, required=False),
            'read_errors': read_errors,
            'read_warnings': read_warnings,
            'section_meta': self._section_meta,
        }
        self._set_cache('all_params', result)
        return result

    def read_basic(self, use_cache: bool = True) -> Optional['BafangUART.BasicParameters']:
        if use_cache:
            cached = self._get_cache('basic')
            if cached:
                return cached
                
        cmd = bytes([0x11, 0x52])
        response = self._send_with_retry(cmd, 0x52)
        
        if not response:
            return None
        self._last_read['basic'] = response
        
        payload = self._payload(response)
        if len(payload) < 24:
            if len(payload) < 2:
                return None
            fallback_levels = [
                {
                    'level': i,
                    'current_percent': 100,
                    'speed_percent': 100,
                }
                for i in range(10)
            ]
            result = self.BasicParameters(
                low_battery_voltage=self._get_byte(payload, 0, 28),
                max_current=self._get_byte(payload, 1, 16),
                speed_limit=self._get_byte(payload, 2, 25),
                wheel_size_code=4,
                wheel_size='28"',
                wheel_circumference=0,
                speedometer_type='Unknown',
                speedometer_signals=0,
                assist_levels=fallback_levels,
                assist_level=1,
                start_current=20,
                start_current_decay=10,
                stop_delay=20,
                current_ramp=15,
                throttle_enabled=True,
                throttle_start_voltage=1100,
                throttle_end_voltage=4200,
                temp_sensor_type_code=None,
                temp_sensor_supported=False,
                temp_sensor_type='Unavailable',
                raw_bytes=list(payload),
                protocol_variant='fallback',
                parse_warning='Short basic payload parsed with defaults',
            )
            self._set_section_meta('basic', 'fallback', result.parse_warning)
            self._set_cache('basic', result)
            self._capture_initial('basic', self._basic_to_writable(result))
            return result

        if self._looks_like_bafang_basic(payload):
            speedmeter_raw = payload[23]
            wheel_diameter = payload[22] / 2
            temp_sensor_type_code = payload[24] if len(payload) > 24 else None
            result = self.BasicParameters(
                low_battery_voltage=payload[0],
                max_current=payload[1],
                speed_limit=25,
                wheel_size_code=self._wheel_size_code_from_diameter(wheel_diameter),
                wheel_size=f'{wheel_diameter:g}"',
                wheel_circumference=0,
                speedometer_type_code=(speedmeter_raw & 0xC0) >> 6,
                speedometer_type=self._pick(["External", "Internal", "Motor phase"], (speedmeter_raw & 0xC0) >> 6),
                speedometer_signals=speedmeter_raw & 0x3F,
                assist_levels=[
                    {
                        'level': i,
                        'current_percent': payload[2 + i],
                        'speed_percent': payload[12 + i],
                    }
                    for i in range(10)
                ],
                assist_level=1,
                start_current=20,
                start_current_decay=10,
                stop_delay=20,
                current_ramp=15,
                throttle_enabled=True,
                throttle_start_voltage=1100,
                throttle_end_voltage=4200,
                temp_sensor_type_code=temp_sensor_type_code,
                temp_sensor_supported=temp_sensor_type_code is not None,
                temp_sensor_type=self._pick(["No sensor", "Controller only", "Motor only", "Both"], temp_sensor_type_code) if temp_sensor_type_code is not None else "Unavailable",
                raw_bytes=list(payload),
                protocol_variant='bafang',
            )
            self._set_section_meta('basic', 'bafang')
            self._set_cache('basic', result)
            self._capture_initial('basic', self._basic_to_writable(result))
            return result

        if len(payload) < 25:
            result = self.BasicParameters(
                low_battery_voltage=self._get_byte(payload, 0, 28),
                max_current=self._get_byte(payload, 1, 16),
                speed_limit=self._get_byte(payload, 2, 25),
                wheel_size_code=self._get_byte(payload, 3, 4),
                wheel_size=self._pick(["700C", "26\"(A)", "26\"(B)", "27.5\"", "28\"", "29\"", "16\"", "20\"", "24\"", "12\""], self._get_byte(payload, 3, 4)),
                wheel_circumference=self._get_byte(payload, 4, 0),
                speedometer_type='Unknown',
                speedometer_signals=0,
                assist_levels=[
                    {
                        'level': i,
                        'current_percent': self._get_byte(payload, i, 100),
                        'speed_percent': self._get_byte(payload, i + 10, 100),
                    }
                    for i in range(10)
                ],
                assist_level=self._get_byte(payload, 17, 1),
                start_current=self._get_byte(payload, 18, 20),
                start_current_decay=self._get_byte(payload, 19, 10),
                stop_delay=self._get_byte(payload, 20, 20),
                current_ramp=self._get_byte(payload, 21, 15),
                throttle_enabled=bool(self._get_byte(payload, 22, 1) & 0x01),
                throttle_start_voltage=self._get_byte(payload, 23, 11) * 100,
                throttle_end_voltage=4200,
                temp_sensor_type_code=None,
                temp_sensor_supported=False,
                temp_sensor_type='Unavailable',
                raw_bytes=list(payload),
                protocol_variant='fallback',
                parse_warning='Incomplete native basic payload parsed with defaults',
            )
            self._set_section_meta('basic', 'fallback', result.parse_warning)
            self._set_cache('basic', result)
            self._capture_initial('basic', self._basic_to_writable(result))
            return result

        data = payload[:25]
        temp_sensor_type_code = payload[25] if len(payload) > 25 else None
        
        result = self.BasicParameters(
            low_battery_voltage=data[0] + 18,
            max_current=data[1],
            speed_limit=data[2],
            wheel_size_code=data[3],
            wheel_size=self._pick(["700C", "26\"(A)", "26\"(B)", "27.5\"", "28\"", "29\"", "16\"", "20\"", "24\"", "12\""], data[3]),
            wheel_circumference=data[4],
            speedometer_type_code=data[5] & 0x03,
            speedometer_type=self._pick(["External", "Internal", "Motor phase"], data[5] & 0x03),
            speedometer_signals=(data[5] >> 2) & 0x0F,
            assist_levels=[{'level': i, 'current_percent': data[i], 'speed_percent': data[i + 10]} for i in range(10)],
            assist_level=data[17],
            start_current=data[18],
            start_current_decay=data[19],
            stop_delay=data[20],
            current_ramp=data[21],
            throttle_enabled=(data[22] & 0x01) == 0x01,
            throttle_start_voltage=data[23] * 100,
            throttle_end_voltage=data[24] * 100,
            temp_sensor_type_code=temp_sensor_type_code,
            temp_sensor_supported=temp_sensor_type_code is not None,
            temp_sensor_type=self._pick(["No sensor", "Controller only", "Motor only", "Both"], temp_sensor_type_code) if temp_sensor_type_code is not None else "Unavailable",
            raw_bytes=list(payload),
            protocol_variant='native'
        )
        self._set_section_meta('basic', 'native')
        self._set_cache('basic', result)
        self._capture_initial('basic', self._basic_to_writable(result))
        return result

    def read_pedal(self, use_cache: bool = True) -> Optional['BafangUART.PedalParameters']:
        if use_cache:
            cached = self._get_cache('pedal')
            if cached:
                return cached
                
        cmd = bytes([0x11, 0x53])
        response = self._send_with_retry(cmd, 0x53)
        
        if not response:
            return None
        self._last_read['pedal'] = response

        data = self._payload(response)
        if len(data) < 11:
            if len(data) < 3:
                return None
            result = self.PedalParameters(
                pedal_type=self._pick(["None", "DH-Sensor-12", "BB-Sensor-32", "DoubleSignal-24"], self._get_byte(data, 0, 3)),
                designated_assist=self._get_byte(data, 1, 0xFF),
                speed_limit=self._get_byte(data, 2, 0xFF),
                signal_number=self._get_byte(data, 5, 6),
                start_pulse=self._get_byte(data, 3, 0x14),
                torque_gain=self._get_byte(data, 4, 0x0A),
                torque_offset=self._get_byte(data, 7, 0x19),
                torque_step=self._get_byte(data, 8, 0x08),
                cadence_gain=self._get_byte(data, 9, 0x14),
                cadence_min=self._get_byte(data, 10, 0x14),
                cadence_max=self._get_byte(data, 10, 0x14),
                pedal_start_current=self._get_byte(data, 3, 0x14),
                pedal_slow_start_mode=self._get_byte(data, 4, 0x0A),
                pedal_signals_before_start=self._get_byte(data, 5, 6),
                pedal_time_to_stop=self._get_byte(data, 7, 0x19),
                pedal_current_decay=self._get_byte(data, 8, 0x08),
                pedal_stop_decay=self._get_byte(data, 9, 0x14),
                pedal_keep_current=self._get_byte(data, 10, 0x14),
                work_mode=self._get_byte(data, 6, 0x0A),
                raw_bytes=list(data),
                protocol_variant='fallback',
                parse_warning='Short pedal payload parsed with defaults',
            )
            self._set_section_meta('pedal', 'fallback', result.parse_warning)
            self._set_cache('pedal', result)
            self._capture_initial('pedal', self._pedal_to_writable(result))
            return result

        if len(data) == 11:
            result = self.PedalParameters(
                pedal_type=self._pick(["None", "DH-Sensor-12", "BB-Sensor-32", "DoubleSignal-24"], data[0]),
                designated_assist=data[1],
                speed_limit=data[2],
                circumference=data[3],
                signal_number=data[5],
                start_pulse=data[3],
                torque_gain=data[4],
                torque_offset=data[7],
                torque_step=data[8],
                cadence_gain=data[9],
                cadence_min=data[10],
                cadence_max=data[10],
                pedal_start_current=data[3],
                pedal_slow_start_mode=data[4],
                pedal_signals_before_start=data[5],
                pedal_time_to_stop=data[7] * 10,
                pedal_current_decay=data[8],
                pedal_stop_decay=data[9] * 10,
                pedal_keep_current=data[10],
                work_mode=data[6],
                raw_bytes=list(data),
                protocol_variant='bafang'
            )
            self._set_section_meta('pedal', 'bafang')
            self._set_cache('pedal', result)
            self._capture_initial('pedal', self._pedal_to_writable(result))
            return result

        data = data[:12]
        result = self.PedalParameters(
            pedal_type=self._pick(["None", "DH-Sensor-12", "BB-Sensor-32", "DoubleSignal-24"], data[0]),
            designated_assist=data[1],
            speed_limit=data[2],
            circumference=data[3],
            signal_number=data[4],
            start_pulse=data[5],
            torque_gain=data[6],
            torque_offset=data[7],
            torque_step=data[8],
            cadence_gain=data[9],
            cadence_min=data[10],
            cadence_max=data[11],
            pedal_start_current=data[5],
            pedal_slow_start_mode=data[6],
            pedal_signals_before_start=data[4],
            pedal_time_to_stop=data[7],
            pedal_current_decay=data[8],
            pedal_stop_decay=data[9],
            pedal_keep_current=data[10],
            work_mode=10,
            raw_bytes=list(data),
            protocol_variant='native'
        )
        self._set_section_meta('pedal', 'native')
        self._set_cache('pedal', result)
        self._capture_initial('pedal', self._pedal_to_writable(result))
        return result

    def read_throttle(self, use_cache: bool = True) -> Optional['BafangUART.ThrottleParameters']:
        if use_cache:
            cached = self._get_cache('throttle')
            if cached:
                return cached
                
        cmd = bytes([0x11, 0x54])
        response = self._send_with_retry(cmd, 0x54)
        
        if not response:
            return None
        self._last_read['throttle'] = response
        
        data = self._payload(response)
        if len(data) < 5:
            if len(data) < 2:
                return None
            result = self.ThrottleParameters(
                start_voltage=self._get_byte(data, 0, 11) * 100,
                end_voltage=self._get_byte(data, 1, 42) * 100,
                start_current=self._get_byte(data, 2, 0x0B),
                mode=self._get_byte(data, 3, 0x23),
                enabled=bool(self._get_byte(data, 4, 1)),
                start_percent=None,
                throttle_mode=self._get_byte(data, 3, 0x23),
                throttle_assist_level=self._get_byte(data, 3, 0x23),
                throttle_speed_limit=self._get_byte(data, 4, 0xFF),
                throttle_start_current=self._get_byte(data, 2, 0x0B),
                assist_level=self._get_byte(data, 3, 0x23),
                speed_limit=self._get_byte(data, 4, 0xFF),
                raw_bytes=list(data),
                protocol_variant='fallback',
                parse_warning='Short throttle payload parsed with defaults',
            )
            self._set_section_meta('throttle', 'fallback', result.parse_warning)
            self._set_cache('throttle', result)
            self._capture_initial('throttle', self._throttle_to_writable(result))
            return result

        if len(data) >= 6 and data[2] in (0, 1) and data[3] in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 255):
            result = self.ThrottleParameters(
                start_voltage=data[0] * 100,
                end_voltage=data[1] * 100,
                start_current=data[5],
                mode=data[2],
                enabled=True,
                assist_level=data[3],
                speed_limit=data[4],
                throttle_mode=data[2],
                throttle_assist_level=data[3],
                throttle_speed_limit=data[4],
                throttle_start_current=data[5],
                start_percent=None,
                raw_bytes=list(data),
                protocol_variant='bafang'
            )
            self._set_section_meta('throttle', 'bafang')
            self._set_cache('throttle', result)
            self._capture_initial('throttle', self._throttle_to_writable(result))
            return result

        start_percent = data[5] if len(data) > 5 else None
        result = self.ThrottleParameters(
            start_voltage=data[0] * 100,
            end_voltage=data[1] * 100,
            start_current=data[2],
            mode=data[3],
            enabled=data[4] == 0x01,
            start_percent=start_percent,
            throttle_mode=data[3],
            throttle_assist_level=data[3],
            throttle_speed_limit=data[4],
            throttle_start_current=data[2],
            assist_level=data[3],
            speed_limit=data[4],
            raw_bytes=list(data),
            protocol_variant='native'
        )
        self._set_section_meta('throttle', 'native')
        self._set_cache('throttle', result)
        self._capture_initial('throttle', self._throttle_to_writable(result))
        return result

    def read_raw_basic(self) -> Optional[Dict[str, Any]]:
        if 'raw_basic' in self._last_read:
            response = self._last_read['raw_basic']
        else:
            cmd = bytes([0x11, 0x52])
            response = self._send_command(cmd)
            if response:
                self._last_read['raw_basic'] = response
        
        if not response:
            return None
        
        return {
            'full_hex': response.hex(),
            'all_bytes': [{'index': i, 'hex': f"0x{b:02X}", 'dec': b, 'bin': format(b, '08b')} for i, b in enumerate(response)],
            'length': len(response),
            'data_section': list(response[2:]) if len(response) > 2 else []
        }

    def read_raw_pedal(self) -> Optional[Dict[str, Any]]:
        if 'raw_pedal' in self._last_read:
            response = self._last_read['raw_pedal']
        else:
            cmd = bytes([0x11, 0x53])
            response = self._send_command(cmd)
            if response:
                self._last_read['raw_pedal'] = response
        
        if not response:
            return None
        
        return {
            'full_hex': response.hex(),
            'all_bytes': [{'index': i, 'hex': f"0x{b:02X}", 'dec': b, 'bin': format(b, '08b')} for i, b in enumerate(response)],
            'length': len(response),
            'data_section': list(response[2:]) if len(response) > 2 else []
        }

    def read_raw_throttle(self) -> Optional[Dict[str, Any]]:
        if 'raw_throttle' in self._last_read:
            response = self._last_read['raw_throttle']
        else:
            cmd = bytes([0x11, 0x54])
            response = self._send_command(cmd)
            if response:
                self._last_read['raw_throttle'] = response
        
        if not response:
            return None
        
        return {
            'full_hex': response.hex(),
            'all_bytes': [{'index': i, 'hex': f"0x{b:02X}", 'dec': b, 'bin': format(b, '08b')} for i, b in enumerate(response)],
            'length': len(response),
            'data_section': list(response[2:]) if len(response) > 2 else []
        }

    def _build_basic_command(self, params: Dict[str, Any]) -> bytes:
        variant = self._section_meta.get('basic', {}).get('variant', 'native')
        if variant == 'fallback':
            raise ValueError('Write blocked in fallback basic variant')
        if variant == 'bafang':
            cmd = bytes([0x16, 0x52, 0x18])
            cmd += bytes([
                self._clamp(params.get('low_battery_voltage', 28), 1, 60, 28),
                self._clamp(params.get('max_current', 16), 1, 100, 16),
            ])
            for i in range(10):
                levels = params.get('assist_levels', [])
                cmd += bytes([self._clamp(levels[i]['current_percent'] if i < len(levels) else params.get(f'assist_current_{i}', 100), 0, 100, 100)])
                cmd += bytes([self._clamp(levels[i]['speed_percent'] if i < len(levels) else params.get(f'assist_speed_{i}', 100), 0, 100, 100)])
            cmd += bytes([
                self._clamp(params.get('wheel_size_code', 4), 0, 255, 4),
                ((self._clamp(params.get('speedometer_type_code', 1), 0, 2, 1) & 0b11) << 6) | (self._clamp(params.get('speedometer_signals', 1), 0, 63, 1) & 0b111111),
            ])
            return cmd + bytes([self._calculate_bafang_write_checksum(cmd)])

        cmd = bytes([0x16, 0x52, 0x24])
        cmd += bytes([
            self._clamp(params.get('low_battery_voltage', 28), 1, 60, 28),
            self._clamp(params.get('max_current', 16), 1, 100, 16),
        ])
        for i in range(10):
            levels = params.get('assist_levels', [])
            cmd += bytes([self._clamp(levels[i]['current_percent'] if i < len(levels) else params.get(f'assist_current_{i}', 100), 0, 100, 100)])
            cmd += bytes([self._clamp(levels[i]['speed_percent'] if i < len(levels) else params.get(f'assist_speed_{i}', 100), 0, 100, 100)])
        cmd += bytes([
            self._clamp(params.get('wheel_size_code', 4), 0, 255, 4),
            ((self._clamp(params.get('speedometer_type_code', 1), 0, 2, 1) & 0b11) << 6) | (self._clamp(params.get('speedometer_signals', 1), 0, 63, 1) & 0b111111),
            self._clamp(params.get('wheel_circumference', 0), 0, 255, 0),
            self._clamp(params.get('start_current', 20), 0, 255, 20),
            self._clamp(params.get('start_current_decay', 10), 0, 255, 10),
            self._clamp(params.get('stop_delay', 20), 0, 255, 20),
            self._clamp(params.get('current_ramp', 15), 0, 255, 15),
            0x01 if params.get('throttle_enabled', True) else 0x00,
            self._clamp(params.get('throttle_start_voltage', 1100) // 100, 0, 255, 11),
            self._clamp(params.get('throttle_end_voltage', 4200) // 100, 0, 255, 42),
            self._clamp(params.get('temp_sensor_type', 3), 0, 3, 3),
            0x00
        ])
        
        return cmd + bytes([self._calculate_checksum(cmd)])

    def _build_pedal_command(self, params: Dict[str, Any]) -> bytes:
        pedal_types = {'None': 0, 'DH-Sensor-12': 1, 'BB-Sensor-32': 2, 'DoubleSignal-24': 3}
        if self._section_meta.get('pedal', {}).get('variant') == 'fallback':
            raise ValueError('Write blocked in fallback pedal variant')

        if self._section_meta.get('pedal', {}).get('variant') == 'bafang':
            time_to_stop = params.get('pedal_time_to_stop', params.get('stop_delay', params.get('torque_offset', 250)))
            stop_decay = params.get('pedal_stop_decay', params.get('stop_decay', params.get('cadence_gain', 0)))
            cmd = bytes([0x16, 0x53, 0x0B])
            cmd += bytes([pedal_types.get(params.get('pedal_type', 'DoubleSignal-24'), 3)])
            cmd += bytes([
                self._clamp(params.get('designated_assist', 0xFF), 0, 255, 0xFF),
                self._clamp(params.get('speed_limit', 0xFF), 0, 255, 0xFF),
                self._clamp(params.get('pedal_start_current', params.get('start_pulse', params.get('start_current', 0x14))), 0, 255, 0x14),
                self._clamp(params.get('pedal_slow_start_mode', params.get('torque_gain', params.get('slow_start_mode', 0x0A))), 0, 255, 0x0A),
                self._clamp(params.get('pedal_signals_before_start', params.get('signal_number', 0x06)), 0, 255, 0x06),
                self._clamp(params.get('work_mode', 0x0A), 0, 255, 0x0A),
                self._clamp(time_to_stop // 10 if time_to_stop > 255 else time_to_stop, 0, 255, 0x19),
                self._clamp(params.get('pedal_current_decay', params.get('torque_step', params.get('current_decay', 0x08))), 0, 255, 0x08),
                self._clamp(stop_decay // 10 if stop_decay > 255 else stop_decay, 0, 255, 0x00),
                self._clamp(params.get('pedal_keep_current', params.get('cadence_min', params.get('keep_current', 0x14))), 0, 100, 0x14),
            ])
            return cmd + bytes([self._calculate_bafang_write_checksum(cmd)])
        
        cmd = bytes([0x16, 0x53, 0x0B])
        cmd += bytes([pedal_types.get(params.get('pedal_type', 'DoubleSignal-24'), 3)])
        cmd += bytes([self._clamp(params.get('designated_assist', 0xFF), 0, 255, 0xFF)])
        cmd += bytes([
            self._clamp(params.get('speed_limit', 0xFF), 0, 255, 0xFF),
            self._clamp(params.get('circumference', 0x00), 0, 255, 0x00),
            self._clamp(params.get('signal_number', 0x06), 0, 255, 0x06),
            self._clamp(params.get('start_pulse', 0x14), 0, 255, 0x14),
            self._clamp(params.get('torque_gain', 0x0A), 0, 255, 0x0A),
            self._clamp(params.get('torque_offset', 0x19), 0, 255, 0x19),
            self._clamp(params.get('torque_step', 0x08), 0, 255, 0x08),
            self._clamp(params.get('cadence_gain', 0x14), 0, 255, 0x14),
            self._clamp(params.get('cadence_min', 0x14), 0, 255, 0x14),
            self._clamp(params.get('cadence_max', 0x14), 0, 255, 0x14)
        ])
        
        return cmd + bytes([self._calculate_checksum(cmd)])

    def _build_throttle_command(self, params: Dict[str, Any]) -> bytes:
        if self._section_meta.get('throttle', {}).get('variant') == 'fallback':
            raise ValueError('Write blocked in fallback throttle variant')
        if self._section_meta.get('throttle', {}).get('variant') == 'bafang':
            cmd = bytes([0x16, 0x54, 0x06])
            cmd += bytes([
                self._clamp(params.get('start_voltage', 1100) // 100, 0, 255, 11),
                self._clamp(params.get('end_voltage', 4200) // 100, 0, 255, 42),
                self._clamp(params.get('throttle_mode', params.get('mode', 0)), 0, 1, 0),
                self._clamp(params.get('throttle_assist_level', params.get('assist_level', params.get('designated_assist', 0xFF))), 0, 255, 0xFF),
                self._clamp(params.get('throttle_speed_limit', params.get('speed_limit', 0xFF)), 0, 255, 0xFF),
                self._clamp(params.get('throttle_start_current', params.get('start_current', 0x0B)), 0, 255, 0x0B),
            ])
            return cmd + bytes([self._calculate_bafang_write_checksum(cmd)])

        cmd = bytes([0x16, 0x54, 0x06])
        cmd += bytes([
            self._clamp(params.get('start_voltage', 1100) // 100, 0, 255, 11),
            self._clamp(params.get('end_voltage', 4200) // 100, 0, 255, 42),
            self._clamp(params.get('start_current', 0x0B), 0, 255, 0x0B),
            self._clamp(params.get('mode', 0x23), 0, 255, 0x23),
            self._clamp(params.get('enabled', 0x00), 0, 1, 0x00)
        ])
        
        return cmd + bytes([self._calculate_checksum(cmd)])

    def write_basic(self, params: Dict[str, Any]) -> Dict[str, Any]:
        if 'basic' not in self._initial_snapshot:
            return {'success': False, 'error': 'Nejprve načtěte controller (Read).', 'code': None}

        guard = self._section_write_guard('basic')
        if guard is not None:
            return guard

        changed = self._changed_only('basic', params)
        if not changed:
            return {'success': True, 'error': None, 'code': None, 'skipped': True, 'message': 'Žádné změněné parametry.'}

        merged = dict(self._initial_snapshot['basic'])
        merged.update(changed)
        cmd = self._build_basic_command(merged)
        response = self._send_command(cmd)
        
        if response is None:
            return {'success': False, 'error': 'No response from controller', 'code': None}
        
        result_code = response[1] if len(response) > 1 else None
        
        variant = self._section_meta.get('basic', {}).get('variant', 'native')
        expected_ok = 0x18 if variant == 'bafang' else 0x24
        if result_code == expected_ok:
            self._invalidate_cache('basic', 'all_params')
            self._initial_snapshot['basic'].update(changed)
            return {'success': True, 'error': None, 'code': result_code}
        
        error_messages = {
            0x00: "Low Battery Protect - chybné nastavení",
            0x01: "Limited Current - chybný proud",
            0x02: "Limit Current Assist0 - chyba v úrovni 0",
            0x03: "Limit Current Assist1 - chyba v úrovni 1",
            0x04: "Limit Current Assist2 - chyba v úrovni 2",
            0x05: "Limit Current Assist3 - chyba v úrovni 3",
            0x06: "Limit Current Assist4 - chyba v úrovni 4",
            0x07: "Limit Current Assist5 - chyba v úrovni 5",
            0x08: "Limit Current Assist6 - chyba v úrovni 6",
            0x09: "Limit Current Assist7 - chyba v úrovni 7",
            0x0A: "Limit Current Assist8 - chyba v úrovni 8",
            0x0B: "Limit Current Assist9 - chyba v úrovni 9",
            0x0C: "Limit Speed Assist0 - chyba rychlosti úrovně 0",
            0x0D: "Limit Speed Assist1 - chyba rychlosti úrovně 1",
            0x0E: "Limit Speed Assist2 - chyba rychlosti úrovně 2",
            0x0F: "Limit Speed Assist3 - chyba rychlosti úrovně 3",
            0x10: "Limit Speed Assist4 - chyba rychlosti úrovně 4",
            0x11: "Limit Speed Assist5 - chyba rychlosti úrovně 5",
            0x12: "Limit Speed Assist6 - chyba rychlosti úrovně 6",
            0x13: "Limit Speed Assist7 - chyba rychlosti úrovně 7",
            0x14: "Limit Speed Assist8 - chyba rychlosti úrovně 8",
            0x15: "Limit Speed Assist9 - chyba rychlosti úrovně 9",
            0x16: "Start Current - chyba startovního proudu",
            0x17: "Start Current Decay - chyba poklesu startovního proudu",
            0x18: "Stop Delay - chyba zpoždění zastavení",
            0x19: "Current Ramp - chyba rampy proudu",
        }
        
        if result_code is not None:
            rc = int(result_code)
            error_msg = error_messages.get(rc, f"Neznámá chyba (kód: 0x{rc:02X})")
        else:
            error_msg = "Neznámá chyba"
        return {'success': False, 'error': error_msg, 'code': result_code}

    def write_pedal(self, params: Dict[str, Any]) -> Dict[str, Any]:
        if 'pedal' not in self._initial_snapshot:
            return {'success': False, 'error': 'Nejprve načtěte controller (Read).', 'code': None}

        guard = self._section_write_guard('pedal')
        if guard is not None:
            return guard

        changed = self._changed_only('pedal', params)
        if not changed:
            return {'success': True, 'error': None, 'code': None, 'skipped': True, 'message': 'Žádné změněné parametry.'}

        merged = dict(self._initial_snapshot['pedal'])
        merged.update(changed)
        cmd = self._build_pedal_command(merged)
        response = self._send_command(cmd)
        
        if response is None:
            return {'success': False, 'error': 'No response from controller', 'code': None}
        
        result_code = response[1] if len(response) > 1 else None
        
        if result_code == 0x0B:
            self._invalidate_cache('pedal', 'all_params')
            self._initial_snapshot['pedal'].update(changed)
            return {'success': True, 'error': None, 'code': result_code}
        
        error_messages = {
            0x00: "Pedal Type - neplatný typ sensoru",
            0x01: "Designated Assist - neplatná úroveň asistence",
            0x02: "Speed Limit - neplatný limit rychlosti",
            0x03: "Circumference - neplatný obvod kola",
            0x04: "Signal Number - neplatný počet signálů",
            0x05: "Start Pulse - neplatný start pulse",
            0x06: "Torque Gain - neplatný torque gain",
            0x07: "Torque Offset - neplatný torque offset",
            0x08: "Torque Step - neplatný torque step",
            0x09: "Cadence Gain - neplatný cadence gain",
            0x0A: "Cadence Min/Max - neplatný cadence limit",
        }
        
        if result_code is not None:
            rc = int(result_code)
            error_msg = error_messages.get(rc, f"Neznámá chyba (kód: 0x{rc:02X})")
        else:
            error_msg = "Neznámá chyba"
        return {'success': False, 'error': error_msg, 'code': result_code}

    def write_throttle(self, params: Dict[str, Any]) -> Dict[str, Any]:
        if 'throttle' not in self._initial_snapshot:
            return {'success': False, 'error': 'Nejprve načtěte controller (Read).', 'code': None}

        guard = self._section_write_guard('throttle')
        if guard is not None:
            return guard

        changed = self._changed_only('throttle', params)
        if not changed:
            return {'success': True, 'error': None, 'code': None, 'skipped': True, 'message': 'Žádné změněné parametry.'}

        merged = dict(self._initial_snapshot['throttle'])
        merged.update(changed)
        cmd = self._build_throttle_command(merged)
        response = self._send_command(cmd)
        
        if response is None:
            return {'success': False, 'error': 'No response from controller', 'code': None}
        
        result_code = response[1] if len(response) > 1 else None
        
        if result_code == 0x06:
            self._invalidate_cache('throttle', 'all_params')
            self._initial_snapshot['throttle'].update(changed)
            return {'success': True, 'error': None, 'code': result_code}
        
        error_messages = {
            0x00: "Start Voltage - neplatné startovní napětí",
            0x01: "End Voltage - neplatné koncové napětí",
            0x02: "Start Current - neplatný startovní proud",
            0x03: "Mode - neplatný režim",
            0x04: "Enabled - neplatné povolení",
        }
        
        if result_code is not None:
            rc = int(result_code)
            error_msg = error_messages.get(rc, f"Neznámá chyba (kód: 0x{rc:02X})")
        else:
            error_msg = "Neznámá chyba"
        return {'success': False, 'error': error_msg, 'code': result_code}

    def write_all(self, basic: Dict[str, Any], pedal: Dict[str, Any], throttle: Dict[str, Any]) -> Dict[str, Any]:
        results = {}
        
        if basic:
            results['basic'] = self.write_basic(basic)
        if pedal:
            results['pedal'] = self.write_pedal(pedal)
        if throttle:
            results['throttle'] = self.write_throttle(throttle)
        
        self._cache.clear()
        
        all_success = all(r.get('success', False) for r in results.values())
        return {
            'success': all_success,
            'results': results
        }

    def write_custom_raw(self, hex_data: str) -> Optional[Dict[str, Any]]:
        try:
            cmd = bytes.fromhex(hex_data.replace(' ', '').replace('0x', ''))
            response = self._send_command(cmd)
            
            if response:
                return {
                    'sent_hex': cmd.hex(),
                    'response_hex': response.hex(),
                    'response_bytes': list(response),
                    'length': len(response)
                }
            return None
        except Exception as e:
            logger.error(f"Custom write error: {e}")
            return None

    def read_live_data(self) -> Optional['BafangUART.LiveData']:
        cmd = bytes([0x11, 0x19])
        response = self._send_command(cmd)

        if not response:
            return None

        payload = self._payload(response)
        if len(payload) < 18:
            return None

        return self.LiveData(
            wheel_speed=(payload[0] << 8) | payload[1],
            motor_rpm=(payload[4] << 8) | payload[5],
            battery_voltage=((payload[6] << 8) | payload[7]) / 10.0,
            battery_current=((payload[8] << 8) | payload[9]) / 10.0,
            motor_current=((payload[10] << 8) | payload[11]) / 10.0,
            controller_temp=payload[12] if payload[12] < 128 else payload[12] - 256,
            motor_temp=payload[13] if payload[13] < 128 else payload[13] - 256,
            torque_sensor=(payload[14] << 8) | payload[15],
            cadence=payload[16],
            assistant_level=payload[17],
            raw_bytes=list(response)
        )

    def read_errors(self) -> Optional['BafangUART.Errors']:
        cmd = bytes([0x11, 0x1A])
        response = self._send_command(cmd)

        if not response:
            return None

        payload = self._payload(response)
        if len(payload) < 3:
            return None

        error_codes = {
            0x01: "Normální", 0x03: "Brzda (E03)", 0x04: "Throttle (E04)",
            0x05: "Throttle (E05)", 0x06: "Throttle (E06)",
            0x11: "Teplota řídící (E11)", 0x12: "Proud (E12)",
            0x13: "Teplota baterie (E13)", 0x14: "Teplota motoru (E14)",
            0x21: "Rychlostní sensor (E21)", 0x22: "BMS (E22)",
            0x23: "Světlo (E23)", 0x24: "Sensor světla (E24)",
            0x25: "Torque (E25)", 0x26: "Torque (E26)", 0x30: "Komunikace (E30)"
        }

        return self.Errors(
            system_status=error_codes.get(payload[0], "Neznámý"),
            error_code=payload[0],
            controller_fw=payload[1],
            motor_fw=payload[2],
            raw_bytes=list(response)
        )

    def read_experimental(self) -> Dict[str, Any]:
        experimental = {}
        cmd_codes = [
            (0x55, "config_version"), (0x56, "unknown_56"), (0x57, "unknown_57"),
            (0x58, "motor_params"), (0x59, "controller_params"), (0x5A, "battery_params"),
            (0x5B, "display_params"), (0x70, "eeprom_1"), (0x71, "eeprom_2"),
            (0x72, "eeprom_3"), (0x73, "eeprom_4"), (0x78, "debug_1"),
            (0x80, "factory_1"), (0x90, "calib_1"),
        ]
        
        for code, name in cmd_codes:
            cmd = bytes([0x11, code])
            response = self._send_command(cmd, timeout=0.1)
            if response and len(response) > 2:
                experimental[name] = {
                    'response_hex': response.hex(),
                    'raw_bytes': list(response),
                    'length': len(response),
                    'data_bytes': list(response[2:])
                }
        
        return experimental

    def torque_calibration(self) -> bool:
        cmd = bytes([0x16, 0xA0, 0x01, 0x01, self._calculate_checksum(bytes([0x16, 0xA0, 0x01, 0x01]))])
        response = self._send_command(cmd)
        return response is not None and len(response) > 1 and response[1] == 0x01

    def set_wheel_circumference(self, circumference_mm: int) -> bool:
        cmd = bytes([0x16, 0x52, 0x01, 0x06, circumference_mm & 0xFF, self._calculate_checksum(bytes([0x16, 0x52, 0x01, 0x06, circumference_mm & 0xFF]))])
        response = self._send_command(cmd)
        return response is not None

    def read_config_version(self) -> Optional[Dict[str, Any]]:
        cmd = bytes([0x11, 0x55])
        response = self._send_command(cmd)
        
        if not response or len(response) < 10:
            return None
        
        return {
            'config_version': response[2],
            'config_checksum': (response[3] << 8) | response[4],
            'param_count': response[5],
            'crc': (response[6] << 8) | response[7],
            'raw_bytes': list(response)
        }

    def reset_to_defaults(self) -> bool:
        cmd = bytes([0x16, 0xFF, 0x01, 0x01, self._calculate_checksum(bytes([0x16, 0xFF, 0x01, 0x01]))])
        response = self._send_command(cmd)
        return response is not None

    def scan_all_commands(self) -> Dict[str, Any]:
        results = {}
        
        for code in range(0x10, 0xFF):
            cmd = bytes([0x11, code])
            response = self._send_command(cmd, timeout=0.05)
            if response and len(response) > 3:
                results[f"0x{code:02X}"] = {
                    'response_hex': response.hex(),
                    'first_byte': f"0x{response[0]:02X}",
                    'length': len(response),
                    'data': list(response[1:])
                }
        
        return results

    def clear_cache(self):
        self._cache.clear()
