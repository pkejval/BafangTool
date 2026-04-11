import serial
import time
import logging
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass

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
    
    def __init__(self, port: str):
        self.port = port
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.device_info: Dict[str, Any] = {}
        self._cache: Dict[str, Tuple[float, Any]] = {}
        self._last_read: Dict[str, bytes] = {}
        self._initial_snapshot: Dict[str, Dict[str, Any]] = {}
        self._allowed_keys: Dict[str, set] = {}

    def connect(self) -> bool:
        try:
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
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False
        self._cache.clear()
        self._last_read.clear()
        self._initial_snapshot.clear()
        self._allowed_keys.clear()

    def _calculate_checksum(self, data: bytes) -> int:
        return sum(data) % 256

    def _send_command(self, cmd: bytes, wait_response: bool = True, timeout: float = 0.15) -> Optional[bytes]:
        if not self.serial or not self.serial.is_open:
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
            
        self.device_info = {
            'manufacturer': bytes(response[2:6]).decode('ascii', errors='ignore'),
            'model': bytes(response[6:10]).decode('ascii', errors='ignore'),
            'hw_version': f"{response[10]}.{response[11]}",
            'fw_version': f"{response[12]}.{response[13]}.{response[14]}.{response[15]}",
            'voltage': {0: "24V", 1: "36V", 2: "48V", 3: "60V", 4: "24V-48V"}.get(response[16], "Unknown"),
            'max_current': response[17],
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

    def _capture_initial(self, section: str, values: Dict[str, Any]):
        if section not in self._initial_snapshot:
            self._initial_snapshot[section] = dict(values)
            self._allowed_keys[section] = set(values.keys())

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

    def _basic_to_writable(self, data: Dict[str, Any]) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            'low_battery_voltage': data.get('low_battery_voltage'),
            'max_current': data.get('max_current'),
            'speed_limit': data.get('speed_limit'),
            'wheel_size_code': data.get('wheel_size_code', 4),
            'wheel_circumference': data.get('wheel_circumference', 0),
            'assist_level': data.get('assist_level', 1),
            'start_current': data.get('start_current', 20),
            'start_current_decay': data.get('start_current_decay', 10),
            'stop_delay': data.get('stop_delay', 20),
            'current_ramp': data.get('current_ramp', 15),
            'throttle_enabled': data.get('throttle_enabled', True),
            'throttle_start_voltage': data.get('throttle_start_voltage', 1100),
            'throttle_end_voltage': data.get('throttle_end_voltage', 4200),
            'temp_sensor_type': data.get('temp_sensor_type_code', 3),
        }
        levels = data.get('assist_levels', []) or []
        for i in range(10):
            if i < len(levels):
                out[f'assist_current_{i}'] = levels[i].get('current_percent', 100)
                out[f'assist_speed_{i}'] = levels[i].get('speed_percent', 100)
            else:
                out[f'assist_current_{i}'] = 100
                out[f'assist_speed_{i}'] = 100
        return out

    def _pedal_to_writable(self, data: Dict[str, Any]) -> Dict[str, Any]:
        return {
            'pedal_type': data.get('pedal_type', 'DoubleSignal-24'),
            'designated_assist': data.get('designated_assist', 0xFF),
            'speed_limit': data.get('speed_limit', 0xFF),
            'circumference': data.get('circumference', 0),
            'signal_number': data.get('signal_number', 6),
            'start_pulse': data.get('start_pulse', 0x14),
            'torque_gain': data.get('torque_gain', 0x0A),
            'torque_offset': data.get('torque_offset', 0x19),
            'torque_step': data.get('torque_step', 0x08),
            'cadence_gain': data.get('cadence_gain', 0x14),
            'cadence_min': data.get('cadence_min', 0x14),
            'cadence_max': data.get('cadence_max', 0x14),
        }

    def _throttle_to_writable(self, data: Dict[str, Any]) -> Dict[str, Any]:
        return {
            'start_voltage': data.get('start_voltage', 1100),
            'end_voltage': data.get('end_voltage', 4200),
            'start_current': data.get('start_current', 0x0B),
            'mode': data.get('mode', 0x23),
            'enabled': data.get('enabled', 0),
        }

    def _pick(self, values: list, idx: int, default: Any = "Unknown") -> Any:
        if 0 <= idx < len(values):
            return values[idx]
        return default

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
            
        result = {
            'device_info': self.device_info,
            'basic': self.read_basic(),
            'pedal': self.read_pedal(),
            'throttle': self.read_throttle(),
        }
        self._set_cache('all_params', result)
        return result

    def read_basic(self, use_cache: bool = True) -> Optional[Dict[str, Any]]:
        if use_cache:
            cached = self._get_cache('basic')
            if cached:
                return cached
                
        cmd = bytes([0x11, 0x52])
        response = self._send_with_retry(cmd, 0x52)
        
        if not response:
            return None
        
        data = response[2:]
        if len(data) < 25:
            return None
        data = data[:26]
        temp_sensor_type_code = data[25] if len(data) > 25 else None
        
        result = {
            'low_battery_voltage': data[0] + 18,
            'max_current': data[1],
            'speed_limit': data[2],
            'wheel_size_code': data[3],
            'wheel_size': self._pick(["700C", "26\"(A)", "26\"(B)", "27.5\"", "28\"", "29\"", "16\"", "20\"", "24\"", "12\""], data[3]),
            'wheel_circumference': data[4],
            'speedometer_type': self._pick(["External", "Internal", "Motor phase"], data[5] & 0x03),
            'speedometer_signals': (data[5] >> 2) & 0x0F,
            'assist_levels': [{'level': i, 'current_percent': data[i], 'speed_percent': data[i + 10]} for i in range(10)],
            'assist_level': data[17],
            'start_current': data[18],
            'start_current_decay': data[19],
            'stop_delay': data[20],
            'current_ramp': data[21],
            'throttle_enabled': (data[22] & 0x01) == 0x01,
            'throttle_start_voltage': data[23] * 100,
            'throttle_end_voltage': data[24] * 100,
            'temp_sensor_type_code': temp_sensor_type_code,
            'temp_sensor_type': self._pick(["No sensor", "Controller only", "Motor only", "Both"], temp_sensor_type_code) if temp_sensor_type_code is not None else "Unavailable",
            'raw_bytes': list(data)
        }
        self._set_cache('basic', result)
        self._capture_initial('basic', self._basic_to_writable(result))
        return result

    def read_pedal(self, use_cache: bool = True) -> Optional[Dict[str, Any]]:
        if use_cache:
            cached = self._get_cache('pedal')
            if cached:
                return cached
                
        cmd = bytes([0x11, 0x53])
        response = self._send_with_retry(cmd, 0x53)
        
        if not response or len(response) < 14:
            return None
        
        data = response[2:14]
        result = {
            'pedal_type': self._pick(["None", "DH-Sensor-12", "BB-Sensor-32", "DoubleSignal-24"], data[0]),
            'designated_assist': data[1],
            'speed_limit': data[2],
            'circumference': data[3],
            'signal_number': data[4],
            'start_pulse': data[5],
            'torque_gain': data[6],
            'torque_offset': data[7],
            'torque_step': data[8],
            'cadence_gain': data[9],
            'cadence_min': data[10],
            'cadence_max': data[11],
            'raw_bytes': list(data)
        }
        self._set_cache('pedal', result)
        self._capture_initial('pedal', self._pedal_to_writable(result))
        return result

    def read_throttle(self, use_cache: bool = True) -> Optional[Dict[str, Any]]:
        if use_cache:
            cached = self._get_cache('throttle')
            if cached:
                return cached
                
        cmd = bytes([0x11, 0x54])
        response = self._send_with_retry(cmd, 0x54)
        
        if not response:
            return None
        
        data = response[2:]
        if len(data) < 5:
            return None
        start_percent = data[5] if len(data) > 5 else None
        result = {
            'start_voltage': data[0] * 100,
            'end_voltage': data[1] * 100,
            'start_current': data[2],
            'mode': data[3],
            'enabled': data[4] == 0x01,
            'start_percent': start_percent,
            'raw_bytes': list(data)
        }
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
        cmd = bytes([0x16, 0x52, 0x24])
        cmd += bytes([
            params.get('low_battery_voltage', 28) - 18,
            params.get('max_current', 16),
            params.get('speed_limit', 25),
            params.get('wheel_size_code', params.get('wheel_size', 4)),
            params.get('wheel_circumference', 0)
        ])
        
        for i in range(10):
            cmd += bytes([
                params.get(f'assist_current_{i}', 100),
                params.get(f'assist_speed_{i}', 100)
            ])
        
        cmd += bytes([
            params.get('assist_level', 1),
            params.get('start_current', 20),
            params.get('start_current_decay', 10),
            params.get('stop_delay', 20),
            params.get('current_ramp', 15),
            0x01 if params.get('throttle_enabled', True) else 0x00,
            params.get('throttle_start_voltage', 1100) // 100,
            params.get('throttle_end_voltage', 4200) // 100,
            params.get('temp_sensor_type', 3),
            0x00
        ])
        
        return cmd + bytes([self._calculate_checksum(cmd)])

    def _build_pedal_command(self, params: Dict[str, Any]) -> bytes:
        pedal_types = {'None': 0, 'DH-Sensor-12': 1, 'BB-Sensor-32': 2, 'DoubleSignal-24': 3}
        
        cmd = bytes([0x16, 0x53, 0x0B])
        cmd += bytes([pedal_types.get(params.get('pedal_type', 'DoubleSignal-24'), 3)])
        cmd += bytes([params.get('designated_assist', 0xFF)])
        cmd += bytes([
            params.get('speed_limit', 0xFF),
            params.get('circumference', 0x00),
            params.get('signal_number', 0x06),
            params.get('start_pulse', 0x14),
            params.get('torque_gain', 0x0A),
            params.get('torque_offset', 0x19),
            params.get('torque_step', 0x08),
            params.get('cadence_gain', 0x14),
            params.get('cadence_min', 0x14),
            params.get('cadence_max', 0x14)
        ])
        
        return cmd + bytes([self._calculate_checksum(cmd)])

    def _build_throttle_command(self, params: Dict[str, Any]) -> bytes:
        cmd = bytes([0x16, 0x54, 0x06])
        cmd += bytes([
            params.get('start_voltage', 1100) // 100,
            params.get('end_voltage', 4200) // 100,
            params.get('start_current', 0x0B),
            params.get('mode', 0x23),
            params.get('enabled', 0x00)
        ])
        
        return cmd + bytes([self._calculate_checksum(cmd)])

    def write_basic(self, params: Dict[str, Any]) -> Dict[str, Any]:
        if 'basic' not in self._initial_snapshot:
            return {'success': False, 'error': 'Nejprve načtěte controller (Read).', 'code': None}

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
        
        if result_code == 0x24:
            self._cache.pop('basic', None)
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
            self._cache.pop('pedal', None)
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
            self._cache.pop('throttle', None)
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

    def read_live_data(self) -> Optional[Dict[str, Any]]:
        cmd = bytes([0x11, 0x19])
        response = self._send_command(cmd)
        
        if not response or len(response) < 20:
            return None
        
        return {
            'wheel_speed': (response[2] << 8) | response[3],
            'motor_rpm': (response[6] << 8) | response[7],
            'battery_voltage': ((response[8] << 8) | response[9]) / 10.0,
            'battery_current': ((response[10] << 8) | response[11]) / 10.0,
            'motor_current': ((response[12] << 8) | response[13]) / 10.0,
            'controller_temp': response[14] if response[14] < 128 else response[14] - 256,
            'motor_temp': response[15] if response[15] < 128 else response[15] - 256,
            'torque_sensor': (response[16] << 8) | response[17],
            'cadence': response[18],
            'assistant_level': response[19],
            'raw_bytes': list(response)
        }

    def read_errors(self) -> Optional[Dict[str, Any]]:
        cmd = bytes([0x11, 0x1A])
        response = self._send_command(cmd)
        
        if not response or len(response) < 6:
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
        
        return {
            'system_status': error_codes.get(response[2], "Neznámý"),
            'error_code': response[2],
            'controller_fw': response[3],
            'motor_fw': response[4],
            'raw_bytes': list(response)
        }

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
        return response is not None and response[1] == 0x01

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
