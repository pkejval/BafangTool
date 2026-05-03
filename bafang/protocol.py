try:
    import serial
except ImportError:
    serial = None
import time
import logging
import threading
import queue
from collections import deque
from typing import Optional, Dict, Any, Tuple, List
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

BAUDRATE = 1200
DATA_BITS = 8
PARITY = 'N'
STOP_BITS = 1
REQUEST_SPACING = 0.3

@dataclass
class CommandDef:
    """Definition of a UART command"""
    read_code: int
    write_code: int = 0x16
    response_id: int = 0
    timeout: float = 0.15


@dataclass
class QueuedCommand:
    cmd: bytes
    wait_response: bool
    timeout: float
    done: threading.Event = field(default_factory=threading.Event)
    response: Optional[bytes] = None
    exception: Optional[BaseException] = None

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
    STATE_DISCONNECTED = 'DISCONNECTED'
    STATE_OPENING = 'OPENING'
    STATE_HANDSHAKING = 'HANDSHAKING'
    STATE_READY = 'READY'
    STATE_BUSY = 'BUSY'
    STATE_ERROR = 'ERROR'

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
        self.state = self.STATE_DISCONNECTED
        self.last_error: Optional[str] = None
        self.device_info: Dict[str, Any] = {}
        self._cache: Dict[str, Tuple[float, Any]] = {}
        self._last_read: Dict[str, bytes] = {}
        self._initial_snapshot: Dict[str, Dict[str, Any]] = {}
        self._allowed_keys: Dict[str, set] = {}
        self._section_meta: Dict[str, Dict[str, Any]] = {}
        self._rx_buffer = bytearray()
        self._rx_frames = deque(maxlen=64)
        self._rx_condition = threading.Condition()
        self._command_lock = threading.Lock()
        self._command_queue: queue.Queue[Optional[QueuedCommand]] = queue.Queue()
        self._command_worker_thread: Optional[threading.Thread] = None
        self._command_worker_stop = threading.Event()
        self._last_command_at = 0.0
        self._reader_thread: Optional[threading.Thread] = None
        self._reader_stop = threading.Event()

    def connect(self) -> bool:
        try:
            self._set_state(self.STATE_OPENING)
            logger.info('Opening Bafang UART port=%s baudrate=%d data=%d parity=%s stop=%d', self.port, BAUDRATE, DATA_BITS, PARITY, STOP_BITS)
            if self.serial is None:
                if serial is None:
                    raise RuntimeError('pyserial is not available and no serial transport was provided')
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=BAUDRATE,
                    bytesize=DATA_BITS,
                    parity=PARITY,
                    stopbits=STOP_BITS,
                    timeout=0.05
                )
            self._drain_input()
            self._start_reader()
            self._start_command_worker()
            time.sleep(REQUEST_SPACING)
            
            self._set_state(self.STATE_HANDSHAKING)
            if self._connect_cmd():
                self.connected = True
                self._set_state(self.STATE_READY)
                logger.info('Bafang UART connected port=%s device_info=%s', self.port, self.device_info)
                return True
            logger.error('Bafang UART handshake failed port=%s', self.port)
            self._set_state(self.STATE_ERROR, 'Handshake failed')
            self._stop_command_worker()
            return False
        except Exception as e:
            logger.exception('Bafang UART connection exception port=%s', self.port)
            self._set_state(self.STATE_ERROR, str(e))
            self._stop_command_worker()
            return False

    def disconnect(self):
        logger.info('Disconnecting Bafang UART port=%s connected=%s', self.port, self.connected)
        self._stop_command_worker()
        self._stop_reader()
        if self.serial and self._serial_is_open():
            self.serial.close()
        if self._external_serial:
            self.serial = None
        self.connected = False
        self._set_state(self.STATE_DISCONNECTED)
        self._cache.clear()
        self._last_read.clear()
        self._initial_snapshot.clear()
        self._allowed_keys.clear()
        self._section_meta.clear()
        with self._rx_condition:
            self._rx_buffer.clear()
            self._rx_frames.clear()
            self._rx_condition.notify_all()

    def _calculate_checksum(self, data: bytes) -> int:
        return sum(data) % 256

    def _calculate_bafang_write_checksum(self, data: bytes) -> int:
        # Bafang UART write checksum is computed from code + length + payload,
        # excluding the leading write marker 0x16.
        return sum(data[1:]) % 256

    def _start_reader(self):
        if self._reader_thread and self._reader_thread.is_alive():
            return
        if not self.serial or not self._serial_is_open():
            return
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, name=f"BafangUART-{self.port}", daemon=True)
        self._reader_thread.start()

    def _stop_reader(self):
        self._reader_stop.set()
        with self._rx_condition:
            self._rx_condition.notify_all()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=0.5)
        self._reader_thread = None

    def _start_command_worker(self):
        if self._command_worker_thread and self._command_worker_thread.is_alive():
            return
        if not self.serial or not self._serial_is_open():
            return
        self._command_worker_stop.clear()
        self._command_worker_thread = threading.Thread(target=self._command_worker_loop, name=f"BafangUARTQueue-{self.port}", daemon=True)
        self._command_worker_thread.start()

    def _stop_command_worker(self):
        self._command_worker_stop.set()
        if self._command_worker_thread and self._command_worker_thread.is_alive():
            self._command_queue.put(None)
            self._command_worker_thread.join(timeout=1.0)
        self._command_worker_thread = None

    def _command_worker_loop(self):
        while not self._command_worker_stop.is_set():
            try:
                item = self._command_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                self._command_queue.task_done()
                break
            try:
                self._set_state(self.STATE_BUSY)
                item.response = self._send_command_direct(item.cmd, item.wait_response, item.timeout)
            except BaseException as exc:
                item.exception = exc
                logger.exception('UART command worker exception port=%s cmd=%s', self.port, item.cmd.hex(' '))
            finally:
                if self.connected and self.state != self.STATE_ERROR:
                    self._set_state(self.STATE_READY)
                item.done.set()
                self._command_queue.task_done()

    def _set_state(self, state: str, error: Optional[str] = None):
        self.state = state
        if error is not None:
            self.last_error = error
        elif state != self.STATE_ERROR:
            self.last_error = None

    def _reader_loop(self):
        while not self._reader_stop.is_set():
            try:
                if not self.serial or not self._serial_is_open():
                    break
                waiting = getattr(self.serial, 'in_waiting', 0)
                if callable(waiting):
                    waiting = waiting()
                size = max(1, min(int(waiting or 1), 2048))
                chunk = self.serial.read(size)
                if chunk:
                    self._append_rx_data(bytes(chunk))
            except Exception as e:
                if not self._reader_stop.is_set():
                    logger.exception('Serial reader exception port=%s', self.port)
                break

    def _append_rx_data(self, data: bytes):
        with self._rx_condition:
            self._rx_buffer.extend(data)
            self._parse_rx_buffer_locked()
            self._rx_condition.notify_all()

    def _parse_rx_buffer_locked(self):
        while len(self._rx_buffer) >= 3:
            if ((self._rx_buffer[0] + self._rx_buffer[1]) & 0xFF) == self._rx_buffer[2]:
                self._rx_frames.append(bytes(self._rx_buffer[:3]))
                del self._rx_buffer[:3]
                continue

            packet_len = self._rx_buffer[1] + 3
            if len(self._rx_buffer) < packet_len:
                # If the declared length is impossible for the current stream,
                # drop one byte and keep searching for the next valid header.
                if self._rx_buffer[1] > 128 and len(self._rx_buffer) > 3:
                    del self._rx_buffer[0]
                    continue
                break

            frame = bytes(self._rx_buffer[:packet_len])
            if (sum(frame[:-1]) & 0xFF) == frame[-1]:
                self._rx_frames.append(frame)
                del self._rx_buffer[:packet_len]
                continue

            del self._rx_buffer[0]

    def _clear_queued_frames(self):
        with self._rx_condition:
            self._rx_buffer.clear()
            self._rx_frames.clear()

    def _infer_response_id(self, cmd: bytes) -> Optional[int]:
        if len(cmd) >= 2 and cmd[0] in (0x11, 0x14, 0x16):
            return cmd[1]
        if len(cmd) >= 2 and cmd[0] == 0x17:
            return cmd[1]
        return None

    def _frame_matches(self, frame: bytes, expected_response_id: Optional[int]) -> bool:
        if expected_response_id is None:
            return True
        return len(frame) > 0 and frame[0] == expected_response_id

    def _wait_for_frame(self, expected_response_id: Optional[int], timeout: float) -> Optional[bytes]:
        deadline = time.monotonic() + max(timeout, 0.3)
        deferred = []
        with self._rx_condition:
            while True:
                for _ in range(len(self._rx_frames)):
                    frame = self._rx_frames.popleft()
                    if self._frame_matches(frame, expected_response_id):
                        for old in reversed(deferred):
                            self._rx_frames.appendleft(old)
                        return frame
                    deferred.append(frame)

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    for old in reversed(deferred):
                        self._rx_frames.appendleft(old)
                    return None
                self._rx_condition.wait(min(remaining, 0.05))

    def _send_command(self, cmd: bytes, wait_response: bool = True, timeout: float = 0.6) -> Optional[bytes]:
        if not self.serial or not self._serial_is_open():
            return None
        if threading.current_thread() is self._command_worker_thread:
            return self._send_command_direct(cmd, wait_response, timeout)

        if not self._command_worker_thread or not self._command_worker_thread.is_alive():
            self._start_command_worker()

        if not self._command_worker_thread or not self._command_worker_thread.is_alive():
            return self._send_command_direct(cmd, wait_response, timeout)

        item = QueuedCommand(bytes(cmd), wait_response, timeout)
        self._command_queue.put(item)
        wait_timeout = max(timeout + REQUEST_SPACING + 5.0, 6.0)
        if not item.done.wait(wait_timeout):
            logger.error('UART command queue timeout port=%s cmd=%s wait_timeout=%.2f', self.port, cmd.hex(' '), wait_timeout)
            return None
        if item.exception is not None:
            logger.error('UART command queue failed port=%s cmd=%s exception=%s', self.port, cmd.hex(' '), item.exception)
            return None
        return item.response

    def _send_command_direct(self, cmd: bytes, wait_response: bool = True, timeout: float = 0.6) -> Optional[bytes]:
        if not self.serial or not self._serial_is_open():
            return None
        
        try:
            if not self._reader_thread or not self._reader_thread.is_alive():
                self._start_reader()
            expected_response_id = self._infer_response_id(cmd)
            with self._command_lock:
                self._pace_command_locked()
                self._clear_queued_frames()
                logger.debug('UART TX port=%s cmd=%s expected_response=%s timeout=%.2f', self.port, cmd.hex(' '), f'0x{expected_response_id:02X}' if expected_response_id is not None else None, timeout)
                self.serial.write(cmd)
                self.serial.flush()
                self._last_command_at = time.monotonic()

                if not wait_response:
                    return b'\x01'

                response = self._wait_for_frame(expected_response_id, timeout)
                if response:
                    logger.debug('UART RX port=%s response=%s', self.port, response.hex(' '))
                else:
                    logger.warning('UART response timeout port=%s cmd=%s expected_response=%s timeout=%.2f', self.port, cmd.hex(' '), f'0x{expected_response_id:02X}' if expected_response_id is not None else None, timeout)
                return response
        except Exception as e:
            logger.exception('Send command exception port=%s cmd=%s', self.port, cmd.hex(' '))
            return None

    def _pace_command_locked(self):
        elapsed = time.monotonic() - self._last_command_at
        if elapsed < REQUEST_SPACING:
            time.sleep(REQUEST_SPACING - elapsed)

    def _drain_input(self):
        for method in ('reset_input_buffer', 'flushInput'):
            fn = getattr(self.serial, method, None)
            if callable(fn):
                try:
                    fn()
                    return
                except Exception:
                    logger.exception('Failed to drain input using %s port=%s', method, self.port)

    def _read_response(self, timeout: float = 0.3, size: int = 2048) -> bytes:
        deadline = time.time() + max(timeout, 0.3)
        buffer = bytearray()
        while time.time() < deadline:
            chunk = self.serial.read(size)
            if chunk:
                buffer.extend(chunk)
                frame = self._extract_frame(bytes(buffer))
                if frame:
                    return frame
            elif buffer:
                frame = self._extract_frame(bytes(buffer), allow_incomplete=True)
                if frame:
                    return frame
                break
        return bytes(buffer)

    def _extract_frame(self, data: bytes, allow_incomplete: bool = False) -> Optional[bytes]:
        idx = 0
        while idx < len(data):
            remaining = data[idx:]
            if len(remaining) >= 3 and ((remaining[0] + remaining[1]) & 0xFF) == remaining[2]:
                return remaining[:3]
            if len(remaining) >= 3:
                packet_len = remaining[1] + 3
                if len(remaining) >= packet_len:
                    frame = remaining[:packet_len]
                    if (sum(frame[:-1]) & 0xFF) == frame[-1]:
                        return frame
                    idx += 1
                    continue
                if allow_incomplete:
                    return remaining
                if idx + 1 < len(data):
                    idx += 1
                    continue
                break
            if allow_incomplete:
                return remaining
            break
        return None

    def _serial_is_open(self) -> bool:
        is_open = getattr(self.serial, 'is_open', None)
        if callable(is_open):
            return bool(is_open())
        if is_open is None:
            return True
        return bool(is_open)

    def _send_with_retry(self, cmd: bytes, expected_response_id: int, max_retries: int = 3) -> Optional[bytes]:
        """Send command with retry on failure"""
        for attempt in range(max_retries):
            response = self._send_command(cmd, timeout=0.8)
            if response and len(response) > 0 and response[0] == expected_response_id:
                return response
            logger.warning(
                'UART command attempt failed port=%s attempt=%d/%d cmd=%s expected_response=0x%02X response=%s',
                self.port,
                attempt + 1,
                max_retries,
                cmd.hex(' '),
                expected_response_id,
                response.hex(' ') if response else None,
            )
            if attempt < max_retries - 1:
                time.sleep(REQUEST_SPACING)
        return None

    def _connect_cmd(self) -> bool:
        cmd = bytes([0x11, 0x51, 0x04, 0xB0, 0x05])
        logger.info('Sending Bafang handshake port=%s cmd=%s', self.port, cmd.hex(' '))
        response = self._send_with_retry(cmd, 0x51)
        
        if not response or response[0] != 0x51 or len(response) < 19:
            logger.error('Invalid Bafang handshake response port=%s response=%s', self.port, response.hex(' ') if response else None)
            return False
            
        payload = self._payload(response)
        self.device_info = {
            'manufacturer': bytes(payload[0:4]).decode('ascii', errors='ignore'),
            'model': bytes(payload[4:8]).decode('ascii', errors='ignore'),
            'hw_version': f"{chr(payload[8])}.{chr(payload[9])}" if len(payload) > 9 else '',
            'fw_version': '.'.join(str(b) for b in payload[10:14]) if len(payload) > 13 else '',
            'voltage': {0: "24V", 1: "36V", 2: "48V", 3: "60V", 4: "24V-48V"}.get(payload[14] if len(payload) > 14 else None, "24V-60V"),
            'max_current': payload[15] if len(payload) > 15 else None,
            'raw_connect_response': response.hex()
        }
        logger.info('Bafang handshake accepted port=%s response=%s device_info=%s', self.port, response.hex(' '), self.device_info)
        return True

    def _read_ascii_info(self, cmd: bytes) -> Optional[str]:
        response = self._send_command(cmd, timeout=0.3)
        if not response:
            return None
        payload = self._payload(response)
        return bytes(payload).decode('ascii', errors='ignore').strip()

    def _refresh_bafang_info(self):
        firmware = self._read_ascii_info(bytes([0x11, 0x50]))
        system_code = self._read_ascii_info(bytes([0x14, 0x13]))
        serial_number = self._read_ascii_info(bytes([0x14, 0x14]))
        model_detail = self._read_ascii_info(bytes([0x14, 0x16]))
        if firmware:
            self.device_info['fw_version'] = firmware
        if system_code:
            self.device_info['system_code'] = system_code
        if serial_number:
            self.device_info['serial_number'] = serial_number
        if model_detail:
            self.device_info['model_detail'] = model_detail

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

    def diagnostic_snapshot(self) -> Dict[str, Any]:
        return {
            'port': self.port,
            'connected': self.connected,
            'state': self.state,
            'last_error': self.last_error,
            'device_info': dict(self.device_info),
            'section_meta': dict(self._section_meta),
            'last_read_hex': {key: value.hex() for key, value in self._last_read.items()},
            'cache_keys': list(self._cache.keys()),
            'queue_size': self._command_queue.qsize(),
            'reader_alive': bool(self._reader_thread and self._reader_thread.is_alive()),
            'command_worker_alive': bool(self._command_worker_thread and self._command_worker_thread.is_alive()),
        }

    def _writable_keys_for_section(self, section: str) -> set:
        variant = self._section_meta.get(section, {}).get('variant', 'native')
        if section == 'basic' and variant == 'bafang':
            keys = {'low_battery_voltage', 'max_current', 'wheel_size_code', 'wheel_diameter_x2', 'speedometer_type_code', 'speedometer_signals'}
            for i in range(10):
                keys.add(f'assist_current_{i}')
                keys.add(f'assist_speed_{i}')
            return keys
        if section == 'throttle' and variant == 'bafang':
            return {'start_voltage', 'end_voltage', 'throttle_mode', 'mode', 'throttle_assist_level', 'assist_level', 'designated_assist', 'throttle_speed_limit', 'speed_limit', 'throttle_start_current', 'start_current'}
        return self._allowed_keys.get(section, set())

    def _validate_range(self, section: str, key: str, value: Any) -> Optional[str]:
        if key == 'pedal_type':
            if value not in {'None', 'DH-Sensor-12', 'BB-Sensor-32', 'DoubleSignal-24'}:
                return f'{key} has unsupported value {value!r}.'
            return None
        if isinstance(value, bool):
            return None
        if not isinstance(value, int):
            return f'{key} must be an integer.'
        ranges = {
            'low_battery_voltage': (1, 60),
            'max_current': (1, 100),
            'speed_limit': (0, 100),
            'wheel_size_code': (0, 8),
            'wheel_diameter_x2': (0, 255),
            'speedometer_type_code': (0, 2),
            'speedometer_signals': (0, 63),
            'start_voltage': (0, 5000),
            'end_voltage': (0, 5000),
            'throttle_mode': (0, 1),
            'mode': (0, 255),
            'throttle_start_current': (0, 255),
            'start_current': (0, 255),
            'pedal_time_to_stop': (0, 2550),
            'pedal_stop_decay': (0, 2550),
            'pedal_keep_current': (0, 100),
        }
        for prefix in ('assist_current_', 'assist_speed_'):
            if key.startswith(prefix):
                ranges[key] = (0, 100)
        low, high = ranges.get(key, (0, 255))
        if value < low or value > high:
            return f'{key}={value} is outside allowed range {low}-{high}.'
        if key in ('start_voltage', 'end_voltage', 'throttle_start_voltage', 'throttle_end_voltage') and value % 100 != 0:
            return f'{key} must be divisible by 100 mV because Bafang UART stores voltage in 100 mV steps.'
        return None

    def _validate_changed(self, section: str, changed: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        writable = self._writable_keys_for_section(section)
        unsupported = sorted(key for key in changed if key not in writable)
        if unsupported:
            return {
                'success': False,
                'error': f'Unsupported {section} fields for variant {self._section_meta.get(section, {}).get("variant", "unknown")}: {", ".join(unsupported)}',
                'code': None,
                'exception_type': 'UnsupportedWriteField',
                'unsupported_fields': unsupported,
            }
        for key, value in changed.items():
            issue = self._validate_range(section, key, value)
            if issue:
                return {'success': False, 'error': issue, 'code': None, 'exception_type': 'ValidationError'}
        return None

    def _verify_write(self, section: str, changed: Dict[str, Any]) -> Dict[str, Any]:
        readers = {
            'basic': lambda: self._basic_to_writable(self.read_basic(use_cache=False)),
            'pedal': lambda: self._pedal_to_writable(self.read_pedal(use_cache=False)),
            'throttle': lambda: self._throttle_to_writable(self.read_throttle(use_cache=False)),
        }
        try:
            latest = readers[section]()
        except Exception:
            logger.exception('Read-after-write verification failed section=%s', section)
            return {'verified': False, 'verification_error': 'Read-after-write failed'}

        mismatches = {}
        for key, expected in changed.items():
            actual = latest.get(key)
            if actual != expected:
                mismatches[key] = {'expected': expected, 'actual': actual}
        if mismatches:
            logger.error('Read-after-write mismatch section=%s mismatches=%s', section, mismatches)
            return {'verified': False, 'verification_error': 'Controller did not confirm written values', 'mismatches': mismatches}
        self._initial_snapshot[section].update(changed)
        return {'verified': True}

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
            logger.exception('Raw command exception hex=%s', command_hex)
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
                logger.exception('Failed to read section=%s', name)
                read_errors[name] = {
                    'message': str(e),
                    'exception_type': e.__class__.__name__,
                }
                self._log_parse_issue(name, read_errors[name]['message'])
                return None
            
        basic = safe_read('basic', self.read_basic)
        # Some controllers return a stale first frame after connect, so refresh
        # basic once before rendering.
        basic = safe_read('basic', lambda: self.read_basic(use_cache=False)) or basic
        pedal = safe_read('pedal', self.read_pedal)
        throttle = safe_read('throttle', self.read_throttle)
        self._refresh_bafang_info()
        live_data = safe_read('live_data', self.read_live_data, required=False)
        errors = safe_read('errors', self.read_errors, required=False)
        result = {
            'device_info': self.device_info,
            'basic': basic,
            'pedal': pedal,
            'throttle': throttle,
            'live_data': live_data,
            'errors': errors,
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
                circumference=0,
                signal_number=data[5],
                start_pulse=data[3],
                torque_gain=data[4],
                torque_offset=data[7] * 10,
                torque_step=data[8],
                cadence_gain=data[9] * 10,
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
            levels = params.get('assist_levels', [])
            for i in range(10):
                cmd += bytes([self._clamp(levels[i]['current_percent'] if i < len(levels) else params.get(f'assist_current_{i}', 100), 0, 100, 100)])
            for i in range(10):
                cmd += bytes([self._clamp(levels[i]['speed_percent'] if i < len(levels) else params.get(f'assist_speed_{i}', 100), 0, 100, 100)])
            cmd += bytes([
                self._clamp(params.get('wheel_diameter_x2', self._wheel_diameter_from_code(params.get('wheel_size_code', 4)) * 2), 0, 255, 56),
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
        
        return cmd + bytes([self._calculate_bafang_write_checksum(cmd)])

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
                self._clamp(time_to_stop // 10, 0, 255, 0x19),
                self._clamp(params.get('pedal_current_decay', params.get('torque_step', params.get('current_decay', 0x08))), 0, 255, 0x08),
                self._clamp(stop_decay // 10, 0, 255, 0x00),
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
        
        return cmd + bytes([self._calculate_bafang_write_checksum(cmd)])

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
        
        return cmd + bytes([self._calculate_bafang_write_checksum(cmd)])

    def _build_serial_number_command(self, serial_number: str) -> bytes:
        payload = str(serial_number).encode('ascii')
        cmd = bytes([0x17, 0x01, len(payload)]) + payload
        return cmd + bytes([self._calculate_bafang_write_checksum(cmd)])

    def write_serial_number(self, serial_number: str) -> Dict[str, Any]:
        serial_number = str(serial_number or '').strip()
        if len(serial_number) < 3 or len(serial_number) > 60:
            return {'success': False, 'error': 'Serial number length must be 3-60 ASCII characters.', 'code': None}
        try:
            serial_number.encode('ascii')
        except UnicodeEncodeError:
            return {'success': False, 'error': 'Serial number must contain ASCII characters only.', 'code': None}

        cmd = self._build_serial_number_command(serial_number)
        response = self._send_command(cmd)

        if response is None:
            return {'success': False, 'error': 'No response from controller', 'code': None}

        result_code = response[1] if len(response) > 1 else None
        if result_code == len(serial_number):
            self.device_info['serial_number'] = serial_number
            return {'success': True, 'error': None, 'code': result_code}

        return {'success': False, 'error': f'Serial number write rejected (code: {result_code})', 'code': result_code}

    def write_basic(self, params: Dict[str, Any]) -> Dict[str, Any]:
        if 'basic' not in self._initial_snapshot:
            return {'success': False, 'error': 'Nejprve načtěte controller (Read).', 'code': None}

        guard = self._section_write_guard('basic')
        if guard is not None:
            return guard

        changed = self._changed_only('basic', params)
        if not changed:
            return {'success': True, 'error': None, 'code': None, 'skipped': True, 'message': 'Žádné změněné parametry.'}
        validation = self._validate_changed('basic', changed)
        if validation is not None:
            return validation

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
            verification = self._verify_write('basic', changed)
            if not verification.get('verified'):
                return {'success': False, 'error': verification.get('verification_error', 'Read-after-write verification failed'), 'code': result_code, **verification}
            return {'success': True, 'error': None, 'code': result_code, **verification}
        
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
        validation = self._validate_changed('pedal', changed)
        if validation is not None:
            return validation

        merged = dict(self._initial_snapshot['pedal'])
        merged.update(changed)
        cmd = self._build_pedal_command(merged)
        response = self._send_command(cmd)
        
        if response is None:
            return {'success': False, 'error': 'No response from controller', 'code': None}
        
        result_code = response[1] if len(response) > 1 else None
        
        if result_code == 0x0B:
            self._invalidate_cache('pedal', 'all_params')
            verification = self._verify_write('pedal', changed)
            if not verification.get('verified'):
                return {'success': False, 'error': verification.get('verification_error', 'Read-after-write verification failed'), 'code': result_code, **verification}
            return {'success': True, 'error': None, 'code': result_code, **verification}
        
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
        validation = self._validate_changed('throttle', changed)
        if validation is not None:
            return validation

        merged = dict(self._initial_snapshot['throttle'])
        merged.update(changed)
        cmd = self._build_throttle_command(merged)
        response = self._send_command(cmd)
        
        if response is None:
            return {'success': False, 'error': 'No response from controller', 'code': None}
        
        result_code = response[1] if len(response) > 1 else None
        
        if result_code == 0x06:
            self._invalidate_cache('throttle', 'all_params')
            verification = self._verify_write('throttle', changed)
            if not verification.get('verified'):
                return {'success': False, 'error': verification.get('verification_error', 'Read-after-write verification failed'), 'code': result_code, **verification}
            return {'success': True, 'error': None, 'code': result_code, **verification}
        
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
            logger.exception('Custom write exception hex=%s', hex_data)
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
        cmd = bytes([0x14, 0x15])
        response = self._send_command(cmd)

        if not response:
            return None

        payload = self._payload(response)
        if len(payload) < 1:
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
            controller_fw=payload[1] if len(payload) > 1 else 0,
            motor_fw=payload[2] if len(payload) > 2 else 0,
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
        payload = bytes([0x16, 0xA0, 0x01, 0x01])
        cmd = payload + bytes([self._calculate_bafang_write_checksum(payload)])
        response = self._send_command(cmd)
        return response is not None and len(response) > 1 and response[1] == 0x01

    def set_wheel_circumference(self, circumference_mm: int) -> bool:
        payload = bytes([0x16, 0x52, 0x01, 0x06, circumference_mm & 0xFF])
        cmd = payload + bytes([self._calculate_bafang_write_checksum(payload)])
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
        payload = bytes([0x16, 0xFF, 0x01, 0x01])
        cmd = payload + bytes([self._calculate_bafang_write_checksum(payload)])
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
