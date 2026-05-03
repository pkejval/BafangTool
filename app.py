import logging
import os
import json
import hashlib
import threading
import time
from dataclasses import asdict, is_dataclass
from datetime import datetime
from flask import Flask, render_template, request, jsonify, Response
from werkzeug.exceptions import HTTPException
from bafang.protocol import BafangUART

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s [%(name)s] %(message)s'
)
logger = logging.getLogger(__name__)

app = Flask(__name__)
app.secret_key = 'BafangTool-secret-key'

PROFILES_FILE = 'profiles.json'
UNSAFE_ENABLED = os.environ.get('BAFANGTOOL_ENABLE_UNSAFE', '').lower() in {'1', 'true', 'yes', 'on'}
controller = None


def to_jsonable(value):
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, dict):
        return {key: to_jsonable(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(item) for item in value]
    return value


class LiveDataService:
    def __init__(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._latest = None
        self._last_error = None
        self._last_update = None
        self._failure_count = 0
        self._interval = 0.5

    def start(self, interval: float = 0.5):
        with self._lock:
            self._interval = max(0.25, min(float(interval or 0.5), 5.0))
            if self._thread and self._thread.is_alive():
                logger.info('Live data service already running interval=%.2fs', self._interval)
                return
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name='BafangLiveData', daemon=True)
            self._thread.start()
            logger.info('Live data service started interval=%.2fs', self._interval)

    def stop(self):
        self._stop.set()
        thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout=1.0)
        with self._lock:
            self._thread = None
        logger.info('Live data service stopped')

    def reset(self):
        self.stop()
        with self._lock:
            self._latest = None
            self._last_error = None
            self._last_update = None
            self._failure_count = 0

    def _run(self):
        while not self._stop.is_set():
            current_controller = controller
            if not current_controller or not current_controller.connected:
                with self._lock:
                    self._last_error = 'Not connected'
                self._stop.wait(self._interval)
                continue
            try:
                data = current_controller.read_live_data()
                with self._lock:
                    if data is not None:
                        self._latest = to_jsonable(data)
                        self._last_error = None
                        self._last_update = time.time()
                        self._failure_count = 0
                    else:
                        self._failure_count += 1
                        self._last_error = 'No live data response'
            except Exception as e:
                logger.exception('Live data read failed')
                with self._lock:
                    self._failure_count += 1
                    self._last_error = str(e)
            self._stop.wait(self._interval)

    def status(self):
        thread = self._thread
        with self._lock:
            age = None if self._last_update is None else max(0.0, time.time() - self._last_update)
            return {
                'running': bool(thread and thread.is_alive()),
                'latest': self._latest,
                'last_error': self._last_error,
                'last_update': self._last_update,
                'age': age,
                'failure_count': self._failure_count,
                'interval': self._interval,
            }


live_service = LiveDataService()


def _api_error_response(message: str, status_code: int, exception_type: str | None = None):
    payload = {'success': False, 'error': message}
    if exception_type:
        payload['exception_type'] = exception_type
    return jsonify(payload), status_code


def api_json(data, status_code: int = 200):
    response = jsonify(to_jsonable(data))
    return (response, status_code) if status_code != 200 else response


def api_success(data=None, **extra):
    payload = {'success': True, 'data': to_jsonable(data) if data is not None else None, 'error': None}
    payload.update(extra)
    return jsonify(payload)


@app.errorhandler(HTTPException)
def handle_http_exception(e: HTTPException):
    logger.warning('HTTP error path=%s method=%s status=%s error=%s', request.path, request.method, e.code, e)
    if request.path.startswith('/api/'):
        description = str(e.description) if e.description is not None else str(e)
        return _api_error_response(description, e.code or 500, e.__class__.__name__)
    return e


@app.errorhandler(Exception)
def handle_unhandled_exception(e: Exception):
    logger.exception('Unhandled exception in request')
    if request.path.startswith('/api/'):
        return _api_error_response(str(e), 500, e.__class__.__name__)
    return jsonify({'error': 'Internal server error'}), 500

def get_available_ports():
    import serial.tools.list_ports
    ports = [{'port': p.device, 'description': p.description} for p in serial.tools.list_ports.comports()]
    logger.info('Detected serial ports count=%d ports=%s', len(ports), ports)
    return ports

def load_profiles():
    if os.path.exists(PROFILES_FILE):
        with open(PROFILES_FILE, 'r') as f:
            return json.load(f)
    return {}

def save_profiles(profiles):
    with open(PROFILES_FILE, 'w') as f:
        json.dump(profiles, f, indent=2)

def require_connection(f):
    from functools import wraps
    @wraps(f)
    def decorated(*args, **kwargs):
        global controller
        if not controller or not controller.connected:
            logger.warning('Rejected API request without controller connection path=%s method=%s', request.path, request.method)
            return jsonify({'error': 'Not connected'}), 400
        return f(*args, **kwargs)
    return decorated


def get_controller() -> BafangUART:
    global controller
    if controller is None:
        raise RuntimeError('Controller not connected')
    return controller


def require_unsafe_enabled():
    if UNSAFE_ENABLED:
        return None
    logger.warning('Rejected unsafe API request path=%s method=%s', request.path, request.method)
    return _api_error_response('Unsafe operation is disabled. Set BAFANGTOOL_ENABLE_UNSAFE=1 to enable it.', 403, 'UnsafeOperationDisabled')


def run_with_live_paused(action: str, callback):
    status = live_service.status()
    was_running = bool(status.get('running'))
    interval = status.get('interval', 0.5)
    if was_running:
        logger.info('Pausing live data before %s', action)
        live_service.stop()
    try:
        return callback()
    finally:
        current_controller = controller
        if was_running and current_controller and current_controller.connected:
            logger.info('Resuming live data after %s', action)
            live_service.start(interval)


def get_controller_binding() -> dict:
    info = get_controller().device_info or {}
    identity = {
        'manufacturer': info.get('manufacturer', ''),
        'model': info.get('model', ''),
        'hw_version': info.get('hw_version', ''),
        'fw_version': info.get('fw_version', ''),
        'voltage': info.get('voltage', ''),
        'max_current': info.get('max_current', ''),
        'raw_connect_response': info.get('raw_connect_response', ''),
    }
    sig_source = '|'.join(str(identity[k]) for k in sorted(identity.keys()))
    identity['signature'] = hashlib.sha256(sig_source.encode('utf-8')).hexdigest()[:16]
    return identity

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/favicon.ico')
def favicon():
    return Response(status=204)

@app.route('/api/ports')
def ports():
    return jsonify(get_available_ports())

@app.route('/api/connect', methods=['POST'])
def connect():
    global controller
    payload = request.json or {}
    port = payload.get('port')
    
    if not port:
        logger.error('Connect requested without selected port')
        return jsonify({'success': False, 'error': 'No port selected'}), 400
    
    if controller and controller.connected:
        logger.info('Disconnecting existing controller before reconnect port=%s', controller.port)
        live_service.reset()
        controller.disconnect()
    
    logger.info('Connecting to controller port=%s', port)
    controller = BafangUART(port)
    if controller.connect():
        logger.info('Controller connected port=%s device_info=%s', port, controller.device_info)
        return jsonify({'success': True})
    logger.error('Controller connection failed port=%s', port)
    return jsonify({'success': False, 'error': 'Connection failed'}), 400

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    global controller
    live_service.reset()
    if controller:
        logger.info('Disconnecting controller port=%s connected=%s', controller.port, controller.connected)
        controller.disconnect()
    return jsonify({'success': True})

@app.route('/api/status')
def status():
    if controller and controller.connected:
        return jsonify({'connected': True, 'port': controller.port, 'state': controller.state, 'last_error': controller.last_error, 'device_info': controller.device_info})
    return jsonify({'connected': False, 'state': controller.state if controller else 'DISCONNECTED'})

@app.route('/api/diagnostics')
@require_connection
def diagnostics():
    return api_success({
        'controller': get_controller().diagnostic_snapshot(),
        'live': live_service.status(),
        'unsafe_enabled': UNSAFE_ENABLED,
    })

@app.route('/api/read')
@require_connection
def read_params():
    logger.info('Reading all known controller parameters')
    return api_json(get_controller().read_all_known_params())

@app.route('/api/write', methods=['POST'])
@require_connection
def write_params():
    data = request.json or {}
    logger.info('Writing all parameter sections keys=%s', sorted(data.keys()))
    result = run_with_live_paused('write_all', lambda: get_controller().write_all(data.get('basic', {}), data.get('pedal', {}), data.get('throttle', {})))
    if not result.get('success', False):
        logger.error('Write all failed result=%s', result)
    return jsonify(result)

@app.route('/api/write_basic', methods=['POST'])
@require_connection
def write_basic():
    result = run_with_live_paused('write_basic', lambda: get_controller().write_basic(request.json or {}))
    if not result.get('success', False):
        logger.error('Write basic failed result=%s', result)
    return jsonify(result)

@app.route('/api/write_pedal', methods=['POST'])
@require_connection
def write_pedal():
    result = run_with_live_paused('write_pedal', lambda: get_controller().write_pedal(request.json or {}))
    if not result.get('success', False):
        logger.error('Write pedal failed result=%s', result)
    return jsonify(result)

@app.route('/api/write_throttle', methods=['POST'])
@require_connection
def write_throttle():
    result = run_with_live_paused('write_throttle', lambda: get_controller().write_throttle(request.json or {}))
    if not result.get('success', False):
        logger.error('Write throttle failed result=%s', result)
    return jsonify(result)

@app.route('/api/live_data')
@require_connection
def live_data():
    return api_json(get_controller().read_live_data() or {})

@app.route('/api/live/start', methods=['POST'])
@require_connection
def live_start():
    payload = request.json or {}
    live_service.start(payload.get('interval', 0.5))
    return jsonify({'success': True, **live_service.status()})

@app.route('/api/live/stop', methods=['POST'])
@require_connection
def live_stop():
    live_service.stop()
    return jsonify({'success': True, **live_service.status()})

@app.route('/api/live/status')
@require_connection
def live_status():
    return jsonify(live_service.status())

@app.route('/api/errors')
@require_connection
def errors():
    return api_json(get_controller().read_errors() or {})

@app.route('/api/torque_calibration', methods=['POST'])
@require_connection
def torque_calibration():
    unsafe_response = require_unsafe_enabled()
    if unsafe_response:
        return unsafe_response
    return jsonify({'success': run_with_live_paused('torque_calibration', lambda: get_controller().torque_calibration())})

@app.route('/api/reset', methods=['POST'])
@require_connection
def reset():
    unsafe_response = require_unsafe_enabled()
    if unsafe_response:
        return unsafe_response
    return jsonify({'success': run_with_live_paused('reset_to_defaults', lambda: get_controller().reset_to_defaults())})

@app.route('/api/config_version')
@require_connection
def config_version():
    return jsonify(get_controller().read_config_version() or {})

@app.route('/api/wheel_circumference', methods=['POST'])
@require_connection
def wheel_circumference():
    unsafe_response = require_unsafe_enabled()
    if unsafe_response:
        return unsafe_response
    payload = request.json or {}
    return jsonify({'success': run_with_live_paused('set_wheel_circumference', lambda: get_controller().set_wheel_circumference(payload.get('circumference', 2105)))})

@app.route('/api/read_experimental')
@require_connection
def read_experimental():
    unsafe_response = require_unsafe_enabled()
    if unsafe_response:
        return unsafe_response
    return jsonify(get_controller().read_experimental())

@app.route('/api/read_raw_basic')
@require_connection
def read_raw_basic():
    return jsonify(get_controller().read_raw_basic() or {})

@app.route('/api/read_raw_pedal')
@require_connection
def read_raw_pedal():
    return jsonify(get_controller().read_raw_pedal() or {})

@app.route('/api/read_raw_throttle')
@require_connection
def read_raw_throttle():
    return jsonify(get_controller().read_raw_throttle() or {})

@app.route('/api/send_raw_command', methods=['POST'])
@require_connection
def send_raw_command():
    unsafe_response = require_unsafe_enabled()
    if unsafe_response:
        return unsafe_response
    payload = request.json or {}
    command = payload.get('command', '')
    logger.info('Sending raw command command=%s', command)
    result = run_with_live_paused('send_raw_command', lambda: get_controller().send_raw_command(command))
    if not result:
        logger.error('Raw command returned no response command=%s', command)
    return jsonify(result or {'error': 'No response'})

@app.route('/api/write_custom_raw', methods=['POST'])
@require_connection
def write_custom_raw():
    unsafe_response = require_unsafe_enabled()
    if unsafe_response:
        return unsafe_response
    payload = request.json or {}
    hex_data = payload.get('hex', '')
    logger.info('Sending custom raw write hex=%s', hex_data)
    result = run_with_live_paused('write_custom_raw', lambda: get_controller().write_custom_raw(hex_data))
    if not result:
        logger.error('Custom raw write returned no response hex=%s', hex_data)
    return jsonify(result or {'error': 'No response'})

@app.route('/api/scan_commands')
@require_connection
def scan_commands():
    unsafe_response = require_unsafe_enabled()
    if unsafe_response:
        return unsafe_response
    return jsonify(get_controller().scan_all_commands())

@app.route('/api/profiles')
def get_profiles():
    return jsonify(load_profiles())

@app.route('/api/profiles/<name>')
def get_profile(name):
    name = name.replace('_', ' ')
    profiles = load_profiles()
    if name in profiles:
        return jsonify(profiles[name])
    return jsonify({'error': 'Profile not found'}), 404

@app.route('/api/profiles/<name>', methods=['POST'])
@require_connection
def save_profile(name):
    name = name.replace('_', ' ')
    profiles = load_profiles()
    data = request.json or {}
    
    profile_data = {
        'name': name,
        'created_at': profiles.get(name, {}).get('created_at', None) or datetime.now().isoformat(),
        'updated_at': datetime.now().isoformat(),
        'config': data.get('config', {}),
        'description': data.get('description', ''),
        'bound_controller': get_controller_binding(),
    }
    
    profiles[name] = profile_data
    save_profiles(profiles)
    return jsonify({'success': True, 'profile': profile_data})

@app.route('/api/profiles/<name>', methods=['DELETE'])
def delete_profile(name):
    name = name.replace('_', ' ')
    profiles = load_profiles()
    if name in profiles:
        del profiles[name]
        save_profiles(profiles)
        return jsonify({'success': True})
    return jsonify({'error': 'Profile not found'}), 404

@app.route('/api/profiles/<name>/apply', methods=['POST'])
@require_connection
def apply_profile(name):
    name = name.replace('_', ' ')
    profiles = load_profiles()
    if name not in profiles:
        return jsonify({'error': 'Profile not found'}), 404
    
    config = profiles[name].get('config', {})
    bound = profiles[name].get('bound_controller')
    current = get_controller_binding()
    if not bound or bound.get('signature') != current.get('signature'):
        return jsonify({
            'success': False,
            'error': 'Profil je vázaný na jiný controller.',
            'profile_controller': bound,
            'current_controller': current,
        }), 409

    result = run_with_live_paused('apply_profile', lambda: get_controller().write_all(config.get('basic', {}), config.get('pedal', {}), config.get('throttle', {})))
    return jsonify(result)

@app.route('/api/profiles/import', methods=['POST'])
@require_connection
def import_profile():
    profiles = load_profiles()
    data = request.json or {}
    
    name = data.get('name', 'Imported')
    profile_data = {
        'name': name,
        'created_at': datetime.now().isoformat(),
        'updated_at': datetime.now().isoformat(),
        'config': data.get('config', {}),
        'description': data.get('description', 'Imported profile'),
        'bound_controller': get_controller_binding(),
        'imported_bound_controller': data.get('bound_controller'),
    }
    
    counter = 1
    original_name = name
    while name in profiles:
        name = f"{original_name} ({counter})"
        counter += 1
    
    profiles[name] = profile_data
    save_profiles(profiles)
    return jsonify({'success': True, 'profile': profile_data})

@app.route('/api/profiles/export/<name>')
def export_profile(name):
    name = name.replace('_', ' ')
    profiles = load_profiles()
    if name in profiles:
        return jsonify(profiles[name])
    return jsonify({'error': 'Profile not found'}), 404

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
