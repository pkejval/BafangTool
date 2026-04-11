import logging
import os
import json
import hashlib
from datetime import datetime
from flask import Flask, render_template, request, jsonify
from werkzeug.exceptions import HTTPException
from bafang.protocol import BafangUART

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
app.secret_key = 'bafang-tool-secret-key'

PROFILES_FILE = 'profiles.json'
controller = None


def _api_error_response(message: str, status_code: int, exception_type: str | None = None):
    payload = {'success': False, 'error': message}
    if exception_type:
        payload['exception_type'] = exception_type
    return jsonify(payload), status_code


@app.errorhandler(HTTPException)
def handle_http_exception(e: HTTPException):
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
    return [{'port': p.device, 'description': p.description} for p in serial.tools.list_ports.comports()]

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
            return jsonify({'error': 'Not connected'}), 400
        return f(*args, **kwargs)
    return decorated


def get_controller() -> BafangUART:
    global controller
    if controller is None:
        raise RuntimeError('Controller not connected')
    return controller


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

@app.route('/api/ports')
def ports():
    return jsonify(get_available_ports())

@app.route('/api/connect', methods=['POST'])
def connect():
    global controller
    payload = request.json or {}
    port = payload.get('port')
    
    if not port:
        return jsonify({'success': False, 'error': 'No port selected'}), 400
    
    if controller and controller.connected:
        controller.disconnect()
    
    controller = BafangUART(port)
    if controller.connect():
        return jsonify({'success': True})
    return jsonify({'success': False, 'error': 'Connection failed'}), 400

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    global controller
    if controller:
        controller.disconnect()
    return jsonify({'success': True})

@app.route('/api/status')
def status():
    if controller and controller.connected:
        return jsonify({'connected': True, 'port': controller.port, 'device_info': controller.device_info})
    return jsonify({'connected': False})

@app.route('/api/read')
@require_connection
def read_params():
    return jsonify(get_controller().read_all_known_params())

@app.route('/api/write', methods=['POST'])
@require_connection
def write_params():
    data = request.json or {}
    result = get_controller().write_all(data.get('basic', {}), data.get('pedal', {}), data.get('throttle', {}))
    return jsonify(result)

@app.route('/api/write_basic', methods=['POST'])
@require_connection
def write_basic():
    return jsonify(get_controller().write_basic(request.json or {}))

@app.route('/api/write_pedal', methods=['POST'])
@require_connection
def write_pedal():
    return jsonify(get_controller().write_pedal(request.json or {}))

@app.route('/api/write_throttle', methods=['POST'])
@require_connection
def write_throttle():
    return jsonify(get_controller().write_throttle(request.json or {}))

@app.route('/api/live_data')
@require_connection
def live_data():
    return jsonify(get_controller().read_live_data() or {})

@app.route('/api/errors')
@require_connection
def errors():
    return jsonify(get_controller().read_errors() or {})

@app.route('/api/torque_calibration', methods=['POST'])
@require_connection
def torque_calibration():
    return jsonify({'success': get_controller().torque_calibration()})

@app.route('/api/reset', methods=['POST'])
@require_connection
def reset():
    return jsonify({'success': get_controller().reset_to_defaults()})

@app.route('/api/config_version')
@require_connection
def config_version():
    return jsonify(get_controller().read_config_version() or {})

@app.route('/api/wheel_circumference', methods=['POST'])
@require_connection
def wheel_circumference():
    payload = request.json or {}
    return jsonify({'success': get_controller().set_wheel_circumference(payload.get('circumference', 2105))})

@app.route('/api/read_experimental')
@require_connection
def read_experimental():
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
    payload = request.json or {}
    result = get_controller().send_raw_command(payload.get('command', ''))
    return jsonify(result or {'error': 'No response'})

@app.route('/api/write_custom_raw', methods=['POST'])
@require_connection
def write_custom_raw():
    payload = request.json or {}
    result = get_controller().write_custom_raw(payload.get('hex', ''))
    return jsonify(result or {'error': 'No response'})

@app.route('/api/scan_commands')
@require_connection
def scan_commands():
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

    result = get_controller().write_all(config.get('basic', {}), config.get('pedal', {}), config.get('throttle', {}))
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
