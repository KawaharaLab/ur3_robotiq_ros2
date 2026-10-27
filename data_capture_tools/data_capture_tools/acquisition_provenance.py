"""Read-only descriptions of user measurements, runtime and source checkout."""
import hashlib
from dataclasses import asdict, is_dataclass
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time


PHYSICAL_FIELDS = ('specimen_id', 'mount_id', 'clamp_gap_mm', 'specimen_width_mm', 'clamp_insertion_depth_mm',
                   'orientation', 'finger_id', 'force_sensor_left_id', 'force_sensor_right_id')


def effective_capture_config(config):
    if is_dataclass(config):
        value = asdict(config)
        value.pop('raw', None)
        return value
    return dict(getattr(config, 'raw', {}))


def read_capture_measurements(config, legacy_path=None):
    """Use the loaded YAML snapshot, or an explicitly selected legacy JSON file.

    Never discover a cwd JSON implicitly: a stale file must not override YAML.
    The loaded config is deliberately not hot-reloaded separately from motion.
    """
    raw_config = getattr(config, 'raw', {})
    task_name = getattr(config, 'task_name', raw_config.get('task_name'))
    if legacy_path is not None:
        result = read_measurements(legacy_path)
        if not result['available']:
            raise ValueError('Explicit specimen_metadata_file does not exist: '+str(legacy_path))
        values = dict(result['values'])
        result['source_file_sha256'] = result['sha256']
        result['source'] = 'explicit_legacy_json'
    else:
        values = raw_config.get('specimen_metadata', {})
        if values is None:
            values = {}
        if not isinstance(values, dict):
            raise ValueError('specimen_metadata must be a YAML mapping')
        values = dict(values)
        result = dict(available=True, source='loaded_capture_yaml',
                      path=str(getattr(config, 'source_path', '') or '') or None)
    if values.get('specimen_id') in (None, ''):
        values['specimen_id'] = task_name
        result['specimen_id_source'] = 'task_name'
    else:
        result['specimen_id_source'] = 'explicit'
    _validate_measurements(values)
    encoded = json.dumps(values, sort_keys=True, ensure_ascii=False, allow_nan=False).encode('utf-8')
    physical = {key: values.get(key) for key in PHYSICAL_FIELDS}
    result.update(values=values, sha256=hashlib.sha256(encoded).hexdigest(),
                  missing_fields=[k for k in PHYSICAL_FIELDS if values.get(k) in (None, '')],
                  physical_condition_id=hashlib.sha256(json.dumps(physical, sort_keys=True).encode()).hexdigest(),
                  note='Optional user-reported measurements, not motion inputs. Unknown values are not inferred. '
                       'IDs describe reported conditions, not physical equivalence; init_id identifies the contact reference.')
    return result


def _validate_measurements(values):
    if values.get('schema_version', 1) != 1:
        raise ValueError('Unsupported specimen metadata schema')
    for key in ('clamp_gap_mm', 'specimen_width_mm', 'measurement_uncertainty_mm',
                'clamp_insertion_depth_mm', 'ambient_temperature_c'):
        value = values.get(key)
        if value is not None and (type(value) not in (int, float) or not math.isfinite(value)
                                  or (key != 'ambient_temperature_c' and value < 0)):
            raise ValueError(key+' must be a finite number or null')
    for key in PHYSICAL_FIELDS + ('material_id', 'measurement_method', 'measured_at', 'notes',
                                  'width_measurement_state', 'contact_location', 'setup_photo', 'prior_use'):
        if key in ('clamp_gap_mm', 'specimen_width_mm', 'clamp_insertion_depth_mm'):
            continue
        if values.get(key) is not None and not isinstance(values[key], str):
            raise ValueError(key+' must be text or null')
    json.dumps(values, allow_nan=False)


def read_measurements(path):
    path = Path(path) if path is not None else None
    if path is None or not path.exists():
        return dict(available=False, path=str(path) if path else None, values={},
                    missing_fields=list(PHYSICAL_FIELDS), physical_condition_id=None,
                    note='Not supplied; dimensions were not measured by the software.')
    raw = path.read_bytes()
    if len(raw) > 65536:
        raise ValueError('Specimen metadata exceeds 64 KiB')
    values = json.loads(raw)
    if not isinstance(values, dict):
        raise ValueError('Specimen metadata must be a JSON object')
    _validate_measurements(values)
    physical = {key: values.get(key) for key in PHYSICAL_FIELDS}
    return dict(available=True, path=str(path.resolve()), sha256=hashlib.sha256(raw).hexdigest(),
                file_mtime_ns=path.stat().st_mtime_ns, values=values,
                missing_fields=[k for k in PHYSICAL_FIELDS if values.get(k) in (None, '')],
                physical_condition_id=hashlib.sha256(json.dumps(physical,sort_keys=True).encode()).hexdigest(),
                note='User-reported measurements; uncertainty is not estimated automatically. Labels do not change motion.')


def runtime_provenance(config_path=None):
    module_dir = Path(__file__).resolve().parent
    hashes = {}
    for name in ('data_capture_node_10times.py','init_recording.py','acquisition_provenance.py',
                 'force_press_worker.py','force_press_collection.py'):
        p = module_dir/name
        if p.exists(): hashes[name] = hashlib.sha256(p.read_bytes()).hexdigest()
    result = dict(captured_system_time_ns=time.time_ns(), captured_monotonic_ns=time.monotonic_ns(),
                  hostname=platform.node(), platform=platform.platform(), python=sys.version,
                  local_timezone=list(time.tzname),
                  monotonic_clock=dict(implementation=time.get_clock_info('monotonic').implementation,
                                       resolution_s=time.get_clock_info('monotonic').resolution),
                  ros_environment={k:os.environ.get(k) for k in ('ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION')},
                  runtime_python_source_sha256=hashes, source_checkout=None,
                  note='Checkout metadata is not proof of the running C++ binary; Python hashes describe loaded source paths.')
    if config_path is not None:
        config_path = Path(config_path).resolve()
        if config_path.is_file():
            result['config_source_sha256'] = hashlib.sha256(config_path.read_bytes()).hexdigest()
        root = next((p for p in config_path.parents if (p/'.git').exists()), None)
        if root:
            try:
                def git(*args):
                    return subprocess.check_output(['git','-C',str(root),*args],timeout=2,
                                                   stderr=subprocess.DEVNULL,text=True,
                                                   env={**os.environ,'GIT_OPTIONAL_LOCKS':'0'}).strip()
                result['source_checkout'] = dict(path=str(root),commit=git('rev-parse','HEAD'),
                                                 tracked_changes=git('status','--porcelain','--untracked-files=no'))
            except (OSError, subprocess.SubprocessError) as e:
                result['source_checkout'] = dict(path=str(root),error=type(e).__name__)
    return result


def legacy_conversion_description():
    return dict(model_id='legacy_piecewise_polynomial_v1_uncalibrated',
                width_to_count=dict(input_unit='m',rounding='truncate toward zero',
                    branches=[dict(min_m=.015,coefficients=[-1399.78,-1328.11,218.384]),
                              dict(min_m=.001,coefficients=[18315.,-1849.82,223.626])],
                    below_1mm_request_count=255),
                count_to_width=dict(output_unit='m',branches=[dict(max_count=200,coefficients=[-3.84615e-7,-5.67622e-4,.142692]),
                                                             dict(max_count=226,coefficients=[8.92857e-6,-4.38036e-3,.533911])],
                                    above_226_width_m=0.),
                limitations=['Forward and reverse fits are not exact inverses and contain boundary discontinuities.',
                             'Not calibrated for custom GelSight contact-surface spacing.',
                             'Half of aperture reduction assumes symmetric motion and is not isolated specimen strain.'])
