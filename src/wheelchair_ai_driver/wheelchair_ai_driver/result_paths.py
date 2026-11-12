"""Select a local dataset without modifying any earlier experiment records.

Writers hold results/experiment.lock while selecting/creating a dataset.
The existing flat results directory remains a valid dataset for compatibility.
"""
import csv
import hashlib
import json
from pathlib import Path
import uuid

from .experiment_store import ExperimentStore, save_json


def fingerprint_id(fingerprint):
    encoded = json.dumps(fingerprint, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(encoded.encode()).hexdigest()


def _safe_path(root, directory):
    root = Path(root).resolve()
    base = (root / 'results').resolve()
    path = (root / directory).resolve()
    if not base.is_relative_to(root) or not path.is_relative_to(base):
        raise ValueError('Result directory must be inside this package\'s results/')
    return path


def active_results(root):
    root = Path(root).resolve()
    marker = root / 'runtime/active_results.json'
    if not marker.exists():
        return _safe_path(root, 'results')
    value = json.loads(marker.read_text())
    directory = value.get('directory')
    if not isinstance(directory, str) or Path(directory).is_absolute():
        raise ValueError('Invalid runtime/active_results.json; existing results were not changed')
    return _safe_path(root, directory)


def activate_results(root, directory):
    root = Path(root).resolve()
    directory = _safe_path(root, directory)
    save_json(root / 'runtime/active_results.json', {'directory': str(directory.relative_to(root))})


def _manifest(directory):
    try:
        return json.loads((directory / 'experiment.json').read_text())
    except (OSError, ValueError):
        return None


def _has_data(directory):
    if not directory.exists():
        return False
    for child in directory.iterdir():
        if child.name in ('experiment.lock', 'COLCON_IGNORE', 'environments'):
            continue
        if child.is_file() or child.is_symlink() or (child.is_dir() and any(child.iterdir())):
            return True
    return False


def select_results(root, fingerprint, create=False):
    """Reuse identical conditions or choose an empty folder; preview is read-only."""
    root = Path(root).resolve()
    base = _safe_path(root, 'results')
    identifier = fingerprint_id(fingerprint)
    environments = base / 'environments'
    candidates = [active_results(root), base]
    candidates.extend(sorted(environments.glob(identifier + '*')))
    directory = next((path for path in candidates if _manifest(path) == fingerprint), None)
    if directory is None:
        if _manifest(base) is None and not _has_data(base):
            directory = base
        else:
            directory = _safe_path(root, environments / identifier)
            if _manifest(directory) is not None or _has_data(directory):
                # Preserve incomplete/corrupt/unknown records as well as valid ones.
                directory = directory.with_name(identifier + '_' + uuid.uuid4().hex[:12])
    directory = _safe_path(root, directory)
    if create:
        directory.mkdir(parents=True, exist_ok=True)
        manifest = directory / 'experiment.json'
        if manifest.exists():
            if _manifest(directory) != fingerprint:
                raise ValueError('Result fingerprint changed while selecting a directory')
        elif _has_data(directory):
            raise ValueError('Refusing to assign a fingerprint to unknown existing results')
        else:
            save_json(manifest, fingerprint)
    return directory


def all_attempts(root):
    """Include old environments when stopping processes from an interrupted run."""
    base = _safe_path(root, 'results')
    return sorted([*base.glob('runs/*/attempt.json'),
                   *base.glob('environments/*/runs/*/attempt.json')])


def restore_csvs_if_needed(directory, fingerprint):
    """Recover the commit-before-CSV crash window, including fully met targets.

    Leave valid existing CSVs untouched; the unchanged store does all CSV formatting.
    Callers hold results/experiment.lock.
    """
    for mode in ('off', 'on'):
        numbers = sorted(int(path.stem.split('_')[-1])
                         for path in (directory / 'records').glob(mode + '_*.json'))
        try:
            with (directory / f'swan_{mode}.csv').open(encoding='utf-8-sig', newline='') as stream:
                rows = list(csv.reader(stream))
            columns = 5 if mode == 'on' and fingerprint.get('schema', 1) >= 2 else 4
            data_rows = rows[1:-1] if numbers else rows[1:]
            valid = (bool(rows) and all(len(row) == columns for row in rows)
                     and len(rows) == len(numbers) + (2 if numbers else 1)
                     and [int(row[0]) for row in data_rows] == numbers
                     and (not numbers or rows[-1][0] == '평균'))
        except (OSError, ValueError, IndexError):
            valid = False
        if not valid:
            ExperimentStore(directory, fingerprint)
            return
