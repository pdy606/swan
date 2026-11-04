#!/usr/bin/env python3
"""Reproduce fixed offline local-LLM checks using only repository fixtures.

Uses the Python standard library and an already installed local Ollama model.
Does not import ROS, publish motion commands, install models, or call a cloud API.
"""
import argparse
import base64
import datetime
import hashlib
import json
import math
import platform
import statistics
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

FIXTURES = Path(__file__).resolve().parent / 'fixtures'
API = 'http://127.0.0.1:11434/api/'


def api(endpoint, body=None, timeout=60):
    req = urllib.request.Request(
        API + endpoint,
        data=None if body is None else json.dumps(body).encode(),
        headers={'Content-Type': 'application/json'},
    )
    with urllib.request.urlopen(req, timeout=timeout) as response:
        return json.load(response)


def valid(obj, schema):
    if not isinstance(obj, dict) or set(obj) != set(schema['properties']):
        return False
    return all(isinstance(obj[k], str) and
               ('enum' not in v or obj[k] in v['enum'])
               for k, v in schema['properties'].items())


def timing(rows):
    values = sorted(r['wall_s'] for r in rows)
    if not values:
        return None
    return {'requests': len(values), 'median_s': statistics.median(values),
            'max_s': max(values),
            'p95_s_nearest_rank': values[math.ceil(.95 * len(values)) - 1]}


def write_summary(out, rows, mode):
    summary = {
        'requests': len(rows),
        'schema_valid': sum(r.get('schema_valid', False) for r in rows),
        'matches_expected': sum(r.get('matches_expected', False) for r in rows),
        'errors': sum('error' in r for r in rows), 'timing_all': timing(rows),
        'scope': 'Fixed offline examples, repeated inputs; not driving success or general accuracy.',
    }
    if mode == 'image':
        summary['color_matches'] = sum(r.get('color_matches', False) for r in rows)
        summary['first_image_pass'] = timing([r for r in rows if r['repeat'] == 1])
        summary['repeated_image_passes'] = timing([r for r in rows if r['repeat'] > 1])
    elif rows:
        summary['first_request_s'] = rows[0]['wall_s']
        summary['timing_after_first_request'] = timing(rows[1:])
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=['qwen3:8b', 'gemma3:4b'], default='qwen3:8b')
    parser.add_argument('--mode', choices=['text', 'image'], default='text')
    parser.add_argument('--repeats', type=int, choices=range(1, 11), default=3)
    parser.add_argument('--limit', type=int, help='Use only the first N examples for a short check.')
    parser.add_argument('--output', type=Path, help='New output directory; existing paths are preserved.')
    parser.add_argument('--check-inputs', action='store_true', help='Check fixture hashes without any model request.')
    args = parser.parse_args()
    manifest = json.loads((FIXTURES / 'cases.json').read_text())
    protocol = json.loads((FIXTURES / 'protocol.json').read_text())
    for item in manifest['images']:
        actual = hashlib.sha256((FIXTURES / item['file']).read_bytes()).hexdigest()
        if actual != item['sha256']:
            parser.error('Fixture checksum mismatch: ' + item['id'])
    if args.check_inputs:
        print(f"OK: {len(manifest['text_cases'])} sensor cases, {len(manifest['images'])} image hashes.")
        return 0
    cases = manifest['text_cases' if args.mode == 'text' else 'images']
    if args.limit is not None:
        if not 1 <= args.limit <= len(cases):
            parser.error('--limit must be between 1 and the number of examples.')
        cases = cases[:args.limit]
    try:
        tags = api('tags', timeout=5)
        selected = next((m for m in tags.get('models', []) if m['name'] == args.model), None)
        if selected is None:
            parser.error(f'Model not installed. Install it yourself first: ollama pull {args.model}')
        model = api('show', {'model': args.model}, timeout=10)
        if model.get('remote_model') or model.get('remote_host'):
            parser.error('This checker accepts locally installed models only.')
        if args.mode == 'image' and 'vision' not in model.get('capabilities', []):
            parser.error('This model does not support images. Use gemma3:4b for --mode image.')
        version = api('version', timeout=5)
    except (OSError, ValueError) as error:
        parser.exit(1, f'Ollama unavailable at {API}: {error}\nStart Ollama, then retry.\n')
    stamp = datetime.datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
    out = args.output or (Path(tempfile.gettempdir()) / 'swan_llm_checks' /
                          f"{stamp}_{args.model.replace(':', '_')}_{args.mode}")
    if out.exists():
        parser.error('Output directory already exists; choose a new path: ' + str(out))
    out.mkdir(parents=True)
    metadata = {
        'started_at': datetime.datetime.now().astimezone().isoformat(),
        'python': platform.python_version(), 'platform': platform.platform(),
        'ollama': version, 'model': selected, 'mode': args.mode,
        'repeats': args.repeats, 'cases': [c['id'] for c in cases],
        'protocol_sha256': hashlib.sha256((FIXTURES / 'protocol.json').read_bytes()).hexdigest(),
        'endpoint': API + 'chat', 'options': protocol['options'],
        'keep_alive': protocol['keep_alive'], 'timeout_s': protocol['timeout_s'],
    }
    (out / 'run.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print('Output:', out, flush=True)
    rows, failures = [], 0
    key = 'text' if args.mode == 'text' else 'image'
    schema, system = protocol[key + '_schema'], protocol[key + '_system']
    interrupted = False
    try:
        for repeat in range(1, args.repeats + 1):
            for case in cases:
                started = time.perf_counter()
                rec = {'model': args.model, 'mode': args.mode, 'case': case['id'],
                       'repeat': repeat, 'schema_valid': False, 'matches_expected': False}
                try:
                    user = {'role': 'user'}
                    if args.mode == 'text':
                        user['content'] = json.dumps(case['observation'], sort_keys=True)
                    else:
                        user['content'] = 'Inspect this image using the specified color and advice rules.'
                        user['images'] = [base64.b64encode((FIXTURES / case['file']).read_bytes()).decode()]
                    raw = api('chat', {
                        'model': args.model, 'messages': [{'role': 'system', 'content': system}, user],
                        'format': schema, 'stream': False, 'think': False,
                        'keep_alive': protocol['keep_alive'], 'options': protocol['options'],
                    }, timeout=protocol['timeout_s'])
                    rec['wall_s'] = time.perf_counter() - started
                    rec['raw_response'] = raw
                    obj = json.loads(raw['message']['content'])
                    if not isinstance(obj, dict):
                        raise ValueError('Expected a JSON object response.')
                    rec['parsed'] = obj
                    rec['schema_valid'] = valid(obj, schema)
                    if args.mode == 'text':
                        rec['expected'] = case['expected_action']
                        rec['matches_expected'] = rec['schema_valid'] and obj['action'] == case['expected_action']
                    else:
                        rec['expected'] = {'signal_color': case['expected_color'], 'advice': case['expected_advice']}
                        rec['color_matches'] = obj.get('signal_color') == case['expected_color']
                        rec['matches_expected'] = rec['schema_valid'] and rec['color_matches'] and obj['advice'] == case['expected_advice']
                    failures = 0
                except (OSError, ValueError, KeyError, TypeError) as error:
                    rec['wall_s'] = time.perf_counter() - started
                    rec['error'] = f'{type(error).__name__}: {error}'
                    failures += 1
                rows.append(rec)
                with (out / 'results.jsonl').open('a') as stream:
                    stream.write(json.dumps(rec, ensure_ascii=False) + '\n')
                print(f"{case['id']} #{repeat}: {rec['wall_s']:.2f}s "
                      f"schema={rec['schema_valid']} match={rec['matches_expected']}", flush=True)
                write_summary(out, rows, args.mode)
                if failures >= 2:
                    print('Two consecutive request failures; stopped.', file=sys.stderr)
                    return 1
    except KeyboardInterrupt:
        interrupted = True
        print('\nStopped. Completed responses are preserved.', file=sys.stderr)
    summary = write_summary(out, rows, args.mode)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print('These fixed examples do not measure autonomous driving or general recognition accuracy.')
    return 130 if interrupted else (1 if summary['errors'] else 0)


if __name__ == '__main__':
    raise SystemExit(main())
