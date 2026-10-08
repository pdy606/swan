"""Small, bounded action contract for an experimental AI wheelchair operator."""

import json
import math
from urllib.request import Request, urlopen


ACTIONS = {
    'stop': (0.0, 0.0),
    'forward': (0.12, 0.0),
    'left': (0.06, 0.35),
    'right': (0.06, -0.35),
}

SCHEMA = {
    'type': 'object',
    'properties': {
        'action': {'type': 'string', 'enum': list(ACTIONS)},
        'reason': {'type': 'string'},
    },
    'required': ['action', 'reason'],
    'additionalProperties': False,
}

SYSTEM = (
    'You are a substitute human operator driving a wheelchair in a Gazebo market. '
    'Choose only one of stop, forward, left, right for the next short interval. '
    'The goal heading error is positive when the goal is to the left. '
    'Prefer a modest forward motion when aligned and clear. '
    'Turn toward the goal if heading error exceeds 0.2 rad. '
    'Stop if front clearance is below 1.0 m or the goal is reached. '
    'Return only the requested JSON.'
)


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_clearances(ranges, angle_min, angle_increment, range_max):
    """Return front, left and right minimum LiDAR distances in metres."""
    bins = {'front': [], 'left': [], 'right': []}
    maximum = range_max if math.isfinite(range_max) and range_max > 0 else 10.0
    for i, distance in enumerate(ranges):
        if math.isnan(distance) or distance <= 0:
            continue
        distance = min(distance, maximum)
        angle = normalize(angle_min + i * angle_increment)
        if abs(angle) <= math.radians(25):
            bins['front'].append(distance)
        elif math.radians(25) < angle <= math.radians(100):
            bins['left'].append(distance)
        elif -math.radians(100) <= angle < -math.radians(25):
            bins['right'].append(distance)
    return {name: min(values) if values else 0.0
            for name, values in bins.items()}


def bounded_action(proposed, clearances, goal_distance):
    """Apply a common stop gate and fixed speed limits independent of the model."""
    if goal_distance <= 0.30:
        return 'stop', 'goal_reached'
    if clearances['front'] < 1.0:
        return 'stop', 'front_clearance'
    if proposed not in ACTIONS:
        return 'stop', 'invalid_action'
    if proposed == 'left' and clearances['left'] < 0.55:
        return 'stop', 'left_clearance'
    if proposed == 'right' and clearances['right'] < 0.55:
        return 'stop', 'right_clearance'
    return proposed, 'accepted'


def ollama_action(endpoint, model, observation, timeout=10):
    """Ask a local Ollama API for one discrete operator action."""
    payload = {
        'model': model,
        'messages': [
            {'role': 'system', 'content': SYSTEM},
            {'role': 'user', 'content': json.dumps(observation, sort_keys=True)},
        ],
        'format': SCHEMA,
        'stream': False,
        'think': False,
        'options': {'temperature': 0, 'num_predict': 80},
    }
    request = Request(endpoint, data=json.dumps(payload).encode('utf-8'),
                      headers={'Content-Type': 'application/json'}, method='POST')
    with urlopen(request, timeout=timeout) as response:
        result = json.load(response)
    message = json.loads(result['message']['content'])
    action = message['action']
    if action not in ACTIONS:
        raise ValueError(f'Unknown AI action: {action!r}')
    return action, str(message.get('reason', ''))[:160], result
