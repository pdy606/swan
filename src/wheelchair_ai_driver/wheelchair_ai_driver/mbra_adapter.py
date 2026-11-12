"""Official MBRA image/goal conventions with an inspected ONNX input contract.

No ROS, velocity controller, detector, costmap, or extra sensor input is used.
The downloaded upstream inference.py remains an unmodified reference.
"""
import base64
import math
from pathlib import Path
import time

import numpy as np
import onnxruntime as ort
from PIL import Image

CONTEXT_SIZE = 6
IMAGE_SIZE = 96
METRIC_SPACING = 0.8
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)[:, None, None]
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)[:, None, None]


def goal_to_pose(goal_xy):
    goal = np.asarray(goal_xy, dtype=np.float32)
    if goal.shape != (2,) or not np.isfinite(goal).all():
        raise ValueError('goal_xy must be two finite values: forward, left (meters)')
    x, y = map(float, goal)
    angle = math.atan2(y, x)
    return np.array([[x / METRIC_SPACING, y / METRIC_SPACING,
                      math.cos(angle), math.sin(angle)]], dtype=np.float32)


def decode_frame(frame):
    """Decode an 8-bit ROS Image, respecting encoding and row padding."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    encoding = frame['encoding'].lower()
    if encoding not in channels:
        raise ValueError(f'Unsupported image encoding: {encoding}')
    width, height, step = (int(frame[key]) for key in ('width', 'height', 'step'))
    count = channels[encoding]
    if min(width, height) <= 0 or step < width * count or height * step > 32 * 1024 * 1024:
        raise ValueError('Invalid or oversized image dimensions/step')
    data = base64.b64decode(frame['data'], validate=True)
    if len(data) != height * step:
        raise ValueError('Image byte count differs from height * step')
    pixels = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    pixels = pixels[:, :width * count].reshape(height, width, count)
    if encoding == 'mono8':
        pixels = np.repeat(pixels, 3, axis=2)
    elif encoding in ('bgr8', 'bgra8'):
        pixels = pixels[:, :, [2, 1, 0]]
    else:
        pixels = pixels[:, :, :3]
    return np.ascontiguousarray(pixels)


def preprocess_frames(frames):
    if len(frames) != CONTEXT_SIZE:
        raise ValueError('MBRA requires six chronological frames')
    prepared = []
    for frame in frames:
        rgb = decode_frame(frame)
        # Upstream uses PIL RGB resize's default (bicubic), then ToTensor/Normalize.
        resized = Image.fromarray(rgb).resize((IMAGE_SIZE, IMAGE_SIZE))
        chw = np.asarray(resized, dtype=np.float32).transpose(2, 0, 1) / np.float32(255)
        prepared.append((chw - MEAN) / STD)
    return np.ascontiguousarray(np.stack(prepared)[None], dtype=np.float32)


def _require_shape(actual, expected, name):
    if len(actual) != len(expected):
        raise ValueError(f'{name}: unexpected rank: {actual}')
    for received, wanted in zip(actual, expected):
        if isinstance(received, int) and received != wanted:
            raise ValueError(f'{name}: unsupported shape {actual}, expected {expected}')


class MbraAdapter:
    def __init__(self, model_path, provider='cpu', threads=2):
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(model_path)
        available = ort.get_available_providers()
        if provider not in ('cpu', 'cuda'):
            raise ValueError('provider must be cpu or cuda')
        if provider == 'cuda' and 'CUDAExecutionProvider' not in available:
            raise RuntimeError(f'CUDA requested but unavailable; providers={available}')
        providers = ['CPUExecutionProvider'] if provider == 'cpu' else [
            'CUDAExecutionProvider', 'CPUExecutionProvider']
        options = ort.SessionOptions()
        options.intra_op_num_threads = int(threads)
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self.session = ort.InferenceSession(str(model_path), sess_options=options,
                                            providers=providers)
        if provider == 'cuda' and 'CUDAExecutionProvider' not in self.session.get_providers():
            raise RuntimeError('Requested CUDA provider failed to initialize')
        inputs = {item.name: item for item in self.session.get_inputs()}
        if set(inputs) != {'obs_images', 'goal_pose'}:
            raise ValueError(f'Unexpected model input names: {list(inputs)}')
        if any(item.type != 'tensor(float)' for item in inputs.values()):
            raise ValueError('MBRA inputs must be float32 tensors')
        self.image_shape = inputs['obs_images'].shape
        self.image_rank = len(self.image_shape)
        expected = [1, 18, 96, 96] if self.image_rank == 4 else [1, 6, 3, 96, 96]
        _require_shape(self.image_shape, expected, 'obs_images')
        _require_shape(inputs['goal_pose'].shape, [1, 4], 'goal_pose')
        outputs = self.session.get_outputs()
        if len(outputs) != 1 or outputs[0].type != 'tensor(float)':
            raise ValueError('Expected one float32 waypoint output')
        _require_shape(outputs[0].shape, [1, 8, 4], outputs[0].name)
        self.info = {
            'model_path': str(model_path.resolve()),
            'onnxruntime_version': ort.__version__,
            'available_providers': available,
            'active_providers': self.session.get_providers(),
            'inputs': [{'name': item.name, 'shape': item.shape, 'type': item.type}
                       for item in self.session.get_inputs()],
            'outputs': [{'name': item.name, 'shape': item.shape, 'type': item.type}
                        for item in outputs],
            'image_rank': self.image_rank,
            'goal_convention': '[forward/0.8, left/0.8, cos(bearing), sin(bearing)]',
        }

    def infer(self, frames, goal_xy):
        start = time.perf_counter()
        context = preprocess_frames(frames)
        images = context.reshape(1, 18, 96, 96) if self.image_rank == 4 else context
        goal = goal_to_pose(goal_xy)
        before_session = time.perf_counter()
        raw = self.session.run(None, {'obs_images': images, 'goal_pose': goal})[0]
        after_session = time.perf_counter()
        if raw.shape != (1, 8, 4) or not np.isfinite(raw).all():
            raise ValueError(f'Invalid waypoint output: shape={raw.shape}')
        xy = raw[0, :, :2] * np.float32(METRIC_SPACING)
        return {
            'trajectory_xy_m': xy.tolist(),
            'raw_waypoints': raw[0].tolist(),
            'goal_pose': goal[0].tolist(),
            'input_shapes': {'obs_images': list(images.shape), 'goal_pose': list(goal.shape)},
            'preprocess_ms': (before_session - start) * 1000,
            'onnx_ms': (after_session - before_session) * 1000,
            'total_ms': (after_session - start) * 1000,
        }
