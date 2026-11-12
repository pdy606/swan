#!/usr/bin/env python3
"""MBRA-PG-Official inference utilities.

MBRA takes 6 ImageNet-normalized 96x96 frames (stacked as 18 channels)
and a goal pose [x, y, cos(yaw), sin(yaw)] with x, y divided by metric_spacing=0.8m.

User provides goal_xy in standard frame (x=forward, y=left, meters).

    nav = MBRAPGNavigator(device="cuda")
    traj, scores = nav.inference_trajectory(obs, goal_xy=np.array([5.0, 0.2]))
    vw = nav.inference_vw(obs, goal_xy=np.array([5.0, 0.2]))
"""

import os
import math
import numpy as np
import torch
import onnxruntime as ort
import yaml
from torchvision import transforms

MODEL_DIR = os.path.dirname(os.path.abspath(__file__))
ONNX_PATH = os.path.join(MODEL_DIR, "mbra.onnx")
INFO_PATH = os.path.join(MODEL_DIR, "model_info.yaml")
METRIC_SPACING = 0.8  # waypoint normalization factor

IMAGENET_TRANSFORM = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225]),
])
IMG_SIZE = 96


class PDController:
    MAX_V = 2.5
    MAX_W = 0.65

    def __init__(self):
        self.last_v = None

    def reset(self):
        self.last_v = None

    def __call__(self, waypoints, dt=1.0):
        EPS = 1e-6
        idx = min(2, waypoints.shape[1] - 1)
        dx = waypoints[:, idx, 0]
        dyr = waypoints[:, -1, 1]
        dxr = waypoints[:, -1, 0]
        v = dx / dt
        w = (torch.atan2(dyr, dxr.abs().clamp(min=EPS))
             * dxr.sign() / dt)
        near_zero = dx.abs() < EPS
        v = torch.where(near_zero, torch.zeros_like(v), v)
        w = torch.where(near_zero, dyr.sign() * (math.pi / 20.0), w)
        if self.last_v is not None:
            v = v.clamp(self.last_v - 0.5, self.last_v + 0.4)
        v = v.clamp(-self.MAX_V, self.MAX_V)
        w = w.clamp(-self.MAX_W, self.MAX_W)
        self.last_v = v
        return v, w


class MBRAPGNavigator:
    """MBRA point-goal navigator.

    All user-facing coordinates: x=forward, y=left, meters.
    """

    context_size = 6
    multimodal = False

    def __init__(
        self,
        onnx_path: str = ONNX_PATH,
        device: str = "cuda",
        max_v: float = 2.5,
        max_w: float = 0.65,
        dt: float = 2.5,
    ):
        self.device = device
        self.dt = dt

        ort.set_default_logger_severity(3)
        providers = [
            ("CUDAExecutionProvider",
             {"arena_extend_strategy": "kSameAsRequested"}),
            "CPUExecutionProvider",
        ]
        self._session = ort.InferenceSession(onnx_path, providers=providers)
        with open(INFO_PATH, "r") as f:
            self._info = yaml.safe_load(f)
        self._controller = PDController()
        self._controller.MAX_V = max_v
        self._controller.MAX_W = max_w

    def reset(self):
        self._controller.reset()

    def _preprocess_images(self, obs_rgb):
        """obs_rgb: (B, C, 3, H, W) float32 [0,1] → (B, C*3, 96, 96) ImageNet-norm."""
        from PIL import Image as PILImage
        B, C = obs_rgb.shape[:2]
        frames = []
        for b in range(B):
            ch_list = []
            for c in range(C):
                frame = (obs_rgb[b, c].transpose(1, 2, 0) * 255).astype(np.uint8)
                pil = PILImage.fromarray(frame).resize((IMG_SIZE, IMG_SIZE))
                ch_list.append(IMAGENET_TRANSFORM(pil).numpy())  # (3, 96, 96)
            frames.append(np.concatenate(ch_list, axis=0))  # (C*3, 96, 96)
        return np.stack(frames)  # (B, 18, 96, 96)

    @staticmethod
    def _goal_to_pose(goal_xy):
        """Convert standard goal [x_fwd, y_left] (meters) → MBRA goal_pose [4].

        MBRA expects [x/spacing, y/spacing, cos(angle), sin(angle)].
        """
        x, y = float(goal_xy[0]), float(goal_xy[1])
        angle = math.atan2(y, x)
        return np.array([
            x / METRIC_SPACING,
            y / METRIC_SPACING,
            math.cos(angle),
            math.sin(angle),
        ], dtype=np.float32)

    def inference_trajectory(self, obs, goal_xy=None):
        """Run model → (trajectory, scores).

        Args:
            obs: (B, 6, 3, H, W) float32 in [0,1]. 6 frames.
            goal_xy: (2,) goal in standard frame [x_fwd, y_left] meters.
                     If None, uses [5.0, 0.0].

        Returns:
            trajectory: np.ndarray (B, 1, 8, 2) waypoints [x, y] meters
            scores:     np.ndarray (B, 1)
        """
        if isinstance(obs, torch.Tensor):
            obs_np = obs.cpu().numpy()
        else:
            obs_np = np.asarray(obs, dtype=np.float32)
        B = obs_np.shape[0]

        obs_norm = self._preprocess_images(obs_np)

        if goal_xy is None:
            goal_xy = np.array([np.random.uniform(2.0, 10.0), np.random.uniform(-2.0, 2.0)])
        goal_pose = self._goal_to_pose(goal_xy)
        goal_batch = np.tile(goal_pose, (B, 1))

        out = self._session.run(None, {
            "obs_images": obs_norm,
            "goal_pose": goal_batch,
        })
        wp_raw = out[0]  # (B, 8, 4) — [x, y, cos(yaw), sin(yaw)]

        # Extract x, y and un-normalize
        wp_xy = wp_raw[:, :, :2] * METRIC_SPACING  # (B, 8, 2) meters
        trajectory = wp_xy[:, np.newaxis]  # (B, 1, 8, 2)
        scores = np.ones((B, 1), dtype=np.float32)
        return trajectory, scores

    def inference_vw(self, obs, goal_xy=None):
        """Run model → (v, w).

        Returns:
            vw: torch.Tensor (B, 2)
        """
        trajectory, _ = self.inference_trajectory(obs, goal_xy)
        best_traj = trajectory[:, 0]
        waypoints = torch.from_numpy(best_traj).float().to(self.device)
        v, w = self._controller(waypoints, dt=self.dt)
        return torch.stack([v, w], dim=1), best_traj
