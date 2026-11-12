"""Evaluation only: accepted Nav2 avoidance episodes, not replan goals.

The original SWAN publishes AVOIDING only after Nav2 accepts a goal.
WAITING_FOR_NAV / REPLANNING / VERIFYING_CLEAR keep that episode open.
COMPLETED and BLOCKED terminate it; a later accepted avoidance is new.
No evaluation observation is returned to either driving controller.
"""


class AvoidanceInterventions:
    END_STATES = frozenset({
        'COMPLETED', 'BLOCKED', 'RECOVERED', 'AUTONOMY_OFF',
        'AUTONOMY_ON', 'SLOPE_MANUAL', 'SLOPE_CLEAR',
    })

    def __init__(self, enabled=True):
        self.enabled = enabled
        self.count = 0
        self.active = False

    def observe(self, status):
        """Return True exactly when a new accepted avoidance starts."""
        status = status.strip().upper()
        if not self.enabled:
            return False
        if status in self.END_STATES:
            self.active = False
        elif status == 'AVOIDING' and not self.active:
            self.active = True
            self.count += 1
            return True
        return False
