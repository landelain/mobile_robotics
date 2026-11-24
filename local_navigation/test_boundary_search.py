
from local_controller import (
    LocalNavigationController,
    NavigationState
)

controller = LocalNavigationController()

# Start as if an obstacle has already been avoided
controller.state = NavigationState.FOLLOW_BOUNDARY
controller.boundary_side = "right"

tests = [
    ("Following box", [0, 0, 0, 0, 2000, 0, 0]),
    ("Box lost", [0, 0, 0, 0, 0, 0, 0]),
    ("Searching", [0, 0, 0, 0, 0, 0, 0]),
    ("Box found", [0, 0, 0, 0, 1900, 0, 0]),
]

for label, sensors in tests:
    state, motors = controller.update(
        proximity=sensors,
        can_return=False
    )
    print(f"{label}: {state.name}, motors={motors}")

# Test search timeout independently
controller = LocalNavigationController()
controller.state = NavigationState.SEARCH_BOUNDARY
controller.boundary_side = "right"

for i in range(10):
    state, motors = controller.update([0] * 7)

print(f"Timeout: {state.name}, motors={motors}")

assert state == NavigationState.STOPPED
assert motors == (0, 0)
