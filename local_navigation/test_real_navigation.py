from tdmclient import ClientAsync
from local_controller import (
    LocalNavigationController,
    NavigationState
)
from obstacle_avoidance import (
    CORNER_CLEAR_THRESHOLD,
    CRITICAL_PROXIMITY_THRESHOLD,
    FRONT_BLOCKED_THRESHOLD,
    FRONT_CLEAR_THRESHOLD,
    SIDE_LOST_THRESHOLD,
    SIDE_VISIBLE_THRESHOLD,
    is_critical_proximity,
)

# Conservative settings for first physical test
MAX_SPEED = 200
FORWARD_SPEED = 150
CRITICAL_THRESHOLD = CRITICAL_PROXIMITY_THRESHOLD
DT = 0.1
MAX_STEPS = 600  # 60 seconds
MAX_TURN_STEPS = 15  # Limit continuous turning
MAX_CORNER_STEPS = 30  # Bound a corner arc if the sensor never clears

def limit_motor_commands(left, right, max_speed=50):
    """
    Scale both motor commands proportionally
    while preserving the steering ratio.
    """
    largest = max(abs(left), abs(right))

    if largest > max_speed:
        scale = max_speed / largest
        left *= scale
        right *= scale

    return int(round(left)), int(round(right))

with ClientAsync() as client:

    async def main():
        print("Searching for Thymio...")

        node = await client.wait_for_node()
        await node.lock()

        controller = LocalNavigationController()

        async def stop_motors():
            await node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })

        try:
            # Initially stop the robot
            await stop_motors()

            # Enable real sensor readings
            await node.watch(variables=True)
            await node.wait_for_variables({"prox.horizontal"})

            print("Connected!")
            print("Starting supervised navigation test.")

            turn_steps = 0
            corner_steps = 0
            avoidance_started = False

            for step in range(MAX_STEPS):

                await client.sleep(DT)

                proximity = list(node.v.prox.horizontal)
                print(f"Sensors: {proximity[:5]}")

                # Diagnostic: assess readiness for boundary following
                center_clear = proximity[2] <= FRONT_CLEAR_THRESHOLD
                side_visible = proximity[4] >= SIDE_VISIBLE_THRESHOLD

                # Conservative front-corner clearance
                front_clear = max(proximity[1:4]) <= CORNER_CLEAR_THRESHOLD

                if controller.state == NavigationState.AVOID_OBSTACLE:
                    print(
                        f"  DEBUG: center_clear={center_clear}, "
                        f"side_visible={side_visible}, "
                        f"front_clear={front_clear}"
                    )

                # Emergency proximity stop
                if is_critical_proximity(proximity, CRITICAL_THRESHOLD):
                    print("Critical proximity! STOP.")
                    break

                # Update your local navigation FSM
                state, motors = controller.update(
                    proximity=proximity,
                    normal_command=(
                        FORWARD_SPEED,
                        FORWARD_SPEED
                    ),
                    can_return=False
                )

                # Track whether avoidance has started
                if state != NavigationState.FOLLOW_PATH:
                    avoidance_started = True

                # Prevent indefinite turning
                if state == NavigationState.AVOID_OBSTACLE:
                    turn_steps += 1
                    if turn_steps >= MAX_TURN_STEPS:
                        print("Turning timeout. STOP.")
                        break
                else:
                    turn_steps = 0

                if state == NavigationState.CORNER_TURN:
                    corner_steps += 1
                    if corner_steps >= MAX_CORNER_STEPS:
                        print("Corner-turn timeout. STOP.")
                        break
                else:
                    corner_steps = 0

                # Don't drive blindly after losing the boundary
                if state == NavigationState.FOLLOW_BOUNDARY:
                    side_index = (
                        4 if controller.boundary_side == "right"
                        else 0
                    )

                    if proximity[side_index] < SIDE_LOST_THRESHOLD:
                        print("Boundary lost. STOP.")
                        break

                # The controller handles ordinary front/corner readings by
                # transitioning to AVOID_OBSTACLE or CORNER_TURN. Keep a
                # fail-safe invariant for an unexpected blocked center while
                # still allowing the controlled corner arc to proceed.
                if (
                    state == NavigationState.FOLLOW_BOUNDARY
                    and proximity[2] >= FRONT_BLOCKED_THRESHOLD
                ):
                    print("Unexpected center blockage. STOP.")
                    break


                # Safety check during boundary search
                if state == NavigationState.SEARCH_BOUNDARY:

                    if max(proximity[1:4]) >= FRONT_BLOCKED_THRESHOLD:
                        print("Obstacle during boundary search. STOP.")
                        break

                # Stop if boundary search has timed out
                if state == NavigationState.STOPPED:
                    print("Boundary not recovered. STOP.")
                    break

                # Limit wheel commands
                left, right = limit_motor_commands(
                    motors[0],
                    motors[1],
                    max_speed=MAX_SPEED
                )

                # Send motor commands to Thymio
                await node.set_variables({
                    "motor.left.target": [left],
                    "motor.right.target": [right]
                })

                
                side_index = (
                    4 if controller.boundary_side == "right"
                    else 0
                )

                print(
                    f"Step {step}: "
                    f"State={state.name} | "
                    f"Boundary={controller.boundary_side} | "
                    f"SideSensor={proximity[side_index]} | "
                    f"Front={proximity[1:4]} | "
                    f"Motors=({left}, {right})"
                )


            print("Navigation test finished.")

        finally:
            # Request motors to stop on normal exit or error
            await stop_motors()
            await node.unlock()
            print("Motors stopped.")

    client.run_async_program(main)
