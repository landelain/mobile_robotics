
from tdmclient import ClientAsync
from local_controller import (
    LocalNavigationController,
    NavigationState
)

# Conservative settings for first physical test
MAX_SPEED = 50
FORWARD_SPEED = 35
CRITICAL_THRESHOLD = 4500
DT = 0.1
MAX_STEPS = 80  # 5 seconds
MAX_TURN_STEPS = 15  # Limit continuous turning

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
            avoidance_started = False

            for step in range(MAX_STEPS):

                await client.sleep(DT)

                proximity = list(node.v.prox.horizontal)

                # Diagnostic: assess readiness for boundary following
                center_clear = proximity[2] < 1000
                side_visible = proximity[4] > 1000

                # Conservative front-corner clearance
                front_clear = max(proximity[1:4]) < 1500

                if controller.state == NavigationState.AVOID_OBSTACLE:
                    print(
                        f"  DEBUG: center_clear={center_clear}, "
                        f"side_visible={side_visible}, "
                        f"front_clear={front_clear}"
                    )

                # Emergency proximity stop
                if max(proximity[:5]) >= CRITICAL_THRESHOLD:
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

                # Don't drive blindly after losing the boundary
                if state == NavigationState.FOLLOW_BOUNDARY:
                    side_index = (
                        4 if controller.boundary_side == "right"
                        else 0
                    )

                    if proximity[side_index] < 300:
                        print("Boundary lost. STOP.")
                        break
                
                # Safety gate for forward motion during boundary following
                if state == NavigationState.FOLLOW_BOUNDARY:

                    # Central sensor must be clear
                    if proximity[2] >= 1000:
                        print("Center obstacle detected. STOP.")
                        break

                    # Check the front corner on the obstacle side
                    corner_index = (
                        3 if controller.boundary_side == "right"
                        else 1
                    )

                    if proximity[corner_index] >= 1500:
                        print("Front corner not clear. STOP.")
                        break


                # Safety check during boundary search
                if state == NavigationState.SEARCH_BOUNDARY:

                    if max(proximity[1:4]) >= 1500:
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
