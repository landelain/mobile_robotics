
from tdmclient import ClientAsync
from obstacle_avoidance import detect_obstacle, choose_direction


TURN_SPEED = 60
TURN_DURATION = 0.7


with ClientAsync() as client:

    async def main():
        print("Searching for Thymio...")

        node = await client.wait_for_node()
        await node.lock()

        try:
            # Stop motors before testing
            await node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })

            await node.watch(variables=True)
            await node.wait_for_variables({"prox.horizontal"})

            print("Connected to Thymio!")
            print("Place an obstacle in front of the robot.")
            print("Waiting for obstacle detection...\n")

            # Wait for an obstacle, then turn once
            for i in range(60):
                await client.sleep(0.2)

                proximity = list(node.v.prox.horizontal)

                if detect_obstacle(proximity):
                    direction = choose_direction(proximity)

                    print(f"Obstacle detected: {proximity}")
                    print(f"Turning {direction}!")

                    if direction == "left":
                        left_motor = -TURN_SPEED
                        right_motor = TURN_SPEED
                    else:
                        left_motor = TURN_SPEED
                        right_motor = -TURN_SPEED

                    await node.set_variables({
                        "motor.left.target": [left_motor],
                        "motor.right.target": [right_motor]
                    })

                    await client.sleep(TURN_DURATION)

                    # Stop immediately after turn
                    await node.set_variables({
                        "motor.left.target": [0],
                        "motor.right.target": [0]
                    })

                    print("Turn completed. Motors stopped.")
                    break

            else:
                print("No obstacle detected during test.")

        finally:
            # Always request zero motor speeds
            await node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })
            await node.unlock()

    client.run_async_program(main)
