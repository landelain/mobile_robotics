
from tdmclient import ClientAsync
from obstacle_avoidance import detect_obstacle, choose_direction

# Connect to Thymio Device Manager
with ClientAsync() as client:

    async def main():
        print("Searching for Thymio...")

        # Wait for the robot and lock it
        node = await client.wait_for_node()
        await node.lock()

        try:
            # Keep motors stopped
            await node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })

            # Enable variable updates
            await node.watch(variables=True)
            await node.wait_for_variables({"prox.horizontal"})

            print("Connected to Thymio!")
            print("Reading proximity sensors...\n")

            # Read 30 measurements
            for i in range(30):
                await client.sleep(0.5)

                proximity = list(node.v.prox.horizontal)

                print(f"Reading {i + 1}: {proximity}")

                # Test obstacle detection using real sensor readings
                if detect_obstacle(proximity):
                    direction = choose_direction(proximity)
                    print(f"  OBSTACLE DETECTED! Turn {direction}")
                else:
                    print("  Path clear")

        finally:
            # Stop motors and release robot
            await node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })
            await node.unlock()

    # Run asynchronous program
    client.run_async_program(main)
