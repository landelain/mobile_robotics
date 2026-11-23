
from tdmclient import ClientAsync
from local_controller import LocalNavigationController


with ClientAsync() as client:

    async def main():
        print("Searching for Thymio...")

        node = await client.wait_for_node()
        await node.lock()

        controller = LocalNavigationController()

        try:
            # Keep motors stopped throughout the test
            await node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })

            # Subscribe to proximity sensor updates
            await node.watch(variables=True)
            await node.wait_for_variables({"prox.horizontal"})

            print("Connected to Thymio!")
            print("Testing FSM with real sensors.")
            print("Motors will remain stopped.\n")

            for i in range(40):
                await client.sleep(0.25)

                proximity = list(node.v.prox.horizontal)

                state, motors = controller.update(
                    proximity=proximity,
                    normal_command=(100, 100),
                    can_return=False
                )

                print(
                    f"Reading {i + 1}: "
                    f"Sensors={proximity} | "
                    f"State={state.name} | "
                    f"Motors={motors}"
                )

        finally:
            await node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })
            await node.unlock()

    client.run_async_program(main)
