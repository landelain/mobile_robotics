
from tdmclient import ClientAsync


with ClientAsync() as client:

    async def main():
        print("Searching for Thymio...")

        node = await client.wait_for_node()
        await node.lock()

        try:
            # Stop motors before starting
            await node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })

            print("Connected! Testing a slow left turn.")

            # Rotate left slowly
            await node.set_variables({
                "motor.left.target": [-40],
                "motor.right.target": [40]
            })

            # Turn for 0.5 seconds
            await client.sleep(1.0)

        finally:
            # Always stop both motors
            await node.set_variables({
                "motor.left.target": [0],
                "motor.right.target": [0]
            })

            await node.unlock()

        print("Motor test completed!")


    client.run_async_program(main)
