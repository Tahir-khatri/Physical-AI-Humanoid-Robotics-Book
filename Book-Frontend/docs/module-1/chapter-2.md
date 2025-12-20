---
id: chapter-2
title: 'Chapter 2: The AI Bridge with rclpy'
sidebar_label: 'The AI Bridge (rclpy)'
---

# Chapter 2: The AI Bridge with rclpy

In Chapter 1, we built the mental model for a robot's nervous system using ROS 2's core components: Nodes, Topics, and Services. Now, it's time to create the bridge that connects our high-level AI logic to this nervous system. This bridge is **`rclpy`**, the official ROS 2 client library for Python.

`rclpy` allows us to write Python programs that can act as ROS 2 nodes, giving our AI the ability to see, hear, and act within the ROS 2 ecosystem. This chapter is a hands-on guide. We will write Python code to create nodes, publish data, and, most importantly, call services to command a robot's actions. By the end, you will have written a simple Python "AI agent" that can control a part of a simulated robot.

## Setting Up Your Python ROS 2 Environment

Before we write code, ensure you have a working ROS 2 environment. `rclpy` is a core part of any ROS 2 installation, so if you have ROS 2, you have `rclpy`. The key is to make sure your Python environment is aware of your ROS 2 installation.

This is typically done by "sourcing" the ROS 2 setup file in your terminal:

```bash
# For ROS 2 Humble Hawksbill
source /opt/ros/humble/setup.bash
```

This command sets up several environment variables, including `PYTHONPATH`, which tells Python where to find the `rclpy` libraries and all the ROS 2 message definitions. You must run this command in every new terminal you use for ROS 2 development.

Let's verify the setup. Open a Python interpreter in your sourced terminal and try to import `rclpy`:

```python
import rclpy
print("rclpy imported successfully!")
```

If this runs without an `ImportError`, your environment is correctly configured.

## Anatomy of a Python ROS 2 Node

Every `rclpy` application follows a basic structure. Let's break down the essential components of a minimal ROS 2 node written in Python.

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    """
    A simple ROS 2 Node that prints a message.
    """
    def __init__(self):
        # The super().__init__() call initializes the Node with a name.
        # This name must be unique in the ROS 2 graph.
        super().__init__('my_first_node')
        self.get_logger().info('Hello from my first ROS 2 node!')

def main(args=None):
    # 1. Initialize the rclpy library
    rclpy.init(args=args)

    # 2. Create an instance of our node
    node = MyFirstNode()

    try:
        # 3. "Spin" the node, which makes it available to the ROS 2 network
        # and allows it to process callbacks (like subscriptions or service requests).
        # We will spin the node until the user presses Ctrl+C.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This block is executed when the user hits Ctrl+C.
        pass
    finally:
        # 4. Cleanly destroy the node
        node.destroy_node()
        # 5. Shutdown the rclpy library
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Let's dissect the `main` function, which is the entry point of our application:

1.  **`rclpy.init()`**: This is the first thing you must do. It initializes the ROS 2 communication system, allowing the program to discover other nodes.
2.  **`node = MyFirstNode()`**: We create an instance of our custom `Node` class. The `super().__init__('node_name')` call inside our class constructor is critical; it registers our node with the ROS 2 system under the given name.
3.  **`rclpy.spin(node)`**: This is the workhorse function. Calling `spin` essentially hands control of our node over to `rclpy`. The function enters a loop, processing any incoming events for the node (like messages on a topic it's subscribed to) and calling the appropriate callback functions. The `spin` function will not return until the node is shut down.
4.  **`node.destroy_node()`**: When `spin` exits (e.g., due to Ctrl+C), we must explicitly destroy the node to free up its resources and unregister it from the network.
5.  **`rclpy.shutdown()`**: Finally, we shut down the entire `rclpy` context.

## Building the AI Bridge: A Service Client

Our goal is to create an AI agent that can command a robot to perform an action. As we learned in Chapter 1, the perfect tool for this is a **Service**. Our Python node will act as a **service client**.

Let's assume a simulated robot is running a node called `head_controller_node` that offers a service named `/head/set_pan_tilt_angle`. We will now write the Python code to call this service.

First, we need the exact definition of the service. Let's say it's a custom service type called `my_robot_interfaces/srv/SetPanTilt`.

-   **Request**: `float32 pan_angle`, `float32 tilt_angle`
-   **Response**: `bool success`

Here is the complete Python node that acts as a client for this service.

```python
import rclpy
from rclpy.node import Node

# We must import the service definition we want to use.
from my_robot_interfaces.srv import SetPanTilt

class AIActionClient(Node):
    """
    A node that acts as an AI agent, calling a service to move the robot's head.
    """
    def __init__(self):
        super().__init__('ai_action_client')
        
        # 1. Create the service client.
        # The first argument is the service type.
        # The second argument is the service name.
        self.client = self.create_client(SetPanTilt, '/head/set_pan_tilt_angle')

        # 2. Add a check to ensure the service is available.
        # It's good practice to wait for the server to be up before trying to call it.
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # 3. Create a request object.
        self.request = SetPanTilt.Request()

    def send_goal_position(self, pan, tilt):
        """
        Sends the goal position to the service server and waits for a response.
        """
        self.request.pan_angle = pan
        self.request.tilt_angle = tilt
        
        # 4. Asynchronously call the service.
        # This returns a "future" object, which is a placeholder for the eventual response.
        self.future = self.client.call_async(self.request)
        
        # We can attach a callback function to be executed when the future is completed.
        self.future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'Sent goal: pan={pan}, tilt={tilt}')

    def goal_response_callback(self, future):
        """
        This function is called when the service server sends back a response.
        """
        # 5. Get the response from the future.
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Goal reached successfully!')
            else:
                self.get_logger().error('Failed to reach goal.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    action_client_node = AIActionClient()

    # Our "AI" logic: command the head to look left.
    action_client_node.send_goal_position(pan=1.57, tilt=0.0)

    # We need to spin the node to allow the response to be received.
    # We will spin until the future is complete.
    while rclpy.ok() and not action_client_node.future.done():
        rclpy.spin_once(action_client_node, timeout_sec=0.1)

    # Clean shutdown
    action_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Deeper Dive into the Client Logic

1.  **`self.create_client(...)`**: This method, provided by the `Node` class, creates the client object. It needs to know the service type (for message structure) and the service name (to find the server).
2.  **`self.client.wait_for_service(...)`**: This is a crucial step for robustness. If you try to call a service whose server isn't running yet, the call will fail. This line creates a loop that politely waits for the server to appear on the network.
3.  **`SetPanTilt.Request()`**: We instantiate the `Request` part of our service definition. This gives us an object with the fields `pan_angle` and `tilt_angle` that we can fill.
4.  **`self.client.call_async(...)`**: This is the non-blocking way to call a service. It sends the request and immediately returns a `Future` object. Your program can continue doing other work while the service is being processed. The `add_done_callback` method is the elegant `rclpy` way to handle the response when it arrives, without blocking the main thread.
5.  **`future.result()`**: Inside the callback, this method retrieves the actual response message sent by the server. It's important to wrap this in a `try...except` block, as `result()` will raise an exception if the service call failed for any reason (e.g., the server node crashed).

In the `main` function, notice the new spin mechanism: `rclpy.spin_once()`. Instead of the blocking `rclpy.spin()`, `spin_once` checks for any pending events (like our service response), processes them, and then returns immediately. We use it in a `while` loop that continues until our future object is "done" (i.e., we've received the response). This is a common pattern for nodes that need to perform a single action and then exit.

## Summary and Next Steps

You have successfully built your first AI-to-robot bridge! You've learned the complete workflow for creating a Python ROS 2 node that acts as a service client. This pattern is fundamental to robotics programming: your AI and decision-making logic will almost always translate its intentions into actions by calling services offered by the robot's various hardware controllers.

You now have the power to command. But what are you commanding? A real humanoid robot is a complex assembly of dozens of links and joints. To command it effectively, and for ROS 2 to understand its structure, we need to describe its physical form.

That is the subject of our next and final chapter in this module, **"Humanoid Anatomy (URDF)"**, where we will learn to create a digital twin of our robot's body.
