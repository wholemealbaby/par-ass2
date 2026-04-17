# 🛠️ How to use the ExplorationController

## 1. Initialization

When adding the controller to a node, ensure the node is added to a MultiThreadedExecutor in your main.py.

# In your main node file
```python
from rclpy.executors import MultiThreadedExecutor

def main():
    # ... init node ...
    executor = MultiThreadedExecutor()
    executor.add_node(my_node)
    executor.spin()
```

## 2. Calling Commands

All movement commands (start, stop, resume, teleop) are now asynchronous. You must use the await keyword when calling them from within the node.

| Action  | Code                           | Description                                           |
|---------|--------------------------------|-------------------------------------------------------|
| Start   | await controller.start()       | Resets frontiers and begins exploration.              |
| Stop    | await controller.stop()        | Halts current navigation and exploration logic.       |
| Resume  | await controller.resume()      | Picks up exploration from the last known state.       |
| Teleop  | await controller.teleop()      | Switches mode to manual override.                      |

## 3. Writing New Callbacks

If you create a new callback that needs to call a controller function, the callback itself must be async.
```python
async def my_new_callback(self, msg):
    self.get_logger().info("Button pressed!")
    # You MUST use await here!
    success = await self.exploration_controller.stop()
```