---
title: Nodes, Topics, and Services
---

# Nodes, Topics, and Services

Understanding the fundamental communication patterns in ROS 2 is essential for developing distributed robotic systems. This module covers the three primary communication mechanisms: nodes, topics, and services.

## Nodes

Nodes are the fundamental building blocks of a ROS 2 system. Each node is a process that performs computation and communicates with other nodes.

### Creating a Node

In Python:
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Node Lifecycle

1. **Initialization**: Node registers with the ROS graph
2. **Execution**: Node performs its computational tasks
3. **Shutdown**: Node cleanly disconnects from the ROS graph

## Topics and Publishers/Subscribers

Topics enable asynchronous, many-to-many communication through a publish/subscribe pattern.

### Publisher

A publisher sends messages to a topic:
```python
publisher = node.create_publisher(String, 'topic_name', 10)
```

### Subscriber

A subscriber receives messages from a topic:
```python
subscriber = node.create_subscription(
    String, 'topic_name', callback_function, 10)
```

### Quality of Service (QoS)

QoS settings control the delivery guarantees for topics:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local
- **History**: Keep all vs. keep last N messages

## Services

Services provide synchronous request/response communication.

### Service Server
```python
service = node.create_service(AddTwoInts, 'add_two_ints', callback)
```

### Service Client
```python
client = node.create_client(AddTwoInts, 'add_two_ints')
```

## Best Practices

1. **Node Design**: Each node should have a single, well-defined responsibility
2. **Topic Naming**: Use descriptive, hierarchical names
3. **Message Types**: Use standard message types when possible
4. **Error Handling**: Implement proper error handling and recovery
5. **Resource Management**: Clean up resources properly during shutdown

## Learning Objectives

After completing this module, you will be able to:
- Create and manage ROS 2 nodes
- Implement publisher/subscriber patterns
- Design service-based communication
- Apply appropriate QoS settings for different use cases