---
title: Cognitive Planning
---

# Cognitive Planning

Cognitive planning in humanoid robotics involves creating intelligent systems that can reason about tasks, plan complex sequences of actions, and adapt to changing environments. This module explores implementing cognitive planning systems that can convert high-level goals into executable robotic behaviors using Large Language Models (LLMs).

## Introduction to Cognitive Planning

Cognitive planning encompasses:
- **Task Decomposition**: Breaking complex goals into subtasks
- **Reasoning**: Using knowledge to determine appropriate actions
- **Planning**: Creating sequences of actions to achieve goals
- **Adaptation**: Adjusting plans based on new information
- **Learning**: Improving planning through experience

## Architecture of Cognitive Planning Systems

### Cognitive Planning Pipeline
1. **Goal Understanding**: Interpret high-level goals
2. **Knowledge Retrieval**: Access relevant information
3. **Plan Generation**: Create action sequences
4. **Plan Execution**: Execute planned actions
5. **Monitoring**: Track execution progress
6. **Adaptation**: Modify plans based on feedback

### System Components
- **Goal Parser**: Understands natural language goals
- **Knowledge Base**: Stores world models and capabilities
- **Planner**: Generates action sequences
- **Executor**: Carries out actions
- **Monitor**: Tracks execution state

## Knowledge Representation for Planning

### Ontology-Based Knowledge
Representing knowledge about the world, objects, and actions:

```python
class KnowledgeBase:
    def __init__(self):
        self.objects = {}
        self.locations = {}
        self.actions = {}
        self.capabilities = {}
        self.rules = []

    def add_object(self, obj_id, properties):
        self.objects[obj_id] = {
            'type': properties.get('type'),
            'location': properties.get('location'),
            'properties': properties.get('properties', {}),
            'relations': properties.get('relations', {})
        }

    def add_location(self, loc_id, properties):
        self.locations[loc_id] = properties

    def add_action(self, action_id, definition):
        self.actions[action_id] = definition

    def get_related_objects(self, obj_id, relation):
        if obj_id in self.objects:
            obj = self.objects[obj_id]
            if relation in obj['relations']:
                return obj['relations'][relation]
        return []
```

### Semantic World Models
```python
class SemanticWorldModel:
    def __init__(self):
        self.spatial_relations = {}
        self.functional_relations = {}
        self.temporal_relations = {}

    def add_spatial_relation(self, obj1, obj2, relation):
        # e.g., "cup" is "on" "table"
        if obj1 not in self.spatial_relations:
            self.spatial_relations[obj1] = {}
        self.spatial_relations[obj1][obj2] = relation

    def is_reachable(self, robot_pos, target_pos):
        # Check if target is within robot's reach
        distance = self.calculate_distance(robot_pos, target_pos)
        return distance <= self.robot_reach_radius

    def get_path_constraints(self, start, goal):
        # Get constraints for path planning
        return {
            'traversable': self.is_traversable(goal),
            'safe': self.is_safe(goal),
            'reachable': self.is_reachable(goal)
        }
```

## Large Language Models for Planning

### Using LLMs for Task Planning
LLMs can help decompose high-level goals into executable actions:

```python
import openai
import json

class LLMPlanner:
    def __init__(self, api_key):
        openai.api_key = api_key
        self.client = openai.OpenAI()

    def generate_plan(self, goal, context, available_actions):
        prompt = f"""
        You are a robot task planner. Given the goal, current context, and available actions,
        create a step-by-step plan to achieve the goal.

        Goal: {goal}
        Context: {context}
        Available Actions: {available_actions}

        Respond with a JSON object containing:
        - "plan": list of action objects with "action", "parameters", and "reasoning"
        - "estimated_steps": number of steps required
        - "potential_issues": list of potential issues

        Ensure each action is executable by the robot and consider the physical constraints.
        """

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.3
        )

        try:
            plan_data = json.loads(response.choices[0].message.content)
            return plan_data
        except json.JSONDecodeError:
            # Handle case where response isn't valid JSON
            return self.parse_plan_from_text(response.choices[0].message.content)
```

### Action Validation
```python
class PlanValidator:
    def __init__(self, knowledge_base):
        self.kb = knowledge_base

    def validate_plan(self, plan):
        for step in plan['plan']:
            action = step['action']
            params = step['parameters']

            # Check if action is available
            if action not in self.kb.actions:
                return False, f"Action '{action}' is not available"

            # Check preconditions
            preconditions = self.kb.actions[action].get('preconditions', [])
            for precondition in preconditions:
                if not self.check_precondition(precondition, params):
                    return False, f"Precondition failed: {precondition}"

            # Check physical feasibility
            if not self.check_physical_feasibility(action, params):
                return False, f"Action not physically feasible: {action}"

        return True, "Plan is valid"

    def check_precondition(self, precondition, params):
        # Implement precondition checking logic
        # This would check if conditions are met before action execution
        pass

    def check_physical_feasibility(self, action, params):
        # Check if action is physically possible
        # Consider robot kinematics, workspace, etc.
        pass
```

## Hierarchical Task Networks (HTN)

### HTN Planning for Complex Tasks
Breaking down complex tasks into manageable subtasks:

```python
class HTNPlanner:
    def __init__(self):
        self.tasks = {}
        self.methods = {}

    def add_task(self, task_name, description):
        self.tasks[task_name] = description

    def add_method(self, task, method_name, decomposition):
        """
        Add a method to decompose a task
        decomposition: list of subtasks to achieve the main task
        """
        if task not in self.methods:
            self.methods[task] = []
        self.methods[task].append({
            'name': method_name,
            'decomposition': decomposition
        })

    def decompose_task(self, task, context):
        """
        Decompose a high-level task into subtasks
        """
        if task not in self.methods:
            # Primitive task - no further decomposition needed
            return [task]

        # Try each method for the task
        for method in self.methods[task]:
            if self.is_applicable(method, context):
                subtasks = []
                for subtask in method['decomposition']:
                    subtasks.extend(self.decompose_task(subtask, context))
                return subtasks

        return []  # No applicable method found

    def is_applicable(self, method, context):
        # Check if method is applicable given current context
        # This would check preconditions, available resources, etc.
        return True
```

## Integration with ROS 2

### Cognitive Planning Node
Creating a ROS 2 node for cognitive planning:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')

        # Publishers and subscribers
        self.plan_pub = self.create_publisher(String, 'execution_plan', 10)
        self.status_pub = self.create_publisher(String, 'planning_status', 10)
        self.goal_sub = self.create_subscription(
            String, 'high_level_goal', self.goal_callback, 10)

        # Initialize planners
        self.llm_planner = LLMPlanner(api_key=self.get_parameter('openai_api_key').value)
        self.knowledge_base = KnowledgeBase()
        self.validator = PlanValidator(self.knowledge_base)

        self.get_logger().info('Cognitive Planner Node initialized')

    def goal_callback(self, msg):
        goal = msg.data
        self.get_logger().info(f'Received high-level goal: {goal}')

        # Get current context
        context = self.get_current_context()

        # Generate plan using LLM
        plan = self.llm_planner.generate_plan(
            goal=goal,
            context=context,
            available_actions=self.get_available_actions()
        )

        # Validate plan
        is_valid, reason = self.validator.validate_plan(plan)
        if is_valid:
            # Execute plan
            self.execute_plan(plan)
        else:
            self.get_logger().error(f'Invalid plan: {reason}')
            self.publish_status(f'Planning failed: {reason}')

    def get_current_context(self):
        # Gather current state information
        context = {
            'robot_pose': self.get_robot_pose(),
            'detected_objects': self.get_detected_objects(),
            'available_tools': self.get_available_tools(),
            'environment_map': self.get_environment_map()
        }
        return context

    def execute_plan(self, plan):
        # Publish plan for execution
        plan_msg = String()
        plan_msg.data = json.dumps(plan)
        self.plan_pub.publish(plan_msg)

        # Monitor execution
        self.monitor_execution(plan)
```

## Plan Execution and Monitoring

### Execution Monitoring
Tracking plan execution and handling failures:

```python
class PlanExecutionMonitor:
    def __init__(self, robot_interface):
        self.robot_interface = robot_interface
        self.current_plan = None
        self.current_step = 0
        self.execution_history = []

    def execute_plan(self, plan):
        self.current_plan = plan
        self.current_step = 0

        for i, step in enumerate(plan['plan']):
            self.current_step = i
            success = self.execute_step(step)

            if not success:
                # Handle failure
                recovery_plan = self.generate_recovery_plan(step, plan)
                if recovery_plan:
                    self.execute_plan(recovery_plan)
                else:
                    # Plan failed completely
                    return False

        return True

    def execute_step(self, step):
        action = step['action']
        params = step['parameters']

        try:
            # Execute the action
            result = self.robot_interface.execute_action(action, params)

            # Log execution
            self.execution_history.append({
                'step': step,
                'result': result,
                'timestamp': time.time()
            })

            return result['success']
        except Exception as e:
            self.get_logger().error(f'Error executing step: {e}')
            return False

    def generate_recovery_plan(self, failed_step, original_plan):
        # Generate a plan to recover from failure
        # This could involve:
        # - Replanning around the failure
        # - Using alternative actions
        # - Requesting human assistance
        pass
```

## Learning and Adaptation

### Plan Learning from Experience
Improving planning through experience:

```python
class PlanLearner:
    def __init__(self):
        self.execution_records = []
        self.success_patterns = {}
        self.failure_patterns = {}

    def record_execution(self, plan, context, outcome, execution_trace):
        record = {
            'plan': plan,
            'context': context,
            'outcome': outcome,
            'trace': execution_trace,
            'timestamp': time.time()
        }
        self.execution_records.append(record)

        # Update patterns based on outcome
        if outcome['success']:
            self.update_success_patterns(plan, context)
        else:
            self.update_failure_patterns(plan, context, outcome['reason'])

    def update_success_patterns(self, plan, context):
        # Learn patterns that lead to success
        for step in plan['plan']:
            action = step['action']
            if action not in self.success_patterns:
                self.success_patterns[action] = []
            self.success_patterns[action].append(context)

    def update_failure_patterns(self, plan, context, reason):
        # Learn patterns that lead to failure
        for step in plan['plan']:
            action = step['action']
            if action not in self.failure_patterns:
                self.failure_patterns[action] = {}
            if reason not in self.failure_patterns[action]:
                self.failure_patterns[action][reason] = []
            self.failure_patterns[action][reason].append(context)

    def suggest_improved_plan(self, goal, context):
        # Use learned patterns to suggest better plans
        # This could involve avoiding actions that commonly fail
        # in similar contexts or preferring actions that commonly succeed
        pass
```

## Cognitive Architectures

### Subsumption Architecture Integration
Combining high-level planning with reactive behaviors:

```python
class CognitiveArchitecture:
    def __init__(self):
        self.high_level_planner = LLMPlanner()
        self.reactive_layer = ReactiveBehaviorLayer()
        self.arbitrator = PlanArbitrator()

    def process_command(self, goal, context):
        # Generate high-level plan
        high_level_plan = self.high_level_planner.generate_plan(goal, context)

        # Generate reactive behaviors for immediate needs
        reactive_behaviors = self.reactive_layer.get_behaviors(context)

        # Arbitrate between planned and reactive actions
        action = self.arbitrator.select_action(high_level_plan, reactive_behaviors, context)

        return action

class PlanArbitrator:
    def __init__(self):
        self.priority_rules = [
            # Safety > Goal achievement > Efficiency
            self.check_safety_constraints,
            self.check_immediate_danger,
            self.check_opportunity,
            self.follow_plan
        ]

    def select_action(self, plan, behaviors, context):
        for rule in self.priority_rules:
            action = rule(plan, behaviors, context)
            if action is not None:
                return action
        return None
```

## Natural Language to ROS 2 Actions

### Converting Natural Language to ROS 2 Commands
Mapping high-level language to specific ROS 2 actions:

```python
class NaturalLanguageMapper:
    def __init__(self):
        self.action_templates = {
            'navigation': {
                'keywords': ['go to', 'move to', 'navigate to', 'walk to', 'travel to'],
                'template': self.create_navigation_goal,
                'topic': '/navigate_to_pose'
            },
            'manipulation': {
                'keywords': ['pick up', 'grasp', 'take', 'get', 'place', 'put'],
                'template': self.create_manipulation_command,
                'topic': '/manipulation/command'
            },
            'interaction': {
                'keywords': ['talk to', 'greet', 'introduce', 'communicate'],
                'template': self.create_interaction_command,
                'topic': '/interaction/command'
            }
        }

    def map_language_to_action(self, text_command):
        for action_type, config in self.action_templates.items():
            for keyword in config['keywords']:
                if keyword in text_command.lower():
                    # Extract parameters and create ROS message
                    ros_msg = config['template'](text_command)
                    return {
                        'topic': config['topic'],
                        'message': ros_msg,
                        'action_type': action_type
                    }

        return None

    def create_navigation_goal(self, command):
        # Extract location from command
        location = self.extract_location(command)
        pose = self.get_pose_for_location(location)

        from geometry_msgs.msg import PoseStamped
        goal = PoseStamped()
        goal.pose = pose
        return goal

    def extract_location(self, command):
        # Use NLP to extract location entities
        # This would involve named entity recognition
        pass
```

## Performance Evaluation

### Metrics for Cognitive Planning
Evaluating the effectiveness of cognitive planning systems:

1. **Plan Success Rate**: Percentage of plans that achieve goals
2. **Plan Efficiency**: Steps taken vs. optimal steps
3. **Adaptability**: Ability to handle unexpected situations
4. **Response Time**: Time to generate and execute plans
5. **Resource Usage**: Computational and energy requirements

### Benchmarking Scenarios
- **Simple Tasks**: Basic navigation and manipulation
- **Complex Tasks**: Multi-step tasks with dependencies
- **Dynamic Environments**: Changing conditions
- **Failure Recovery**: Handling plan failures

## Best Practices

1. **Validation**: Always validate plans before execution
2. **Safety**: Prioritize safety in plan generation
3. **Modularity**: Design modular planning components
4. **Monitoring**: Continuously monitor plan execution
5. **Learning**: Incorporate learning from experience

## Learning Objectives

After completing this module, you will be able to:
- Design cognitive planning systems for humanoid robots
- Implement knowledge representation for planning
- Integrate LLMs for task decomposition
- Create hierarchical task networks
- Monitor and adapt plan execution
- Convert natural language to robotic actions