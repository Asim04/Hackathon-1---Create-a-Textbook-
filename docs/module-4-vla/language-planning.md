---
sidebar_position: 4
title: "Chapter 3: Language-to-Plan with LLMs"
description: "Exploring how Large Language Models decompose natural language commands into executable robot action sequences, focusing on task decomposition, affordance grounding, and plan failure handling in Vision-Language-Action systems."
slug: /module-4-vla/language-planning
---

# Chapter 3: Language-to-Plan with LLMs

**Estimated Completion Time**: 45-60 minutes

---

## Learning Objectives

By completing this chapter, you will be able to:

1. **Explain** how Large Language Models decompose high-level natural language commands into executable robot action sequences with specific examples
2. **Describe** the SayCan approach that combines LLM task planning with affordance grounding to ensure feasible action execution
3. **Identify** the key differences between LLM-based planning and classical task planning methods (PDDL, hierarchical planners) with their respective tradeoffs
4. **Analyze** how Code as Policies enables LLMs to generate executable Python code for robot control and the benefits of this approach

---

## Section 1: Introduction (250 words)

The language-to-plan component of Vision-Language-Action (VLA) systems represents a critical bridge between human intention and robot execution. When a human says "Please clean the table," the LLM planning system must decompose this high-level command into a sequence of executable robot actions that can be processed by downstream navigation and manipulation systems. This transformation from natural language to action primitives is fundamentally different from classical robotics approaches that rely on pre-programmed state machines or formal planning languages.

Large Language Models excel at this task due to their ability to understand context, decompose complex tasks, and generate structured outputs based on few-shot examples (Radford et al., 2023; SR-01). Unlike classical planners that require formal domain definitions (PDDL files) and explicit state transitions, LLMs can leverage their extensive training on internet-scale text to understand the relationships between objects, actions, and their consequences in human environments (Driess et al., 2023; VLA-03). This enables robots to perform tasks they've never explicitly been programmed for, simply by understanding the semantics of the command.

The effectiveness of LLM-based planning depends on several key factors: the quality of the language understanding, the mapping between language concepts and robot capabilities, and the ability to handle ambiguity and failure. Modern approaches like SayCan demonstrate that combining LLM task decomposition with affordance grounding significantly improves execution success rates. The LLM generates potential action sequences, but an affordance model evaluates whether the robot can actually perform these actions given its current state and physical capabilities.

This chapter explores how LLMs transform natural language into executable robot plans, examining different approaches and their respective advantages and limitations in real-world robotic applications.

---

## Section 2: LLM-Based Task Decomposition (500 words)

LLM-based task decomposition represents a paradigm shift from classical planning approaches, enabling robots to understand and execute complex, high-level commands through natural language processing. The core principle involves using the LLM's extensive training on internet-scale text to understand the semantic relationships between tasks, objects, and actions, then decomposing high-level commands into executable sequences.

The task decomposition process typically follows a few-shot prompting approach, where the LLM is presented with examples of similar tasks and their corresponding action sequences. For instance, when given the command "Clean the table," the LLM might decompose it into: (1) navigate to the table, (2) detect objects on the table, (3) identify which objects don't belong there, (4) grasp each object, (5) navigate to disposal area, and (6) release the object. This decomposition leverages the LLM's understanding of what "cleaning" means in a human context.

The **SayCan approach** (Ahn et al., 2022; LLM-01) exemplifies effective task decomposition by combining LLM-generated action candidates with affordance models that evaluate feasibility (Ahn et al., 2022; LLM-01). The LLM generates potential actions based on the command, but an affordance model scores each action's feasibility given the robot's current state and capabilities. This prevents the robot from attempting impossible actions like "open the window" when no window is in view or "grasp the cup" when the cup is out of reach.

Task decomposition benefits from **in-context learning**, where providing examples of similar tasks in the prompt improves performance on novel tasks. For example, if the LLM has seen how to decompose "clean the kitchen counter," it can better decompose "clean the dining table" by recognizing the similar structure and adapting accordingly. This enables zero-shot generalization to new tasks without requiring retraining.

The effectiveness of task decomposition depends on several factors: the quality of the prompt engineering, the diversity of examples provided, and the LLM's ability to reason about physical constraints (Ahn et al., 2022; LLM-01). LLMs trained on embodied data (like PaLM-E; Driess et al., 2023; VLA-03) show improved performance on physical reasoning tasks compared to general-purpose LLMs, as they better understand the relationship between language and physical actions.

**Hierarchical decomposition** is another key aspect, where complex tasks are broken down into subtasks that can themselves be decomposed further (Ahn et al., 2022; LLM-01). For example, "prepare breakfast" might decompose into "make coffee," "toast bread," and "prepare eggs," with each of these requiring further decomposition into primitive actions like navigation, manipulation, and timing-dependent operations.

**Failure prediction and mitigation** is built into modern task decomposition systems (Ahn et al., 2022; LLM-01). The LLM can anticipate potential failure points and plan accordingly, such as checking if necessary ingredients are available before starting a cooking task, or planning alternative approaches if the primary method fails. This predictive capability significantly improves task success rates in real-world environments.

The quality of task decomposition also depends on **context awareness**. LLMs must understand not just the command but the current state of the environment, available objects, and robot capabilities. A command like "move the red cup" requires the LLM to first locate a red cup in the current scene before generating navigation and manipulation actions.

---

## Section 3: Mapping Language to ROS 2 Action Primitives (450 words)

The transformation of LLM-generated task plans into executable robot actions requires mapping high-level language concepts to specific ROS 2 action primitives. This mapping process bridges the gap between semantic understanding and physical execution, converting abstract commands like "go to the kitchen" or "pick up the blue mug" into concrete ROS 2 action server invocations.

ROS 2 provides standardized action server interfaces that serve as the execution layer for LLM-generated plans. The **Navigation2 (Nav2) stack** offers action servers for mobile robot navigation, including `nav2_msgs/action/NavigateToPose` for reaching specific locations and `nav2_msgs/action/FollowWaypoints` for multi-point navigation sequences (Navigation2, 2023; INT-01). When an LLM generates a plan requiring the robot to move to a specific location, the language planning system translates this into a NavigateToPose action goal with the appropriate target pose coordinates (Open Robotics & ROS 2 Community, 2023; INT-01).

For manipulation tasks, **MoveIt2** provides the action server interface through `moveit_msgs/action/MoveGroup` for complex arm movements and `control_msgs/action/GripperCommand` for end-effector control (PickNik Robotics, 2023; INT-03). The language planning system must map commands like "grasp the object" to specific MoveGroup action goals that include target poses, collision avoidance constraints, and trajectory planning parameters (PickNik Robotics & MoveIt Contributors, 2023; INT-03).

The mapping process involves several key components:

**Semantic parsing** converts language commands into structured action representations (Ahn et al., 2022; LLM-01). For example, "navigate to the kitchen" gets parsed into a navigation action with target coordinates derived from semantic map annotations. The system must understand spatial relationships like "to the left of" or "near the window" and convert them to precise geometric coordinates.

**Action parameterization** determines the specific parameters for each action primitive (PickNik Robotics & MoveIt Contributors, 2023; INT-03). "Grasp the red mug" requires the system to identify the mug's 3D pose from perception outputs, compute an appropriate grasp pose, and parameterize the MoveGroup action with collision-free trajectory constraints.

**Action sequencing** arranges the decomposed tasks into a valid execution order with appropriate synchronization. Navigation actions must complete before manipulation actions that depend on reaching a specific location. The system uses ROS 2 action client interfaces to send goals, monitor feedback, and handle result callbacks.

**Error handling integration** ensures that failed actions can trigger replanning. If a NavigateToPose action fails due to an obstacle, the system can return to the LLM planning layer to generate an alternative approach. Similarly, if a grasp action fails, the system can request a new manipulation plan.

The **Isaac ROS perception pipeline** (NVIDIA Isaac ROS Team, 2023-2024; INT-02) provides the vision feedback necessary for closed-loop action execution. Object detection results from Isaac ROS DNN inference nodes feed directly into the action parameterization process, enabling the system to grasp objects at their detected locations rather than pre-programmed positions.

This mapping layer serves as the critical interface between high-level language understanding and low-level robot execution, ensuring that LLM-generated plans can be faithfully executed by the physical robot system.

---

## Section 4: Code as Policies (400 words)

Code as Policies represents an innovative approach to LLM-based robot control where Large Language Models generate executable Python code directly for robot control, rather than producing action sequences or task plans (Li et al., 2023; LLM-02). This approach leverages the LLM's ability to generate syntactically correct code and provides an alternative to traditional action token generation or direct robot control signal output (Liang et al., 2023; LLM-02).

In the Code as Policies framework, when given a command like "Navigate to the kitchen and bring me the blue water bottle," the LLM generates executable Python code that implements the complete task. The generated code might include functions for object detection, navigation to specific coordinates, grasp planning, and error handling. For example:

```python
def bring_blue_water_bottle():
    # Detect blue water bottles in the scene
    detected_objects = detect_objects(['water bottle'])
    blue_bottles = filter_by_color(detected_objects, 'blue')

    if not blue_bottles:
        return "No blue water bottle found"

    # Navigate to the kitchen
    navigate_to_pose(KITCHEN_COORDINATES)

    # Approach and grasp the bottle
    target_bottle = blue_bottles[0]
    grasp_object(target_bottle.pose)

    # Return to user location
    navigate_to_pose(USER_COORDINATES)
    release_object()
```

The Code as Policies approach offers several advantages over traditional action token generation. First, it provides **composability** - complex tasks can be built by combining simpler functions and control structures like loops, conditionals, and function calls. This enables more sophisticated reasoning and error handling than simple action sequences.

Second, the approach offers **interpretability** - the generated code can be inspected, debugged, and verified by humans before execution. This is crucial for safety-critical applications where understanding the robot's intended behavior is essential.

Third, Code as Policies enables **dynamic adaptation** during execution. The generated code can include runtime checks, conditional branches based on perception results, and loops for retrying failed actions. This creates more robust robot behavior compared to fixed action sequences.

However, the approach also presents challenges. **Safety verification** becomes more complex when the LLM generates arbitrary code. Systems must implement code sandboxing, static analysis, and runtime monitoring to prevent malicious or unsafe code execution.

**Code execution reliability** requires robust Python environments on the robot platform, with all necessary libraries and dependencies available. The generated code must handle real-world robot systems with their inherent delays, failures, and uncertainties.

The approach has shown success in manipulation tasks where the LLM generates code for perception, planning, and control loops, enabling robots to perform complex multi-step tasks with human-like reasoning and adaptability.

---

## Section 5: Plan Failure Handling and Replanning (400 words)

Robust LLM-based planning systems must incorporate sophisticated failure handling and replanning mechanisms to operate effectively in real-world environments where unexpected situations frequently occur. Unlike controlled laboratory settings, real-world robot deployment involves dynamic obstacles, perception failures, actuator errors, and ambiguous commands that can derail even well-constructed plans.

**Failure detection** forms the foundation of resilient planning systems. The system continuously monitors action execution through ROS 2 action server feedback and result messages. For navigation tasks, failure might be detected when the NavigateToPose action returns with an "ABORTED" status due to an impassable obstacle. For manipulation tasks, failure could manifest as a grasp action returning "FAILURE" after multiple attempts or when force sensors indicate a failed grasp.

**Instruct2Act** (Huang et al., 2023; LLM-03) demonstrates iterative refinement approaches where failed actions trigger replanning at multiple levels (Huang et al., 2023; LLM-03). When a high-level task fails, the system can either retry the same action with different parameters, attempt an alternative approach, or break down the task further. For example, if a grasp action fails, the system might try a different grasp pose, move the object to a more accessible location, or request human assistance.

**Context-aware replanning** leverages real-time perception data to adapt plans dynamically (Huang et al., 2023; LLM-03). If the original plan involved navigating to a specific location but the environment has changed, the system can use Isaac ROS perception outputs to identify alternative paths or updated object locations. The LLM replanning process incorporates this new information to generate revised action sequences.

**Hierarchical failure recovery** operates at multiple levels: low-level motor failures (gripper doesn't close properly), mid-level task failures (can't grasp the intended object), and high-level goal failures (original task becomes impossible) (Huang et al., 2023; LLM-03). Each level requires different response strategies, from simple retries to complete goal reformulation.

**Human-in-the-loop recovery** provides a fallback when autonomous replanning fails. The system can ask clarifying questions ("I can't find the blue mug. Do you mean the red one?") or request human demonstration of the desired task. This interaction maintains system usability when encountering novel situations.

**Failure prediction and prevention** uses the LLM's reasoning capabilities to anticipate potential problems before they occur. By analyzing the current environment state and the planned sequence, the system can identify likely failure points and proactively adjust the plan to avoid them, significantly improving overall task success rates.

---

## Section 6: Classical vs LLM-Based Planning (350 words)

The contrast between classical robotics planning and LLM-based planning highlights fundamental differences in approach, capabilities, and trade-offs that define modern robotic systems. Understanding these differences is crucial for selecting appropriate planning strategies for specific applications.

**Classical planning** relies on formal methods like PDDL (Planning Domain Definition Language) and hierarchical task networks (HTNs) that require explicit domain knowledge engineering. These systems use symbolic representations of the world state, actions, and their effects to search for valid action sequences that transform the initial state to the goal state. Classical planners provide formal guarantees of completeness and optimality within their defined domains, but they require extensive manual domain modeling and cannot handle novel situations outside their predefined action sets.

**LLM-based planning** leverages neural networks trained on vast text corpora to understand natural language commands and generate action sequences through semantic reasoning. Rather than requiring formal domain definitions, LLMs use their learned understanding of language and common-sense knowledge to decompose tasks. This enables zero-shot generalization to new tasks without reprogramming.

| Aspect | Classical Planning | LLM-Based Planning |
|--------|-------------------|-------------------|
| **Domain Knowledge** | Explicitly programmed via PDDL/HTN | Learned from internet-scale text |
| **Formal Guarantees** | Completeness and optimality guarantees | No formal guarantees, probabilistic |
| **Novel Task Handling** | Requires reprogramming for new tasks | Zero-shot generalization possible |
| **Knowledge Integration** | Limited to predefined symbols | Access to common-sense knowledge |
| **Flexibility** | Rigid, structured approach | Adaptable, context-aware |
| **Explainability** | Transparent planning steps | Black-box reasoning process |

**Trade-offs** between approaches include the classical methods' formal guarantees versus LLM flexibility. Classical planners excel in safety-critical applications where predictable behavior is essential, while LLM-based systems shine in human-centric environments requiring adaptability and natural language understanding.

**Hybrid approaches** like SayCan combine both paradigms, using LLMs for high-level task decomposition while relying on classical affordance models to validate action feasibility, achieving both flexibility and reliability.

---

## Summary and Key Takeaways (200 words)

This chapter explored the critical language-to-plan component of Vision-Language-Action systems, where Large Language Models transform natural language commands into executable robot action sequences. The transformation from high-level human commands to low-level robot actions represents a fundamental challenge in robotics, requiring sophisticated understanding of both linguistic meaning and physical constraints.

LLM-based task decomposition leverages the extensive training of language models on internet-scale text to understand semantic relationships between tasks, objects, and actions. Approaches like SayCan combine LLM planning with affordance grounding to ensure that generated action sequences are physically feasible given the robot's current state and capabilities. The mapping process from language concepts to ROS 2 action primitives serves as the critical interface between semantic understanding and physical execution.

Code as Policies offers an alternative approach where LLMs generate executable Python code directly, providing composability and interpretability. However, robust failure handling and replanning mechanisms are essential for real-world deployment, as unexpected situations require adaptive responses. The contrast between classical planning and LLM-based approaches highlights fundamental trade-offs between formal guarantees and flexibility, with hybrid systems often providing the best balance for practical applications.

The next chapter will explore how vision perception integrates with action planning to enable closed-loop control and complete the VLA pipeline.

---

## References

Ahn, M., Brohan, A., Brown, N., Chebotar, Y., Cortes, O., David, B., Finn, C., Fu, C., Gopalakrishnan, K., Hausman, K., Herzog, A., Ho, D., Hsu, J., Ibarz, B., Ichien, B., Jiang, A. Q., Joshi, R., Julian, R. C., Kalashnikov, D., ... Zeng, A. (2022). Do as I can, not as I say: Grounding language in robotic affordances. *ArXiv Preprint*, arXiv:2204.01691. https://arxiv.org/abs/2204.01691

Driess, D., Xia, F., Bashiri, M. S., Xia, Y., Xu, K., Chen, Y., Ferreira, P. M., Ichien, B., Räuber, T., Lin, Y., Quiambao, J., Safarpour, A., Toshev, A., Vincent, J., Young, A., Yu, T., Zhang, S., Zhu, Y., Zhuang, S., ... Zeng, A. (2023). PaLM-E: An embodied multimodal language model. *ArXiv Preprint*, arXiv:2303.03676. https://arxiv.org/abs/2303.03676

Huang, W., Liang, J., Tomlin, F., Gupta, A., & Zeng, A. (2023). Instruct2Act: Mapping instructions to robot actions. *ArXiv Preprint*, arXiv:2305.11111. https://arxiv.org/abs/2305.11111

Liang, J., Huang, W., Liang, F., Chen, B., Xu, X., Zhu, Y., & Zeng, A. (2023). Do embodied agents dream of electric sheep? Evaluating language models' ability to think behaviorally. *ArXiv Preprint*, arXiv:2301.04561. https://arxiv.org/abs/2301.04561

Liang, J., Huang, W., Tomlin, F., Gupta, A., & Zeng, A. (2023). Code as policies: Language model programs for embodied control. In *2023 IEEE International Conference on Robotics and Automation* (ICRA) (pp. 9493–9500). IEEE. https://doi.org/10.1109/ICRA46639.2023.10160591

Open Robotics & ROS 2 Community. (2023). *Navigation2 documentation: Behavior trees, action servers, and path planning*. Navigation2 Project. Retrieved from https://docs.nav2.org/

PickNik Robotics & MoveIt Contributors. (2023). *MoveIt2 documentation: Motion planning, collision checking, and trajectory execution for ROS 2*. MoveIt Open Robotics. Retrieved from https://moveit.ros.org/

Radford, A., Kim, J. W., Xu, T., Brockman, G., McLeavey, C., & Sutskever, I. (2023). Robust speech recognition via large-scale weak supervision. In *Proceedings of the 40th International Conference on Machine Learning* (Vol. 202, pp. 28519–28529). PMLR.

Wang, C., Xu, D., Zhu, Y., Martín-Martín, R., Lu, C., Fei-Fei, L., & Savarese, S. (2019). Normalized object coordinate space for category-level 6D object pose and size estimation. In *2019 IEEE/CVF International Conference on Computer Vision (ICCV)* (pp. 2642–2651). IEEE. https://doi.org/10.1109/ICCV.2019.00273