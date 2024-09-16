# FHI Mission Stack

Author: [Michael Terzer](mailto:michael.terzer@fraunhofer.it) - Fraunhofer Italia (2023)

This stack includes a set of tools to manage robot skills, tasks and missions at Fraunhofer Italia. 

### Docker

The docker image of this stack inherits from the [behavior_tree_stack]((https://fhi-git.fraunhofer.it/behavior-tree/behavior_tree_stack)):release image. The behavior_tree_stack represents the base layer and provides tools to manage, run, log and handle behavior trees. For further information see the documentation of the behavior_tree_stack.

### Skills

The folder `skills` contains a list of submodules of robotic skills, beeing implemented as behavior tree nodes (according to the used library [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP.git)). The skills can be seen as an interface from the behavior tree to (mostly) internally developed libraries such as the [waypoint_manager](https://fhi-git.fraunhofer.it/rise-libs/ro/pl/moveit2/tools/waypoint_manager). The libraries normally run a ROS2 service or action server which is called by the behavior tree nodes. In order to be able to call these services and actions, the messages have to be included as well (if no debian package is available). These messages can be included under the folder [dependencies](https://fhi-git.fraunhofer.it/rise-libs/ro/mi/fhi_mission_stack/-/tree/main/dependencies).

### Behavior Tree

This stack represents a collection of behavior tree nodes which can be loaded from a behavior tree. The behavior_tree_stack includes the neccessary tools to load these skills once they have been specified in the behavior tree description file`.xml`. The behavior_tree_stack is also able to discover the skills automatically, once built and sourced in the same ROS2 workspace environment. 
So to create a behavior tree and use the skills of this stack, take a look at the bt_service_server or bt_action_server.

### Licence

concert_application_ws is licensed under the terms of the Apache License 2.0. The project has recieved financial support by the Horizon 2020 EU Project CONCERT.

### Citation

```
@article{Terzer2024,
 title = {A Facilitated Construction Robot Programming Approach using Building Information Modelling},
 author = {Terzer, Michael and Flatscher, Tobit and Magri, Marco and Garbin, Simone and Emig, Julius and Giusti, Andrea},
 journal = {10th International Conference on Control, Decision and Information Technologies CoDIT24},
 year = {2024},
 note = {to be published},
 publisher={IEEE}
}
```
