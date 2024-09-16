# concert_mission_ws

Author: [Michael Terzer](michael.terzer@fraunhofer.it) Fraunhofer Italia 2023

## Description

This workspace includes the mission tasks modelling and parametrization tools developed within the CONCERT project. The  repository comes with a docker setup which inherits from the `fhi_mission_stack:release` and thus can access the robotic skill library from the fhi_mission_stack. Application specific skills for the CONCERT project are hosted in this workspace.

## How to run

The bt_action_server is the application that runs a ROS2 action server and recieves a BehaviorTreeDescriptor request to run a behavior tree through the bt_executor. Run the bt_action_server:

```
ros2 launch bt_action_server send_bt_action_server.launch.py
```

# Licence

concert_mission_ws is licensed under the terms of the Apache License 2.0. The project has recieved financial support by the Horizon 2020 EU Project [CONCERT](https://concertproject.eu/).

### Citation

```
@article{Terzer2024,
title = {A Facilitated Construction Robot Programming Approach using Building Information Modelling},
author = {Terzer, Michael and Flatscher, Tobit and Magri, Marco and Garbin, Simone and Emig, Julius and Giusti, Andrea},
journal = {10th International Conference on Control, Decision and Information Technologies CoDIT24},
year = {2024},
note = {to be published}
publisher={IEEE}
}
```


