---
publish: false
title: 
description: 
permalink: 
aliases: 
tags: 
created: 2025-05-05
modified: 2025-05-05
---

Basics:
- <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html>
- <https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html>
- <https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html>
- <https://design.ros2.org/articles/roslaunch.html>

Useful structure of launch file (see examples of my own projects)

How to deal with paths

Pointers on how to define launch arguments

Add some common patterns such as including another launch file and adding within GroupAction (see my older projects for examples).

Advanced:
- <https://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html>

## Intuition: how do launch files work?

Launch files work by defining a set of _actions to perform_.

Note that your Python launch files do not actually start the nodes when they run: the Python code you write is used to create the _launch actions_, and the launch actions themselves will start the ROS setup.

You cannot "start" launch actions using regular Python code, e.g. using a loop or if statement. Also, the values of variables do not always work the way you expect. Finally, and most importantly, substitutions are only evaluated once the launch description is started, _not_ when the Python code to create the launch description is executed.

### Control flow patterns in launch files

| Action                           | Regular Python                | Launch equivalent                                                       |
| -------------------------------- | ----------------------------- | ----------------------------------------------------------------------- |
| Conditionally execute some code  | `if <condition>:`             | `condition=IfCondition(<substitution>)`                                 |
| Conditionally skip some code     | `if not <condition>`:<br><br> | `condition=UnlessCondition(<substitution>)`                             |
| Define command line arguments    | …                           | `DeclareLaunchArgument()`                                               |
| Call with command line arguments | python3 -m <module> arg1 arg2 | ros2 launch <pkg> <launch.py> arg1:=value arg2:=value                   |
| Import code from other module    | import …                    | `IncludeLaunchDescription()` (ideally inside a `GroupAction()`)         |
| Remap topics                     | -                             | `remappings={"old": new"}.items()` or `SetRemap` (inside `GroupAction`) |
| Wait for some time               | time.sleep()                  | TimerAction or RosTimer                                                 |
| Wait until other stuff is done   | -                             | RegisterEventHandler(event_handler=OnProcessExit())                     |
| Execute arbitrary function       | call function                 | OpaqueFunction                                                          |

Other cool features:

- Event handlers
- Respawning
- Required
