---
publish: true
title: 
description: 
permalink: 
aliases: 
tags: 
created: 2025-05-05
modified: 2025-05-05
---

## Status

**Not done, priority: high**

- Strategies for git are described.
- How to organize packages must still be described.
- How to initialize a package must still be described & examples must be written.
- **Important**: how to handle dependencies, reuse documentation from:
	- <https://bitbucket.org/ctw-bw/ros-2-guidelines/src/main/README.md>
	- <https://bitbucket.org/ctw-bw/ros-2-guidelines/src/main/Managing%20non-ROS%20Python%20dependencies.md>

## Stuff to write about

- Git inside each package?
- How to organise ROS 2 packages (what goes in what package, how much stuff, etc.)
    - Give example of `nakama_handeye`?
- Generating package templates

Links to official ROS 2 documentation

- <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>
- <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html>
- <https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html>
- <https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html>
- <https://docs.ros.org/en/humble/How-To-Guides/Documenting-a-ROS-2-Package.html>
- <https://docs.ros.org/en/humble/How-To-Guides/Releasing/Releasing-a-Package.html>
- <https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html>
- <https://ros.org/reps/rep-0144.html>
- <https://ros.org/reps/rep-2005.html>

**TODO: Look at the ROS 2 cookbook**: <https://github.com/mikeferguson/ros2_cookbook/tree/main>

Advanced topics:
- `ament_cmake`: <https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html>
- <https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/cmake.md#safely-defaulting-to-release-build>

## Creating a package

## Organising packages

TODO: Read <https://automaticaddison.com/naming-and-organizing-packages-in-large-ros-2-projects/>

TODO: Read <https://automaticaddison.com/organizing-files-and-folders-inside-a-ros-2-package/>

As shown in the [documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#packages-in-a-workspace), packages should be placed in the workspace's `src/` folder. Packages can also be placed in subfolder of `src/`: Colcon will traverse every folder until it finds a `package.xml` file. This means you can have the following workspace structure:

```text
workspace_folder/
    src/
        package_1/
            package.xml
            ...

        subfolder/
            package_2/
                package.xml
                ...
            
            package_2/
                package.xml
                ...

        really/
            deep/
                folder/
                    structure/
                        package_4/
                            package.xml
                            ...

```

These subfolders also allow us to have multiple Git repositories inside the workspace, where each Git repository hold one or more packages.

### ROS packages in Git repositories

There are two main strategies for using Git with ROS packages:

1. Group multiple (tightly) related ROS packages into a Git repository.
     - It is **easier** to have many packages inside a single Git repository.
        - Easier to share with other people (single `git clone` gets them all the required software).
        - Clear place for documentation (a single README entry point).
        - Easier to keep up to date with changes made (single `git pull` gets you all the latest changes).

2. Have a single ROS package as a Git repository.
    - It is **more maintainable** to _intelligently_ split packages in multiple Git repositories
        - _This only applies if you have put thought in the organisation of the packages_: see [[#Choosing how to organise ROS packages in Git]].
        - It forces a degree of independence between your packages.
        - It makes more sense for packages which require many more OR much fewer updates than other packages.
        - It allows people who use your packages to pick and choose which parts to use.

An example of both strategies:

1. `franka_ros2` (<https://github.com/frankaemika/franka_description>) groups many ROS packages (e.g. `franka_bringup`, `franka_example_controllers`, etc.) into one Git repository.
    1. Why is this strategy used?
        1. The packages are tightly coupled and depend on the same versions of dependencies (in this case, `libfranka`).
        2. It allows people to get all Franka-related ROS 2 stuff with one set of install instructions.
    2. In fact, there is even a package inside the Git repository called `franka_ros2` which only contains the `package.xml` file and lists all other packages in the Git repository as dependencies. That way, other people can list the ROS package `franka_ros2` as a dependency and automatically depend on all other packages in the `franka_ros2` Git repository. This is called a _meta package_.
2. `franka_description` (<https://github.com/frankaemika/franka_description>) is a Git repository which is only a single ROS package.
    1. Why is this strategy used?
        1. This package is only rarely updated (the robot dimensions do not change).
        2. This package is reused by multiple other projects (such as `franka_ros` and `franka_ros2`).

#### Choosing how to organise ROS packages in Git

Which strategy works best for your case depends on how you want other people to use your package(s). Here are some things to keep in mind:

- If one of the packages requires specific install instructions which are not required by the other packages, you should split them into different Git repositories.
    - E.g. the package requires a Python virtual environment, the Ubuntu real-time kernel, or requires specific dependencies / connections which are only present on one PC.
    - This prevents the package from causing errors from using `colcon build` in the wrong way alongside other packages, such as missing dependencies.
    - Make sure to include clear documentation and clear links to which packages are related!
    - Example: `nakama_handeye` (<https://bitbucket.org/ctw-bw/nakama_handeye>) requires a PC with Zed and connected to the camera, while `nakama_handeye_robot_bringup` (<https://bitbucket.org/ctw-bw/nakama_handeye_robot_bringup>) requires a different PC with the Real-Time kernel and attached to the Franka robot.

- If one of the packages is much more likely to be reused by other people, you can consider moving that to a different Git repository.
    - E.g. interfaces packages (containing ROS message / service / action definitions) can often be moved to a different Git repository. If other people want to interact with your nodes, they typically only need the interfaces and not the nodes themselves.

- You'll often start with multiple packages inside a single Git repository and eventually split some of them into a different repository later. That is fine.
