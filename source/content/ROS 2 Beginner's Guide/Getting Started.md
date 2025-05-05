The ROS 2 Beginner's Guide will walk you through ROS 2 and its capabilities.

ROS 2 is immense and easy to get lost in, even with the extensive documentation and tutorials on the official website (<https://docs.ros.org/en/humble/index.html>). Even so, the official tutorials teach some concepts and practises which are not ideal for developing robotic applications at Nakama (and in general in our unbiased opinion). These guidelines will point those things out and explain how we want to approach them instead.

## Suggested reading order

We start with a set of suggested readings in a suggested order.

**I recommend taking at least a week to work through this list** because it is a lot of information. Also **plan some other tasks in the meantime**, so this list is not a full-time task. Taking more time makes it stick a lot better & bearable (dare I say it could even be fun?)

This list is heavily inspired by [Mike Ferguson's ROS 2 Cookbook](https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/getting_started.md).

1. Read [[00 Notes/thesis/ros/ROS 2 guidelines for Nakama/ROS 2 Beginner's Guide/Installation|Installation]] to install ROS 2 on your machine.

2. Read all pages of the [Basic Concepts](https://docs.ros.org/en/humble/Concepts/Basic.html) from the official website. They will introduce you to the basic concepts needed to understand the later steps. Don't worry if you do not understand part or if you cannot see how they integrate: later pages of the Beginner's Guide will explain in more detail and link back to the Basic Concepts.
    - Also read [Topics vs Services vs Actions](https://docs.ros.org/en/humble/How-To-Guides/Topics-Services-Actions.html)

3. Do the following tutorials from the [Beginner: CLI tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html) tutorials. They are somewhat boring but also essential to get started with using ROS before writing your own code. Again, later pages of the Beginner's Guide will link back to these tutorials when relevant.
    - [Using `turtlesim`, `ros2`, and `rqt`](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
    - [Understanding nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
    - [Understanding topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
    - [Understanding parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
    - [Understanding services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
    - [Using `rqt_console` to view logs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)
    - [Using `ros2doctor` to identify issues](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Getting-Started-With-Ros2doctor.html)
    - [Launching nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)
    We deliberately skip some tutorials which cover more advanced use cases (e.g. actions), we'll come back to those later.
    
4. Do the following tutorials from the [Beginner: Client libraries](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html) tutorials. These tutorials will let you write some actual ROS 2 code.
    - Before you start, please keep the following points in mind:
        - The official tutorials often have code with what we consider bad practises w.r.t. code style or organization. Later in the Beginner's Guide, we will teach what we consider good practises and link back to these tutorials when relevant.
        - Pick the language that you are planning to write your ROS 2 nodes in first (either Python or C++). It is easiest to focus on one language at a time.
    - First read [[Workspaces]]up to and including section [[Workspaces#Create a workspace]], then follow the other tutorials in this list.
    - [Creating a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
    - [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) or [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
    - [Using parameters in a class (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html) or [Using parameters in a class (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
    - [Writing a simple service and client (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html) or [Writing a simple service and client (Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
    - [Creating custom msg and srv files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
    - [Implementing custom interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)

5. The launch system is used to start a complex arrangement of nodes and configuration in a predefined way. When you use more than 2 nodes, or your nodes require parameters, it is already worth it to use a launch file to start them up. Follow all [Launch tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html). Again, later pages of the Beginner's Guide will link back to these tutorials when relevant.

6. Nearly there. Before we get started with the other pages of the Beginner's Guide, there is a selected number of intermediate and advanced concepts which are good to know about. Read the following pages:
    - [Quality of Service settings](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
    - [TF2](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html)
    - [REP 103: Standard Units of Measure and Coordinate Conventions](https://ros.org/reps/rep-0103.html)

7. Now it is time to learn about the concepts of the tutorials in more depth. That is done using the following pages of the Beginner's Guide.
    1. **These pages are not done yet, but will be published in the (near) future.**
