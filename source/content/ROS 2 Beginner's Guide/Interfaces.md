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

## Related documentation

Already covered in [[Getting started]]:
- [Basic Concepts: Interfaces](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)
- [Tutorial: Creating custom msg and srv files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Tutorial: Implementing custom interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)
- [REP 103: Standard Units of Measure and Coordinate Conventions](https://ros.org/reps/rep-0103.html)

New resources:
- [ROS 2 design page on Interface Definition](https://design.ros2.org/articles/legacy_interface_definition.html)
    - Almost all of this is already covered in the Concepts and Tutorials, but there are some additional technical details which could be interesting.

## Background

ROS applications typically communicate through interfaces of three types:

- [[#Messages]]: `.msg` files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.
    
- [[#Services]]: `.srv` files describe a service. They are composed of two parts (which are actually two separate messages): a request and a response.

- [[#Actions]]: `.action` files describe actions. They are composed of three parts (again, those are all separate messages): a goal, a result, and feedback.

ROS 2 uses a special description language, the Interface Definition Language (IDL) to describe the interfaces. From IDL, Python and C++ source code is generated to be used in other ROS packages.

It is common for interfaces to be defined in a separate dedicated package. See [[Packages]] for more information on how to organise interface packages, if you decide to define custom interfaces.

## Messages

Messages are a data packet to exchange information between different ROS programs running at the same time. They are most commonly used by Publishers and Subscribers.

## Services

_This section is not yet written._

## Actions

_This section is not yet written._

## Custom interfaces

Section [[#Fields]] explains in detail how to define custom interfaces. Before going into details on _how_ to define custom interfaces, it is best to first consider whether you _should_ and how it can be done meaningfully.

### Do I need a custom interface?

Before creating a custom interface package, please consider the following questions (click the links to the sections for more information on how to answer the question):

1. Can you use [[#ROS 2 `common_interfaces`]]
2. If not, can you use [[#Other existing ROS interfaces]]
3. If not, you have to create [[#Your own custom interfaces package]] and follow the instructions in the rest of this document.

#### ROS 2 `common_interfaces`

Whenever possible, you should use interfaces defined in one of the [ROS 2 `common_interfaces`](https://github.com/ros2/common_interfaces) instead of defining your own since they are widely available, have clear semantics, and will cover almost all of your use cases.

However, you need to ask yourself some questions when selecting one of the `common_interfaces`:

- Does this interface cover _all_ information that I need to send in a single message?
    - I.e. do I need to send more information than is included in the interface? If so, you need to create a custom interface which includes the selected common interface and also the other data that you require.
    - Example: you want to send a bounding box ([`sensor_msgs/msg/RegionOfInterest.msg`](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/RegionOfInterest.msg)) but you also want to include a timestamp, so you need to create a new interface with a field for the `sensor_msgs/RegionOfInterest` and a field for the timestamp.
- Does this interface _semantically_ match the type of data that I will send?
    - Example: I could send a 2D array representing a cost map for path planning as a `sensor_msgs/msg/Image` since they are both 2D data structures. But conceptually, they are different things, therefore it does not make sense to use this data structure.
- Bonus question: is the interface part of `std_msgs`? Then do not use it, they are deprecated. Almost all interfaces in `std_msgs` can be represented by a raw field type instead.
    - The only exception to this is `std_msgs/msg/Header`, which is recommended to be nested in other ROS interfaces.

If the `common_interfaces` are not suitable, move on to the next section.

#### Other existing ROS interfaces

There are many community-contributed ROS interface packages which might exactly match your use case. You can find them in your organization's repositories, or search on Google for interfaces packages, e.g. on Github.

If you find such a package, check if it is suitable with the same questions from [[Interfaces#ROS 2 `common_interfaces`|the previous section]] apply for this package. If it is not suitable, move on to the next section.

If the package is suitable, you can check if you can install it with binaries. Go to <https://index.ros.org/> and search for the package name. If it exists, you can install it using:

```bash
sudo apt install ros-humble-<package-name-with-hyphens>

# E.g. for package vision_msgs
sudo apt install ros-humble-vision-msgs
```

If you cannot install the package with binaries, you can also clone the package to your workspace and build it alongside your own packages.

> [!TIP]
> For Nakama Robotics Lab, we have defined a bunch of common interfaces which might just suit your needs: <https://bitbucket.org/ctw-bw/nakama_common_interfaces>

#### Your own custom interfaces package

If you cannot use `common_interfaces` or another released ROS package, you need to create your own custom interfaces package with the instructions below.

### Designing meaningful interfaces

In ROS 2, interfaces are the fundamental way nodes communicate. A well-designed interface should be **clear, semantic, and closely related to a physical object or concept** in your system.

#### Use semantic names

The name of the interface and its fields should be **self-explanatory**. This makes it easier for developers to understand the interface’s purpose without needing additional documentation.

**Example:** If a robot arm needs to send its entire state, a good interface name might be `msg/FrankaRobotState.msg`. This is better than something vague like `msg/DataPacket.msg`.

In the first case, both the interface name (`FrankaRobotState`) and its fields (e.g. `joint_states`, and `battery`) directly describe their meaning.

#### Interfaces should represent physical objects or concepts

Whenever possible, an interface should correspond to a **real-world entity** (something "physical"). This can be a description of a physical object (i.e. pose, velocity or dimensions of an object) or a physical concept (i.e. a transformation, path through space, timestamp, or estimated position).

For example, if your robot has a LiDAR sensor, the interface published would be `sensor_msgs/LaserScan`. This directly represents a LiDAR scan, making it intuitive for anyone working with the system.

However, sometimes an interface does not represent something "physical", e.g. strings for logging or metadata. If that is the case, clearly document what the interface represents instead.

#### Use sub-messages for complex concepts

Sometimes, a concept is too complex to represent in a single interface. In that case, **break it down into smaller, meaningful sub-messages**. This makes your message parts modular and reusable.

> [!IMPORTANT]
> Apply the same principles in [[#Designing meaningful interfaces]] to all sub-messages as well. Getting a good trade-off granularity (i.e. splitting up as much as possible) and practicality is not always easy.

For example, an interface representing a robot’s entire state might be too large to handle effectively. Instead, you could compose it from multiple smaller messages while clearly documenting each field:

```idl
# nakama_franka_interfaces/msg/FrankaRobotState.msg

std_msgs/Header header
sensor_msgs/JointState joint_states
geometry_msgs/PoseWithCovariance pose
sensor_msgs/BatteryState battery
```

> [!TIP]
> Whenever possible, use messages defined in one of the [ROS 2 `common_interfaces`](https://github.com/ros2/common_interfaces) since they are widely available, have clear semantics, and will cover almost all of your use cases. The same questions in section [[#Do I need a custom interface?]] apply in deciding whether one of the `common_interfaces` is suitable as field in your custom interface.

#### Combine all related data into a single interface

It might be tempting to split up complex data that a node produces into multiple interfaces and publish on separate topics, so that any subscribers can choose which parts of the data they are interested in.

This could make a subscribed node's life much harder. If the complex data is spread over multiple topics, and the interfaces on those topics do not arrive at the same time (which they won't), the subscribed node needs to correlate and synchronize all topics somehow. This is often annoying and hard to get right. It would be much easier to simply obtain all data on the subscriber and not use the parts the node is not interested in.

When designing your interface, try to keep in mind how it will be used: will the data likely be processed at the same time? If so, strongly consider combining it into a single interface.

#### Use comments to document

Use comments in your interface files to clarify your interface and its fields. **Documentation is critical** to ensure other people understand how your interface should be used, since semantic naming can only get you so far.

There are no conventions or styles to adhere to, but you can read some interface definitions in `common_interfaces` (e.g. [`sensor_msgs/msg/RegionOfInterest`](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/RegionOfInterest.msg), [`geometry_msgs/msg/VelocityStamped`](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/VelocityStamped.msg)) to get a feeling for how to properly document your interface with comments.

Here is an example of how the example interface from before can be properly commented:

```idl
# nakama_franka_interfaces/msg/FrankaRobotState.msg

# Header
# The header.frame_id is set to the base link of the robot, which is "base_link" by default.
std_msgs/Header header

# Joint state of the robot
sensor_msgs/JointState joint_states

# End-effector pose
# This is the same pose that is obtained when applying forward kinematics to the joint_states, but to prevent recalculation, it is also included in this message
geometry_msgs/PoseWithCovariance pose

# Current robot battery state
sensor_msgs/BatteryState battery
```

> [!TIP]
> When an interface package is a into Python or C++ code, the documentation is not included. You have two options to view the documentation:
> - Go to the interface package source files.
> - Use CLI tool `ros2 interface show`:
> ```bash
>     cd ~/<your_ws>
>     source install/setup.bash
>     ros2 interface show your_interfaces_pkg/msg/YourMsg
>     ```

### Fields

Every interface consists of number of fields to represent the data. Each field consists of a type and a name, separated by a space, i.e:

```idl
fieldtype1 fieldname1
fieldtype2 fieldname2
fieldtype3 fieldname3
```

For example:

```idl
int32 my_int
string my_string
```

#### Field types

For a complete list of field types, see [the Field types section of the Interfaces Concepts](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html#field-types) of the official documentation.

#### Strings

A string can also have a max size

```idl
string<=5 up_to_five_characters_string "Hello"
```

#### Complex types (nested interfaces)

You can nest messages into a new interface definition. This way, you can compose complex interfaces while keeping DRY. This is commonly used in common ROS interfaces e.g. `geometry_msgs/msg/PoseStamped` ([source](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg)) is composed of a `geometry_msgs/msg/Pose` ([source](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Pose.msg)) for pose information and `std_msgs/msg/Header` ([source](https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Header.msg)) for timestamp and reference frame information.

Whenever possible, include messages defined in one of the [ROS 2 `common_interfaces`](https://github.com/ros2/common_interfaces) instead of defining your own since they are widely available, have clear semantics, and will cover almost all of your use cases.

You can only include messages into any other interface (message, service, or action). You cannot include a service or action into another interface.

The nested message will be a field inside the new interface. The type of the field is the name of the package and the PascalCase name of the included message:

```idl
# Syntax
<pkg_name>/<MsgName> <fieldname>

# Examples
std_msgs/Header header
my_custom_interfaces_pkg/CoolMessage cool_field
```

> [!NOTE]
> A common syntax error is typing `pkg_name/msg/MsgType` instead of `pkg_name/MsgType` in the interface definition file.
>
> This confusion comes from the Python and C++ client libraries, where you _do_ need to include messages using the `msg` part in the path (e.g. `from pkg_name.msg import MsgType` and `#include "pkg_name/msg/msg_type.hpp"`)

#### Constants

You can define a constant in an interface as any field, but its value can never be changed programmatically. To define a constant, define it with an UPPERCASE name and use the equal sign to define the value:

```idl
# Syntax
<constanttype> <CONSTANTNAME> = <constantvalue>

# Examples
int32 X = -123
string FOO = "foo"
int32[] SIZES = [-200, -100, 0, 100, 200]
```

You can use this to let users of the interface know that you expect specific values. However, it is not possible to limit a field to only constant types (enumerate). So it is best to clearly document how you expect the constants to be used with comments inside the interface.

```idl
# msg/Color.msg

# Possible colors
string RED = "Red"
string BLUE = "Blue"
string GREEN = "Green"

# Color field
# Important: use one of the possible color constants
string value
```

#### Default values

If no default value is defined, a common one is used:

- for `bool` it is `false`
- for _numeric_ types it is a `0` value
- for `string` it is an _empty_ string
- for _static size arrays_ it is an array of N elements with its fields zero-initialized
- for _bounded size arrays_ and _dynamic size arrays_ it is an empty array `[]`

But you can also assign your own default value:

```idl
# Syntax
<fieldtype> <fieldname> <defaultvalue>

# Examples
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]
```

A default value can be set using a constant:

```idl
# msg/Color.msg

# Possible colors
string RED = "Red"
string BLUE = "Blue"
string GREEN = "Green"

# Color field, use one of the possible color constants
string value RED
```

#### Arrays

Here are some common patterns for defining arrays.

##### Static array (array with fixed length)

If no default value is given, it is initialized to an array of N elements with its fields zero-initialized, e.g. `[0, 0, 0, 0, 0]`

```idl
int32[5] static_array

float64[3] static_array_with_defaults [-0.5, 3.1415, 0.99]
```

##### Unbounded dynamic array (array without length)

If no default value is given, it is initialized to an empty array `[]`.

```idl
int32[] unbounded_dynamic_array

bool[] unbounded_dynamic_array_with_defaults [true, false, false, true]
```

##### Bounded dynamic array (array with max length)

If no default value is given, it is initialized to an empty array `[]`.

```idl
int32[<=5] bounded_dynamic_array

int32[<=5] bounded_dynamic_array_with_defaults1 [0, 0]
int32[<=5] bounded_dynamic_array_with_defaults2 [1, 2, 3, 4, 5]
```

##### Arrays of complex types

The patterns above can also be applied to complex types (i.e. other interfaces). Because it is not possible to declare default values for complex types, it is also not possible to have default values for arrays of complex types.

```idl
sensor_msgs/Image[<=4] up_to_four_images_array
```

##### Arrays of strings

You can even have an array of (possibly sized) strings. Like complex types, string arrays do not support default values.

```idl


string[4] exactly_four_unbounded_strings
string<=3[] unbounded_array_with_strings_of_up_to_five_characters_each
string<=5[<=3] up_to_three_strings_with_up_to_five_characters_each

```

#### Multidimensional arrays

Arrays in interfaces can only be **one dimensional**. If you want a multidimensional array or matrix you need to _flatten_ the matrix into a 1D array and include information on the original shape to reconstruct it later.

> [!WARNING]
>
> Note that these instructions tend to be inefficient, especially for large matrices. This holds for packing/unpacking the matrices (in your nodes) but also for the amount of network traffic required. Please consider carefully if you need to send such a large array or if you can use only part of the data.

#### Using `multidimensional_arrays` package

In the repository [`nakama_common_interfaces`](https://bitbucket.org/ctw-bw/nakama_common_interfaces), there are packages to create & use general multidimensional arrays.

##### Fixed-dimensional matrix

If you have a fixed-sized matrix, e.g. 2D or 3D, you can define the shape directly using a field for each dimension. This prevents possible errors in the shape packing/unpacking since it is more explicit.

Example for 3D matrix:

```idl
# Define the size of all dimensions, i.e. how large is each dimension
int32 size_x
int32 size_y
int32 size_z

# 1D data array, which is a flattened version of the multidimensional array
float64[] data
```

Example for 2D matrix e.g. a representation of an image:

```idl
# Define the size of all dimensions, i.e. how large is each dimension
int32 width
int32 height

# 1D data array, which is a flattened version of the multidimensional array
float64[] data
```

##### Arbitrary multidimensional matrix

If you want to be flexible or don't know the number of array dimensions beforehand, you can use the following interface pattern. Use a `shape` array field to define how many dimensions there are and how large each dimension is, and a `data` field to hold the flattened matrix data.

- The number of elements in the `shape` array is the dimensionality of the matrix (e.g. 3 elements in `size` for a 3D matrix).
- Each element in the `shape` array determines how large that dimension is (e.g. `[320, 480, 3]` for a 3D matrix which represents a 320×480 image with 3 colour channels)

```idl
# Define the size of all dimensions, i.e. how large is each dimension
int32[] shape

# 1D data array, which is a flattened version of the multidimensional array
# The type of this field can be changed according to your needs
float64[] data
```
