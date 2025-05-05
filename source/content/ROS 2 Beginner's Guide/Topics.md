Topics are basically an abstraction level which allows data transfer between programming languages and PCs.

Basics:
- <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html>
- <https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html>
- <https://docs.ros.org/en/humble/How-To-Guides/Topics-Services-Actions.html>
- <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html>
- <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>

Remapping

Do not publish `std_msgs` but use `example_interfaces` instead. Or better yet, use one of the `common_interfaces` and / or define custom interfaces.

Topic names and expansion:
- E.g. `~/topic` vs `/topic` vs `topic`
- <https://design.ros2.org/articles/topic_and_service_names.html>
- <https://ros.org/reps/rep-0135.html> (namespaces for drivers)

## Advanced

QoS:
- <https://docs.ros.org/en/humble/Tutorials/Demos/Quality-of-Service.html>
- <https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html>

Optimizing speed:
- <https://docs.ros.org/en/humble/How-To-Guides/Configure-ZeroCopy-loaned-messages.html>
