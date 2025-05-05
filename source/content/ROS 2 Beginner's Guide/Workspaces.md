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

A **workspace** is a structured directory containing ROS 2 packages. It includes the **source code**, the **built programs** and **installation data** for all packages inside it. The directory structure is necessary to use the build tool Colcon.

A workspace usually has a general name to identify which packages are in there (e.g. `nakama` for all custom Nakama packages), suffixed with `_ws` to indicate it is a workspace, e.g. `vlm_ws` or `nakama_franka_ws`. This is not enforced but good practice.

This guide will use `<your_ws>` as placeholder, **replace `<your_ws>` with your workspace name**.

## Relevant ROS 2 documents

- [Configuring environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- [Using `colcon` to build packages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Creating a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

## Terminology

- **Workspace**: a directory containing ROS 2 packages. This includes source code and built programs to run.
- **Source a workspace**: make all built programs from a workspace available to use in the current terminal.
- **Underlay**: Workspace which must contain the dependencies for all packages in your overlay(s).
- **Overlay**: Workspace where new packages can be added without interfering with any of the underlays.

## Folder structure

The folder structure is as follows:

- `<your_ws>/`: the **workspace root**. Here is where you will run Colcon commands from.
    - `src/`: contains the packages' source code. You need to create this folder yourself.
        - `example_package/`:
        - `another_package/`
    - `build/`: contains intermediate files for building packages. Created automatically by `colcon build`.
        - …
    - `install/`: where every package is installed to. Created automatically by `colcon build`.
        - `setup.bash`: file to **source** this workspace, more on that in [[#Source the workspace|Source the workspace]]
        - …
    `logs/`: logged information about Colcon commands. Created automatically by `colcon build`.

## Using workspaces

### Create a workspace

To create a workspace, use the following commands:

```bash
# Create the workspace root
# Replace <your_ws> with a nice name for your workspace
mkdir ~/<your_ws>
cd ~/<your_ws>

# Create the source directory where all the package's source code will go
mkdir src
```

That's it! You now have an empty ROS 2 workspace.

### Build the workspace

To build a workspace, run `colcon build` from the **workspace root**.:

```bash
cd path/to/<your_ws>
colcon build
```

 Important: do not run this command from the terminal where you have also sourced this workspace, more on that in [[#Source the workspace|Source the workspace]]

What happens when you build a workspace?

1. Colcon searches every folder and subfolder inside `<your_ws>/src/` for files called `package.xml`. This file indicates that the containing folder is a ROS 2 package (more on that in [[#Packages|Packages]]).
2. Inside every package found, Colcon builds the package by following the instructions inside `CMakeLists.txt` or `setup.py`, depending on the package build type (more on that in [[#Packages|Packages]]).
3. Any intermediate files during the build process are placed in `<your_ws>/build/`, in a subfolder for each package. If you have built (parts of) the package before, Colcon will reuse these files to shorten the build time.
4. The final package result is placed in `<your_ws>/install/`, in a subfolder for each package.
5. A file `<your_ws>/install/setup.bash` is created with instructions for the terminal to **source** the workspace (more on that in [[#Source the workspace|Source the workspace]])

#### Useful flags for `colcon build`

- `--symlink-install`
    When developing Python packages or launch files, it is useful to use the `--symlink-install` flag during the build. This prevents the need to run `colcon build` every time you make a change to the code because the package contents inside the `install/` folder are linked to the package source inside the `src/` folder instead of copied (the default behaviour):

    ```bash
    colcon build --symlink-install
    ```

- `--packages-up-to`
    Sometimes you just want to build the package that you are working on _along with all its dependencies_. This can cut down on build time drastically, especially if there are many or big packages in your workspace. You can give multiple package names:

    ```bash
    colcon build --packages-up-to example_package_1 example_package_2
    ```

#### Remove the build (clean build)

When you get errors you don't understand, or want to make sure that older files do not interfere with a newer build, you can do a **clean build**. This ensures that all your programs are built from the ground up and no previous (erroneous) parts are left in the `build/` folder to mess up the build process.

To do a clean build, simply remove the `build/` and `install/` folders and start the build again:

```bash
cd path/to/<your_ws>
rm -rf build install
colcon build
```

### Source the workspace

Once a workspace is built, the workspace can be **sourced** to make all built programs available in the current terminal. This will add the installed executables and ROS packages to the paths in the current terminal, so they can be recognized and run. This is necessary for the ROS commands to recognize your packages.

To source the workspace, run the following command from the workspace root:

```bash
source install/setup.bash
```

Important: once you have **sourced** a workspace in a terminal, you should not **build** that same workspace in the same terminal. Colcon should give a warning but do not count on it.

## Multiple workspaces, overlays, and underlays

**TODO: Add figure to make multiple workspaces and overlays/underlays clearer**

It is common to have multiple workspaces active at the same time. For example, at Nakama, we have a workspace called `franka_ros2_ws` which contains all the packages from `franka_ros2`, and we have `nakama_ws` which contains all of our custom code & some other third-party packages.

Because the code in `nakama_ws` requires parts of `franka_ros2`, we need to **source** the workspace `franka_ros2_ws` before building code inside `nakama_ws`. `franka_ros2_ws` provides **dependencies** for `nakama_ws`. Therefore, we call `franka_ros2_ws` an **underlay** for `nakama_ws`, and `nakama_ws` is an **overlay**. We have configured Ubuntu to automatically source `franka_ros2_ws` when a new terminal is started (using the file `~/.bashrc`), so you do not need to do so yourself every time you start a new terminal.

Actually, every new workspace you make is an overlay already: they all use the ROS 2 installation as underlay. The ROS 2 installation includes the base ROS 2 installation, as well as any packages installed from binary (using `sudo apt install ros-humble-package-name`). These packages are usually sourced automatically when a new terminal is started (using the file `~/.bashrc`).
