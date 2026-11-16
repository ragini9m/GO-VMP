# Global Optimization for View Motion Planning(GO-VMP)

> **Thesis extension: consistency-aware view selection.** The folder [consistency_overlap/](consistency_overlap/) adds an overlap constraint (g) to the GO-VMP optimization, so that selected views can be registered against each other and pose errors become detectable. It is standalone (C++/Gurobi and Python) and does not change the ROS planner. Setup, build and run commands are in [consistency_overlap/README.md](consistency_overlap/README.md).

## Usage

Clone the repository with submodules:
```
git clone --recursive https://github.com/AllenIsaacJose/GO-VMP.git
```

### 1. Native Build

**Install prerequisites**
- Ubuntu 20.04 LTS (Focal Fossa)
- ros-noetic-desktop-full
- [Open3D v0.17](https://github.com/isl-org/Open3D/releases/download/v0.17.0/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.17.0.tar.xz)

Run the setup script to configure the workspace:
```
cd GO-VMP
./setup_workspace.sh
```

Build the workspace:
```
catkin build
```

Source the workspace:
```
source devel_release/setup.bash
```

### 2. Setup with Docker and VS Code

Install Docker, VS Code, and the Dev Containers extension in VS Code. The repository contains a Dockerfile and devcontainer.json in the .devcontainer folder. When opening the folder with VS Code, it should ask you whether you want to reopen the folder in a container. If not, you can press F1 and search for "Dev Containers: Rebuild and Reopen in Container". After the container is open, run "docker_setup_workspace.sh" to initialize and build the workspaces. Subsequently, you can simply run "catkin build". Run "source devel_release/setup.bash" to source the workspace.

## Setting up Gurobi Solver

Follow the [link](https://ca.cs.uni-bonn.de/doku.php?id=tutorial:gurobi-install) to install the Gurobi solver version 11.0.1 on Linux systems.

The Gurobi version used can be adjusted in [view_motion_planner/CMakeLists.txt](https://github.com/AllenIsaacJose/GO-VMP/blob/main/src/view_motion_planner/CMakeLists.txt)
```
set(GUROBI_HOME "/opt/gurobi1101/linux64")

target_link_libraries(planner_node ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PCL_LIBRARIES} ${OMPL_LIBRARIES} tbb gurobi_c++ gurobi110)
target_link_libraries(plan_on_map ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PCL_LIBRARIES} ${OMPL_LIBRARIES} tbb gurobi_c++ gurobi110)
```

## Run view motion planner (example)

Launch simulation:
```
roslaunch ur_with_cam_gazebo ur_with_cam.launch base:=trolley world_name:=world23
```

Run view motion planner:
```
rosrun view_motion_planner planner_node
```

Launch rviz:
```
roslaunch ur_with_cam_gazebo rviz.launch
```

Run rqt:
```
rqt
```

Select the ROI Viewpoint Planner plugin: Plugins -> Configuration -> ROI Viewpoint Planner

It should automatically switch to the view motion planner tab.

To run the complete pipeline, change "mode" to "PLAN_WITH_TROLLEY".
