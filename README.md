# Inverse Demo

## Dependencies 

### Packages

For moving the Frankas:

- [`franka_description`](https://github.com/frankarobotics/franka_description/) for the Franka URDFs;
- [`franka_ros2`](https://github.com/frankarobotics/franka_ros2) for the ROS2 franka integration. This, in turn, requires [`libfranka`](https://github.com/frankarobotics/libfranka) to be installed on the machine;
- [`franka_ros2_multimanual`](https://github.com/idra-lab/franka_ros2_multimanual) for the bimanual setup hardware interface;

For the motion planner:

- clone the [`mdvcpplib`](https://github.com/matteodv99tn/mdv_cpp_lib) on the `inverse` branch in the ROS 2 workspace;

For the human body tracker:

1. install the [`torchure_smplx`](https://github.com/Hydran00/torchure_smplx) library (more instructions there).
1. put in the workspace src the [`smpl_ros`](https://github.com/idra-lab/smpl_ros) library by following the instructions in the corresponding repository. 



### Submodules modifications
- Change `runtime_package` argument in `easy_ur_control/launch/easy_ur_control.launch.py` to `inverse_bringup`
- Change F/T sensor name under `bota_ft_sensor_driver/rokubimini_serial/rokubimini_serial.launch.py to the right model (e.g. BFT-DENS-SER-M8)
- If compilations error in `mdv_cpp_lib` arise, comment the line `#include <rerun/archetypes/series_points.hpp>` in both `rerun.cpp` and `rerun.hpp` 



### Rosdep

All other dependencies for the ROS 2 packages can be easily installed through `rosdep`:

1. update the package list `sudo apt-get update`;
1. update the rosdep database `rosdep update`;
1. install the dependencies:
   ```
   rosdep install --ignore-src -y --from-paths </path/to/ws>/src
   ```

## Task description
- Task 1: The robot on the right pick and place the small block on the right to the mounting and the operator screw the two screws.  
- Task 2: The robot on the left pick and place the big block on the left over the right hole using a visuo/tactile peg-in-hole strategy, then it keep it in place while the human screws it.
- Task 3: The robot on the left, fully autonomously pick the cable and put it on the plug using a visuo/tactile peg-in-hole strategy

### Project Scheme
This following scheme is the block diagram of the demo:
```mermaid
---
config:
  theme: 'neutral'
---
graph TD;
    A(Orchestrator)-->B(Motion Planner & Action Monitoring);
    C(Registered Mesh)-->B
    B-->D(Impedance Controller Robot Left);
    B-->E(Impedance Controller Robot Right);
    F(Human Body Tracker)-->A(Orchestrator);
    D-->G(Task 2-3);
    E-->H(Task 1);
```


## Running the code

### Main tasks execution

1. On one terminal launch the following:
   ```
   ros2 launch inverse_bringup inverse_ur.launch.py
   ```
   This will:

   - create the connection with the UR robot; the robot will start using the `cartesian_compliance_controller` with the parameters defined in the [`ur_controllers.yaml`](inverse_bringup/config/ur_controllers.yaml) file;
   - spawn RViz;
   - TODO: add all frames to the launch file 
   - start the motion planner.

1. For starting the execution of the actual task, on another terminal you must run the orchestrator as follows:
   ```
   ros2 run inverse_orchestrator orchestrator
   ```
   To properly decide which task to execute, you must first make sure to update the [`orchestrate`](https://github.com/idra-lab/inverse-demo/blob/faac88be8ce0415ba8300680b065f9e9fffc180c/inverse_orchestrator/inverse_orchestrator/orchestrator.py#L61-L63) function within the `Orchestrator` node defined in [`orchestrator.py`](./inverse_orchestrator/inverse_orchestrator/orchestrator.py).

   As of now, within the program execution, the human input is required as trigger to continue the motion plan. This can be simply achieved by pressing _enter_ on the keyboard whenever the robot is in grasp/release position. The robot will then take care to resume the execution.


### Skill recording

To record the a human skill (DMP based) that then can be executed by the robot:

1. Use the [following launch file](./inverse_bringup/launch/teach.launch.py) to start the robot in gravity compensation
   ```
   ros2 launch inverse_bringup teach.launch.py
   ```
1. Once the robot is in the desired initial configuration, call the service `/learn_skill` as follows:
   ```
   ros2 service call /learn_skill inverse_msgs/srv/LearnSkill "
        skill_name: ''
        registration_duration_secs: 10.0
        num_basis: 30"
   ```
   Feel free to adjust the parameters at your will. About the number of basis, chose a low number (10-12) if you need to do a simple task, while increase up to 40-50 for very complex motions.

**Note:**
When learning, only 1 trajectory of the robot is actually recorded. The recorded pose is the one of a TF2 transform of the frame specified in the [`node_parameters.yaml`](./inverse_bringup/config/node_parameters.yaml) file, specifically in the [arguments for the `skill_learner` node](https://github.com/idra-lab/inverse-demo/blob/faac88be8ce0415ba8300680b065f9e9fffc180c/inverse_bringup/config/parameters.yaml#L23-L30).
For the setup with UR, the `tool0` frame is reference to the TCP of the robot.

