# Team Contribution Plan (4 Members)

Use this file to present clear ownership and integration effort for the RDSim project.

## 1) Project Statement (Use in Presentation)

RDSim is an end-to-end autonomous delivery robot simulator built on ROS 2 Humble and Gazebo Classic.
The project integrates simulation, localization, planning, and control into one reproducible workflow.

## 2) Team Split (Recommended)

## Member 1: Simulation and Robot Modeling

Primary ownership:
- Robot model setup and simulation assets
- Gazebo world and robot spawn flow
- Sensor plugin and kinematic simulation readiness

What this member should demonstrate:
- Robot and world load correctly
- Robot state is visible in simulation
- Basic robot movement path can be observed in Gazebo

Integration responsibility:
- Ensure simulation outputs are usable by localization and Nav2 stacks

Expected evidence:
- Launch screenshots or short clip of world plus robot
- Topic availability from simulated sensors

## Member 2: Localization and State Estimation

Primary ownership:
- Sensor fusion and pose estimation flow
- Coordinate frame consistency (map, odom, base_link)
- Localization stack stability in runtime

What this member should demonstrate:
- Robot pose stays stable in RViz
- Odometry and localization topics are publishing continuously
- Initial pose correction and recovery behavior work

Integration responsibility:
- Provide reliable pose source to navigation stack

Expected evidence:
- Topic output for odometry and localization
- RViz screenshot showing stable pose and transforms

## Member 3: Navigation and Autonomy Logic

Primary ownership:
- Nav2 bringup, path planning, and local control
- Goal handling and route execution
- Obstacle-aware navigation behavior

What this member should demonstrate:
- Goal is accepted and path is generated
- Robot follows path and reaches destination
- Replanning happens when path becomes constrained

Integration responsibility:
- Consume localization output and produce velocity commands

Expected evidence:
- Goal-to-arrival demo
- Runtime command stream and status output

## Member 4: System Integration, DevOps, and Documentation

Primary ownership:
- End-to-end launch orchestration and reproducibility
- Environment setup in WSL and dependency resolution
- Runbook, troubleshooting, and demo reliability

What this member should demonstrate:
- Fresh machine setup sequence
- One reliable launch flow for demonstration
- Recovery steps for common failures (stuck Gazebo, GUI issues, lifecycle checks)

Integration responsibility:
- Make all modules run together consistently and make demo repeatable

Expected evidence:
- Clean run instructions
- Troubleshooting checklist
- Final demo orchestration

## 2.1 File Ownership Matrix (Concrete Files)

Important rule for presentation:
- Mark a file as Created only if your team authored major logic in it.
- Mark a file as Maintained if your team modified or tuned it.
- Mark a file as Integrated/Validated if your team connected it and verified runtime behavior.

Member 1 (Simulation and Robot Modeling) candidate files:
- [Created/Maintained] rdsim_description/urdf/rdsim.urdf.xacro
- [Created/Maintained] rdsim_description/urdf/rdsim.gazebo.xacro
- [Maintained] rdsim_description/launch/rdsim_description.launch.py
- [Integrated/Validated] rdsim_description/launch/rdsim_gazebo.launch.py
- [Maintained] rdsim_gazebo/worlds/small_city.world
- [Integrated/Validated] rdsim_gazebo/launch/rdsim_gazebo_world.launch.py

Member 2 (Localization and State Estimation) candidate files:
- [Maintained] rdsim_localization/config/dual_ekf_navsat_params.yaml
- [Maintained] rdsim_localization/launch/hdl_localization.launch.py
- [Created/Maintained] rdsim_localization/src/initialpose_to_setpose.cpp
- [Integrated/Validated] rdsim_nav2/map/map.yaml
- [Integrated/Validated] rdsim_nav2/map/map.pcd

Member 3 (Navigation and Autonomy Logic) candidate files:
- [Maintained] rdsim_nav2/launch/nav2_gazebo.launch.py
- [Maintained] rdsim_nav2/param/rdsim_nav2.yaml
- [Maintained] rdsim_nav2/map/topology.yaml
- [Maintained] rdsim_scenario/behavior_trees/delivery.xml
- [Created/Maintained] rdsim_scenario/plugins/navigate_pose_action_node.cpp
- [Maintained] rdsim_scenario/include/rdsim_scenario/plugins/navigate_pose_action_node.hpp
- [Maintained] rdsim_scenario/include/rdsim_scenario/plugins/navigate_topology_action_node.hpp
- [Maintained] rdsim_scenario/src/rdsim_scenario.cpp
- [Maintained] rdsim_scenario/apps/rdsim_scenario_node.cpp

Member 4 (System Integration, DevOps, Documentation) candidate files:
- [Integrated/Validated] rdsim_gazebo/launch/rdsim_gps_navigation.launch.py
- [Maintained] rdsim_launcher/rdsim_launcher.yaml
- [Maintained] docker/install.sh
- [Maintained] docker/run_command.sh
- [Maintained] README.md
- [Created/Maintained] RUN_INSTRUCTIONS.md
- [Created/Maintained] TEAM_CONTRIBUTION.md

Runtime proof each member can show:
- Member 1: Gazebo world and robot spawn successfully.
- Member 2: stable localization output and pose alignment in RViz.
- Member 3: goal accepted, path planned, robot reaches destination.
- Member 4: one-command launch flow and recovery from runtime issues.

## 3) Integration Story (What To Say)

Use this sequence in your presentation:
1. Simulation publishes robot and sensor data.
2. Localization fuses sensor inputs to estimate robot pose.
3. Navigation consumes pose and map data, then computes path and control outputs.
4. Control outputs drive the robot in simulation.
5. System integration ensures all layers launch and run reliably on one workflow.

## 4) Speaking Script Split (About 1 Minute Each)

Member 1 script:
- I handled the simulation layer, including robot model and world setup. My goal was to provide a physically consistent environment and sensor-ready simulation so higher-level autonomy modules could run correctly.

Member 2 script:
- I handled localization and state estimation. I integrated sensor fusion and transform consistency so the robot has a stable, continuous pose estimate used by navigation.

Member 3 script:
- I handled the autonomy and navigation stack. I integrated Nav2 planning and control so the robot can receive goals, plan collision-aware paths, and execute motion commands to reach the destination.

Member 4 script:
- I handled integration and reproducibility. I ensured all modules run together in WSL with stable launch and troubleshooting steps, and I prepared the runbook for a reliable live demo.

## 5) Contribution Proof Checklist

Use this checklist before submission:
- Each member has at least one owned module and one integration task.
- Each member has one runtime proof item (topic output, lifecycle state, or demo clip).
- Team can explain one data flow from sensor to motion command end-to-end.
- Team can recover from one failure case during demo.

## 6) Demo Day Flow (Short)

1. Launch full system.
2. Set initial pose in RViz.
3. Send goal in RViz.
4. Show robot movement in Gazebo.
5. Show command and status topics in terminal.
6. Explain which member owns each visible step.

## 7) Fill-In Section (Replace With Actual Names)

- Member 1 Name: ____________________
- Member 2 Name: ____________________
- Member 3 Name: ____________________
- Member 4 Name: ____________________

- Member 1 Owned Files/Packages: ____________________
- Member 2 Owned Files/Packages: ____________________
- Member 3 Owned Files/Packages: ____________________
- Member 4 Owned Files/Packages: ____________________

- Member 1 Claim Type Per File (Created/Maintained/Integrated): ____________________
- Member 2 Claim Type Per File (Created/Maintained/Integrated): ____________________
- Member 3 Claim Type Per File (Created/Maintained/Integrated): ____________________
- Member 4 Claim Type Per File (Created/Maintained/Integrated): ____________________

- One integration issue solved by team: ____________________
- One runtime metric shown in demo: ____________________
