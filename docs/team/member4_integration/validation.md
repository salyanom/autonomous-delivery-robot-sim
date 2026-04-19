# Member 4 Validation Log

## Runtime Checks
- Full stack launch succeeds: Run `ros2 launch rdsim_gazebo rdsim_gps_navigation.launch.py` in WSL and confirm Gazebo + RViz open without launch-time errors.
- Lifecycle nodes active: Verify `bt_navigator`, `controller_server`, `planner_server`, and `map_server` report `active`.
- Recovery procedure verified: Kill stuck Gazebo process and relaunch successfully using the runbook steps.

## Commands Used
```bash
ros2 launch rdsim_gazebo rdsim_gps_navigation.launch.py
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /map_server
```

## Evidence
- Screenshot/Video path: docs/team/member4_integration/evidence/
- Notes:
	- Attach terminal output snippets for lifecycle checks.
	- Include one screenshot of Gazebo + RViz running together.
	- Record any recovery steps used during demo rehearsal.
