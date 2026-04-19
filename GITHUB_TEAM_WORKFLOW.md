# GitHub Repository and Branch Workflow (4-Member Team)

This guide defines exactly how your team should create the GitHub repository, create branches, organize folders, and push files according to contribution ownership.

## 1) Create the Team Repository

Use one shared team repository on GitHub.

Recommended settings:
- Repository name: rdsim-team-project
- Visibility: private during development, public only if needed for final submission
- Default branch: main
- Add all 4 members as collaborators with write access

Optional but recommended:
- Enable branch protection on main (no direct push)
- Require pull request before merge to main
- Require at least 1 reviewer

## 2) Push Existing Project to Team Repo

Run in WSL terminal from the project root:

```bash
cd ~/ros2_ws/src/RDSim

# If remote origin points somewhere else, keep it as upstream
# and add your team repo as origin.
git remote -v
git remote rename origin upstream 2>/dev/null || true
git remote add origin <TEAM_REPO_URL>

# Ensure main branch naming is consistent
git branch -M main

# Push full project
git push -u origin main
```

If your project uses submodules, also run:

```bash
git submodule update --init --recursive
```

## 3) Branches Each Member Should Create

Create one integration branch and one feature branch per member.

Required branches:
- develop (team integration branch)
- feat/member1-simulation-modeling
- feat/member2-localization-estimation
- feat/member3-navigation-autonomy
- feat/member4-integration-devops-docs

Create them with:

```bash
# create develop from main
git checkout main
git pull origin main
git checkout -b develop
git push -u origin develop

# member-specific branches from develop
git checkout develop
git pull origin develop
git checkout -b feat/member1-simulation-modeling
git push -u origin feat/member1-simulation-modeling

git checkout develop
git checkout -b feat/member2-localization-estimation
git push -u origin feat/member2-localization-estimation

git checkout develop
git checkout -b feat/member3-navigation-autonomy
git push -u origin feat/member3-navigation-autonomy

git checkout develop
git checkout -b feat/member4-integration-devops-docs
git push -u origin feat/member4-integration-devops-docs
```

## 4) Folder Plan (What To Create)

Keep code changes inside existing package folders. Do not create random top-level folders.

Use this structure for team evidence and reporting:

```text
docs/
  team/
    member1_simulation/
      contribution.md
      validation.md
    member2_localization/
      contribution.md
      validation.md
    member3_navigation/
      contribution.md
      validation.md
    member4_integration/
      contribution.md
      validation.md
```

These files are already scaffolded in this repository. Each member should update their own two files:
- contribution.md: what was implemented/maintained/integrated
- validation.md: runtime proof (commands, outputs, screenshots)

## 5) What Each Member Should Push (Code File Scope)

Only push files in your owned scope unless doing a reviewed integration change.

Member 1 (Simulation and Robot Modeling):
- rdsim_description/urdf/rdsim.urdf.xacro
- rdsim_description/urdf/rdsim.gazebo.xacro
- rdsim_description/launch/rdsim_description.launch.py
- rdsim_description/launch/rdsim_gazebo.launch.py
- rdsim_gazebo/worlds/small_city.world
- rdsim_gazebo/launch/rdsim_gazebo_world.launch.py

Member 2 (Localization and State Estimation):
- rdsim_localization/config/dual_ekf_navsat_params.yaml
- rdsim_localization/launch/hdl_localization.launch.py
- rdsim_localization/src/initialpose_to_setpose.cpp
- rdsim_nav2/map/map.yaml
- rdsim_nav2/map/map.pcd

Member 3 (Navigation and Autonomy):
- rdsim_nav2/launch/nav2_gazebo.launch.py
- rdsim_nav2/param/rdsim_nav2.yaml
- rdsim_nav2/map/topology.yaml
- rdsim_scenario/behavior_trees/delivery.xml
- rdsim_scenario/plugins/navigate_pose_action_node.cpp
- rdsim_scenario/include/rdsim_scenario/plugins/navigate_pose_action_node.hpp
- rdsim_scenario/include/rdsim_scenario/plugins/navigate_topology_action_node.hpp
- rdsim_scenario/src/rdsim_scenario.cpp
- rdsim_scenario/apps/rdsim_scenario_node.cpp

Member 4 (Integration, DevOps, Documentation):
- rdsim_gazebo/launch/rdsim_gps_navigation.launch.py
- rdsim_launcher/rdsim_launcher.yaml
- docker/install.sh
- docker/run_command.sh
- README.md
- RUN_INSTRUCTIONS.md
- TEAM_CONTRIBUTION.md
- GITHUB_TEAM_WORKFLOW.md

## 6) Daily Git Routine for Each Member

Run this before starting work:

```bash
git checkout <YOUR_BRANCH>
git fetch origin
git rebase origin/develop
```

After making changes:

```bash
git add <files_you_changed>
git commit -m "feat(memberX): short clear description"
git push origin <YOUR_BRANCH>
```

Open Pull Request:
- Source: feat/memberX-...
- Target: develop
- Include summary, changed files, test/launch proof

## 7) Merge Policy

Use this order to reduce conflicts:
1. Member 1 PR to develop
2. Member 2 PR to develop
3. Member 3 PR to develop
4. Member 4 integration PR to develop
5. Final team PR: develop -> main

Rules:
- No direct push to main
- Re-run launch demo after each merge group
- Resolve conflicts on source branch, then update PR

## 8) Final Submission Tag

After the final verified demo:

```bash
git checkout main
git pull origin main
git tag -a v1.0-demo -m "RDSim team final demo"
git push origin v1.0-demo
```

## 9) Pull Request Checklist (Use in Every PR)

- Scope matches member ownership
- Build/launch still works
- No unrelated file edits
- contribution.md updated in docs/team/memberX_*/
- validation.md updated with proof
