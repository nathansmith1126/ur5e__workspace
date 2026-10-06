# Controlling the lab UR5e from an Ubuntu desktop

This guide covers the lab's ROS 1 Noetic workspace, MoveIt, and the UR5e teach pendant running PolyScope 5. Menu names may vary slightly with the installed PolyScope and Robotiq URCap versions. Use the separate Gazebo instructions below for simulation. You can skip 1 if you are on Nathan's account. You only need to step 1 once unless you add excessive packages. 

The commands assume the workspace is at `~/ur5e_ws`. Keep each launch terminal open while using the robot.

## 1. Prepare and build the workspace

On a new computer, first provision the lab's ROS Noetic environment, including catkin, the UR ROS driver, UR MoveIt configuration, and the project's dependencies. Gazebo also requires `ur_gazebo`. Cloning this repository alone does not install these prerequisites.

Clone the [lab workspace](https://github.com/nathansmith1126/ur5e__workspace), including its submodules:

```bash
cd ~
git clone --recurse-submodules https://github.com/nathansmith1126/ur5e__workspace.git ur5e_ws
cd ~/ur5e_ws
```

If the workspace already exists, use it instead of cloning over it. For an existing clone with missing submodules, run `git submodule update --init --recursive` from the workspace root.

Before making edits, create a descriptively named branch and document the proposed change in your commit or pull request description. For example:

```bash
git switch -c docs/ur5e-operation-guide
```

Example description: “Document hardware startup, correct shell and Python commands, and explain the Gazebo workflow.”

Build from the workspace root:

```bash
cd ~/ur5e_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated --install
source setup_project.sh
```

**Why `--install`?** The current `setup_project.sh` sources `~/ur5e_ws/install_isolated/setup.bash`. A plain `catkin_make_isolated` build populates the development environment but does not generate the install environment this script expects. Source the base ROS installation before the first build, since the workspace setup file may not exist yet.

The setup script also assumes the workspace is named `~/ur5e_ws`; update its paths if you use another location. To check that required packages are discoverable after sourcing:

```bash
rospack find ur_robot_driver
rospack find ur5e_moveit_config
```

## 2. Turn on and initialize the arm

Follow the lab's startup procedure with the robot workspace clear and the teach pendant accessible. Verify that the mounted tool, payload, and installation settings match the physical setup.

1. Follow the manufacturer's startup sequence: engage the pendant emergency stop, then press the pendant's physical **power button** and wait for PolyScope to boot.
2. Open **Initialize Robot**, using the startup prompt or the robot status/power indicator in the lower-left corner.
3. When the area is clear and it is safe to proceed, release the emergency stop. The robot should show **Power off**.
4. Tap **ON** and wait for **Idle**. This powers the arm but leaves the brakes engaged.
5. Verify the active payload and mounting configuration, then tap **Start** to release the brakes. Clicking and slight movement can occur during brake release.

The **Start** button on the initialization screen enables the arm; the program **Play** button used later starts the External Control program. See [UR's UR5e startup instructions](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-usr-man/complianceUR5e/H_g5_sections/firstuse/quickstart_en.htm).

## 3. Activate the Robotiq gripper

1. Open **Installation → URCaps → Gripper → Dashboard** on the teach pendant.
2. If necessary, use **Scan** to detect the connected gripper.
3. Select the connected gripper and tap **Activate**. Keep the fingers clear: activation can move them.
4. Wait for activation to complete and resolve any reported gripper fault before continuing.

Activation initializes the gripper so it can accept position commands. The tab is **Installation**, singular. See the [Robotiq 2F-85/2F-140 manual](https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_e-Series_PDF_20190206.pdf).

For this checkout, `src/Utils/misc.py` uses robot IP `172.22.22.2` and gripper ID `3`. Check that the configured ID matches the pendant before running gripper scripts.

## 4. Start the UR driver — terminal 1

The desktop and robot must be connected over Ethernet with compatible network settings. This lab uses `172.22.22.2` for the **robot controller**. The desktop must have its own distinct address on the configured robot subnet.

The saved robot installation must have the **External Control URCap** configured with the **desktop's IP address**. This is the address the robot connects back to. Keep the lab's configured port consistent with the driver. This guide uses a program started manually on the pendant, so use **Local Control** for that workflow. See [UR's PolyScope 5 robot setup documentation](https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/setup/robot_setup.rst).

In terminal 1:

```bash
cd ~/ur5e_ws
source setup_project.sh
roslaunch ur_robot_driver ur5e_bringup.launch \
  robot_ip:=172.22.22.2 \
  kinematics_config:="${HOME}/my_ur5e_calibration.yaml"
```

The driver connects ROS to the robot and publishes its state. The calibration YAML must exist and belong to this specific arm; it supplies the robot's calibrated kinematics. Obtain the lab's calibration file if it is missing. See the [UR driver setup and calibration instructions](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#extract-calibration-information).

Leave this terminal running. Receiving joint states does not yet mean the robot is accepting motion commands; start the pendant program in step 7.

## 5. Start MoveIt — terminal 2

Open a new terminal:

```bash
cd ~/ur5e_ws
source setup_project.sh
roslaunch ur5e_moveit_config moveit_planning_execution.launch
```

MoveIt plans paths and sends trajectories to the driver's controller. Run the hardware launch without `sim:=true`. The simulation variant appears below.

## 6. Start RViz — terminal 3

Open another terminal:

```bash
cd ~/ur5e_ws
source setup_project.sh
roslaunch ur5e_moveit_config moveit_rviz.launch \
  rviz_config:="$(rospack find ur5e_moveit_config)/launch/moveit.rviz"
```

RViz visualizes the robot state and provides the MoveIt **MotionPlanning** panel. Moving a goal marker changes the requested target; **Plan** computes a trajectory for inspection. **Execute** commands the connected robot to follow that trajectory, once External Control is running. Preview the plan before executing it.

The default robot configuration does not automatically model the lab table, gripper, or other obstacles. Verify the planning scene against the physical workspace before executing motion. See the [UR ROS driver MoveIt example](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md#control-the-robot-using-moveit).

## 7. Start the External Control program on the pendant

1. Open the saved lab program containing the **External Control** program node. In PolyScope 5, use **Run → Load Program**, or the **Open** file menu to load it into the **Program** view.
2. Select the lab's saved program, referred to in the original notes as `ROS_external control`. Confirm the actual filename on the pendant; this is a lab-defined name.
3. Confirm the associated installation has the correct desktop IP in its External Control settings.
4. Press the program **Play** button (triangle) and complete any required startup prompts.
5. Check terminal 1 for:

```text
Robot ready to receive control commands.
```

Keep the program running while controlling the arm from ROS. If it stops, the robot must reconnect through External Control before further execution. The driver must already be running when this program starts. See the [UR driver control instructions](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md#control-the-robot).

## 8. Run a Python test script — terminal 4

From a new terminal:

```bash
cd ~/ur5e_ws
source setup_project.sh open_venv
```

The optional `open_venv` argument activates the existing `~/ur5e_ws/.venv` as well as sourcing ROS and the workspace. It does not create the environment or install Python dependencies. If it reports that `.venv` is missing, complete the lab's Python environment setup before running scripts.

Optionally launch VS Code from this terminal:

```bash
code .
```

Review the selected script's targets before running it. The current test scripts execute motion automatically after successful planning; they do not wait for RViz approval.

For the arm motion example:

```bash
python3 -m src.test_scripts.test_move
```

For the hardware pick-and-place example, after activating the gripper and reviewing all pickup, intermediate, and placement poses:

```bash
python3 -m src.test_scripts.pick_place
```

Run these as separate examples. The pick-and-place script's optional `--pose-from-camera` argument additionally requires the `/localize_object_server/get_location` service and correctly configured camera-to-robot coordinates.

**Python syntax correction:** `-m` takes a dotted module name, with no `.py` extension. From this workspace root, use `python3 -m src.test_scripts.test_move`, not `python3 -m test_move.py`. The module form also supports the scripts' `src.Utils` imports.

## Gazebo simulation workflow

For simulation, skip arm initialization, physical gripper activation, and the pendant External Control program. Use the following launches **instead of** the hardware driver and hardware MoveIt launches. Stop any hardware session first and ensure these terminals use the intended simulation ROS master.

**Terminal 1 — Gazebo:**

```bash
cd ~/ur5e_ws
source setup_project.sh
roslaunch ur_gazebo ur5e_bringup.launch
```

**Terminal 2 — MoveIt for Gazebo:**

```bash
cd ~/ur5e_ws
source setup_project.sh
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
```

**Terminal 3 — RViz:** use the same RViz commands as step 6.

**Terminal 4 — arm test:** use the setup and `test_move` command from step 8 after the simulator and MoveIt are ready. Gazebo must be unpaused for simulated motion to advance.

The `sim:=true` argument selects the simulation controller configuration; it does not start Gazebo. See the upstream [MoveIt launch file](https://github.com/ros-industrial/universal_robot/blob/noetic-devel/ur5e_moveit_config/launch/moveit_planning_execution.launch) and [UR5e Gazebo launch file](https://github.com/ros-industrial/universal_robot/blob/noetic-devel/ur_gazebo/launch/ur5e_bringup.launch).

**The current `pick_place` script is hardware-specific:** it opens a TCP connection to the physical gripper at `172.22.22.2:63352`. Starting Gazebo does not redirect that connection to a simulated gripper. Use the arm-only test for this simulation workflow.

## Common startup problems

| Symptom | What to check |
| --- | --- |
| `install_isolated/setup.bash` is missing | Build successfully with `catkin_make_isolated --install`; confirm the workspace path matches `setup_project.sh`. |
| `roslaunch` cannot find a package | Source the setup script in that terminal; verify the package is installed or built in the sourced workspace. |
| Driver cannot connect to the robot | Check Ethernet, controller IP, desktop subnet, and robot power. `ping -c 4 172.22.22.2` is a basic connectivity check, not a complete driver test. |
| No “ready to receive control commands” message | Check that the External Control program is running and its installation points to the desktop IP; inspect driver and pendant errors. |
| RViz shows the robot but execution fails | Check the External Control program, active trajectory controller, pendant state, and speed slider. Visualization alone does not confirm motion readiness. |
| Python cannot import `src` | Run the dotted module command from `~/ur5e_ws`. |
| Python cannot import a ROS or project dependency | Check the selected interpreter, virtual environment, successful workspace build, and setup-script output. |
| Gripper readiness check fails | Check activation, faults, and the configured gripper ID against the pendant. |

## End the session

Let commanded motion finish, stop the pendant External Control program, then stop the Python and launch processes with **Ctrl+C** in their terminals. Follow the lab's normal arm power-off and controller shutdown procedure when finished. For an unsafe condition, use the lab's emergency-stop procedure rather than relying on terminal shutdown.
