# MRS UAV Gazebo Simulator

This package provides UAV model definitions for the Gazebo simulator and a custom spawning mechanism that assembles the drones dynamically from provided arguments.


## ROS2

> :warning: **Attention please: This package is under development due to ROS2 transition.**
>
> Please keep in mind that the migration will take time and the components you are used to in ROS1 may not be available yet.

![thumbnail](.fig/thumbnail.jpg)


### Pre-defined UAVs
> :warning: **The following platforms have been ported to ROS2.**
> Only default pixhawk sensors and a garmin rangefinder are currently available.

| Model         | Spawn argument | Simulation                       |
|---------------|----------------|----------------------------------|
| Holybro x500  | `--x500`       | ![](.fig/x500_simulation.jpg)    |
| DJI f450      | `--f450`       | ![](.fig/f450_simulation.jpg)    |
| DJI f330      | `--f330`       | ![](.fig/f330_simulation.jpg)    |
| DJI f550      | `--f550`       | ![](.fig/f550_simulation.jpg)    |
| F4F RoboFly   | `--robofly`    | ![](.fig/robofly_simulation.jpg) |
| T-Drones m690 | `--m690`       | ![](.fig/m690_simulation.jpg)    |
| Tarot t650    | `--t650`       | ![](.fig/t650_simulation.jpg)    |
<!-- | NAKI II       | `--naki` | ![](.fig/naki_simulation.jpg) | -->



### Adding a custom UAV

A custom drone model can be added from an external package.
Please look at [mrs_gazebo_custom_drone_example](https://github.com/ctu-mrs/mrs_gazebo_custom_drone_example) for an example.
The [custom drone](https://ctu-mrs.github.io/docs/simulations/gazebo/custom_drone) wiki page contains a detailed description of all the important steps and configuration parts.

## Starting the simulation

Use one of the prepared Tmuxinator sessions in [`roscd mrs_uav_gazebo_simulator/tmux`](./tmux) as an example:

- [one_drone](./tmux/one_drone)
- [one_drone_ardupilot_claude](./tmux/one_drone_ardupilot_claude) (one drone controlled by ArduPilot SITL)
<!-- - [one_drone_3dlidar](./tmux/one_drone_3dlidar) -->
<!-- - [one_drone_realsense](./tmux/one_drone_realsense) -->
<!-- - [three_drones](./tmux/three_drones) -->

## ArduPilot SITL backend

The spawner supports two autopilot backends: **PX4** (default) and **ArduPilot SITL**. A robot
spawned with the `--use-ardupilot` keyword gets the [ArduPilotPlugin](https://github.com/ArduPilot/ardupilot_gazebo)
compiled from the `ardupilot_gazebo` package in its SDF and an ArduPilot SITL instance
(`arducopter --model JSON`) running instead of PX4. The rest of the pipeline (MAVROS,
`mrs_uav_core`, tmux sessions) stays the same; use `mrs_uav_ardupilot_api` instead of
`mrs_uav_px4_api` as the hardware API.

### Prerequisites

1. **ArduPilot SITL built** (provides the `arducopter` binary):

   ```bash
   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
   cd ~/ardupilot
   Tools/environment_install/install-prereqs-ubuntu.sh -y
   ./waf configure --board sitl && ./waf copter
   ```

   The spawner resolves the binary from `ardupilot_config/binary_path`, the `ARDUPILOT_HOME`
   environment variable, or `$HOME/ardupilot/build/sitl/bin/arducopter` by default.

2. **The `ardupilot_gazebo` package built in this workspace.** Its ament hooks export
   `GZ_SIM_SYSTEM_PLUGIN_PATH`/`GZ_SIM_RESOURCE_PATH` automatically when the workspace is sourced.

### Usage

```bash
ros2 service call /mrs_drone_spawner/spawn mrs_msgs/srv/String "value: 1 --x500 --use-ardupilot"
```

or run the prepared session:

```bash
cd $(ros2 pkg prefix mrs_uav_gazebo_simulator)/share/mrs_uav_gazebo_simulator/tmux/one_drone_ardupilot_claude
./start.sh
```

### How it works

* The x500 SDF template swaps the PX4 `MulticopterMotorModel` propellers for lift-drag
  propelled propellers and adds the `ArduPilotPlugin` (JSON FDM backend).
* The spawner launches MAVROS (with the ArduPilot plugin-list from `mrs_uav_ardupilot_api`)
  and `arducopter --model JSON -I<ID> --sysid <ID+1> --defaults <parm>` per robot.
* Ports follow the ArduPilot multi-instance convention `base + 10 * UAV_ID`
  (configurable in `config/spawner_params.yaml` under `ardupilot_config`):

  | Purpose | Port (`UAV_ID = 1`) |
  |---|---|
  | JSON FDM (SITL <-> Gazebo plugin) | 9002+10 = **9012** |
  | MAVROS link (TCP, SITL GCS server) | 5760+10 = **5770** |
  | QGroundControl (optional, UDP) | 14570 |

* The SITL `--home` coordinate has to match the MRS `world_origin`
  (default `47.397743,8.545594,340,0`, see `tmux/*/config/world_config.yaml`).
* The default SITL parameter file is [`config/ardupilot/mrs_copter_defaults.parm`](config/ardupilot/mrs_copter_defaults.parm)
  (Quad-X frame, `GUID_OPTIONS=8` for pass-through thrust, `FS_THR_ENABLE=0`,
  `DISARM_DELAY=0`, battery monitoring). The MRS x500 propeller order matches the
  ArduCopter Quad X motor order 1:1.

### Limitations

* Only the **x500** template currently supports the ArduPilot backend (other platforms need
  the same propeller swap; the macros in `generic_components.sdf.jinja` are reusable).
* The Garmin rangefinder (`--enable-rangefinder`) is PX4-only. The SITL JSON backend does
  not consume Gazebo rangefinders in this setup; use `--enable-rangefinder-external` if you
  only need the ROS topic.
* GPS comes from ArduPilot SITL's own sensor model (fed by the true model pose), not from
  the Gazebo NavSat sensor.
* `mrs_uav_ardupilot_api` implements attitude/attitude-rate command inputs only
  (position/velocity/acceleration commands are stubs). The default MRS control pipeline
  outputs attitude commands, which is sufficient for normal flight.

## Using the MRS drone spawner in your simulations

The drone models are dynamically created in runtime using the [MRS drone spawner](https://ctu-mrs.github.io/docs/simulations/gazebo/drone_spawner). The UAV platforms can be additionally equipped by adding [components](models/mrs_robots_description/sdf/component_snippets.sdf.jinja) (rangefinders, LiDARs, cameras, plugins etc.).

### Start the Gazebo simulator

To start the example Gazebo world call:

```bash
ros2 launch mrs_uav_gazebo_simulator simulation.launch.py world_file:=$(ros2 pkg prefix mrs_gazebo_common_resources)/share/mrs_gazebo_common_resources/worlds/grass_plane.sdf gz_headless:=false
```

At this point the Gazebo world will only contain the environment with grass plane but with no vehicles yet.

### Spawning the UAVs

The `simulation.launch.py` will automatically start the `mrs_drone_spawner` as a ROS2 node. If you use a custom launch file to start Gazebo, you can launch the spawner separately:

```bash
ros2 launch mrs_uav_gazebo_simulator mrs_drone_spawner.launch.py
```

The `mrs_drone_spawner` will perform the following tasks:

* Generate SDF models from the UAV templates

* Add optional components (sensors, plugins...) based on the user input

* Run PX4 SITL and Mavros (or ArduPilot SITL with `--use-ardupilot`), and ensure that all ports are correctly linked with the Gazebo simulator

* Remove all subprocesses on exit

Vehicles are added to the simulation by calling the `spawn` service of the `mrs_drone_spawner`.
The service takes one string argument, which specifies the vehicle ID, type and sensor configuration.
Example: spawn a single vehicle with ID 1, type X500, with a down-facing laser rangefinder:

```bash
ros2 service call /mrs_drone_spawner/spawn mrs_msgs/srv/String "value: 1 --x500 --enable-rangefinder"
```

To use ArduPilot SITL instead of PX4, add `--use-ardupilot` (only x500 is currently supported):

```bash
ros2 service call /mrs_drone_spawner/spawn mrs_msgs/srv/String "value: 1 --x500 --use-ardupilot"
```

To display the basic use manual for the spawner, call the service with the argument ` --help`. **NOTE**: String argument cannot start with a dash. Add a space before the dashes to avoid errors. The service call returns the full help text, but the formatting may be broken. Please refer to the terminal running `simulation` or `mrs_drone_spawner` where the help text is also printed with proper formatting.

```bash
ros2 service call /mrs_drone_spawner/spawn mrs_msgs/srv/String "value: --help"
```

You can also display a manual for a specific platform. This will list all the components that can be equipped to the selected platform, and their brief description.
```bash
ros2 service call /mrs_drone_spawner/spawn mrs_msgs/srv/String "value: --x500 --help"
```

Multiple vehicles may be spawned with one service call:
```bash
ros2 service call /mrs_drone_spawner/spawn mrs_msgs/srv/String "value: 1 2 3 4 5 --t650 --enable-bluefox-camera --enable-rangefinder"
```

The default parameters of some components may be reconfigured by adding `param:=value` after the component keyword. Multiple params may be used at the same time:
```bash
ros2 service call /mrs_drone_spawner/spawn mrs_msgs/srv/String "value: 1 --x500 --enable-rangefinder --enable-ouster model:=OS0-32 horizontal_samples:=128 update_rate:=10"
```
The list of components and their reconfigurable parameters can be displayed using the platform-specific help.

For more details, please refer to the [MRS drone spawner](https://ctu-mrs.github.io/docs/simulations/gazebo/drone_spawner) page.
