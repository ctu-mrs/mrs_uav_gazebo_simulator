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
<!-- - [one_drone_3dlidar](./tmux/one_drone_3dlidar) -->
<!-- - [one_drone_realsense](./tmux/one_drone_realsense) -->
<!-- - [three_drones](./tmux/three_drones) -->

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

* Run PX4 SITL and Mavros, and ensure that all ports are correctly linked with the Gazebo simulator

* Remove all subprocesses on exit

Vehicles are added to the simulation by calling the `spawn` service of the `mrs_drone_spawner`.
The service takes one string argument, which specifies the vehicle ID, type and sensor configuration.
Example: spawn a single vehicle with ID 1, type X500, with a down-facing laser rangefinder:

```bash
ros2 service call /mrs_drone_spawner/spawn mrs_msgs/srv/String "value: 1 --x500 --enable-rangefinder"
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
ros2 service call /mrs_drone_spawner/spawn mrs_msgs/srv/String "value: 1 --x500 --enable-rangefinder --enable-ouster model:=OS0-32 use_gpu:=True horizontal_samples:=128 update_rate:=10"
```
The list of components and their reconfigurable parameters can be displayed using the platform-specific help.

For more details, please refer to the [MRS drone spawner](https://ctu-mrs.github.io/docs/simulations/gazebo/drone_spawner) page.
