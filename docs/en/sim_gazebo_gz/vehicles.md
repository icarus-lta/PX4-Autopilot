# Gazebo Vehicles

This topic lists/displays the vehicles supported by the PX4 [Gazebo](../sim_gazebo_gz/index.md) simulation, and the `make` commands required to run them (the commands are run from a terminal in the **PX4-Autopilot** directory).

The models are included in PX4 as a submodule that is fetched from the [Gazebo Models Repository](../sim_gazebo_gz/gazebo_models.md).

Supported vehicle types include: mutirotor, VTOL, Plane, Rover, Airship.

:::warning
See [Gazebo Classic Vehicles](../sim_gazebo_classic/vehicles.md) for vehicles that work with the older [Gazebo "Classic" simulation](../sim_gazebo_classic/index.md).
Note that vehicle models are not interchangeable between the two versions of the simulator: the vehicles on this page only work with (new) [Gazebo](../sim_gazebo_gz/index.md).
:::

## Multicopter

### X500 Quadrotor

```sh
make px4_sitl gz_x500
```

### X500 Quadrotor with Visual Odometry

```sh
make px4_sitl gz_x500_vision
```

![x500 in Gazebo](../../assets/simulation/gazebo/vehicles/x500.png)

### X500 Quadrotor with Depth Camera (Front-facing)

This model has a forward-facting depth camera attached, modelled on the [OAK-D](https://shop.luxonis.com/products/oak-d).

```sh
make px4_sitl gz_x500_depth
```

![x500 with depth camera in Gazebo](../../assets/simulation/gazebo/vehicles/x500_depth.png)

### X500 Quadrotor with Monocular Camera

This model has a simple monocular camera sensor attached (there is no physical camera visualization on the model itself).

```sh
make px4_sitl gz_x500_mono_cam
```

::: info
The camera cannot yet be used to stream video or for image capture in QGroundControl.
[PX4-Autopilot#22563](https://github.com/PX4/PX4-Autopilot/issues/22563) can be used to track the additional work needed to fully enable these use cases.
:::

### X500 Quadrotor with Monocular Camera (Down-facing)

This model has a simple monocular camera sensor attached facing down (there is no physical camera visualization on the model itself).

This can be used with the [Aruco world](../sim_gazebo_gz/worlds.md#aruco) to test precision landing.

```sh
make px4_sitl gz_x500_mono_cam_down
```

### X500 Quadrotor with 1D LIDAR (Down-facing)

This model has a LIDAR attached to the bottom, modelled on the [Lightware LW20/C](../sensor/sfxx_lidar.md).

It has a range between 0.1 and 100m.

The model can be used for testing [rangefinder](../sensor/rangefinders.md) use cases like [landing](../flight_modes_mc/land.md) or [terrain following](../flying/terrain_following_holding.md).

```sh
make px4_sitl gz_x500_lidar_down
```

![x500 with down-facing 1D LIDAR in Gazebo](../../assets/simulation/gazebo/vehicles/x500_lidar_down.png)

### X500 Quadrotor with 1D LIDAR (Front-facing)

This model has a LIDAR attached to the front, modelled on the [Lightware LW20/C](../sensor/sfxx_lidar.md).

It has a range between 0.2 and 100m.

The model can be used for testing [Collision Prevention](../computer_vision/collision_prevention.md#gazebo-simulation).

```sh
make px4_sitl gz_x500_lidar_front
```

![x500 with front-facing 1D LIDAR in Gazebo](../../assets/simulation/gazebo/vehicles/x500_lidar_front.png)

### X500 Quadrotor with 2D LIDAR

This model has a 2D LIDAR attached, modelled on the [Hokuyo UTM-30LX](https://www.hokuyo-aut.jp/search/single.php?serial=169).
It has a range between 0.1 and 30m, and scans in a 270° arc.
The model can be used for testing [Collision Prevention](../computer_vision/collision_prevention.md#gazebo-simulation).

```sh
make px4_sitl gz_x500_lidar_2d
```

![x500 with 2D LIDAR in Gazebo](../../assets/simulation/gazebo/vehicles/x500_lidar_2d.png)

::: info
The sensor information is written to the [ObstacleDistance](../msg_docs/ObstacleDistance.md) UORB message used by collision prevention.
:::

### X500 Quadrotor with Gimbal (Front-facing)

This model has a [gimbal](../advanced/gimbal_control.md) attached to the front with angular ranges of

- roll: [- $\frac{\pi}{4}$, $\frac{\pi}{4}$]
- pitch: [- $\frac{3\pi}{4}$, $\frac{\pi}{4}$]
- yaw: infinite rotation

The gimbal joints uses position control with a kinematic chain ZXY.

According to the file [Gimbal model.sdf file](https://github.com/PX4/PX4-gazebo-models/blob/main/models/gimbal/model.sdf):

- the default horizontal field of view is 2.0 rad ~= 115°
- the default vertical field of view is 0.8848 rad ~= 50.7°

![Quadrotor(x500) with gimbal (Front-facing) in Gazebo](../../assets/simulation/gazebo/vehicles/x500_gimbal.png).

```sh
make px4_sitl gz_x500_gimbal
```

## Plane/Fixed-wing

### Standard Plane

```sh
make px4_sitl gz_rc_cessna
```

![Plane in Gazebo](../../assets/simulation/gazebo/vehicles/rc_cessna.png)

### Advanced Plane

<Badge type="tip" text="PX4 v1.15" />

```sh
make px4_sitl gz_advanced_plane
```

![Advanced Plane in Gazebo](../../assets/simulation/gazebo/vehicles/advanced_plane.png)

::: info
The difference between the Advanced Plane and the "regular plane" lies in the Lift Physics that the two models use:

- You can configure the _Advanced Lift Drag_ plugin used by the model to more closely match a particular vehicle using the [Advanced Lift Drag Tool](../sim_gazebo_gz/tools_avl_automation.md).
- For more detail on the lift calculations for the Advanced Plane, see [PX4-SITL_gazebo-classic/src/liftdrag_plugin/README.md](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/liftdrag_plugin/README.md)

:::

## VTOL

### Standard VTOL

```sh
make px4_sitl gz_standard_vtol
```

![Standard VTOL in Gazebo Classic](../../assets/simulation/gazebo/vehicles/standard_vtol.png)

### Quad Tailsitter VTOL

A VTOL tailsitter model that uses differential thrust for pitch, roll, and yaw control.

```sh
make px4_sitl gz_quadtailsitter
```

![VTOL quad tailsitter in Gazebo](../../assets/simulation/gazebo/vehicles/vtol_quad_tailsitter.png)

### Tiltrotor VTOL

A VTOL Plane, where during the transition the front two motors will tilt forward and be used for forward thrust.

```sh
make px4_sitl gz_tiltrotor
```

![VTOL Tiltrotor in Gazebo](../../assets/simulation/gazebo/vehicles/vtol_tiltrotor.png)

## Rover

### Differential Rover

[Differential Rover](../frames_rover/index.md#differential) uses the [rover world](../sim_gazebo_gz/worlds.md#rover) by default.

```sh
make px4_sitl gz_rover_differential
```

![Differential Rover in Gazebo](../../assets/simulation/gazebo/vehicles/rover_differential.png)

### Ackermann Rover

[Ackermann Rover](../frames_rover/index.md#ackermann) uses the [rover world](../sim_gazebo_gz/worlds.md#rover) by default.

```sh
make px4_sitl gz_rover_ackermann
```

![Ackermann Rover in Gazebo](../../assets/simulation/gazebo/vehicles/rover_ackermann.png)

### Mecanum Rover

[Mecanum Rover](../frames_rover/index.md#mecanum) uses the [rover world](../sim_gazebo_gz/worlds.md#rover) by default.

```sh
make px4_sitl gz_rover_mecanum
```

![Mecanum Rover in Gazebo](../../assets/simulation/gazebo/vehicles/rover_mecanum.png)

## Airship

Airships get all of their weight support from the [Buoyancy](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Buoyancy.html) system, which is a world plugin, so all of them use the [lta world](../sim_gazebo_gz/worlds.md#lta).

Each airship is a family of models named by the fluid physics they carry, in a fixed order: `<vehicle>[_am][_fin][_drag]`.
The bare name is the rigid hull with buoyancy and no other fluid physics, `_am` adds the hull's fluid added mass, `_fin` adds fin lift, and `_drag` adds hull drag and rotational damping.
The Generic Airship is the exception: its fins are its flaps, so every member carries them and its family has no `_fin` token.
Every variant except `_am` is a thin merge include of the model with one token fewer, so the vehicle body is written once, plus the `_am` copy: `<fluid_added_mass>` lives in the link's inertial, which a plain merge include cannot change.
The airframe number's ones digit is a mask of the name's tokens: 1 for `_drag`, 2 for `_fin`, 4 for `_am`, summed, so the Generic Airship uses 2500, 2501, 2504 and 2505 only.

Running one control loop against several plants separates what it owes to the model from what it owes to itself.
Regress the allocator on the rigid hull; tune the loop on the complete one.
World wind reaches only the fins' lift, not the hull's drag or added mass, so disturb these models with an external force rather than the world's wind.

### Generic Airship

[Generic Airship](../frames_airship/index.md) as its flight airframe declares it: two fixed forward thrusters and four fin flaps, two elevators and two rudders, on a rigid hull that meets no air.
The flaps are its only attitude authority, and like real ones they give nothing until the hull moves, so fly it forward first.
Nothing on the rigid hull limits that speed; the Hull Drag members settle at one.
On the rigid hull the fins are the only yaw moment, so in a steady turn they carry no net side force: a held rudder sets a sideslip rather than a turn, and hull drag adds a weak one.
The Added Mass members turn because there the fins must also hold the hull's Munk moment.
In pitch, likewise, a held elevator makes heave rather than holding an attitude.
Since the fins are the actuators, every member of this family carries them and there is no `_fin` variant.

```sh
make px4_sitl gz_airship_fixed
```

### Generic Airship (Hull Drag)

The same vehicle with hull drag and rotational damping.

```sh
make px4_sitl gz_airship_fixed_drag
```

### Generic Airship (Added Mass)

The same vehicle with the hull's fluid added mass, which brings the Munk moment, and the tail that was sized against it.
No drag.

```sh
make px4_sitl gz_airship_fixed_am
```

### Generic Airship (Complete)

Everything the hull has: added mass, hull drag and rotational damping.
This is the plant to fly it on.

```sh
make px4_sitl gz_airship_fixed_am_drag
```

### Generic Airship (Independent Vectoring)

The same hull with two independently tilting thrust pods instead of the flaps: a torque keeps accelerating it and nothing damps a rotation.

```sh
make px4_sitl gz_airship_vectored_independent
```

### Generic Airship (Independent Vectoring, Hull Drag)

The same vehicle with hull drag and rotational damping.

```sh
make px4_sitl gz_airship_vectored_independent_drag
```

### Generic Airship (Independent Vectoring, Fins)

The same vehicle with its four fins as lifting surfaces, isolated from the other physics so that their contribution can be measured on its own.
The plant they exist for is the Added Mass, Fins variant, where they have the Munk moment to oppose.

```sh
make px4_sitl gz_airship_vectored_independent_fin
```

### Generic Airship (Independent Vectoring, Added Mass)

The same vehicle with the hull's fluid added mass and nothing to oppose the Munk moment it brings.
It exists to measure that term on its own.

```sh
make px4_sitl gz_airship_vectored_independent_am
```

### Generic Airship (Independent Vectoring, Added Mass, Fins)

Added mass and fin lift together, the Munk moment and the one thing on this hull that opposes it, with no drag.
With no hull drag, the fins are also all that damps a rate here.

```sh
make px4_sitl gz_airship_vectored_independent_am_fin
```

### Generic Airship (Independent Vectoring, Complete)

Everything: added mass, fin lift, hull drag and rotational damping.
This is the plant to tune the loops against.

```sh
make px4_sitl gz_airship_vectored_independent_am_fin_drag
```

### Cloudship

[Cloudship](../frames_airship/index.md): collective thrust on one tilt, with a reversible tail thruster for yaw.
The rigid hull.

```sh
make px4_sitl gz_cloudship
```

### Cloudship (Hull Drag)

The same vehicle with hull drag and rotational damping.

```sh
make px4_sitl gz_cloudship_drag
```

### Cloudship (Fins)

The same vehicle with its four fins as lifting surfaces, isolated from the other physics so that their contribution can be measured on its own.
The plant they exist for is the Added Mass, Fins variant, where they have the Munk moment to oppose.

```sh
make px4_sitl gz_cloudship_fin
```

### Cloudship (Added Mass)

The same vehicle with the hull's fluid added mass.
The physics engine folds added mass into the spatial inertia, Coriolis terms included, so this plant carries the Munk moment: a hull slipping sideways is turned further sideways, and nothing in this model opposes it.
It exists to measure that term on its own.

```sh
make px4_sitl gz_cloudship_am
```

### Cloudship (Added Mass, Fins)

Added mass and fin lift together, the Munk moment and the one thing on this hull that opposes it, with no drag.
With no hull drag, the fins are also all that damps a rate here.

```sh
make px4_sitl gz_cloudship_am_fin
```

### Cloudship (Complete)

Everything: added mass, fin lift, hull drag and rotational damping.
This is the plant to tune the yaw loop against.

```sh
make px4_sitl gz_cloudship_am_fin_drag
```
