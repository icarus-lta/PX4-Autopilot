# Airships

<LinkedBadge type="warning" text="Experimental" url="../airframes/#experimental-vehicles"/>

:::warning
Support for airships is [experimental](../airframes/index.md#experimental-vehicles).
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!
:::

PX4 supports three airship geometries:

- **Generic airship:** <Badge type="tip" text="PX4 v1.18" /> two forward thrusters and a cruciform tail with elevators and rudders.
- **Generic airship (independent vectoring):** <Badge type="tip" text="PX4 v1.18" /> two side pods, each a propeller on its own tilt servo.
- **Cloudship:** starboard, port and tail thrusters with a thrust tilt servo.

The frame configurations are shown in [Airframes Reference > Airship](../airframes/airframe_reference.md#airship).

## Simulation

PX4 provides synthetic simulation models for [Gazebo](../sim_gazebo_gz/index.md) of both airships.
Each is a family of models named by the fluid physics they carry, `<vehicle>[_am][_fin][_drag]`, described in the [Airship](../sim_gazebo_gz/vehicles.md#airship) section of the vehicle list:

- [Generic Airship (Independent Vectoring)](../sim_gazebo_gz/vehicles.md#generic-airship-independent-vectoring), and its [Hull Drag](../sim_gazebo_gz/vehicles.md#generic-airship-independent-vectoring-hull-drag) variant
- [Cloudship](../sim_gazebo_gz/vehicles.md#cloudship), with its [Hull Drag](../sim_gazebo_gz/vehicles.md#cloudship-hull-drag), [Fins](../sim_gazebo_gz/vehicles.md#cloudship-fins), [Added Mass](../sim_gazebo_gz/vehicles.md#cloudship-added-mass), [Added Mass, Fins](../sim_gazebo_gz/vehicles.md#cloudship-added-mass-fins) and [Complete](../sim_gazebo_gz/vehicles.md#cloudship-complete) variants

Running an airframe against several of its plants separates what the control loop owes to the model from what it owes to itself.

The Cloudship also has an older [Gazebo Classic](../sim_gazebo_classic/index.md) model, which uses a different allocation and is not kept in step with the flight airframe.
