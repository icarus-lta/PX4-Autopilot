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

[Gazebo](../sim_gazebo_gz/index.md) models are provided for the independently vectored airship:

- `make px4_sitl gz_airship_vectored_independent` — rigid hull, no aerodynamic forces.
- `make px4_sitl gz_airship_vectored_independent_aero` — the same vehicle with hull drag and rotational damping.

Running the same airframe against both separates what the control loop owes to the model from what it owes to itself.
