# Airships

<LinkedBadge type="warning" text="Experimental" url="../airframes/#experimental-vehicles"/>

:::warning
Support for airships is [experimental](../airframes/index.md#experimental-vehicles).
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!
:::

PX4 supports three airship geometries:

- **Generic airship:** <Badge type="tip" text="PX4 v1.18" /> two forward thrusters and a cruciform tail with elevators and rudders.
- **Generic airship (independent vectoring):** <Badge type="tip" text="main (PX4 v2.0)" /> two side pods, each a propeller on its own tilt servo.
- **Cloudship:** starboard, port and tail thrusters with a thrust tilt servo.

The frame configurations are shown in [Airframes Reference > Airship](../airframes/airframe_reference.md#airship).

## Configuration

The [standard configuration](../config/index.md) covers the steps shared with other frames, such as sensor calibration, radio control and safety.
The airship-specific part is the actuator geometry and the yaw rate controller, covered here.

### Actuator Geometry

All three frame configurations select the airship [control allocator](../concept/control_allocation.md), [CA_AIRFRAME](../advanced_config/parameter_reference.md#CA_AIRFRAME) `16: Airship`.
It models two side propulsion pods, `Motor 1` starboard and `Motor 2` port, each a propeller on a tilt servo, an optional tail yaw thruster `Motor 3`, and control surfaces.
The outputs are declared in that order: the motors, then the control surfaces, then the tilt servos.
Assign them to outputs and test them in [Actuators](../config/actuators.md) as for any other frame.

The pod propellers are non-reversible.
The allocator keeps their motor commands non-negative and reverses the thrust only by tilting the pods through their range.
Leave the [CA_R_REV](../advanced_config/parameter_reference.md#CA_R_REV) bits of `Motor 1` and `Motor 2` (the **Bidirectional** checkbox in the actuator geometry) cleared: the allocator ignores them, but the output driver still treats such a motor as [bidirectional](../config/actuators.md#bidirectional-motors), so on a PWM output zero thrust is sent as the middle of the output range, about half throttle on a one-way ESC.

The geometry is described by the `CA_AIRSHIP_*` parameters.
Each airframe sets the ones of its layout; measure the values of your vehicle and set them in the QGroundControl [Parameters](../advanced_config/parameters.md) screen:

1. [CA_AIRSHIP_GRP](#CA_AIRSHIP_GRP): How the pods are driven.
   `Collective` drives both pods with one thrust command and one tilt command on a single tilt servo; there is no differential thrust, so the pods produce no yaw or roll torque.
   `Independent` gives each pod its own thrust and tilt command on its own tilt servo, and the differential thrust makes yaw and roll torque.
   The pods never produce pitch torque.
   A torque axis the pods do not serve falls to the control surfaces and, for yaw, the tail thruster; with neither it is reported as unallocated.
2. [CA_AIRSHIP_TLMIN](#CA_AIRSHIP_TLMIN) / [CA_AIRSHIP_TLMAX](#CA_AIRSHIP_TLMAX) [deg]: Measure the pod tilt angle at the two end stops of the tilt servo, at minimum and at maximum servo output.
   Zero is thrust forward and positive tilts the thrust up.
   Setting both to zero fixes the pods thrusting forward and allocates no tilt servo; the ground station's actuator list still shows the pod tilt servos after the control surfaces, which the firmware never drives.
3. [CA_AIRSHIP_TLT_R](#CA_AIRSHIP_TLT_R) [deg/s]: Measure the slew rate of the tilt servo and set it here; zero disables the limit.
   The allocator projects the pod thrust onto the tilt angle the servo has reached, which is why the limit belongs to this parameter.

   ::: warning
   Do not slew the tilt servo with `CA_SVn_SLEW`.
   That slew runs after the airship model and would desynchronise the thrust projection from the actual servo position.
   :::

4. [CA_AIRSHIP_TAIL](#CA_AIRSHIP_TAIL): Enable the tail yaw thruster, `Motor 3`.
   It serves the yaw the pods and the control surfaces leave unmet, one unit of motor command per unit of normalised yaw torque; there is no moment-arm parameter.
   Make it reversible with [CA_R_REV](../advanced_config/parameter_reference.md#CA_R_REV) bit 2 (`Motor 3`), otherwise it can push only one way; the output driver and the ESC must support reversible motors as well (see [Bidirectional Motors](../config/actuators.md#bidirectional-motors)).
5. [CA_AIRSHIP_CS_K](#CA_AIRSHIP_CS_K): The fraction of the control-surface torque allocation the allocator trusts to be delivered aerodynamically; the pods and the tail serve the rest of the demand.
   The surfaces themselves are configured as on a fixed-wing vehicle ([Control Surfaces Geometry](../config/actuators.md#control-surfaces-geometry)).
   The credit has no airspeed scaling: at the default of 1 the surfaces serve their axes alone up to their travel, and at rest the allocator credits them torque that still air does not deliver.
   At 0 they still deflect, but the pods and the tail serve the whole demand.
   With yaw surfaces, the [yaw rate controller](#yaw-rate-controller) closes its loop only below 1.

The three frame configurations set the geometry as follows.

#### Generic Airship

`2500` fixes the thrusters forward (collective grouping, `CA_AIRSHIP_TLMIN` and `CA_AIRSHIP_TLMAX` 0) and declares the four fin flaps as control surfaces, two elevators and two rudders (`CA_SV_CS_COUNT` 4).
Pitch and yaw come from the surfaces alone, at the default credit of 1, and nothing else on the vehicle can serve either axis.
The yaw rate loop therefore stays open and the yaw stick is yaw torque in every mode.

#### Generic Airship (Independent Vectoring)

`2520` vectors the pods independently over a full 360° tilt range (`CA_AIRSHIP_TLMIN` -180, `CA_AIRSHIP_TLMAX` 180) at 120°/s.
Set `CA_AIRSHIP_TLMIN` and `CA_AIRSHIP_TLMAX` to the tilt angles at the servo end stops and `CA_AIRSHIP_TLT_R` to the servo slew rate of your vehicle.
Yaw comes from the pods' differential thrust and the yaw rate loop closes.

#### Cloudship

`2507` drives both thrusters collectively on one tilt servo over ±90° at 120°/s, with the tail thruster reversible (`CA_AIRSHIP_TAIL` 1, `CA_R_REV` 4).
The symmetric tilt range keeps level thrust at servo output 0.
The tilt range and the slew rate are assumed, not measured on a vehicle: measure them and set them.
Yaw comes from the tail and the yaw rate loop closes.

## Tuning

Roll, pitch and thrust are stick passthrough and have no controller to tune: the throttle stick is forward thrust, the roll stick is roll torque, and the pitch stick is both pitch torque, for elevators, and vertical thrust, for tilting pods.
An axis the frame cannot serve is reported as unallocated.
Yaw has the rate controller below.

### Yaw Rate Controller

In the manual modes with rate control, [Acro](../flight_modes_mc/acro.md), Stabilized, Altitude and Position, the yaw stick commands a yaw rate that a PI loop with setpoint feedforward closes on the measured yaw rate.
The loop closes while armed with valid manual input, on the ground included, and only where the propulsion gets yaw to make: independent pods (`CA_AIRSHIP_GRP`) or a tail thruster (`CA_AIRSHIP_TAIL`), with the yaw surfaces, if any, not credited in full (`CA_AIRSHIP_CS_K` below 1).
In Manual mode, in the non-manual modes, and on a frame whose yaw comes from control surfaces alone, the yaw stick is passed through as yaw torque and the `AS_*` parameters do nothing: the loop has no airspeed scaling.
In a mode that enables none of manual, rate and attitude control, such as [Offboard](../flight_modes/offboard.md) with thrust and torque setpoints, the controller publishes no thrust or torque setpoint, so the offboard setpoints drive the allocator alone.
The integrator stops in a direction the allocator reports it cannot serve, and is cleared whenever the loop opens.

Tune in this order:

1. [THR_MDL_FAC](../advanced_config/parameter_reference.md#THR_MDL_FAC): Linearise the motor thrust first.
   The gains work in normalised torque, which the motors deliver only if their thrust is linear in their command; the factor applies to every motor output, the reversible tail included.
   Measure the thrust curve of the motors that make yaw and set the factor as described in [Thrust Curve](../config_mc/pid_tuning_guide_multicopter.md#thrust-curve).
   The gain defaults below were checked on the Gazebo models, whose airframes set `THR_MDL_FAC` 1 because the simulated thrust is quadratic in the rotor speed command.
   The flight airframes leave it at the default of 0, so with a propeller whose thrust grows faster than its command, a unit of yaw command buys less torque at the low end than the gains were tuned for.
2. [AS_YAWRATE_MAX](#AS_YAWRATE_MAX) [deg/s]: The yaw rate that full yaw stick deflection commands.
   The stick passes through [MAN_DEADZONE](../advanced_config/parameter_reference.md#MAN_DEADZONE) first, so the centre band commands zero rate.
   To find the yaw rate the vehicle can reach, hold full yaw stick in Manual mode, where the stick is full yaw torque, and read the steady `xyz[2]` of [VehicleAngularVelocity](../msg_docs/VehicleAngularVelocity.md) from the log.
3. [AS_YAWRATE_P](#AS_YAWRATE_P): Proportional gain, the control output per rad/s of yaw rate error.

   ::: tip
   Fly in Acro mode and hold the yaw stick at a few different levels for a couple of seconds each.
   From the flight log plot `yaw` of [VehicleRatesSetpoint](../msg_docs/VehicleRatesSetpoint.md), which is published only while the loop is closed, over `xyz[2]` of [VehicleAngularVelocity](../msg_docs/VehicleAngularVelocity.md).
   Increase the gain if the measured rate does not track the setpoint fast enough, decrease it if the rate overshoots, and repeat until you are satisfied with the tracking.
   :::

4. [AS_YAWRATE_FF](#AS_YAWRATE_FF): Feedforward, the control output per rad/s of yaw rate setpoint.
   It supplies the torque that holds a turn against the hull's yaw damping.
   Without it the integrator has to build that torque up during the turn and unwind it afterwards, so a well-damped hull can keep turning for more than ten seconds after the stick is released.
   To measure it, set the feedforward to 0 and hold a steady turn in Acro mode in calm air until the measured rate reaches the setpoint, which takes tens of seconds on a well-damped hull.
   The integrator then holds the turn, and `yawspeed_integ` of [RateCtrlStatus](../msg_docs/RateCtrlStatus.md), averaged over one full 360° turn, divided by the turn rate in rad/s is about the value that holds that rate.
   If `yawspeed_integ` sits at [AS_YR_INT_LIM](#AS_YR_INT_LIM), the integrator cannot hold the turn: measure at a smaller rate, or raise the limit for the measurement.
   Turning without forward speed, the damping grows faster than linearly with the rate, so a value that holds one rate there overshoots at smaller ones.
   Leave it at 0 on a hull with little yaw damping.
5. [AS_YAWRATE_I](#AS_YAWRATE_I): Integral gain, which removes steady yaw disturbances such as wind.
6. [AS_YR_INT_LIM](#AS_YR_INT_LIM): Integrator limit, in normalised yaw torque.
   The integrator is logged as `yawspeed_integ` of [RateCtrlStatus](../msg_docs/RateCtrlStatus.md), published while the loop is closed and once more when it opens.
   Raise the limit if the integrator sits at it against a steady disturbance; lower it to shorten the settling after large yaw trim changes.

The defaults were tuned on the Gazebo models of the Generic Airship (Independent Vectoring), rigid, with fins and complete, and of the Cloudship, rigid and complete.
The feedforward is set per model in their airframes: 1.0 on the complete vectored airship, 0.3 on its fin variant and 0.05 on the complete Cloudship; the other models and the flight airframes leave it at 0.
Treat the values as a starting point and retune on the vehicle.

### Parameter Overview

| Parameter                                                                                                   | Description                             | Unit  |
| ----------------------------------------------------------------------------------------------------------- | --------------------------------------- | ----- |
| <a id="CA_AIRSHIP_GRP"></a>[CA_AIRSHIP_GRP](../advanced_config/parameter_reference.md#CA_AIRSHIP_GRP)       | Pod grouping: collective or independent | -     |
| <a id="CA_AIRSHIP_TLMIN"></a>[CA_AIRSHIP_TLMIN](../advanced_config/parameter_reference.md#CA_AIRSHIP_TLMIN) | Tilt angle at minimum servo output      | deg   |
| <a id="CA_AIRSHIP_TLMAX"></a>[CA_AIRSHIP_TLMAX](../advanced_config/parameter_reference.md#CA_AIRSHIP_TLMAX) | Tilt angle at maximum servo output      | deg   |
| <a id="CA_AIRSHIP_TLT_R"></a>[CA_AIRSHIP_TLT_R](../advanced_config/parameter_reference.md#CA_AIRSHIP_TLT_R) | Tilt slew rate limit                    | deg/s |
| <a id="CA_AIRSHIP_TAIL"></a>[CA_AIRSHIP_TAIL](../advanced_config/parameter_reference.md#CA_AIRSHIP_TAIL)    | Tail yaw thruster                       | -     |
| <a id="CA_AIRSHIP_CS_K"></a>[CA_AIRSHIP_CS_K](../advanced_config/parameter_reference.md#CA_AIRSHIP_CS_K)    | Control-surface torque credit           | -     |
| <a id="AS_YAWRATE_MAX"></a>[AS_YAWRATE_MAX](../advanced_config/parameter_reference.md#AS_YAWRATE_MAX)       | Yaw rate at full stick                  | deg/s |
| <a id="AS_YAWRATE_P"></a>[AS_YAWRATE_P](../advanced_config/parameter_reference.md#AS_YAWRATE_P)             | Yaw rate proportional gain              | -     |
| <a id="AS_YAWRATE_I"></a>[AS_YAWRATE_I](../advanced_config/parameter_reference.md#AS_YAWRATE_I)             | Yaw rate integral gain                  | -     |
| <a id="AS_YR_INT_LIM"></a>[AS_YR_INT_LIM](../advanced_config/parameter_reference.md#AS_YR_INT_LIM)          | Yaw rate integrator limit               | -     |
| <a id="AS_YAWRATE_FF"></a>[AS_YAWRATE_FF](../advanced_config/parameter_reference.md#AS_YAWRATE_FF)          | Yaw rate feedforward                    | -     |

## Simulation

PX4 provides synthetic [Gazebo](../sim_gazebo_gz/index.md) simulation models for all three geometries.
Each is a family of models named by the fluid physics they carry, `<vehicle>[_am][_fin][_drag]`, described in the [Airship](../sim_gazebo_gz/vehicles.md#airship) section of the vehicle list:

- [Generic Airship](../sim_gazebo_gz/vehicles.md#generic-airship), the 2500 airframe with fixed thrusters and fin flaps, with its [Hull Drag](../sim_gazebo_gz/vehicles.md#generic-airship-hull-drag), [Added Mass](../sim_gazebo_gz/vehicles.md#generic-airship-added-mass) and [Complete](../sim_gazebo_gz/vehicles.md#generic-airship-complete) variants
- [Generic Airship (Independent Vectoring)](../sim_gazebo_gz/vehicles.md#generic-airship-independent-vectoring), with its [Hull Drag](../sim_gazebo_gz/vehicles.md#generic-airship-independent-vectoring-hull-drag), [Fins](../sim_gazebo_gz/vehicles.md#generic-airship-independent-vectoring-fins), [Added Mass](../sim_gazebo_gz/vehicles.md#generic-airship-independent-vectoring-added-mass), [Added Mass, Fins](../sim_gazebo_gz/vehicles.md#generic-airship-independent-vectoring-added-mass-fins) and [Complete](../sim_gazebo_gz/vehicles.md#generic-airship-independent-vectoring-complete) variants
- [Cloudship](../sim_gazebo_gz/vehicles.md#cloudship), with its [Hull Drag](../sim_gazebo_gz/vehicles.md#cloudship-hull-drag), [Fins](../sim_gazebo_gz/vehicles.md#cloudship-fins), [Added Mass](../sim_gazebo_gz/vehicles.md#cloudship-added-mass), [Added Mass, Fins](../sim_gazebo_gz/vehicles.md#cloudship-added-mass-fins) and [Complete](../sim_gazebo_gz/vehicles.md#cloudship-complete) variants

Running an airframe against several of its plants separates what the control loop owes to the model from what it owes to itself.

The Cloudship also has an older [Gazebo Classic](../sim_gazebo_classic/index.md) model, which uses a different allocation and is not kept in step with the flight airframe.
