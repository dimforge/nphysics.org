# Performance and accuracy tuning
Unfortunately in the world of physics simulation, performance and accuracy are not always compatible. It is often
necessary to sacrifice performance for accuracy and vice versa. For example, video games will often favor performance
to simulate a visually-appealing but not totally realistic scene. On the other hand, higher accuracy is often needed
for, e.g., virtual training applications where the accuracy of the simulation directly affects the quality of the
training. This section discusses several parameters that may be worth tuning to achieve the best performance/accuracy
compromise for your application.

## Choice of contact model
Contact models control the nature and number of constraints generated for each contact. Therefore, the choice of a
contact model adapted to your simulation strongly affects the overall performance and accuracy. Contact models are
presented in a dedicated [chapter](contact_models.md) of this guide. As a rule of thumb, you should:

* Use the `SignoriniModel` if you only care about non-penetration (and don't care about friction).
* Use the `SignoriniCoulombPyramidModel` for handling friction as well.

## Integration parameters
Various aspects of the integrator implemented on **nphysics** can be modified through the
[integration parameters](/rustdoc/nphysics3d/solver/struct.IntegrationParameters.html). Those parameters can be modified
by retrieving a mutable reference to the `IntegrationParameters` structure associated to a mechanical world:
`mechanical_world.integration_parameters`. Most of those parameters are somewhat advanced and should not be modified
unless you know their meaning and effect. Several of them are about tuning parameters for optimal simulation stability,
or for compromising between efficiency and accuracy. They are given default values that work well in the context of
video-games or animations. For more realistic simulations you may want to change those parameters to favor accuracy over
performance:


| Field                 | Description                                                           |
|--                      | --                                                                    |
| `dt` | The timestep used for each update of the physics engine. The default is $1 / 60$ seconds. This typically corresponds to a refresh rate of 60Hz. Smaller timesteps yield better accuracy of the integrator. Large timesteps increase the negative effect of some approximations (linearization of various parts of the equations of motion) and may result in missed collisions (because of collisions that may occur in-between timesteps for fast-moving objects). |
| `erp` | The Error Reduction Parameter in $[0, 1]$ is the proportion of the positional error to be corrected at each time step. The default is $0.2$. An ERP set to 0 means that no penetration will be corrected, and an ERP set to 1 means all the penetrations between two solids will be corrected in a single timestep. While setting 1 seems appealing, this will cause objects to jitter in practice because penetration correction may add or remove kinetic energy to the system. |
| `warmstart_coeff` | Each cached impulse are multiplied by this coefficient in $[0, 1]$ when they are re-used to initialize the constraints solver. The default is $1.0$. A high warm-start coefficient causes faster convergence but may add kinetic energy to the system (and cause jitters) for fast objects subject to non-elastic contacts. However $1.0$ still remains a good choice for practical applications as it allows convergence of the constraints solver even if the maximum number of solver iterations is small. |
| `restitution_velocity_threshold` | Contacts at points where the involved bodies have a relative velocity smaller than this threshold wont be affected by the restitution force. The default is $1.0$. |
| `allowed_linear_error` | Amount of penetration the engine wont attempt to fix. The default is $0.001$ meters. Setting this to 0 would cause jittering because contacts would start and stop being active at a high frequency for resting objects (due to inevitable numerical errors). A value too large would cause noticeable penetrations. |
| `allowed_angular_error` | Amount of angular drift for joint limits the engine wont attempt to correct. The default is $0.001$ radians. This is similar to `allowed_linear_error` but for angular joint limits. |
| `max_linear_correction` | Maximum translation-based penetration correction performed by the non-linear position solver during a single iteration. The default is $100.0$. A value too small would prevent penetrations from being corrected fast enough while a large value could cause jittering due to the solver overshooting (i.e. applying a correction that is too large). |
| `max_angular_correction` | Maximum rotation-based penetration correction during one step of the non-linear position solver. The default is $0.2$ radians. A value too small would make positional correction unrealistic because most of it would be using translations only. A value too large would cause the solver to apply rotations that are large enough to disturb penetration correction of other contact points from the same contact manifold. |
| `max_stabilization_multiplier` | Maximum non-linear position-based penetration correction scaling parameter when the constraint correction direction is close to the kernel of the involved multibody's jacobian. The default is $0.2$. This is used when the position correction requires a motion that is not feasible for a given multibody. In those situations, the solver would attempt to apply a correction that is too large, causing the multibody joints to change position significantly. |
| `max_velocity_iterations` | Maximum number of iterations performed by the velocity constraints solver for impulse computation. More iterations imply greater accuracy at the cost of increased computation times. |
| `max_position_iterations` | Maximum number of iterations performed by the position-based constraints solver for penetration correction. More iterations imply greater accuracy at the cost of increased computation times. |
