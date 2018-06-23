# Rigid-body simulation with contacts

## Initializing the physics world
The [`World`](/rustdoc/nphysics3d/world/struct.World.html) structure is at the core of **nphysics**. It will contain all the objects to be handled by the physics engine and will perform all the collision detection, dynamics integrations, and event generation. It is easily initialized with `World::new()`. This constructor will initialize the physics engine with all its default parameters, including:

* A gravity set to zero. This can be changed by, e.g., `world.set_gravity(Vector3::y() * -9.81)`.
* The initialization a `CollisionWorld` from the **ncollide** [crate](http://ncollide.org) to handle collision detection.
* The use of an impulse-based constraint solver to handle contacts. By default a Sinorini-Coulomb contact model with
polyhedral friction cones is used. Other contact models can be chosen afterward. See the dedicated [section](contact_models.md)).

Once you created the physics world, it will be empty. The next steps are then to add bodies like [rigid-bodies](/rigid_body_simulations_with_contacts/#rigid-bodies) or [multibody parts](/joint_constraints_and_multibodies/#multibodies) to which [colliders](/rigid_body_simulations_with_contacts/#colliders) or [sensors](/event_handling_and_sensors/#sensors) can be attached.

## Rigid-bodies
A rigid-body is the most simple type of body supported by **nphysics**. It can be seen as the aggregation of a position, orientation, and mass properties (rotational inertia tensor, mass, and center of mass). It does not holds any information regarding its shape which can optionally be specified by attaching one or multiple [colliders](/rigid_body_simulations_with_contacts/#colliders) to it. A rigid-body with no collider with be affected by all the forces the world is aware of, but not by contacts (because it does not have any shape that can be collided to).

### Adding a rigid-body to the world

### Getting a rigid-body from the world
A reference to a rigid-body added to the world can be retrieved from its handle. Three methods are available on the world:

* **`world.rigid_body(handle)`:** returns a reference to a [RigidBody](/rustdoc/nphysics3d/object/struct.RigidBody.html) structure if it has been found.
* **`world.body(handle)`:** returns a reference to a [Body](/rustdoc/nphysics3d/object/enum.Body.html) which itself is an enum encapsulating all the bodies supported by **nphysics** (currently rigid bodies and multibodies are supported).
* **`world.body_part(handle)`:** returns a reference to a [BodyPart](/rustdoc/nphysics3d/object/enum.Body.html) which itself is an enum encapsulating all the body parts supported by **nphysics**. Since a rigid-body is composed of only one piece, the `BodyPart::RigidBody` simply contains a reference to the `RigidBody` structure. The distinction between a _body_ and a _body part_ becomes apparent for [multibodies](/joint_constraints_and_multibodies/#multibodies).

The method to be chose depends on the context. `.rigid_body(...)` should cover most scenarios. The other methods are generally useful when writing code that operate independently from the actual variant of body or body part.

### Rigid-body statuses
A rigid-body can be four different statuses identified by the [`object::BodyStatus`](/rustdoc/nphysics3d/object/enum.BodyStatus.html) enum:

* **`BodyStatus::Dynamic`:** This is the default status. Indicates the rigid-body is affected by external forces, inertial forces (gyroscopic, coriolis, etc.) and contacts.
* **`BodyStatus::Static`:** Indicates the rigid-body does not move. It acts as if it has an infinite mass and won't be affected by any force. It will continue to collide with dynamic rigid-bodies but not with static and kinematic ones. This is typically used for temporarily freezing a rigid-body. Note that, as mentioned in the [next section](/rigid_body_simulations_with_contacts/#colliders) it is possible to add colliders attached to a special `BodyHandle::ground()` body. Therefore, objects that will never move should be simulated with colliders attached to the `BodyHandle::ground()` instead of with a collider attached with a rigid-body with a `Static` status.
* **`BodyStatus::Kinematic`:** Indicates the rigid-body velocity must not be altered by the physics engine. The user is free to set any velocity and the rigid-body position will be integrated at each update accordingly. This is typically used for **platforms** as shown is that [demo](/demo_body_status3/).
* **`BodyStatus::Disabled`:** Indicates the rigid-body should be completely ignored by the physics engine. This will effectively remove all contacts this rigid-body is involved with, and disable (but not remove) all joint constraints.

To change the status of a rigid-body, you need to retrieve a mutable reference to it and then call the `.set_status(...)` method. For example:

```rust
let mut rb = world.rigid_body_mut(handle).expect("Rigid body not found.");
rb.set_status(BodyStatus::Kinematic);
// Sets the velocity of the kinematic rigid-body.
rb.set_velocity(Velocity::linear(0.0, 0.0, 1.0));
```

Note that those statuses can also be applied to other types of bodies, including [multibodies](/joint_constraints_and_multibodies/#multibodies).


## Colliders
## Gravity and external forces
## Running the simulation