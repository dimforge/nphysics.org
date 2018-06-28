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
Colliders represent the geometric shapes that generate contacts and contact events when they touch. A collider is added to the world using `.add_collider(...)` with the following arguments:

* `margin`: the thickness that surrounds the collider's shape.
<center>
![FIXME MARGIN ILLUSTRATION](fixme)
</center>
* `shape`: the collider's shape. Possible shapes are [planes](http://ncollide.org/geometric_representations/#plane), [balls](http://ncollide.org/geometric_representations/#ball), [cuboids](http://ncollide.org/geometric_representations/#cuboid), [convex polyhedra](http://ncollide.org/geometric_representations/#convex-hull), [polylines](http://ncollide.org/geometric_representations/#polyline), [triangle meshes](http://ncollide.org/geometric_representations/#trimesh), and [compound shapes](http://ncollide.org/geometric_representations/#composite-shapes). All those structures are defined by the **ncollide** crate.
* `parent`: the body part the collider is attached to. This can be set to `BodyHandle::ground()` if the collider is to be attached to the ground (i.e. it will be static).
* `to_parent`: the position of the collider relative to its parent body part. At each timestep, the actual world-space position of the collider is updated if its parent moved. If the parent is `BodyHandle::ground()`, the collider has a constant world-space position equal to `to_parent`.
* `material`: the [material](http://localhost:8000/rustdoc/nphysics3d/object/struct.Material.html) of the collider describing the friction and restitution coefficient to be applied during contact resolution.

After adding a collider to the world, a collider handle is returned by the physics world. This handle is to be used to retrieve a reference to the corresponding `Collider` [structure](/rustdoc/nphysics3d/object/type.Collider.html) or to remove it from the world using `world.remove_colliders(...)`. Note that the `Collider`structure is simply a [CollisionObject](http://ncollide.org/collision_detection_pipeline/#collision-objects) from the **ncollide** crate under the hood. Therefore, all the [methods](http://ncollide.org/rustdoc/ncollide3d/world/struct.CollisionObject.html) of `CollisionObject` are available, including:

| Method                | Description                                                  |
|--                     | --                                                           |
| `.handle()`           | The handle of this collider on the world.                    |
| `.position()`         | The world-space position of this collider                    |
| `.shape()`            | The geometrical shape of this collider.                      |
| `.collision_groups()` | Groups used to prevent interactions with some other objects. |
| `.data()`             | Data associated to the collider by **nphysics**.             |

Additionally, the `.data()` method provide access to a `ColliderData` [structure](/rustdoc/nphysics3d/object/struct.ColliderData.html) managed by **nphysics** and with the following methods:

| Method                 | Description                                                           |
|--                      | --                                                                    |
| `.body()`              | The body this collider is attached to.                                |
| `.margin()`            | The margin surrounding this collider.                                 |
| `.position_wrt_body()` | The position of this collider relative to the body it is attached to. |
| `.material()`          | The material given to this collider.                                  |

The following shows an example of addition of a collider to the world:
```rust
// Create a collider attached to the ground and with the default material.
let collider1 = world.add_collider(
    0.04,
    cuboid,
    BodyHandle::ground(),
    Isometry3::identity(),
    Material::default(),
);


// Create a collider attached to a previously-added rigid body with handle `body_handle`.
let material = Material::new(0.5, 0.3); // Custom material.
let collider2 = world.add_collider(
    0.04,
    cuboid,
    body_handle,
    Isometry3::identity(),
    material
);
```

The following shows how to retrieve the handle of the body attached to a collider:
```rust
// Assuming `collider_handle` is a valid handle of a collider previously added to the world.
let collider = world.collider(collider_handle).expect("Collider not found.");
let body_handle = collider.data().body();
```

## Gravity and external forces
By default, the physics world is initialized with a gravity set to zero. This can be changed by the world's `.set_gravity(...)` method:

```rust
let mut world = World::new();
world.set_gravity(Vector3::y() * -9.81); // or Vector2 in 2D.
```

Other external forces can also be applied to the bodies on the physics world. This is what [`ForceGenerators`](/rustdoc/nphysics3d/force_generator/trait.ForceGenerator.html) are for. A force generator as a structure implementing the `ForceGenerator` trait which requires one method: `.apply(params, bodies)`. The `params` is a reference to [integration parameters](/performance_tuning/#integration-parameters) allowing you to retrieve, e.g., the timestep duration (in second) for the current update of the physical world with `params.dt`. The `bodies` parameter allows you to retrieve mutable references to the rigid bodies to be affected by the force generator. The following example shows the definition of a generator of a radial force proportional to the position of a body wrt. a point:

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#force_generator_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#force_generator_3D">3D example</a></li>
</ul>

<div class="tab-content" markdown="1">
  <div id="force_generator_2D" class="tab-pane in active">
```rust
use nphysics2d::solver::IntegrationParameters;
use nphysics2d::force_generator::ForceGenerator;
use nphysics2d::object::{BodyHandle, BodySet};
use nphysics2d::math::Velocity;

pub struct RadialForce {
    parts: Vec<BodyHandle>, // Body parts affected by the force generator.
    center: Point2<f32>,
}

impl RadialForce {
    // Creates a new radial force generator.
    pub fn new(center: Point2<N>, parts: Vec<BodyHandle>) -> Self {
        RadialForce {
            parts,
            center,
        }
    }

    /// Add a body part to be affected by this force generator.
    pub fn add_body_part(&mut self, body: BodyHandle) {
        self.parts.push(body)
    }
}

impl ForceGenerator<f32> for RadialForce {
    fn apply(&mut self, _: &IntegrationParameters<N>, bodies: &mut BodySet<N>) -> bool {
        for handle in &self.parts {
            // Generate the force only if the body has not been removed from the world.
            if bodies.contains(handle) {
                let mut part = bodies.body_part_mut(handle);

                // The `.as_ref()` retrieves a `BodyPart` from the `BodyPartMut`.
                let delta_pos = part.as_ref().center_of_mass() - self.center;

                // We set the force such that it is equal to ten times the distance
                // between the body part and self.center.
                let force = Force::linear(delta_pos * 10.0);

                // Apply the force.
                part.apply_force(&force);
            }
        }

        // If `false` is returned, the physis world will remove
        // this force generator after this call.
        true
    }
}
```
  </div>
  <div id="force_generator_3D" class="tab-pane">
```rust
use nphysics3d::solver::IntegrationParameters;
use nphysics3d::force_generator::ForceGenerator;
use nphysics3d::object::{BodyHandle, BodySet};
use nphysics3d::math::Velocity;

pub struct RadialForce {
    parts: Vec<BodyHandle>, // Body parts affected by the force generator.
    center: Point3<f32>,
}

impl RadialForce {
    // Creates a new radial force generator.
    pub fn new(center: Point3<N>, parts: Vec<BodyHandle>) -> Self {
        RadialForce {
            parts,
            center,
        }
    }

    /// Add a body part to be affected by this force generator.
    pub fn add_body_part(&mut self, body: BodyHandle) {
        self.parts.push(body)
    }
}

impl ForceGenerator<f32> for RadialForce {
    fn apply(&mut self, _: &IntegrationParameters<N>, bodies: &mut BodySet<N>) -> bool {
        for handle in &self.parts {
            // Generate the force only if the body has not been removed from the world.
            if bodies.contains(handle) {
                let mut part = bodies.body_part_mut(handle);

                // The `.as_ref()` retrieves a `BodyPart` from the `BodyPartMut`.
                let delta_pos = part.as_ref().center_of_mass() - self.center;

                // We set the force such that it is equal to ten times the distance
                // between the body part and self.center.
                let force = Force::linear(delta_pos * 10.0);

                // Apply the force.
                part.apply_force(&force);
            }
        }

        // If `false` is returned, the physis world will remove
        // this force generator after this call.
        true
    }
}
```
</div>
</div>

Two forces generators are currently implemented in **nphysics**:

1. The [Spring](/rustdoc/nphysics3d/force_generator/struct.Spring.html) applies opposite forces to two bodies. The forces magnitudes are proportional to the distance between the bodies.
2. The [ConstantAcceleration](/rustdoc/nphysics3d/force_generator/struct.ConstantAcceleration.html) force generator applies a linear and angular force at the center of mass of some specified bodies. The force is such that the linear and angular accelerations specified at the construction of this force generator are added to the affected bodies.

## Running the simulation
To compute new positions and velocities of the various bodies on the physical world, you have to call the `world.step()` method. Typically, a game loop will look like this:

```rust
loop {
    handle_input_events();
    world.step();
    handle_physics_events();
    render_scene();
}
```

Physics events are detailed on a dedicated [page](event_handling_and_sensors.md) of this guide. One physics step will detect contacts between the colliders, generate contact points, compute forces accordingly, and update the position of all bodies. Forces taken into account by **nphysics** are:

* Contact forces due to impacts between colliders.
* Gravity and force generators presented in the previous [section](/rigid_body_simulations_with_contacts/#gravity-and-external-forces).
* Gyroscopic and coriolis forces due to rotations and the non-linear nature of multibody joints parametrization.

Each call to `world.step()` will advance the simulation by a time equal to $1/60$ seconds, which is typical if your application has a refresh rate of 60Hz. The length of this timestep can be retrieved by `world.timestep()` and modified using `world.set_timestep(...)`. Note that if you use SI units, i.e., the [International System of Units](https://en.wikipedia.org/wiki/International_System_of_Units), this timestep is to be given in seconds. Keep in mind that the timestep length strongly affects the accuracy of the simulation: the smaller the timestep, the more accurate the simulation will be.