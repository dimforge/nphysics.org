# Rigid-body simulation with contacts
The real-time simulation of rigid-bodies subjected to forces and contacts is the main feature of a physics engine for
video-games or animation. Rigid-bodies are typically used to simulate the dynamics of non-deformable solids as well as
well as to integrate the trajectory of solids which velocities are controlled by the user (e.g. moving platforms). On
the other hand, rigid-bodies are not enough to simulate, e.g., cars, ragdolls, or robotic systems, as those use-cases
require adding restrictions on the relative motion between their parts with joints. Joints are the topic of
[the next chapter](joint_constraints_and_multibodies.md).

In this chapter, we first show how to initialize a [physics world](#initializing-the-physics-world) which will construct
all that is to be physically simulated and will drive the physics simulation. 

Then we introduce the [colliders](#colliders) which are geometric shapes responsible for generating contacts or simulating
sensors. Those colliders may be attached to [rigid-bodies](#rigid-bodies) which are responsible for the simulation of
the object trajectory under various forces including gravity. Colliders may also be attached to other kind of bodies
like multibodies and deformable bodies described in next chapters.

Finally, external forces can be applied to some bodies of the world by creating [force generators](#gravity-and-external-forces).

!!! Note "Terminology: **body** and **body part**"
    Throughout this guide, the term **body** is used to talk indifferently about rigid-bodies, multibodies and deformable
    bodies. The smallest constituent of a body is called a **body part**. Because a rigid-body is composed of a single
    indivisible piece, it is itself its only body part. A multibody however can be divided into several multibody links
    and each link is said to be a body part.


## Initializing the physics world
The [`World`](/rustdoc/nphysics3d/world/struct.World.html) structure is at the core of **nphysics**. It will contain all
the objects to be handled by the physics engine and will perform all the collision detection, dynamics integrations, and
event generation. It is easily created with:
 
```rust
let mut world = World::new();
```
 
This constructor will initialize the physics engine with all its default parameters, including:

* A gravity set to zero. This can be changed using, e.g., `world.set_gravity(Vector3::y() * -9.81)`.
* The initialization a [ColliderWorld](/rustdoc/nphysics3d/world/struct.ColliderWorld.html) which is a wrapper for the
  `CollisionWorld` from the **ncollide2d** or **ncollide3d** [crate](https://ncollide.org) to handle collision detection.
* The use of an impulse-based constraint solver to handle contacts. By default a Sinorini-Coulomb contact model with
polyhedral friction cones is used. Other contact models can be chosen afterward. Refer to the dedicated [section](contact_models.md).

Once the physics world is created, it will be empty. The next steps are then to add colliders that may optionally be attached
body parts.

## Colliders
Colliders represent the geometric shapes that generate contacts and contact events when they touch. A collider is built
and added to the world by a `ColliderDesc` structure following the builder pattern:


<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#collider_desc_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#collider_desc_3D">3D example</a></li>
</ul>

<div class="tab-content" markdown="1">
  <div id="collider_desc_2D" class="tab-pane in active">
```rust
use na::{Vector2, Isometry2};
use ncollide2d::shape::{ShapeHandle, Ball};
use ncollide2d::world::CollisionGroups;
use nphysics2d::object::ColliderDesc;
use nphysics2d::material::{MaterialHandle, BasicMaterial};

let shape = ShapeHandle::new(Ball::new(1.5));
ColliderDesc::new(shape)
    // The collider translation wrt. the body part it is attached to.
    // Default: zero vector.
    .translation(Vector2::y() * 5.0)
    // The collider rotation wrt. the body part it is attached to.
    // Default: no rotation.
    .rotation(5.0)
    // The collider position wrt. the body part it is attached to.
    // Will override `.translation(...)` and `.rotation(...)`.
    // Default: the identity isometry.
    .position(Isometry2::new(Vector2::new(1.0, 2.0), PI))
    // The collider density. If non-zero the collider's mass and angular inertia will be added
    // to the inertial properties of the body part it is attached to.
    // Default: 0.0
    .density(1.3)
    // The material of this collider.
    // Default: BasicMaterial::default()
    // with restitution: 0.0, friction: 0.5, combine mode: Average.
    .material(MaterialHandle::new(BasicMaterial::new(0.3, 0.8)))
    // The name of this collider.
    // Default: ""
    .name("My collider".to_owned())
    // The solid margin surrounding this collider. Should always be non-zero.
    // Default: 0.01
    .margin(0.02)
    // The collision group this collider is part of and interacts with.
    // Default: part of all groups and interacts with all groups.
    .collision_groups(CollisionGroups::new()
                        .with_membership(&[1, 6])
                        .with_whitelist(&[1, 3, 5]))
    // The distance tolerance for predictive contacts generation.
    // Default: 0.002
    .linear_prediction(0.01)
    // The angular tolerance for predictive contacts generation.
    // Default: PI / 180.0 * 5.0
    .angular_prediction(0.1)
    // Whether this collider is a sensor, i.e., generate only proximity events.
    // Default: false
    .sensor(true)
    // All done, actually build the collider into the `world`.
    .build(&mut world);
```
  </div>
  <div id="collider_desc_3D" class="tab-pane">
```rust
use na::{Vector3, Isometry3};
use ncollide3d::shape::{ShapeHandle, Ball};
use ncollide3d::world::CollisionGroups;
use nphysics3d::object::ColliderDesc;
use nphysics3d::material::{MaterialHandle, BasicMaterial};

let shape = ShapeHandle::new(Ball::new(1.5));
ColliderDesc::new(shape)
    // The collider translation wrt. the body part it is attached to.
    // Default: zero vector.
    .translation(Vector3::y() * 5.0)
    // The collider rotation wrt. the body part it is attached to.
    // Default: no rotation.
    .rotation(Vector3::y() * 5.0)
    // The collider position wrt. the body part it is attached to.
    // Will override `.translation(...)` and `.rotation(...)`.
    // Default: the identity isometry.
    .position(Isometry3::new(Vector3::new(1.0, 2.0, 3.0), Vector3::y() * PI))
    // The collider density. If non-zero the collider's mass and angular inertia will be added
    // to the inertial properties of the body part it is attached to.
    // Default: 0.0
    .density(1.3)
    // The material of this collider.
    // Default: BasicMaterial::default()
    // with restitution: 0.0, friction: 0.5, combine modes: Average.
    .material(MaterialHandle::new(BasicMaterial::new(0.3, 0.8)))
    // The name of this collider.
    // Default: ""
    .name("My collider".to_owned())
    // The solid margin surrounding this collider. Should always be non-zero.
    // Default: 0.01
    .margin(0.02)
    // The collision group this collider is part of and interacts with.
    // Default: part of all groups and interacts with all groups.
    .collision_groups(CollisionGroups::new()
                        .with_membership(&[1, 6])
                        .with_whitelist(&[1, 3, 5]))
    // The distance tolerance for predictive contacts generation.
    // Default: 0.002
    .linear_prediction(0.01)
    // The angular tolerance for predictive contacts generation.
    // Default: PI / 180.0 * 5.0
    .angular_prediction(0.1)
    // Whether this collider is a sensor, i.e., generate only proximity events.
    // Default: false
    .sensor(true)
    // All done, actually build the collider into the `world`.
    .build(&mut world);
```
  </div>
</div>

A `ColliderDesc` must be initialized with a collision shape (a `Ball` in this example). Possible shapes are
[planes](https://ncollide.org/geometric_representations/#plane), [balls](https://ncollide.org/geometric_representations/#ball),
[cuboids](https://ncollide.org/geometric_representations/#cuboid), [capsule](https://ncollide.org/geometric_representations/#capsule),
[heightfield](https://ncollide.org/geometric_representations/#cuboid), [convex polyhedra](https://ncollide.org/geometric_representations/#convex-hull),
[polylines](https://ncollide.org/geometric_representations/#polyline), [triangle meshes](https://ncollide.org/geometric_representations/#trimesh),
and [compound shapes](https://ncollide.org/geometric_representations/#compound).
All those structures are defined by the **ncollide2d** or **ncollide3d** crate.

!!! Note
    All the colliders are stored into the `ColliderWorld` itself part of the physics world. It is possible to
    interact with the collider world by retrieving a reference to it with `world.collider_world()`.

Here are details about some of the configurable collider properties:

* `density`: the density of the collider. If the collider is attached to a body part as described in the
[next section](#collider-with-parent) then this density is used to compute the collider shape's mass, center of mass,
and angular inertia which are added to the attached body part's inertia properties.
* `name`: the name of the collider. This is purely descriptive and can be used as an inefficient way to retrieve a
reference to the collider once it's created in the physics world. This is provided for convenience only and it is
recommended to refer to a collider by its handle instead of its name.

```rust
for collider in world.collider_world().colliders_with_name("My collider") {
    // Yields all the colliders with the name "My collider"
}
```

* `margin`: the thickness that surrounds the collider's shape. This thickness is automatically added by **nphysics** for
performance and numerical accuracy reasons. Because this margin implicitly enlarges the shapes of colliders added to the
world, it can be useful to compensate it by adding a slightly smaller shape. For example, assume we want to add a cube
with half-extents all equal to 1.0. With a margin set to, e.g., 0.01, we could first create a slightly smaller cube:
`Cuboid::new(Vector3::repeat(1.0 - 0.01))` so that when the margin is added by **nphysics**, the half-extents will be
exactly 1.0 again:

<center>
![Collider margin](img/collider_margin.svg)
</center>

* `position`: the position of the collider relative to its parent body part. At each timestep, the actual world-space
position of the collider is updated if its parent moved. If the parent of this collider is `BodyHandle::ground()`, the
collider has a constant world-space position equal to `position`.
* `material`: the [material](/rustdoc/nphysics3d/object/struct.Material.html) of the collider
describing the friction and restitution coefficient to be applied during contact resolution.

After adding a collider to the world, a mutable reference to this collider is returned by the `.build` method. A handle
to the collider can be retrieved by calling `collider.handle()`. This handle is to be used to later
retrieve a reference to the corresponding `Collider` [structure](/rustdoc/nphysics3d/object/type.Collider.html) or to
remove it from the world using `world.remove_colliders(...)`. A reference to a collider provides access to its properties:

| Method                | Description                                                  |
|--                     | --                                                           |
| `.handle()`           | The handle of this collider on the world.                    |
| `.position()`         | The world-space position of this collider                    |
| `.shape()`            | The geometrical shape of this collider.                      |
| `.collision_groups()` | Groups used to prevent interactions with some other objects. |
| `.body()`              | The body this collider is attached to.                                |
| `.margin()`            | The margin surrounding this collider.                                 |
| `.position_wrt_body()` | The position of this collider relative to the body it is attached to. |
| `.material()`          | The material given to this collider.                                  |
| `.material_mut()`      | Mutable reference to the material given to this collider.             |

### Collider with parent
It is possible to attach a collider to a body part by specifying the body part handle when building the collider:

```rust
let parent_handle = RigidBodyDesc::new()
    .build(&mut world)
    .part_handle();

let shape = ShapeHandle::new(Ball::new(1.5));
ColliderDesc::new(shape)
    .density(1.0)
    .translation(Vector3::y() * 5.0)
    .build_with_parent(parent_handle, &mut world);
```

Another, more convenient, alternative is to attached the collider directly to the body constructor. The following
example will automatically create a collider whenever a rigid body is created with the initialized `RigidBodyDesc`:

```rust
let shape = ShapeHandle::new(Ball::new(1.5));
let collider_desc = ColliderDesc::new(shape)
    .density(1.0)
    .translation(Vector3::y() * 5.0);
    
RigidBodyDesc::new()
    .collider(&collider_desc)
    .build(&mut world);
```

With this second approach the collider will be created at the same time as the rigid body. Re-using the same `RigidBodyDesc`
to build other rigid bodies will also create new colliders for each call to `.build`.

!!! Note
    If a collider is given a density, it will contribute to the mass and angular inertia of the body part it is
    attached to.

The following shows two ways to retrieve the handle of the body attached to a collider:

```rust
// Assuming `collider_handle` is a valid handle of a collider previously added to the world.
let collider = world.collider(collider_handle).expect("Collider not found.");
let body_handle = collider.body();

// One-liner:
let body_handle = world.collider_body_handle(collider_handle).expect("Collider not found.");
```

### Collision groups
It is possible to configure a subset of colliders to collide only with another subset of colliders. This is the goal of
**collision groups**. Collision groups are demonstrated on the [Collision groups](demo_collision_groups3) example where
only colliders with the same colors can collide with one another. The source-code of the demo is available on
[github](https://github.com/rustsim/nphysics/blob/master/examples3d/collision_groups3.rs). A collider's collision group
can be configured in a `ColliderDesc` before it is built into the physics world.

Because this is a feature from the **ncollide2d** and **ncollide3d** crates, an in-depth description of collision
groups is given in [that section](https://ncollide.org/collision_detection_pipeline/#collision-groups) of their
dedicated user guide.


## Rigid-bodies
A rigid-body is the simplest type of body supported by **nphysics**. It can be seen as the aggregation of a position,
orientation, and mass properties (rotational inertia tensor, mass, and center of mass). It does not hold any information
regarding its shape which can optionally be specified by attaching one or multiple [colliders](/rigid_body_simulations_with_contacts/#colliders)
to it. A rigid-body with no collider will be affected by all the forces the world is aware of, but not by contacts
(because it does not have any shape that can be collided to).

### Adding a rigid-body to the world
A rigid-body can only created by a `RigidBodyDesc` structure that is based on the builder pattern:


<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#rigid_body_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#rigid_body_desc_3D">3D example</a></li>
</ul>

<div class="tab-content" markdown="1">
  <div id="rigid_body_desc_2D" class="tab-pane in active">
```rust
use na::{Vector2, Point2, Isometry2};
use nphysics2d::object::{BodyStatus, RigidBodyDesc};
use nphysics2d::math::{Velocity, Inertia};

RigidBodyDesc::new()
    // The rigid body translation.
    // Default: zero vector.
    .translation(Vector2::y() * 5.0)
    // The rigid body rotation.
    // Default: no rotation.
    .rotation(5.0)
    // The rigid body position. Will override `.translation(...)` and `.rotation(...)`.
    // Default: the identity isometry.
    .position(Isometry2::new(Vector2::new(1.0, 2.0), PI))
    // Whether or not this rigid body is affected by gravity.
    // Default: true
    .gravity_enabled(false)
    // The status of this rigid body.
    // Default: BodyStatus::Dynamic
    .status(BodyStatus::Kinematic)
    // The name of this rigid body.
    // Default: ""
    .name("my rigid body".to_owned())
    // The velocity of this body.
    // Default: zero velocity.
    .velocity(Velocity::linear(1.0, 2.0))
    // The angular inertia tensor of this rigid body, expressed on its local-space.
    // Default: the zero matrix.
    .angular_inertia(3.0)
    // The rigid body mass.
    // Default: 0.0
    .mass(1.2)
    // The mass and angular inertia of this rigid body expressed in
    // its local-space. Default: zero.
    // Will override `.mass(...)` and `.angular_inertia(...)`.
    .local_inertia(Inertia::new(1.0, 3.0))
    // The center of mass of this rigid body expressed in its local-space.
    // Default: the origin.
    .local_center_of_mass(Point2::new(1.0, 2.))
    // The threshold for putting this rigid body to sleep.
    // Default: Some(ActivationStatus::default_threshold())
    .sleep_threshold(None)
    // The translations that will be locked for this rigid body.
    // Default: nothing is locked (false everywhere).
    .kinematic_translations(Vector2::new(true, false))
    // The translations that will be locked for this rigid body.
    // Default: nothing is locked (false everywhere).
    .kinematic_rotation(true)
    // Add a collider that will be attached to this rigid body.
    // If the collider has a non-zero density, its mass and angular
    // inertia will be added to this rigid body.
    // Default: no collider.
    .collider(&collider_desc)
    // All done, actually build the rigid-body into the `world`.
    .build(&mut world);
```
  </div>
  <div id="rigid_body_desc_3D" class="tab-pane">
```rust
use na::{Vector3, Point3, Isometry3, Matrix3};
use nphysics3d::object::{BodyStatus, RigidBodyDesc};
use nphysics3d::math::{Velocity, Inertia};

RigidBodyDesc::new()
    // The rigid body translation.
    // Default: zero vector.
    .translation(Vector3::y() * 5.0)
    // The rigid body rotation.
    // Default: no rotation.
    .rotation(Vector3::y() * 5.0)
    // The rigid body position. Will override `.translation(...)` and `.rotation(...)`.
    // Default: the identity isometry.
    .position(Isometry3::new(Vector3::new(1.0, 2.0, 3.0), Vector3::y() * PI))
    // Whether or not this rigid body is affected by gravity.
    // Default: true
    .gravity_enabled(false)
    // The status of this rigid body.
    // Default: BodyStatus::Dynamic
    .status(BodyStatus::Kinematic)
    // The name of this rigid body.
    // Default: ""
    .name("my rigid body".to_owned())
    // The velocity of this body.
    // Default: zero velocity.
    .velocity(Velocity::linear(1.0, 2.0, 3.0))
    // The angular inertia tensor of this rigid body, expressed on its local-space.
    // Default: the zero matrix.
    .angular_inertia(Matrix3::from_diagonal_element(3.0))
    // The rigid body mass.
    // Default: 0.0
    .mass(1.2)
    // The mass and angular inertia of this rigid body expressed in
    // its local-space. Default: zero.
    // Will override `.mass(...)` and `.angular_inertia(...)`.
    .local_inertia(Inertia::new(1.0, Matrix3::from_diagonal_element(3.0)))
    // The center of mass of this rigid body expressed in its local-space.
    // Default: the origin.
    .local_center_of_mass(Point3::new(1.0, 2.0, 3.0))
    // The threshold for putting this rigid body to sleep.
    // Default: Some(ActivationStatus::default_threshold())
    .sleep_threshold(None)
    // The translations that will be locked for this rigid body.
    // Default: nothing is locked (false everywhere).
    .kinematic_translations(Vector3::new(true, false, true))
    // The translations that will be locked for this rigid body.
    // Default: nothing is locked (false everywhere).
    .kinematic_rotations(Vector3::new(true, false, true))
    // Add a collider that will be attached to this rigid body.
    // If the collider has a non-zero density, its mass and angular
    // inertia will be added to this rigid body.
    // Default: no collider.
    .collider(&collider_desc)
    // All done, actually build the rigid-body into the `world`.
    .build(&mut world);
```
  </div>
</div>

All the properties are optional. The only calls that are required are `RigidBodyDesc::new()` to initialize the builder,
 and `.build(&mut world)` to actually build the rigid body. Note that a single builder can be re-used to build several
 rigid bodies. It is possible to change some of the builder properties with methods prefixed by `set_` before building
 another rigid body:
 
```rust
let mut rb_desc = RigidBodyDesc::new()
   .rotation(Vector3::new(1.0, 2.0, 3.0))
   .mass(1.2);

// Build a first rigid body.
rb_desc.build(&mut world);

// Change the rigid body translation and velocity before building
// another one. It will still have the same mass and rotation as
// initialized above.
rb_desc.set_translation(Vector3::new(10.0, 0.0, 2.0))
       .set_velocity(Velocity::linear(1.0, 3.0, 0.0))
       .build(&mut world);
```

Typically, the inertia and center of mass are set to the inertia and center of mass resulting from the shapes of the [colliders](#colliders) attached to the rigid-body, however this is not a requirement.

!!! Note
    The `.build(...)` method of the `RigidBodyDesc` returns a mutable reference to the newly created rigid-body. It is
    then possible to further modify the rigid body, or retrieve its handle:
    
```rust
let rigid_body = RigidBodyDesc::new()
                    .translation(Vector3::x() * 2.0)
                    .build(&mut world);
rigid_body.set_mass(10.0);
let handle = rigid_body.handle();
```
    
!!! Note
    No two bodies in the same physics world can share the same handle. This handle is what you should store for future
    addressing of this rigid body as it isrequired by various operations  including: attaching colliders or constraints
    to the rigid-body, retrieving a reference to the underlying `RigidBody` structure created by the `World` (as described
    in the next section), removing the rigid-body from the world, etc.

### Getting a `RigidBody` from the world
A reference to a rigid-body added to the world can be retrieved from its handle. Two methods are available:

* **`world.rigid_body(handle)`:** returns a reference to a [RigidBody](/rustdoc/nphysics3d/object/struct.RigidBody.html)
  structure if it has been found.
* **`world.body(handle)`:** returns a reference to a [Body](/rustdoc/nphysics3d/object/enum.Body.html) which itself is
  a trait implemented by all the bodies supported by **nphysics**.

The method to choose depends on the context. `.rigid_body(...)` should cover most scenarios. The other methods are
generally useful when writing code that operate independently from the actual variant of body or body part.

## Body statuses
Any body can have one of four different statuses identified by the [`object::BodyStatus`](/rustdoc/nphysics3d/object/enum.BodyStatus.html)
enumeration:

* **`BodyStatus::Dynamic`:** This is the default status when creating any body. It indicates the body is affected by
    external forces, inertial forces (gyroscopic, coriolis, etc.) and contacts.
* **`BodyStatus::Static`:** Indicates the body cannot move. It acts as if it has an infinite mass and will not be
    affected by any force. It will continue to collide with dynamic bodies but not with static nor with kinematic
    ones. This is typically used for temporarily freezing a body.

!!! Note
    It is possible to add colliders attached to a special body with handle `BodyHandle::ground()`. Therefore, objects
    that will never move should be simulated with colliders attached to the `BodyHandle::ground()` instead of with a
    collider attached with a rigid-body with a `Static` status.

* **`BodyStatus::Kinematic`:** Indicates the body velocity must not be altered by the physics engine. The user is
    free to set any velocity and the body position will be integrated at each update accordingly. This is typically
    used for **platforms** as shown in [that demo](/demo_body_status3/).
* **`BodyStatus::Disabled`:** Indicates the body should be completely ignored by the physics engine. In practice,
    this will remove all contacts this body is involved with and disable (but not remove) all joint constraints
    attached to it.

For example, to change the status of a rigid-body, you need to retrieve a mutable reference to it and then call the
`.set_status(...)` method:

```rust
let mut rb = world.rigid_body_mut(handle).expect("Rigid-body not found.");
rb.set_status(BodyStatus::Kinematic);
// Sets the velocity of the kinematic rigid-body.
rb.set_velocity(Velocity::linear(0.0, 0.0, 1.0));
```

Note that those statuses can also be applied to other types of bodies, including
[multibodies](/joint_constraints_and_multibodies/#multibodies) and [deformable bodies](/deformable_bodies/).

## Kinematic degrees of freedom
Some bodies allow to mark only some of its degrees of freedom as kinematic. For example:

* It is possible to mark as kinematic some translations or rotations of a rigid body. This is useful to prevent
  all translations and/or rotations wrt. some specific coordinate axises, or to control them at the velocity level.
  This is achieved by the `.kinematic_translations` and `kinematic_rotations` modifiers at construction-time, or by
  the `.set_translations_kinematic` and `.set_rotations_kinematic` methods after construction:
  
```rust
let mut world = World::new();
let handle = RigidBodyDesc::new()
    // Translations along the y and z axises will be locked and controllable at the
    // velocity level by the user.
    .kinematic_translations(Vector3::new(false, true, true));
    .velocity(Velocity::linear(1.0, 0.0, 3.0))
    .build(&mut world)
    .handle();
    
// NOTE: the velocity of the rigid body along the `y` and `z` axis will remain
// constant independently from any forces. Therefore this body will be fixed
// in translation wrt the `y` axis, and will translate with a velocity of `3.0`
// along the `z` axis.
```

This second example changes the kinematic degrees of freedom of a rigid body after its creation to the world, using its
handle. Now the translations along `x` as well as rotations wrt. the axis `x` and `z` will be locked and controllable
at the velocity level:

```rust
world.rigid_body_mut(handle)
     .expect("Rigid body not found")
     .set_translations_kinematic(Vector3::new(true, false, false))
     .set_rotations_kinematic(Vector3::new(true, false, true));
```

!!! Note "Rigid bodies that cannot rotate"
    It can be useful to prevent a rigid-body from rotating at all, e.g., for the player entity of a video game.
    This can be achieved by setting all its rotations as kinematic, or, equivalently, by calling
    `.disable_all_rotations()` on the rigid body. Note that `.disable_all_rotations()` will also set the rigid-body
    angular velocity to zero in addition to marking all its rotational degrees of freedom as kinematic.
  
* It is possible to mark some nodes of a deformable body as kinematic so that those nodes remain fixed in space or can
  be controlled by the user at the velocity level. See the [dofrmable bodies](/deformable_bodies/) section.
  
!!! Note ""Controllable at the velocity level""
    We used the term **controllable at the velocity level** multiple times here. This means it is still possible for the
    user to set a non-zero velocity to a kinematic body or a dynamic body with kinematic degrees of freedom. A kinematic
    body or the kinematic degrees of freedoms will keep this velocity constant no matter what. This is useful for platform
    that should have a specific trajectory without being disturbed by any force of the world. Of course, the user can
    still modify this trajectory by changing the velocity of those kinematic elements at any time.

## Gravity, forces, and impulses
There are three ways of applying forces to a body in **nphysics**. The most common force is the gravity and
is treated as a special case by **nphysics**. Other permanent external forces can be simulated using
[force generators](#permanent-force-generators). Finally, forces, impulses, and instantaneous velocity or acceleration
changes can be [applied](#one-time-force-application-and-impulses) for a single timestep to each body part individually.

### Gravity
Because the gravity is such a common force, it is a special case within **nphysics**.
By default, the physics world is initialized with a gravity set to zero. This can be changed by the world's
`.set_gravity(...)` method:

```rust
let mut world = World::new();
world.set_gravity(Vector3::y() * -9.81); // or Vector2 in 2D.
```

It is possible to cancel the effect of the gravity on a specific body:

* By disabling gravity for this body before it is constructed by callin the `.gravity_enabled(false)` setter. Example:

```rust
let mut rb_desc = RigidBodyDesc::new()
   // ... other initialization accessors.
   .gravity_enabled(false)
   .build(&mut world);
```
  
* Or by calling `body.enable_gravity(false)` on a body that has already been created into the physics world. This
    `.enable_gravity` method is part of the [`Body`](https://www.nphysics.org/rustdoc/nphysics3d/object/trait.Body.html)
    trait:
    
```rust
let mut rb = world.rigid_body_mut(rb_handle).expect("Rigid body not found.");
rb.enable_gravity(false);
```

### Permanent force generators
Other external forces can also be applied to the bodies on the physics world. This is what
[`ForceGenerators`](/rustdoc/nphysics3d/force_generator/trait.ForceGenerator.html) are for. A force generator as a
structure implementing the `ForceGenerator` trait which requires one method: `.apply(params, bodies)`. The `params` is
a reference to [integration parameters](/performance_tuning/#integration-parameters) allowing you to retrieve, e.g., the
timestep duration (in second) for the current update of the physical world with `params.dt`, or the total time `params.t`
elapsed in the physics world since its creation. The `bodies` parameter allows you to retrieve mutable references to the
rigid-bodies to be affected by the force generator. The following example shows the definition of a generator of a radial
force proportional to the position of a body wrt. a point:

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#force_generator_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#force_generator_3D">3D example</a></li>
</ul>

<div class="tab-content" markdown="1">
  <div id="force_generator_2D" class="tab-pane in active">
```rust
use na::Point2;
use nphysics2d::solver::IntegrationParameters;
use nphysics2d::force_generator::ForceGenerator;
use nphysics2d::object::{BodyPartHandle, BodySet};
use nphysics2d::math::{Force, ForceType};

pub struct RadialForce {
    parts: Vec<BodyPartHandle>, // Body parts affected by the force generator.
    center: Point2<f32>,
}

impl RadialForce {
    // Creates a new radial force generator.
    pub fn new(center: Point2<f32>, parts: Vec<BodyPartHandle>) -> Self {
        RadialForce {
            parts,
            center,
        }
    }

    /// Add a body part to be affected by this force generator.
    pub fn add_body_part(&mut self, body: BodyPartHandle) {
        self.parts.push(body)
    }
}

impl ForceGenerator<f32> for RadialForce {
    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        for handle in &self.parts {
            // Generate the force only if the body has not been removed from the world.
            if let Some(body) = bodies.body_mut(handle.0) {
                let part = body.part(handle.1).unwrap();

                // The `.as_ref()` retrieves a `BodyPart` from the `BodyPartMut`.
                let delta_pos = part.center_of_mass() - self.center;

                // We set the attraction force such that it is equal to ten times the distance
                // between the body part and self.center.
                let force = Force::linear(delta_pos * -10.0);

                // Apply the force.
                body.apply_force(handle.1, &force, ForceType::Force, false);
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
use na::Point3;
use nphysics3d::solver::IntegrationParameters;
use nphysics3d::force_generator::ForceGenerator;
use nphysics3d::object::{BodyPartHandle, BodySet};
use nphysics3d::math::{Force, ForceType};

pub struct RadialForce {
    parts: Vec<BodyPartHandle>, // Body parts affected by the force generator.
    center: Point3<f32>,
}

impl RadialForce {
    // Creates a new radial force generator.
    pub fn new(center: Point3<f32>, parts: Vec<BodyPartHandle>) -> Self {
        RadialForce {
            parts,
            center,
        }
    }

    /// Add a body part to be affected by this force generator.
    pub fn add_body_part(&mut self, body: BodyPartHandle) {
        self.parts.push(body)
    }
}

impl ForceGenerator<f32> for RadialForce {
    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        for handle in &self.parts {
            // Generate the force only if the body has not been removed from the world.
            if let Some(body) = bodies.body_mut(handle.0) {
                let part = body.part(handle.1).unwrap();

                // The `.as_ref()` retrieves a `BodyPart` from the `BodyPartMut`.
                let delta_pos = part.center_of_mass() - self.center;

                // We set the attraction force such that it is equal to ten times the distance
                // between the body part and self.center.
                let force = Force::linear(delta_pos * -10.0);

                // Apply the force.
                body.apply_force(handle.1, &force, ForceType::Force, false);
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

2. The [ConstantAcceleration](/rustdoc/nphysics3d/force_generator/struct.ConstantAcceleration.html) force generator 
applies a linear and angular force at the center of mass of some specified body parts. The force is such that the linear
and angular accelerations specified at the construction of this force generator are added to the affected bodies.
1. The [Spring](/rustdoc/nphysics3d/force_generator/struct.Spring.html) applies opposite forces to two bodies. The forces
magnitudes are proportional to the distance between the bodies.

### One-time force application and impulses
It is possible to apply a force to any body part, at any of its points. In the following, let us call $~f$ a force we
apply at the center of mass of a body part. We distinguish four kinds of forces listed in the `ForceType` enum:

* `ForceType::Force`: a force as in the formula $~f = mass × acceleration$. Applying this force to a body will
   add a velocity of $~\frac{f}{mass} × dt$ (where $~dt$ is the time step length) to the body part's velocity at the next 
   timestep.
* `ForceType::AccelerationChange`: a direct acceleration change that will add $~f × dt$ to the body part's velocity at the next timestep.
* `ForceType::Impulse`: an impulsive force that immediately adds $~\frac{f}{mass}$ to the body part's velocity.
* `ForceType::VelocityChange`: a direct velocity change that immediately adds $~f$ to the body part's velocity.

!!! Note
    All forces applied to a body part are cleared during the next timestep. Therefore, a persistent force
    should be re-applied at each frame.
    
Several methods part of the `Body` trait can be called in order to apply a force to a body part:
    
```rust
use nphysics2d::math::{Force, ForceType}; // For 2D
use nphysics3d::math::{Force, ForceType}; // For 3D

// Force application at the body part's center of mass.
fn apply_force(&mut self, part_id: usize, f: &Force<N>, force_type: ForceType, auto_wake_up: bool);
fn apply_local_force(&mut self, part_id: usize, f: &Force<N>, force_type: ForceType, auto_wake_up: bool);

// Force application at a custom point on the body part.
fn apply_force_at_point(&mut self, part_id: usize, f: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool);
fn apply_local_force_at_point(&mut self, part_id: usize, f: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool);
fn apply_force_at_local_point(&mut self, part_id: usize, f: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool);
fn apply_local_force_at_local_point(&mut self, part_id: usize, f: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool);
```

Every method take very similar arguments among which:

* **part_id** is the index of the body's part you want to apply the force to. For rigid bodies, this argument is ignored
  and can take any value.
* **auto_wake_up** controls whether the body affected by the force should be waken-up automatically because of this force
  application. This should typically be set to `true` whenever you are applying a one-time force manually. This should
  likely be set to `false` if you are applying a continuous force from a force generator (so that bodies reaching a dynamic
  equilibrium can be put to sleep again).



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

Physics events are detailed on a dedicated [page](interaction_handling_and_sensors.md) of this guide. One physics step
will detect contacts between the colliders, generate contact points, compute forces accordingly, and update the positions
of all bodies. Forces taken into account by **nphysics** are:

* Contact forces due to impacts between colliders.
* Gravity and force generators presented in the previous [section](/rigid_body_simulations_with_contacts/#gravity-and-external-forces).
* Gyroscopic and coriolis forces due to rotations and the non-linear nature of multibody joints parametrization.

Each call to `world.step()` will advance the simulation by a time equal to $1/60$ seconds, which is adapted if your
application has a refresh rate of 60Hz. The length of this timestep can be retrieved by `world.timestep()` and modified
using `world.set_timestep(...)`. Note that if you use SI units, i.e., the
[International System of Units](https://en.wikipedia.org/wiki/International_System_of_Units), this timestep is to be
given in seconds. Keep in mind that the timestep length strongly affects the accuracy of the simulation: the smaller the
timestep, the more accurate the simulation will be.
