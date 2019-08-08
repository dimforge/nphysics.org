# The nphysics testbed
The **nphysics_testbed2d** and **nphysic_testbed3d** crates provide pure-Rust, WASM-compatible, tools for displaying and
interacting easily with a physical scene. They are based on the [kiss3d](https://crates.io/crates/kiss3d) graphics engine.
Basically, all you have to do is setup your physics `World`, and then call `Testbed::new(world).run()` to obtain a
fully-functional windowed application with 2D or 3D display of every colliders on your `World`. This application
allows some controls like starting/stopping/pausing the simulation, and grabbing an object with the mouse.

!!! Note
    All the interactive demos from this website have been built using those testbeds. The source code of those demos
    can be found on the [examples2d](https://github.com/rustsim/nphysics/tree/master/examples2d) and
    [examples3d](https://github.com/rustsim/nphysics/tree/master/examples3d) of the **nphysics** github repository.

In this chapter we will describe an example of the setup of a 3D physical scene with a pile of boxes. This scene will
then be displayed and simulated using the **nphysics_testbed3d** crate. The full code can be found
[on github](https://github.com/rustsim/nphysics/blob/master/examples3d/boxes3.rs). The code for 2D analogous of this demo can be found
[on github](https://github.com/rustsim/nphysics/blob/master/examples2d/boxes2.rs).

## Initializing the project
First, initialize a new binary project, e.g., using cargo:

```sh
cargo new --bin example_nphysics_testbed_3d
cd example_nphysics_testbed_3d
```

Then, edit your `Cargo.toml` file to add **nphysics-testbed3d**, **nphysics3d**, **ncollide3d** (for collider shapes)
and **nalgebra** (for vectors and matrices) as dependencies:

```toml
[package]
name    = "examples_nphysics_testbed_3d"
version = "0.1.0"
authors = [ "you" ]

[dependencies]
nalgebra   = "0.18"
ncollide3d = "0.20"
nphysics3d = "0.12"
nphysics_testbed3d = "0.6"
```

Now modify the `src/main.rs` to use `na` as an alias for `nalgebra`:

```rust
extern crate nalgebra as na;
```

## Setting-up the physics world
The next step is to setup the physics world. First we will `use` a few elements to get the tools we need to initialize our physics worlds:

```rust
use na::{Point3, Vector3};                                            // For configuring and positioning bodies.
use ncollide3d::shape::{Cuboid, ShapeHandle};                         // Shapes for colliders.
use nphysics3d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle}; // For building bodies and colliders.
use nphysics3d::force_generator::DefaultForceGeneratorSet;            // The set of force generators (which we will leave empty).
use nphysics3d::joint::DefaultJointConstraintSet;                     // The set of joints (which we will leave empty).
use nphysics3d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld};  // The physics worlds to be initialized.
use nphysics_testbed3d::Testbed;                                      // The testbed to display/run the simulation.
```

The first thing we want to do is create a geometrical world as well as a mechanical world world with its default parameters
and a gravity that points toward the negative $y$-axis:

```rust
let mut geometrical_world = DefaultGeometricalWorld::new();
let mut mechanical_world = DefaultMechanicalWorld::new(Vector3::y() * -9.81);
```

Then we create all the sets needed for stepping te simulation. We will not actually add joints nor force generators in
this example, so their sets don't have to be mutable.

```rust
let mut bodies = DefaultBodySet::new();
let mut colliders = DefaultColliderSet::new();
let joint_constraints = DefaultJointConstraintSet::new();
let force_generators = DefaultForceGeneratorSet::new();
```

Then, we initialize the ground represented as a large box.

```rust
let ground_thickness = 0.2;
let ground_shape =
    ShapeHandle::new(Cuboid::new(Vector3::new(3.0, ground_thickness, 3.0)));

// Build a static ground body and add it to the body set.
let ground_handle = bodies.insert(Ground::new());

// Build the collider.
let co = ColliderDesc::new(ground_shape)
    .translation(Vector3::y() * -ground_thickness)
    .build(BodyPartHandle(ground_handle, 0));
// Add the collider to the collider set.
colliders.insert(co);
```

Note that this collider will never move so we can create a body of type `Ground` and attach our collider to it. In a
simulation where the ground is composed of multiple colliders, we would also attach those colliders to the single
 `ground_handle` created here.

!!! Note
    Instead of a body of type `Ground`, we could have used a `RigidBody` with a status set to `BodyStatus::Static` instead.
    That would have worked equally well. The only difference is that `Ground` occupies much less memory and is slightly
    more efficient since it is guaranteed to never move, and alway have a position equal to the origin.

Finally, we can setup our pile of boxes. This time, because our boxes are dynamic, we will have to create one rigid-body
for each box, and attach one collider to each rigid-body.

```rust
let num = 6;
let rad = 0.1;

let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

let shift = (rad + ColliderDesc::<f32>::default_margin()) * 2.0;
let centerx = shift * (num as f32) / 2.0;
let centery = shift / 2.0;
let centerz = shift * (num as f32) / 2.0;
let height = 3.0;

for i in 0usize..num {
    for j in 0usize..num {
        for k in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + height;
            let z = k as f32 * shift - centerz;

            // Build the rigid body.
            let rb = RigidBodyDesc::new()
                .translation(Vector3::new(x, y, z))
                .build();
            // Insert the rigid body to the body set.
            let rb_handle = bodies.insert(rb);

            // Build the collider.
            let co = ColliderDesc::new(cuboid.clone())
                .density(1.0)
                .build(BodyPartHandle(rb_handle, 0));
            // Insert the collider to the body set.
            colliders.insert(co);
        }
    }
}
```

!!! Note
    The `shift` value is used to move each rigid body into a place where it does not overlap with any others. It is
    equal to `(rad + collider_desc.get_margin()) * 2.0` because each collider has a radius equal to `rad`, and a margin
    equal to `collider_desc.get_margin()` (or equivalently `ColliderDesc::<f32>::default_margin()`) since we did not
    specify a collider margin explicitly.

## Running the testbed
Finally, all that remains to do is set-up the testbed and run it:

```rust
// Tell the testbed your ground has an handle equal to `ground_handle`.
// This will make it be drawn in gray.
testbed.set_ground_handle(Some(ground_handle));
// Provide to the testbed all the components of our physics simulation.
testbed.set_world(mechanical_world, geometrical_world, bodies, colliders, joint_constraints, force_generators);
// Adjust the initial camera pose.
testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
```

The `.look_at(...)` method will position the camera at the coordinates $(-4, 1, -4)$ and orient it such that it points toward the point at $(0, 1, 0)$. The `.run()` method will open a window and run the simulation until the window is closed (either by pressing the close button or by pressing the key 'Q').

## More customizations
The **nphysics** testbed provides a few more methods for customizing your simulation. Those methods can be called even if the testbed is created without a `World` (which can be set later with the `.set_world(...)` method).

| Methods                | Description |
|--                      | --          |
| `Testbed::new_empty()` | Create a testbed with no physics world. This is useful to set the color of bodies while you are still initializing the physics world. |
| `.set_world(...)` | Set the dynamic/geometrical worlds as well as the body/collider/joint/force generator sets to be managed by the testbed. |
| `.hide_performance_counters()` | Disable the output to `stdout` of physics timings. |
| `.set_body_color(world, body_handle, color)` | Sets the color of the colliders attached to the specified body. This overrides the default random color. An example can be found on the [collision groups](https://github.com/rustsim/nphysics/blob/master/examples3d/collision_groups3.rs#L133) demo for setting the dynamic bodies colors to green or blue. |
| `.set_collider_color(world, collider_handle, color)` | Sets the color of the specified collider. This overrides the default random color. An example can be found on the [collision groups](https://github.com/rustsim/nphysics/blob/master/examples3d/collision_groups3.rs#L70) demo for setting the static colliders colors to green or blue. |
| `.add_callback(f)`  | Adds a callback to be executed at each render loop, before the next call to `world.step()`. An example can be found on the [sensor](https://github.com/rustsim/nphysics/blob/master/examples3d/sensor3.rs#L94) demo for handling proximity events to recolor of colliders intersecting the sensor. |
