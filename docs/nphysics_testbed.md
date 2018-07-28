# The nphysics testbed
The **nphysics_testbed2d** and **nphysic_testbed3d** crates provide pure-Rust, WASM-compatible, tools for displaying and interacting easily with a physical scene. They are based on the [kiss3d](https://crates.io/crates/kiss3d) graphics engine. Basically, all you have to do is setup your physics `World`, and then call `Testbed::new(world).run()` to obtain a fully-functional windowed application with 2D or 3D display of every colliders on your `World`. This application will allow some controls like starting/stopping/pausing the simulation, and grabbing an object with the mouse.

!!! Note
    All the interactive demos from this website have been build using those testbeds. Their source codes can be found on the [examples2d](https://github.com/sebcrozet/nphysics/tree/master/examples2d) and [examples3d](https://github.com/sebcrozet/nphysics/tree/master/examples3d) of the **nphysics** repository.

In this chapter we will describe an example of the setup of a 3D physical scene with a pile of boxes. This scene will then be displayed and simulated using the **nphysics_testbed3d** crate. The full code can be found [on github](https://github.com/sebcrozet/nphysics/blob/master/examples3d/boxes3.rs). The end-result corresponds to the 3D [Stack of boxes](/demo_boxes3/) demo. The code fore 2D analogous of this demo can be found [on github](https://github.com/sebcrozet/nphysics/blob/master/examples2d/boxes2.rs) which corresponds to the 2D [Stack of boxes](/demo_boxes2/) demo.

## Initializing the project
First, initialize a new binary project, e.g., using cargo:

```sh
cargo new --bin example_nphysics_testbed_3d
cd example_nphysics_testbed_3d
```

Then, edit your `Cargo.toml` file to add **nphysics-testbed3d**, **nphysics3d**, **ncollide3d** (for collider shapes) and **nalgebra** (for vectors and matrices) as dependencies:

```toml
[package]
name    = "examples_nphysics_testbed_3d"
version = "0.1.0"
authors = [ "you" ]

[dependencies]
nalgebra   = "0.16"
ncollide3d = "0.17"
nphysics3d = "0.9"
nphysics-testbed3d = "0.1"
```

Now modify the `src/main.rs` to add the corresponding `extern crate` directives. Here, we use `na` as an alias for `nalgebra`:

```rust
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;
```

## Setting-up the physics world
The next step is to setup the physics world. First we will `use` a few elements to get the tools we need to initialize our `World`:

```rust
use na::{Isometry3, Point3, Vector3};            // For configuring and positioning bodies.
use ncollide3d::shape::{Cuboid, ShapeHandle};    // Shapes for colliders.
use nphysics3d::object::{BodyHandle, Material};  // Body handle and collider material.
use nphysics3d::volumetric::Volumetric;          // To retrieve the center of mass and inertia properties of a shape.
use nphysics3d::world::World;                    // The physics world to be initialized.
use nphysics_testbed3d::Testbed;                 // The testbed to display/run the simulation.
```

The first thing we want to do is create a new physics world with its default parameters, and change the default gravity (initialized to zero by default) so that is points toward the negative $y$-axis:

```rust
let mut world = World::new();
world.set_gravity(Vector3::y() * -9.81);
```

Then, we initialize the ground represented as a huge box.

```rust
const COLLIDER_MARGIN: f32 = 0.01;

let ground_size = 50.0;
let ground_shape =
    ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
let ground_pos = Isometry3::new(Vector3::y() * -ground_size, na::zero());

world.add_collider(
    COLLIDER_MARGIN,
    ground_shape,
    BodyHandle::ground(),
    ground_pos,
    Material::default(),
);
```
Two elements are notable here:

1. This collider will never move so we don't need a body and can simply attach it to the `BodyHandle::ground()`.
2. We want our cube to have a half-width of 50 exactly. Therefore, we remove `COLLIDER_MARGIN` (here equal to 0.01) from the cuboid half-extents. This is then regained by the margin added by **nphysics** on the collider (and specified as the first parameter of `.add_collider(...)`). This concept of margin is explained in the [section on colliders](rigid_body_simulations_with_contacts/#colliders).

Finally, we can setup our pile of boxes. This time, because our boxes are dynamic, we will have to create one rigid-body for each box, and attach one collider to each rigid-body.

```rust
let num = 7; // There will be 7 * 7 * 7 = 343 boxes here.
let rad = 0.1;
let shift = rad * 2.0;
let centerx = shift * (num as f32) / 2.0;
let centery = shift / 2.0;
let centerz = shift * (num as f32) / 2.0;
let height = 2.0;

let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad - COLLIDER_MARGIN)));
let inertia = geom.inertia(1.0);
let center_of_mass = geom.center_of_mass();

for i in 0usize..num {
    for j in 0usize..num {
        for k in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + height;
            let z = k as f32 * shift - centerz;

            /*
                * Create the rigid-body.
                */
            let pos = Isometry3::new(Vector3::new(x, y, z), na::zero());
            let handle = world.add_rigid_body(pos, inertia, center_of_mass);

            /*
             * Create the collider and attach it to the body we just created.
             */
            world.add_collider(
                COLLIDER_MARGIN,
                geom.clone(),
                handle,
                Isometry3::identity(),
                Material::default(),
            );
        }
    }
}
```

!!! Note
    The `inertia` and `center_of_mass` properties needed for the rigid-body creation are computed from the collision shape here using `geom.inertia(1.0)` (with 1.0 the desired density of the solid) and `geom.center_of_mass()`. This required importing the [Volumetric](/rustdoc/nphysics3d/volumetric/trait.Volumetric.html) trait with `use nphysics3d::volumetric::Volumetric;`.

## Running the testbed
Finally, all that remains to do is set-up the testbed:

```rust
let mut testbed = Testbed::new(world);
testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
testbed.run();
```

The `.look_at(...)` method will position the camera at the coordinates $(-4, 1, -4)$ and orient it such that it points toward the point at $(0, 1, 0)$. The `.run()` method will open a window and run the simulation until the window is closed (either by pressing the close button or by pressing the key 'Q').

## More customizations
The **nphysics** testbed provides a few more methods for customizing your simulation. Those methods can be called even if the testbed is created without a `World` (which can be set later with the `.set_world(...)` method).

| Methods                | Description |
|--                      | --          |
| `Testbed::new_empty()` | Create a testbed with no physics world. This is useful to set the color of bodies while you are still initializing the physics world. |
| `.set_world(world)` | Set the physics world to be managerd by the testbed. |
| `.hide_performance_counters()` | Disable the output to `stdout` of physics timings. |
| `.set_body_color(world, body_handle, color)` | Sets the color of the colliders attached to the specified body. This overrides the default random color. An example can be found on the [collision groups](https://github.com/sebcrozet/nphysics/blob/master/examples3d/collision_groups3.rs#L133) demo for setting the dynamic bodies colors to green or blue. |
| `.set_collider_color(world, collider_handle, color)` | Sets the color of the specified collider. This overrides the default random color. An example can be found on the [collision groups](https://github.com/sebcrozet/nphysics/blob/master/examples3d/collision_groups3.rs#L70) demo for setting the static colliders colors to green or blue. |
| `.add_callback(f)`  | Adds a callback to be executed at each render loop, before the next call to `world.step()`. An example can be found on the [sensor](https://github.com/sebcrozet/nphysics/blob/master/examples3d/sensor3.rs#L94) demo for handling proximity events to recolor of colliders intersecting the sensor. |