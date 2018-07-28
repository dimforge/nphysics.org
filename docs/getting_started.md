# Getting started
**nphysics** relies on the official Rust package manager
[Cargo](http://crates.io) for dependency resolution and compilation. Therefore,
making **nphysics** ready to be used by your project is simply a matter of
adding a new dependency to your `Cargo.toml` file. You can either use the **nphysics2d**
crate for 2D physics simulation or the **nphysics3d** crate for 3D physics simulation. You can even use both
if you need both 2D and 3D in your application. Note that you will probably
need **nalgebra** as well because it defines algebraic entities
(vectors, points, transformation matrices) used by most types of **nphysics**. Similarly,
you will probably need **ncollide2d** or **ncollide3d** which defines all the geometric
entities for collision shapes (spheres, cubes, triangle meshes, etc.)

```toml
[dependencies]
nalgebra = "0.16"
# Choose the one you need, or both.
ncollide2d = "0.17"
ncollide3d = "0.17"
# Choose the one you need, or both.
nphysics2d = "0.9"
nphysics3d = "0.9"
```

Until **nphysics** reaches 1.0, it is strongly recommended to always use its
latest version, though you might encounter breaking changes from time to time.
Once your `Cargo.toml` file is set up, the corresponding crate must be imported
by your project with the usual `extern crate` directive:
```rust
extern crate nphysics2d; // If you need 2D.
extern crate nphysics3d; // If you need 3D.
```

## Cargo example
Here is an example of Cargo file for compiling an executable depending on **nphysics**:

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#cargo_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#cargo_3D">3D example</a></li>
</ul>

<div class="tab-content" markdown="1">
  <div id="cargo_2D" class="tab-pane in active">
```toml
[package]
name    = "example-using-nphysics"
version = "0.0.0"
authors = [ "You" ]

[dependencies]
nalgebra = "0.16"
ncollide2d = "0.17"
nphysics2d = "0.9"

[[bin]]
name = "example"
path = "./example.rs"
```
  </div>
  <div id="cargo_3D" class="tab-pane">
```toml
[package]
name    = "example-using-nphysics"
version = "0.0.0"
authors = [ "You" ]

[dependencies]
nalgebra = "0.16"
ncollide3d = "0.17"
nphysics3d = "0.9"

[[bin]]
name = "example"
path = "./example.rs"
```
  </div>
</div>