# WASM compatibility
The **nphysics** crate, as well as the [nphysics testbed](nphysics_testbed.md) are compatible with WebAssembly (WASM) by relying on the [stdweb](https://crates.io/crates/stdweb) crate. For example, all the demos on this website (see the Demo menu on top of this page) are the result of compiling the nphysics testbed for the `wasm32-unknown-unknown` target using [cargo-web](https://github.com/koute/cargo-web). This section describes how to setup your development environment to work with **nphysics** for WASM. This is a pretty standard procedure and your file hierarchy should basically look like [that](https://github.com/koute/stdweb/tree/master/examples/canvas) in the end.

## Setup your development tools
First, make sure you work with the nightly compiler:
```sh
rustup toolchain install nightly
rustup default nightly
```

You have to install the `wasm32-unknown-unknown` target with `rustup` as well as the `cargo-web` binary which we will use for compiling and deploying our project:

```sh
rustup target add wasm32-unknown-unknown
cargo install -f cargo-web
```

## Setup and build your project
Now we will create a binary that depends on **nphysics2d** and compile it with `cargo-web`.
First initialize your project with:

```sh
cargo new --bin demo_nphysics_wasm
cd demo_nphysics_wasm
```

Edit the `Cargo.toml` file to include **nphysics2d** as a dependency:

```toml
[package]
name = "demo_nphysics_wasm"
version = "0.1.0"
authors = [ "you" ]

[dependencies]
nphysics2d = "0.9"
```

Before compiling our project, we need to add a `.html` file which will serve as a web page containing our WASM app.
This can be done by adding an `index.html` file on a new `static/` folder.

```html
<!doctype html>
<html lang="en">
	<head>
		<meta charset="utf-8">
		<title>nphysics wasm example</title>
		<style>
			html, body, canvas {
				margin: 0px;
				padding: 0px;
				width: 100%;
				height: 100%;
				overflow: hidden;
			}
		</style>
	</head>
	<body>
		<canvas id="canvas"></canvas>
		<script src="canvas.js"></script>
	</body>
</html>
```

Note that we added here a canvas which will be useful if you intend to do some rendering.
Finally, we can build our application with:

```sh
cargo web deploy --target=wasm32-unknown-unknown --release
```

Further details on the `cargo web` command can be found [here](https://github.com/koute/cargo-web#features).
Using **nphysics** itself on your WASM project is just as simple as using **nphysics** for other platforms: add an `extern crate nphysics2d` (or `extern crate nphysics3d`) to your `main.rs` file and use all the features you need as usual!