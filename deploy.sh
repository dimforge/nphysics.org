#! /bin/sh
set -x

nphysics_dir="../rust-dev/nphysics-dev"
nphysics3d_dir="../rust-dev/nphysics-dev/build/nphysics3d"
testbed3d_dir="../rust-dev/nphysics-dev/nphysics_testbed3d"

mkdir -p deploy
mkdir -p static

demos="balls3 boxes3 convex3 trimesh3 cross3 body_status3 collision_groups3 joints3 constraints3 ragdoll3 sensor3"

for demo in $demos; do
    cp $nphysics_dir/examples3d/$demo.rs .
    sed -e "s/{{demo}}/$demo/g" template/Cargo.toml > Cargo.toml
    sed -i '' -e "s#{{nphysics3d_dir}}#$nphysics3d_dir#g" Cargo.toml
    sed -i '' -e "s#{{testbed3d_dir}}#$testbed3d_dir#g" Cargo.toml
    sed -e "s/{{demo}}/$demo/g" template/index.html > static/index.html
    cargo web deploy --target=wasm32-unknown-unknown --release
    cp target/deploy/$demo.js docs/demo/.
    cp target/deploy/$demo.wasm docs/demo/.
    sed -i '' -e "s#$demo.wasm#/demo/$demo.wasm#g" docs/demo/$demo.js
    sed -e "s/{{demo}}/$demo/g" template/demo.md > docs/demo_$demo.md

    rm $demo.rs
done

rm Cargo.toml
rm -r static