#! /bin/sh
set -x

nphysics_dir="../nphysics"
nphysics3d_dir="../nphysics/build/nphysics3d"
testbed3d_dir="../nphysics/build/nphysics_testbed3d"
nphysics2d_dir="../nphysics/build/nphysics2d"
testbed2d_dir="../nphysics/build/nphysics_testbed2d"

mkdir -p deploy
mkdir -p static
mkdir -p docs/demo

demos="all_examples3" # "capsules3 heightfield3 balls3 boxes3 convex3 trimesh3 cross3 kinematic3 collision_groups3 multibody3 constraints3 ragdoll3 sensor3 conveyor_belt3 plasticity3 fem_volume3"

for demo in $demos; do
    cp $nphysics_dir/examples3d/*.rs .
    sed -e "s/{{demo}}/$demo/g" template/Cargo3.toml > Cargo.toml
    sed -i -e "s#{{nphysics3d_dir}}#$nphysics3d_dir#g" Cargo.toml
    sed -i -e "s#{{testbed3d_dir}}#$testbed3d_dir#g" Cargo.toml
    CARGO_INCREMENTAL=0 cargo web deploy --target=wasm32-unknown-unknown --release
    cp target/deploy/$demo.js docs/demo/.
    cp target/deploy/$demo.wasm docs/demo/.
    sed -i -e "s#$demo.wasm#/demo/$demo.wasm#g" docs/demo/$demo.js
    sed -e "s/{{demo}}/$demo/g" template/demo.md > docs/$demo.md

    rm *.rs
done

demos="all_examples2" # "capsules2 heightfield2 balls2 boxes2 convex2 polyline2 cross2 kinematic2 collision_groups2 multibody2 constraints2 ragdoll2 sensor2 conveyor_belt2 plasticity2 fem_surface2"
for demo in $demos; do
    cp $nphysics_dir/examples2d/*.rs .
    sed -e "s/{{demo}}/$demo/g" template/Cargo2.toml > Cargo.toml
    sed -i -e "s#{{nphysics2d_dir}}#$nphysics2d_dir#g" Cargo.toml
    sed -i -e "s#{{testbed2d_dir}}#$testbed2d_dir#g" Cargo.toml
    CARGO_INCREMENTAL=0 cargo web deploy --target=wasm32-unknown-unknown --release
    cp target/deploy/$demo.js docs/demo/.
    cp target/deploy/$demo.wasm docs/demo/.
    sed -i -e "s#$demo.wasm#/demo/$demo.wasm#g" docs/demo/$demo.js
    sed -e "s/{{demo}}/$demo/g" template/demo.md > docs/$demo.md

    rm *.rs
done

rm Cargo.toml
rm -r static
