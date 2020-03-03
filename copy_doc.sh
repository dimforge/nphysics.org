#! /bin/sh

out_dir=./docs/rustdoc
nphysics_dir=../nphysics

echo "Generating the documentation..."
cd $nphysics_dir; cargo doc -p nphysics2d -p nphysics3d --no-deps
cd -
rm -rf docs/rustdoc
cp -r $nphysics_dir/target/doc $out_dir

echo "... documentation generated!"

./fix_rustdoc.sh
