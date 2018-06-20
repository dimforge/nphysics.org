#! /bin/sh

out_dir=./docs/rustdoc
nphysics_dir=../rust-dev/nphysics-dev

echo "Generating the documentation..."
cd $nphysics_dir; cargo doc --no-deps
cd -
rm -rf docs/rustdoc
cp -r $nphysics_dir/target/doc $out_dir

echo "... documentation generated!"

./fix_rustdoc.sh
