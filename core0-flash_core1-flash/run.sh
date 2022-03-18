#!/bin/sh

pushd ../core1-blinky-ram
./build.sh
popd
cargo run --release

