#!/bin/sh

cd ../core1-blinky-ram
./build.sh
cd -
cargo run --release

