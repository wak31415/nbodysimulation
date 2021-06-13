# nbodysimulation

## Compilation instructions:
* Install `rustup` from either the distribution's package manager or [the official website](https://rustup.rs/)
* Download the compiler and the `cargo` package manager for the `stable` version of Rust by running `rustup default stable`
* Download the packages, compile the project and run it by running `cargo run` or `cargo run <num_bodies>`

## Running `nbodysimulation`

* `cargo run` reads the `starting_configuration.json` for reproducible results
* `cargo run <num_bodies>` generates a fixed number of bodies over a finite space with random locations, positions and velocities

The master branch implements the safe implementation. If you would like to try out the raw pointer implementation using the unsafe code blocks, you can checkout the `raw_pointer_concurrency` branch.