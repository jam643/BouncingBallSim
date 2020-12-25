# Bouncing Ball Simulator
Simple real-time simulation of elastic ball collisions in C++ using SFML to animate. My goal with creating this was to learn C++, cmake, and conan. 

You can interact with the balls (left click = repulsive force, right click = attractive force).

![gif](bouncing_balls.gif)

## Installation (linux)
* Prereqs: cmake, conan, pipenv
* `pipenv shell`
* `pipenv sync`
* `cd build/`
* `conan install ..` (to use a specific profile add `--profile=<name>`)
* `cmake ..`
* `make`
* `./bin/BouncingBalls`
