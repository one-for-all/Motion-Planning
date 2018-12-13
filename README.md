# 6-881 Final Project - Motion Planning
[![Build Status](https://travis-ci.org/RobotLocomotion/6-881-examples.svg?branch=master)](https://travis-ci.org/RobotLocomotion/6-881-examples)

This repository hosts the final project for [MIT 6.881 Intelligent Robot Manipulation](https://manipulation.csail.mit.edu/) during 2019 Fall.
The project uses several [tools](https://github.com/RobotLocomotion/6-881-examples) accompanying the course.

## Project Description
This project implements and evaluates three robotic motion planning algorithms:
* Rapidly-exploring Random Tree (RRT) 
* Rapidly-exploring Random Tree Star (RRT*)
* Lazy Shortest Path (LazySP)

The algorithms are developed in the context of the KUKA iiwa arm, using the [Drake](https://drake.mit.edu/) toolbox.

## Pre-reqs
In the root directory of this repository, run the following command in a terminal to build a docker image that includes Drake and denpendencies for PDDLStream:
```bash
$ docker build -t mit6881 -f ubuntu16_04_mit6881.dockerfile --build-arg DRAKE_VERSION=20181203 .
``` 

## Use
In the root directory of this repository, run 
```bash
$ python ./docker_run.py --os [your operating system]
``` 
where `[your operating system]` should be replaced with `mac` or `linux`. This command will start a docker container (virtual machine) with the docker image you have created. The `6-881-examples` folder on the host machine (your laptop/desktop) is mounted to `/6-881-examples` in the docker container. 

In the docker container, run
```bash
$ terminator
```
to launch `terminator`, a popular terminal multiplexer on linux. The terminator window is launched with a dark green background to distinct itself from terminals running on the host machine. 





