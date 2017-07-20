# OpenSirka
This is a collection of programs to do Inertial Motion Capturing, that is motion capturing with gyro- and accelerometers only; without magnetometers.

The programs originate from the research project [SIRKA][2], which was aimed at developing motion capturing workwear. For instance, to make the workwear of workers in the maritime industry provide real-time estimates of the workers' postures. The main challenge of [SIRKA][2], which these programs solve, was to provide complete and drift-free estimates of the relative 3d-orientations of adjacent limbs without any magnetic or external sensing whatsoever. How this works is described in [Posture from Motion][1].

At present, not all software components developed during SIRKA are part of this repository. Notably, the mechanism to provide accumulated inertial measurements to the estimator itself in a synchronized manner is missing (yet). However, properly formed example data may be found in the [example_data](example_data/) directory, which has been recorded using the actual SIRKA suit in 2016.

# Building and Installing
There are three components to build and install. [libimureading](libimureading/) is a small library to read the recorded sensor data from files. It is needed for both the [calibrator](calibrator/) and the [postureestimator](postureestimator/). The posture estimator, well, estimates postures and provides a visualization of the result while doing so. The posture estimator needs a specification of where the sensors are relative to the joints adjacent to the bodies the sensors are mounted on. Such a specification is provided with the example data. It can also be obtained using the [calibrator](calibrator/) automatically from recorded sensor data (which is, in fact, how the provided specification was obtained). Thus, the calibrator _does not_ calibrate sensors. Instead it calibrates the placement of the sensors and the skeleton of the human whose motions are to be captured by the posture estimator.

## Build and install libimureading
### Dependencies
TODO: List dependencies
### Building
TODO: Build and install instructions

## Build and install the calibrator
If you just want to see the estimator work on the example data, you can skip this, because the calibration results are provided with the examples data.
### Dependencies
TODO: List dependencies
### Building
TODO: Build and install instructions
### Usage
TODO: Usage instructions.

## Build and install the posture estimator
### Dependencies
TODO: List dependencies
### Building
TODO: Build and install instructions
### Usage
TODO: Usage instructions.

[1]: http://www.informatik.uni-bremen.de/agbkb/publikationen/bibsearch/detail_e.htm?pk_int=3372
[2]: https://www-cps.hb.dfki.de/research/projects/SIRKA
