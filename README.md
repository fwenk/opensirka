# OpenSirka
This is a collection of programs to do Inertial Motion Capturing, that is motion capturing with gyro- and accelerometers only; without magnetometers.

The programs originate from the research project [SIRKA][2], which was aimed at developing motion capturing workwear. For instance, to make the workwear of workers in the maritime industry provide real-time estimates of the workers' postures. The main challenge of [SIRKA][2], which these programs solve, was to provide complete and drift-free estimates of the relative 3d-orientations of adjacent limbs without any magnetic or external sensing whatsoever. How this works is described in [Posture from Motion][1].

At present, not all software components developed during SIRKA are part of this repository. Notably, the mechanism to provide accumulated inertial measurements to the estimator itself in a synchronized manner is missing (yet). However, properly formed example data may be found in the [example_data](example_data/) directory, which has been recorded using the actual SIRKA suit in 2016.

# Building and Installing
There are three components to build and install. [libimureading](libimureading/) is a small library to read the recorded sensor data from files. It is needed for both the [calibrator](calibrator/) and the [postureestimator](postureestimator/). The posture estimator, well, estimates postures and provides a visualization of the result while doing so. The posture estimator needs a specification of where the sensors are relative to the joints adjacent to the bodies the sensors are mounted on. Such a specification is provided with the example data. It can also be obtained using the [calibrator](calibrator/) automatically from recorded sensor data (which is, in fact, how the provided specification was obtained). Thus, the calibrator _does not_ calibrate sensors. Instead it calibrates the placement of the sensors and the skeleton of the human whose motions are to be captured by the posture estimator.

## Build and install libimureading
### Dependencies
You need the usual compilers and standard libraries as well as cmake, boost and Eigen. Building has been tested both on mac os and Linux (Fedora 25).
### Building
In a freshly cloned repository, create a build directory inside the [libimureading](libimureading/) directory. Change into the build directory and create Makefiles. Change the install prefix to whatever suits you. On most platforms, `/usr/local` is the default. 
```
mkdir libimureading/build
cd libimureading/build
cmake -DCMAKE_INSTALL_PREFIX=~/delete_me/demo_installation \
-DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" ../
```
The generated Makefiles are then used to build and install:
```
$ make install
```

## Build the calibrator
If you just want to see the estimator work on the example data, you can skip this, because the calibration results are provided with the examples data.
### Dependencies
TODO: List dependencies
### Building
TODO: Build and install instructions
### Usage
TODO: Usage instructions.

## Build the posture estimator
There is a demo [video][3] of installing and starting the posture estimator which approximately follows the instructions below.
### Dependencies
You need all the dependencies of libimureading, libimureading itself and OpenSceneGraph.
### Building
You know the drill by know; in the OpenSirka-Repository, create a build directory inside the [postureestimator](postureestimator/) directory. Change into the build directory and create Makefiles. If you specified the install prefix when building the libimureading library, you need to specify it here as the imu reading prefix.
```
cd postureestimator/build
cmake -DCMAKE_BUILD_TYPE=Release \
-DIMU_READING_PREFIX=~/delete_me/demo_installation \
-G "Unix Makefiles" ../
```
Build it with
```
make
```
This should have produced a binary called `M2AccuReplay`, which contains a small driver program for the posture estimator.
### Usage
Still in the build directory, which contains the binary `M2AccuReplay` just created, you can start the posture estimator by specifying the accumulated example sensor data and the location of the calibration.
```
./M2AccuReplay --realtime 1 --replay ../../example_data/imu_data \
--calibrations ../../example_data/output/calibration \
--initial_orientations ../../example_data/output/calibration
```

On program start, a window with a rendering of the posture estimate appears. Joints are drawn in red, sensors in green. Joints are connected by blue rods to sensors which are mounted on adjacent bodies. To get a better sense of the skeleton, you may press the `j` key to toggle red connections between adjacent joints.

`--realtime 1` lets the driver program replay the sensor data (approximately) as fast as it was recorded (as opposed to as fast as possible). `--replay <path>` specifies the path to the sensor data, `--calibrations <path>` the path to the output of the calibrator. `--initial_orientations <path>` specifies the path to the directory containing orientations used to initialize the estimator with. These are part of the calibrator's output.

However, specifying initial orientations is optional. If initial orientations are not provided, it is very, very likely that the orientation estimates are wrong in the beginning: The rotations around the vertical axis will be wrong intially. The estimator corrects these errors when the whole skeleton accelerates, e.g. starts and stops to walk. However, as long the the wearer of the suit stands completely still, no such accelerations occur.

[1]: http://www.informatik.uni-bremen.de/agbkb/publikationen/bibsearch/detail_e.htm?pk_int=3372
[2]: https://www-cps.hb.dfki.de/research/projects/SIRKA
[3]: https://www.dropbox.com/s/1shwxhybmyamolw/postureestimator_demo_1080.mp4?dl=0
