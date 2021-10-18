# rotation-optimization

## What is it about?

A test repository to play around with lie group optimization.
Currently its featuring a simple optimization using ceres.

Two test scenes are provided:
- a very simple scene with 3 cameras and 3 points
- a simple scene with a randomly generated point cloud and a "double grid flight" setup of cameras, i.e. cameras positioned in a grid looking "down" in negative z direction.

The error to evaluate is a simple reprojection error with a simple pinhole camera model only using focal length and principal point. For the experiment we add some noise to the rotations in the above setup and see whether it can recover to the original state via the ceres optimization.

Currently rotation parameter optimization works using both automatic and analytic derivatives on either (global) angle-axis parametrization or quaternion parametrization (using local angle-axis parametrization). In the main.cpp it can be easily switched by commenting in/out the respective lines.

## How to run?

We definitely need the two dependencies: Eigen and Ceres. For that I use conan, so install conan, e.g. via

```
pip install conan
```

Than you should be able to

```
conan install .
```

which will make the dependencies available to the project. You can now create a build folder and run cmake and make in it, e.g.

```
mkdir build && cd build
cmake ..
make
```

Now you should be able to run the app*, e.g. from the build dir

```
bin/app
```

* Tested for ubuntu 20.4 with gcc 9.3.0.
