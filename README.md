# rotation-optimization

## What is it about?

A test repository to play around with optimization on the 3D rotation Lie Group.
Currently its featuring a simple optimization setup using ceres and
a collection of different parametrizations to compare. Two test scenes are provided:
- a very simple scene with 3 cameras and 3 points
- a simple scene with a randomly generated point cloud and a "double grid flight" setup of cameras,
i.e. cameras positioned in a grid looking "down" in negative z direction.

The error to evaluate is a simple reprojection error with a simple pinhole camera model
using only focal length and principal point. For the experiment we add some noise to the rotations
in the above setup and see whether it can recover to the original state using ceres' default
optimization settings.

The optimization works for all provided parametrizations (automatic and analytic). Currently
the switch between Quaternion-, RotationMatrix- or AngleAxis-parameterization is set by an alias
in the main.cpp and can easily be switched by commenting in/out the respective lines.
All of them feature a cost function with automatic derivative. For Quaternion- and RotationMatrix-
parameterization I further provided several implementations for the analytic derivative, that can
be called using bool switches in the implementation. (TODO: Propagate to the main app interface.)

On [my webpage](https://zaenker.jimdofree.com/diverse/quaternions-ceres/) I wrote down the
mathematical derivation for the jacobian split used in this code as well as some interesting findings.

## How to run?

We definitely need the two dependencies: Eigen and Ceres. For that I use conan, so to run the app install conan, e.g. via

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
