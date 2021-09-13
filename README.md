# rotation-optimization

## What is it about?

A test repository to play around with lie group optimization.
Currently its featuring a simple optimization using ceres.

Two test scenes are provided: 
- a very simple scene with 3 cameras and 3 points
- a simple scene with a randomly generated point cloud and a "double grid flight" setup of cameras, i.e. cameras positioned in a grid looking "down" in negative z direction.

The error to evaluate is a simple reprojection error with a simple pinhole camera model only using focal length and principal point. For the experiment we add some noise to the rotations in the above setup and see whether it can recover to the original state via the ceres optimization.

For now it works using automatic derivatives on both (global) angle-axis parametrization as well as quaternion parametrization (using local tangent space parametrization).
The version using an analytic derivative is still WIP, there is still something wrong and it won't converge yet. Interestingly the web is quite silent about the analytic derivative, so far I could not find a source to refer to. 
The closest to this problem is one approach trying to trick ceres and its local parametrization, s.t. the derivative dprojection/dtheta(3D tangent space) can be implemented in the Evaluate() function instead of dprojection/dq(4D quaternion manifold). I will update the repo + page once I get this working.

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

Now you should be able to run the app, e.g. from the build dir

```
bin/app
```
