# Trajectory Optimization with Hard Contacts

This is an unofficial implementation of [1]. The algorithm makes use of concepts from bi-level optimization to find
gradients of the system dynamics including the constraint forces and subsequently solve the optimal control problem with
the unconstrained iLQR algorithm.

Click the [link](https://youtu.be/FF3h_-KwoyE) to watch the result video.

## Implementation

The robot used here has nearly the same configurations as in [1], which is a planer hopper with 4 degree of freedoms.
The robot's kinetic and dynamics models are generated using [`RobCoGen`](https://robcogenteam.bitbucket.io/).

The time stepping scheme in [1] has some problem where the end effector is easy to penetrate the ground, which
significantly hinders the optimization. Some fixes are taken place according to [2].

To improve the stability of iLQR iterations, regularization and line-search schemes in [3] are used.

## Running

This project is written in `c++` and is only tested under Linux.

### Requirements

The code depends on the linear algebra library [`Eigen3`](https://eigen.tuxfamily.org).

[`RobCoGen`](https://robcogenteam.bitbucket.io/) headers are required to be installed, even if not switching into other
robot models.

The code also depends on [`CppAD`](https://github.com/coin-or/CppAD.git)
and [`CppADCodeGen`](https://github.com/joaoleal/CppADCodeGen.git). These are both `c++` header-only libraries that
support fast auto-differentiation.

### Clone and Compile

#### Clone the repository

```shell
$ git clone https://github.com/cryscan/to-ihc-2.git
$ cd to-ihc-2
```

#### Build the project

```shell
$ mkdir build
$ cd build
$ cmake ..
$ make
```

#### Run the code

To run the code, create a `in.txt` under the `build` folder with the following contents:

```
# Iterations and defect_limit (unused)
10 -1.0

# Initial configuration
-0.18 0.0 0.785 -1.57 0.0 0.0 0.0 0.0

# Target configuration
0.0 1.0 0.785 -1.57 0.0 0.0 0.0 0.0

# Running cost
0.1 0.1 1.0 1.0 0.01 0.01 0.1 0.1
0.01 0.01

# Final cost
10000.0 10000.0 100.0 100.0 1.0 1.0 1.0 1.0
0.0 0.0

# Horizon and initial trajectory file
200 ../init.txt
```

Then run

```shell
$ ./control
```

In the first run it generates codes for dynamics, cost, etc. and compiles them, which may take some time. It won't
compile again in future runs if the compiled dynamic libraries exist.

Very quickly the result trajectory will be stored in `out.txt`. It can be visualized by running

```shell
$ python3 ../visual.py
```

Alternatively, running

```shell
$ ./simulation
```

generates a rollout without any control signals.

# Reference

1. J. Carius, R. Ranftl, V. Koltun and M. Hutter, "Trajectory Optimization With Implicit Hard Contacts," in IEEE
   Robotics and Automation Letters, vol. 3, no. 4, pp. 3316-3323, Oct. 2018, doi: 10.1109/LRA.2018.2852785.
2. J. Carius, R. Ranftl, V. Koltun and M. Hutter, "Trajectory Optimization for Legged Robots With Slipping Motions," in
   IEEE Robotics and Automation Letters, vol. 4, no. 3, pp. 3013-3020, July 2019, doi: 10.1109/LRA.2019.2923967.
3. Y. Tassa, T. Erez and E. Todorov, "Synthesis and stabilization of complex behaviors through online trajectory
   optimization," 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems, 2012, pp. 4906-4913, doi:
   10.1109/IROS.2012.6386025.