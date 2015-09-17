Peripersonal Space
=================

This repository deals with the implementation of peripersonal space representations on the iCub humanoid robot.

## Overall Idea

The basic principle concerns the use of the tactile system in order to build a representation of space immediately surrounding the body - i.e. peripersonal space. In particular, the iCub skin acts as a reinforcement for the visual system, with the goal of enhancing the perception of the surrounding world. By exploiting a temporal and spatial congruence between a purely visual event (e.g. an object approaching the robot's body) and a purely tactile event (e.g. the same object eventually touching a skin part), a representation is learned that allows the robot to autonomously establish a margin of safety around its body through interaction with the environment - extending its cutaneous tactile space into the space surrounding it.

We consider a scenario where external objects are approaching individual skin parts. A volume is chosen to demarcate a theoretical visual "receptive field" around every taxel. Learning then proceeds in a distributed, event-driven manner - every taxel stores and continuously updates a record of the count of positive (resulting in contact) and negative examples it has encountered.

## Video

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/hVQginjCTC0/0.jpg)](http://www.youtube.com/watch?v=hVQginjCTC0)

## Repository Structure

### PeriPersonal Space Library

The library is located under the `lib` directory. It wraps some useful methods and utilities used throughout the code. Most notably:
 * A Parzen Window Estimator class, both in 1D and 2D.
 * An inverse kinematics solver for the 12DoF or 14DoF self-touch task
 * A revised, improved `iKin` library with non standard kinematics in order to allow for reading a DH chain from the end-effector up to the base of the kinematic chain.

### Modules

The modules are located under the `modules` directory. Some of them include:
 * `doubleTouch`: this module implements the double touch paradigm. A video describing this can be found here  (https://youtu.be/hVQginjCTC0)
 * `ultimateTracker`:
 * `visuoTactileRF`:
 * `visuoTactileWrapper`:

## Authors

 * Alessandro Roncone (@alecive)
 * Matej Hoffman (@matejhof)
 * Ugo Pattacini (@pattacini)
