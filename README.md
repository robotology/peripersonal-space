Peripersonal Space
=================

This repository deals with the implementation of peripersonal space representations on the iCub humanoid robot.

## Overall Idea

The basic principle concerns the use of the tactile system in order to build a representation of space immediately surrounding the body - i.e. peripersonal space. In particular, the iCub skin acts as a reinforcement for the visual system, with the goal of enhancing the perception of the surrounding world. By exploiting a temporal and spatial congruence between a purely visual event (e.g. an object approaching the robot's body) and a purely tactile event (e.g. the same object eventually touching a skin part), a representation is learned that allows the robot to autonomously establish a margin of safety around its body through interaction with the environment - extending its cutaneous tactile space into the space surrounding it.

We consider a scenario where external objects are approaching individual skin parts. A volume is chosen to demarcate a theoretical visual "receptive field" around every taxel. Learning then proceeds in a distributed, event-driven manner - every taxel stores and continuously updates a record of the count of positive (resulting in contact) and negative examples it has encountered.

## Video

For a video on the peripersonal space, click on the image below (you will be redirected to a youtube video):

[![peripersonal space video](http://img.youtube.com/vi/3IaXxNwC_7E/0.jpg)](http://www.youtube.com/watch?v=3IaXxNwC_7E)

## Demo
To demonstrate the functionality of this software - similarly to the above video - you can find instructions under [app/README_DEMO.md](https://github.com/robotology/peripersonal-space/blob/master/app/README_DEMO.md)

## Publications

Roncone A, Hoffmann M, Pattacini U, Fadiga L, Metta G (2016) Peripersonal Space and Margin of Safety around the Body: Learning Visuo-Tactile Associations in a Humanoid Robot with Artificial Skin. PLoS ONE 11(10): e0163713. [[doi:10.1371/journal.pone.0163713]](http://dx.doi.org/10.1371/journal.pone.0163713).

Roncone, A.; Hoffmann, M.; Pattacini, U. & Metta, G. (2015), Learning peripersonal space representation through artificial skin for avoidance and reaching with whole body surface, in 'Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on', pp. 3366-3373. [[IEEE Xplore]](http://dx.doi.org/10.1109/IROS.2015.7353846) [[postprint]](https://sites.google.com/site/matejhof/publications/RonconeEtAl_PPS_IROS2015postprint.pdf?attredirects=0).

## Repository Structure

### PeriPersonal Space Library

The library is located under the `lib` directory. It wraps some useful methods and utilities used throughout the code. Most notably:
 * A Parzen Window Estimator class, both in 1D and 2D.
 * An inverse kinematics solver for the 12DoF or 14DoF self-touch task
 * A revised, improved `iKin` library with non standard kinematics in order to allow for reading a DH chain from the end-effector up to the base of the kinematic chain.

### Modules

The modules are located under the `modules` directory. Some of them include:
 * `doubleTouch`: this module implements the double touch paradigm. A video describing this can be found here  (https://youtu.be/pfse424t5mQ)
 * `ultimateTracker`: A 3D tracker able to track objects with stereoVision and disparity map.
 * `visuoTactileRF`: The core module which manages the peripersonal space and does the actual learning of visuo-tactile representation
 * `visuoTactileWrapper`: A corollary module which "wraps" a variety of input signals used for the peripersonal space learning (e.g. double touch signal, optical flow, 3D optical flow from the `ultimateTracker`, ...) and sends them to the `visuoTactileRF` module

## Authors

 * [Alessandro Roncone (@alecive)](https://github.com/alecive)
 * [Matej Hoffmann (@matejhof)](https://github.com/matejhof)
 * [Ugo Pattacini (@pattacini)](https://github.com/pattacini)
