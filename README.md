Peripersonal Space
=================

This repository deals with the implementation of peripersonal space representations on the iCub humanoid robot.

## Overall Idea

The basic principle concerns the use of the tactile system is used in order to build a representation of space immediately surrounding the body - i.e. peripersonal space. In particular, the iCub skin acts as a reinforcement for the visual system, with the goal of enhancing the perception of the surrounding world. By exploiting a temporal and spatial congruence between a purely visual event (e.g. an object approaching the robot's body) and a purely tactile event (e.g. the same object eventually touching a skin part), a representation has been learned that allows the robot to autonomously establish a margin of safety around its body through interaction with the environment - extending its cutaneous tactile space into the space surrounding it.

We considered a scenario where external objects were approaching individual skin parts. A volume was chosen to demarcate a theoretical visual "receptive field" around every taxel. Learning then proceeded in a distributed, event-driven manner - every taxel stored and continuously updated a record of the count of positive (resulting in contact) and negative examples it has encountered.

## Authors

 * Alessandro Roncone (@alecive)
 * Matej Hoffman (@matejhof)
 * Ugo Pattacini (@pattacini)
