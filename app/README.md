Basic Peripersonal Space Demo
=================
This demo illustrates the most important features of learning and using the peripersonal space representation as implemented in this software on the iCub humanoid robot.

Use the main branch of the repository. 

## Prerequisites
1. `CalibCameras`
2. `iCubStartup` (need `iKinCartesianController`, `iKinCartesianSolver`,  `iKinGazeCtrl`, `wholebodyDynamics`, `gravityCompensator`)
3. `Skin_Gui_All` - in fact, the demo operates on hand and forearms only.
4. `iCubGui`
  * For `iCubGui`, connect everything apart from `/wholeBodyDynamics/contacts:o` `/iCubGui/forces`

Note, for correct operation of the PPS, you need to have the skin of the hands and forearms configured correctly. This pertains in particular to:
 1. taxel position files - as per https://github.com/robotology/icub-main/tree/master/app/skinGui/conf/positions - make sure you have the latest ones that are appropriate for your robot in the relevant directories - in particular the left/right_forearm_V2.txt in case you have a V2 skin version robot.
 2. `skinManager` configuration, in particular the `skinManAll.ini` (in `icub-main/app/skinGui/conf`). The `taxelPositionFiles` section specifies which taxel positions files will be loaded. For V2 skin, make sure it refers to V2 forearms files.

There are several modalities that can be run with the `visuoTactileRF` software.
`visuoTactileRF --help`
For the demo, we usually use: 1D version (only position) and red ball as stimulus. 

## Red ball version

The [pf3dtracker](https://github.com/robotology/icub-basic-demos/tree/master/pf3dTracker) is used to track a red ball in the left eye camera image. 
The `frameGrabberGui2` of left camera thus needs to be adjusted appropriately. Rule of thumb: mainly go to `Features (adv)` and pull the `Saturation` knob from about 0.4 to about 0.6.

Application script to use: [pps_1D_pf3dTracker_prelearned.xml](https://github.com/robotology/peripersonal-space/tree/master/app/scripts/pps_1D_pf3dTracker_prelearned.xml)

Note: you may want to run it from the display machine in order to get `pf3dtracker` viewer there. 

### PPS activation demonstration

Using the above script:

1. Run everything apart from `demoAvoidance`
2. Connect everything 

This gives basic PPS functionality - as you approach the robot with red ball, you should see PPS activations on `skinGuis` (green) and `iCubGui` (red arrows).

Alternatively, from terminal: 

```[visuoTactileRF --help]```

```visuotactileRF --taxelsFile taxelsFiles/taxels1D_45cmRF_skinV2_perfect_all.ini --rate 20```

### Avoidance demo

run `demoAvoidance [--autoConnect]`

If not `autoConnect` - need to connect the `/visuoTactileRF/skin_events:o` to `/avoidance/data:i`  - in yarpmanager

N.B. Check if robot can run in impedance mode - otherwise, run in stiff mode (`demoAvoidance --stiff`).

#### Catching variant 
That is robot will reach with activated skin part rather than avoid.

`demoAvoidance --catching`

### Learning illustration

Simplest: start `visuotactileRF` with no taxels file specified. This starts from a blank state. 
Approach with red ball and touch skin. After a while, you will see green activations.
The default `.ini` file is `taxels1D.ini`.  If you save using `rpc` (see below) or at module exit it will be saved to `taxels1D_out.ini`.

#### rpc port

`yarp rpc /visuoTactileRF/rpc:i`
You can `save`, `load` - from `.ini` file, `reset`, `stop` (learning), `rest` (restore learning - call only after a stop).

#### “All in one” demo version
That is, learning and demonstration.

`visuotactileRF --taxelsFile taxelsFiles/taxels1D_45cmRF_skinV2_perfect_allButRightForearm.ini`

With this file, there is blank right forearm - so you can show learning there, with correct (already learned responses) for other body parts.

## “Arbitrary objects” version

Use the script `PPS_with_Optical_Flow` 

(The script has both - `pf3dtracker` - red ball - and "ultimate tracker". These are two variants two track the stimulus - do not run them at the same time.)

For arbitrary objects, the ultimate tracker pipeline is needed:, `motionCUT`, Structure_from_Motion (`SFM` - in reality stereovision), `templatePFTtracker`, and `ultimateTracker` 

1. run all `yarpviews`
2. run `motionCUT`, `templatePFTracker`, `SFM`; `connect` - connects to yarpviews
3. run `ultimateTracker`, connect
4. run `visuoTactileRF`, `visuoTactileWrapper`; `connect`

`demoAvoidance` can be added - see instructions for the red ball above.

## Troubleshooting, adaptation to robot problems (e.g. skin malfunction) etc.

If some skin parts are firing wrong, you can call `visuoTactileRF` with a specific body part - e.g. `--leftHand`, `--leftForeArm`, …
