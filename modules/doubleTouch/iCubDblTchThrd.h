/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 * This thread detects a touched taxel on the skin (through readings from the
 * skinContactList port), and it moves the "controlateral" limb toward
 * the affected taxel.
*/

#ifndef __DOUBLETOUCHTHREAD_H__
#define __DOUBLETOUCHTHREAD_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <gsl/gsl_math.h>

#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>

#include <iCub/periPersonalSpace/iCubDblTchSlv.h>
#include <iCub/periPersonalSpace/utils.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::skinDynLib;

using namespace std;

class doubleTouchThread: public RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string name;
    // Name of the robot (to address the module toward icub or icubSim):
    string robot;
    // Type of the chain (either "R2L", "L2R", or "both")
    string type;
    // Flag to know if a recording session is needed.
    int record;
    // Name of the file that will store data
    string filename;
    // Name of the file that will store data
    string color;

    /***************************************************************************/
    // CONTACT-RELATED VARIABLES:
    Vector cntctPosLink;    // Position in i-th link RF
    Vector cntctPosWRF;     // Position in WRF
    Vector cntctPosEE;      // Position in end-eff RF
    Vector cntctNormDir;    // Normal Direction
    Matrix cntctH0;         // RT matrix located in the contact with the 
                            // x axis normal to the cover
    int cntctLinkNum;       // Link number
    double cntctPressure;   // Pressure
    string cntctArm;        // Affected Arm
    skinContact cntctSkin;  // SkinContact
    string cntctSkinPart;   // SkinPart (verbose form)
    string currentTask;     // current task (either R2L or L2R)

    /***************************************************************************/
    // INTERNAL VARIABLES:
    int  step;    // Flag to know in which step the thread is
    bool recFlag; // Flag to know if the recording module has to record
    int  iter;    // iterator to keep track of the recording steps

    // SkinPart to be handled
    // it can be either 2 (forearm_left), 5 (forearm_right)
    // or even 3 (upperarm_left) and 6 (upperarm_right) [TO DO]
    int R2LskinPart;
    int L2RskinPart;

    // Port that reads contacts:
    BufferedPort<iCub::skinDynLib::skinContactList> *skinPort;
    BufferedPort<Bottle> *outPort;

    // Driver for "classical" interfaces
    PolyDriver       ddR; // right arm device driver
    PolyDriver       ddL; // left arm  device driver
    PolyDriver       ddG; // gaze controller  driver

    // "Classical" interfaces - RIGHT ARM
    IEncoders         *iencsR;
    IPositionControl  *iposR;
    IControlMode      *ictrlR;
    IImpedanceControl *iimpR;
    IControlLimits    *ilimR;
    Vector            *encsR;
    iCubArm           *armR;
    int jntsR;
    // "Classical" interfaces - LEFT ARM
    IEncoders         *iencsL;
    IPositionControl  *iposL;
    IControlMode      *ictrlL;
    IImpedanceControl *iimpL;
    IControlLimits    *ilimL;
    Vector            *encsL;
    iCubArm           *armL;
    int jntsL;

    // Wrappers (to be arm-agnostic) M = Master S = Slave
    IPositionControl  *iposM;   IPositionControl  *iposS;
    IEncoders         *iencsM;  IEncoders         *iencsS;
    IControlMode      *ictrlM;  IControlMode      *ictrlS;
    Vector            *encsM;   Vector            *encsS;
    iCubArm           *armM;    iCubArm           *armS;

    // Gaze controller interface
    IGazeControl       *igaze;
    int contextGaze;

    // IPOPT STUFF
    iCubDoubleTouch_Variables *g;      // guess
    iCubDoubleTouch_Variables *s0;     // solution - waypoint
    iCubDoubleTouch_Variables *s1;     // solution
    iCubDoubleTouch_Solver    *slv;    // solver
    Vector solution;

    // CUSTOM LIMB (for testing the achievement of the task)
    iCubCustomLimb *testLimbL2R;
    iCubCustomLimb *testLimbR2L;

    // CHECKMOTIONDONE VARIABLES:
    Vector oldEER;
    Vector oldEEL;

    /**
    * Aligns joint bounds according to the actual limits of the robot
    */
    bool alignJointsBounds();

    /**
    * Checks if the motion has finished. To be implemented in future releases
    * (the old version is not available any more because of the changes)
    */
    bool checkMotionDone();

    /**
    * Creates a delay (what for? In order to remember
    * that I have to improve my module and remove this!)
    */
    void delay(int sec);

    /**
    * Reads the contact from either /skinManager/skin_events:o or
    * /wholeBodyDynamics/contacts:o , and handles the skinContacts
    */
    void detectContact(skinContactList *_sCL);

    /**
    * Finds the proper H0 for the limb
    * @param sc is the skinContact for which the H0 has to be computed
    */
    Matrix findH0(skinContact &sc);

    /**
    * Moves arms to starting (home) position
    */
    void goToRest();

    /**
    * Solves the Inverse Kinematic task
    * @param s is the type of movement (either "standard" or "waypoint")
    */
    void solveIK()    { solveIK("standard"); };

    void solveIK(string s);

    /**
    * Goes to the configuration found by solveIK()
    */
    void goToTaxel();
    void goToTaxelMaster();
    void goToTaxelSlave();

    /**
    * Handles the gaze controller for each step of the module
    */
    bool handleGaze();

    /**
    * Locates the contact in World Reference Frame's coordinates
    * @param sc is the skinContact for which the location is computed
    */
    Vector locateContact(skinContact &sc);

    /**
    * Find the final configuration for the gaze interface to look at.
    */
    Vector findFinalConfiguration();

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

    /**
    * Verify the degree of achievements of the task, by reading joints encoders
    */
    void testAchievement();

    /**
    * Verify if the skin is touched. If so, it stores data.
    */
    bool testAchievement2(skinContactList *_sCL);

    /**
    * If testAchievement2 fails, we need a way to come back to the starting point!
    * This way will be a touch in the forearm :)
    */
    bool exitFromDeadlock(skinContactList *_sCL);

public:
    // CONSTRUCTOR
    doubleTouchThread(int _rate, const string &_name, const string &_robot,
                      int _v, const string _type, int _record, string _filename, string _color);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
    // AFTERSTART
    // void afterStart(bool s)
    // {
    //     if (s)
    //         printMessage(0,"Thread started successfully :)\n");
    //     else
    //         printMessage(0,"Thread did not start :(\n");
    // }
};

#endif

// PERSONAL NOTES (remember to remove them sooner or later!)
// iCub/main/src/tools/skinManagerGui
// iCub/main/src/modules/skinManager
// iCub/main/src/libraries/skinDynLib
//
// A skinContactList is represented as a list of lists
// where each list is a skinContact
// basically, it's a vector of skinContact
//
// A skinContact is a list of 8 elements that are:
// - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
// - a list of 3 double, i.e. the CoP
// - a list of 3 double, i.e. the force
// - a list of 3 double, i.e. the moment
// - a list of 3 double, i.e. the geometric center
// - a list of 3 double, i.e. the normal direction
// - a list of N int, i.e. the active taxel ids
// - a double, i.e. the pressure
// 
// ((48725 4 4 5) (-0.017 0.062 -0.036) (0.476424 0.109944 0.611614)
// (0.0 0.0 0.0) (-0.017 0.062 -0.036) (-0.585 -0.135 -0.751) (134) 16.288001)

