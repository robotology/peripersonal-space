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

#ifndef __TACTILESERVOTHREAD_H__
#define __TACTILESERVOTHREAD_H__

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

using namespace std;

class tactileServoThread: public RateThread
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

    int startup_left_context_id;
    int startup_right_context_id;
    int contextGaze;
//     
    /***************************************************************************/
    // INTERNAL VARIABLES:

    // Driver for "classical" interfaces
    PolyDriver       ddL;       // left arm  device driver
    PolyDriver       ddR;       // right arm device driver
    PolyDriver       ddT;
    PolyDriver       dcartL;    // left arm  device driver
    PolyDriver       dcartR;    // right arm device driver
    PolyDriver       ddG;
    BufferedPort<Bottle> ft_inL, ft_inR;

    // "Classical" interfaces - LEFT ARM
    IEncoders         *iencsL;
    IPositionControl  *iposL;
    ICartesianControl *icartL;
    IInteractionMode  *imodeL;
    IImpedanceControl *iimpL;
    IControlLimits    *ilimL;
    Vector            *encsL;
    iCubArm           *armL;
    int jntsL;

    // "Classical" interfaces - RIGHT ARM
    IEncoders         *iencsR;
    IPositionControl  *iposR;
    ICartesianControl *icartR;
    IInteractionMode  *imodeR;
    IImpedanceControl *iimpR;
    IControlLimits    *ilimR;
    Vector            *encsR;
    iCubArm           *armR;
    int jntsR;

    // "Classical" interfaces - TORSO
    IEncoders         *iencsT;
    IControlLimits    *ilimT;
    Vector            *encsT;
    int jntsT;

    IGazeControl    *igaze;

    iCubFinger *fingerL;
    iCubFinger *fingerR;

    Vector ft_avg;

    int movementCnt;

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

public:
    // CONSTRUCTOR
    tactileServoThread(int _rate, const string &_name, const string &_robot, int _v);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
protected:
    //my variable definition
    Vector ft_vector,ft_vector_old,ft_vector_filter;
    Matrix initPose, currPose;
    Vector position_0_g,orientation_0_g;  
    Vector DesVec;
    Vector Desf, Dest;
    Matrix PCtrl, OrienCtrl;
    bool getFT();
    void getCurPose();
    Vector getnewposition(Vector v);

    Vector getNewDesVec(const string &arm="left_arm");
    bool calibrateFTReadings();
    void updateChains();
};

#endif

