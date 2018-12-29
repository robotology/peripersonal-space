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

#ifndef __ULTTRACKERTHREAD_H__
#define __ULTTRACKERTHREAD_H__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iostream>
#include <string>
#include <vector>

#include <iCub/periPersonalSpace/utils.h>
#include "kalmanThread.h"

using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

class utManagerThread: public PeriodicThread
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

    /***************************************************************************/
    // INTERNAL VARIABLES:
    kalmanThread          *kalThrd;     // Pointer to the kalmanThread in order to access its data.

    bool useDispBlobber;

    int    stateFlag;
    double timeNow;

    BufferedPort<Bottle>       *motionCUTBlobs;  // port for reading from motionCUT
    Bottle                     *motionCUTBottle; // bottle used for the port
    Vector                      motionCUTPos;    // current position of the center of the blob
    vector <yarp::sig::Vector>  oldMcutPoss;     // old positions

    BufferedPort<Bottle>  *dispBlobberTarget;   // port for reading from the dispBlobber
    Bottle                *dispBlobberBottle;   // bottle used for the port
    Vector                 dispBlobberPos;      // current position of the blob

    BufferedPort<Bottle>  *templatePFTrackerTarget;  // port for reading from templatePFTracker
    Bottle                *templatePFTrackerBottle;  // bottle used for the port
    Vector                 templatePFTrackerPos;     // current position of the center of the blob

    RpcClient SFMrpcPort;
    Vector    SFMPos;

    Port outPortGui;
    Port outPortEvents;

    Vector    kalOut;

    bool noInput();

    /**
    * Standard mode in which the templateTracker and the SFM are used.
    * @return int the state of the kalman filter
    **/
    int run_with_templateTracker_SFM();

    /**
     * New mode in which the 3D point is provided by the dispBlobber
     * @return int the state of the kalman filter
    **/
    int run_with_dispBlobber();

    /**
    * Processes data coming from the motionCUT.
    * @return true/false if a a stable moving object is detected
    **/
    bool processMotion();

    /**
    * Checks the stability of the motionCUT's blob center. 
    **/
    bool stabilityCheck();

    /**
    * Initializes the template tracker by giving a first snapshot for it to track it down
    **/
    bool initializeTracker();

    /**
    * Reads the output from the tracker, and retrieves the center of the tracked object into the image plane
    **/
    bool readFromTracker();

    /**
    * Converts the 2D tracked point into a 3D point thanks to the stereovision.
    * @return true/false for success/failure (if the stereoVision fails it returns false)
    **/
    bool getPointFromStereo();

    /**
    * Reads the output from the dispBlobber, and retrieves the 3D point of the center of the nearest blob
    * @return true/false for success/failure (if the stereoVision fails it returns false)
    **/
    bool getPointFromDispBlobber();

    /**
    * Handles the kalman filter, by inizializing it and feeding it with stereo 3D data.
    **/
    bool manageKalman();

    /**
    * Handles the iCubGui, by drawing the tracked object on the screen.
    **/
    void sendGuiTarget();

    /**
    * Deletes the object from the gui if it is not tracked any more.
    **/
    void deleteGuiTarget();

    /**
    * Sends out the 3D position of the tracked point. May be used by the gaze controller later.
    **/
    void sendData();

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    **/
    int printMessage(const int l, const char *f, ...) const;
    
public:
    // CONSTRUCTOR
    utManagerThread(int _rate, const string &_name, const string &_robot, int _v, kalmanThread *_kT, bool _useDispBlobber);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
};

#endif

// empty line to make gcc happy
