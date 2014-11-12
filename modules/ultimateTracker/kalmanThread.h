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

#ifndef __VTRFTHREAD_H__
#define __VTRFTHREAD_H__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <vector>
#include <math.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/kalman.h>
#include <iCub/periPersonalSpace/utils.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

using namespace std;

#define KALMAN_INIT     0
#define KALMAN_NORMAL   1
#define KALMAN_NOINPUT  2
#define KALMAN_NEWINPUT 3
#define KALMAN_STOPPED  4

class kalmanThread: public RateThread
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
    // If there's no data for this amount of time[ms], the filter gets stopped
    double noDataThres;

    /***************************************************************************/
    // INTERNAL VARIABLES:
    double timeNow;                     // time of the latest acquisition

    Vector kalIn;        // Input variable                
    Mutex  inputMutex;   // Mutex that handles the writing/reading of kalIn

    int    kalState;     // State of the thread
    Mutex  stateMutex;   // Mutex that handles the writing/reading of kalState

    Vector kalOut;       // Estimated kalman variable
    Mutex  outMutex;     // Mutex that handles the writing/reading of kalOut

    vector <iCub::ctrl::Kalman> posVelKalman;
    unsigned int kalOrder;      // The order of the kalman filters
    double       kalTs;         // The rate of the kalman filters
    double       kalThres;

    Matrix kalA;
    Matrix kalH;
    Matrix kalQ;
    Matrix kalR;
    Matrix kalP;

    bool kalmanPredict();

    bool kalmanUpdate();

    /**
    * Generates the proper kalman matrices, starting from the order
    **/
    bool generateMatrices();

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    **/
    int printMessage(const int l, const char *f, ...) const;
    
public:
    // CONSTRUCTOR
    kalmanThread(int _rate, const string &_name, const string &_robot, int _v, double _nDThr, unsigned int _kalOrder);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    /**
    * Initialize the kalman filter. This triggers the state to KALMAN_NORMAL.
    **/
    bool kalmanInit(const Vector &inVec);

    /**
    * Sets the new input measurement. This triggers the state to KALMAN_NEWINPUT.
    **/
    bool setKalmanInput(const Vector &inVec);

    /**
    * Gets the latest data. This triggers the state to become KALMAN_NOINPUT.
    **/
    bool getKalmanInput(Vector &inVec);

    /**
    * Aligns kalOut to the new state of the kalman filter.
    **/
    bool setKalmanOutput();

    /**
    * Sets the latest output from kalman filter.
    **/
    bool getKalmanOutput(Vector &inVec);

    /**
    * Gets the current state of the filter. This triggers the state to KALMAN_NEWDATA as well.
    **/
    bool setKalmanState(const int s);

    /**
    * Gets the current state of the filter.
    **/
    bool getKalmanState(int &s);
};

#endif

// empty line to make gcc happy
