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

#ifndef __FGTTRACKERTHREAD_H__
#define __FGTTRACKERTHREAD_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/Drivers.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <iCub/periPersonalSpace/utils.h>


using namespace yarp;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;

using namespace std;

class fgtThread: public RateThread
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
    int    stateFlag;
    double timeNow;

    BufferedPort<ImageOf<PixelRgb> > imagePortInR;   // port for reading images
    BufferedPort<ImageOf<PixelRgb> > imagePortInL;   // port for reading images
    BufferedPort<ImageOf<PixelRgb> > imagePortOutR;  // port for streaming images
    BufferedPort<ImageOf<PixelRgb> > imagePortOutL;  // port for streaming images
    ImageOf<PixelRgb> *imageInR;
    ImageOf<PixelRgb> *imageInL;
    ImageOf<PixelRgb> imageOutR;
    ImageOf<PixelRgb> imageOutL;

    BufferedPort<Bottle>  doubleTouchPort;           // input from the doubleTouch
    Bottle               *doubleTouchBottle;
    int doubleTouchStep;                             // the step the doubleTouch is in

    BufferedPort<Bottle>  outPort;                   // output the doubleTouch

    Vector fingerL;
    Vector fingerR;
    Vector fingertip;

    Vector HSVmin;
    Vector HSVmax;

    // Gaze Controller
    PolyDriver       ddG; // gaze  controller  driver
    IGazeControl  *igaze;
    int contextGaze;

    bool processImages(ImageOf<PixelRgb> &_oL, ImageOf<PixelRgb> &_oR);
    bool sendImages();
    bool get3DPoint();

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    **/
    int printMessage(const int l, const char *f, ...) const;
    
public:
    // CONSTRUCTOR
    fgtThread(int _rate, const string &_name, const string &_robot, int _v, const Vector &_hsvmin, const Vector &_hsvmax);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    void setHMin(int _val);
    void setHMax(int _val);
};

#endif

// empty line to make gcc happy
