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

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <gsl/gsl_math.h>

#include <iCub/iKin/iKinFwd.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <vector>

#include <cv.h>
#include <highgui.h>
#include <iCub/ctrl/math.h>
#include <iCub/periPersonalSpace/utils.h>
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::iKin;
using namespace iCub::ctrl;

using namespace std;

class vtRFThread: public RateThread
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
    // Name of the robot (to address the module toward icub or icubSim):
    ResourceFinder* rf;

    /***************************************************************************/
    // INTERNAL VARIABLES:
    BufferedPort<ImageOf<PixelRgb> > *imagePortInR;  // port for reading images
    BufferedPort<ImageOf<PixelRgb> > *imagePortInL;  // port for reading images
    BufferedPort<ImageOf<PixelRgb> > *imagePortOutR; // port for streaming images
    BufferedPort<ImageOf<PixelRgb> > *imagePortOutL; // port for streaming images
    ImageOf<PixelRgb> *imageInR;
    ImageOf<PixelRgb> *imageInL;

    BufferedPort<Bottle> *dTPort;           // input from the doubleTouch
    Bottle               *dTBottle;

    BufferedPort<Bottle>   *eventsPort;     // input from the visuoTactileWrapper
    Bottle                 *event;
    vector <IncomingEvent>  incomingEvents;

    BufferedPort<Vector> *skinGuiPortForearmL;     // output to the skinGui
    BufferedPort<Vector> *skinGuiPortForearmR;    
    BufferedPort<Vector> *skinGuiPortHandL;     
    BufferedPort<Vector> *skinGuiPortHandR;    
    Vector               *skinGuiVecL;

    BufferedPort<iCub::skinDynLib::skinContactList> *skinPortIn;  // input from the skinManager
    BufferedPort<Bottle> *skinPortOut; // input for the events
    
    // iCubSkin
    vector <string>   filenames;
    vector <skinPart> iCubSkin;
    vector <skinPart> iCubSkinOld;

    vector <IncomingEvent> eventsBuffer; // the events to be buffered for the learning

    string path;            // path the module will save taxels in
    string taxelsFile;      // file to save taxels in

    // H - it's not a proper skinPart, but it works :)
    // It's used to project:
    //   H0  -> the taxel that has to be touched
    //   HN  -> the finger that has to touch
    //   EER -> the right end-effector (the wrist)
    //   EEL -> the left  end-effector (the wrist)
    vector <skinPart> H;

    // EYEWRAPPER
    eyeWrapper      *eWR;
    eyeWrapper      *eWL;

    string      currentTask;

    bool eventsFlag;
    bool learningFlag;

    double timeNow;

    // Driver for "classical" interfaces
    PolyDriver       ddR; // right arm device driver
    PolyDriver       ddL; // left arm  device driver
    PolyDriver       ddT; // torso controller  driver
    PolyDriver       ddH; // head  controller  driver
    PolyDriver       ddG; // gaze  controller  driver

    // "Classical" interfaces - RIGHT ARM
    IEncoders         *iencsR;
    yarp::sig::Vector *encsR;
    iCubArm           *armR;
    int                jntsR;
    // "Classical" interfaces - LEFT ARM
    IEncoders         *iencsL;
    yarp::sig::Vector *encsL;
    iCubArm           *armL;
    int                jntsL;
    // "Classical" interfaces - TORSO
    IEncoders         *iencsT;
    yarp::sig::Vector *encsT;
    int                jntsT;
    // "Classical" interfaces - HEAD
    IEncoders         *iencsH;
    yarp::sig::Vector *encsH;
    int                jntsH;

    // Gaze controller interface
    IGazeControl       *igaze;
    int contextGaze;

    /**
    * Locates a taxel in the World Reference Frame
    **/
    yarp::sig::Vector locateTaxel(const yarp::sig::Vector &_pos, const string &arm);

    /**
    * Project a point from its position in the WRF to the image plane of an eye
    **/
    bool projectIntoImagePlane(vector <skinPart> &spw, const string &eye, const bool flag);
    
    // if flag == 1, use the old projection
    bool projectPoint(const string &type, const yarp::sig::Vector &x,
                      yarp::sig::Vector &px, const bool flag);

    /**
    * 
    **/
    void drawTaxel(const yarp::sig::Vector &px, const string &bodypart,
                   ImageOf<PixelRgb> &Im, const int act, const bool flag);

    /**
    * 
    **/
    void handleImages(string eye);

    /**
    *
    **/
    bool setTaxelPosesFromFile(const string filePath,skinPart &spw);

    /**
    *
    **/
    bool projectIncomingEvent();

    /**
    *
    **/
    void manageSkinEvents();

    /**
    *
    **/
    void sendContactsToSkinGui();

    /**
    * detects a contact:
    * idx : index of the iCubSkin
    * v   : IDs of the taxels
    **/
    bool detectContact(iCub::skinDynLib::skinContactList *_sCL, int &idx,
                       std::vector <unsigned int> &v);

    /**
    *
    **/
    IncomingEvent projectIntoTaxelRF(const Matrix &RF,const Matrix &T_a,const IncomingEvent &e);
    Vector projectIntoTaxelRF(const Matrix &RF,const Matrix &T_a,const Vector &wrfpos);

    /**
    *
    **/
    bool computeResponse();

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    **/
    int printMessage(const int l, const char *f, ...) const;

public:
    // CONSTRUCTOR
    vtRFThread(int _rate, const string &_name, const string &_robot, int _v,
               const ResourceFinder &_moduleRF, vector<string> _fnames,
               double _hV, const ResourceFinder &_eyeCalibRF);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    /**
    * 
    *
    **/
    bool pushExtrinsics(const Matrix &M, string eye);

    /**
    *
    **/
    bool trainTaxels(const std::vector<unsigned int> IDv, const int IDx);

    /**
    *
    **/
    bool bufferizeEvents();

    /**
    *
    **/
    bool save();

    /**
    *
    **/
    bool load();


    /**
    *
    **/
    bool resetParzenWindows();

    /**
    *
    **/
    bool stopLearning();

    /**
    *
    **/
    bool restoreLearning();
};

#endif

// empty line to make gcc happy
