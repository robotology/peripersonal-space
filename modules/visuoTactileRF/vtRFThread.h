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
#include <set>
#include <list>

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
    Port imagePortOutR;                              // port for streaming images
    Port imagePortOutL;                              // port for streaming images
    ImageOf<PixelRgb> *imageInR;
    ImageOf<PixelRgb> *imageInL;

    BufferedPort<Bottle> *dTPort;           // input from the doubleTouch
    Bottle               *dTBottle;

    BufferedPort<Bottle>   *eventsPort;     // input from the visuoTactileWrapper
    Bottle                 *event;
    vector <IncomingEvent>  incomingEvents;

    Port skinGuiPortForearmL;     // output to the skinGui
    Port skinGuiPortForearmR;    
    Port skinGuiPortHandL;     
    Port skinGuiPortHandR;    

    BufferedPort<iCub::skinDynLib::skinContactList> *skinPortIn;  // input from the skinManager
    Port skinPortOut;                                             // output for the events
    Port dataDumperPortOut;                                       // output for the dataDumper (quick thing
    yarp::sig::Vector dumpedVector;
    
    // iCubSkin
    vector <string>   filenames;
    vector <skinPart1D> iCubSkin1D;
    vector <skinPart2D> iCubSkin2D;
    int iCubSkinSize;

    vector <IncomingEvent> eventsBuffer; // the events to be buffered for the learning

    string path;            // path the module will save taxels in
    string taxelsFile;      // file to save taxels in
    string modality;        // modality to use (either 1D or 2D)

    // H - it's not a proper skinPart, but it works :)
    // It's used to project:
    //   H0  -> the taxel that has to be touched
    //   HN  -> the finger that has to touch
    //   EER -> the right end-effector (the wrist)
    //   EEL -> the left  end-effector (the wrist)
    // vector <skinPart> H;

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

    // Stamp for the setEnvelope for the ports
    yarp::os::Stamp ts;

    /**
    * Locates a taxel in the World Reference Frame, given its
    * position w.r.t. the arm
    * @param _pos is the taxel position wrt the arm
    * @param part is the chain the taxel is attached to. It can be
    *             either left_forearm, left_hand, right_forearm, or right_hand
    * @return the position of the taxel in the WRF
    **/
    yarp::sig::Vector locateTaxel(const yarp::sig::Vector &_pos,
                                  const string &part);

    /**
    * Projects all the taxels belonging to a skinPart from their
    * position in the WRF into the image plane of an eye
    * @param sP   is the skinPart to project
    * @param _eye is the eye to project the skinPart into. It can be
    *             either rightEye, or leftEye
    * @return true/false for success/failure
    **/
    bool projectIntoImagePlane(vector <skinPart> &sP, const string &_eye);
    
    /**
    * Projects a generic 3D point from its position in the WRF
    * into the image plane of an eye
    * @param x    is the 3D point (x,y,z) in the WRF
    * @param px   is the 2D point (u,v) in the image plane
    * @param _eye is the eye to project the point into. It can be
    *             either rightEye, or leftEye
    * @return true/false for success/failure
    **/
    bool projectPoint(const yarp::sig::Vector &x,
                      yarp::sig::Vector &px, const string &_eye);

    /**
    * Handles the images, by drawing taxels in either the left or right eye.
    * @param _eye is the eye to draw the taxels into. It can be
    *             either rightEye, or leftEye.
    **/
    void drawTaxels(string _eye);

    /**
    * Physically draws a taxel into an image, given its position and its activation.
    * Color changes according to the body part involved as well as the activation.
    * @param Im   is the image to draw the taxel into
    * @param px   is the vector of 2D coordinates (u,v) of the point to be drawed
    * @param part is the chain the taxel is attached to. It can be
    *             either left_forearm, left_hand, right_forearm, or right_hand
    * @param act  is the activation level of the taxel
    **/
    void drawTaxel(ImageOf<PixelRgb> &Im, const yarp::sig::Vector &px,
                   const string &part, const int act);

    /**
    * Finds out the positions of the taxels w.r.t. their respective limbs
    **/
    bool setTaxelPosesFromFile1D(const string filePath,skinPart1D &spw);
    bool setTaxelPosesFromFile2D(const string filePath,skinPart2D &spw);

    /**
    * If it is defined for the respective skin part, it fills the taxelIDtoRepresentativeTaxelID vector that is indexed by taxel IDs
    * and returns taxel IDs of their representatives - e.g. triangle centers.
    **/
    void initRepresentativeTaxels(skinPart2D &sP);
   
    
    /**
    * For all active taxels, it returns a set of "representative" active taxels if they were defined for the respective skin part 
    * E.g. if at least one taxel from a triangle was active, the center of this triangle will be part of the output list 
    * @param IDv  is a vector of IDs of the taxels activated
    * @param IDx  is the index of the iCubSkin affected by the contact
                  (basically, the index of the skinPart that has been touched)
    * @param v    is a vector of IDs of the representative taxels activated
    **/
    bool getRepresentativeTaxelsToTrain(const std::vector<unsigned int> IDv, const int IDx, std::vector<unsigned int> &v);
        
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
    * Detects a contact onto the skin:
    * @param _sCL is the skinContactList from which sorting out the events
    *             (usually provided by the skinManager)
    * @param _IDx is the index of the iCubSkin affected by the contact
                  (basically, the index of the skinPart that has been touched)
    * @param _IDv is a vector of IDs of the taxels activated
    **/
    bool detectContact(iCub::skinDynLib::skinContactList *_sCL, int &_IDx,
                       std::vector <unsigned int> &_IDv);

    /**
    *
    **/
    IncomingEvent4Taxel2D projectIntoTaxelRF(const Matrix &RF,const Matrix &T_a,const IncomingEvent &e);

    /**
    *
    **/
    bool computeResponse();

    /**
    * Compute the NRM and TTC for a specific event
    **/
    bool computeX(IncomingEvent4Taxel2D &ie);

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    **/
    int printMessage(const int l, const char *f, ...) const;

public:
    // CONSTRUCTOR
    vtRFThread(int _rate, const string &_name, const string &_robot, const string &_modality,
               int _v, const ResourceFinder &_moduleRF, vector<string> _fnames,
               double _hV, const ResourceFinder &_eyeCalibRF);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    /**
    * Pushes extrinsic parameters to either the right or the left eye
    * @param M   is the transform matrix to push
    * @param eye is the eye to push the matrix into
    **/
    bool pushExtrinsics(const Matrix &M, string eye);

    /**
    * Trains the taxels according to the incoming event.
    * @param IDv is the transform matrix to push
    * @param IDx is the eye to push the matrix into
    **/
    bool trainTaxels(const std::vector<unsigned int> IDv, const int IDx);

    /**
    * Saving function. It saves the skinParts as well as their receptive fields.
    **/
    bool save();

    /**
    * Loading function. It saves the skinParts as well as their receptive fields.
    **/
    bool load();

    /**
    * Resets the learning
    **/
    void resetParzenWindows();

    /**
    * Stops the learning
    **/
    bool stopLearning();

    /**
    * Restores the learning
    **/
    bool restoreLearning();
};

#endif

// empty line to make gcc happy