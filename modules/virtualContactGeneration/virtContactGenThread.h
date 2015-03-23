/*
 * Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Matej Hoffmann
 * email:  matej.hoffmann@iit.it
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
*/

#ifndef __VIRTCONTACTGENTHREAD_H__
#define __VIRTCONTACTGENTHREAD_H__

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <stdarg.h>
#include <string> 
#include <iostream>
#include <fstream>
#include <sstream>

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>

#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <iCub/periPersonalSpace/utils.h>

using namespace std;

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

using namespace iCub::skinDynLib;

class virtContactGenerationThread: public RateThread
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
    // Resource finder used to find for files and configurations:
    ResourceFinder* rf;
    //the period used by the thread. 
    int threadPeriod; 
    // type of selection of contacts - e.g. random
    string type;
    
    
    //based on .ini file, contains a list of skin parts that will be part of the virtual contact generation
    vector<SkinPart> activeSkinPartsNames;
    map<SkinPart,string> skinPartPosFilePaths;
    //will contain actual skin parts with list of taxels and their positions
    map<SkinPart,skinPartTaxel> activeSkinParts;
    
    /***************************************************************************/
    // INTERNAL VARIABLES
    // Port with the fake contacts:
    BufferedPort<skinContactList> *skinEventsOutPort;
        
    SkinPart skinPartPicked;
  
    /**
    * Initializes vector of skin parts - activeSkinParts based on the activeSkinPartsNames. 
    * Uses the class members (activeSkinPartsNames and activeSkinParts) and modifies activeSkinParts - fills it with appropriate taxel objects etc.
    */
    int initSkinParts();
       
    
    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

    
public:
    // CONSTRUCTOR
    virtContactGenerationThread(int _rate, const string &_name, const string &_robot,
                      int _v, const string &_type, const vector<SkinPart> &_activeSkinPartsNames, const map<SkinPart,string> &_skinPartPosFilePaths);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

};

#endif
