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
/**
\defgroup virtualContactGenerationModule virtualContactGenerationModule

@ingroup periPersonalSpace

Generates fake skinContact events (aka skinEvents) that can be streamed to a port. 

Date first release: 19/03/2015

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This module generates fake skinContact events (aka skinEvents) that can be streamed to a port. Taxels from different skin parts can be chosen at random. 

\section lib_sec Libraries 
YARP, ICUB libraries 

\section parameters_sec Parameters

--context       \e path
- Where to find the called resource.

--from          \e from
- The name of the .ini file with the configuration parameters.

--name          \e name
- The name of the module (default virtualContactGeneration).

--robot         \e rob
- The name of the robot (either "icub" or "icub"). Default icubSim.

--rate          \e rate
- The period used by the thread. Default 100ms, i.e. 10 Hz.

--verbosity  \e verb
- Verbosity level (default 0). The higher is the verbosity, the more
  information is printed out.

--type \e type
- Type of selection of contacts - e.g. randon.


\section portsc_sec Ports Created
- <i> /<name>/contacts:i </i> it sends out the virtual skinContacts created.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 12.04).

\author: Matej Hoffmann
*/ 

#include <yarp/os/all.h>
#include <string> 
#include <vector>

#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>

#include "virtContactGenThread.h"

using namespace yarp;
using namespace yarp::os;
using namespace std;

/**
* \ingroup virtualContactGenerationModule
*
* The module that achieves the fake skinContact events generation.
*  
*/
class virtualContactGeneration: public RFModule 
{
private:
    virtContactGenThread *virtContactGenThrd;
  
    string robot;
    string name;
    int verbosity;
    int threadPeriod;
    string type;
    
    vector<iCub::skinDynLib::SkinPart> activeSkinParts;
 

public:
    virtualContactGeneration()
    {
        virtContactGenThrd = 0;
    }

    bool configure(ResourceFinder &rf)
    {
        
        name = "virtualContactGeneration";
        robot =  "icubSim";
        threadPeriod = 100; //period of the virtContactGenThread in ms
        verbosity = 0;
        type = "random"; //selection of the 

        //******************************************************
        //********************** CONFIGS ***********************

        //******************* GENERAL GROUP ******************
        Bottle &bGeneral=rf.findGroup("general");
        bGeneral.setMonitor(rf.getMonitor());  
        
           //******************* NAME ******************
          if (bGeneral.check("name"))
            {
                name = bGeneral.find("name").asString();
                yInfo("Module name set to %s", name.c_str());
            }
            else yInfo("Module name set to default, i.e. %s", name.c_str());
            setName(name.c_str());

            //******************* ROBOT ******************
            if (bGeneral.check("robot"))
            {
                robot = bGeneral.find("robot").asString();
                yInfo("Robot is: %s", robot.c_str());
            }
            else yInfo("Could not find robot option in the config file; using %s as default",robot.c_str());

            //****************** rate ******************
            if (bGeneral.check("rate"))
            {
                rate = bGeneral.find("rate").asInt();
                yInfo("virtContactGenThread rateThread working at %i ms.",threadPeriod);
            }
            else yInfo("Could not find rate in the config file; using %i ms as default period",threadPeriod);
           
            //******************* VERBOSE ******************
            if (bGeneral.check("verbosity"))
            {
                verbosity = bGeneral.find("verbosity").asInt();
                yInfo("verbosity set to %i", verbosity);
            }
            else yInfo("Could not find verbosity option in the config file; using %i as default",verbosity);  
            
            //******************* TYPE ******************
            if (bGeneral.check("type"))
            {
                type = bGeneral.find("type").asString();
                yInfo("Type is: %s", type.c_str());
            }
            else yInfo("Could not find type option in the config file; using %s as default",type.c_str());

          //*************** ACTIVE SKIN PARTS GROUP ****************
          Bottle &bSkinParts=rf.findGroup("skin_parts");
          bSkinParts.setMonitor(rf.getMonitor());

       
        

        return true;
    }

    bool close()
    {
        cout << "ULTIMATE TRACKER: Stopping threads.." << endl;
        if (utMngrThrd)
        {
            utMngrThrd -> stop();
            delete utMngrThrd;
            utMngrThrd =  0;
        }

        if (kalThrd)
        {
            kalThrd -> stop();
            delete kalThrd;
            kalThrd =  0;
        }
        return true;
    }

    double getPeriod()
    {
        return 0.05;
    }

    bool updateModule()
    {
        return true;
    }
};