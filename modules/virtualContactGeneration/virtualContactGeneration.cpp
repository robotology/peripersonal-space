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
- Type of selection of contacts - e.g. random.


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

using namespace iCub::skinDynLib;

/**
* \ingroup virtualContactGenerationModule
*
* The module that achieves the fake skinContact events generation.
*  
*/
class virtualContactGeneration: public RFModule 
{
private:
    virtContactGenerationThread *virtContactGenThrd;
  
    string robot;
    string name;
    int verbosity;
    int threadPeriod;
    string type;    
       
    vector<SkinPart> activeSkinPartsVector;
 

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
        
        SkinPart partOfSkin;
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
                threadPeriod = bGeneral.find("rate").asInt();
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

           if (bSkinParts.check("SKIN_LEFT_HAND"))
           {
                if(bSkinParts.find("SKIN_LEFT_HAND").asString()=="on"){
                    partOfSkin = SKIN_LEFT_HAND;
                    activeSkinPartsVector.push_back(partOfSkin);
                    yInfo("Adding SKIN_LEFT_HAND to active skin parts.");
                }
               
           }
           else yInfo("SKIN_LEFT_HAND set to default, i.e. off");
           
           if (bSkinParts.check("SKIN_LEFT_FOREARM"))
           {
                if(bSkinParts.find("SKIN_LEFT_FOREARM").asString()=="on"){
                    partOfSkin = SKIN_LEFT_FOREARM;
                    activeSkinPartsVector.push_back(partOfSkin);
                    yInfo("Adding SKIN_LEFT_FOREARM to active skin parts.");
                }
               
           }
           else yInfo("SKIN_LEFT_FOREARM set to default, i.e. off"); 
            
           if (bSkinParts.check("SKIN_LEFT_UPPER_ARM"))
           {
                if(bSkinParts.find("SKIN_LEFT_UPPER_ARM").asString()=="on"){
                    partOfSkin = SKIN_LEFT_UPPER_ARM;
                    activeSkinPartsVector.push_back(partOfSkin);
                    yInfo("Adding SKIN_LEFT_UPPER_ARM to active skin parts.");
                }
               
           }
           else yInfo("SKIN_LEFT_UPPER_ARM set to default, i.e. off"); 
           
           if (bSkinParts.check("SKIN_RIGHT_HAND"))
           {
                if(bSkinParts.find("SKIN_RIGHT_HAND").asString()=="on"){
                    partOfSkin = SKIN_RIGHT_HAND;
                    activeSkinPartsVector.push_back(partOfSkin);
                    yInfo("Adding SKIN_RIGHT_HAND to active skin parts.");
                }
               
           }
           else yInfo("SKIN_RIGHT_HAND set to default, i.e. off");
           
           if (bSkinParts.check("SKIN_RIGHT_FOREARM"))
           {
                if(bSkinParts.find("SKIN_RIGHT_FOREARM").asString()=="on"){
                    partOfSkin = SKIN_RIGHT_FOREARM;
                    activeSkinPartsVector.push_back(partOfSkin);
                    yInfo("Adding SKIN_RIGHT_FOREARM to active skin parts.");
                }
               
           }
           else yInfo("SKIN_RIGHT_FOREARM set to default, i.e. off"); 
            
           if (bSkinParts.check("SKIN_RIGHT_UPPER_ARM"))
           {
                if(bSkinParts.find("SKIN_RIGHT_UPPER_ARM").asString()=="on"){
                    partOfSkin = SKIN_RIGHT_UPPER_ARM;
                    activeSkinPartsVector.push_back(partOfSkin);
                    yInfo("Adding SKIN_RIGHT_UPPER_ARM to active skin parts.");
                }
               
           }
           else yInfo("SKIN_RIGHT_UPPER_ARM set to default, i.e. off"); 
           
           
        //******************************************************
        //*********************** THREAD **********************
        virtContactGenThrd = new virtContactGenerationThread(threadPeriod,name,robot,verbosity,type,activeSkinPartsVector);
        if (!virtContactGenThrd -> start())
        {
              delete virtContactGenThrd;
              virtContactGenThrd = 0;
              yError("virtContactGenThrd wasn't instantiated!!");
                    return false;
        }
        yInfo("virtContactGenThrd instantiated...");
        return true;
    }

    bool close()
    {
        cout << "virtContactGeneration: Stopping thread.." << endl;
        if (virtContactGenThrd)
        {
            virtContactGenThrd -> stop();
            delete virtContactGenThrd;
            virtContactGenThrd =  0;
        }
      
        return true;
    }

    double getPeriod()
    {
        return 1.0;
    }

    bool updateModule()
    {
        return true;
    }
};


/**
* Main function.
*/
int main(int argc, char * argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("periPersonalSpace");
    rf.setDefaultConfigFile("virtualContactGeneration.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {   
        yInfo(""); 
        yInfo("Options:");
        yInfo("");
        yInfo("   --context    path:  where to find the called resource");
        yInfo("   --from       from:  the name of the .ini file.");
        yInfo("   --name       name:  the name of the module (default virtualContactGeneration).");
        yInfo("   --robot      robot: the name of the robot. Default icubSim.");
        yInfo("   --rate       rate:  the period used by the thread. Default 100ms.");
        yInfo("   --verbosity  int:   verbosity level (default 0).");
        yInfo("   --type       type:  selection of fake contacts - e.g. 'random'.");
     
        yInfo("");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    virtualContactGeneration vCG;
    return vCG.runModule(rf);
}
// empty line to make gcc happy