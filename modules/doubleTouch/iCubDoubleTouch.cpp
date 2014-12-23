/* DOUBLE TOUCH v. 0.2
 * Copyright (C) 2013 RobotCub Consortium
 * Author:  Alessandro Roncone
 * email:   alessandro.roncone@iit.it
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
*/

/**
\defgroup doubleTouchModule doubleTouchModule

@ingroup periPersonalSpace

A module able to accomplish the double touch task.

Date first release: 30/06/2013

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is a module for implementing the Double Touch on the iCub.

\section lib_sec Libraries 
YARP, ICUB libraries and IPOPT

\section parameters_sec Parameters

--context    \e path
- Where to find the called resource.

--from       \e from
- The name of the .ini file with the configuration parameters.

--name       \e name
- The name of the module (default doubleTouch).

--robot      \e rob
- The name of the robot (either "icub" or "icubSim"). Default icubSim.

--rate       \e rate
- The period used by the thread. Default 100ms.

--verbosity  \e verb
- Verbosity level (default 0). The higher is the verbosity, the more
  information is printed out.

--record     \e rec
- If to record data or not. The double touch slightly differs according to this
  parameter.

--color      \e col
- Robot color (black or white or whatever) - is MANDATORY if a recording session
  with rec==2 is started

--type       \e type
- Type of the task (right now it can be either "LtoR" or "RtoL" or "both")

--filename   \e file
- The name of the file to be saved in case of a recording session.
  Default 'calibration.txt'. A date is appended at the beginning for completeness.

\section portsc_sec Ports Created
- <i> /<name>/contacts:i </i> it reads the skinContacts from the skinManager.

- <i> /<name>/records:o </i> it prints out some data about the state of the
  recording.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 12.04, Debian Squeeze).

\author: Alessandro Roncone
*/ 

#include <yarp/os/all.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string.h> 
#include <ctime>
#include <sstream>

#include "iCubDblTchThrd.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/**
* \ingroup doubleTouchModule
*
* The module that achieves the doubleTouch task.
*  
*/
class doubleTouch: public RFModule
{
private:
    doubleTouchThread *dblTchThrd;
    RpcClient             rpcClnt;
    RpcServer             rpcSrvr;

    string robot,name,type,filename,color;

    int verbosity,rate,record;

public:
    doubleTouch()
    {
        dblTchThrd=0;

        robot    = "icubSim";
        name     = "doubleTouch";
        type     = "LtoR";
        filename = ".txt";
        color    = "";

        verbosity = 0;      // verbosity
        rate      = 100;    // rate of the doubleTouchThread
        record    = 0;      // record data
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                //-----------------
                case VOCAB4('c','o','n','n'):
                {
                    Network yarpNetwork;
                    if (yarpNetwork.connect("/skinManager/skin_events:o",("/"+name+"/contacts:i").c_str()))
                        reply.addVocab(ack);
                    else
                        reply.addVocab(nack);
                    return true;
                }

                //-----------------
                case VOCAB4('s','t','a','r'):
                {
                    dblTchThrd = new doubleTouchThread(rate, name, robot, verbosity,
                                                       type, record, filename, color);
                    bool strt = dblTchThrd -> start();
                    if (!strt)
                    {
                        delete dblTchThrd;
                        dblTchThrd = 0;
                        yError("doubleTouchThread wasn't instantiated!!");
                        reply.addVocab(nack);
                    }
                    else
                        reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case VOCAB4('d','i','s','c'):
                {
                    Network yarpNetwork;
                    if (yarpNetwork.disconnect("/skinManager/skin_events:o",("/"+name+"/contacts:i").c_str()))
                        reply.addVocab(ack);
                    else
                        reply.addVocab(nack);
                    return true;
                }

                //-----------------
                case VOCAB4('s','t','o','p'):
                {
                    if (dblTchThrd)
                    {
                        yInfo("DOUBLE TOUCH: Stopping threads..");
                        dblTchThrd->stop();
                        delete dblTchThrd;
                        dblTchThrd=0;
                    }
                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    bool configure(ResourceFinder &rf)
    {
        //******************************************************
        //********************** CONFIGS ***********************
            bool alignEyes = rf.check("alignEyes");
        //******************* NAME ******************
            if (rf.check("name"))
            {
                name = rf.find("name").asString();
                yInfo("Module name set to %s", name.c_str());
            }
            else yInfo("Module name set to default, i.e. %s", name.c_str());
            setName(name.c_str());

        //******************* ROBOT ******************
            if (rf.check("robot"))
            {
                robot = rf.find("robot").asString();
                yInfo("Robot is: %s", robot.c_str());
            }
            else yInfo("Could not find robot option in the config file; using %s as default",robot.c_str());

        //******************* TYPE ******************
            if (rf.check("type"))
            {
                type = rf.find("type").asString();
                yInfo("Type is: %s", type.c_str());
            }
            else yInfo("Could not find type option in the config file; using %s as default",type.c_str());

        //******************* VERBOSE ******************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                yInfo("doubleTouchThread verbosity set to %i", verbosity);
            }
            else yInfo("Could not find verbosity option in the config file; using %i as default",verbosity);

        //****************** rate ******************
            if (rf.check("rate"))
            {
                rate = rf.find("rate").asInt();
                yInfo("doubleTouchThread rateThread working at %i ms.",rate);
            }
            else yInfo("Could not find rate in the config file; using %i as default",rate);

        //****************** record ******************
            if (rf.check("record"))
            {
                record = rf.find("record").asInt();
                yInfo("doubleTouchThread record variable is set to %i",record);
            }
            else yInfo("Could not find record in the config file; using %i as default",record);

        //***************** Filename *****************
            if (rf.check("filename")) {
                filename = rf.find("filename").asString();
                yInfo("Module filename set to %s", filename.c_str());
            }
            else yInfo("Module filename set to default, i.e. %s", filename.c_str());

        //***************** color *****************
            if (rf.check("color")) {
                color = rf.find("color").asString();
                yInfo("Robot color set to %s", color.c_str());
            }
            else 
            {
                if (record==2)
                {
                    yError("Robot color wasn't instantiated, and this should be a recording session for calibration purposes!.");
                    yInfo("I will finish here.");
                    return false;
                }
            }

        // Let's add some contextual info (the date) to the file created!
            time_t now = time(0);
            tm *ltm = localtime(&now);
            string time = int_to_string(1900 + ltm->tm_year)+"_"+int_to_string(1+ltm->tm_mon)+"_"+
                          int_to_string(ltm->tm_mday)+"_"+int_to_string(1+ltm->tm_hour)+"_"+
                          int_to_string(1+ltm->tm_min)+"_";
            if (record==2)
            {
                filename = "../calibration_data/"+time+filename;
            }
            else if (record==1)
            {
                filename = "../vRFlearning_data/"+time+filename;
            }
            else
            {
                filename = "../data/"+time+filename;
            }
            yInfo("Storing file set to: %s",filename.c_str());

        //******************************************************
        //************************ PORTS ***********************
        if (alignEyes)
        {
            rpcClnt.open(("/"+name+"/rpc:o").c_str());
            rpcSrvr.open(("/"+name+"/rpc:i").c_str());
            attach(rpcSrvr);
        }
        else
        {
            dblTchThrd = new doubleTouchThread(rate, name, robot, verbosity,
                                               type, record, filename, color);
            bool strt = dblTchThrd -> start();
            if (!strt)
            {
                delete dblTchThrd;
                dblTchThrd = 0;
                yInfo("ERROR!!! doubleTouchThread wasn't instantiated!!");
                return false;
            }
        }

        return true;
    }

    bool close()
    {
        yInfo("DOUBLE TOUCH: Stopping threads..");
        if (dblTchThrd)
        {
            dblTchThrd->stop();
            delete dblTchThrd;
            dblTchThrd=0;
        }

        return true;
    }

    double getPeriod()  { return 1.0; }
    bool updateModule() { return true; }
};

/**
* Main function.
*/
int main(int argc, char * argv[])
{
    Network yarp;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("periPersonalSpace");
    rf.setDefaultConfigFile("doubleTouchDemo.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {   
        yInfo(""); 
        yInfo("Options:");
        yInfo("");
        yInfo("   --context    path:  where to find the called resource");
        yInfo("   --from       from:  the name of the .ini file.");
        yInfo("   --name       name:  the name of the module (default doubleTouch).");
        yInfo("   --robot      robot: the name of the robot. Default icubSim.");
        yInfo("   --rate       rate:  the period used by the thread. Default 100ms.");
        yInfo("   --verbosity  int:   verbosity level (default 0).");
        yInfo("   --record     int:   if to record data or not.");
        yInfo("       record==0 -> nothing is recorded, the double touch is iterating over and");
        yInfo("                    over again. Demonstrative and testing purposes.");
        yInfo("       record==1 -> recording for visuo-tactile reference frames purposes.");
        yInfo("       record==2 -> recording for kinematic calibration purposes.");
        yInfo("   --color      color: robot color (black or white - MANDATORY!)");
        yInfo("   --type       type:  the type of task (default 'LtoR'). Allowed type names:");
        yInfo("   --type              'RtoL','LtoR','RHtoL','LHtoR','both_5DOF','both_7DOF','both_LtoR','both_RtoL'");
        yInfo("   --filename   file:  the name of the file to be saved in case of");
        yInfo("                       a recording session. Default 'calibration.txt'.");
        yInfo("                       A date is appended at the beginning for completeness.");
        yInfo("   --alignEyes  flag:  if or not to use the rpc-thing and sync with alignEyes module.");
        yInfo("");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    doubleTouch dblTch;
    return dblTch.runModule(rf);
}
// empty line to make gcc happy
