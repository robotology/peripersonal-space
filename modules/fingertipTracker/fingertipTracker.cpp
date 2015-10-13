/* FINGERTIP TRACKER v. 1.0
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
\defgroup fingertipTrackerModule fingertipTrackerModule

@ingroup periPersonalSpace

THE fingertip tracker. It tracks the green fingertip placed on the right hand during a doubleTouch session.

Date first release: 23/05/2014

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
THE fingertip tracker. It tracks the green fingertip placed on the right hand during a doubleTouch session.

\section lib_sec Libraries 
YARP, ICUB libraries and OPENCV

\section parameters_sec Parameters

--context       \e path
- Where to find the called resource.

--from          \e from
- The name of the .ini file with the configuration parameters.

--name          \e name
- The name of the module (default fingertipTracker).

--robot         \e rob
- The name of the robot (either "icub" or "icub"). Default icub.

--rate          \e rate
- The period used by the thread. Default 100ms.

\section portsc_sec Ports Created

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 12.04, Debian Squeeze, Debian Wheezy).

\author: Alessandro Roncone and Ugo Pattacini
*/ 

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Log.h>
#include <string> 

#include "fgtThread.h"


using namespace yarp;
using namespace yarp::os;
using namespace std;

/**
* \ingroup fingertipTrackerModule
*
* The module that achieves the fingertipTracker task.
*  
*/
class fingertipTracker: public RFModule 
{
private:
    fgtThread *fgtThrd;

    // RpcClient  rpcClnt;
    RpcServer  rpcSrvr;

    string robot;
    string name;
    int verbosity;
    int rate;

public:
    fingertipTracker()
    {
        fgtThrd    = 0;
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack =Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                //-----------------
                case VOCAB3('s','e','t'):
                {
                    reply.addString(command.get(1).asString());

                    if (command.get(1).asString() == "hmin")
                    {
                        fgtThrd -> setHMin(command.get(2).asInt());
                        reply.addVocab(ack);
                    }
                    else if (command.get(1).asString() == "hmax")
                    {
                        fgtThrd -> setHMax(command.get(2).asInt());
                        reply.addVocab(ack);
                    }
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
        name  = "fingertipTracker";
        robot = "icub";

        verbosity   =   0;   // verbosity
        rate        =  20;   // rate of the thread [ms]

        //******************************************************
        //********************** CONFIGS ***********************

        //******************* NAME ******************
            if (rf.check("name"))
            {
                name = rf.find("name").asString();
                yInfo("Module name set to %s",name.c_str());
            }
            else yInfo("Module name set to default, i.e.  %s",name.c_str());
            setName(name.c_str());

        //******************* ROBOT ******************
            if (rf.check("robot"))
            {
                robot = rf.find("robot").asString();
                yInfo("Robot is:  %s",robot.c_str());
            }
            else yInfo("Could not find robot option in the config file; using %s as default.",robot.c_str());

        //******************* VERBOSE ******************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                yInfo("fgtThread verbosity set to  %i",verbosity);
            }
            else yInfo("Could not find verbosity option in config file; using %i as default.",verbosity);

        //****************** rate ******************
            if (rf.check("rate"))
            {
                rate = rf.find("rate").asInt();
                yInfo("fgtThread rateThread working at %i ms.",rate);
            }
            else yInfo("Could not find rate in the config file; using %i ms as default.",rate);

        //******************************************************
        //*********************** THREADS **********************
            Vector HSVmin(3,0.0);
            Vector HSVmax(3,0.0);

            HSVmin[0] = 40.0;
            HSVmin[1] = 50.0;
            HSVmin[2] = 150.0;

            HSVmax[0] = 80.0;
            HSVmax[1] = 255.0;
            HSVmax[2] = 255.0;


            fgtThrd = new fgtThread(rate, name, robot, verbosity, HSVmin, HSVmax);
            if (!fgtThrd -> start())
            {
                delete fgtThrd;
                fgtThrd = 0;
                yError(" fgtThread wasn't instantiated!!");
                return false;
            }
            yInfo("FINGERTIP TRACKER: fgtThread istantiated...");

        //******************************************************
        //************************ PORTS ***********************
            // rpcClnt.open(("/"+name+"/rpc:o").c_str());
            rpcSrvr.open(("/"+name+"/rpc:i").c_str());
            attach(rpcSrvr);

        return true;
    }

    bool close()
    {
        yInfo("FINGERTIP TRACKER: Stopping threads..");
        if (fgtThrd)
        {
            fgtThrd -> stop();
            delete fgtThrd;
            fgtThrd =  0;
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

/**
* Main function.
*/
int main(int argc, char * argv[])
{
    Network yarp;

    
    ResourceFinder moduleRF;
    moduleRF.setVerbose(false);
    moduleRF.setDefaultContext("periPersonalSpace");
    moduleRF.setDefaultConfigFile("fingertipTracker.ini");
    moduleRF.configure(argc,argv);

    if (moduleRF.check("help"))
    {    
        yInfo("");
        yInfo("Options:");
        yInfo("");
        yInfo("   --context     path:  where to find the called resource (default periPersonalSpace).");
        yInfo("   --from        from:  the name of the .ini file (default fingertipTracker.ini).");
        yInfo("   --name        name:  the name of the module (default fingertipTracker).");
        yInfo("   --robot       robot: the name of the robot. Default icub.");
        yInfo("   --verbosity   int:   verbosity level (default 0).");
        yInfo("   --rate        int:   the period used by the thread. Default 20ms.");
        yInfo("");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        yError("No Network!!!");
        return -1;
    }

    fingertipTracker fgtTrack;
    return fgtTrack.runModule(moduleRF);
}

// empty line to make gcc happy
