/* VISUO TACTILE RECEPTIVE FIELDS v. 1.0
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
\defgroup ultimateTrackerModule ultimateTrackerModule

@ingroup periPersonalSpace

THE ultimate tracker. It tracks everything without any hiccup!

Date first release: 23/05/2014

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is nothing but a tracker. But it's the ultimate one!

\section lib_sec Libraries 
YARP, ICUB libraries and OPENCV

\section parameters_sec Parameters

--context       \e path
- Where to find the called resource.

--from          \e from
- The name of the .ini file with the configuration parameters.

--name          \e name
- The name of the module (default ultimateTracker).

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

#include <yarp/os/all.h>
#include <string> 

#include "utManagerThread.h"
#include "kalmanThread.h"

using namespace yarp;
using namespace yarp::os;
using namespace std;

/**
* \ingroup ultimateTrackerModule
*
* The module that achieves the ultimateTracker task.
*  
*/
class ultimateTracker: public RFModule 
{
private:
    utManagerThread *utMngrThrd;
    kalmanThread    *kalThrd;

    RpcClient        rpcClnt;
    RpcServer        rpcSrvr;

    string robot;
    string name;
    int verbosity;
    int managerRate;
    int kalmanRate;
    int kalmanOrder;
    int timeThres;

    bool useNearBlobber;

public:
    ultimateTracker()
    {
        utMngrThrd    = 0;
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
                case createVocab('s','a','v','e'):
                {
                    // int res=Vocab::encode("saved");
                    // if (utMngrThrd -> save())
                    // {
                    //     reply.addVocab(ack);
                    // }
                    // else
                    //     reply.addVocab(nack);
                    
                    // reply.addVocab(res);
                    // return true;
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
        name  = "ultimateTracker";
        robot = "icub";

        verbosity   =   0;      // verbosity
        managerRate =  20;      // rate of the utManagerThread [ms]
        kalmanRate  =  10;      // rate of the kalmanThread [ms]
        kalmanOrder =   4;      // order of the kalman filters
        timeThres   = 100;      // time threshold for the kalman thread [ms]

        //******************************************************
        //********************** CONFIGS ***********************

        //******************* NAME ******************
            if (rf.check("name"))
            {
                name = rf.find("name").asString();
                cout << "Module name set to "<<name<<endl;  
            }
            else cout << "Module name set to default, i.e. " << name << endl;
            setName(name.c_str());

        //******************* ROBOT ******************
            if (rf.check("robot"))
            {
                robot = rf.find("robot").asString();
                cout << "Robot is: " << robot << endl;
            }
            else cout << "Could not find robot option in the config file; using "
                      << robot << " as default.\n";

        //******************* VERBOSE ******************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                cout << "utManagerThread verbosity set to " << verbosity << endl;
            }
            else cout << "Could not find verbosity option in " <<
                         "config file; using "<< verbosity <<" as default.\n";

        //******************* VERBOSE ******************
            if (rf.check("useNearBlobber"))
            {
                useNearBlobber = rf.find("useNearBlobber").asInt()!=0;
                cout << "utManagerThread useNearBlobber set to " << useNearBlobber << endl;
            }
            else cout << "Could not find useNearBlobber option in " <<
                         "config file; using "<< useNearBlobber <<" as default.\n";

        //****************** managerRate ******************
            if (rf.check("managerRate"))
            {
                managerRate = rf.find("managerRate").asInt();
                cout << "utManagerThread managerRateThread working at " << managerRate << " ms.\n";
            }
            else cout << "Could not find managerRate in the config file; using "
                      << managerRate << " ms as default.\n";

        //****************** kalmanRate ******************
            if (rf.check("kalmanRate"))
            {
                kalmanRate = rf.find("kalmanRate").asInt();
                cout << "utManagerThread kalmanRateThread working at " << kalmanRate << " ms.\n";
            }
            else cout << "Could not find kalmanRate in the config file; using "
                      << kalmanRate << " ms as default.\n";

        //****************** kalmanOrder ******************
            if (rf.check("kalmanOrder"))
            {
                kalmanOrder = rf.find("kalmanOrder").asInt();
                cout << "Kalman filters' order has been set to " << kalmanOrder << ".\n";
            }
            else cout << "Could not find kalmanOrder in the config file; using "
                      << kalmanOrder << " as default.\n";

        //****************** timeThres ******************
            if (rf.check("timeThres"))
            {
                timeThres = rf.find("timeThres").asInt();
                cout << "utManagerThread timeThresThread working at " << timeThres << " ms.\n";
            }
            else cout << "Could not find timeThres in the config file; using "
                      << timeThres << " ms as default.\n";

        //******************************************************
        //*********************** THREADS **********************
            string kalThrdName = name + "/Kalman";
            kalThrd = new kalmanThread(kalmanRate, kalThrdName, robot, verbosity, timeThres, kalmanOrder);
            if (!kalThrd -> start())
            {
                delete kalThrd;
                kalThrd = 0;
                cout << "\nERROR!!! kalmanThread wasn't instantiated!!\n";
                return false;
            }
            cout << "ULTIMATE TRACKER: kalmanThread istantiated...\n";

            string managerThrdName = name + "/Manager";
            utMngrThrd = new utManagerThread(managerRate, managerThrdName, robot, verbosity, kalThrd, useNearBlobber);
            if (!utMngrThrd -> start())
            {
                delete utMngrThrd;
                utMngrThrd = 0;
                cout << "\nERROR!!! utManagerThread wasn't instantiated!!\n";
                return false;
            }
            cout << "ULTIMATE TRACKER: utManagerThread istantiated...\n";

        //******************************************************
        //************************ PORTS ***********************
            rpcClnt.open(("/"+name+"/rpc:o").c_str());
            rpcSrvr.open(("/"+name+"/rpc:i").c_str());
            attach(rpcSrvr);

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

/**
* Main function.
*/
int main(int argc, char * argv[])
{
    Network yarp;

    ResourceFinder moduleRF;
    moduleRF.setVerbose(true);
    moduleRF.setDefaultContext("periPersonalSpace");
    moduleRF.setDefaultConfigFile("ultimateTracker.ini");
    moduleRF.configure(argc,argv);

    if (moduleRF.check("help"))
    {    
        cout << endl << "Options:" << endl;
        cout << "   --context     path:  where to find the called resource (default periPersonalSpace)." << endl;
        cout << "   --from        from:  the name of the .ini file (default ultimateTracker.ini)." << endl;
        cout << "   --name        name:  the name of the module (default ultimateTracker)." << endl;
        cout << "   --robot       robot: the name of the robot. Default icub." << endl;
        cout << "   --verbosity   int:   verbosity level (default 0)." << endl;
        cout << "   --managerRate int:   the period used by the manager thread. Default 20ms." << endl;
        cout << "   --kalmanRate  int:   the period used by the kalman  thread. Default 10ms." << endl;
        cout << "   --kalmanOrder int:   the order of the dynamic model underneath the kalman filter. Default 4 (constant jerk)." << endl;
        cout << "   --timeThres   int:   the threshold after which the kalman gets stoppe. Default 100ms." << endl;
        cout << endl;
        return 0;
    }

    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    ultimateTracker ultTrack;
    return ultTrack.runModule(moduleRF);
}

// empty line to make gcc happy
