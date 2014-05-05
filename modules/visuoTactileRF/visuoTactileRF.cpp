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
\defgroup visuoTactileRFModule visuoTactileRFModule

@ingroup periPersonalSpace

A module that handles the visuoTactile Receptive Fields

Date first release: 23/10/2013

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is a module for implementing the VisuoTactile ReceptiveFields on the iCub.

\section lib_sec Libraries 
YARP, ICUB libraries and OPENCV

\section parameters_sec Parameters

--context       \e path
- Where to find the called resource.

--from          \e from
- The name of the .ini file with the configuration parameters.

--name          \e name
- The name of the module (default visuoTactileRF).

--robot         \e rob
- The name of the robot (either "icub" or "icub"). Default icub.

--rate          \e rate
- The period used by the thread. Default 100ms.

--taxelsFile    \e file
- The name of the file the receptive fields are gonna be saved in (and loaded from).
  Default 'taxels2D.ini'.

\section portsc_sec Ports Created
- <i> /<name>/contacts:i </i> it reads the skinContacts from the skinManager.

- <i> /<name>/records:o </i> it prints out some data about the state of the
  recording.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 12.04, Debian Squeeze, Debian Wheezy).

\author: Alessandro Roncone and Matej Hoffmann
*/ 

#include <yarp/os/all.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string> 

#include "vtRFThread.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/**
* \ingroup visuoTactileRFModule
*
* The module that achieves the visuoTactileRF task.
*  
*/
class visuoTactileRF: public RFModule 
{
private:
    vtRFThread *vtRFThrd;

    RpcClient             rpcClnt;
    RpcServer             rpcSrvr;

    string robot,name;
    int verbosity,rate;

public:
    visuoTactileRF()
    {
        vtRFThrd    = 0;
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
                case VOCAB4('s','a','v','e'):
                {
                    int res=Vocab::encode("saved");
                    if (vtRFThrd -> save())
                    {
                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);
                    
                    reply.addVocab(res);
                    return true;
                }
                case VOCAB4('l','o','a','d'):
                {
                    int res=Vocab::encode("loaded");
                    if (vtRFThrd -> load())
                    {
                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);
                    
                    reply.addVocab(res);
                    return true;
                }
                case VOCAB4('r','e','s','e'):
                {
                    vtRFThrd -> resetParzenWindows();
                    reply.addVocab(ack);
                    return true;
                }
                case VOCAB4('s','t','o','p'):
                {
                    vtRFThrd -> stopLearning();
                    reply.addVocab(ack);
                    return true;
                }
               case VOCAB4('r','e','s','t'):
                {
                    vtRFThrd -> restoreLearning();
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
        name  = "visuoTactileRF";
        robot = "icub";

        verbosity  = 0;      // verbosity
        rate       = 100;    // rate of the vtRFThread

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
                      << robot << " as default\n";

        //******************* VERBOSE ******************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                cout << "vtRFThread verbosity set to " << verbosity << endl;
            }
            else cout << "Could not find verbosity option in " <<
                         "config file; using "<< verbosity <<" as default\n";

        //****************** rate ******************
            if (rf.check("rate"))
            {
                rate = rf.find("rate").asInt();
                cout << "vtRFThread rateThread working at " << rate << " ms\n";
            }
            else cout << "Could not find rate in the config file; using "
                      << rate << " ms as default\n";

        //************* skinTaxels' Resource finder **************
            ResourceFinder skinRF;
            skinRF.setVerbose(false);
            skinRF.setDefaultContext("skinGui");                //overridden by --context parameter
            skinRF.setDefaultConfigFile("skinManForearms.ini"); //overridden by --from parameter
            skinRF.configure(0,NULL);

            vector<string> filenames;
            int partNum=4;

            Bottle &skinEventsConf = skinRF.findGroup("SKIN_EVENTS");
            if(!skinEventsConf.isNull())
            {
                printf("SKIN_EVENTS section found\n");

                if(skinEventsConf.check("skinParts"))
                {
                    Bottle* skinPartList = skinEventsConf.find("skinParts").asList();
                    partNum=skinPartList->size();
                }

                if(skinEventsConf.check("taxelPositionFiles"))
                {
                    Bottle *taxelPosFiles = skinEventsConf.find("taxelPositionFiles").asList();

                    for(int i=0;i<partNum;i++)     // all of the skinparts
                    // for(int i=1;i<2;i++)           // only left forearm
                    // for(int i=0;i<1;i++)           // only left hand
                    // for(int i=1;i<partNum;i++)     // everything but the left hand
                    {
                        if (i==2)                   // only right hand
                        {
                            string taxelPosFile = taxelPosFiles->get(i).asString().c_str();
                            string filePath(skinRF.findFile(taxelPosFile.c_str()));
                            if (filePath!="")
                            {
                                printf("*** VisuoTactileRF: filePath [%i] %s\n",i,filePath.c_str());
                                filenames.push_back(filePath);
                            }
                        }
                    }
                    // filenames.pop_back();           // only left forearm
                }
            }
            else
            {
                printf("ERROR! No skin's configuration files found.\n");
                return 0;
            }

        //*************** eyes' Resource finder ****************
            ResourceFinder gazeRF;
            // gazeRF.setQuiet();
            gazeRF.setDefaultContext("iKinGazeCtrl");
            robot=="icub"?gazeRF.setDefaultConfigFile("config.ini"):gazeRF.setDefaultConfigFile("configSim.ini");
            gazeRF.configure(0,NULL);
            double head_version=gazeRF.check("headV2")?2.0:1.0;

            ResourceFinder eyeAlignRF;

            if (gazeRF.check("camerasFile"))
            {
                eyeAlignRF.setVerbose(false);
                gazeRF.check("camerasContext")?
                eyeAlignRF.setDefaultContext(gazeRF.find("camerasContext").asString().c_str()):
                eyeAlignRF.setDefaultContext(gazeRF.getContext().c_str());
                eyeAlignRF.setDefaultConfigFile(gazeRF.find("camerasFile").asString().c_str());            
                eyeAlignRF.configure(0,NULL);
            }

        //******************************************************
        //*********************** THREAD **********************
            if( filenames.size() > 0 )
            {
                vtRFThrd = new vtRFThread(rate, name, robot, verbosity, rf,
                                          filenames, head_version, eyeAlignRF);
                if (!vtRFThrd -> start())
                {
                    delete vtRFThrd;
                    vtRFThrd = 0;
                    cout << "\nERROR!!! vtRFThread wasn't instantiated!!\n";
                    return false;
                }
                cout << "VISUO TACTILE RECEPTIVE FIELDS: vtRFThread istantiated...\n";
            }
            else {
                vtRFThrd = 0;
                cout << "\nERROR!!! vtRFThread wasn't instantiated. No filenames have been given!\n";
                return false;
            }

        //******************************************************
        //************************ PORTS ***********************
            rpcClnt.open(("/"+name+"/rpc:o").c_str());
            rpcSrvr.open(("/"+name+"/rpc:i").c_str());
            attach(rpcSrvr);

        return true;
    }

    bool close()
    {
        cout << "VISUO TACTILE RECEPTIVE FIELDS: Stopping thread.." << endl;
        if (vtRFThrd)
        {
            vtRFThrd->stop();
            delete vtRFThrd;
            vtRFThrd=0;
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
    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder moduleRF;
    moduleRF.setVerbose(false);
    moduleRF.setDefaultContext("periPersonalSpace");
    moduleRF.setDefaultConfigFile("visuoTactileRF.ini");
    moduleRF.configure(argc,argv);

    if (moduleRF.check("help"))
    {    
        cout << endl << "Options:" << endl;
        cout << "   --context    path:   where to find the called resource (default periPersonalSpace)." << endl;
        cout << "   --from       from:   the name of the .ini file (default visuoTactileRF.ini)." << endl;
        cout << "   --name       name:   the name of the module (default visuoTactileRF)." << endl;
        cout << "   --robot      robot:  the name of the robot. Default icub." << endl;
        cout << "   --rate       rate:   the period used by the thread. Default 50ms." << endl;
        cout << "   --verbosity  int:    verbosity level (default 0)." << endl;
        cout << "   --taxelsFile string: the file from which load and save taxels (default taxels2D.ini)." << endl;
        cout << endl;
        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    visuoTactileRF visTacRF;
    return visTacRF.runModule(moduleRF);
}

// empty line to make gcc happy
