/* TACTILE SERVOING v. 0.2
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

#include <yarp/os/all.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string.h> 
#include <ctime>
#include <sstream>

#include "tctServoThread.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/**
* \ingroup tactileServoingModule
*
* The module that achieves the tactileServoing task.
*  
*/
class tactileServoing: public RFModule
{
private:
    tactileServoThread *tctSrvThrd;
    RpcClient             rpcClnt;
    RpcServer             rpcSrvr;

    string robot,name;

    int verbosity,rate;

public:
    tactileServoing()
    {
        tctSrvThrd=0;

        robot    = "icubSim";
        name     = "tactileServoing";

        verbosity = 0;     // verbosity
        rate      = 50;    // rate of the tactileServoThread
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
            }
        }

        reply.addVocab(nack);
        return true;
    }

    bool configure(ResourceFinder &rf)
    {
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
                cout << "tactileServoThread verbosity set to " << verbosity << endl;
            }
            else cout << "Could not find verbosity option in the" <<
                         "config file; using "<< verbosity <<" as default\n";

        //****************** rate ******************
            if (rf.check("rate"))
            {
                rate = rf.find("rate").asInt();
                cout << "tactileServoThread rateThread working at " << rate << " ms\n";
            }
            else cout << "Could not find rate in the config file; using "
                      << rate << " ms as default\n";

            tctSrvThrd = new tactileServoThread(rate, name, robot, verbosity);

            bool strt = tctSrvThrd -> start();
            if (!strt)
            {
                delete tctSrvThrd;
                tctSrvThrd = 0;
                cout << "ERROR!!! tactileServoThread wasn't instantiated!!\n";
                return false;
            }

        return true;
    }

    bool close()
    {
        cout << "TACTILE SERVOING: Stopping threads.." << endl;
        if (tctSrvThrd)
        {
            tctSrvThrd->stop();
            delete tctSrvThrd;
            tctSrvThrd=0;
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
    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("periPersonalSpace");
    rf.setDefaultConfigFile("tactileServoing.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {    
        cout << endl << "Options:" << endl;
        cout << "   --context    path:  where to find the called resource" << endl;
        cout << "   --from       from:  the name of the .ini file." << endl;
        cout << "   --name       name:  the name of the module (default tactileServoing)." << endl;
        cout << "   --robot      robot: the name of the robot. Default icubSim." << endl;
        cout << "   --rate       rate:  the period used by the thread. Default 100ms." << endl;
        cout << "   --verbosity  int:   verbosity level (default 0)." << endl;
        cout << endl;
        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    tactileServoing tctSrvMod;
    return tctSrvMod.runModule(rf);
}
// empty line to make gcc happy
