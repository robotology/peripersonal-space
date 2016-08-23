/* TEST SKINDYNLIB v. 1.0
 * Copyright (C) 2013 RobotCub Consortium
 * Author:  Alessandro Roncone & Matej Hoffmann
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
\defgroup testSkinDynLibModule testSkinDynLibModule

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
- The name of the module (default testSkinDynLib).

--robot         \e rob
- The name of the robot (either "icub" or "icub"). Default icub.

--rate          \e rate
- The period used by the thread. Default 100ms.

--taxelsFile    \e file
- The name of the file the receptive fields are gonna be saved in (and loaded from).
  Default 'taxels1D.ini' if modality is 1D, otherwise 'taxels2D.ini'.

\section portsc_sec Ports Created
- <i> /<name>/contacts:i </i> it reads the skinContacts from the skinManager.

- <i> /<name>/records:o </i> it prints out some data about the state of the
  recording.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.

\section tested_os_sec Tested OS
Linux (Ubuntu 12.04, Ubuntu 14.04, Debian Squeeze, Debian Wheezy).

\author: Alessandro Roncone
*/

#include <yarp/os/all.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string>

#include <iCub/skinDynLib/iCubSkin.h>

using namespace yarp::os;

using namespace std;

/**
* \ingroup testSkinDynLibModule
*
* The module that achieves the testSkinDynLib task.
*
*/
class testSkinDynLib: public RFModule
{
private:

public:
    testSkinDynLib()
    {
    }

    bool configure(ResourceFinder &rf)
    {
        string taxelPosFile="";
        string filePath="";
        int verbosity = rf.check("verbosity",Value(0)).asInt();

        //************* skinTaxels' Resource finder **************
            ResourceFinder skinRF;
            skinRF.setVerbose(false);
            skinRF.setDefaultContext("skinGui");                //overridden by --context parameter
            skinRF.setDefaultConfigFile("skinManForearms.ini"); //overridden by --from parameter
            skinRF.configure(0,NULL);

            Bottle &skinEventsConf = skinRF.findGroup("SKIN_EVENTS");
            if(!skinEventsConf.isNull())
            {
                if(skinEventsConf.check("skinParts"))
                {
                    Bottle* skinPartList = skinEventsConf.find("skinParts").asList();
                }

                if(skinEventsConf.check("taxelPositionFiles"))
                {
                    Bottle *taxelPosFiles = skinEventsConf.find("taxelPositionFiles").asList();

                    taxelPosFile = taxelPosFiles->get(1).asString().c_str();
                    filePath = skinRF.findFile(taxelPosFile.c_str());
                }
            }
            else
            {
                yError(" No skin configuration files found.");
                return 0;
            }

            // iCub::skinDynLib::skinPart sP;
            // sP.setTaxelPosesFromFile(filePath);
            // sP.print(1);

            iCub::skinDynLib::iCubSkin iCS;
            // iCub::skinDynLib::iCubSkin iCS("skinManForearms.ini","skinGui");
            iCS.print(verbosity);

        return true;
    }

    bool close()
    {
        yInfo("TEST SKINDYNLIB: Stopping module..");

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
    moduleRF.configure(argc,argv);

    if (moduleRF.check("help"))
    {
        yInfo(" ");
        yInfo("Options:");
        yInfo("  --verbosity  int:    verbosity level (default 0).");
        yInfo(" ");
        return 0;
    }

    testSkinDynLib visTacRF;
    return visTacRF.runModule(moduleRF);
}

// empty line to make gcc happy
