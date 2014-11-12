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

#include <stdio.h>
#include <string>
#include <cctype>
#include <algorithm>
#include <map>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

#define PPS_AVOIDANCE_RADIUS        0.2     // [m]
#define PPS_AVOIDANCE_VELOCITY      0.10    // [m/s]
#define PPS_AVOIDANCE_PERSISTENCE   0.3     // [s]
#define PPS_AVOIDANCE_TIMEOUT       1.0     // [s]

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


//*************************************
class Avoidance: public RFModule,
                 public PortReader
{
protected:
    PolyDriver driverCartL,driverCartR;
    PolyDriver driverJointL,driverJointR;
    int contextL,contextR;
    Port dataPort;
    int motionGain;

    Mutex mutex;
    struct Data {
        ICartesianControl *iarm;
        Vector point, dir;
        Vector home_x, home_o;
        double persistence, timeout;
        Data(): point(3,0.0), dir(3,0.0),
                home_x(3,0.0), home_o(4,0.0),
                persistence(0.0), timeout(-1.0) { }
    };
    map<string,Data> data;
    
    //********************************************
    bool read(ConnectionReader &connection)
    {
        Bottle input;
        input.read(connection);
        if (input.size()>=7)
        {
            string arm=input.get(0).asString().c_str();
            transform(arm.begin(),arm.end(),arm.begin(),::tolower);
            mutex.lock();
            map<string,Data>::iterator it=data.find(arm);
            if (it!=data.end())
            {
                Data &d=it->second;
                d.point[0]=input.get(1).asDouble();
                d.point[1]=input.get(2).asDouble();
                d.point[2]=input.get(3).asDouble();
                d.dir[0]=input.get(4).asDouble();
                d.dir[1]=input.get(5).asDouble();
                d.dir[2]=input.get(6).asDouble();
                d.persistence=PPS_AVOIDANCE_PERSISTENCE;
                d.timeout=PPS_AVOIDANCE_TIMEOUT;
            }
            mutex.unlock();
        }
        return true;
    }

    //********************************************
    void manageArm(Data &data)
    {
        if (data.persistence>0.0)
        {
            Vector x,o;
            data.iarm->getPose(x,o);
            if (norm(x-data.home_x)>PPS_AVOIDANCE_RADIUS)
                data.iarm->stopControl();
            else
                data.iarm->setTaskVelocities((motionGain*PPS_AVOIDANCE_VELOCITY/norm(data.dir))*data.dir,Vector(4,0.0)); 

            data.persistence=std::max(data.persistence-getPeriod(),0.0);
            if (data.persistence==0.0)
                data.iarm->stopControl();
        }
        else if (data.timeout>0.0)
        {
            data.timeout=std::max(data.timeout-getPeriod(),0.0);
        }
        else if (data.timeout==0.0)
        {
            data.iarm->goToPose(data.home_x,data.home_o);
            data.timeout=-1.0;
        }
    }
    
public:
    //********************************************
    bool configure(ResourceFinder &rf)
    {
        data["left"]=Data();
        data["left"].home_x[0]=-0.30;
        data["left"].home_x[1]=-0.20;
        data["left"].home_x[2]=+0.05;

        Matrix R(4,4);
        R(0,0)=-1.0; R(2,1)=-1.0; R(1,2)=-1.0; R(3,3)=1.0;
        data["left"].home_o=dcm2axis(R);

        data["right"]=data["left"];
        data["right"].home_x[1]=-data["right"].home_x[1];

        string name=rf.check("name",Value("avoidance")).asString().c_str();
        motionGain = -1.0;
        if (rf.check("catching"))
        {
            motionGain = 1.0;
        }

        Property optionCartL;
        optionCartL.put("device","cartesiancontrollerclient");
        optionCartL.put("remote","/icub/cartesianController/left_arm");
        optionCartL.put("local",("/"+name+"/cart/left_arm").c_str());
        if (!driverCartL.open(optionCartL))
        {
            close();
            return false;
        }

        Property optionCartR;
        optionCartR.put("device","cartesiancontrollerclient");
        optionCartR.put("remote","/icub/cartesianController/right_arm");
        optionCartR.put("local",("/"+name+"/cart/right_arm").c_str());
        if (!driverCartR.open(optionCartR))
        {
            close();
            return false;
        }

        Property optionJointL;
        optionJointL.put("device","remote_controlboard");
        optionJointL.put("remote","/icub/left_arm");
        optionJointL.put("local",("/"+name+"/joint/left_arm").c_str());
        if (!driverJointL.open(optionJointL))
        {
            close();
            return false;
        }

        Property optionJointR;
        optionJointR.put("device","remote_controlboard");
        optionJointR.put("remote","/icub/right_arm");
        optionJointR.put("local",("/"+name+"/joint/right_arm").c_str());
        if (!driverJointR.open(optionJointR))
        {
            close();
            return false;
        }

        driverCartL.view(data["left"].iarm);
        driverCartR.view(data["right"].iarm);

        data["left"].iarm->storeContext(&contextL);
        data["right"].iarm->storeContext(&contextR);

        Vector dof;
        data["left"].iarm->getDOF(dof);
        dof=0.0; dof[3]=dof[4]=dof[5]=dof[6]=1.0;
        data["left"].iarm->setDOF(dof,dof);  data["right"].iarm->setDOF(dof,dof);
        data["left"].iarm->setTrajTime(0.9); data["right"].iarm->setTrajTime(0.9);

        data["left"].iarm->goToPoseSync(data["left"].home_x,data["left"].home_o);
        data["right"].iarm->goToPoseSync(data["right"].home_x,data["right"].home_o);
        data["left"].iarm->waitMotionDone();
        data["right"].iarm->waitMotionDone();

        IInteractionMode  *imode; driverJointL.view(imode);
        IImpedanceControl *iimp;  driverJointL.view(iimp);
        for (int i=0; i<2; i++)
        {
            imode->setInteractionMode(0,VOCAB_IM_COMPLIANT); iimp->setImpedance(0,0.4,0.03); 
            imode->setInteractionMode(1,VOCAB_IM_COMPLIANT); iimp->setImpedance(1,0.4,0.03);
            imode->setInteractionMode(2,VOCAB_IM_COMPLIANT); iimp->setImpedance(2,0.4,0.03);
            imode->setInteractionMode(3,VOCAB_IM_COMPLIANT); iimp->setImpedance(3,0.2,0.01);
            imode->setInteractionMode(4,VOCAB_IM_COMPLIANT); iimp->setImpedance(4,0.2,0.0);

            driverJointR.view(imode);
            driverJointR.view(iimp);
        }

        dataPort.open(("/"+name+"/data:i").c_str());
        dataPort.setReader(*this);
        return true; 
    }

    //********************************************
    bool updateModule()
    {
        mutex.lock();
        manageArm(data["left"]);
        manageArm(data["right"]);
        mutex.unlock(); 
        return true;
    }

    //********************************************
    double getPeriod()
    {
        return 0.05;
    }

    //********************************************
    bool close()
    {
        dataPort.close();

        if (driverCartL.isValid())
        {
            data["left"].iarm->stopControl();
            data["left"].iarm->restoreContext(contextL);
            driverCartL.close(); 
        }

        if (driverCartR.isValid())
        {
            data["right"].iarm->stopControl();
            data["right"].iarm->restoreContext(contextR);
            driverCartR.close();
        }

        if (driverJointL.isValid())
        {
            IInteractionMode *imode;
            driverJointL.view(imode);
            for (int j=0; j<5; j++)
                imode->setInteractionMode(j,VOCAB_IM_STIFF);

            driverJointL.close();
        }

        if (driverJointR.isValid())
        {
            IInteractionMode *imode;
            driverJointR.view(imode);
            for (int j=0; j<5; j++)
                imode->setInteractionMode(j,VOCAB_IM_STIFF);

            driverJointR.close();
        }

        return true;
    }
};



//********************************************
int main(int argc, char * argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.configure(argc,argv);
    Avoidance module;
    return module.runModule(rf);
}


