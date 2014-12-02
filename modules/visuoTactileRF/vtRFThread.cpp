#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iomanip>

#include "vtRFThread.h"

#define RADIUS         2 // radius in px of every taxel (in the images)
#define RFEXTENSION  0.3 // extension of the receptive field (0.3 m)
#define HAND_LEFT      1
#define FOREARM_LEFT   2
#define HAND_RIGHT     4
#define FOREARM_RIGHT  5

// enum SkinPart { 
//     SKIN_PART_UNKNOWN=0, 
//     SKIN_LEFT_HAND, SKIN_LEFT_FOREARM, SKIN_LEFT_UPPER_ARM, 
//     SKIN_RIGHT_HAND, SKIN_RIGHT_FOREARM, SKIN_RIGHT_UPPER_ARM, 
//     SKIN_FRONT_TORSO, 
//     SKIN_PART_ALL, SKIN_PART_SIZE
// };

IncomingEvent eventFromBottle(const Bottle &b)
{
    IncomingEvent ie;
    ie.fromBottle(b);
    return ie;
};

vtRFThread::vtRFThread(int _rate, const string &_name, const string &_robot, int _v,
                       const ResourceFinder &_moduleRF, vector<string> _fnames,
                       double _hV, const ResourceFinder &_eyeCalibRF) :
                       RateThread(_rate), name(_name), robot(_robot), verbosity(_v),
                       filenames(_fnames)
{
    //******************* PORTS ******************
        imagePortInR  = new BufferedPort<ImageOf<PixelRgb> >;
        imagePortInL  = new BufferedPort<ImageOf<PixelRgb> >;
        imagePortOutR = new BufferedPort<ImageOf<PixelRgb> >;
        imagePortOutL = new BufferedPort<ImageOf<PixelRgb> >;
        dTPort        = new BufferedPort<Bottle>;
        eventsPort    = new BufferedPort<Bottle>;
        skinGuiPortForearmL  = new BufferedPort<Vector>;
        skinGuiPortForearmR  = new BufferedPort<Vector>;
        skinGuiPortHandL  = new BufferedPort<Vector>;
        skinGuiPortHandR  = new BufferedPort<Vector>;
        skinPortIn    = new BufferedPort<iCub::skinDynLib::skinContactList>;
        skinPortOut   = new BufferedPort<Bottle>;

    //******************* ARMS, EYEWRAPPERS, WHATEVER ******************
        armR = new iCubArm("right");
        armL = new iCubArm("left");

        eWR  = new eyeWrapper("right",_hV,_eyeCalibRF);
        eWL  = new eyeWrapper("left", _hV,_eyeCalibRF);
        
        rf = const_cast<ResourceFinder*>(&_moduleRF);

        eventsFlag   = true;
        learningFlag = true;

    //******************* PATH ******************
        path = rf->getHomeContextPath().c_str();
        path = path+"/";
        taxelsFile = rf->check("taxelsFile", Value("taxels.ini")).asString().c_str();
        printMessage(0,"Storing file set to: %s\n", (path+taxelsFile).c_str());
}

bool vtRFThread::threadInit()
{
    imagePortInR        -> open(("/"+name+"/imageR:i").c_str());
    imagePortInL        -> open(("/"+name+"/imageL:i").c_str());
    imagePortOutR       -> open(("/"+name+"/imageR:o").c_str());
    imagePortOutL       -> open(("/"+name+"/imageL:o").c_str());
    dTPort              -> open(("/"+name+"/input:i").c_str());
    eventsPort          -> open(("/"+name+"/events:i").c_str());
    skinGuiPortForearmL -> open(("/"+name+"/skinGuiForearmL:o").c_str());
    skinGuiPortForearmR -> open(("/"+name+"/skinGuiForearmR:o").c_str());
    skinGuiPortHandL    -> open(("/"+name+"/skinGuiHandL:o").c_str());
    skinGuiPortHandR    -> open(("/"+name+"/skinGuiHandR:o").c_str());
    skinPortIn          -> open(("/"+name+"/skin_events:i").c_str());
    skinPortOut         -> open(("/"+name+"/skin_events:o").c_str());

    // I know that I should remove this but it's harmless (and I'm overly lazy)
    if (robot=="icub")
    {
        Network::connect("/icub/camcalib/left/out",("/"+name+"/imageL:i").c_str());
        Network::connect("/icub/camcalib/right/out",("/"+name+"/imageR:i").c_str());
    }
    else
    {
        Network::connect("/icubSim/cam/left",("/"+name+"/imageL:i").c_str());
        Network::connect("/icubSim/cam/right",("/"+name+"/imageR:i").c_str());
    }

    Network::connect(("/"+name+"/imageL:o").c_str(),"/vtRF/left");
    Network::connect(("/"+name+"/imageR:o").c_str(),"/vtRF/right");

    Network::connect("/doubleTouch/status:o",("/"+name+"/input:i").c_str());
    Network::connect("/visuoTactileWrapper/events:o",("/"+name+"/events:i").c_str());

    Network::connect(("/"+name+"/skinGuiForearmL:o").c_str(),"/vtRFSkinGui/left_forearm:i");
    Network::connect(("/"+name+"/skinGuiForearmR:o").c_str(),"/vtRFSkinGui/right_forearm:i");
    Network::connect(("/"+name+"/skinGuiHandL:o").c_str(),"/vtRFSkinGui/left_hand:i");
    Network::connect(("/"+name+"/skinGuiHandR:o").c_str(),"/vtRFSkinGui/right_hand:i");
       
    Network::connect("/skinManager/skin_events:o",("/"+name+"/skin_events:i").c_str());

    /**************************/
        Property OptR;
        OptR.put("robot",  robot.c_str());
        OptR.put("part",   "right_arm");
        OptR.put("device", "remote_controlboard");
        OptR.put("remote",("/"+robot+"/right_arm").c_str());
        OptR.put("local", ("/"+name +"/right_arm").c_str());

        if (!ddR.open(OptR))
        {
            printMessage(0,"ERROR: could not open right_arm PolyDriver!\n");
            return false;
        }
        bool ok = 1;
        if (ddR.isValid())
        {
            ok = ok && ddR.view(iencsR);
        }
        if (!ok)
        {
            printMessage(0,"\nERROR: Problems acquiring right_arm interfaces!!!!\n");
            return false;
        }
        iencsR->getAxes(&jntsR);
        encsR = new yarp::sig::Vector(jntsR,0.0);

    /**************************/
        Property OptL;
        OptL.put("robot",  robot.c_str());
        OptL.put("part",   "left_arm");
        OptL.put("device", "remote_controlboard");
        OptL.put("remote",("/"+robot+"/left_arm").c_str());
        OptL.put("local", ("/"+name +"/left_arm").c_str());

        if (!ddL.open(OptL))
        {
            printMessage(0,"ERROR: could not open left_arm PolyDriver!\n");
            return false;
        }
        ok = 1;
        if (ddL.isValid())
        {
            ok = ok && ddL.view(iencsL);
        }
        if (!ok)
        {
            printMessage(0,"\nERROR: Problems acquiring left_arm interfaces!!!!\n");
            return false;
        }
        iencsL->getAxes(&jntsL);
        encsL = new yarp::sig::Vector(jntsL,0.0);

    /**************************/
        Property OptT;
        OptT.put("robot",  robot.c_str());
        OptT.put("part",   "torso");
        OptT.put("device", "remote_controlboard");
        OptT.put("remote",("/"+robot+"/torso").c_str());
        OptT.put("local", ("/"+name +"/torso").c_str());

        if (!ddT.open(OptT))
        {
            printMessage(0,"ERROR: could not open torso PolyDriver!\n");
            return false;
        }
        ok = 1;
        if (ddT.isValid())
        {
            ok = ok && ddT.view(iencsT);
        }
        if (!ok)
        {
            printMessage(0,"\nERROR: Problems acquiring head interfaces!!!!\n");
            return false;
        }
        iencsT->getAxes(&jntsT);
        encsT = new yarp::sig::Vector(jntsT,0.0);

    /**************************/
        Property OptH;
        OptH.put("robot",  robot.c_str());
        OptH.put("part",   "head");
        OptH.put("device", "remote_controlboard");
        OptH.put("remote",("/"+robot+"/head").c_str());
        OptH.put("local", ("/"+name +"/head").c_str());

        if (!ddH.open(OptH))
        {
            printMessage(0,"ERROR: could not open head PolyDriver!\n");
            return false;
        }
        ok = 1;
        if (ddH.isValid())
        {
            ok = ok && ddH.view(iencsH);
        }
        if (!ok)
        {
            printMessage(0,"\nERROR: Problems acquiring head interfaces!!!!\n");
            return false;
        }
        iencsH->getAxes(&jntsH);
        encsH = new yarp::sig::Vector(jntsH,0.0);

    /**************************/
        printMessage(1,"Setting up iCubSkin...\n");
        for(unsigned int i=0;i<filenames.size();i++)
        {
            string filePath = filenames[i];
            printMessage(1,"i: %i filePath: %s\n",i,filePath.c_str());
            skinPart sP;
            if ( setTaxelPosesFromFile(filePath,sP) )
                iCubSkin.push_back(sP);
        }
        load();

        printMessage(0,"iCubSkin correctly instantiated. Size: %i\n",iCubSkin.size());
        if (verbosity>= 2)
        {
            for (size_t i = 0; i < iCubSkin.size(); i++)
            {
                iCubSkin[i].print();
            }
        }

        for (int i = 0; i < 4; i++)
        {
            H.push_back(skinPart());
            H[i].taxel.push_back(Taxel());
        }

        H[0].name =  "H0";
        H[1].name =  "HN";
        H[2].name = "EEL";
        H[3].name = "EER";

    return true;
}

void vtRFThread::run()
{
    incomingEvents.clear();

    // read from the input ports
    dTBottle = dTPort       -> read(false);
    event    = eventsPort   -> read(false);
    imageInR = imagePortInR -> read(false);
    imageInL = imagePortInL -> read(false); 
    iCub::skinDynLib::skinContactList *skinContacts  = skinPortIn -> read(false);

    // process the port coming from the doubleTouch (for visualization)
    if (dTBottle != NULL)
    {
        H[0].taxel[0].Pos = matrixFromBottle(*dTBottle,4,4,4).subcol(0,3,3);
        H[1].taxel[0].Pos = matrixFromBottle(*dTBottle,20,4,4).subcol(0,3,3);

        if (currentTask == "R2L")
        {
            H[0].taxel[0].WRFPos=locateTaxel(H[0].taxel[0].Pos,"left_forearm");
            H[1].taxel[0].WRFPos=locateTaxel(H[1].taxel[0].Pos,"right_hand");
        }
        else if (currentTask == "L2R")
        {
            H[0].taxel[0].WRFPos=locateTaxel(H[0].taxel[0].Pos,"right_forearm");
            H[1].taxel[0].WRFPos=locateTaxel(H[1].taxel[0].Pos,"left_hand");
        }
    }

    // project taxels in World Reference Frame
    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
        {
            iCubSkin[i].taxel[j].WRFPos=locateTaxel(iCubSkin[i].taxel[j].Pos,iCubSkin[i].name);
        }
    }
    H[2].taxel[0].WRFPos=locateTaxel(H[2].taxel[0].Pos,"left_hand");
    H[3].taxel[0].WRFPos=locateTaxel(H[3].taxel[0].Pos,"right_hand");

    // process the port coming from the visuoTactileWrapper
    if (event != NULL)
    {
        // read teh events
        printMessage(3,"Event! size %d: %i ", event->size());
        for (int i = 0; i < event->size(); i++)
        {
            incomingEvents.push_back(IncomingEvent(*(event -> get(i).asList())));
            if (verbosity>=3)   
                incomingEvents.back().print();
        }

        // manage the buffer
        timeNow     = yarp::os::Time::now();
        if (eventsFlag)
        {
            eventsFlag  = false;
            printMessage(0,"Starting the buffer..\n");
        }
        else
            bufferizeEvents();

        // limit the size of the buffer to 100, i.e. 5 seconds of acquisition
        if (eventsBuffer.size() >= 100)
        {
            eventsBuffer.erase(eventsBuffer.begin());
            printMessage(3,"Too many samples: removing the older element from the buffer..\n");
        }

        // detect contacts and train the taxels
        if (skinContacts && eventsBuffer.size()>10)
        {
            std::vector<unsigned int> IDv; IDv.clear();
            int IDx = -1;
            if (detectContact(skinContacts, IDx, IDv))
            {
                printMessage(0,"Contact! Training the taxels..\n");
                timeNow     = yarp::os::Time::now();
                if (learningFlag == true )
                {
                	eventsFlag  = trainTaxels(IDv,IDx);
                }
                else
	                printMessage(0,"No Learning has been put in place.\n");

                eventsBuffer.clear();
            }
        }
    }
    // if there's no input for more than 2 seconds, clear the buffer
    else if (yarp::os::Time::now() - timeNow > 2)
    {
        eventsFlag = true;
        eventsBuffer.clear();
        timeNow = yarp::os::Time::now();
        printMessage(2,"No significant event in the last 2 seconds. Erasing the buffer.. \n");
    }
    
    if (imageInR!=NULL)
    {
        projectIntoImagePlane(iCubSkin,"rightEye",false);
        projectIntoImagePlane(       H,"rightEye",false);
        handleImages("rightEye");    // draw things in the images
    }

    if (imageInL!=NULL)
    {
        projectIntoImagePlane(iCubSkin,"leftEye",false);
        projectIntoImagePlane(       H,"leftEye",false);
        handleImages("leftEye");     // draw things in the images
    }
    
    if (incomingEvents.size()>0)
    {
        projectIncomingEvent();     // project event onto the taxels' RF
        computeResponse();          // compute the response of each taxel
    }

    sendContactsToSkinGui();        // self explicative
    manageSkinEvents();
}

void vtRFThread::manageSkinEvents()
{
    // main/src/modules/skinManager/src/compensationThread.cpp:250
    vector <int> taxelsIDs; 
    string bodypart;
    int iCubSkinID;
    bool isThereAnEvent = false;

    if (incomingEvents.size()>0)  // if there's an event
    {
        for (size_t i = 0; i < iCubSkin.size(); i++) // cycle through the skinparts
        {
            if (!isThereAnEvent)       // process only one contact at a time
            {
                for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++) // cycle through the taxels
                {
                    if (iCubSkin[i].taxel[j].Resp > 150)
                    {
                        taxelsIDs.push_back(iCubSkin[i].taxel[j].ID);
                        isThereAnEvent = true;
                    }
                }
                if (isThereAnEvent)
                {
                    bodypart   = iCubSkin[i].name;
                    iCubSkinID = i;
                }
                else
                    taxelsIDs.clear();
            }
        }
    }

    if (isThereAnEvent && taxelsIDs.size()>0)
    {
        Vector geoCenter(3,0.0), normalDir(3,0.0);
        Bottle &b = skinPortOut->prepare();
        b.clear();

        if (bodypart == "left_forearm" || bodypart == "left_hand")
        {
            b.addString("left");
        }
        else if (bodypart == "right_forearm" || bodypart == "right_hand")
        {
            b.addString("right");
        }

        for (size_t i = 0; i < taxelsIDs.size(); ++i)
        {
            for (size_t j = 0; j < iCubSkin[iCubSkinID].taxel.size(); j++)
            {
                if (iCubSkin[iCubSkinID].taxel[j].ID == taxelsIDs[i])
                {
                    geoCenter += iCubSkin[iCubSkinID].taxel[j].WRFPos;
                    normalDir += locateTaxel(iCubSkin[iCubSkinID].taxel[j].Norm,bodypart);
                }
            }
        }

        geoCenter /= taxelsIDs.size();
        normalDir /= taxelsIDs.size();
        vectorIntoBottle(geoCenter,b);
        vectorIntoBottle(normalDir,b);
        skinPortOut->write();     // send something anyway (if there is no contact the bottle is empty)
    }
}

void vtRFThread::sendContactsToSkinGui()
{
    for(size_t i=0; i<iCubSkin.size(); i++)
    {
        if(iCubSkin[i].name == "left_forearm")
        {
            Vector& resp2skin_leftForearm = skinGuiPortForearmL->prepare();
            resp2skin_leftForearm.resize(iCubSkin[i].size,0.0);
            if (incomingEvents.size()>0)
            {
                for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
                {
                    resp2skin_leftForearm[iCubSkin[i].taxel[j].ID] = iCubSkin[i].taxel[j].Resp*100/255;
                }
           }
           skinGuiPortForearmL->write(); 
        }
        else if(iCubSkin[i].name == "right_forearm")
        {
            Vector& resp2skin_rightForearm = skinGuiPortForearmR->prepare();
            resp2skin_rightForearm.resize(iCubSkin[i].size,0.0);
            if (incomingEvents.size()>0)
            {
                for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
                {
                    resp2skin_rightForearm[iCubSkin[i].taxel[j].ID] = iCubSkin[i].taxel[j].Resp*100/255;
                }
           }
           skinGuiPortForearmR->write(); 
        }
        else if(iCubSkin[i].name == "left_hand")
        {
            Vector& resp2skin_leftHand = skinGuiPortHandL->prepare();
            resp2skin_leftHand.resize(iCubSkin[i].size,0.0);
            if (incomingEvents.size()>0)
            {
                for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
                {
                    resp2skin_leftHand[iCubSkin[i].taxel[j].ID] = iCubSkin[i].taxel[j].Resp*100/255;
                }
            }
            skinGuiPortHandL->write(); 
        }
        else if(iCubSkin[i].name == "right_hand")
        {
            Vector& resp2skin_rightHand = skinGuiPortHandR->prepare();
            resp2skin_rightHand.resize(iCubSkin[i].size,0.0);
            if (incomingEvents.size()>0)
            {
                for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
                {
                    resp2skin_rightHand[iCubSkin[i].taxel[j].ID] = iCubSkin[i].taxel[j].Resp*100/255;
                }
            }
            skinGuiPortHandR->write(); 
        }
    }
}

bool vtRFThread::detectContact(iCub::skinDynLib::skinContactList *_sCL, int &idx,
                               std::vector <unsigned int> &v)
{
    // Search for a suitable contact:
    for(iCub::skinDynLib::skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++)
    {
        if( it -> getPressure() > 15 )
        {
            for (size_t i = 0; i < iCubSkin.size(); i++)
            {
                if ((it -> getSkinPart() == FOREARM_RIGHT && iCubSkin[i].name == "right_forearm") ||
                    (it -> getSkinPart() == FOREARM_LEFT  && iCubSkin[i].name == "left_forearm" ) ||
                    (it -> getSkinPart() ==    HAND_RIGHT && iCubSkin[i].name == "right_hand"   ) ||
                    (it -> getSkinPart() ==    HAND_LEFT  && iCubSkin[i].name == "left_hand"    )    )
                {
                    idx = i;
                    v = it -> getTaxelList();

                    printMessage(1,"Contact! Taxels' ID:");
                    if (verbosity>=1)
                    {
                        for (size_t i = 0; i < v.size(); i++)
                            printf("\t%i",v[i]);

                        printf("\n");
                    }

                    return true;
                }
            }
        }
    }
    return false;
}

bool vtRFThread::load()
{
    string fileName=rf->findFile("taxelsFile").c_str();
    Property data; data.fromConfigFile(fileName.c_str());
    Bottle b; b.read(data);    
    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        Bottle bb = b.findGroup(iCubSkin[i].name.c_str());
        int nTaxels = bb.find("nTaxels").asInt();

        Bottle *bbb;
        for (int j = 0; j < nTaxels; j++)
        {
            bbb = bb.get(j+2).asList();
            iCubSkin[i].taxel[j].ID = bbb->get(0).asInt();
            iCubSkin[i].taxel[j].pwe.setHist(vectorFromBottle(*bbb->get(1).asList(),0,10));
        }
    }

    return true;
}

bool vtRFThread::save()
{
    string fnm=path+taxelsFile;
    ofstream myfile;
    myfile.open(fnm.c_str(),ios::trunc);

    if (myfile.is_open())
    {
        for (size_t i = 0; i < iCubSkin.size(); i++)
        {
            myfile << "[" << iCubSkin[i].name << "]" << endl;
            myfile << "nTaxels\t" << iCubSkin[i].taxel.size() << endl;
            Bottle data;

            for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
            {
                data.clear();
                Bottle &values = data.addList();
                for (int k = 0; k < iCubSkin[i].taxel[j].pwe.getHistSize(); k++)
                {
                    values.addInt(iCubSkin[i].taxel[j].pwe.getHist(k));
                }
                myfile << iCubSkin[i].taxel[j].ID << "\t" << data.toString() << endl;
            }
        }
    }
    myfile.close();
    return true;
}

bool vtRFThread::bufferizeEvents()
{
    // deprecated
    // if (eventsBuffer.size()>0 && incomingEvents.size()>0)
    // {
    //     eventsBuffer.push_back(incomingEvents.back());
    //     Vector boh = eventsBuffer.back().Pos - incomingEvents.back().Pos;
    //     if (norm(boh) > 0.0005 )
    //         eventsBuffer.push_back(incomingEvents.back());
    //     else
    //         return false;
    // }
    // else if (incomingEvents.size()>0)
        eventsBuffer.push_back(incomingEvents.back());

    printMessage(3,"I'm bufferizing! Size %i\n",eventsBuffer.size());

    return true;
}

bool vtRFThread::trainTaxels(const std::vector<unsigned int> IDv, const int IDx)
{
    std::vector<unsigned int> v = IDv;

    Matrix T_a = eye(4);                     // transform matrix relative to the arm
    if ((iCubSkin[IDx].name == "left_forearm") || (iCubSkin[IDx].name == "left_hand"))
    {
        iencsL->getEncoders(encsL->data());
        yarp::sig::Vector qL=encsL->subVector(0,6);
        armL -> setAng(qL*CTRL_DEG2RAD);
        if (iCubSkin[IDx].name == "left_forearm") 
            T_a = armL -> getH(3+4, true);
        else //(iCubSkin[i].name == "left_hand") 
            T_a = armL -> getH(3+6, true);
    }
    else if ((iCubSkin[IDx].name == "right_forearm") || (iCubSkin[IDx].name == "right_hand"))
    {
        iencsR->getEncoders(encsR->data());
        yarp::sig::Vector qR=encsR->subVector(0,6);
        armR -> setAng(qR*CTRL_DEG2RAD);
        if (iCubSkin[IDx].name == "right_forearm")
            T_a = armR -> getH(3+4, true);
        else //(iCubSkin[i].name == "right_hand") 
            T_a = armR -> getH(3+6, true);
    }
    else
    {
        printMessage(0,"ERROR in trainTaxels!\n");
        return false;
    }

    for (size_t j = 0; j < iCubSkin[IDx].taxel.size(); j++)
    {
        for (size_t k = 0; k < eventsBuffer.size(); k++)
        {
            IncomingEvent projection = projectIntoTaxelRF(iCubSkin[IDx].taxel[j].RF,T_a,eventsBuffer[k]);

            bool flag = false;
            for (size_t w = 0; w < v.size(); w++)
            {
                if (iCubSkin[IDx].taxel[j].ID == v[w])
                {
                    flag = true;
                }
            }
            if (flag)
            {
                printMessage(2,"Training Taxels: ID %i k %i norm(pos) %g eventsBuffer(k) %s\n",iCubSkin[IDx].taxel[j].ID,k,norm(projection.Pos),eventsBuffer[k].Pos.toString().c_str());
                iCubSkin[IDx].taxel[j].pwe.addSample(norm(projection.Pos));
            }
            // a negative sample is added only if
            // 1. the z is positive
            // 2. the x and the y are between -75% and +75% of the RF's extension (i.e. 15cm)
            else if (!flag && projection.Pos[2]>=0 && 
                      projection.Pos[0] >= - iCubSkin[IDx].taxel[j].pwe.getExt() * 75 /100 &&
                      projection.Pos[0] <=   iCubSkin[IDx].taxel[j].pwe.getExt() * 75 /100 &&
                      projection.Pos[1] >= - iCubSkin[IDx].taxel[j].pwe.getExt() * 75 /100 &&
                      projection.Pos[1] <=   iCubSkin[IDx].taxel[j].pwe.getExt() * 75 /100 )
            {
                iCubSkin[IDx].taxel[j].pwe.removeSample(norm(projection.Pos));
            }
        }
    }

    return true;
}

bool vtRFThread::projectIncomingEvent()
{
    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        Matrix T_a = eye(4);                               // transform matrix relative to the arm
        if ((iCubSkin[i].name == "left_forearm") || (iCubSkin[i].name == "left_hand"))
        {
            iencsL->getEncoders(encsL->data());
            yarp::sig::Vector qL=encsL->subVector(0,6);
            armL -> setAng(qL*CTRL_DEG2RAD);
            if (iCubSkin[i].name == "left_forearm") 
                T_a = armL -> getH(3+4, true);
            else //(iCubSkin[i].name == "left_hand") 
                T_a = armL -> getH(3+6, true);
        }
        else if ((iCubSkin[i].name == "right_forearm") || (iCubSkin[i].name == "right_hand"))
        {
            iencsR->getEncoders(encsR->data());
            yarp::sig::Vector qR=encsR->subVector(0,6);
            armR -> setAng(qR*CTRL_DEG2RAD);
            if (iCubSkin[i].name == "right_forearm")
                T_a = armR -> getH(3+4, true);
            else //(iCubSkin[i].name == "right_hand") 
                T_a = armR -> getH(3+6, true);
        }
        else
            printMessage(0,"ERROR in projectIncomingEvent!\n");

        for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
        {
            iCubSkin[i].taxel[j].Evnt=projectIntoTaxelRF(iCubSkin[i].taxel[j].RF,T_a,incomingEvents[incomingEvents.size()-1]);
            // if (j==100)
            // {
                printMessage(4,"projection -> i: %i\t Taxel %i\t Event: ",i,j);
                if (verbosity>=4)
                    iCubSkin[i].taxel[j].Evnt.print();
            // }
        }
    }
    return true;
}

IncomingEvent vtRFThread::projectIntoTaxelRF(const Matrix &RF,const Matrix &T_a,const IncomingEvent &e)
{
    IncomingEvent Event_projected = e;

    Matrix T_a_proj = T_a * RF;

    Vector p=e.Pos; p.push_back(1);
    Vector v=e.Vel; v.push_back(1);

    Event_projected.Pos = SE3inv(T_a_proj)*p;        Event_projected.Pos.pop_back();
    Event_projected.Vel = SE3inv(T_a_proj)*v;        Event_projected.Vel.pop_back();

    if (e.Radius != -1.0)
    {
        Event_projected.Pos(2) -= Event_projected.Radius;
    }

    return Event_projected;
}

Vector vtRFThread::projectIntoTaxelRF(const Matrix &RF,const Matrix &T_a,const Vector &wrfpos)
{
    Matrix T_a_proj = T_a * RF;

    Vector p=wrfpos;
    p.push_back(1);
    p = SE3inv(T_a_proj)*p;
    p.pop_back();

    return p;
}

bool vtRFThread::resetParzenWindows()
{
    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
        {
            iCubSkin[i].taxel[j].resetParzenWindow();
        }
    }
}

bool vtRFThread::stopLearning()
{
	learningFlag = false;
	return true;
}

bool vtRFThread::restoreLearning()
{
	learningFlag = true;
	return true;
}

bool vtRFThread::computeResponse()
{
    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
        {
            iCubSkin[i].taxel[j].computeResponse();
        }
    }

    return true;
}

bool vtRFThread::projectPoint(const string &type, const yarp::sig::Vector &x, yarp::sig::Vector &px, const bool flag)
{
    if (x.length()<3)
    {
        fprintf(stdout,"Not enough values given for the point!\n");
        return false;
    }

    bool isLeft=(type=="leftEye");

    yarp::sig::Matrix  *Prj=(isLeft?eWL->Prj:eWR->Prj);
    iCubEye            *eye=(isLeft?eWL->eye:eWR->eye);

    if (Prj)
    {
        iencsT->getEncoders(encsT->data());
        yarp::sig::Vector torso=*encsT;
        iencsH->getEncoders(encsH->data());
        yarp::sig::Vector  head=*encsH;

        yarp::sig::Vector q(8);
        q[0]=torso[2]; 
        q[1]=torso[1];
        q[2]=torso[0];
        q[3]=head[0];
        q[4]=head[1];
        q[5]=head[2];
        q[6]=head[3];
        if (isLeft)
            q[7]=head[4]+head[5]/2.0;
        else
            q[7]=head[4]-head[5]/2.0;
        q=CTRL_DEG2RAD*q;
        
        yarp::sig::Vector xo=x;
        if (xo.length()<4)
            xo.push_back(1.0);  // impose homogeneous coordinates

        eye->setAng(q);
        yarp::sig::Vector xe;
        // find position wrt the camera frame
        // if (flag)
            xe=SE3inv(eye->getH())*xo;
        // else
        //  xe=SE3inv(eye->getH())*xo;

        // find the 2D projection
        px=*Prj*xe;
        px=px/px[2];
        px.pop_back();

        return true;
    }
    else
    {
        fprintf(stdout,"Unspecified projection matrix for %s camera!\n",type.c_str());
        return false;
    }
}

bool vtRFThread::pushExtrinsics(const Matrix &M, string eye)
{
    if (eye == "leftEye")
    {
        eWL->eye->setHN(M);
    }
    else if (eye == "rightEye")
    {
        eWR->eye->setHN(M);
    }
    else
        return false;

    return true;
}

void vtRFThread::drawTaxel(const yarp::sig::Vector &px, const string &bodypart, ImageOf<PixelRgb> &Im, const int act, const bool flag)
{
    int u = (int)px(0);
    int v = (int)px(1);
    int r = RADIUS;

    if (act>0)
        r+=1;

    if ((u >= r) && (u <= 320 - r) && (v >= r) && (v <= 240 - r))
    {
        for (int x=-r; x<r; x++)
        {
            for (int y=-r; y<r; y++)
            {
                if (bodypart=="left_forearm" || bodypart=="right_forearm" || bodypart=="left_hand" || bodypart=="right_hand")
                {
                    if (act>0)
                    {
                        (Im.pixel(u+x,v+y)).r = 50;
                        (Im.pixel(u+x,v+y)).g = 50+act*205/255;
                        (Im.pixel(u+x,v+y)).b = 100-act*200/255;
                    }
                    else
                    {
                        (Im.pixel(u+x,v+y)).r = 50;
                        (Im.pixel(u+x,v+y)).g = 50;
                        (Im.pixel(u+x,v+y)).b = 100;
                    }
                }
                else if (bodypart=="H0" || bodypart=="HN")
                {
                    (Im.pixel(u+x,v+y)).r = 255;
                    (Im.pixel(u+x,v+y)).g = 125;
                    (Im.pixel(u+x,v+y)).b = 125;
                }
                else if (bodypart=="EER" || bodypart=="EEL")
                {
                    (Im.pixel(u+x,v+y)).r = 125;
                    (Im.pixel(u+x,v+y)).g = 125;
                    (Im.pixel(u+x,v+y)).b = 255;
                }
            }
        }
    }
}

void vtRFThread::handleImages(string eye)
{
    if (eye=="rightEye")
    {
        ImageOf<PixelRgb> &imgOutR= imagePortOutR->prepare();
        
        for (size_t i = 0; i < iCubSkin.size(); i++)
        {
            for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
            {
                drawTaxel(iCubSkin[i].taxel[j].pxRR,iCubSkin[i].name,*imageInR,iCubSkin[i].taxel[j].Resp,false);
            }
        }

        for (size_t i = 0; i < H.size(); i++)
        {
            drawTaxel(H[i].taxel[0].pxRR,H[i].name, *imageInR,H[i].taxel[0].Resp,false);
        }

        imgOutR.copy(*imageInR);
        imagePortOutR->write();
    }
    else if (eye=="leftEye")
    {
        ImageOf<PixelRgb> &imgOutL= imagePortOutL->prepare();
        
        for (size_t i = 0; i < iCubSkin.size(); i++)
        {
            for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
            {
                drawTaxel(iCubSkin[i].taxel[j].pxLL,iCubSkin[i].name,*imageInL,iCubSkin[i].taxel[j].Resp,false);
            }
        }

        for (size_t i = 0; i < H.size(); i++)
        {
            drawTaxel(H[i].taxel[0].pxLL,H[i].name, *imageInL,H[i].taxel[0].Resp,false);
        }

        imgOutL.copy(*imageInL);
        imagePortOutL->write();
    }
}

bool vtRFThread::projectIntoImagePlane(vector <skinPart> &sP, const string &eye, const bool flag)
{
    for (size_t i = 0; i < sP.size(); i++)
    {
        for (size_t j = 0; j < sP[i].taxel.size(); j++)
        {
            if (eye=="rightEye")
                projectPoint(eye,sP[i].taxel[j].WRFPos,sP[i].taxel[j].pxRR,flag);
            else if (eye=="leftEye")
                projectPoint(eye,sP[i].taxel[j].WRFPos,sP[i].taxel[j].pxLL,flag);
            else
                printMessage(0,"ERROR in projectIntoImagePlane!\n");
        }
    }

    return true;
}

yarp::sig::Vector vtRFThread::locateTaxel(const yarp::sig::Vector &_pos, const string &arm)
{
    yarp::sig::Vector pos=_pos;
    yarp::sig::Vector WRFpos(4,0.0);
    Matrix T = eye(4);

    if (arm=="left_forearm" || arm=="left_hand")
    {
        iencsL->getEncoders(encsL->data());
        yarp::sig::Vector qL=encsL->subVector(0,6);
        armL -> setAng(qL*CTRL_DEG2RAD);
    }
    else if (arm=="right_forearm" || arm=="right_hand")
    {
        iencsR->getEncoders(encsR->data());
        yarp::sig::Vector qR=encsR->subVector(0,6);
        armR -> setAng(qR*CTRL_DEG2RAD);
    }
    else
    {
        printMessage(0,"ERROR! locateTaxel() failed!\n");
    }

    if      (arm == "left_forearm" ) { T = armL -> getH(3+4, true); } // torso + up to elbow
    else if (arm == "right_forearm") { T = armR -> getH(3+4, true); } // torso + up to elbow
    else if (arm == "left_hand")     { T = armL -> getH(3+6, true); } // torso + up to wrist
    else if (arm == "right_hand")    { T = armR -> getH(3+6, true); } // torso + up to wrist
    else
    {
        printMessage(0,"ERROR! locateTaxel() failed!\n");
    }

    pos.push_back(1);
    WRFpos = T * pos;
    WRFpos.pop_back();

    return WRFpos;
}

bool vtRFThread::setTaxelPosesFromFile(const string filePath, skinPart &sP)
{
    string line;
    ifstream posFile;
    yarp::sig::Vector taxelPos(3,0.0);
    yarp::sig::Vector taxelNorm(3,0.0);

    // Remove Path (Linux Only)
    sP.name = strrchr(filePath.c_str(), '/');
    sP.name = sP.name.c_str() ? sP.name.c_str() + 1 : filePath.c_str();
    if (sP.name == "left_forearm_mesh.txt"){
        sP.name = "left_forearm";
    }
    else if (sP.name == "left_forearm_nomesh.txt"){
        sP.name = "left_forearm";
    }
    else if (sP.name == "right_forearm_mesh.txt"){
        sP.name = "right_forearm";
    }
    else if (sP.name == "right_forearm_nomesh.txt"){
        sP.name = "right_forearm";
    }
    else if (sP.name == "left_hand_V2_1.txt"){
        sP.name = "left_hand";
    }
    else if (sP.name == "right_hand_V2_1.txt"){
        sP.name = "right_hand";
    }
    else{
        printMessage(0,"ERROR! Unexpected skin part file name: %s.\n",sP.name.c_str());
        return false;
    }
    // Remove "_mesh.txt"
    //sP.name = sP.name.substr(0, sP.name.find_last_of("_"));
       
    // Open File
    posFile.open(filePath.c_str());  
    if (!posFile.is_open())
           return false;

    // Acquire taxels
    posFile.clear(); 
    posFile.seekg(0, std::ios::beg);//rewind iterator
    for(unsigned int i= 0; getline(posFile,line); i++)
    {
        line.erase(line.find_last_not_of(" \n\r\t")+1);
        if(line.empty())
                continue;
        string number;
        istringstream iss(line, istringstream::in);
        for(unsigned int j = 0; iss >> number; j++ )
        {
            if(j<3)
                taxelPos[j]    = strtod(number.c_str(),NULL);
            else
                taxelNorm[j-3] = strtod(number.c_str(),NULL);
        }

        // the NULL taxels will be automatically discarded
        if (norm(taxelNorm) != 0 || norm(taxelPos) != 0)
        {
            sP.size++;
            sP.taxel.push_back(Taxel(taxelPos,taxelNorm,i));
        }
        else
            sP.size++;
    }

    return true;
}

int vtRFThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"*** %s: ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);

        return ret;
    }
    else
        return -1;
}

void vtRFThread::threadRelease()
{
    printMessage(0,"Saving taxels..\n");
        save();
    
    printMessage(0,"Closing controllers..\n");
        ddR.close();
        ddL.close();

    printMessage(0,"Deleting misc stuff..\n");
        delete armR;
        armR = NULL;
        delete armL;
        armL = NULL;

    printMessage(0,"Closing ports..\n");
        closePort(imagePortInR);
        printMessage(1,"imagePortInR successfully closed!\n");
        closePort(imagePortInL);
        printMessage(1,"imagePortInL successfully closed!\n");
        closePort(imagePortOutR);
        printMessage(1,"imagePortOutR successfully closed!\n");
        closePort(imagePortOutL);
        printMessage(1,"imagePortOutL successfully closed!\n");
        closePort(dTPort);
        printMessage(1,"dTPort successfully closed!\n");
        closePort(eventsPort);
        printMessage(1,"eventsPort successfully closed!\n");
        closePort(skinGuiPortForearmL);
        printMessage(1,"skinGuiPortForearmL successfully closed!\n");
        closePort(skinGuiPortForearmR);
        printMessage(1,"skinGuiPortForearmR successfully closed!\n");
        closePort(skinGuiPortHandL);
        printMessage(1,"skinGuiPortHandL successfully closed!\n");
        closePort(skinGuiPortHandR);
        printMessage(1,"skinGuiPortHandR successfully closed!\n");
}

// empty line to make gcc happy
