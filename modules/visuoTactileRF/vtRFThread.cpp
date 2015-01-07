#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iomanip>

#include "vtRFThread.h"

#define RADIUS         2 // radius in px of every taxel (in the images)
#define RFEXTXMIN   -0.1 // lower limit of the receptive field [D]   (-0.1 m)
#define RFEXTXMAX    0.2 // upper limit of the receptive field [D]   (+0.2 m)
#define RFEXTYMIN    0.0 // lower limit of the receptive field [TTC] (+0.0 s)
#define RFEXTYMAX    3.0 // upper limit of the receptive field [TTC] (+3.0 s)
#define RFEXTENSION  0.3 // extension of the receptive field in 1D   (+0.3 m)
#define HAND_LEFT      1
#define FOREARM_LEFT   2
#define HAND_RIGHT     4
#define FOREARM_RIGHT  5
#define SKIN_THRES	   7 // Threshold with which a contact is detected

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

vtRFThread::vtRFThread(int _rate, const string &_name, const string &_robot, const string &_modality,
                       int _v, const ResourceFinder &_moduleRF, vector<string> _fnames,
                       double _hV, const ResourceFinder &_eyeCalibRF) :
                       RateThread(_rate), name(_name), robot(_robot), modality(_modality),
                       verbosity(_v), filenames(_fnames)
{
    //******************* PORTS ******************
        imagePortInR  = new BufferedPort<ImageOf<PixelRgb> >;
        imagePortInL  = new BufferedPort<ImageOf<PixelRgb> >;
        dTPort        = new BufferedPort<Bottle>;
        eventsPort    = new BufferedPort<Bottle>;
        skinPortIn    = new BufferedPort<iCub::skinDynLib::skinContactList>;

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
        if (modality=="1D")
        {
            taxelsFile = rf->check("taxelsFile", Value("taxels1D.ini")).asString().c_str();
        }
        else if (modality=="2D")
        {
            taxelsFile = rf->check("taxelsFile", Value("taxels2D.ini")).asString().c_str();
        }
        printMessage(0,"Storing file set to: %s\n", (path+taxelsFile).c_str());
}

bool vtRFThread::threadInit()
{
    imagePortInR        -> open(("/"+name+"/imageR:i").c_str());
    imagePortInL        -> open(("/"+name+"/imageL:i").c_str());
    imagePortOutR.open(("/"+name+"/imageR:o").c_str());
    imagePortOutL.open(("/"+name+"/imageL:o").c_str());
    dTPort              -> open(("/"+name+"/input:i").c_str());
    eventsPort          -> open(("/"+name+"/events:i").c_str());
    skinGuiPortForearmL.open(("/"+name+"/skinGuiForearmL:o").c_str());
    skinGuiPortForearmR.open(("/"+name+"/skinGuiForearmR:o").c_str());
    skinGuiPortHandL.open(("/"+name+"/skinGuiHandL:o").c_str());
    skinGuiPortHandR.open(("/"+name+"/skinGuiHandR:o").c_str());
    skinPortIn          -> open(("/"+name+"/skin_events:i").c_str());
    skinPortOut.open(("/"+name+"/skin_events:o").c_str());
    dataDumperPortOut.open(("/"+name+"/dataDumper:o").c_str());

    /**
    * I know that I should remove this but it's harmless (and I'm overly lazy)
    **/
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

        ts.update();


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
        iCubSkinSize=filenames.size();
        if (modality=="1D")
        {
            for(unsigned int i=0;i<filenames.size();i++)
            {
                string filePath = filenames[i];
                printMessage(1,"i: %i filePath: %s\n",i,filePath.c_str());
                skinPart1D sP;
                if ( setTaxelPosesFromFile1D(filePath,sP) )
                    iCubSkin1D.push_back(sP);
            }
            load();

            yInfo("iCubSkin1D correctly instantiated. Size: %i",iCubSkin1D.size());
            if (verbosity>= 2)
            {
                for (size_t i = 0; i < iCubSkin1D.size(); i++)
                {
                    iCubSkin1D[i].print();
                }
            }
            iCubSkinSize = iCubSkin1D.size();
        }
        else
        {
            for(unsigned int i=0;i<filenames.size();i++)
            {
                string filePath = filenames[i];
                printMessage(1,"i: %i filePath: %s\n",i,filePath.c_str());
                skinPart2D sP;
                if ( setTaxelPosesFromFile2D(filePath,sP) )
                    iCubSkin2D.push_back(sP);
            }
            load();

            yInfo("iCubSkin2D correctly instantiated. Size: %i\n",iCubSkin2D.size());
            if (verbosity>= 2)
            {
                for (size_t i = 0; i < iCubSkin2D.size(); i++)
                {
                    iCubSkin2D[i].print();
                }
            }
            iCubSkinSize = iCubSkin2D.size();
        }
    return true;
}

void vtRFThread::run()
{
    ts.update();
    incomingEvents.clear();

    // read from the input ports
    dTBottle = dTPort       -> read(false);
    event    = eventsPort   -> read(false);
    imageInR = imagePortInR -> read(false);
    imageInL = imagePortInL -> read(false); 
    iCub::skinDynLib::skinContactList *skinContacts  = skinPortIn -> read(false);

    dumpedVector.resize(0,0.0);

    // project taxels in World Reference Frame
    if (modality=="1D")
    {
        for (size_t i = 0; i < iCubSkin1D.size(); i++)
        {
            for (size_t j = 0; j < iCubSkin1D[i].taxel.size(); j++)
            {
                iCubSkin1D[i].taxel[j].WRFPos=locateTaxel(iCubSkin1D[i].taxel[j].Pos,iCubSkin1D[i].name);
                printMessage(6,"iCubSkin1D[%i].taxel[%i].WRFPos %s\n",i,j,iCubSkin1D[i].taxel[j].WRFPos.toString().c_str());
            }
        }
    }
    else
    {
        for (size_t i = 0; i < iCubSkin2D.size(); i++)
        {
            for (size_t j = 0; j < iCubSkin2D[i].taxel.size(); j++)
            {
                iCubSkin2D[i].taxel[j].WRFPos=locateTaxel(iCubSkin2D[i].taxel[j].Pos,iCubSkin2D[i].name);
                printMessage(6,"iCubSkin2D[%i].taxel[%i].WRFPos %s\n",i,j,iCubSkin2D[i].taxel[j].WRFPos.toString().c_str());
            }
        }
    }

    // process the port coming from the visuoTactileWrapper
    if (event != NULL)
    {
        // read the events
        for (size_t i = 0; i < event->size(); i++)
        {
            incomingEvents.push_back(IncomingEvent(*(event -> get(i).asList())));
            if (verbosity>=3)
            {
                printf("[EVENT]");
                incomingEvents.back().print();
            }   
        }

        // manage the buffer
        timeNow     = yarp::os::Time::now();
        if (eventsFlag)
        {
            eventsFlag  = false;
            yInfo("Starting the buffer..");
        }
        else
        {
            eventsBuffer.push_back(incomingEvents.back());
            yDebug("I'm bufferizing! Size %i",eventsBuffer.size());
        }

        // limit the size of the buffer to 60, i.e. 3 seconds of acquisition
        if (eventsBuffer.size() >= 60)
        {
            eventsBuffer.erase(eventsBuffer.begin());
            yTrace("Too many samples: removing the older element from the buffer..");
        }

        // detect contacts and train the taxels
        if (skinContacts && eventsBuffer.size()>10)
        {
            std::vector<unsigned int> IDv; IDv.clear();
            int IDx = -1;
            if (detectContact(skinContacts, IDx, IDv))
            {
                yInfo("Contact! Training the taxels..");
                timeNow     = yarp::os::Time::now();
                if (learningFlag == true )
                {
                    eventsFlag  = trainTaxels(IDv,IDx);
                }
                else
                {
                    yWarning("No Learning has been put in place.");
                    size_t txlsize = modality=="1D"?iCubSkin1D[0].taxel.size():iCubSkin2D[0].taxel.size();
                    for (size_t j = 0; j < txlsize; j++)
                    {
                        dumpedVector.push_back(0.0);
                    }
                }
                eventsBuffer.clear();
            }
            else
            {
                size_t txlsize = modality=="1D"?iCubSkin1D[0].taxel.size():iCubSkin2D[0].taxel.size();
                for (size_t j = 0; j < txlsize; j++)
                {
                    dumpedVector.push_back(0.0);
                }
            }
        }
        else
        {
            size_t txlsize = modality=="1D"?iCubSkin1D[0].taxel.size():iCubSkin2D[0].taxel.size();
            for (size_t j = 0; j < txlsize; j++)
            {
                dumpedVector.push_back(0.0);
            }
        }
    }
    // if there's no input for more than 2 seconds, clear the buffer
    else if (yarp::os::Time::now() - timeNow > 2.0)
    {
        eventsFlag = true;
        eventsBuffer.clear();
        timeNow = yarp::os::Time::now();
        yInfo("No significant event in the last 2 seconds. Erasing the buffer..");
        dumpedVector.push_back(2.0);
    }
    else
    {
        size_t txlsize = modality=="1D"?iCubSkin1D[0].taxel.size():iCubSkin2D[0].taxel.size();
        for (size_t j = 0; j < txlsize; j++)
        {
            dumpedVector.push_back(0.0);
        }
    }
    
    // Superimpose the taxels onto the right eye
    if (imageInR!=NULL)
        drawTaxels("rightEye");

    // Superimpose the taxels onto the left eye
    if (imageInL!=NULL)
        drawTaxels("leftEye");
    
    if (incomingEvents.size()>0)
    {
        projectIncomingEvent();     // project event onto the taxels' RF
        computeResponse();          // compute the response of each taxel
    }
 
    sendContactsToSkinGui();        // self explicative
    manageSkinEvents();

    // manage the dumped port
    if (dumpedVector.size()>0)
    {
        Bottle bd;
        bd.clear();
        
        vectorIntoBottle(dumpedVector,bd);
        dataDumperPortOut.setEnvelope(ts);
        dataDumperPortOut.write(bd);
    }
}

void vtRFThread::manageSkinEvents()
{
    // main/src/modules/skinManager/src/compensationThread.cpp:250
    vector <int> taxelsIDs; 
    string part = "";
    int iCubSkinID=-1;
    bool isThereAnEvent = false;

    if (incomingEvents.size()>0)  // if there's an event
    {
        for (size_t i = 0; i < iCubSkinSize; i++) // cycle through the skinparts
        {
            if (!isThereAnEvent)       // process only one contact at a time
            {
                if (modality=="1D")
                {
                    for (size_t j = 0; j < iCubSkin1D[i].taxel.size(); j++) // cycle through the taxels
                    {
                        if (iCubSkin1D[i].taxel[j].Resp > 100)
                        {
                            taxelsIDs.push_back(iCubSkin1D[i].taxel[j].ID);
                            isThereAnEvent = true;
                        }
                    }
                    if (isThereAnEvent)
                    {
                        part   = iCubSkin1D[i].name;
                        iCubSkinID = i;
                    }
                    else
                        taxelsIDs.clear();
                }
                else
                {
                    for (size_t j = 0; j < iCubSkin2D[i].taxel.size(); j++) // cycle through the taxels
                    {
                        if (iCubSkin2D[i].taxel[j].Resp > 100)
                        {
                            taxelsIDs.push_back(iCubSkin2D[i].taxel[j].ID);
                            isThereAnEvent = true;
                        }
                    }
                    if (isThereAnEvent)
                    {
                        part   = iCubSkin2D[i].name;
                        iCubSkinID = i;
                    }
                    else
                        taxelsIDs.clear();
                }
            }
        }
    }

    if (isThereAnEvent && taxelsIDs.size()>0)
    {
        Vector geoCenter(3,0.0), normalDir(3,0.0);
        int w = 0, w_sum = 0;

        Bottle b;
        b.clear();

        if (part == "left_forearm" || part == "left_hand")
        {
            b.addString("left");
        }
        else if (part == "right_forearm" || part == "right_hand")
        {
            b.addString("right");
        }

        for (size_t i = 0; i < taxelsIDs.size(); ++i)
        {
            if (modality=="1D")
            {
                for (size_t j = 0; j < iCubSkin1D[iCubSkinID].taxel.size(); j++)
                {
                    if (iCubSkin1D[iCubSkinID].taxel[j].ID == taxelsIDs[i])
                    {
                        w = iCubSkin1D[iCubSkinID].taxel[j].Resp;
                        geoCenter += iCubSkin1D[iCubSkinID].taxel[j].WRFPos*w;
                        normalDir += locateTaxel(iCubSkin1D[iCubSkinID].taxel[j].Norm,part)*w;
                        w_sum += w;
                    }
                }
            }
            else
            {
                for (size_t j = 0; j < iCubSkin2D[iCubSkinID].taxel.size(); j++)
                {
                    if (iCubSkin2D[iCubSkinID].taxel[j].ID == taxelsIDs[i])
                    {
                        w = iCubSkin2D[iCubSkinID].taxel[j].Resp;
                        geoCenter += iCubSkin2D[iCubSkinID].taxel[j].WRFPos*w;
                        normalDir += locateTaxel(iCubSkin2D[iCubSkinID].taxel[j].Norm,part)*w;
                        w_sum += w;
                    }
                }
            }
        }

        geoCenter /= w_sum;
        normalDir /= w_sum;
        vectorIntoBottle(geoCenter,b);
        vectorIntoBottle(normalDir,b);
        skinPortOut.setEnvelope(ts);
        skinPortOut.write(b);     // send something anyway (if there is no contact the bottle is empty)
    }
}

void vtRFThread::sendContactsToSkinGui()
{
    Vector respToSkin;

    for(size_t i=0; i<iCubSkinSize; i++)
    {
        string iCubSkinName="";

        if (modality=="1D")
        {
            respToSkin.resize(iCubSkin1D[i].size,0.0);   // resize the vector to the skinPart
            iCubSkinName=iCubSkin1D[i].name;
        }
        else
        {
            respToSkin.resize(iCubSkin2D[i].size,0.0);   // resize the vector to the skinPart
            iCubSkinName=iCubSkin2D[i].name;
        }

        if (incomingEvents.size()>0)
        {
            if (modality=="1D")
            {
                for (size_t j = 0; j < iCubSkin1D[i].taxel.size(); j++)
                {
                    respToSkin[iCubSkin1D[i].taxel[j].ID] = iCubSkin1D[i].taxel[j].Resp*100/255;
                }
            }
            else
            {
                for (size_t j = 0; j < iCubSkin2D[i].taxel.size(); j++)
                {
                    if(iCubSkin2D[i].Repr2TaxelList.empty())
                    {  
                        //we simply light up the taxels themselves
                        respToSkin[iCubSkin2D[i].taxel[j].ID] = iCubSkin2D[i].taxel[j].Resp;
                    }
                    else
                    { 
                        //we light up all the taxels represented by the particular taxel
                        list<unsigned int> l = iCubSkin2D[i].Repr2TaxelList[iCubSkin2D[i].taxel[j].ID];

                        if (l.empty())
                        {
                            yWarning("skinPart %d Taxel %d : no list of represented taxels is available, even if Repr2TaxelList is not empty",i,iCubSkin2D[i].taxel[j].ID);
                            respToSkin[iCubSkin2D[i].taxel[j].ID] = iCubSkin2D[i].taxel[j].Resp;
                        }
                        else
                        {
                            for(list<unsigned int>::const_iterator iter_list = l.begin(); iter_list != l.end(); iter_list++)
                            {
                                //for all the represented taxels, we assign the activation of the super-taxel
                                respToSkin[*iter_list] =  iCubSkin2D[i].taxel[j].Resp;
                            } 
                        }
                    }
                }
            }
        }
        
        if(iCubSkinName == "left_forearm")
        {
            skinGuiPortForearmL.setEnvelope(ts);
            skinGuiPortForearmL.write(respToSkin); 
        }
        else if(iCubSkinName == "right_forearm")
        {
            skinGuiPortForearmR.setEnvelope(ts);
            skinGuiPortForearmR.write(respToSkin); 
        }
        else if(iCubSkinName == "left_hand")
        {
            skinGuiPortHandL.setEnvelope(ts);
            skinGuiPortHandL.write(respToSkin); 
        }
        else if(iCubSkinName == "right_hand")
        {
            skinGuiPortHandR.setEnvelope(ts);
            skinGuiPortHandR.write(respToSkin); 
        }
    }
}

bool vtRFThread::detectContact(iCub::skinDynLib::skinContactList *_sCL, int &idx,
                               std::vector <unsigned int> &v)
{
    // Search for a suitable contact:
    for(iCub::skinDynLib::skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++)
    {
        if( it -> getPressure() > SKIN_THRES && (it -> getTaxelList()).size() > 2 )
        {
            for (size_t i = 0; i < iCubSkinSize; i++)
            {
                string iCubSkinName="";
                if (modality=="1D")
                    iCubSkinName=iCubSkin1D[i].name;
                else
                    iCubSkinName=iCubSkin2D[i].name;

                if ((it -> getSkinPart() == FOREARM_RIGHT && iCubSkinName == "right_forearm") ||
                    (it -> getSkinPart() ==  FOREARM_LEFT && iCubSkinName == "left_forearm" ) ||
                    (it -> getSkinPart() ==    HAND_RIGHT && iCubSkinName == "right_hand"   ) ||
                    (it -> getSkinPart() ==     HAND_LEFT && iCubSkinName == "left_hand"    )    )
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
    rf->setVerbose(true);
    string fileName=rf->findFile("taxelsFile").c_str();
    rf->setVerbose(false);
    if (fileName=="")
    {
        yWarning("[vtRF::load] No filename has been found. Skipping..");
        return false;
    }

    yInfo("File loaded: %s", fileName.c_str());
    Property data; data.fromConfigFile(fileName.c_str());
    Bottle b; b.read(data);
    yDebug("iCubSkinSize %i",iCubSkinSize);

    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        if (modality=="1D")
        {
            Bottle bb = b.findGroup(iCubSkin1D[i].name.c_str());

            if (bb.size() > 0)
            {
                Bottle *bbb;
                int nTaxels = bb.find("nTaxels").asInt();
                int size    = bb.find("size").asInt();
                double ext  = bb.find("ext").asDouble();
                int bNum    = bb.find("binsNum").asInt();

                printMessage(0,"size %i\tnTaxels %i\text %g\tbinsNum %i\n",size,nTaxels,ext,bNum);

                iCubSkin1D[i].size = size;

                for (size_t j = 0; j < nTaxels; j++)
                {
                    bbb = bb.get(j+5).asList();
                    yDebug("Reading taxel %s",bbb->toString().c_str());
                    iCubSkin1D[i].taxel[j].ID = bbb->get(0).asInt();
                    iCubSkin1D[i].taxel[j].pwe.resize(ext,bNum);
                    
                    iCubSkin1D[i].taxel[j].pwe.setHist(vectorFromBottle(*bbb->get(1).asList(),0,bNum));
                }
                printf("Debug\n");
            }
        }
        else
        {
            Bottle bb = b.findGroup(iCubSkin2D[i].name.c_str());

            if (bb.size() > 0)
            {
                int nTaxels = bb.find("nTaxels").asInt();
                int size    = bb.find("size").asInt();
                std::vector<double> extX;
                std::vector<double> extY;
                std::vector<int>    bNum;
                std::vector<int>    mapp;

                Bottle *bbb;
                bbb = bb.find("extX").asList();
                extX.push_back(bbb->get(0).asDouble());
                extX.push_back(bbb->get(1).asDouble());
                bbb = bb.find("extY").asList();
                extY.push_back(bbb->get(0).asDouble());
                extY.push_back(bbb->get(1).asDouble());
                bbb = bb.find("binsNum").asList();
                bNum.push_back(bbb->get(0).asInt());
                bNum.push_back(bbb->get(1).asInt());
                bbb = bb.find("Mapping").asList();

                printMessage(6,"size %i\tnTaxels %i\textX %g  %g\n",size,nTaxels,extX[0],extX[1]);
                printMessage(6,"extY %g  %g\tbNum %i  %i\t\n",extY[0],extY[1],bNum[0],bNum[1]);
                printMessage(6,"mapp\n");
                for (size_t j = 0; j < size; j++)
                {
                    mapp.push_back(bbb->get(j).asInt());
                    if (verbosity>=6)
                    {
                        printf("%i ",mapp[j]);
                    }
                }
                printMessage(6,"\n");
                iCubSkin2D[i].size = size;
                iCubSkin2D[i].Taxel2Repr = mapp;

                for (size_t j = 0; j < nTaxels; j++)
                {
                    bbb = bb.get(j+7).asList();
                    iCubSkin2D[i].taxel[j].ID = bbb->get(0).asInt();
                    iCubSkin2D[i].taxel[j].pwe.resize(extX,extY,bNum);
                    
                    iCubSkin2D[i].taxel[j].pwe.setPosHist(matrixFromBottle(*bbb->get(1).asList(),0,bNum[0],bNum[1]));
                    iCubSkin2D[i].taxel[j].pwe.setNegHist(matrixFromBottle(*bbb->get(2).asList(),0,bNum[0],bNum[1]));
                }
            }
        }
    }
    printf("Debug\n");
    return true;
}

bool vtRFThread::save()
{
    string fnm=path+taxelsFile;
    ofstream myfile;
    myfile.open(fnm.c_str(),ios::trunc);

    if (myfile.is_open())
    {
        for (size_t i = 0; i < iCubSkinSize; i++)
        {
            if (modality=="1D")
            {
                double ext  = iCubSkin1D[i].taxel[0].pwe.getExt();
                int    bNum = iCubSkin1D[i].taxel[0].pwe.getHistSize();

                myfile << "[" << iCubSkin1D[i].name << "]" << endl;
                myfile << "size\t"    << iCubSkin1D[i].size << endl;    
                myfile << "nTaxels\t" << iCubSkin1D[i].taxel.size() << endl;
                myfile << "ext \t"    << ext << endl;
                myfile << "binsNum\t" << bNum << endl;

                for (size_t j = 0; j < iCubSkin1D[i].taxel.size(); j++)
                {
                    Bottle data;
                    data.clear();
                    Bottle &valuesPos = data.addList();
                    int bNum = iCubSkin1D[i].taxel[j].pwe.getHistSize();

                    for (size_t k = 0; k < bNum; k++)
                    {
                        valuesPos.addInt(iCubSkin1D[i].taxel[j].pwe.getHist(k));
                    }
                    myfile << iCubSkin1D[i].taxel[j].ID << "\t\t" << data.toString() << endl;
                }
            }
            else
            {
                std::vector<double> extX = iCubSkin2D[i].taxel[0].pwe.getExtX();
                std::vector<double> extY = iCubSkin2D[i].taxel[0].pwe.getExtY();
                std::vector<int>    bNum = iCubSkin2D[i].taxel[0].pwe.getHistSize();

                myfile << "[" << iCubSkin2D[i].name << "]" << endl;
                myfile << "size\t"    << iCubSkin2D[i].size << endl;    
                myfile << "nTaxels\t" << iCubSkin2D[i].taxel.size() << endl;
                myfile << "extX\t( "  << extX[0] << "\t" << extX[1] << " )\n";
                myfile << "extY\t( "  << extY[0] << "\t" << extY[1] << " )\n";
                myfile << "binsNum\t( " << bNum[0] << "\t" << bNum[1] << " )\n";

                Bottle data;
                data.clear();
                Bottle &representatives = data.addList();
                for (size_t q = 0; q < iCubSkin2D[i].Taxel2Repr.size(); q++)
                {
                    representatives.addInt(iCubSkin2D[i].Taxel2Repr[q]);
                } 
                myfile << "Mapping\t" << data.toString() << endl;

                for (size_t j = 0; j < iCubSkin2D[i].taxel.size(); j++)
                {
                    data.clear();
                    Bottle &valuesPos = data.addList();
                    std::vector<int>    bNum = iCubSkin2D[i].taxel[j].pwe.getHistSize();

                    for (size_t k = 0; k < bNum[0]; k++)
                    {
                        for (size_t kk = 0; kk < bNum[1]; kk++)
                        {
                            valuesPos.addInt(iCubSkin2D[i].taxel[j].pwe.getPosHist(k,kk));
                        }
                    }
                    myfile << iCubSkin2D[i].taxel[j].ID << "\t\t" << data.toString() << "\t";

                    data.clear();
                    Bottle &valuesNeg = data.addList();

                    for (size_t k = 0; k < bNum[0]; k++)
                    {
                        for (size_t kk = 0; kk < bNum[1]; kk++)
                        {
                            valuesNeg.addInt(iCubSkin2D[i].taxel[j].pwe.getNegHist(k,kk));
                        }
                    }
                    myfile << data.toString() << endl;
                }
            }
        }
    }
    myfile.close();
    return true;
}

bool vtRFThread::trainTaxels(const std::vector<unsigned int> IDv, const int IDx)
{
    std::vector<unsigned int> v = IDv;
    string iCubSkinName = "";
    if (modality=="1D")
    {
        iCubSkinName=iCubSkin1D[IDx].name;
    }
    else
    {
        iCubSkinName=iCubSkin2D[IDx].name;
    }

    Matrix T_a = eye(4);                     // transform matrix relative to the arm
    if ((iCubSkinName == "left_forearm") || (iCubSkinName == "left_hand"))
    {
        iencsL->getEncoders(encsL->data());
        yarp::sig::Vector qL=encsL->subVector(0,6);
        armL -> setAng(qL*CTRL_DEG2RAD);
        if (iCubSkinName == "left_forearm") 
            T_a = armL -> getH(3+4, true);
        else //(iCubSkin[i].name == "left_hand") 
            T_a = armL -> getH(3+6, true);
    }
    else if ((iCubSkinName == "right_forearm") || (iCubSkinName == "right_hand"))
    {
        iencsR->getEncoders(encsR->data());
        yarp::sig::Vector qR=encsR->subVector(0,6);
        armR -> setAng(qR*CTRL_DEG2RAD);
        if (iCubSkinName == "right_forearm")
            T_a = armR -> getH(3+4, true);
        else //(iCubSkin[i].name == "right_hand") 
            T_a = armR -> getH(3+6, true);
    }
    else
    {
        printMessage(0,"ERROR in trainTaxels!\n");
        return false;
    }

    if (modality=="1D")
    {
        for (size_t j = 0; j < iCubSkin1D[IDx].taxel.size(); j++)
        {
            bool itHasBeenTouched = false;
            for (size_t w = 0; w < v.size(); w++)
            {
                if (iCubSkin1D[IDx].taxel[j].ID == v[w])
                {
                    itHasBeenTouched = true;
                }
            }

            for (size_t k = 0; k < eventsBuffer.size(); k++)
            {
                IncomingEvent projection = projectIntoTaxelRF(iCubSkin1D[IDx].taxel[j].RF,T_a,eventsBuffer[k]);

                if (itHasBeenTouched)
                {
                    printMessage(4,"Training Taxels: ID %i k %i NORM %g\n",iCubSkin1D[IDx].taxel[j].ID,k,norm(projection.Pos));
                    iCubSkin1D[IDx].taxel[j].pwe.addSample(norm(projection.Pos));
                }
                // a negative sample is added only if
                // 1. the z is positive
                // 2. the x and the y are between -75% and +75% of the RF's extension (i.e. 15cm)
                else if (!itHasBeenTouched && projection.Pos[2]>=0 && 
                          projection.Pos[0] >= - iCubSkin1D[IDx].taxel[j].pwe.getExt() * 75 /100 &&
                          projection.Pos[0] <=   iCubSkin1D[IDx].taxel[j].pwe.getExt() * 75 /100 &&
                          projection.Pos[1] >= - iCubSkin1D[IDx].taxel[j].pwe.getExt() * 75 /100 &&
                          projection.Pos[1] <=   iCubSkin1D[IDx].taxel[j].pwe.getExt() * 75 /100 )
                {
                    iCubSkin1D[IDx].taxel[j].pwe.removeSample(norm(projection.Pos));
                }
            }

            if (itHasBeenTouched == true)   dumpedVector.push_back(1.0);
            else                            dumpedVector.push_back(-1.0);
        }
    }
    else
    {
        for (size_t j = 0; j < iCubSkin2D[IDx].taxel.size(); j++)
        {
            bool itHasBeenTouched = false;
            for (size_t w = 0; w < v.size(); w++)
            {
                if (iCubSkin2D[IDx].taxel[j].ID == v[w])
                {
                    itHasBeenTouched = true;
                }
            }

            for (size_t k = 0; k < eventsBuffer.size(); k++)
            {
                IncomingEvent4Taxel2D projection = projectIntoTaxelRF(iCubSkin2D[IDx].taxel[j].RF,T_a,eventsBuffer[k]);
                printMessage(4,"Training Taxels: skinPart %d ID %i k %i NORM %g TTC %g\n",IDx,iCubSkin2D[IDx].taxel[j].ID,k,projection.NRM,projection.TTC);

                if (itHasBeenTouched == true)
                {
                    iCubSkin2D[IDx].taxel[j].addSample(projection);
                }
                else
                {
                    iCubSkin2D[IDx].taxel[j].removeSample(projection);
                }
            }

            if (itHasBeenTouched == true)   dumpedVector.push_back(1.0);
            else                            dumpedVector.push_back(-1.0);
        }
    }

    return true;
}

bool vtRFThread::projectIncomingEvent()
{
    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        string iCubSkinName="";
        if (modality=="1D")
            iCubSkinName=iCubSkin1D[i].name;
        else
            iCubSkinName=iCubSkin2D[i].name;

        Matrix T_a = eye(4);               // transform matrix relative to the arm
        if ((iCubSkinName == "left_forearm") || (iCubSkinName == "left_hand"))
        {
            iencsL->getEncoders(encsL->data());
            yarp::sig::Vector qL=encsL->subVector(0,6);
            armL -> setAng(qL*CTRL_DEG2RAD);
            if (iCubSkinName == "left_forearm") 
                T_a = armL -> getH(3+4, true);
            else //(iCubSkinName == "left_hand") 
                T_a = armL -> getH(3+6, true);
        }
        else if ((iCubSkinName == "right_forearm") || (iCubSkinName == "right_hand"))
        {
            iencsR->getEncoders(encsR->data());
            yarp::sig::Vector qR=encsR->subVector(0,6);
            armR -> setAng(qR*CTRL_DEG2RAD);
            if (iCubSkinName == "right_forearm")
                T_a = armR -> getH(3+4, true);
            else //(iCubSkinName == "right_hand") 
                T_a = armR -> getH(3+6, true);
        }
        else
            printMessage(0,"ERROR in projectIncomingEvent!\n");

        if (modality=="1D")
        {
            for (size_t j = 0; j < iCubSkin1D[i].taxel.size(); j++)
            {
                iCubSkin1D[i].taxel[j].Evnt=projectIntoTaxelRF(iCubSkin1D[i].taxel[j].RF,T_a,
                                                             incomingEvents[incomingEvents.size()-1]);

                // There's a reason behind this choice
                dumpedVector.push_back(iCubSkin1D[i].taxel[j].Evnt.Pos[0]);
                dumpedVector.push_back(iCubSkin1D[i].taxel[j].Evnt.Pos[1]);
                dumpedVector.push_back(iCubSkin1D[i].taxel[j].Evnt.Pos[2]);

                // if (j==100)
                // {
                    printMessage(4,"Projection -> i: %i\tID %i\tEvent: ",i,j);
                    if (verbosity>=4)
                        iCubSkin1D[i].taxel[j].Evnt.print();
                // }
            }
        }
        else
        {
            for (size_t j = 0; j < iCubSkin2D[i].taxel.size(); j++)
            {
                iCubSkin2D[i].taxel[j].Evnt=projectIntoTaxelRF(iCubSkin2D[i].taxel[j].RF,T_a,
                                                             incomingEvents[incomingEvents.size()-1]);

                // There's a reason behind this choice
                dumpedVector.push_back(iCubSkin2D[i].taxel[j].Evnt.Pos[0]);
                dumpedVector.push_back(iCubSkin2D[i].taxel[j].Evnt.Pos[1]);
                dumpedVector.push_back(iCubSkin2D[i].taxel[j].Evnt.Pos[2]);

                // if (j==100)
                // {
                    printMessage(4,"Projection -> i: %i\tID %i\tEvent: ",i,j);
                    if (verbosity>=4)
                        iCubSkin2D[i].taxel[j].Evnt.print();
                // }
            }
        }
    }
    return true;
}

IncomingEvent4Taxel2D vtRFThread::projectIntoTaxelRF(const Matrix &RF,const Matrix &T_a,const IncomingEvent &e)
{
    IncomingEvent4Taxel2D Event_projected = e;

    Matrix T_a_proj = T_a * RF;

    Vector p=e.Pos; p.push_back(1);
    Vector v=e.Vel; v.push_back(1);

    Event_projected.Pos = SE3inv(T_a_proj)*p;        Event_projected.Pos.pop_back();
    Event_projected.Vel = SE3inv(T_a_proj)*v;        Event_projected.Vel.pop_back();

    if (e.Radius != -1.0)
    {
        Event_projected.Pos(2) -= Event_projected.Radius;
    }
    if (modality=="2D")
    {
        computeX(Event_projected);
    }

    return Event_projected;
}

bool vtRFThread::computeX(IncomingEvent4Taxel2D &ie)
{
    int sgn = ie.Pos[2]>=0?1:-1;
    ie.NRM = sgn * norm(ie.Pos);
    // printf("ie.Vel %g\n", norm(ie.Vel));

    // if (norm(ie.Vel) < 0.38 && norm(ie.Vel) > 0.34)
    // {
    //     ie.TTC = 10000.0;
    // }
    // else
    {
        ie.TTC = -norm(ie.Pos)*norm(ie.Pos)/dot(ie.Pos,ie.Vel);
    }

    return true;
}

void vtRFThread::resetParzenWindows()
{
    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        if (modality=="1D")
        {
            for (size_t j = 0; j < iCubSkin1D[i].taxel.size(); j++)
            {
                iCubSkin1D[i].taxel[j].resetParzenWindow();
            }
        }
        else
        {
            for (size_t j = 0; j < iCubSkin2D[i].taxel.size(); j++)
            {
                iCubSkin2D[i].taxel[j].resetParzenWindow();
            }
        }
    }
}

bool vtRFThread::computeResponse()
{
    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        if (modality=="1D")
        {
            for (size_t j = 0; j < iCubSkin1D[i].taxel.size(); j++)
            {
                iCubSkin1D[i].taxel[j].computeResponse();
            }
        }
        else
        {
            for (size_t j = 0; j < iCubSkin2D[i].taxel.size(); j++)
            {
                iCubSkin2D[i].taxel[j].computeResponse();
            }
        }
    }

    return true;
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

void vtRFThread::drawTaxels(string _eye)
{
    if (modality=="1D")    projectIntoImagePlane(iCubSkin1D,_eye);
    else                   projectIntoImagePlane(iCubSkin2D,_eye);

    ImageOf<PixelRgb> imgOut;

    if (_eye=="rightEye")
    {
        imgOut.copy(*imageInR);
    }
    else if (_eye=="leftEye")
    {
        imgOut.copy(*imageInL);
    }
    else
    {
        yError("Error in drawTaxels! Returning..");
        return;
    }

    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        if (modality=="1D")
        {
            for (size_t j = 0; j < iCubSkin1D[i].taxel.size(); j++)
            {
                drawTaxel(imgOut,iCubSkin1D[i].taxel[j].px,iCubSkin1D[i].name,iCubSkin1D[i].taxel[j].Resp);
                printMessage(6,"iCubSkin1D[%i].taxel[%i].px %s\n",i,j,iCubSkin1D[i].taxel[j].px.toString().c_str());
            }
        }
        else
        {
            for (size_t j = 0; j < iCubSkin2D[i].taxel.size(); j++)
            {
                drawTaxel(imgOut,iCubSkin2D[i].taxel[j].px,iCubSkin2D[i].name,iCubSkin2D[i].taxel[j].Resp);
                printMessage(6,"iCubSkin2D[%i].taxel[%i].px %s\n",i,j,iCubSkin2D[i].taxel[j].px.toString().c_str());
            }
        }
    }

    _eye=="rightEye"?imagePortOutR.setEnvelope(ts):imagePortOutL.setEnvelope(ts);
    _eye=="rightEye"?imagePortOutR.write(imgOut):imagePortOutL.write(imgOut);
}


void vtRFThread::drawTaxel(ImageOf<PixelRgb> &Im, const yarp::sig::Vector &px,
                           const string &part, const int act)
{
    int u = (int)px(0);
    int v = (int)px(1);
    int r = RADIUS;

    if (act>0)
        r+=1;

    if ((u >= r) && (u <= 320 - r) && (v >= r) && (v <= 240 - r))
    {
        for (size_t x=0; x<2*r; x++)
        {
            for (size_t y=0; y<2*r; y++)
            {
                if (part=="left_forearm" || part=="right_forearm" ||
                    part=="left_hand" || part=="right_hand")
                {
                    if (act>0)
                    {
                        (Im.pixel(u+x-r,v+y-r)).r = 50;
                        (Im.pixel(u+x-r,v+y-r)).g = 50+act*205/255;
                        (Im.pixel(u+x-r,v+y-r)).b = 100-act*200/255;
                    }
                    else
                    {
                        (Im.pixel(u+x-r,v+y-r)).r = 50;
                        (Im.pixel(u+x-r,v+y-r)).g = 50;
                        (Im.pixel(u+x-r,v+y-r)).b = 100;
                    }
                }
                else if (part=="H0" || part=="HN")
                {
                    (Im.pixel(u+x-r,v+y-r)).r = 255;
                    (Im.pixel(u+x-r,v+y-r)).g = 125;
                    (Im.pixel(u+x-r,v+y-r)).b = 125;
                }
                else if (part=="EER" || part=="EEL")
                {
                    (Im.pixel(u+x-r,v+y-r)).r = 125;
                    (Im.pixel(u+x-r,v+y-r)).g = 125;
                    (Im.pixel(u+x-r,v+y-r)).b = 255;
                }
            }
        }
    }
}


bool vtRFThread::projectIntoImagePlane(vector <skinPart1D> &sP, const string &eye)
{
    for (size_t i = 0; i < sP.size(); i++)
    {
        for (size_t j = 0; j < sP[i].taxel.size(); j++)
        {
            if (eye=="rightEye" || eye=="leftEye")
                projectPoint(sP[i].taxel[j].WRFPos,sP[i].taxel[j].px,eye);
            else
                yError("ERROR in projectIntoImagePlane!\n");
        }
    }

    return true;
}

bool vtRFThread::projectIntoImagePlane(vector <skinPart2D> &sP, const string &eye)
{
    for (size_t i = 0; i < sP.size(); i++)
    {
        for (size_t j = 0; j < sP[i].taxel.size(); j++)
        {
            if (eye=="rightEye" || eye=="leftEye")
                projectPoint(sP[i].taxel[j].WRFPos,sP[i].taxel[j].px,eye);
            else
                yError("ERROR in projectIntoImagePlane!\n");
        }
    }

    return true;
}


bool vtRFThread::projectPoint(const yarp::sig::Vector &x,
                              yarp::sig::Vector &px, const string &_eye)
{
    if (x.length()<3)
    {
        fprintf(stdout,"Not enough values given for the point!\n");
        return false;
    }

    bool isLeft=(_eye=="leftEye");

    yarp::sig::Matrix  *Prj=(isLeft?eWL->Prj:eWR->Prj);
    iCubEye            *eye=(isLeft?eWL->eye:eWR->eye);

    if (Prj)
    {
        iencsT->getEncoders(encsT->data());
        yarp::sig::Vector torso=*encsT;
        iencsH->getEncoders(encsH->data());
        yarp::sig::Vector  head=*encsH;

        yarp::sig::Vector q(8);
        q[0]=torso[2];       q[1]=torso[1];        q[2]=torso[0];
        q[3]=head[0];        q[4]=head[1];
        q[5]=head[2];        q[6]=head[3];
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
        xe=SE3inv(eye->getH())*xo;

        // find the 2D projection
        px=*Prj*xe;
        px=px/px[2];
        px.pop_back();

        return true;
    }
    else
    {
        fprintf(stdout,"Unspecified projection matrix for %s camera!\n",_eye.c_str());
        return false;
    }
}

yarp::sig::Vector vtRFThread::locateTaxel(const yarp::sig::Vector &_pos, const string &part)
{
    yarp::sig::Vector pos=_pos;
    yarp::sig::Vector WRFpos(4,0.0);
    Matrix T = eye(4);

    if (part=="left_forearm" || part=="left_hand")
    {
        iencsL->getEncoders(encsL->data());
        yarp::sig::Vector qL=encsL->subVector(0,6);
        armL -> setAng(qL*CTRL_DEG2RAD);
    }
    else if (part=="right_forearm" || part=="right_hand")
    {
        iencsR->getEncoders(encsR->data());
        yarp::sig::Vector qR=encsR->subVector(0,6);
        armR -> setAng(qR*CTRL_DEG2RAD);
    }
    else
    {
        printMessage(0,"ERROR! locateTaxel() failed!\n");
    }

    if      (part == "left_forearm" ) { T = armL -> getH(3+4, true); } // torso + up to elbow
    else if (part == "right_forearm") { T = armR -> getH(3+4, true); } // torso + up to elbow
    else if (part == "left_hand")     { T = armL -> getH(3+6, true); } // torso + up to wrist
    else if (part == "right_hand")    { T = armR -> getH(3+6, true); } // torso + up to wrist
    else    {  printMessage(0,"ERROR! locateTaxel() failed!\n"); }

    pos.push_back(1);
    WRFpos = T * pos;
    WRFpos.pop_back();

    return WRFpos;
}

//see also Compensator::setTaxelPosesFromFile in icub-main/src/modules/skinManager/src/compensator.cpp
bool vtRFThread::setTaxelPosesFromFile1D(const string filePath, skinPart1D &sP)
{
    string line;
    ifstream posFile;
    yarp::sig::Vector taxelPos(3,0.0);
    yarp::sig::Vector taxelNorm(3,0.0);

    // Remove Path (Linux Only)
    sP.name = strrchr(filePath.c_str(), '/');
    sP.name = sP.name.c_str() ? sP.name.c_str() + 1 : filePath.c_str();

    // Remove "_mesh.txt"
    if      (sP.name == "left_forearm_mesh.txt")    { sP.name = "left_forearm"; }
    else if (sP.name == "left_forearm_nomesh.txt")  { sP.name = "left_forearm"; }
    else if (sP.name == "right_forearm_mesh.txt")   { sP.name = "right_forearm"; }
    else if (sP.name == "right_forearm_nomesh.txt") { sP.name = "right_forearm"; }
    else if (sP.name == "left_hand_V2_1.txt")       { sP.name = "left_hand"; }
    else if (sP.name == "right_hand_V2_1.txt")      { sP.name = "right_hand"; }
    else
    {
        printMessage(0,"ERROR! Unexpected skin part file name: %s.\n",sP.name.c_str());
        return false;
    }
    //sP.name = sP.name.substr(0, sP.name.find_last_of("_"));
       
    // Open File
    posFile.open(filePath.c_str());  
    if (!posFile.is_open())
        return false;

    // Acquire taxels (different for 1D and 2D only because the 2D case cannot handle all of the taxels,
    // so a subset of them [i.e. the representative taxels] has been used)
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

        // the NULL taxels will be automatically discarded - most skin patches are not full and padded with 0s
        if (norm(taxelNorm) != 0 || norm(taxelPos) != 0)
        {
            sP.size++;
            sP.taxel.push_back(Taxel1D(taxelPos,taxelNorm,i));
        }
        else
        {
            sP.size++;
        }
    }

    return true;
}

//see also Compensator::setTaxelPosesFromFile in icub-main/src/modules/skinManager/src/compensator.cpp
bool vtRFThread::setTaxelPosesFromFile2D(const string filePath, skinPart2D &sP)
{
    string line;
    ifstream posFile;
    yarp::sig::Vector taxelPos(3,0.0);
    yarp::sig::Vector taxelNorm(3,0.0);

    // Remove Path (Linux Only)
    sP.name = strrchr(filePath.c_str(), '/');
    sP.name = sP.name.c_str() ? sP.name.c_str() + 1 : filePath.c_str();

    // Remove "_mesh.txt"
    if      (sP.name == "left_forearm_mesh.txt")    { sP.name = "left_forearm"; }
    else if (sP.name == "left_forearm_nomesh.txt")  { sP.name = "left_forearm"; }
    else if (sP.name == "right_forearm_mesh.txt")   { sP.name = "right_forearm"; }
    else if (sP.name == "right_forearm_nomesh.txt") { sP.name = "right_forearm"; }
    else if (sP.name == "left_hand_V2_1.txt")       { sP.name = "left_hand"; }
    else if (sP.name == "right_hand_V2_1.txt")      { sP.name = "right_hand"; }
    else
    {
        printMessage(0,"ERROR! Unexpected skin part file name: %s.\n",sP.name.c_str());
        return false;
    }
    //sP.name = sP.name.substr(0, sP.name.find_last_of("_"));
       
    // Open File
    posFile.open(filePath.c_str());  
    if (!posFile.is_open())
        return false;

    // Acquire taxels (different for 1D and 2D only because the 2D case cannot handle all of the taxels,
    // so a subset of them [i.e. the representative taxels] has been used)
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

        if (sP.name == "left_forearm" || sP.name == "right_forearm")
        {
            // the taxels at the centers of respective triangles [note that i == taxelID == (line in the .txt file +1)]
            // e.g. first triangle of upper arm is at lines 1-12, center at line 4, thus i=2 
            // if(  (i==3) || (i==15)  ||  (i==27) ||  (i==39) ||  (i==51) ||  (i==63) ||  (i==75) ||  (i==87) ||
            //     (i==99) || (i==111) || (i==123) || (i==135) || (i==147) || (i==159) || (i==171) || (i==183) ||
            //    (i==207) || (i==255) || (i==291) || (i==303) || (i==315) || (i==339) || (i==351) )

            // if(  (i==3) ||  (i==39) || (i==207) || (i==255) || (i==291)) // Taxels that are evenly distributed throughout the forearm
                                                                         // in order to cover it as much as we can
            // if(  (i==3) ||  (i==15) ||  (i==27) || (i==183)) // taxels that are in the big patch but closest to the little patch (internally)
                                                                // 27 is proximal, 15 next, 3 next, 183 most distal
            // if((i==135) || (i==147) || (i==159) || (i==171))  // this is the second column, farther away from the stitch
                                                                 // 159 is most proximal, 147 is next, 135 next,  171 most distal
            // if((i==87) || (i==75)  || (i==39)|| (i==51)) // taxels that are in the big patch and closest to the little patch (externally)
            //                                              // 87 most proximal, 75 then, 39 then, 51 distal

            if((i==27) || (i==15) || (i==3) || (i==183) ||              // taxels used for the experimentations on the pps paper
               (i==147) || (i==135) || (i==75) || (i==39) || (i==51))
            {
                sP.size++;
                sP.taxel.push_back(Taxel2D(taxelPos,taxelNorm,i));
            }
            else
            {
                sP.size++;
            }
        }
        else if (sP.name == "left_hand")
        { //we want to represent the 48 taxels of the palm (ignoring fingertips) with 5 taxels -
         // manually marking 5 regions of the palm and selecting their "centroids" as the representatives
            if((i==99) || (i==101) || (i==109) || (i==122) || (i==134)) 
            {
                sP.size++;
                sP.taxel.push_back(Taxel2D(taxelPos,taxelNorm,i));
            }
            else
            {
                sP.size++;
            }
        }
        else if (sP.name == "right_hand")
        { //right hand has different taxel nr.s than left hand 
            // if((i==101) || (i==103) || (i==118) || (i==137)) // || (i==124)) remove one taxel
            if((i==101) || (i==103) || (i==118) || (i==137)) // || (i==124)) remove one taxel
            {
                sP.size++;
                sP.taxel.push_back(Taxel2D(taxelPos,taxelNorm,i));
            }
            else
            {
                sP.size++;
            }
        }
    }
    initRepresentativeTaxels(sP);

    return true;
}

void vtRFThread::initRepresentativeTaxels(skinPart2D &sP)
{
    int i=0;
    list<unsigned int> taxels_list;
    if (sP.name == "left_forearm" || sP.name == "right_forearm")
    {
        for (i=0;i<sP.size;i++)
        {
            //4th taxel of each 12 is the triangle midpoint
            sP.Taxel2Repr.push_back(((i/12)*12)+3); //initialize all 384 taxels with triangle center as the representative
            //fill a map of lists here somehow
        }
        
        // set to -1 the Taxel2Repr for all the taxels that don't exist
        for (i=192;i<=203;i++)
        {
            sP.Taxel2Repr[i]=-1; //these taxels don't exist
        }
        for (i=216;i<=251;i++)
        {
            sP.Taxel2Repr[i]=-1; //these taxels don't exist
        }
        for (i=264;i<=287;i++)
        {
            sP.Taxel2Repr[i]=-1; //these taxels don't exist
        }
        for (i=324;i<=335;i++)
        {
            sP.Taxel2Repr[i]=-1; //these taxels don't exist
        }
        for (i=360;i<=383;i++)
        {
            sP.Taxel2Repr[i]=-1; //these taxels don't exist
        }
        
        //let's set up the inverse - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        for(i=0;i<=11;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[3] = taxels_list;
        
        taxels_list.clear(); 
        for(i=12;i<=23;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[15] = taxels_list;
        
        taxels_list.clear(); 
        for(i=24;i<=35;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[27] = taxels_list;
        
        taxels_list.clear(); 
        for(i=36;i<=47;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[39] = taxels_list;
        
        taxels_list.clear(); 
        for(i=48;i<=59;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[51] = taxels_list;
        
        taxels_list.clear(); 
        for(i=60;i<=71;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[63] = taxels_list;
        
        taxels_list.clear(); 
        for(i=72;i<=83;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[75] = taxels_list;
        
        taxels_list.clear(); 
        for(i=84;i<=95;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[87] = taxels_list;
        
        taxels_list.clear(); 
        for(i=96;i<=107;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[99] = taxels_list;
        
        taxels_list.clear(); 
        for(i=108;i<=119;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[111] = taxels_list;
        
        taxels_list.clear(); 
        for(i=120;i<=131;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[123] = taxels_list;
        
        taxels_list.clear(); 
        for(i=132;i<=143;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[135] = taxels_list;
        
        taxels_list.clear(); 
        for(i=144;i<=155;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[147] = taxels_list;
        
        taxels_list.clear(); 
        for(i=156;i<=167;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[159] = taxels_list;
        
        taxels_list.clear(); 
        for(i=168;i<=179;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[171] = taxels_list;
        
        taxels_list.clear(); 
        for(i=180;i<=191;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[183] = taxels_list;
        //up to here - upper (full) patch on forearm
        
        //from here - lower patch with many dummy taxels
        taxels_list.clear(); 
        for(i=204;i<=215;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[207] = taxels_list;
        
        taxels_list.clear(); 
        for(i=252;i<=263;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[255] = taxels_list;
        
        taxels_list.clear(); 
        for(i=288;i<=299;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[291] = taxels_list;
        
        taxels_list.clear(); 
        for(i=300;i<=311;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[303] = taxels_list;
        
        taxels_list.clear(); 
        for(i=312;i<=323;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[315] = taxels_list;
        
        taxels_list.clear(); 
        for(i=336;i<=347;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[339] = taxels_list;
        
        taxels_list.clear(); 
        for(i=348;i<=359;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[351] = taxels_list;
    }
    else if(sP.name == "left_hand")
    {
       for(i=0;i<sP.size;i++)
       {
          sP.Taxel2Repr.push_back(-1); //let's fill all the 192 with -1 - half of the taxels don't exist and for fingertips, 
          //we don't have positions either
       }
       //upper left area of the palm - at thumb
       for (i=121;i<=128;i++)
       {
            sP.Taxel2Repr[i] = 122;
       }
       sP.Taxel2Repr[131] = 122; //thermal pad
       
       //let's set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
       taxels_list.clear(); 
       for(i=121;i<=128;i++)
       {
           taxels_list.push_back(i);
       }
       taxels_list.push_back(131);
       sP.Repr2TaxelList[122] = taxels_list;
       
        
       //upper center of the palm
       for (i=96;i<=99;i++)
       {
            sP.Taxel2Repr[i] = 99;
       }
       sP.Taxel2Repr[102] = 99;
       sP.Taxel2Repr[103] = 99;
       sP.Taxel2Repr[120] = 99;
       sP.Taxel2Repr[129] = 99;
       sP.Taxel2Repr[130] = 99;
       
       //let's set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
       taxels_list.clear(); 
       for(i=96;i<=99;i++)
       {
           taxels_list.push_back(i);
       }
       taxels_list.push_back(102);
       taxels_list.push_back(103);
       taxels_list.push_back(120);
       taxels_list.push_back(129);
       taxels_list.push_back(130);
       sP.Repr2TaxelList[99] = taxels_list;
        
       
       //upper right of the palm (away from the thumb)
       sP.Taxel2Repr[100] = 101;
       sP.Taxel2Repr[101] = 101;
       for (i=104;i<=107;i++)
       {
            sP.Taxel2Repr[i] = 101; //N.B. 107 is thermal pad
       }
       sP.Taxel2Repr[113] = 101;
       sP.Taxel2Repr[116] = 101;
       sP.Taxel2Repr[117] = 101;
       
       //let's set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
       taxels_list.clear(); 
       taxels_list.push_back(100);
       taxels_list.push_back(101);
       for(i=104;i<=107;i++)
       {
           taxels_list.push_back(i);
       }
       taxels_list.push_back(113);
       taxels_list.push_back(116);
       taxels_list.push_back(117);
       sP.Repr2TaxelList[101] = taxels_list;
       
       
       //center area of the palm
       for(i=108;i<=112;i++)
       {
        sP.Taxel2Repr[i] = 109;
       }
       sP.Taxel2Repr[114] = 109;
       sP.Taxel2Repr[115] = 109;
       sP.Taxel2Repr[118] = 109;
       sP.Taxel2Repr[142] = 109;
       sP.Taxel2Repr[143] = 109;
     
       //let's set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
       taxels_list.clear(); 
       for(i=108;i<=112;i++)
       {
           taxels_list.push_back(i);
       }
       taxels_list.push_back(114);
       taxels_list.push_back(115);
       taxels_list.push_back(118);
       taxels_list.push_back(142);
       taxels_list.push_back(143);
       sP.Repr2TaxelList[109] = taxels_list;
       
       
       //lower part of the palm
       sP.Taxel2Repr[119] = 134; // this one is thermal pad
       for(i=132;i<=141;i++)
       {
        sP.Taxel2Repr[i] = 134;
       }
       
       //let's set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
       taxels_list.clear(); 
       taxels_list.push_back(119);
       for(i=132;i<=141;i++)
       {
           taxels_list.push_back(i);
       }
       sP.Repr2TaxelList[134] = taxels_list;
              
    }
    else if(sP.name == "right_hand")
    {
       for(i=0;i<sP.size;i++)
       {
          sP.Taxel2Repr.push_back(-1); //let's fill all the 192 with -1 - half of the taxels don't exist and for fingertips, 
          //we don't have positions either
       }
       //upper left area - away from thumb on this hand
        sP.Taxel2Repr[96] = 101;
        sP.Taxel2Repr[97] = 101;
        sP.Taxel2Repr[98] = 101;
        sP.Taxel2Repr[100] = 101;
        sP.Taxel2Repr[101] = 101;
        sP.Taxel2Repr[107] = 101; //thermal pad
        sP.Taxel2Repr[110] = 101;
        sP.Taxel2Repr[111] = 101;
        sP.Taxel2Repr[112] = 101;
        
        //let's set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        taxels_list.push_back(96);
        taxels_list.push_back(97);
        taxels_list.push_back(98);
        taxels_list.push_back(100);
        taxels_list.push_back(101);
        taxels_list.push_back(107);
        taxels_list.push_back(110);
        taxels_list.push_back(111);
        taxels_list.push_back(112);
        sP.Repr2TaxelList[101] = taxels_list;
        
        //upper center of the palm
        sP.Taxel2Repr[99] = 103;
        for(i=102;i<=106;i++)
        {
           sP.Taxel2Repr[i] = 103;
        }
        sP.Taxel2Repr[127] = 103;
        sP.Taxel2Repr[129] = 103;
        sP.Taxel2Repr[130] = 103;
        
        //let's set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        taxels_list.push_back(99);
        for(i=102;i<=106;i++)
        {
            taxels_list.push_back(i);
        }
        taxels_list.push_back(127);
        taxels_list.push_back(129);
        taxels_list.push_back(130);
        sP.Repr2TaxelList[103] = taxels_list;
        
        
        //upper right center of the palm - at thumb
        for(i=120;i<=126;i++)
        {
           sP.Taxel2Repr[i] = 124;
        }
        sP.Taxel2Repr[128] = 124;
        sP.Taxel2Repr[131] = 124; //thermal pad
        
        //let's set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        for(i=120;i<=126;i++)
        {
            taxels_list.push_back(i);
        }
        taxels_list.push_back(128);
        taxels_list.push_back(131);
        sP.Repr2TaxelList[124] = taxels_list;
        
        
        //center of palm
        sP.Taxel2Repr[108] = 118;
        sP.Taxel2Repr[109] = 118;
        for(i=113;i<=118;i++)
        {
            sP.Taxel2Repr[i] = 118;
        }
        sP.Taxel2Repr[142] = 118;
        sP.Taxel2Repr[143] = 118;
        
        //let's set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        taxels_list.push_back(108);
        taxels_list.push_back(109);
        for(i=113;i<=118;i++)
        {
            taxels_list.push_back(i);
        }
        taxels_list.push_back(142);
        taxels_list.push_back(143);
        sP.Repr2TaxelList[118] = taxels_list;
            
        //lower palm
        sP.Taxel2Repr[119] = 137; //thermal pad
        for(i=132;i<=141;i++)
        {
            sP.Taxel2Repr[i] = 137; //139 is another thermal pad
        }
        
        //let's set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        taxels_list.push_back(119);
        for(i=132;i<=141;i++)
        {
            taxels_list.push_back(i);
        }
        sP.Repr2TaxelList[137] = taxels_list;
    }
}

bool vtRFThread::getRepresentativeTaxelsToTrain(const std::vector<unsigned int> IDv, const int IDx, std::vector<unsigned int> &v)
{
    //unordered_set would be better, but that is only experimentally supported by some compilers.
    std::set<unsigned int> rep_taxel_IDs_set;
    
    if (iCubSkin2D[IDx].Taxel2Repr.empty())
    {
            v = IDv; //we simply copy the activated taxels
            return false;
    }
    else
    {
        for (std::vector<unsigned int>::const_iterator it = IDv.begin() ; it != IDv.end(); ++it)
        {
            if (iCubSkin2D[IDx].Taxel2Repr[*it] == -1)
            {
                printMessage(0,"ERROR: [%s] taxel %u activated, but representative taxel undefined - ignoring.\n",iCubSkin2D[IDx].name.c_str(),*it);
            }
            else
            {
                //add all the representatives that were activated to the set
                rep_taxel_IDs_set.insert(iCubSkin2D[IDx].Taxel2Repr[*it]);
            }
        }

        for (std::set<unsigned int>::const_iterator itr = rep_taxel_IDs_set.begin(); itr != rep_taxel_IDs_set.end(); ++itr)
        {
            v.push_back(*itr); //add the representative taxels that were activated to the output taxel ID vector    
        }

        if (v.empty())
        {
            printMessage(0,"ERROR! Representative taxels' vector is empty! Skipping\n");
            return false;
        }
        
        if (verbosity>=4)
        {
            printMessage(4,"Representative taxels 'touched' on skin part %d: \n",IDx);
            for(std::vector<unsigned int>::const_iterator it = v.begin() ; it != v.end(); ++it)
            {
                printf("%d ",*it);
            }
        }
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
        ddT.close();
        ddH.close();

    printMessage(0,"Deleting misc stuff..\n");
        delete armR;
        armR = NULL;
        delete armL;
        armL = NULL;
        delete eWR;
        eWR  = NULL;
        delete eWL;
        eWL  = NULL;

    printMessage(0,"Closing ports..\n");
        closePort(imagePortInR);
        printMessage(1,"    imagePortInR successfully closed!\n");
        closePort(imagePortInL);
        printMessage(1,"    imagePortInL successfully closed!\n");

        // closePort(imagePortOutR);
        imagePortOutR.interrupt();
        imagePortOutR.close();
        printMessage(1,"    imagePortOutR successfully closed!\n");
        // closePort(imagePortOutL);
        imagePortOutL.interrupt();
        imagePortOutL.close();
        printMessage(1,"    imagePortOutL successfully closed!\n");

        closePort(dTPort);
        printMessage(1,"    dTPort successfully closed!\n");
        closePort(eventsPort);
        printMessage(1,"    eventsPort successfully closed!\n");

        closePort(skinPortIn);
        printMessage(1,"    skinPortIn successfully closed!\n");

        // closePort(skinGuiPortForearmL);
        skinGuiPortForearmL.interrupt();
        skinGuiPortForearmL.close();
        printMessage(1,"    skinGuiPortForearmL successfully closed!\n");
        // closePort(skinGuiPortForearmR);
        skinGuiPortForearmR.interrupt();
        skinGuiPortForearmR.close();
        printMessage(1,"    skinGuiPortForearmR successfully closed!\n");
        // closePort(skinGuiPortHandL);
        skinGuiPortHandL.interrupt();
        skinGuiPortHandL.close();
        printMessage(1,"    skinGuiPortHandL successfully closed!\n");
        // closePort(skinGuiPortHandR);
        skinGuiPortHandR.interrupt();
        skinGuiPortHandR.close();
        printMessage(1,"    skinGuiPortHandR successfully closed!\n");
    printMessage(0,"DONE.\n");
}

// empty line to make gcc happy
