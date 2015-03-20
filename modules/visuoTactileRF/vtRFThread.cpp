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
        if (rf->check("taxelsFile"))
        {
            taxelsFile = rf -> find("taxelsFile").asString();
        }
        else
        {
            taxelsFile = "taxels"+modality+".ini";
            rf -> setDefault("taxelsFile",taxelsFile.c_str());
        }
        yInfo("Storing file set to: %s", (path+taxelsFile).c_str());
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

        Network::connect(("/"+name+"/skinGuiForearmL:o").c_str(),"/skinGui/left_forearm_virtual:i");
        Network::connect(("/"+name+"/skinGuiForearmR:o").c_str(),"/skinGui/right_forearm_virtual:i");
        Network::connect(("/"+name+"/skinGuiHandL:o").c_str(),"/skinGui/left_hand_virtual:i");
        Network::connect(("/"+name+"/skinGuiHandR:o").c_str(),"/skinGui/right_hand_virtual:i");
           
        Network::connect("/skinManager/skin_events:o",("/"+name+"/skin_events:i").c_str());

        ts.update();

    /**************************/
        if (rf->check("rightHand") || rf->check("rightForeArm") ||
            (!rf->check("rightHand") && !rf->check("rightForeArm") && !rf->check("leftHand") && !rf->check("leftForeArm")))
        {
            Property OptR;
            OptR.put("robot",  robot.c_str());
            OptR.put("part",   "right_arm");
            OptR.put("device", "remote_controlboard");
            OptR.put("remote",("/"+robot+"/right_arm").c_str());
            OptR.put("local", ("/"+name +"/right_arm").c_str());

            if (!ddR.open(OptR))
            {
                yError(": could not open right_arm PolyDriver!\n");
                return false;
            }
            bool ok = 1;
            if (ddR.isValid())
            {
                ok = ok && ddR.view(iencsR);
            }
            if (!ok)
            {
                yError("[vtRFThread] Problems acquiring right_arm interfaces!!!!\n");
                return false;
            }
            iencsR->getAxes(&jntsR);
            encsR = new yarp::sig::Vector(jntsR,0.0);
        }

    /**************************/
        if (rf->check("leftHand") || rf->check("leftForeArm") ||
            (!rf->check("rightHand") && !rf->check("rightForeArm") && !rf->check("leftHand") && !rf->check("leftForeArm")))
        {
            Property OptL;
            OptL.put("robot",  robot.c_str());
            OptL.put("part",   "left_arm");
            OptL.put("device", "remote_controlboard");
            OptL.put("remote",("/"+robot+"/left_arm").c_str());
            OptL.put("local", ("/"+name +"/left_arm").c_str());

            if (!ddL.open(OptL))
            {
                yError(": could not open left_arm PolyDriver!\n");
                return false;
            }
            bool ok = 1;
            if (ddL.isValid())
            {
                ok = ok && ddL.view(iencsL);
            }
            if (!ok)
            {
                yError("[vtRFThread] Problems acquiring left_arm interfaces!!!!\n");
                return false;
            }
            iencsL->getAxes(&jntsL);
            encsL = new yarp::sig::Vector(jntsL,0.0);
        }

    /**************************/
        Property OptT;
        OptT.put("robot",  robot.c_str());
        OptT.put("part",   "torso");
        OptT.put("device", "remote_controlboard");
        OptT.put("remote",("/"+robot+"/torso").c_str());
        OptT.put("local", ("/"+name +"/torso").c_str());

        if (!ddT.open(OptT))
        {
            yError("Could not open torso PolyDriver!");
            return false;
        }
        bool ok = 1;
        if (ddT.isValid())
        {
            ok = ok && ddT.view(iencsT);
        }
        if (!ok)
        {
            yError("Problems acquiring head interfaces!!!!");
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
            yError("ERROR: could not open head PolyDriver!");
            return false;
        }
        ok = 1;
        if (ddH.isValid())
        {
            ok = ok && ddH.view(iencsH);
        }
        if (!ok)
        {
            yError("Problems acquiring head interfaces!!!!");
            return false;
        }
        iencsH->getAxes(&jntsH);
        encsH = new yarp::sig::Vector(jntsH,0.0);

    /**************************/
        yDebug("Setting up iCubSkin...");
        iCubSkinSize=filenames.size();

        for(unsigned int i=0;i<filenames.size();i++)
        {
            string filePath = filenames[i];
            yDebug("i: %i filePath: %s",i,filePath.c_str());
            skinPartPWE sP(modality);
            if ( setTaxelPosesFromFile(filePath,sP) )
                iCubSkin.push_back(sP);
        }
        load();

        yInfo("iCubSkin correctly instantiated. Size: %i",iCubSkin.size());
        if (verbosity>= 2)
        {
            for (size_t i = 0; i < iCubSkin.size(); i++)
            {
                iCubSkin[i].print();
            }
        }
        iCubSkinSize = iCubSkin.size();

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
    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        for (size_t j = 0; j < iCubSkin[i].txls.size(); j++)
        {
            iCubSkin[i].txls[j]->WRFPos=locateTaxel(iCubSkin[i].txls[j]->Pos,iCubSkin[i].name);
            printMessage(5,"iCubSkin[%i].txls[%i].WRFPos %s\n",i,j,iCubSkin[i].txls[j]->WRFPos.toString().c_str());
        }
    }

    // process the port coming from the visuoTactileWrapper
    if (event != NULL)
    {
        // read the events
        for (size_t i = 0; i < event->size(); i++)
        {
            incomingEvents.push_back(IncomingEvent(*(event -> get(i).asList())));
            printMessage(3,"[EVENT] %s", incomingEvents.back().toString().c_str());
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

        // limit the size of the buffer to 80, i.e. 4 seconds of acquisition
        if (eventsBuffer.size() >= 80)
        {
            eventsBuffer.erase(eventsBuffer.begin());
            yTrace("Too many samples: removing the older element from the buffer..");
        }

        // detect contacts and train the taxels
        if (skinContacts && eventsBuffer.size()>20)
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
                    size_t txlsize = iCubSkin[0].txls.size();
                    for (size_t j = 0; j < txlsize; j++)
                    {
                        dumpedVector.push_back(0.0);
                    }
                }
                eventsBuffer.clear();
            }
            else
            {
                size_t txlsize = iCubSkin[0].txls.size();
                for (size_t j = 0; j < txlsize; j++)
                {
                    dumpedVector.push_back(0.0);
                }
            }
        }
        else
        {
            size_t txlsize = iCubSkin[0].txls.size();
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
        size_t txlsize = iCubSkin[0].txls.size();
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
                for (size_t j = 0; j < iCubSkin[i].txls.size(); j++) // cycle through the taxels
                {
                    if (iCubSkin[i].txls[j]->Resp > 100)
                    {
                        taxelsIDs.push_back(iCubSkin[i].txls[j]->ID);
                        isThereAnEvent = true;
                    }
                }
                if (isThereAnEvent)
                {
                    part   = iCubSkin[i].name;
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
            for (size_t j = 0; j < iCubSkin[iCubSkinID].txls.size(); j++)
            {
                if (iCubSkin[iCubSkinID].txls[j]->ID == taxelsIDs[i])
                {
                    w = iCubSkin[iCubSkinID].txls[j]->Resp;
                    geoCenter += iCubSkin[iCubSkinID].txls[j]->WRFPos*w;
                    normalDir += locateTaxel(iCubSkin[iCubSkinID].txls[j]->Norm,part)*w;
                    w_sum += w;
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

        respToSkin.resize(iCubSkin[i].size,0.0);   // resize the vector to the skinPart
        iCubSkinName=iCubSkin[i].name;

        if (incomingEvents.size()>0)
        {
            for (size_t j = 0; j < iCubSkin[i].txls.size(); j++)
            {
                if(iCubSkin[i].Repr2TaxelList.empty())
                {  
                    //we simply light up the taxels themselves
                    respToSkin[iCubSkin[i].txls[j]->ID] = iCubSkin[i].txls[j]->Resp;
                }
                else
                { 
                    //we light up all the taxels represented by the particular taxel
                    list<unsigned int> l = iCubSkin[i].Repr2TaxelList[iCubSkin[i].txls[j]->ID];

                    if (l.empty())
                    {
                        yWarning("skinPart %d Taxel %d : no list of represented taxels is available, even if Repr2TaxelList is not empty",i,iCubSkin[i].txls[j]->ID);
                        respToSkin[iCubSkin[i].txls[j]->ID] = iCubSkin[i].txls[j]->Resp;
                    }
                    else
                    {
                        for(list<unsigned int>::const_iterator iter_list = l.begin(); iter_list != l.end(); iter_list++)
                        {
                            //for all the represented taxels, we assign the activation of the super-taxel
                            respToSkin[*iter_list] =  iCubSkin[i].txls[j]->Resp;
                        } 
                    }
                }
            }
        }


        Bottle colorBottle;
        colorBottle.addInt(0);
        colorBottle.addInt(200);
        colorBottle.addInt(100);

        // Bottle outputBottle=

        // Bottle colorBottle,compensatedDataBottle;
        // colorBottle.addList().read(color);
        // compensatedDataBottle.addList().read(compensatedData2Send);

        // Property& outputData = compensatedTactileDataPort.prepare();
        // outputData.put("color",colorBottle.get(0));
        // outputData.put("data",compensatedDataBottle.get(0));

        // compensatedTactileDataPort.write();

        BufferedPort<Bottle> *outPort;
        if(iCubSkinName == "left_forearm")
        {
            outPort = &skinGuiPortForearmL; 
        }
        else if(iCubSkinName == "right_forearm")
        {
            outPort = &skinGuiPortForearmR; 
        }
        else if(iCubSkinName == "left_hand")
        {
            outPort = &skinGuiPortHandL; 
        }
        else if(iCubSkinName == "right_hand")
        {
            outPort = &skinGuiPortHandR;
        }

        Bottle dataBottle;
        dataBottle.addList().read(respToSkin);

        Bottle& outputBottle=outPort->prepare();
        outputBottle.clear();

        outputBottle.addList() = *(dataBottle.get(0).asList());
        outputBottle.addList() = colorBottle;

        outPort->setEnvelope(ts);
        outPort->write();
    }
}

bool vtRFThread::detectContact(iCub::skinDynLib::skinContactList *_sCL, int &idx,
                               std::vector <unsigned int> &idv)
{
    // Search for a suitable contact. It has this requirements:
    //   1. it has to be higher than SKIN_THRES
    //   2. more than two taxels should be active for that contact (in order to avoid spikes)
    //   3. it should be in the proper skinpart (forearms and hands)
    //   4. it should activate one of the taxels used by the module
    //      (e.g. the fingers will not be considered)
    for(iCub::skinDynLib::skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++)
    {
        idv.clear();
        if( it -> getPressure() > SKIN_THRES && (it -> getTaxelList()).size() > 2 )
        {
            for (size_t i = 0; i < iCubSkinSize; i++)
            {
                string iCubSkinName="";
                iCubSkinName=iCubSkin[i].name;

                if ((it -> getSkinPart() == FOREARM_RIGHT && iCubSkinName == "right_forearm") ||
                    (it -> getSkinPart() ==  FOREARM_LEFT && iCubSkinName == "left_forearm" ) ||
                    (it -> getSkinPart() ==    HAND_RIGHT && iCubSkinName == "right_hand"   ) ||
                    (it -> getSkinPart() ==     HAND_LEFT && iCubSkinName == "left_hand"    )    )
                {
                    idx = i;
                    std::vector <unsigned int> txlList = it -> getTaxelList();

                    bool itHasBeenTouched = false;

                    getRepresentativeTaxels(txlList, idx, idv);

                    if (idv.size()>0)
                    {
                        // printf("I have been touched!!!!!!\n");
                        itHasBeenTouched = true;
                    }

                    if (itHasBeenTouched)
                    {
                        if (verbosity>=1)
                        {
                            printMessage(1,"Contact! Skin part: %s\tTaxels' ID: ",iCubSkinName.c_str());
                            for (size_t i = 0; i < idv.size(); i++)
                                printf("\t%i",idv[i]);

                            printf("\n");
                        }

                        return true;
                    }
                }
            }
        }
    }
    return false;
}

string vtRFThread::load()
{
    // rf->setVerbose(true);
    // string fileName=rf->findFile("taxelsFile").c_str();
    // rf->setVerbose(false);
    // if (fileName=="")
    // {
    //     yWarning("[vtRF::load] No filename has been found. Skipping..");
    //     string ret="";
    //     return ret;
    // }

    // yInfo("File loaded: %s", fileName.c_str());
    // Property data; data.fromConfigFile(fileName.c_str());
    // Bottle b; b.read(data);
    // yDebug("iCubSkinSize %i",iCubSkinSize);

    // for (size_t i = 0; i < iCubSkinSize; i++)
    // {
    //     if (modality=="1D")
    //     {
    //         Bottle bb = b.findGroup(iCubSkin1D[i].name.c_str());

    //         if (bb.size() > 0)
    //         {
    //             int nTaxels = bb.find("nTaxels").asInt();
    //             int size    = bb.find("size").asInt();
    //             std::vector<double> extX;
    //             std::vector<int>    bNum;
    //             std::vector<int>    mapp;

    //             Bottle *bbb;
    //             bbb = bb.find("extX").asList();
    //             extX.push_back(bbb->get(0).asDouble());
    //             extX.push_back(bbb->get(1).asDouble());
    //             bbb = bb.find("binsNum").asList();
    //             bNum.push_back(bbb->get(0).asInt());
    //             bbb = bb.find("Mapping").asList();

    //             yDebug("    [%s] size %i\tnTaxels %i\textX %g  %g\tbinsNum %i",iCubSkin1D[i].name.c_str(),size,nTaxels,extX[0],extX[1],bNum[0]);
    //             printMessage(3,"mapp\n");
    //             for (size_t j = 0; j < size; j++)
    //             {
    //                 mapp.push_back(bbb->get(j).asInt());
    //                 if (verbosity>=3)
    //                 {
    //                     printf("%i ",mapp[j]);
    //                 }
    //             }
    //             printf("\n");
    //             iCubSkin1D[i].size = size;
    //             iCubSkin1D[i].Taxel2Repr = mapp;

    //             for (size_t j = 0; j < nTaxels; j++)
    //             {
    //                 bbb = bb.get(j+6).asList();
    //                 printMessage(3,"Reading taxel %s\n",bbb->toString().c_str());

    //                 for (int k = 0; k < iCubSkin1D[i].txls.size(); k++)
    //                 {
    //                     if (iCubSkin1D[i].txls[k]->ID == bbb->get(0).asInt())
    //                     {
    //                         iCubSkin1D[i].txls[k]->pwe->resize(extX,bNum);
    //                         iCubSkin1D[i].txls[k]->pwe->setPosHist(matrixFromBottle(*bbb->get(1).asList(),0,bNum[0],1));
    //                         iCubSkin1D[i].txls[k]->pwe->setNegHist(matrixFromBottle(*bbb->get(2).asList(),0,bNum[0],1));
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //     else
    //     {
    //         Bottle bb = b.findGroup(iCubSkin[i].name.c_str());

    //         if (bb.size() > 0)
    //         {
    //             int nTaxels = bb.find("nTaxels").asInt();
    //             int size    = bb.find("size").asInt();
    //             std::vector<double> extX;
    //             std::vector<double> extY;
    //             std::vector<int>    bNum;
    //             std::vector<int>    mapp;

    //             Bottle *bbb;
    //             bbb = bb.find("extX").asList();
    //             extX.push_back(bbb->get(0).asDouble());
    //             extX.push_back(bbb->get(1).asDouble());
    //             bbb = bb.find("extY").asList();
    //             extY.push_back(bbb->get(0).asDouble());
    //             extY.push_back(bbb->get(1).asDouble());
    //             bbb = bb.find("binsNum").asList();
    //             bNum.push_back(bbb->get(0).asInt());
    //             bNum.push_back(bbb->get(1).asInt());
    //             bbb = bb.find("Mapping").asList();

    //             yDebug("    [%s] size %i\tnTaxels %i\textX %g  %g\n",iCubSkin[i].name.c_str(),size,nTaxels,extX[0],extX[1]);
    //             yDebug("    extY %g  %g\tbinsNum %i  %i\n",extY[0],extY[1],bNum[0],bNum[1]);
    //             printMessage(5,"mapp\n");
    //             for (size_t j = 0; j < size; j++)
    //             {
    //                 mapp.push_back(bbb->get(j).asInt());
    //                 if (verbosity>=6)
    //                 {
    //                     printf("%i ",mapp[j]);
    //                 }
    //             }
    //             printf("\n");
    //             iCubSkin[i].size = size;
    //             iCubSkin[i].Taxel2Repr = mapp;

    //             for (size_t j = 0; j < nTaxels; j++)
    //             {
    //                 bbb = bb.get(j+7).asList();
    //                 printMessage(5,"Reading taxel %s\n",bbb->toString().c_str());

    //                 for (int k = 0; k < iCubSkin1D[i].txls.size(); k++)
    //                 {
    //                     if (iCubSkin[i].txls[k]->ID == bbb->get(0).asInt())
    //                     {
    //                         iCubSkin[i].txls[k]->pwe->resize(extX,extY,bNum);
    //                         iCubSkin[i].txls[k]->pwe->setPosHist(matrixFromBottle(*bbb->get(1).asList(),0,bNum[0],bNum[1]));
    //                         iCubSkin[i].txls[k]->pwe->setNegHist(matrixFromBottle(*bbb->get(2).asList(),0,bNum[0],bNum[1]));
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    // return fileName;
}

string vtRFThread::save()
{
    int    lastindex     = taxelsFile.find_last_of("."); 
    string taxelsFileRaw = taxelsFile.substr(0, lastindex);

    string fnm=path+taxelsFileRaw+"_out.ini";
    ofstream myfile;
    yInfo("Saving to: %s", fnm.c_str());
    myfile.open(fnm.c_str(),ios::trunc);

    if (myfile.is_open())
    {
        for (size_t i = 0; i < iCubSkinSize; i++)
        {
            myfile << "modality\t" << iCubSkin[i].modality << endl;

            Bottle data;
            data.clear();

            Matrix           getExt = iCubSkin[i].txls[0]->pwe->getExt();
            matrixIntoBottle(getExt,data);

            std::vector<int> hSize  = iCubSkin[i].txls[0]->pwe->getHistSize();

            myfile << "[" << iCubSkin[i].name << "]" << endl;
            myfile << "size\t"    << iCubSkin[i].size << endl;    
            myfile << "nTaxels\t" << iCubSkin[i].txls.size() << endl;
            myfile << "ext\t"     << data.toString() << "\n";
            myfile << "hSize\t( " << hSize[0] << hSize[1] << " )\n";

            data.clear();
            Bottle &representatives = data.addList();
            for (size_t q = 0; q < iCubSkin[i].Taxel2Repr.size(); q++)
            {
                representatives.addInt(iCubSkin[i].Taxel2Repr[q]);
            } 
            myfile << "Mapping\t" << data.toString() << endl;

            for (size_t j = 0; j < iCubSkin[i].txls.size(); j++)
            {
                std::vector<int> bNum  = iCubSkin[i].txls[j]->pwe->getHistSize();
                Matrix           pHist = iCubSkin[i].txls[j]->pwe->getPosHist();
                Matrix           nHist = iCubSkin[i].txls[j]->pwe->getNegHist();

                data.clear();
                matrixIntoBottle(pHist,data);
                myfile << iCubSkin[i].txls[j]->ID << "\t\t" << data.toString() << "\t";

                data.clear();
                matrixIntoBottle(nHist,data);
                myfile << data.toString() << endl;
            }
        }
    }
    myfile.close();
    return fnm;
}

bool vtRFThread::trainTaxels(const std::vector<unsigned int> IDv, const int IDx)
{
    std::vector<unsigned int> v;
    string iCubSkinName = "";

    iCubSkinName=iCubSkin[IDx].name;
    getRepresentativeTaxels(IDv, IDx, v);

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
        yError(" in trainTaxels!\n");
        return false;
    }

    for (size_t j = 0; j < iCubSkin[IDx].txls.size(); j++)
    {
        bool itHasBeenTouched = false;
        for (size_t w = 0; w < v.size(); w++)
        {
            if (iCubSkin[IDx].txls[j]->ID == v[w])
            {
                itHasBeenTouched = true;
            }
        }

        for (size_t k = 0; k < eventsBuffer.size(); k++)
        {
            IncomingEvent4TaxelPWE projection = projectIntoTaxelRF(iCubSkin[IDx].txls[j]->FoR,T_a,eventsBuffer[k]);
            printMessage(4,"Training Taxels: skinPart %d ID %i k %i NORM %g TTC %g\n",IDx,iCubSkin[IDx].txls[j]->ID,k,projection.NRM,projection.TTC);

            if (itHasBeenTouched == true)   iCubSkin[IDx].txls[j]->addSample(projection);
            else                            iCubSkin[IDx].txls[j]->removeSample(projection);
        }

        if (itHasBeenTouched == true)   dumpedVector.push_back(1.0);
        else                            dumpedVector.push_back(-1.0);
    }

    return true;
}

bool vtRFThread::projectIncomingEvent()
{
    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        string iCubSkinName="";
        iCubSkinName=iCubSkin[i].name;

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
            yError(" in projectIncomingEvent!\n");

        // yInfo("T_A:\n%s",T_a.toString().c_str());

        for (size_t j = 0; j < iCubSkin[i].txls.size(); j++)
        {
            iCubSkin[i].txls[j]->Evnt=projectIntoTaxelRF(iCubSkin[i].txls[j]->FoR,T_a,
                                                         incomingEvents[incomingEvents.size()-1]);

            // There's a reason behind this choice
            dumpedVector.push_back(iCubSkin[i].txls[j]->Evnt.Pos[0]);
            dumpedVector.push_back(iCubSkin[i].txls[j]->Evnt.Pos[1]);
            dumpedVector.push_back(iCubSkin[i].txls[j]->Evnt.Pos[2]);

            printMessage(5,"Projection -> i: %i\tID %i\tEvent: %s\n",i,j,iCubSkin[i].txls[j]->Evnt.toString().c_str());
        }
    }
    return true;
}

IncomingEvent4TaxelPWE vtRFThread::projectIntoTaxelRF(const Matrix &RF,const Matrix &T_a,const IncomingEvent &e)
{
    IncomingEvent4TaxelPWE Event_projected = e;

    Matrix T_a_proj = T_a * RF;

    Vector p=e.Pos; p.push_back(1);
    Vector v=e.Vel; v.push_back(1);

    Event_projected.Pos = SE3inv(T_a_proj)*p;        Event_projected.Pos.pop_back();
    Event_projected.Vel = SE3inv(T_a_proj)*v;        Event_projected.Vel.pop_back();

    if (e.Radius != -1.0)
    {
        Event_projected.Pos(2) -= Event_projected.Radius;
    }

    Event_projected.computeNRMTTC();

    return Event_projected;
}

void vtRFThread::resetParzenWindows()
{
    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        for (size_t j = 0; j < iCubSkin[i].txls.size(); j++)
        {
            iCubSkin[i].txls[j]->resetParzenWindowEstimator();
        }
    }
}

bool vtRFThread::computeResponse()
{
    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        for (size_t j = 0; j < iCubSkin[i].txls.size(); j++)
        {
            iCubSkin[i].txls[j]->computeResponse();
            printMessage(4,"\t\t\tID %i\tResponse %i\n",j,iCubSkin[i].txls[j]->Resp);
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
    projectIntoImagePlane(iCubSkin,_eye);

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
        for (size_t j = 0; j < iCubSkin[i].txls.size(); j++)
        {
            drawTaxel(imgOut,iCubSkin[i].txls[j]->px,iCubSkin[i].name,iCubSkin[i].txls[j]->Resp);
            printMessage(6,"iCubSkin[%i].txls[%i].px %s\n",i,j,iCubSkin[i].txls[j]->px.toString().c_str());
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


bool vtRFThread::projectIntoImagePlane(vector <skinPartPWE> &sP, const string &eye)
{
    for (size_t i = 0; i < sP.size(); i++)
    {
        for (size_t j = 0; j < sP[i].txls.size(); j++)
        {
            if (eye=="rightEye" || eye=="leftEye")
            {
                projectPoint(sP[i].txls[j]->WRFPos,sP[i].txls[j]->px,eye);
            }
            else
            {
                yError("ERROR in projectIntoImagePlane!\n");
                return false;
            }
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
        yError(" locateTaxel() failed!\n");
    }

    if      (part == "left_forearm" ) { T = armL -> getH(3+4, true); } // torso + up to elbow
    else if (part == "right_forearm") { T = armR -> getH(3+4, true); } // torso + up to elbow
    else if (part == "left_hand")     { T = armL -> getH(3+6, true); } // torso + up to wrist
    else if (part == "right_hand")    { T = armR -> getH(3+6, true); } // torso + up to wrist
    else    {  yError(" locateTaxel() failed!\n"); }

    pos.push_back(1);
    WRFpos = T * pos;
    WRFpos.pop_back();

    return WRFpos;
}

//see also Compensator::setTaxelPosesFromFile in icub-main/src/modules/skinManager/src/compensator.cpp
bool vtRFThread::setTaxelPosesFromFile(const string filePath, skinPartPWE &sP)
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
        yError(" Unexpected skin part file name: %s.\n",sP.name.c_str());
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
            if(  (i==3) || (i==15)  ||  (i==27) ||  (i==39) ||  (i==51) ||  (i==63) ||  (i==75) ||  (i==87) ||
                (i==99) || (i==111) || (i==123) || (i==135) || (i==147) || (i==159) || (i==171) || (i==183) ||
               (i==207) || (i==255) || (i==291) || (i==303) || (i==315) || (i==339) || (i==351) )

            // if(  (i==3) ||  (i==39) || (i==207) || (i==255) || (i==291)) // Taxels that are evenly distributed throughout the forearm
                                                                         // in order to cover it as much as we can
            // if(  (i==3) ||  (i==15) ||  (i==27) || (i==183)) // taxels that are in the big patch but closest to the little patch (internally)
                                                                // 27 is proximal, 15 next, 3 next, 183 most distal
            // if((i==135) || (i==147) || (i==159) || (i==171))  // this is the second column, farther away from the stitch
                                                                 // 159 is most proximal, 147 is next, 135 next,  171 most distal
            // if((i==87) || (i==75)  || (i==39)|| (i==51)) // taxels that are in the big patch and closest to the little patch (externally)
            //                                              // 87 most proximal, 75 then, 39 then, 51 distal

            // if((i==27)  || (i==15)  || (i==3)   || (i==183) ||              // taxels used for the experimentations on the pps paper
            //    (i==135) || (i==147) || (i==159) || (i==171) ||
            //    (i==87)  || (i==75)  || (i==39)  || (i==51))

            // if((i==27)  || (i==15)  || (i==3)   || (i==183) ||              // taxels used for the experimentations on the IROS paper
            //    (i==87)  || (i==75)  || (i==39)  || (i==51))
            {
                sP.size++;
                if (modality=="1D")
                {
                    sP.txls.push_back(new TaxelPWE1D(taxelPos,taxelNorm,i));
                }
                else
                {
                    sP.txls.push_back(new TaxelPWE2D(taxelPos,taxelNorm,i));
                }
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
                if (modality=="1D")
                {
                    sP.txls.push_back(new TaxelPWE1D(taxelPos,taxelNorm,i));
                }
                else
                {
                    sP.txls.push_back(new TaxelPWE2D(taxelPos,taxelNorm,i));
                }
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
                if (modality=="1D")
                {
                    sP.txls.push_back(new TaxelPWE1D(taxelPos,taxelNorm,i));
                }
                else
                {
                    sP.txls.push_back(new TaxelPWE2D(taxelPos,taxelNorm,i));
                }
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

void vtRFThread::initRepresentativeTaxels(skinPart &sP)
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

bool vtRFThread::getRepresentativeTaxels(const std::vector<unsigned int> IDv, const int IDx, std::vector<unsigned int> &v)
{
    //unordered_set would be better, but that is only experimentally supported by some compilers.
    std::set<unsigned int> rep_taxel_IDs_set;
    
    if (iCubSkin[IDx].Taxel2Repr.empty())
    {
        v = IDv; //we simply copy the activated taxels
        return false;
    }
    else
    {
        for (std::vector<unsigned int>::const_iterator it = IDv.begin() ; it != IDv.end(); ++it)
        {
            if (iCubSkin[IDx].Taxel2Repr[*it] == -1)
            {
                yWarning("[%s] taxel %u activated, but representative taxel undefined - ignoring.",iCubSkin[IDx].name.c_str(),*it);
            }
            else
            {
                rep_taxel_IDs_set.insert(iCubSkin[IDx].Taxel2Repr[*it]); //add all the representatives that were activated to the set
            }
        }

        for (std::set<unsigned int>::const_iterator itr = rep_taxel_IDs_set.begin(); itr != rep_taxel_IDs_set.end(); ++itr)
        {
            v.push_back(*itr); //add the representative taxels that were activated to the output taxel ID vector    
        }

        if (v.empty())
        {
            yWarning("Representative taxels' vector is empty! Skipping.");
            return false;
        }
        
        if (verbosity>=4)
        {
            printMessage(4,"Representative taxels on skin part %d: \n",IDx);
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
        fprintf(stdout,"[%s] ",name.c_str());

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
