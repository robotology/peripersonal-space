#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iomanip>

#include "vtRFThread.h"

#define RADIUS         2 // radius in px of every taxel (in the images)
#define RFEXTXMIN   -0.1 // lower limit of the receptive field [D] (-0.1 m)
#define RFEXTXMAX    0.2 // upper limit of the receptive field [D] (+0.2 m)
#define RFEXTYMIN    0.0 // lower limit of the receptive field [TTC] (+0.0 s)
#define RFEXTYMAX    3.0 // upper limit of the receptive field [TTC] (+3.0 s)
#define HAND_LEFT      1
#define FOREARM_LEFT   2
#define HAND_RIGHT     4
#define FOREARM_RIGHT  5
#define SKIN_THRES	  15 // Threshold with which a contact is detected

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
        taxelsFile = rf->check("taxelsFile", Value("taxels_2D.ini")).asString().c_str();
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
    // dataDumperPortOut.open(("/"+name+"/dataDumper:o").c_str());

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
        //for(unsigned int i=0;i<1;i++)
        {
            string filePath = filenames[i];
            printMessage(1,"i: %i filePath: %s\n",i,filePath.c_str());
            skinPart sP;
            if ( setTaxelPosesFromFile(filePath,sP) )
            {
                iCubSkin.push_back(sP);
            }
        }
        
        printMessage(0,"Loading from files..\n");
        load();

        printMessage(0,"iCubSkin correctly instantiated. Size: %i\n",iCubSkin.size());
        if (verbosity>= 2)
        {
            for (size_t i = 0; i < iCubSkin.size(); i++)
            {
                iCubSkin[i].print(verbosity);
            }
        }

        // for (size_t i = 0; i < 4; i++)
        // {
        //     H.push_back(skinPart());
        //     H[i].taxel.push_back(Taxel());
        // }

        // H[0].name =  "H0";
        // H[1].name =  "HN";
        // H[2].name = "EEL";
        // H[3].name = "EER";

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
    dumpedVector.resize(3,0.0);

    // process the port coming from the doubleTouch (for visualization)
    // if (dTBottle != NULL)
    // {
    //     H[0].taxel[0].Pos = matrixFromBottle(*dTBottle,4,4,4).subcol(0,3,3);
    //     H[1].taxel[0].Pos = matrixFromBottle(*dTBottle,20,4,4).subcol(0,3,3);

    //     if (currentTask == "R2L")
    //     {
    //         H[0].taxel[0].WRFPos=locateTaxel(H[0].taxel[0].Pos,"left_forearm");
    //         H[1].taxel[0].WRFPos=locateTaxel(H[1].taxel[0].Pos,"right_hand");
    //     }
    //     else if (currentTask == "L2R")
    //     {
    //         H[0].taxel[0].WRFPos=locateTaxel(H[0].taxel[0].Pos,"right_forearm");
    //         H[1].taxel[0].WRFPos=locateTaxel(H[1].taxel[0].Pos,"left_hand");
    //     }
    // }

    // project taxels in World Reference Frame
    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
        {
            iCubSkin[i].taxel[j].WRFPos=locateTaxel(iCubSkin[i].taxel[j].Pos,iCubSkin[i].name);
            printMessage(5,"iCubSkin[%i].taxel[%i].WRFPos %s\n",i,j,iCubSkin[i].taxel[j].WRFPos.toString().c_str());
        }
    }
    // H[2].taxel[0].WRFPos=locateTaxel(H[2].taxel[0].Pos,"left_hand");
    // H[3].taxel[0].WRFPos=locateTaxel(H[3].taxel[0].Pos,"right_hand");

    // process the port coming from the visuoTactileWrapper
    if (event != NULL)
    {
        // read the events
        for (size_t i = 0; i < event->size(); i++)
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
        {
            eventsBuffer.push_back(incomingEvents.back());
            printMessage(4,"I'm bufferizing! Size %i\n",eventsBuffer.size());
        }

        // limit the size of the buffer to 100, i.e. 5 seconds of acquisition
        if (eventsBuffer.size() >= 100)
        {
            eventsBuffer.erase(eventsBuffer.begin());
            printMessage(1,"Too many samples: removing the older element from the buffer..\n");
        }

        // detect contacts and train the taxels
        if (skinContacts && eventsBuffer.size()>20)
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
                {
                    printMessage(0,"No Learning has been put in place.\n");
                    dumpedVector.push_back(0);
                }
                eventsBuffer.clear();
            }
        }
        else
            dumpedVector.push_back(0);
    }
    // if there's no input for more than 2 seconds, clear the buffer
    else if (yarp::os::Time::now() - timeNow > 2)
    {
        eventsFlag = true;
        eventsBuffer.clear();
        timeNow = yarp::os::Time::now();
        printMessage(2,"No significant event in the last 2 seconds. Erasing the buffer.. \n");
    }
    
    // Superimpose the taxels onto the right eye
    if (imageInR!=NULL)
        drawTaxels("rightEye");
    else
        printMessage(5,"No imageInR!\n");

    // Superimpose the taxels onto the left eye
    if (imageInL!=NULL)
        drawTaxels("leftEye");
    else
        printMessage(5,"No imageInL!\n");
    
    if (incomingEvents.size()>0)
    {
        projectIncomingEvent();     // project event onto the taxels' RF
        computeResponse();          // compute the response of each taxel
    }
//For testing 
//     // project taxels in World Reference Frame
//       for (size_t i = 0; i < iCubSkin.size(); i++)
//       {
//          for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
//          {
//                iCubSkin[i].taxel[j].Resp=150; //testing
//          }
//     } 
 
    sendContactsToSkinGui();        // self explicative
    manageSkinEvents();
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
                    part   = iCubSkin[i].name;
                    iCubSkinID = i;
                }
                else
                    taxelsIDs.clear();
            }
        }

        // manage the dumped port
        if (dumpedVector.size()>0)
        {
            Bottle bd;
            bd.clear();
            
            vectorIntoBottle(dumpedVector,bd);
            dataDumperPortOut.write(bd);
        }
    }

    if (isThereAnEvent && taxelsIDs.size()>0)
    {
        Vector geoCenter(3,0.0), normalDir(3,0.0);
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
            for (size_t j = 0; j < iCubSkin[iCubSkinID].taxel.size(); j++)
            {
                if (iCubSkin[iCubSkinID].taxel[j].ID == taxelsIDs[i])
                {
                    geoCenter += iCubSkin[iCubSkinID].taxel[j].WRFPos;
                    normalDir += locateTaxel(iCubSkin[iCubSkinID].taxel[j].Norm,part);
                }
            }
        }

        geoCenter /= taxelsIDs.size();
        normalDir /= taxelsIDs.size();
        vectorIntoBottle(geoCenter,b);
        vectorIntoBottle(normalDir,b);
        skinPortOut.write(b);     // send something anyway (if there is no contact the bottle is empty)
    }


}

void vtRFThread::sendContactsToSkinGui()
{
    Vector respToSkin;

    for(size_t i=0; i<iCubSkin.size(); i++)
    {
        respToSkin.resize(iCubSkin[i].size,0.0);   // resize the vector to the skinPart

        if (incomingEvents.size()>0)
        {
            for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
            {
                if(iCubSkin[i].Repr2TaxelList.empty())
                {  
                	//we simply light up the taxels themselves
                    respToSkin[iCubSkin[i].taxel[j].ID] = iCubSkin[i].taxel[j].Resp*100/255;
                }
                else
                { 
                    //we light up all the taxels represented by the particular taxel
                    list<unsigned int> l = iCubSkin[i].Repr2TaxelList[iCubSkin[i].taxel[j].ID];

                    if (l.empty())
                    {
                        printMessage(0,"WARNING: skinPart %d Taxel %d : no list of represented taxels is available,	even if Repr2TaxelList is not empty\n",i,iCubSkin[i].taxel[j].ID);
                        respToSkin[iCubSkin[i].taxel[j].ID] = iCubSkin[i].taxel[j].Resp*100/255;
                    }
                    else
                    {
                        for(list<unsigned int>::const_iterator iter_list = l.begin(); iter_list != l.end(); iter_list++)
                        {
                        	//for all the represented taxels, we assign the activation of the super-taxel
                            respToSkin[*iter_list] =  iCubSkin[i].taxel[j].Resp*100/255;
                        } 
                    }
                }
            }
        }
        
        if(iCubSkin[i].name == "left_forearm")
        {
           skinGuiPortForearmL.write(respToSkin); 
        }
        else if(iCubSkin[i].name == "right_forearm")
        {
           skinGuiPortForearmR.write(respToSkin); 
        }
        else if(iCubSkin[i].name == "left_hand")
        {
            skinGuiPortHandL.write(respToSkin); 
        }
        else if(iCubSkin[i].name == "right_hand")
        {
            skinGuiPortHandR.write(respToSkin); 
        }
    }
}

bool vtRFThread::detectContact(iCub::skinDynLib::skinContactList *_sCL, int &_IDx,
                               std::vector <unsigned int> &_IDv)
{
    // Search for a suitable contact:
    for(iCub::skinDynLib::skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++)
    {
        if( it -> getPressure() > SKIN_THRES )
        {
            for (size_t i = 0; i < iCubSkin.size(); i++)
            {
                if ((it -> getSkinPart() == FOREARM_RIGHT && iCubSkin[i].name == "right_forearm") ||
                    (it -> getSkinPart() == FOREARM_LEFT  && iCubSkin[i].name == "left_forearm" ) ||
                    (it -> getSkinPart() ==    HAND_RIGHT && iCubSkin[i].name == "right_hand"   ) ||
                    (it -> getSkinPart() ==    HAND_LEFT  && iCubSkin[i].name == "left_hand"    )    )
                {
                    _IDx = i;
                    _IDv = it -> getTaxelList();

                    printMessage(1,"Contact! SkinPart %i Taxels' ID:",_IDx);
                    if (verbosity>=1)
                    {
                        for (size_t i = 0; i < _IDv.size(); i++)
                            printf("\t%i",_IDv[i]);

                        printf("\n");
                    }

                    return true; //TODO - in the future this should be modified to process all the skinContacts
                }
            }
        }
    }
    return false;
}

bool vtRFThread::load()
{
    string fileName=rf->findFile("taxelsFile").c_str();
    printMessage(0,"File loaded: %s\n", fileName.c_str());
    Property data; data.fromConfigFile(fileName.c_str());
    Bottle b; b.read(data);

    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        Bottle bb = b.findGroup(iCubSkin[i].name.c_str());

        if (bb.size() > 0)
        {
            int nTaxels = bb.find("nTaxels").asInt();
            int size    = bb.find("size").asInt();
            std::vector<double> extX;
            std::vector<double> extY;
            std::vector<int>    hSiz;
            std::vector<int>    mapp;

            Bottle *bbb;
            bbb = bb.find("extX").asList();
            extX.push_back(bbb->get(0).asDouble());
            extX.push_back(bbb->get(1).asDouble());
            bbb = bb.find("extY").asList();
            extY.push_back(bbb->get(0).asDouble());
            extY.push_back(bbb->get(1).asDouble());
            bbb = bb.find("binsNum").asList();
            hSiz.push_back(bbb->get(0).asInt());
            hSiz.push_back(bbb->get(1).asInt());
            bbb = bb.find("Mapping").asList();

            printMessage(6,"size %i\tnTaxels %i\textX %g  %g\n",size,nTaxels,extX[0],extX[1],extY[0],extY[1]);
            printMessage(6,"extY %g  %g\thSiz %i  %i\t\n",extY[0],extY[1],hSiz[0],hSiz[1]);
            printMessage(6,"mapp\n");
            for (size_t j = 0; j < size; j++)
            {
                mapp.push_back(bbb->get(j).asInt());
                printMessage(6,"%i ",mapp[j]);
            }
            printMessage(6,"\n");

            iCubSkin[i].size = size;
            iCubSkin[i].Taxel2Repr = mapp;

            for (size_t j = 0; j < nTaxels; j++)
            {
                bbb = bb.get(j+7).asList();
                iCubSkin[i].taxel[j].ID = bbb->get(0).asInt();
                iCubSkin[i].taxel[j].pwe2D.resize(extX,extY,hSiz);
                iCubSkin[i].taxel[j].pwe2D.setPosHist(matrixFromBottle(*bbb->get(1).asList(),0,hSiz[0],hSiz[1]));
                iCubSkin[i].taxel[j].pwe2D.setNegHist(matrixFromBottle(*bbb->get(2).asList(),0,hSiz[0],hSiz[1]));
            }
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
            std::vector<double> extX = iCubSkin[i].taxel[0].pwe2D.getExtX();
            std::vector<double> extY = iCubSkin[i].taxel[0].pwe2D.getExtY();
            std::vector<int>    hSiz = iCubSkin[i].taxel[0].pwe2D.getHistSize();

            myfile << "[" << iCubSkin[i].name << "]" << endl;
            myfile << "size\t"    << iCubSkin[i].size << endl;    
            myfile << "nTaxels\t" << iCubSkin[i].taxel.size() << endl;
            myfile << "extX\t( "  << extX[0] << "\t" << extX[1] << " )\n";
            myfile << "extY\t( "  << extY[0] << "\t" << extY[1] << " )\n";
            myfile << "binsNum\t( " << hSiz[0] << "\t" << hSiz[1] << " )\n";

            Bottle data;
            data.clear();
            Bottle &representatives = data.addList();
            for (size_t q = 0; q < iCubSkin[i].Taxel2Repr.size(); q++)
            {
                representatives.addInt(iCubSkin[i].Taxel2Repr[q]);
            } 
            myfile << "Mapping\t" << data.toString() << endl;

            for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
            {
                data.clear();
                Bottle &valuesPos = data.addList();
                std::vector<int>    hSiz = iCubSkin[i].taxel[j].pwe2D.getHistSize();

                for (size_t k = 0; k < hSiz[0]; k++)
                {
                    for (size_t kk = 0; kk < hSiz[1]; kk++)
                    {
                        valuesPos.addInt(iCubSkin[i].taxel[j].pwe2D.getPosHist(k,kk));
                    }
                }
                myfile << iCubSkin[i].taxel[j].ID << "\t\t" << data.toString() << "\t";

                data.clear();
                Bottle &valuesNeg = data.addList();

                for (size_t k = 0; k < hSiz[0]; k++)
                {
                    for (size_t kk = 0; kk < hSiz[1]; kk++)
                    {
                        valuesNeg.addInt(iCubSkin[i].taxel[j].pwe2D.getNegHist(k,kk));
                    }
                }
                myfile << data.toString() << endl;
            }
        }
    }
    myfile.close();
    return true;
}

bool vtRFThread::trainTaxels(const std::vector<unsigned int> IDv, const int IDx)
{
    std::vector<unsigned int> v; 					//vector of taxel IDs activated on the skin
    getRepresentativeTaxelsToTrain(IDv, IDx, v);
    //otherwise, do simply:  //v = IDv;

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

    for (int w = 0; w < v.size(); w++)
    {
        for (size_t j = 0; j < iCubSkin[IDx].taxel.size(); j++)
        {
            if (iCubSkin[IDx].taxel[j].ID == v[w])
            {
                for (size_t k = 0; k < eventsBuffer.size(); k++)
                {
                    IncomingEvent4Taxel projection = projectIntoTaxelRF(iCubSkin[IDx].taxel[j].RF,T_a,eventsBuffer[k]);
                    printMessage(3,"Training Taxels: skinPart %d ID %i k %i NORM %g TTC %g\n",IDx,iCubSkin[IDx].taxel[j].ID,k,projection.NRM,projection.TTC);
                    iCubSkin[IDx].taxel[j].addSample(projection);
                    dumpedVector.push_back(1);
                }
            }
            else
            {
                for (size_t k = 0; k < eventsBuffer.size(); k++)
                {
                    IncomingEvent4Taxel projection = projectIntoTaxelRF(iCubSkin[IDx].taxel[j].RF,T_a,eventsBuffer[k]);
                    printMessage(3,"Remove Taxels: skinPart %d ID %i k %i NORM %g TTC %g\n",IDx,iCubSkin[IDx].taxel[j].ID,k,projection.NRM,projection.TTC);
                    iCubSkin[IDx].taxel[j].removeSample(projection);
                    dumpedVector.push_back(-1);
                }
            }
        }
    }

    // for (size_t j = 0; j < iCubSkin[IDx].taxel.size(); j++)
    // {
    //     for (size_t k = 0; k < eventsBuffer.size(); k++)
    //     {
    //         printf("before %i %i\t",j,k);
    //         IncomingEvent4Taxel projection = projectIntoTaxelRF(iCubSkin[IDx].taxel[j].RF,T_a,eventsBuffer[k]);
    //         printf("after\t");

    //         bool flag = false;
    //         for (size_t w = 0; w < v.size(); w++)
    //         {
    //            //these nested for-loops can be made more efficient
    //             if (iCubSkin[IDx].taxel[j].ID == v[w])
    //             {
    //                 flag = true;
    //             }
    //         }

    //         if (flag)
    //         {

    //         }
    //         else
    //         {
    //             printMessage(7,"Removing Taxels: ID %i k %i NORM %g TTC %g\n",iCubSkin[IDx].taxel[j].ID,k,projection.NRM,projection.TTC);
    //             iCubSkin[IDx].taxel[j].removeSample(projection);
    //         }

    //         // if (flag)
    //         // {
    //         //     printMessage(2,"Training Taxels: ID %i k %i norm(pos) %g eventsBuffer(k) %s\n",iCubSkin[IDx].taxel[j].ID,k,norm(projection.Pos),eventsBuffer[k].Pos.toString().c_str());
    //         //     iCubSkin[IDx].taxel[j].pwe.addSample(norm(projection.Pos));
    //         // }
    //         // // a negative sample is added only if
    //         // // 1. the z is positive
    //         // // 2. the x and the y are between -75% and +75% of the RF's extension (i.e. 15cm)
    //         // else if (!flag && projection.Pos[2]>=0 && 
    //         //           projection.Pos[0] >= - iCubSkin[IDx].taxel[j].pwe.getExt() * 75 /100 &&
    //         //           projection.Pos[0] <=   iCubSkin[IDx].taxel[j].pwe.getExt() * 75 /100 &&
    //         //           projection.Pos[1] >= - iCubSkin[IDx].taxel[j].pwe.getExt() * 75 /100 &&
    //         //           projection.Pos[1] <=   iCubSkin[IDx].taxel[j].pwe.getExt() * 75 /100 )
    //         // {
    //         //     iCubSkin[IDx].taxel[j].pwe.removeSample(norm(projection.Pos));
    //         // }
    //         printf("end\n");
    //     }
    //     printf("qqqqasdfoijasdio\n");
    // }

    return true;
}

bool vtRFThread::projectIncomingEvent()
{
    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        Matrix T_a = eye(4);               // transform matrix relative to the arm
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
            iCubSkin[i].taxel[j].Evnt=projectIntoTaxelRF(iCubSkin[i].taxel[j].RF,T_a,
                                                         incomingEvents[incomingEvents.size()-1]);
            dumpedVector=iCubSkin[i].taxel[j].Evnt.Pos;


            // if (j==100)
            // {
                printMessage(5,"Projection -> i: %i\tID %i\tEvent: ",i,j);
                if (verbosity>=5)
                    iCubSkin[i].taxel[j].Evnt.print();
            // }
        }
    }
    return true;
}

IncomingEvent4Taxel vtRFThread::projectIntoTaxelRF(const Matrix &RF,const Matrix &T_a,const IncomingEvent &e)
{
    IncomingEvent4Taxel Event_projected = e;

    Matrix T_a_proj = T_a * RF;

    Vector p=e.Pos; p.push_back(1);
    Vector v=e.Vel; v.push_back(1);

    Event_projected.Pos = SE3inv(T_a_proj)*p;        Event_projected.Pos.pop_back();
    Event_projected.Vel = SE3inv(T_a_proj)*v;        Event_projected.Vel.pop_back();

    if (e.Radius != -1.0)
    {
        Event_projected.Pos(2) -= Event_projected.Radius;
    }
    computeX(Event_projected);

    return Event_projected;
}

// Vector vtRFThread::projectIntoTaxelRF(const Matrix &RF,const Matrix &T_a,const Vector &wrfpos)
// {
//     Matrix T_a_proj = T_a * RF;

//     Vector p=wrfpos;
//     p.push_back(1);
//     p = SE3inv(T_a_proj)*p;
//     p.pop_back();

//     return p;
// }

bool vtRFThread::computeX(IncomingEvent4Taxel &ie)
{
    int sgn = ie.Pos[2]>=0?1:-1;
    ie.NRM = sgn * norm(ie.Pos);
    // printf("ie.Vel %g\n", norm(ie.Vel));

    if (norm(ie.Vel) < 0.4)
    {
        ie.TTC = 10000.0;
    }
    else
    {
        ie.TTC = -norm(ie.Pos)*norm(ie.Pos)/dot(ie.Pos,ie.Vel);
    }

    return true;
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
    // projectIntoImagePlane(       H,_eye);

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
        printMessage(0,"ERROR in drawTaxels! Returning..\n");
        return;
    }

    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        for (size_t j = 0; j < iCubSkin[i].taxel.size(); j++)
        {
            drawTaxel(imgOut,iCubSkin[i].taxel[j].px,iCubSkin[i].name,iCubSkin[i].taxel[j].Resp);
            printMessage(6,"iCubSkin[%i].taxel[%i].px %s\n",i,j,iCubSkin[i].taxel[j].px.toString().c_str());
        }
    }

    // for (size_t i = 0; i < H.size(); i++)
    // {
    //     drawTaxel(imgOut,H[i].taxel[0].px,H[i].name,H[i].taxel[0].Resp);
    // }

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


bool vtRFThread::projectIntoImagePlane(vector <skinPart> &sP, const string &eye)
{
    for (size_t i = 0; i < sP.size(); i++)
    {
        for (size_t j = 0; j < sP[i].taxel.size(); j++)
        {
            if (eye=="rightEye" || eye=="leftEye")
                projectPoint(sP[i].taxel[j].WRFPos,sP[i].taxel[j].px,eye);
            else
                printMessage(0,"ERROR in projectIntoImagePlane!\n");
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
bool vtRFThread::setTaxelPosesFromFile(const string filePath, skinPart &sP)
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

        // the NULL taxels will be automatically discarded - most skin patches are not full (192 taxels) and are padded with 0s
        //if (norm(taxelNorm) != 0 || norm(taxelPos) != 0)
        //{
            // if (i>100 && i<104)
            // {
         //   if (i%10 == 1) // Take 1 out of 10
        if (sP.name == "left_forearm" || sP.name == "right_forearm")
        {
            // the taxels at the centers of respective triangles [note that i == taxelID == (line in the .txt file +1)]
            // e.g. first triangle of upper arm is at lines 1-12, center at line 4, thus i=2 
            if((i==3) || (i==15)  || (i==27)  || (i==39)  || (i==51)  || (i==63)  || (i==75)  || (i==87)  || 
              (i==99) || (i==111) || (i==123) || (i==135) || (i==147) || (i==159) || (i==171) || (i==183) || //upper patch ends here 
              (i==207)|| (i==255) || (i==291) || (i==303) || (i==315) || (i==339) || (i==351)) //lower patch
            {
                sP.size++;
                sP.taxel.push_back(Taxel(taxelPos,taxelNorm,i));
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
                sP.taxel.push_back(Taxel(taxelPos,taxelNorm,i));
            }
            else
            {
                sP.size++;
            }
        }
        else if (sP.name == "right_hand")
        { //right hand has different taxel nr.s than left hand 
            if((i==101))// || (i==103) || (i==124) || (i==118) || (i==137))  //only one taxel
            {
                sP.size++;
                sP.taxel.push_back(Taxel(taxelPos,taxelNorm,i));
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
              

bool vtRFThread::getRepresentativeTaxelsToTrain(const std::vector<unsigned int> IDv, const int IDx, std::vector<unsigned int> &v)
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
                printMessage(0,"ERROR: [%s] taxel %u activated, but representative taxel undefined - ignoring.\n",iCubSkin[IDx].name.c_str(),*it);
            }
            else
            {
                //add all the representatives that were activated to the set
                rep_taxel_IDs_set.insert(iCubSkin[IDx].Taxel2Repr[*it]);
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
