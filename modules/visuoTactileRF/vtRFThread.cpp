#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iomanip>

#include "vtRFThread.h"

#define RADIUS              2 // Radius in px of every taxel (in the images)
#define SKIN_THRES	        7 // Threshold with which a contact is detected

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
    ppsEventsPortOut.open(("/"+name+"/pps_events_aggreg:o").c_str());
    dataDumperPortOut.open(("/"+name+"/dataDumper:o").c_str());

    /**
    * I know that I should remove this but it's harmless (and I'm overly lazy)
    **/
     /*   if (robot=="icub")
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
       */
        
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
                yError("[vtRFThread] : could not open right_arm PolyDriver!\n");
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
                yError("[vtRFThread] : could not open left_arm PolyDriver!\n");
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
            yError("[vtRFThread] Could not open torso PolyDriver!");
            return false;
        }
        bool ok = 1;
        if (ddT.isValid())
        {
            ok = ok && ddT.view(iencsT);
        }
        if (!ok)
        {
            yError("[vtRFThread] Problems acquiring head interfaces!!!!");
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
            yError("[vtRFThread] could not open head PolyDriver!");
            return false;
        }
        ok = 1;
        if (ddH.isValid())
        {
            ok = ok && ddH.view(iencsH);
        }
        if (!ok)
        {
            yError("[vtRFThread] Problems acquiring head interfaces!!!!");
            return false;
        }
        iencsH->getAxes(&jntsH);
        encsH = new yarp::sig::Vector(jntsH,0.0);

    /**************************/
        // try
        // {
        //     Taxel *base = new Taxel();
        //     Taxel *derived = new TaxelPWE1D();
        //     TaxelPWE1D *casted;

        //     casted = dynamic_cast<TaxelPWE1D*>(derived);
        //     if (casted==0) cout << "Null pointer on first type-cast.\n";

        //     casted = dynamic_cast<TaxelPWE1D*>(base);
        //     if (casted==0) cout << "Null pointer on second type-cast.\n";
        // }
        // catch (exception& e)
        // {
        //     cout << "Exception: " << e.what() << endl;
        // }
    /**************************/
        // Vector taxelPos(3,0.5);
        // Vector taxelNrm(3,0.2);

        // std::vector<Taxel*> taxels;
        // taxels.push_back(new TaxelPWE1D(taxelPos,taxelNrm,2));

        // printf("*********\n\n*********\n");
        // taxels[0] -> print(10);
        // printf("*********\n\n*********\n");
        // TaxelPWE1D *t  = dynamic_cast<TaxelPWE1D*>(taxels[0]);
        // t -> print();
        // taxels[0] -> setID(199);
        // printf("*********\n\n*********\n");
        // t -> print(10);
        // printf("*********\n\n*********\n");
    /**************************/
        yDebug("Setting up iCubSkin...");
        iCubSkinSize = filenames.size();

        for(unsigned int i=0;i<filenames.size();i++)
        {
            string filePath = filenames[i];
            yDebug("i: %i filePath: %s",i,filePath.c_str());
            skinPartPWE sP(modality);
            if ( setTaxelPosesFromFile(filePath,sP) )
            {
                iCubSkin.push_back(sP);
            }
        }
        load();
            // printf("*********\n\n*********\n");
            // iCubSkin[0].print();
            // printf("*********\n\n*********\n");

            // TaxelPWE1D *tdc = dynamic_cast<TaxelPWE1D*>(iCubSkin[0].taxels[0]);
            // cout << tdc << endl;
            // if (tdc)
            // {
            //     cout << tdc->pwe << endl;
            //     // cout << *tdc << endl;
            //     tdc -> print(5);
            //     // TaxelPWE1D *tpwe = dynamic_cast<TaxelPWE1D*>(iCubSkin[0].taxels[0]);
            //     printf("\n\n\n\n\n");
            // }

        yInfo("iCubSkin correctly instantiated. Size: %i",iCubSkin.size());
        if (verbosity>= 2)
        {
            for (size_t i = 0; i < iCubSkin.size(); i++)
            {
                iCubSkin[i].print(verbosity);
            }
        }
        iCubSkinSize = iCubSkin.size();

    return true;
}

void vtRFThread::run()
{
    // read from the input ports
    dTBottle                       = dTPort       -> read(false);
    event                          = eventsPort   -> read(false);
    imageInR                       = imagePortInR -> read(false);
    imageInL                       = imagePortInL -> read(false); 
    skinContactList *skinContacts  = skinPortIn   -> read(false);

    dumpedVector.resize(0,0.0);

    //not used anymore
    /*
    // project taxels in World Reference Frame
    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
        {
            iCubSkin[i].taxels[j]->setWRFPosition(locateTaxel(iCubSkin[i].taxels[j]->getPosition(),iCubSkin[i].name));
            printMessage(7,"iCubSkin[%i].taxels[%i].WRFPos %s\n",i,j,iCubSkin[i].taxels[j]->getWRFPosition().toString().c_str());
        }
    }*/

    Bottle inputEvents;
    inputEvents.clear();

    if (event == NULL)
    {
        // if there is nothing from the port but there was a previous event,
        // and it did not pass more than 0.2 seconds from the last data, let's use that
        if ((yarp::os::Time::now() - timeNow <= 0.2) && incomingEvents.size()>0)
        {
            Bottle &b = inputEvents.addList();
            b = incomingEvents.back().toBottle();
        }
    }
    else
    {
        timeNow     = yarp::os::Time::now();
        inputEvents = *event;
    }

    ts.update();
    incomingEvents.clear();

    // process the port coming from the visuoTactileWrapper
    if (inputEvents.size() != 0)
    {
        // read the events
        for (size_t i = 0; i < inputEvents.size(); i++)
        {
            incomingEvents.push_back(IncomingEvent(*(inputEvents.get(i).asList())));
            printMessage(3,"[EVENT] %s\n", incomingEvents.back().toString().c_str());
        }

        // manage the buffer
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
                    size_t taxelsize = iCubSkin[0].taxels.size();
                    for (size_t j = 0; j < taxelsize; j++)
                    {
                        dumpedVector.push_back(0.0);
                    }
                }
                eventsBuffer.clear();
            }
            else
            {
                size_t taxelsize = iCubSkin[0].taxels.size();
                for (size_t j = 0; j < taxelsize; j++)
                {
                    dumpedVector.push_back(0.0);
                }
            }
        }
        else
        {
            size_t taxelsize = iCubSkin[0].taxels.size();
            for (size_t j = 0; j < taxelsize; j++)
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

        for (size_t j = 0; j < iCubSkin[0].taxels.size(); j++)
        {
            dumpedVector.push_back(0.0);
        }
    }
    else
    {
        for (size_t j = 0; j < iCubSkin[0].taxels.size(); j++)
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
    string part = SkinPart_s[SKIN_PART_UNKNOWN];
    int iCubSkinID=-1;
    bool isThereAnEvent = false;

    Bottle & out = ppsEventsPortOut.prepare();     out.clear();
    Bottle b;     b.clear();

    if (incomingEvents.size()>0)  // if there's an event
    {
        for (size_t i = 0; i < iCubSkinSize; i++) // cycle through the skinparts
        {
            b.clear(); //so there will be one bottle per skin part (if there was a significant event)
            taxelsIDs.clear();
            isThereAnEvent = false;

            //take only highly activated "taxels"
            for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++) // cycle through the taxels
            {
                if (dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp > 50)
                {
                    taxelsIDs.push_back(iCubSkin[i].taxels[j]->getID());
                    isThereAnEvent = true;
                }
            }
            if (isThereAnEvent && taxelsIDs.size()>0)
            {
                Vector geoCenter(3,0.0), normalDir(3,0.0);
                int w = 0;
                int w_max = 0;
                int w_sum = 0;
                part  = iCubSkin[i].name;

                //the output format on the port will be:
                //(SkinPart_enum x y z n1 n2 n3 magnitude SkinPart_string)
                //paralleling the one produced in skinEventsAggregator skinEventsAggregThread::run()
               
                b.addInt(getSkinPartFromString(iCubSkin[i].name));
                
                for (size_t k = 0; k < taxelsIDs.size(); k++)
                {
                    for (size_t p = 0; p < iCubSkin[i].taxels.size(); p++) //these two loops are not an efficient implementation
                    {
                        if (iCubSkin[i].taxels[p]->getID() == taxelsIDs[k])
                        {
                            w = dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[p])->Resp;
                            printMessage(4,"part %s: pps taxel ID %d, pos (%s), activation: %d\n",part.c_str(),taxelsIDs[k],iCubSkin[i].taxels[p]->getPosition().toString().c_str(),w);
                            //geoCenter += iCubSkin[i].taxels[p]->getWRFPosition()*w; //original code
                            //The final geoCenter and normalDir will be a weighted average of the activations
                            geoCenter += iCubSkin[i].taxels[p]->getPosition()*w; //Matej, 24.2., changing convention - link not root FoR
                            normalDir += locateTaxel(iCubSkin[i].taxels[p]->getNormal(),part)*w;
                            w_sum += w;
                            if (w>w_max)
                                w_max = w;
                        }
                    }
                }

                geoCenter /= w_sum;
                normalDir /= w_sum;
                vectorIntoBottle(geoCenter,b);
                vectorIntoBottle(normalDir,b);
                b.addInt(w_max);
                b.addString(part);
                out.addList().read(b);
            }
        }
   
         ppsEventsPortOut.setEnvelope(ts);
         ppsEventsPortOut.write();     // let's send only if there was en event
    }

   
}

void vtRFThread::sendContactsToSkinGui()
{
    Vector respToSkin;

    for(size_t i=0; i<iCubSkinSize; i++)
    {
        respToSkin.resize(iCubSkin[i].size,0.0);   // resize the vector to the skinPart

        if (incomingEvents.size()>0)
        {
            for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
            {
                if(iCubSkin[i].repr2TaxelList.empty())
                {  
                    //we simply light up the taxels themselves
                    respToSkin[iCubSkin[i].taxels[j]->getID()] = dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp;
                }
                else
                { 
                    //we light up all the taxels represented by the particular taxel
                    list<unsigned int> l = iCubSkin[i].repr2TaxelList[iCubSkin[i].taxels[j]->getID()];

                    if (l.empty())
                    {
                        yWarning("skinPart %d Taxel %d : no list of represented taxels is available, even if repr2TaxelList is not empty",i,iCubSkin[i].taxels[j]->getID());
                        respToSkin[iCubSkin[i].taxels[j]->getID()] = dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp;
                    }
                    else
                    {
                        for(list<unsigned int>::const_iterator iter_list = l.begin(); iter_list != l.end(); iter_list++)
                        {
                            //for all the represented taxels, we assign the activation of the super-taxel
                            respToSkin[*iter_list] =  dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp;
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
        if(iCubSkin[i].name == SkinPart_s[SKIN_LEFT_FOREARM])
        {
            outPort = &skinGuiPortForearmL; 
        }
        else if(iCubSkin[i].name == SkinPart_s[SKIN_RIGHT_FOREARM])
        {
            outPort = &skinGuiPortForearmR; 
        }
        else if(iCubSkin[i].name == SkinPart_s[SKIN_LEFT_HAND])
        {
            outPort = &skinGuiPortHandL; 
        }
        else if(iCubSkin[i].name == SkinPart_s[SKIN_RIGHT_HAND])
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
                if (SkinPart_s[it -> getSkinPart()] == iCubSkin[i].name)
                {
                    idx = i;
                    std::vector <unsigned int> txlList = it -> getTaxelList();

                    bool itHasBeenTouched = false;

                    getRepresentativeTaxels(txlList, idx, idv);

                    if (idv.size()>0)
                    {
                        itHasBeenTouched = true;
                    }

                    if (itHasBeenTouched)
                    {
                        if (verbosity>=1)
                        {
                            printMessage(1,"Contact! Skin part: %s\tTaxels' ID: ",iCubSkin[i].name.c_str());
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
    rf->setVerbose(true);
    string fileName=rf->findFile("taxelsFile").c_str();
    rf->setVerbose(false);
    if (fileName=="")
    {
        yWarning("[vtRF::load] No filename has been found. Skipping..");
        string ret="";
        return ret;
    }

    yInfo("[vtRF::load] File loaded: %s", fileName.c_str());
    Property data; data.fromConfigFile(fileName.c_str());
    Bottle b; b.read(data);
    yDebug("[vtRF::load] iCubSkinSize %i",iCubSkinSize);

    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        Bottle bb = b.findGroup(iCubSkin[i].name.c_str());

        if (bb.size() > 0)
        {
            string modality = bb.find("modality").asString();
            int nTaxels     = bb.find("nTaxels").asInt();
            int size        = bb.find("size").asInt();

            iCubSkin[i].size     = size;
            iCubSkin[i].modality = modality;

            Matrix ext;
            std::vector<int>    bNum;
            std::vector<int>    mapp;

            Bottle *bbb;
            bbb = bb.find("ext").asList();
            if (modality=="1D")
            {
                ext = matrixFromBottle(*bbb,0,1,2);
            }
            else
            {
                ext = matrixFromBottle(*bbb,0,2,2);
            }
            
            bbb = bb.find("binsNum").asList();
            bNum.push_back(bbb->get(0).asInt());
            bNum.push_back(bbb->get(1).asInt());
            
            bbb = bb.find("Mapping").asList();
            yDebug("[vtRF::load][%s] size %i\tnTaxels %i\text %s\tbinsNum %i %i",iCubSkin[i].name.c_str(),size,
                                                  nTaxels,toVector(ext).toString(3,3).c_str(),bNum[0],bNum[1]);
            printMessage(3,"Mapping\n");
            for (size_t j = 0; j < size; j++)
            {
                mapp.push_back(bbb->get(j).asInt());
                if (verbosity>=3)
                {
                    printf("%i ",mapp[j]);
                }
            }
            if (verbosity>=3) printf("\n");
            iCubSkin[i].taxel2Repr = mapp;

            for (size_t j = 0; j < nTaxels; j++)
            {
                // 7 are the number of lines in the skinpart group that are not taxels
                bbb = bb.get(j+7).asList();
                printMessage(3,"Reading taxel %s\n",bbb->toString().c_str());

                for (int k = 0; k < iCubSkin[i].taxels.size(); k++)
                {
                    if (iCubSkin[i].taxels[k]->getID() == bbb->get(0).asInt())
                    {
                        if (dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[k])->pwe->resize(ext,bNum))
                        {
                            dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[k])->pwe->setPosHist(matrixFromBottle(*bbb->get(1).asList(),0,bNum[0],bNum[1]));
                            dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[k])->pwe->setNegHist(matrixFromBottle(*bbb->get(2).asList(),0,bNum[0],bNum[1]));
                        }
                    }
                }
            }
        }
    }

    return fileName;
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
            Bottle data;
            data.clear();

            Matrix           getExt = dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[0])->pwe->getExt();
            matrixIntoBottle(getExt,data);

            std::vector<int> bNum  = dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[0])->pwe->getHistSize();

            myfile << "[" << iCubSkin[i].name << "]" << endl;
            myfile << "modality\t"  << iCubSkin[i].modality << endl;
            myfile << "size    \t"  << iCubSkin[i].size << endl;    
            myfile << "nTaxels \t"  << iCubSkin[i].taxels.size() << endl;
            myfile << "ext     \t(" << data.toString() << ")\n";
            myfile << "binsNum \t(" << bNum[0] << "\t" << bNum[1] << ")\n";

            data.clear();
            Bottle &representatives = data.addList();
            for (size_t q = 0; q < iCubSkin[i].taxel2Repr.size(); q++)
            {
                representatives.addInt(iCubSkin[i].taxel2Repr[q]);
            } 
            myfile << "Mapping\t" << data.toString() << endl;

            for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
            {
                data.clear();
                data=dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->TaxelPWEIntoBottle();
                myfile << data.toString() << "\n";
            }
        }
    }
    myfile.close();
    return fnm;
}

bool vtRFThread::trainTaxels(const std::vector<unsigned int> IDv, const int IDx)
{
    std::vector<unsigned int> v;
    getRepresentativeTaxels(IDv, IDx, v);

    Matrix T_a = eye(4);                     // transform matrix relative to the arm
    if ((iCubSkin[IDx].name == SkinPart_s[SKIN_LEFT_FOREARM]) || (iCubSkin[IDx].name == SkinPart_s[SKIN_LEFT_HAND]))
    {
        iencsL->getEncoders(encsL->data());
        yarp::sig::Vector qL=encsL->subVector(0,6);
        armL -> setAng(qL*CTRL_DEG2RAD);
        if (iCubSkin[IDx].name == SkinPart_s[SKIN_LEFT_FOREARM]) 
            T_a = armL -> getH(3+4, true);
        else //(iCubSkin[i].name == SkinPart_s[SKIN_LEFT_HAND]) 
            T_a = armL -> getH(3+6, true);
    }
    else if ((iCubSkin[IDx].name == SkinPart_s[SKIN_RIGHT_FOREARM]) || (iCubSkin[IDx].name == SkinPart_s[SKIN_RIGHT_HAND]))
    {
        iencsR->getEncoders(encsR->data());
        yarp::sig::Vector qR=encsR->subVector(0,6);
        armR -> setAng(qR*CTRL_DEG2RAD);
        if (iCubSkin[IDx].name == SkinPart_s[SKIN_RIGHT_FOREARM])
            T_a = armR -> getH(3+4, true);
        else //(iCubSkin[i].name == SkinPart_s[SKIN_RIGHT_HAND]) 
            T_a = armR -> getH(3+6, true);
    }
    else
    {
        yError("[vtRFThread] in trainTaxels!\n");
        return false;
    }

    for (size_t j = 0; j < iCubSkin[IDx].taxels.size(); j++)
    {
        bool itHasBeenTouched = false;
        for (size_t w = 0; w < v.size(); w++)
        {
            if (iCubSkin[IDx].taxels[j]->getID() == v[w])
            {
                itHasBeenTouched = true;
            }
        }

        for (size_t k = 0; k < eventsBuffer.size(); k++)
        {
            IncomingEvent4TaxelPWE projection = projectIntoTaxelRF(iCubSkin[IDx].taxels[j]->getFoR(),T_a,eventsBuffer[k]);
            printMessage(3,"Training Taxels: skinPart %d ID %i k %i NORM %g TTC %g\n",IDx,iCubSkin[IDx].taxels[j]->getID(),k,projection.NRM,projection.TTC);

            if (itHasBeenTouched == true)  dynamic_cast<TaxelPWE*>(iCubSkin[IDx].taxels[j])->addSample(projection);
            else                           dynamic_cast<TaxelPWE*>(iCubSkin[IDx].taxels[j])->removeSample(projection);
        }

        if (itHasBeenTouched == true)   dumpedVector.push_back(1.0);
        else                            dumpedVector.push_back(-1.0);
    }

    return true;
}

bool vtRFThread::projectIncomingEvent()
{
    for (size_t k = 0; k < incomingEvents.size(); k++)
    {
        for (size_t i = 0; i < iCubSkinSize; i++)
        {
            Matrix T_a = eye(4);               // transform matrix relative to the arm
            if ((iCubSkin[i].name == SkinPart_s[SKIN_LEFT_FOREARM]) || (iCubSkin[i].name == SkinPart_s[SKIN_LEFT_HAND]))
            {
                iencsL->getEncoders(encsL->data());
                yarp::sig::Vector qL=encsL->subVector(0,6);
                armL -> setAng(qL*CTRL_DEG2RAD);
                if (iCubSkin[i].name == SkinPart_s[SKIN_LEFT_FOREARM]) 
                    T_a = armL -> getH(3+4, true);
                else //(iCubSkin[i].name == SkinPart_s[SKIN_LEFT_HAND]) 
                    T_a = armL -> getH(3+6, true);
            }
            else if ((iCubSkin[i].name == SkinPart_s[SKIN_RIGHT_FOREARM]) || (iCubSkin[i].name == SkinPart_s[SKIN_RIGHT_HAND]))
            {
                iencsR->getEncoders(encsR->data());
                yarp::sig::Vector qR=encsR->subVector(0,6);
                armR -> setAng(qR*CTRL_DEG2RAD);
                if (iCubSkin[i].name == SkinPart_s[SKIN_RIGHT_FOREARM])
                    T_a = armR -> getH(3+4, true);
                else //(iCubSkin[i].name == SkinPart_s[SKIN_RIGHT_HAND]) 
                    T_a = armR -> getH(3+6, true);
            }
            else
                yError("[vtRFThread] in projectIncomingEvent!\n");

            // yInfo("T_A:\n%s",T_a.toString().c_str());

            for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
            {
                dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Evnt=projectIntoTaxelRF(iCubSkin[i].taxels[j]->getFoR(),T_a,
                                                             incomingEvents[k]);

            // There's a reason behind this choice
            dumpedVector.push_back(dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Evnt.Pos[0]);
            dumpedVector.push_back(dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Evnt.Pos[1]);
            dumpedVector.push_back(dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Evnt.Pos[2]);

            printMessage(5,"Projection -> i: %i\tID %i\tEvent: %s\n",i,j,dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Evnt.toString().c_str());
            }
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
        for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
        {
            dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->resetParzenWindowEstimator();
        }
    }
}

bool vtRFThread::computeResponse()
{
    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
        {
            dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->computeResponse();
            printMessage(4,"\tID %i\tResponse %i\n",j,dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp);
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
        yError("[vtRFThread] Error in drawTaxels! Returning..");
        return;
    }

    for (size_t i = 0; i < iCubSkinSize; i++)
    {
        for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
        {
            drawTaxel(imgOut,iCubSkin[i].taxels[j]->getPx(),iCubSkin[i].name,dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp);
            printMessage(6,"iCubSkin[%i].taxels[%i].px %s\n",i,j,iCubSkin[i].taxels[j]->getPx().toString().c_str());
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
                if (part == SkinPart_s[SKIN_LEFT_FOREARM] || part == SkinPart_s[SKIN_RIGHT_FOREARM] ||
                    part == SkinPart_s[SKIN_LEFT_HAND]    || part == SkinPart_s[SKIN_RIGHT_HAND])
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
            }
        }
    }
}


bool vtRFThread::projectIntoImagePlane(vector <skinPartPWE> &sP, const string &eye)
{
    for (size_t i = 0; i < sP.size(); i++)
    {
        for (size_t j = 0; j < sP[i].taxels.size(); j++)
        {
            if (eye=="rightEye" || eye=="leftEye")
            {
                yarp::sig::Vector WRFpos = sP[i].taxels[j]->getWRFPosition();
                yarp::sig::Vector px     = sP[i].taxels[j]->getPx();
                projectPoint(WRFpos,px,eye);
                sP[i].taxels[j]->setPx(px);
            }
            else
            {
                yError("[vtRFThread] ERROR in projectIntoImagePlane!\n");
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

    if (part == SkinPart_s[SKIN_LEFT_FOREARM] || part == SkinPart_s[SKIN_LEFT_HAND])
    {
        iencsL->getEncoders(encsL->data());
        yarp::sig::Vector qL=encsL->subVector(0,6);
        armL -> setAng(qL*CTRL_DEG2RAD);
    }
    else if (part == SkinPart_s[SKIN_RIGHT_FOREARM] || part == SkinPart_s[SKIN_RIGHT_HAND])
    {
        iencsR->getEncoders(encsR->data());
        yarp::sig::Vector qR=encsR->subVector(0,6);
        armR -> setAng(qR*CTRL_DEG2RAD);
    }
    else
    {
        yError("[vtRFThread] locateTaxel() failed!\n");
    }

    if      (part == SkinPart_s[SKIN_LEFT_FOREARM] ) { T = armL -> getH(3+4, true); } // torso + up to elbow
    else if (part == SkinPart_s[SKIN_RIGHT_FOREARM]) { T = armR -> getH(3+4, true); } // torso + up to elbow
    else if (part == SkinPart_s[SKIN_LEFT_HAND])     { T = armL -> getH(3+6, true); } // torso + up to wrist
    else if (part == SkinPart_s[SKIN_RIGHT_HAND])    { T = armR -> getH(3+6, true); } // torso + up to wrist
    else    {  yError("[vtRFThread] locateTaxel() failed!\n"); }

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
    yarp::sig::Vector taxelNrm(3,0.0);
    yarp::sig::Vector taxelPosNrm(6,0.0);

    string filename = strrchr(filePath.c_str(), '/');
    filename = filename.c_str() ? filename.c_str() + 1 : filePath.c_str();

    if      (filename == "left_forearm_mesh.txt")    { sP.name = SkinPart_s[SKIN_LEFT_FOREARM]; }
    else if (filename == "left_forearm_nomesh.txt")  { sP.name = SkinPart_s[SKIN_LEFT_FOREARM]; }
    else if (filename == "right_forearm_mesh.txt")   { sP.name = SkinPart_s[SKIN_RIGHT_FOREARM]; }
    else if (filename == "right_forearm_nomesh.txt") { sP.name = SkinPart_s[SKIN_RIGHT_FOREARM]; }
    else if (filename == "left_hand_V2_1.txt")       { sP.name = SkinPart_s[SKIN_LEFT_HAND]; }
    else if (filename == "right_hand_V2_1.txt")      { sP.name = SkinPart_s[SKIN_RIGHT_HAND]; }
    else
    {
        yError("[vtRFThread] Unexpected skin part file name: %s.\n",filename.c_str());
        return false;
    }
    //filename = filename.substr(0, filename.find_last_of("_"));
       
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("skinGui");            //overridden by --context parameter
    rf.setDefaultConfigFile(filePath.c_str()); //overridden by --from parameter
    if (!rf.configure(0,NULL))
    {
        yError("[vtRFThread] ResourceFinder was not configured correctly! Filename:");
        yError("%s",filename.c_str());
        return false;
    }
    rf.setVerbose(true);

    yarp::os::Bottle &calibration = rf.findGroup("calibration");
    if (calibration.isNull())
    {
        yError("[vtRFThread::setTaxelPosesFromFile] No calibration group found!");
        return false;
    }
    printMessage(6,"[vtRFThread::setTaxelPosesFromFile] found %i taxels (not all of them are valid taxels).\n", calibration.size()-1);

    // First item of the bottle is "calibration", so we should not use it
    for (int i = 1; i < calibration.size()-1; i++)
    {
        taxelPosNrm = vectorFromBottle(*(calibration.get(i).asList()),0,6);
        taxelPos = taxelPosNrm.subVector(0,2);
        taxelNrm = taxelPosNrm.subVector(3,5);
        
        if (sP.name == SkinPart_s[SKIN_LEFT_FOREARM] || sP.name == SkinPart_s[SKIN_RIGHT_FOREARM])
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
                    sP.taxels.push_back(new TaxelPWE1D(taxelPos,taxelNrm,i));
                }
                else
                {
                    sP.taxels.push_back(new TaxelPWE2D(taxelPos,taxelNrm,i));
                }
            }
            else
            {
                sP.size++;
            }
        }
        else if (sP.name == SkinPart_s[SKIN_LEFT_HAND])
        { //we want to represent the 48 taxels of the palm (ignoring fingertips) with 5 taxels -
         // manually marking 5 regions of the palm and selecting their "centroids" as the representatives
            if((i==99) || (i==101) || (i==109) || (i==122) || (i==134)) 
            {
                sP.size++;
                if (modality=="1D")
                {
                    sP.taxels.push_back(new TaxelPWE1D(taxelPos,taxelNrm,i));
                }
                else
                {
                    sP.taxels.push_back(new TaxelPWE2D(taxelPos,taxelNrm,i));
                }
            }
            else
            {
                sP.size++;
            }
        }
        else if (sP.name == SkinPart_s[SKIN_RIGHT_HAND])
        { //right hand has different taxel nr.s than left hand 
            // if((i==101) || (i==103) || (i==118) || (i==137)) // || (i==124)) remove one taxel
            if((i==101) || (i==103) || (i==118) || (i==137)) // || (i==124)) remove one taxel
            {
                sP.size++;
                if (modality=="1D")
                {
                    sP.taxels.push_back(new TaxelPWE1D(taxelPos,taxelNrm,i));
                }
                else
                {
                    sP.taxels.push_back(new TaxelPWE2D(taxelPos,taxelNrm,i));
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
    printMessage(6,"[vtRFThread::initRepresentativeTaxels] Initializing representative taxels for %s\n",sP.name.c_str());
    int i=0;
    list<unsigned int> taxels_list;
    if (sP.name == SkinPart_s[SKIN_LEFT_FOREARM] || sP.name == SkinPart_s[SKIN_RIGHT_FOREARM])
    {
        for (i=0;i<sP.size;i++)
        {
            //4th taxel of each 12 is the triangle midpoint
            sP.taxel2Repr.push_back(((i/12)*12)+3); //initialize all 384 taxels with triangle center as the representative
            //fill a map of lists here somehow
        }
        
        // set to -1 the taxel2Repr for all the taxels that don't exist
        for (i=192;i<=203;i++)
        {
            sP.taxel2Repr[i]=-1; //these taxels don't exist
        }
        for (i=216;i<=251;i++)
        {
            sP.taxel2Repr[i]=-1; //these taxels don't exist
        }
        for (i=264;i<=287;i++)
        {
            sP.taxel2Repr[i]=-1; //these taxels don't exist
        }
        for (i=324;i<=335;i++)
        {
            sP.taxel2Repr[i]=-1; //these taxels don't exist
        }
        for (i=360;i<=383;i++)
        {
            sP.taxel2Repr[i]=-1; //these taxels don't exist
        }
        
        // Set up the inverse - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        for(i=0;i<=11;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[3] = taxels_list;
        
        taxels_list.clear(); 
        for(i=12;i<=23;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[15] = taxels_list;
        
        taxels_list.clear(); 
        for(i=24;i<=35;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[27] = taxels_list;
        
        taxels_list.clear(); 
        for(i=36;i<=47;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[39] = taxels_list;
        
        taxels_list.clear(); 
        for(i=48;i<=59;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[51] = taxels_list;
        
        taxels_list.clear(); 
        for(i=60;i<=71;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[63] = taxels_list;
        
        taxels_list.clear(); 
        for(i=72;i<=83;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[75] = taxels_list;
        
        taxels_list.clear(); 
        for(i=84;i<=95;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[87] = taxels_list;
        
        taxels_list.clear(); 
        for(i=96;i<=107;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[99] = taxels_list;
        
        taxels_list.clear(); 
        for(i=108;i<=119;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[111] = taxels_list;
        
        taxels_list.clear(); 
        for(i=120;i<=131;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[123] = taxels_list;
        
        taxels_list.clear(); 
        for(i=132;i<=143;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[135] = taxels_list;
        
        taxels_list.clear(); 
        for(i=144;i<=155;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[147] = taxels_list;
        
        taxels_list.clear(); 
        for(i=156;i<=167;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[159] = taxels_list;
        
        taxels_list.clear(); 
        for(i=168;i<=179;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[171] = taxels_list;
        
        taxels_list.clear(); 
        for(i=180;i<=191;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[183] = taxels_list;
        //up to here - upper (full) patch on forearm
        
        //from here - lower patch with many dummy taxels
        taxels_list.clear(); 
        for(i=204;i<=215;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[207] = taxels_list;
        
        taxels_list.clear(); 
        for(i=252;i<=263;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[255] = taxels_list;
        
        taxels_list.clear(); 
        for(i=288;i<=299;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[291] = taxels_list;
        
        taxels_list.clear(); 
        for(i=300;i<=311;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[303] = taxels_list;
        
        taxels_list.clear(); 
        for(i=312;i<=323;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[315] = taxels_list;
        
        taxels_list.clear(); 
        for(i=336;i<=347;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[339] = taxels_list;
        
        taxels_list.clear(); 
        for(i=348;i<=359;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[351] = taxels_list;
    }
    else if(sP.name == SkinPart_s[SKIN_LEFT_HAND])
    {
        for(i=0;i<sP.size;i++)
        {
            // Fill all the 192 with -1 - half of the taxels don't exist, 
            // and for fingertips we don't have positions either
            sP.taxel2Repr.push_back(-1); 
        }

        // Upper left area of the palm - at thumb
        for (i=121;i<=128;i++)
        {
            sP.taxel2Repr[i] = 122;
        }
        sP.taxel2Repr[131] = 122; //thermal pad

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        for(i=121;i<=128;i++)
        {
            taxels_list.push_back(i);
        }
        taxels_list.push_back(131);
        sP.repr2TaxelList[122] = taxels_list;

        // Upper center of the palm
        for (i=96;i<=99;i++)
        {
            sP.taxel2Repr[i] = 99;
        }
        sP.taxel2Repr[102] = 99;
        sP.taxel2Repr[103] = 99;
        sP.taxel2Repr[120] = 99;
        sP.taxel2Repr[129] = 99;
        sP.taxel2Repr[130] = 99;

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
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
        sP.repr2TaxelList[99] = taxels_list;

        // Upper right of the palm (away from the thumb)
        sP.taxel2Repr[100] = 101;
        sP.taxel2Repr[101] = 101;
        for (i=104;i<=107;i++)
        {
            sP.taxel2Repr[i] = 101; //N.B. 107 is thermal pad
        }
        sP.taxel2Repr[113] = 101;
        sP.taxel2Repr[116] = 101;
        sP.taxel2Repr[117] = 101;

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
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
        sP.repr2TaxelList[101] = taxels_list;

        // Center area of the palm
        for(i=108;i<=112;i++)
        {
            sP.taxel2Repr[i] = 109;
        }
        sP.taxel2Repr[114] = 109;
        sP.taxel2Repr[115] = 109;
        sP.taxel2Repr[118] = 109;
        sP.taxel2Repr[142] = 109;
        sP.taxel2Repr[143] = 109;

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
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
        sP.repr2TaxelList[109] = taxels_list;

        // Lower part of the palm
        sP.taxel2Repr[119] = 134; // this one is thermal pad
        for(i=132;i<=141;i++)
        {
            sP.taxel2Repr[i] = 134;
        }

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        taxels_list.push_back(119);
        for(i=132;i<=141;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[134] = taxels_list;
    }
    else if(sP.name == SkinPart_s[SKIN_RIGHT_HAND])
    {
       for(i=0;i<sP.size;i++)
       {
          sP.taxel2Repr.push_back(-1); //let's fill all the 192 with -1 - half of the taxels don't exist and for fingertips, 
          //we don't have positions either
       }
       //upper left area - away from thumb on this hand
        sP.taxel2Repr[96] = 101;
        sP.taxel2Repr[97] = 101;
        sP.taxel2Repr[98] = 101;
        sP.taxel2Repr[100] = 101;
        sP.taxel2Repr[101] = 101;
        sP.taxel2Repr[107] = 101; //thermal pad
        sP.taxel2Repr[110] = 101;
        sP.taxel2Repr[111] = 101;
        sP.taxel2Repr[112] = 101;
        
        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
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
        sP.repr2TaxelList[101] = taxels_list;
        
        //upper center of the palm
        sP.taxel2Repr[99] = 103;
        for(i=102;i<=106;i++)
        {
           sP.taxel2Repr[i] = 103;
        }
        sP.taxel2Repr[127] = 103;
        sP.taxel2Repr[129] = 103;
        sP.taxel2Repr[130] = 103;
        
        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        taxels_list.push_back(99);
        for(i=102;i<=106;i++)
        {
            taxels_list.push_back(i);
        }
        taxels_list.push_back(127);
        taxels_list.push_back(129);
        taxels_list.push_back(130);
        sP.repr2TaxelList[103] = taxels_list;
        
        
        //upper right center of the palm - at thumb
        for(i=120;i<=126;i++)
        {
           sP.taxel2Repr[i] = 124;
        }
        sP.taxel2Repr[128] = 124;
        sP.taxel2Repr[131] = 124; //thermal pad
        
        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        for(i=120;i<=126;i++)
        {
            taxels_list.push_back(i);
        }
        taxels_list.push_back(128);
        taxels_list.push_back(131);
        sP.repr2TaxelList[124] = taxels_list;
        
        
        //center of palm
        sP.taxel2Repr[108] = 118;
        sP.taxel2Repr[109] = 118;
        for(i=113;i<=118;i++)
        {
            sP.taxel2Repr[i] = 118;
        }
        sP.taxel2Repr[142] = 118;
        sP.taxel2Repr[143] = 118;
        
        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        taxels_list.push_back(108);
        taxels_list.push_back(109);
        for(i=113;i<=118;i++)
        {
            taxels_list.push_back(i);
        }
        taxels_list.push_back(142);
        taxels_list.push_back(143);
        sP.repr2TaxelList[118] = taxels_list;
            
        //lower palm
        sP.taxel2Repr[119] = 137; //thermal pad
        for(i=132;i<=141;i++)
        {
            sP.taxel2Repr[i] = 137; //139 is another thermal pad
        }
        
        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear(); 
        taxels_list.push_back(119);
        for(i=132;i<=141;i++)
        {
            taxels_list.push_back(i);
        }
        sP.repr2TaxelList[137] = taxels_list;
    }
}

bool vtRFThread::getRepresentativeTaxels(const std::vector<unsigned int> IDv, const int IDx, std::vector<unsigned int> &v)
{
    //unordered_set would be better, but that is only experimentally supported by some compilers.
    std::set<unsigned int> rep_taxel_IDs_set;
    
    if (iCubSkin[IDx].taxel2Repr.empty())
    {
        v = IDv; //we simply copy the activated taxels
        return false;
    }
    else
    {
        for (std::vector<unsigned int>::const_iterator it = IDv.begin() ; it != IDv.end(); ++it)
        {
            if (iCubSkin[IDx].taxel2Repr[*it] == -1)
            {
                yWarning("[%s] taxel %u activated, but representative taxel undefined - ignoring.",iCubSkin[IDx].name.c_str(),*it);
            }
            else
            {
                rep_taxel_IDs_set.insert(iCubSkin[IDx].taxel2Repr[*it]); //add all the representatives that were activated to the set
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
            printMessage(4,"Representative taxels on skin part %d: ",IDx);
            for(std::vector<unsigned int>::const_iterator it = v.begin() ; it != v.end(); ++it)
            {
                printf("%d ",*it);
            }
            printf("\n");
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
    yDebug("[vtRF::threadRelease]Saving taxels..\n");
        save();

    
    yDebug("[vtRF::threadRelease]Closing controllers..\n");
        ddR.close();
        ddL.close();
        ddT.close();
        ddH.close();

    yDebug("[vtRF::threadRelease]Deleting misc stuff..\n");
        delete armR;
        armR = NULL;
        delete armL;
        armL = NULL;
        delete eWR;
        eWR  = NULL;
        delete eWL;
        eWL  = NULL;

    yDebug("[vtRF::threadRelease]Closing ports..\n");
        closePort(imagePortInR);
        yDebug("  imagePortInR successfully closed!\n");
        closePort(imagePortInL);
        yDebug("  imagePortInL successfully closed!\n");

        // closePort(imagePortOutR);
        imagePortOutR.interrupt();
        imagePortOutR.close();
        yDebug("  imagePortOutR successfully closed!\n");
        // closePort(imagePortOutL);
        imagePortOutL.interrupt();
        imagePortOutL.close();
        yDebug("  imagePortOutL successfully closed!\n");

        closePort(dTPort);
        yDebug("  dTPort successfully closed!\n");
        closePort(eventsPort);
        yDebug("  eventsPort successfully closed!\n");

        closePort(skinPortIn);
        yDebug("  skinPortIn successfully closed!\n");

        ppsEventsPortOut.interrupt();
        ppsEventsPortOut.close();
        yDebug("ppsEventsPortOut successfully closed!\n");
        
        // closePort(skinGuiPortForearmL);
        skinGuiPortForearmL.interrupt();
        skinGuiPortForearmL.close();
        yDebug("  skinGuiPortForearmL successfully closed!\n");
        // closePort(skinGuiPortForearmR);
        skinGuiPortForearmR.interrupt();
        skinGuiPortForearmR.close();
        yDebug("  skinGuiPortForearmR successfully closed!\n");
        // closePort(skinGuiPortHandL);
        skinGuiPortHandL.interrupt();
        skinGuiPortHandL.close();
        yDebug("  skinGuiPortHandL successfully closed!\n");
        // closePort(skinGuiPortHandR);
        skinGuiPortHandR.interrupt();
        skinGuiPortHandR.close();
        yDebug("  skinGuiPortHandR successfully closed!\n");
    yInfo("[vtRF::threadRelease] done.\n");
}

// empty line to make gcc happy
