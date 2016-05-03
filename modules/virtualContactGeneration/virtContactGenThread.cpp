#include "virtContactGenThread.h"
#include <yarp/sig/Image.h>

//see also Compensator::setTaxelPosesFromFile in icub-main/src/modules/skinManager/src/compensator.cpp
//see also dbool vtRFThread::setTaxelPosesFromFile(const string filePath, skinPartPWE &sP)
int virtContactGenerationThread::initSkinParts()
{
    SkinPart skin_part_name; 
    int OFFSET = 4; //the taxel pos files have as of May 2015 first 4 lines with metainfo
       
    string line;
    ifstream posFile;
    yarp::sig::Vector taxelPos(3,0.0);
    yarp::sig::Vector taxelNorm(3,0.0);
   
    string filename; 
    //go through skin parts and initialize them
    for (std::vector<SkinPart>::const_iterator it = activeSkinPartsNames.begin() ; it != activeSkinPartsNames.end(); ++it)
    {
        skin_part_name = *it;
        iCub::skinDynLib::skinPart skinPartWithTaxels; 
        
        // Open File
        posFile.open(skinPartPosFilePaths[skin_part_name].c_str());  
        if (!posFile.is_open())
        {
           yWarning("[virtContactGenerationThread] File %s has not been opened!",skinPartPosFilePaths[skin_part_name].c_str());
           return false;
        }
        printMessage(4,"[virtContactGenerationThread] Initializing %s from %s.\n",
                        SkinPart_s[skin_part_name].c_str(),skinPartPosFilePaths[skin_part_name].c_str());
        posFile.clear(); 
        posFile.seekg(0, std::ios::beg);//rewind iterator
        
        switch(skin_part_name)
        {
            case SKIN_LEFT_HAND:
            case SKIN_RIGHT_HAND:
                for(int i= 0; getline(posFile,line); i++)
                {
                    line.erase(line.find_last_not_of(" \n\r\t")+1);
                    if((line.empty()) || (i<OFFSET)) //skip empty lines and first 4 lines
                        continue;
                    string number;
                    istringstream iss(line, istringstream::in);
                    taxelPos.zero(); taxelNorm.zero();
                    for(unsigned int j = 0; iss >> number; j++ )
                    {
                        if(j<3)
                            taxelPos[j] = strtod(number.c_str(),NULL);
                        else
                            taxelNorm[j-3] = strtod(number.c_str(),NULL);
                    }
                    skinPartWithTaxels.setSize(skinPartWithTaxels.getSize()+1); //this is incremented for all lines - size of "port"
                    //all palm taxels, without thermal pads
                    if((i>=96+OFFSET) && (i<=143+OFFSET) && (i!=107+OFFSET) && (i!=119+OFFSET) && (i!=131+OFFSET) && (i!=139+OFFSET))
                    {
                        skinPartWithTaxels.taxels.push_back(new Taxel(taxelPos,taxelNorm,i-OFFSET));
                        printMessage(10,"[virtContactGenerationThread] Pushing taxel ID:%d, pos:%f %f %f; norm:%f %f %f.\n",
                                         i-OFFSET,taxelPos[0],taxelPos[1],taxelPos[2],taxelNorm[0],taxelNorm[1],taxelNorm[2]);
                    }
                }
                if(skinPartWithTaxels.getSize() != 192)
                {
                    yWarning("[virtContactGenerationThread]::initSkinParts():initalizing %s from file, 192 positions expected, but %d present.\n",
                                                                                 SkinPart_s[skin_part_name].c_str(),skinPartWithTaxels.getSize());
                }
                skinPartWithTaxels.name = skin_part_name;
                if (skin_part_name == SKIN_LEFT_HAND)
                {
                   activeSkinParts[SKIN_LEFT_HAND] = skinPartWithTaxels;
                   printMessage(4,"[virtContactGenerationThread]Adding SKIN_LEFT_HAND to activeSkinParts, it now has %d members.\n",activeSkinParts.size());
                }
                else    // skin_part_name == SKIN_RIGHT_HAND
                {   
                    activeSkinParts[SKIN_RIGHT_HAND] = skinPartWithTaxels;
                    printMessage(4,"[virtContactGenerationThread]Adding SKIN_RIGHT_HAND to activeSkinParts, it now has %d members.\n",activeSkinParts.size());
                }
                break;
                
            case SKIN_LEFT_FOREARM:
            case SKIN_RIGHT_FOREARM:

                for(int i= 0; getline(posFile,line); i++)
                {
                    line.erase(line.find_last_not_of(" \n\r\t")+1);
                    if(line.empty() || (i<OFFSET)) //skip empty lines and first 4 lines
                        continue;
                    string number;
                    istringstream iss(line, istringstream::in);
                    taxelPos.zero(); taxelNorm.zero();
                    for(unsigned int j = 0; iss >> number; j++ )
                    {
                        if(j<3)
                            taxelPos[j] = strtod(number.c_str(),NULL);
                        else
                            taxelNorm[j-3] = strtod(number.c_str(),NULL);
                    }
                    skinPartWithTaxels.setSize(skinPartWithTaxels.getSize()+1); //this is incremented for all lines - size of "port"

                    if((i>=0+OFFSET) && (i<=191+OFFSET)) //first patch - full one, 16 triangles, lower part of forearm (though in Marco's files, it is called upper)
                    {
                        if( (((i-OFFSET) % 12) != 6) && (((i-OFFSET) % 12) != 10))
                        { //every 7th and 11th taxel of a triangle are thermal pads  
                            skinPartWithTaxels.taxels.push_back(new Taxel(taxelPos,taxelNorm,i-OFFSET));
                            printMessage(10,"[virtContactGenerationThread]Pushing taxel ID:%d, pos:%f %f %f; norm:%f %f %f.\n",
                                             i-OFFSET,taxelPos[0],taxelPos[1],taxelPos[2],taxelNorm[0],taxelNorm[1],taxelNorm[2]);
                        }
                    }
                    else if((i>=192+OFFSET) && (i<=383+OFFSET))
                    { //second patch - 7 triangles in skin V1, upper part of forearm (though in Marco's files, it is called lower)
                        if( ((i>=204+OFFSET) && (i<=215+OFFSET)) && (i!=204+6+OFFSET) && (i!=204+10+OFFSET))
                        { //first triangle without thermal pads
                            skinPartWithTaxels.taxels.push_back(new Taxel(taxelPos,taxelNorm,i-OFFSET));
                            printMessage(10,"[virtContactGenerationThread]Pushing taxel ID:%d, pos:%f %f %f; norm:%f %f %f.\n",
                                             i-OFFSET,taxelPos[0],taxelPos[1],taxelPos[2],taxelNorm[0],taxelNorm[1],taxelNorm[2]);
                        }
                        else  if( ((i>=252+OFFSET) && (i<=263+OFFSET)) && (i!=252+6+OFFSET) && (i!=252+10+OFFSET))
                        { //2nd triangle without thermal pads
                            skinPartWithTaxels.taxels.push_back(new Taxel(taxelPos,taxelNorm,i-OFFSET));
                            printMessage(10,"[virtContactGenerationThread]Pushing taxel ID:%d, pos:%f %f %f; norm:%f %f %f.\n",
                                             i-OFFSET,taxelPos[0],taxelPos[1],taxelPos[2],taxelNorm[0],taxelNorm[1],taxelNorm[2]);
                        }
                        else if( ((i>=288+OFFSET) && (i<=299+OFFSET)) && (i!=288+6+OFFSET) && (i!=288+10+OFFSET))
                        { //3rd triangle without thermal pads
                            skinPartWithTaxels.taxels.push_back(new Taxel(taxelPos,taxelNorm,i-OFFSET));
                            printMessage(10,"[virtContactGenerationThread]Pushing taxel ID:%d, pos:%f %f %f; norm:%f %f %f.\n",
                                             i-OFFSET,taxelPos[0],taxelPos[1],taxelPos[2],taxelNorm[0],taxelNorm[1],taxelNorm[2]);
                        }
                        else if( ((i>=300+OFFSET) && (i<=311+OFFSET)) && (i!=300+6+OFFSET) && (i!=300+10+OFFSET))
                        { //4th triangle without thermal pads
                            skinPartWithTaxels.taxels.push_back(new Taxel(taxelPos,taxelNorm,i-OFFSET));
                            printMessage(10,"[virtContactGenerationThread]Pushing taxel ID:%d, pos:%f %f %f; norm:%f %f %f.\n",
                                             i-OFFSET,taxelPos[0],taxelPos[1],taxelPos[2],taxelNorm[0],taxelNorm[1],taxelNorm[2]);
                        }
                        else if( ((i>=312+OFFSET) && (i<=323+OFFSET)) && (i!=312+6+OFFSET) && (i!=312+10+OFFSET))
                        { //5th triangle without thermal pads
                            skinPartWithTaxels.taxels.push_back(new Taxel(taxelPos,taxelNorm,i-OFFSET));
                            printMessage(10,"[virtContactGenerationThread]Pushing taxel ID:%d, pos:%f %f %f; norm:%f %f %f.\n",
                                             i-OFFSET,taxelPos[0],taxelPos[1],taxelPos[2],taxelNorm[0],taxelNorm[1],taxelNorm[2]);
                        }
                        else if( ((i>=336+OFFSET) && (i<=347+OFFSET)) && (i!=336+6+OFFSET) && (i!=336+10+OFFSET))
                        { //6th triangle without thermal pads
                            skinPartWithTaxels.taxels.push_back(new Taxel(taxelPos,taxelNorm,i-OFFSET));
                            printMessage(10,"[virtContactGenerationThread]Pushing taxel ID:%d, pos:%f %f %f; norm:%f %f %f.\n",
                                             i-OFFSET,taxelPos[0],taxelPos[1],taxelPos[2],taxelNorm[0],taxelNorm[1],taxelNorm[2]);
                        }
                        else if( ((i>=348+OFFSET) && (i<=359+OFFSET)) && (i!=348+6+OFFSET) && (i!=348+10+OFFSET))
                        { //7th triangle without thermal pads
                            skinPartWithTaxels.taxels.push_back(new Taxel(taxelPos,taxelNorm,i-OFFSET));
                            printMessage(10,"[virtContactGenerationThread]Pushing taxel ID:%d, pos:%f %f %f; norm:%f %f %f.\n",
                                             i-OFFSET,taxelPos[0],taxelPos[1],taxelPos[2],taxelNorm[0],taxelNorm[1],taxelNorm[2]);
                        }
                    }
                }
                if(skinPartWithTaxels.getSize() != 384)
                {
                    yWarning("[virtContactGenerationThread]::initSkinParts():initalizing %s from file, 384 positions expected, but %d present.\n",
                                                                                  SkinPart_s[skin_part_name].c_str(),skinPartWithTaxels.getSize());
                }
                skinPartWithTaxels.name = skin_part_name;
                if (skin_part_name == SKIN_LEFT_FOREARM)
                {
                   activeSkinParts[SKIN_LEFT_FOREARM] = skinPartWithTaxels;

                   printMessage(4,"[virtContactGenerationThread]Adding SKIN_LEFT_FOREARM to activeSkinParts, it now has %d members.\n",
                                                                                                                activeSkinParts.size());
                }
                else
                {  // skin_part_name == SKIN_RIGHT_FOREARM
                    activeSkinParts[SKIN_RIGHT_FOREARM] = skinPartWithTaxels;
                    printMessage(4,"[virtContactGenerationThread]Adding SKIN_RIGHT_FOREARM to activeSkinParts, it now has %d members.\n",
                                                                                                                  activeSkinParts.size());
                }
                break;
                
            default: 
                yError("[virtContactGenerationThread] Asked to initialize skinDynLib::SkinPart:: %d, but it is not implemented yet.\n",
                                                                                                                       skin_part_name);
                return -1;
        }
        
        posFile.close();
    }
    
    return 0;
    
}

void virtContactGenerationThread::printInitializedSkinParts()
{
    
    for (std::map<SkinPart,iCub::skinDynLib::skinPart>::const_iterator it = activeSkinParts.begin() ; it != activeSkinParts.end(); ++it)
    {
        iCub::skinDynLib::skinPart locSkinPartTaxel = it->second;
        vector<Taxel*> taxels = locSkinPartTaxel.taxels;
        printMessage(6,"Iterating through activeSkinParts (%d members), now: it->first: %d, locSkinPartTaxel.name: %s.\n",
                                                          activeSkinParts.size(),it->first,locSkinPartTaxel.name.c_str());
        ofstream outFile;   
        outFile.open(locSkinPartTaxel.name.c_str());
        if (outFile.fail())          // Check for file creation and return error.
        {
           printMessage(6,"Error opening %s for output.\n",it->second.name.c_str());
           continue;
        }

        for (vector<Taxel*>::const_iterator it_taxel = taxels.begin(); it_taxel!= taxels.end(); ++it_taxel)
        {
            outFile << (**it_taxel).getPosition()[0] << " " <<(**it_taxel).getPosition()[1]
                    << " " <<(**it_taxel).getPosition()[2] << " " <<(**it_taxel).getNormal()[0]
                    << " " <<(**it_taxel).getNormal()[1] << " " <<(**it_taxel).getNormal()[2]
                    << " " <<(**it_taxel).getID() <<endl; 
        }
        printMessage(6,"Wrote to file %s for output.\n",locSkinPartTaxel.name.c_str());
        outFile.close();
    }
       
}


virtContactGenerationThread::virtContactGenerationThread(int _rate, const string &_name, const string &_robot, int _v,
                                                         const string &_type, const vector<SkinPart> &_activeSkinPartsNames,
                                                         const map<SkinPart,string> &_skinPartPosFilePaths) :  
                                                         RateThread(_rate),name(_name), robot(_robot), verbosity(_v), type(_type),
                                                         activeSkinPartsNames(_activeSkinPartsNames), skinPartPosFilePaths(_skinPartPosFilePaths)
{
   
    skinEventsOutPort = new BufferedPort<iCub::skinDynLib::skinContactList>;
  
}

bool virtContactGenerationThread::threadInit()
{
    ts.update();
    
    skinEventsOutPort->open(("/"+name+"/virtualContacts:o").c_str());
    
     /* initialize random seed: */
    srand ((unsigned int)time(NULL));
    
    int returnValue = initSkinParts();
    if(returnValue == -1)
    {
        yError("[virtContactGenerationThread] Could not initialize skin parts.\n");
        return false;  
    }
    if(verbosity > 5)
    {
        printInitializedSkinParts();
    }
    if (activeSkinPartsNames.size() != activeSkinParts.size())
    {
        yError("[virtContactGenerationThread] activeSkinPartsNames and activeSkinParts have different size (%lu vs. %lu).\n",
                                                                       activeSkinPartsNames.size(),activeSkinParts.size());
        return false;    
    }
    
    taxelIDinList.clear();
    skinPartIndexInVector = rand() % activeSkinPartsNames.size(); //so e.g. for size 3, this should give 0, 1, or 2, which is right
    skinPartPickedName = activeSkinPartsNames[skinPartIndexInVector];
    skinPartPicked = activeSkinParts[skinPartPickedName];
    taxelPickedIndex = rand() % skinPartPicked.taxels.size(); 
    taxelPicked = *(skinPartPicked.taxels[taxelPickedIndex]);
    taxelIDinList.push_back(taxelPicked.getID()); //there will be only a single taxel in the list, but we want to keep the information which taxel it was
    printMessage(3,"Randomly selecting taxel ID: %d, from %s. Pose in local FoR (pos,norm): %f %f %f; norm:%f %f %f.\n",
                    taxelPicked.getID(),SkinPart_s[skinPartPickedName].c_str(),taxelPicked.getPosition()[0],taxelPicked.getPosition()[1],
                    taxelPicked.getPosition()[2],taxelPicked.getNormal()[0],taxelPicked.getNormal()[1],taxelPicked.getNormal()[2]);  
    
    return true;
}

void virtContactGenerationThread::run()
{
    ts.update();
     
    if (type == "random")
    {
        taxelIDinList.clear();
        skinPartIndexInVector = rand() % activeSkinPartsNames.size(); //so e.g. for size 3, this should give 0, 1, or 2, which is right
        skinPartPickedName = activeSkinPartsNames[skinPartIndexInVector];
        skinPartPicked = activeSkinParts[skinPartPickedName];
        taxelPickedIndex = rand() % skinPartPicked.taxels.size(); 
        taxelPicked = *(skinPartPicked.taxels[taxelPickedIndex]);
        taxelIDinList.push_back(taxelPicked.getID()); //there will be only a single taxel in the list, but we want to keep the information which taxel it was
        printMessage(3,"Randomly selecting taxel ID: %d, from %s. Pose in local FoR (pos,norm): %f %f %f; norm:%f %f %f.\n",
                        taxelPicked.getID(),SkinPart_s[skinPartPickedName].c_str(),taxelPicked.getPosition()[0],
                        taxelPicked.getPosition()[1],taxelPicked.getPosition()[2],taxelPicked.getNormal()[0],
                        taxelPicked.getNormal()[1],taxelPicked.getNormal()[2]);  
    }
    
    skinContact c(getBodyPart(skinPartPickedName), skinPartPickedName, getLinkNum(skinPartPickedName), taxelPicked.getPosition(), 
                                          taxelPicked.getPosition(),taxelIDinList,VIRT_CONTACT_PRESSURE,taxelPicked.getNormal());  
    //   skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
    //  const yarp::sig::Vector &_geoCenter, std::vector<unsigned int> _taxelList, double _pressure, const yarp::sig::Vector &_normalDir);
    printMessage(3,"Creating skin contact as follows: %s.\n",c.toString().c_str());
        
    //see also void SimulatorModule::sendSkinEvents(iCub::skinDynLib::skinContactList& skinContactListReport)
    //and compensationThread.cpp void CompensationThread::sendSkinEvents() 
    skinContactList &listWithPickedSkinContact = skinEventsOutPort->prepare();
    listWithPickedSkinContact.clear();;
    listWithPickedSkinContact.push_back(c);
    skinEventsOutPort->setEnvelope(ts);
    skinEventsOutPort->write();
    
}

int virtContactGenerationThread::printMessage(const int l, const char *f, ...) const
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

void virtContactGenerationThread::threadRelease()
{
    activeSkinPartsNames.clear();
    skinPartPosFilePaths.clear();
    activeSkinParts.clear();   
    taxelIDinList.clear(); 
    
    printMessage(0,"Closing ports..\n");
    closePort(skinEventsOutPort);
    printMessage(1,"skinEventsOutPort successfully closed!\n");
}


