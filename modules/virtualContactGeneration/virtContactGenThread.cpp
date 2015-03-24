#include "virtContactGenThread.h"
#include <yarp/sig/Image.h>

//see also Compensator::setTaxelPosesFromFile in icub-main/src/modules/skinManager/src/compensator.cpp
//see also dbool vtRFThread::setTaxelPosesFromFile(const string filePath, skinPartPWE &sP)
int virtContactGenerationThread::initSkinParts()
{
    SkinPart skin_part_name; 
    skinPartTaxel *skinPartWithTaxels; 
    
    string line;
    ifstream posFile;
    yarp::sig::Vector taxelPos(3,0.0);
    yarp::sig::Vector taxelNorm(3,0.0);
   
    string filename; 
    //go through skin parts and initialize them
    for (std::vector<SkinPart>::const_iterator it = activeSkinPartsNames.begin() ; it != activeSkinPartsNames.end(); ++it){
        skin_part_name = *it;
        
        // Open File
        posFile.open(skinPartPosFilePaths[skin_part_name].c_str());  
        if (!posFile.is_open())
        {
           yWarning("[virtContactGenerationThread] File %s has not been opened!",skinPartPosFilePaths[skin_part_name].c_str());
           return false;
        }
        posFile.clear(); 
        posFile.seekg(0, std::ios::beg);//rewind iterator
    
        skinPartTaxel skinPartWithTaxels;
        
        switch(skin_part_name){
            case SKIN_LEFT_HAND:
            case SKIN_RIGHT_HAND:
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
                            taxelPos[j] = strtod(number.c_str(),NULL);
                        else
                            taxelNorm[j-3] = strtod(number.c_str(),NULL);
                    }
                    skinPartWithTaxels.size++; //this is incremented for all lines - size of "port"
                    if((i>=96) && (i<=143) && (i!=107) && (i!=119) && (i!=131) && (i!=139)) //all palm taxels, without thermal pads
                    {
                        skinPartWithTaxels.txls.push_back(new Taxel(taxelPos,taxelNorm,i));
                    }
               }
               if (skin_part_name == SKIN_LEFT_HAND){
                    activeSkinParts[SKIN_LEFT_HAND] = skinPartWithTaxels;
               }
               else{  // skin_part_name == SKIN_RIGHT_HAND
                    activeSkinParts[SKIN_RIGHT_HAND] = skinPartWithTaxels;
               }
               break;
            default: 
                yError("[virtContactGenerationThread] Asked to initialize skinDynLib::SkinPart:: %d, but that skin part is not implemented yet.\n",skin_part_name);
                return -1;
        }
        
       
    }
    
    return 0;
    
}


virtContactGenerationThread::virtContactGenerationThread(int _rate, const string &_name, const string &_robot, int _v, const string &_type, const vector<SkinPart> &_activeSkinPartsNames, const map<SkinPart,string> &_skinPartPosFilePaths): 
RateThread(_rate),name(_name), robot(_robot), verbosity(_v), type(_type), activeSkinPartsNames(_activeSkinPartsNames), skinPartPosFilePaths(_skinPartPosFilePaths)
{
   
    skinEventsOutPort = new BufferedPort<iCub::skinDynLib::skinContactList>;
  
}

bool virtContactGenerationThread::threadInit()
{
       
    skinEventsOutPort->open(("/"+name+"/virtualContacts:i").c_str());
    
     /* initialize random seed: */
    srand (time(NULL));
    
  
    
    return true;
}

void virtContactGenerationThread::run()
{

    if (type == "random"){
        int skinPartIndexInVector = rand() % activeSkinPartsNames.size(); //so e.g. for size 3, this should give 0, 1, or 2, which is right
        skinPartPicked = activeSkinPartsNames[skinPartIndexInVector];
    }
    
}

int virtContactGenerationThread::printMessage(const int l, const char *f, ...) const
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

void virtContactGenerationThread::threadRelease()
{
     printMessage(0,"Closing ports..\n");
     closePort(skinEventsOutPort);
     printMessage(1,"skinEventsOutPort successfully closed!\n");

}


