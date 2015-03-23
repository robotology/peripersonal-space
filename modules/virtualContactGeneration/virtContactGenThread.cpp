#include "virtContactGenThread.h"
#include <yarp/sig/Image.h>


int virtContactGenerationThread::initSkinParts()
{
    SkinPart skin_part_name; 
    
    string line;
    ifstream posFile;
    yarp::sig::Vector taxelPos(3,0.0);
    yarp::sig::Vector taxelNorm(3,0.0);
   
    string filename; 
    //go through skin parts and initialize them
    for (std::vector<SkinPart>::const_iterator it = activeSkinPartsNames.begin() ; it != activeSkinPartsNames.end(); ++it){
        skin_part_name = *it;
        
        switch(skin_part_name){
            case SKIN_LEFT_HAND: 
                ;
                break;
            default: 
                yError("[virtContactGenerationThread] Asked to initialize skinDynLib::SkinPart:: %d, but that skin part is not implemented yet.\n",skin_part_name);
                return -1;
        }
        
        //initialize the skinPart container  - create a skin part object and then add it to a vector of the skin parts
        //init will include adding the valid taxels (again, object from utils), analogous to vtRFThread::setTaxelPosesFromFile
        // I guess I will add only real taxel (not thermal pads) and only those for which I have positions 
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


