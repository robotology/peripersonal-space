#include "virtContactGenThread.h"
#include <yarp/sig/Image.h>

virtContactGenerationThread::virtContactGenerationThread(int _rate, const string &_name, const string &_robot, int _v, const string &_type, vector<SkinPart> &_activeSkinParts): 
RateThread(_rate),name(_name), robot(_robot), verbosity(_v), type(_type), activeSkinParts(_activeSkinParts)
{
   
    skinEventsOutPort = new BufferedPort<iCub::skinDynLib::skinContactList>;
  
}

bool virtContactGenerationThread::threadInit()
{
    SkinPart skin_part; 
    
    skinEventsOutPort->open(("/"+name+"/virtualContacts:i").c_str());
    
     /* initialize random seed: */
    srand (time(NULL));
    
    //go through skin parts and initialize them
    for (std::vector<SkinPart>::const_iterator it = activeSkinParts.begin() ; it != activeSkinParts.end(); ++it){
        skin_part = *it;
        //TODO initialize the skinPart - create a skin part object and then add it to a vector of the skin parts
        //init will include adding the valid taxels (again, object from utils), analogous to vtRFThread::setTaxelPosesFromFile
        // I guess I will add only real taxel (not thermal pads) and only those for which I have positions 
    }
    
    return true;
}

void virtContactGenerationThread::run()
{

    if (type == "random"){
        int skinPartIndexInVector = rand() % activeSkinParts.size(); //so e.g. for size 3, this should give 0, 1, or 2, which is right
        skinPartPicked = activeSkinParts[skinPartIndexInVector];
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

