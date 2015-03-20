#include "virtContactGenThread.h"
#include <yarp/sig/Image.h>

virtContactGenerationThread::virtContactGenerationThread(int _rate, const string &_name, const string &_robot, int _v, const string &_type, vector<SkinPart> &_activeSkinParts): 
RateThread(_rate),name(_name), robot(_robot), verbosity(_v), type(_type), activeSkinParts(_activeSkinParts)
{
   
    skinEventsOutPort = new BufferedPort<iCub::skinDynLib::skinContactList>;
  
}

bool virtContactGenerationThread::threadInit()
{
    skinEventsOutPort->open(("/"+name+"/virtualContacts:i").c_str());
    
    return true;
}

void virtContactGenerationThread::run()
{

    ;
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

