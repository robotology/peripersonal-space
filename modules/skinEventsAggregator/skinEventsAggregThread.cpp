#include "skinEventsAggregThread.h"


skinEventsAggregThread::skinEventsAggregThread(int _rate, const string &_name, const string &_robot, int _v, const map<SkinPart,string> &_skinPartPosFilePaths): 
RateThread(_rate),name(_name), robot(_robot), verbosity(_v), skinPartPosFilePaths(_skinPartPosFilePaths)
{
   ;
    
}

bool skinEventsAggregThread::threadInit()
{
       
    ts.update();
    skinEventsPortIn.open(("/"+name+"/skin_events:i").c_str());
    skinEvAggregPortOut.open(("/"+name+"/skin_events:o").c_str());
       
    return true;
}

void skinEventsAggregThread::run()
{
    ts.update();
}

int skinEventsAggregThread::printMessage(const int l, const char *f, ...) const
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

void skinEventsAggregThread::threadRelease()
{
    printMessage(0,"Closing ports..\n");
    skinEventsPortIn.interrupt();
    skinEventsPortIn.close();
    printMessage(1,"skinEventPortIn successfully closed!\n");
    skinEvAggregPortOut.interrupt();
    skinEvAggregPortOut.close();
    printMessage(1,"skinEvAggregPortOut successfully closed!\n");
}


