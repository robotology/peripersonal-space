#include "skinEventsAggregThread.h"

#define SKIN_ACTIVATION_MAX_ICUB_SIM 100
#define SKIN_ACTIVATION_MAX_ICUB 30

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

using namespace iCub::skinDynLib;


/*************** public methods section ********************************************/

skinEventsAggregThread::skinEventsAggregThread(int _rate, const string &_name, const string &_robot, int _v, const map<SkinPart,string> &_skinPartPosFilePaths): 
RateThread(_rate),name(_name), robot(_robot), verbosity(_v), skinPartPosFilePaths(_skinPartPosFilePaths)
{
   ;
    
}

bool skinEventsAggregThread::threadInit()
{
       
    ts.update();
    skinEventsPortIn.open(("/"+name+"/skin_events:i").c_str());
    skinEvAggregPortOut.open(("/"+name+"/skin_events_aggreg:o").c_str());
   
    if (robot == "icub")
        SKIN_ACTIVATION_MAX = SKIN_ACTIVATION_MAX_ICUB;
    else if (robot == "icubSim")
        SKIN_ACTIVATION_MAX = SKIN_ACTIVATION_MAX_ICUB_SIM;
    else
        yError("skinEventsAggregThread::threadInit(): unknown robot type");
    
        
    return true;
}

void skinEventsAggregThread::run()
{
    int indexOfBiggestContact = -1;
    skinContact biggestContactInSkinPart;
    Vector geoCenter(3,0.0), normalDir(3,0.0);
    double activation = 0.0;
    Bottle out; out.clear();
    Bottle b;     b.clear();
        
    ts.update();
        
    skinContactList *scl = skinEventsPortIn.read(false);
        
    if(scl)
    {
        if(!(scl->empty())){  
            //Probably source of crazy inefficiencies, here just to reach a working state as soon as possible \todo TODO
            map<SkinPart, skinContactList> contactsPerSkinPart = scl->splitPerSkinPart();
            for(map<SkinPart,skinContactList>::iterator it=contactsPerSkinPart.begin(); it!=contactsPerSkinPart.end(); it++)
            {
                indexOfBiggestContact = getIndexOfBiggestContactInList(it->second);
                if (indexOfBiggestContact != -1){
                    b.clear();
                    biggestContactInSkinPart = (it->second)[indexOfBiggestContact];
                    //the output prepared should have identical format to the one prepared in  void vtRFThread::manageSkinEvents()    
                    b.addString(biggestContactInSkinPart.getSkinPartName());
                    vectorIntoBottle(biggestContactInSkinPart.getGeoCenter(),b);
                    vectorIntoBottle(biggestContactInSkinPart.getNormalDir(),b);
                    b.addDouble(min(1.0,(biggestContactInSkinPart.getPressure()/SKIN_ACTIVATION_MAX))); // % pressure "normalized" with ad hoc constant
                    out.addList().read(b);
                }
            }
            skinEvAggregPortOut.setEnvelope(ts);
            skinEvAggregPortOut.write(out);     // send something anyway (if there is no contact the bottle is empty)
        }
    }
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
 
 
/*************** protected methods section ********************************************/

int skinEventsAggregThread::getIndexOfBiggestContactInList(iCub::skinDynLib::skinContactList &sCL)
{
    int index = -1;
    unsigned int maxActivatedTaxels = 0;
    if (! sCL.empty())
    {
        for(skinContactList::iterator c=sCL.begin(); c!=sCL.end(); c++)
        {
             if( c->getActiveTaxels() > maxActivatedTaxels)
             {
                maxActivatedTaxels = c->getActiveTaxels();
                index = std::distance( sCL.begin(), c);
             }
        }
    }
    if (index == -1)
       yError("skinEventsAggregThread::getIndexOfBiggestContactInList: returning index -1");
    
    return index;
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