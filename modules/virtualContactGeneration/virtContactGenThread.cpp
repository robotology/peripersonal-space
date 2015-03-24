#include "virtContactGenThread.h"
#include <yarp/sig/Image.h>

//see also Compensator::setTaxelPosesFromFile in icub-main/src/modules/skinManager/src/compensator.cpp
//see also dbool vtRFThread::setTaxelPosesFromFile(const string filePath, skinPartPWE &sP)
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
        skinPartTaxel skinPartWithTaxels; 
        
        // Open File
        posFile.open(skinPartPosFilePaths[skin_part_name].c_str());  
        if (!posFile.is_open())
        {
           yWarning("[virtContactGenerationThread] File %s has not been opened!",skinPartPosFilePaths[skin_part_name].c_str());
           return false;
        }
        printMessage(4,"Initializing %s from %s.\n",SkinPart_s[skin_part_name].c_str(),skinPartPosFilePaths[skin_part_name].c_str());
        posFile.clear(); 
        posFile.seekg(0, std::ios::beg);//rewind iterator
        
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
                    taxelPos.zero(); taxelNorm.zero();
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
                        printMessage(10,"Pushing taxel ID:%d, pos:%f %f %f; norm:%f %f %f.\n",i,taxelPos[0],taxelPos[1],taxelPos[2],taxelNorm[0],taxelNorm[1],taxelNorm[2]);
                    }
                }
                if(skinPartWithTaxels.size != 192){
                    yWarning("[virtContactGenerationThread]::initSkinParts():initalizing %s from file, 192 positions expected, but %d present.\n",SkinPart_s[skin_part_name].c_str(),skinPartWithTaxels.size);
                }
                skinPartWithTaxels.name = skin_part_name;
                if (skin_part_name == SKIN_LEFT_HAND){
                   activeSkinParts[SKIN_LEFT_HAND] = skinPartWithTaxels;
                   printMessage(4,"Adding SKIN_LEFT_HAND to activeSkinParts, it now has %d members.\n",activeSkinParts.size());
                }
                else{  // skin_part_name == SKIN_RIGHT_HAND
                    activeSkinParts[SKIN_RIGHT_HAND] = skinPartWithTaxels;
                    printMessage(4,"Adding SKIN_RIGHT_HAND to activeSkinParts, it now has %d members.\n",activeSkinParts.size());
                }
                break;
            default: 
                yError("[virtContactGenerationThread] Asked to initialize skinDynLib::SkinPart:: %d, but that skin part is not implemented yet.\n",skin_part_name);
                return -1;
        }
        
        posFile.close();
        
       
    }
    
    return 0;
    
}

void virtContactGenerationThread::printInitializedSkinParts()
{
    
    for (std::map<SkinPart,skinPartTaxel>::const_iterator it = activeSkinParts.begin() ; it != activeSkinParts.end(); ++it){
        skinPartTaxel locSkinPartTaxel = it->second;
        vector<Taxel*> taxels = locSkinPartTaxel.txls;
        printMessage(6,"Iterating through activeSkinParts (%d members), now: it->first: %d, locSkinPartTaxel.name:%d, %s.\n",activeSkinParts.size(),it->first,locSkinPartTaxel.name,SkinPart_s[locSkinPartTaxel.name].c_str());
        ofstream outFile;   
        outFile.open(SkinPart_s[locSkinPartTaxel.name].c_str());
        if (outFile.fail())          // Check for file creation and return error.
        {
           printMessage(6,"Error opening %s for output.\n",SkinPart_s[it->second.name].c_str());
           continue;
        }
        for (vector<Taxel*>::const_iterator it_taxel = taxels.begin(); it_taxel!= taxels.end(); ++it_taxel){
             outFile << (**it_taxel).Pos[0] << " " <<(**it_taxel).Pos[1] << " " <<(**it_taxel).Pos[2] << " " <<(**it_taxel).Norm[0] << " " <<(**it_taxel).Norm[1] << " " <<(**it_taxel).Norm[2] << " " <<(**it_taxel).ID <<endl; 
        }
        printMessage(6,"Wrote to file %s for output.\n",SkinPart_s[locSkinPartTaxel.name].c_str());
        outFile.close();             
    }
    


    
    
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
    
    initSkinParts();
    
    if(verbosity > 5){
        printInitializedSkinParts();
    }
    
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


