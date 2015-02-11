#include "fgtThread.h"

fgtThread::fgtThread(int _rate, const string &_name, const string &_robot, int _v,
                     const Vector &_hsvmin, const Vector &_hsvmax) :
                     RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    stateFlag = 0;
    timeNow   = yarp::os::Time::now();

    fingerL.resize(2,0.0);
    fingerR.resize(2,0.0);
    fingertip.resize(3,0.0);

    HSVmin.resize(3,0.0);
    HSVmax.resize(3,0.0);

    HSVmin = _hsvmin;
    HSVmax = _hsvmax;
}

bool fgtThread::threadInit()
{
    imagePortInR. open(("/"+name+"/imageR:i").c_str());
    imagePortInL. open(("/"+name+"/imageL:i").c_str());
    imagePortOutR.open(("/"+name+"/imageR:o").c_str());
    imagePortOutL.open(("/"+name+"/imageL:o").c_str());

    doubleTouchPort.open(("/"+name+"/doubleTouch:i").c_str());
    outPort.open(("/"+name+"/out:o").c_str());


    // I know that I should remove this but it's harmless (and I'm overly lazy)
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

        Network::connect(("/"+name+"/imageL:o").c_str(),"/yvL");
        Network::connect(("/"+name+"/imageR:o").c_str(),"/yvR");
        Network::connect("/doubleTouch/status:o",("/"+name+"/doubleTouch:i").c_str());
        Network::connect(("/"+name+"/out:o").c_str(),"/visuoTactileWrapper/fingertipTracker:i");

    Property OptGaze;
    OptGaze.put("device","gazecontrollerclient");
    OptGaze.put("remote","/iKinGazeCtrl");
    OptGaze.put("local",("/"+name+"/gaze").c_str());

    if ((!ddG.open(OptGaze)) || (!ddG.view(igaze))){
       yError("could not open the Gaze Controller!\n");
       return false;
    }

    igaze -> storeContext(&contextGaze);
    igaze -> setSaccadesStatus(false);
    igaze -> setNeckTrajTime(0.75);
    igaze -> setEyesTrajTime(0.5);

    return true;
}

void fgtThread::run()
{
    yTrace("Running..");

    ImageOf<PixelRgb> *tmpL = imagePortInL.read(false);
    if(tmpL!=NULL)
    {
        imageInL = tmpL;
    }
    ImageOf<PixelRgb> *tmpR = imagePortInR.read(false);
    if(tmpR!=NULL)
    {
        imageInR = tmpR;
    }

    // process the doubleTouch
    // if(doubleTouchBottle = doubleTouchPort.read(false))
    // {
    //     if(doubleTouchBottle != NULL)
    //     {
    //         if (doubleTouchBottle->get(3).asString() != "")
    //         {
    //             doubleTouchStep = doubleTouchBottle->get(0).asInt();

    //             if(doubleTouchStep>1 && doubleTouchStep<7)
    //             {
                        if (imageInL!=NULL && imageInR!=NULL)
                        {
                            yDebug("Processing images..");

                            bool fgt = 0;
                            if (fgt=processImages(imageOutL,imageOutR))
                            {
                                get3DPoint();    
                            }
                            else
                            {
                                yTrace("finger not found. fingerL: %s fingerR: %s",
                                        fingerL.toString(3,3).c_str(),fingerR.toString(3,3).c_str());
                            }
                        
                            Bottle& output = outPort.prepare();
                            output.clear();
                            output.addInt(fgt);
                            if (fgt)
                            {
                                vectorIntoBottle(fingertip,output);
                            }

                            outPort.write();
                            
                            sendImages();
                        }
    //             }
    //         }
    //     }
    // }
}

bool fgtThread::processImages(ImageOf<PixelRgb> &_oL, ImageOf<PixelRgb> &_oR)
{
    bool resultL = false;
    bool resultR = false;

    _oR.resize(*imageInR);
    _oL.resize(*imageInL);
    // 
    ImageOf<PixelMono> maskL;
    ImageOf<PixelMono> maskR;
    maskL.resize(*imageInL);
    maskR.resize(*imageInR);

    ImageOf<PixelRgb> imageOL=*imageInL;
    ImageOf<PixelRgb> imageOR=*imageInR;

    cv::Mat imgL((IplImage*)imageOL.getIplImage());
    cv::Mat imgR((IplImage*)imageOR.getIplImage());
    cv::Mat mskL((IplImage*)maskL.getIplImage());
    cv::Mat mskR((IplImage*)maskR.getIplImage());
    cv::Mat imgLHSV;
    cv::Mat imgRHSV;

    yTrace("I'm converting the images to in HSV");
    cvtColor(imgL,imgLHSV,CV_RGB2HSV);
    cvtColor(imgR,imgRHSV,CV_BGR2HSV);

    yTrace("I'm filtering according to the HSV");
    cv::inRange(imgLHSV, cv::Scalar(HSVmin[0],HSVmin[1],HSVmin[2]), cv::Scalar(HSVmax[0],HSVmax[1],HSVmax[2]),mskL);
    cv::inRange(imgRHSV, cv::Scalar(HSVmin[0],HSVmin[1],HSVmin[2]), cv::Scalar(HSVmax[0],HSVmax[1],HSVmax[2]),mskR);

    yTrace("I'm blurring them in order to remove noise.");
    medianBlur(mskL, mskL, 5);
    medianBlur(mskR, mskR, 5);

    // The mask could have more than one blob: find the biggest one
    // (hopefully the fingertip)
    yTrace("Let's find the biggest contour");
    vector<vector<cv::Point> > contours;
    double area = -1;
    cv::RotatedRect rect;

    contours.clear();
    cv::findContours(mskL,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    int idx=-1;
    int largestArea=0;

    if (contours.size()>0)
    {
        for (size_t i=0; i<contours.size(); i++)
        {
            area=cv::contourArea(contours[i]);
            if (area>largestArea)
            {
                largestArea=area;
                idx=i;
            }
        }

        if (largestArea>16)
        {
            rect=cv::fitEllipse(contours[idx]);
            cv::Point fgtL=rect.center;
            fingerL[0] = fgtL.x;
            fingerL[1] = fgtL.y;

            cv::ellipse(imgL, rect, cv::Scalar(40,50,50), -2);
            cv::circle(imgL, fgtL, 3, cv::Scalar(175,125,0), -1);
            resultL = true;
        }
    }

    area = -1;
    contours.clear();
    cv::findContours(mskR,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    idx=-1;
    largestArea=0;

    if (contours.size()>0)
    {
        for (size_t i=0; i<contours.size(); i++)
        {
            area=cv::contourArea(contours[i]);
            if (area>largestArea)
            {
                largestArea=area;
                idx=i;
            }
        }
        yTrace("largestArea: %i",largestArea);
        if (largestArea>16 && largestArea<200)
        {
            rect=cv::fitEllipse(contours[idx]);
            cv::Point fgtR=rect.center;
            fingerR[0] = fgtR.x;
            fingerR[1] = fgtR.y;

            cv::ellipse(imgR, rect, cv::Scalar(40,50,50), 2);
            cv::circle (imgR, fgtR, 3, cv::Scalar(175,125,0), -1);
            resultR = true;
        }
    }

    _oL = imageOL;
    _oR = imageOR;

    return resultL && resultR;
}

bool fgtThread::sendImages()
{
    imagePortOutL.prepare()=imageOutL;
    imagePortOutL.write();

    imagePortOutR.prepare()=imageOutR;
    imagePortOutR.write();

    return true;
}

bool fgtThread::get3DPoint()
{
    yarp::sig::Vector x(3,0.0);

    igaze->triangulate3DPoint(fingerL, fingerR, x);

    yDebug("3D point found! %s fingerL: %s fingerR: %s",x.toString(3,3).c_str(),
            fingerL.toString(3,3).c_str(),fingerR.toString(3,3).c_str());

    fingertip = x;

    return true;
}

void fgtThread::setHMax(int _val)
{
    HSVmax[0]=double(_val);
    yInfo("Setting hmax to %i. Result %g", _val, HSVmax[0]);
}

void fgtThread::setHMin(int _val)
{
    HSVmin[0]=double(_val);
    yInfo("Setting hmin to %i. Result %g", _val, HSVmin[0]);
}

int fgtThread::printMessage(const int l, const char *f, ...) const
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

void fgtThread::threadRelease()
{
    printMessage(0,"Closing gaze controller..\n");
        igaze -> restoreContext(contextGaze);
        igaze -> stopControl();
        ddG.close();
}

// empty line to make gcc happy
