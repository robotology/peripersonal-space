#include "iCub/periPersonalSpace/utils.h"

yarp::sig::Vector toVector(yarp::sig::Matrix m)
{
    Vector res(m.rows()*m.cols(),0.0);
    
    for (int r = 0; r < m.rows(); r++)
    {
        res.setSubvector(r*m.cols(),m.getRow(r));
    }

    return res;
}

void closePort(Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

yarp::sig::Matrix matrixFromBottle(const Bottle b, int in, const int r, const int c)
{
    yarp::sig::Matrix m(r,c);
    m.zero();
    
    for (int i = 0; i<r; i++)
    {
        for (int j = 0; j<c; j++)
        {
            m(i,j) =  b.get(in).asDouble();
            in++;
        }
    }
    
    return m;
}

yarp::sig::Vector vectorFromBottle(const Bottle b, int in, const int size)
{
    yarp::sig::Vector v(size,0.0);

    for (int i = 0; i < size; i++)
    {
        v[i] = b.get(in).asDouble();
        in++;
    }
    return v;
}

void vectorIntoBottle(const yarp::sig::Vector v, Bottle &b)
{
    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addDouble(v[i]);
    }
}

void matrixIntoBottle(const yarp::sig::Matrix m, Bottle &b)
{
    Vector v = toVector(m);
    
    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addDouble(v[i]);
    }
}

string int_to_string( const int a )
{
    std::stringstream ss;
    ss << a;
    return ss.str();
}

/****************************************************************/
/* INCOMING EVENT WRAPPER
/****************************************************************/
    IncomingEvent::IncomingEvent()
    {
        Pos.resize(3,0.0);
        Vel.resize(3,0.0);
        Src="";
        Radius=-1.0;
    }

    IncomingEvent::IncomingEvent(const Vector &p, const Vector &v, const double r, const string &s)
    {
        Pos = p;
        Vel = v;
        Src = s;
        Radius = r;
    }

    IncomingEvent::IncomingEvent(const IncomingEvent &e)
    {
        *this = e;
    }

    IncomingEvent::IncomingEvent(const Bottle &b)
    {
        fromBottle(b);
    }

    IncomingEvent & IncomingEvent::operator=(const IncomingEvent &e)
    {
        Pos    = e.Pos;
        Vel    = e.Vel;
        Src    = e.Src;
        Radius = e.Radius;
        return *this;
    }

    Bottle IncomingEvent::toBottle()
    {
        Bottle b;
        b.clear();

        b.addDouble(Pos[0]);
        b.addDouble(Pos[1]);
        b.addDouble(Pos[2]);
        b.addDouble(Vel[0]);
        b.addDouble(Vel[1]);
        b.addDouble(Vel[2]);
        b.addDouble(Radius);
        b.addString(Src);

        return b;
    }

    bool IncomingEvent::fromBottle(const Bottle &b)
    {
        Pos.resize(3,0.0);
        Vel.resize(3,0.0);
        Src="";
        Radius=-1.0;

        Pos[0] = b.get(0).asDouble();
        Pos[1] = b.get(1).asDouble();
        Pos[2] = b.get(2).asDouble();

        Vel[0] = b.get(3).asDouble();
        Vel[1] = b.get(4).asDouble();
        Vel[2] = b.get(5).asDouble();

        Radius = b.get(6).asDouble();
        Src    = b.get(7).asString();

        return true;
    }

    void IncomingEvent::print()
    {
        printf("Pos: %s\t Vel: %s\t Radius %g\t Src %s\n",Pos.toString().c_str(),Vel.toString().c_str(),Radius,Src.c_str());
    }

    string IncomingEvent::toString(int precision) const
    {
        stringstream res;
        res << "Pos: "<< Pos.toString() << "\t Vel: "<< Vel.toString()
            << "\t Radius: "<< Radius << "\t Src: "<< Src;
        return res.str();
    }

/****************************************************************/
/* TAXEL WRAPPER
/****************************************************************/
    Taxel::Taxel()
    {
        ID   = 0;
        Resp = 0;
        Pos.resize(3,0.0);
        WRFPos.resize(3,0.0);
        Norm.resize(3,0.0);
        pxR.resize(2,0.0);
        pxRR.resize(2,0.0);
        pxL.resize(2,0.0);
        pxLL.resize(2,0.0);
        RF = eye(4);        
    }

    Taxel::Taxel(const Vector &p, const Vector &n)
    {
        Pos  = p;
        Norm = n;
        setRF();
    }

    Taxel::Taxel(const Vector &p, const Vector &n, const int &i)
    {
        ID   = i;
        Pos  = p;
        Norm = n;
        setRF();
    }

    Taxel & Taxel::operator=(const Taxel &t)
    {
        ID     = t.ID;
        Resp   = t.Resp;
        Pos    = t.Pos;
        WRFPos = t.WRFPos;
        Norm   = t.Norm;
        pxR    = t.pxR;
        pxRR   = t.pxRR;
        pxL    = t.pxL;
        pxLL   = t.pxLL;
        RF     = t.RF;
        return *this;
    }

    void Taxel::print(int verbosity)
    {
        if (verbosity == 0)
            printf("ID %i \tPos %s \tNorm %s\n", ID,
                    Pos.toString().c_str(), Norm.toString().c_str());
        else 
            printf("ID %i \tPos %s \tNorm %s \n\tHst %s\n", ID,
                    Pos.toString().c_str(), Norm.toString().c_str(),
                    pwe.getHist().toString().c_str());
    }

    string Taxel::toString(int precision)
    {
        stringstream res;
        res << "ID: " << ID << "\tPos: "<< Pos.toString() << "\t Norm: "<< Norm.toString();

        if (precision)
            res << "\n Hst: "<< pwe.getHist().toString() << endl;
        return res.str();
    }

    void Taxel::setRF()
    {
        if (Norm == zeros(3))
        {
            RF=eye(4);
            return;
        }
        
        // Set the proper orientation for the touching end-effector
        Vector x(3,0.0), z(3,0.0), y(3,0.0);

        z = Norm;
        if (z[0] == 0.0)
        {
            z[0] = 0.00000001;
        }
        y[0] = -z[2]/z[0]; y[2] = 1;
        x = -1*(cross(z,y));
        
        // Let's make them unitary vectors:
        x = x / norm(x);
        y = y / norm(y);
        z = z / norm(z);

        RF=eye(4);
        RF.setSubcol(x,0,0);
        RF.setSubcol(y,0,1);
        RF.setSubcol(z,0,2);
        RF.setSubcol(Pos,0,3);
    }

    bool Taxel::resetParzenWindow()
    {
        pwe.resetHist();
        return true;
    }

    bool Taxel::computeResponse()
    {
        double RFExtension = double(pwe.getExt());
        if (norm(Evnt.Pos) <= RFExtension && Evnt.Pos[2] >= 0)
        {
            //Resp = ((RFExtension - norm(Evnt.Pos))/RFExtension)*255;
            Resp = (int)pwe.getF_X_scaled(norm(Evnt.Pos));
        }
        else
            Resp = 0;

        return true;
    }

/****************************************************************/
/* SKINPART WRAPPER
/****************************************************************/
    skinPart::skinPart()
    {
        name="";
        size= 0;
    }

    skinPart & skinPart::operator=(const skinPart &spw)
    {
        name =spw.name;
        taxel=spw.taxel;
        size =spw.size;
        return *this;
    }

    void skinPart::print(int verbosity)
    {
        printf("**********\n");
        printf("name: %s\t", name.c_str());
        printf("size: %i\n", size);
        for (size_t i = 0; i < taxel.size(); i++)
            taxel[i].print(verbosity);
        printf("**********\n");
    }

    string skinPart::toString(int precision)
    {
        stringstream res;
        res << "**********\n" << "Name: " << name << "\tSize: "<< size << endl;
        for (size_t i = 0; i < taxel.size(); i++)
            res << taxel[i].toString(precision);
        res << "**********\n";
        return res.str();
    }

    bool skinPart::toProperty(Property &info)
    {
        return true;
    }

/****************************************************************/
/* EYE WRAPPER
/****************************************************************/
    bool getAlignHN(const ResourceFinder &rf, const string &type,
                    iKinChain *chain, const bool verbose)
    {
        ResourceFinder &_rf=const_cast<ResourceFinder&>(rf);
        if ((chain!=NULL) && _rf.isConfigured())
        {
            string message=_rf.findFile("from").c_str();
            if (!message.empty())
            {
                message+=": aligning matrix for "+type;
                Bottle &parType=_rf.findGroup(type.c_str());
                if (!parType.isNull())
                {
                    if (Bottle *bH=parType.find("HN").asList())
                    {
                        int i=0;
                        int j=0;

                        Matrix HN(4,4); HN=0.0;
                        for (int cnt=0; (cnt<bH->size()) && (cnt<HN.rows()*HN.cols()); cnt++)
                        {
                            HN(i,j)=bH->get(cnt).asDouble();
                            if (++j>=HN.cols())
                            {
                                i++;
                                j=0;
                            }
                        }

                        // enforce the homogeneous property
                        HN(3,0)=HN(3,1)=HN(3,2)=0.0;
                        HN(3,3)=1.0;

                        chain->setHN(HN);

                        if (verbose)
                        {
                            fprintf(stdout,"%s found:\n",message.c_str());
                            fprintf(stdout,"%s\n",HN.toString(3,3).c_str());
                        }

                        return true;
                    }
                }
            }
            else
            {
                message=_rf.find("from").asString().c_str();
                message+=": aligning matrix for "+type;
            }
            if (verbose)
                fprintf(stdout,"%s not found!\n",message.c_str());
        }

        return false;
    };

    /************************************************************************/
    bool getCamPrj(const ResourceFinder &rf, const string &type,
                   Matrix **Prj, const bool verbose)
    {
        ResourceFinder &_rf=const_cast<ResourceFinder&>(rf);
        *Prj=NULL;

        if (!_rf.isConfigured())
            return false;
        
        string message=_rf.findFile("from").c_str();
        if (!message.empty())
        {
            message+=": intrinsic parameters for "+type;
            Bottle &parType=_rf.findGroup(type.c_str());
            if (!parType.isNull())
            {
                if (parType.check("fx") && parType.check("fy") &&
                    parType.check("cx") && parType.check("cy"))
                {
                    double fx=parType.find("fx").asDouble();
                    double fy=parType.find("fy").asDouble();
                    double cx=parType.find("cx").asDouble();
                    double cy=parType.find("cy").asDouble();

                    if (verbose)
                    {
                        fprintf(stdout,"%s found:\n",message.c_str());
                        fprintf(stdout,"fx = %g\n",fx);
                        fprintf(stdout,"fy = %g\n",fy);
                        fprintf(stdout,"cx = %g\n",cx);
                        fprintf(stdout,"cy = %g\n",cy);
                    }

                    Matrix K=eye(3,3);
                    Matrix Pi=zeros(3,4);

                    K(0,0)=fx; K(1,1)=fy;
                    K(0,2)=cx; K(1,2)=cy; 
                    
                    Pi(0,0)=Pi(1,1)=Pi(2,2)=1.0; 

                    *Prj=new Matrix;
                    **Prj=K*Pi;

                    return true;
                }
            }
        }
        else
        {
            message=_rf.find("from").asString().c_str();
            message+=": intrinsic parameters for "+type;
        }

        if (verbose)
            fprintf(stdout,"%s not found!\n",message.c_str());

        return false;
    }

    /************************************************************************/
    eyeWrapper::eyeWrapper(string _name, double _hV, const ResourceFinder &_eyeCalibRF) :
               name(_name), headVersion(_hV)
    {
        if (name=="left")
        {
            eye=new iCubEye(headVersion>1.0?"left_v2":"left");
        }
        else if (name=="right")
        {
            eye=new iCubEye(headVersion>1.0?"right_v2":"right");
        }

        // remove constraints on the links
        // we use the chains for logging purpose
        eye->setAllConstraints(false);

        // release links
        eye->releaseLink(0);
        eye->releaseLink(1);
        eye->releaseLink(2);

        // add aligning matrices read from configuration file
        if (name=="left")
        {
            getAlignHN(_eyeCalibRF,"ALIGN_KIN_LEFT",eye->asChain(),true);
        }
        else if (name=="right")
        {
            getAlignHN(_eyeCalibRF,"ALIGN_KIN_RIGHT",eye->asChain(),true);
        }

        bool ret=0;

        // get camera projection matrix
        if (name=="left")
        {
            ret=getCamPrj(_eyeCalibRF,"CAMERA_CALIBRATION_LEFT",&Prj,true);
        }
        else if (name=="right")
        {
            ret=getCamPrj(_eyeCalibRF,"CAMERA_CALIBRATION_RIGHT",&Prj,true);
        }

        if (!ret)
            Prj=NULL;
    }

    eyeWrapper & eyeWrapper::operator= (const eyeWrapper &ew)
    {
        name        = ew.name;
        eye         = ew.eye;
        headVersion = ew.headVersion;
        Prj         = ew.Prj;
        return *this;
    }

// empty line to make gcc happy
