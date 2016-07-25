#include "iCub/periPersonalSpace/utils.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::iKin;
using namespace iCub::skinDynLib;

using namespace std;

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

void matrixOfIntIntoBottle(const yarp::sig::Matrix m, Bottle &b)
{
    Vector v = toVector(m);
    
    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addInt(int(v[i]));
    }
}

string int_to_string( const int a )
{
    std::stringstream ss;
    ss << a;
    return ss.str();
}

unsigned int factorial(unsigned int n) 
{
    if (n == 0)
       return 1;
    return n * factorial(n - 1);
}

/****************************************************************/
/* INCOMING EVENT WRAPPER
*****************************************************************/
    IncomingEvent::IncomingEvent()
    {
        Pos.resize(3,0.0);
        Vel.resize(3,0.0);
        Src="";
        Radius=-1.0;
        threat = 0.0;
    }

    IncomingEvent::IncomingEvent(const Vector &p, const Vector &v, const double r, const string &s)
    {
        Pos = p;
        Vel = v;
        Src = s;
        Radius = r;
        threat = 0.0;
    }
    
    IncomingEvent::IncomingEvent(const Vector &p, const Vector &v, const double r, const double _threat, const std::string &s)
    {
        Pos = p;
        Vel = v;
        Src = s;
        Radius = r;
        threat = _threat;
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
        b.addDouble(threat);

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
        threat = b.get(8).asDouble();

        return true;
    }

    void IncomingEvent::print()
    {
        yDebug("\tPos: %s\t Vel: %s\t Radius %g\t Src %s Threat: %g\n",Pos.toString().c_str(),Vel.toString().c_str(),Radius,Src.c_str(),threat);
    }

    string IncomingEvent::toString() const
    {
        stringstream res;
        res << "Pos: "<< Pos.toString(3,3) << "\t Vel: "<< Vel.toString(3,3)
            << "\t Radius: "<< Radius << "\t Src: "<< Src << "\t threat: "<< threat ;
        return res.str();
    }

/****************************************************************/
/* INCOMING EVENT 4 TAXEL WRAPPER
*****************************************************************/
    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE() : IncomingEvent()
    {
        NRM = 0;
        TTC = 0;
    }

    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE(const Vector &p, const Vector &v,
                                                   const double r, const string &s) :
                                                   IncomingEvent(p,v,r,s)
    {
        computeNRMTTC();
    }

    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE(const IncomingEvent4TaxelPWE &e)
    {
        *this = e;
        computeNRMTTC();
    }

    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE(const IncomingEvent &e)
    {
        *this = e;   
    }

    IncomingEvent4TaxelPWE & IncomingEvent4TaxelPWE::operator=(const IncomingEvent4TaxelPWE &e)
    {
        IncomingEvent::operator=(e);
        TTC    = e.TTC;
        NRM    = e.NRM;
        return *this;
    }

    IncomingEvent4TaxelPWE & IncomingEvent4TaxelPWE::operator=(const IncomingEvent &e)
    {
        IncomingEvent::operator=(e);
        computeNRMTTC();
        return *this;
    }

    void IncomingEvent4TaxelPWE::computeNRMTTC()
    {
        int sgn = Pos[2]>=0?1:-1;
        NRM = sgn * norm(Pos);

        // if (norm(Vel) < 0.38 && norm(Vel) > 0.34)
        // {
        //     TTC = 10000.0;
        // }
        // else
        // 
        if (dot(Pos,Vel)==0) TTC = 0;
        else                 TTC = -norm(Pos)*norm(Pos)/dot(Pos,Vel);
    }

    std::vector<double> IncomingEvent4TaxelPWE::getNRMTTC()
    {
        std::vector<double> x;
        x.push_back(NRM);
        x.push_back(TTC);

        return x;
    }

    void IncomingEvent4TaxelPWE::print()
    {
        yDebug("\tNRM: %g\t TTC: %g \t %s", NRM, TTC, IncomingEvent::toString().c_str());
    }

    string IncomingEvent4TaxelPWE::toString() const
    {
        stringstream res;
        res << "NRM: "<< NRM << "\t TTC: " << TTC << "\t "<< IncomingEvent::toString();
        return res.str();
    }

/****************************************************************/
/* TAXEL WRAPPER
*****************************************************************/
    // void Taxel::init()
    // {
    //     ID      = 0;
    //     Pos.resize(3,0.0);
    //     WRFPos.resize(3,0.0);
    //     Norm.resize(3,0.0);
    //     px.resize(2,0.0);
    //     FoR = eye(4);
    // }

    // Taxel::Taxel()
    // {
    //     init();
    // }

    // Taxel::Taxel(const Vector &p, const Vector &n)
    // {
    //     init();
    //     Pos  = p;
    //     Norm = n;
    //     setFoR();
    // }

    // Taxel::Taxel(const Vector &p, const Vector &n, const int &i)
    // {
    //     init();
    //     ID   = i;
    //     Pos  = p;
    //     Norm = n;
    //     setFoR();
    // }

    // Taxel & Taxel::operator=(const Taxel &t)
    // {
    //     ID      = t.ID;
    //     Pos     = t.Pos;
    //     WRFPos  = t.WRFPos;
    //     Norm    = t.Norm;
    //     px      = t.px;
    //     FoR     = t.FoR;
    //     return *this;
    // }

    // void Taxel::setFoR()
    // {
    //     if (Norm == zeros(3))
    //     {
    //         FoR=eye(4);
    //         return;
    //     }
        
    //     // Set the proper orientation for the touching end-effector
    //     Vector x(3,0.0), z(3,0.0), y(3,0.0);

    //     z = Norm;
    //     if (z[0] == 0.0)
    //     {
    //         z[0] = 0.00000001;    // Avoid the division by 0
    //     }
    //     y[0] = -z[2]/z[0]; y[2] = 1;
    //     x = -1*(cross(z,y));
        
    //     // Let's make them unitary vectors:
    //     x = x / norm(x);
    //     y = y / norm(y);
    //     z = z / norm(z);

    //     FoR=eye(4);
    //     FoR.setSubcol(x,0,0);
    //     FoR.setSubcol(y,0,1);
    //     FoR.setSubcol(z,0,2);
    //     FoR.setSubcol(Pos,0,3);
    // }

/****************************************************************/
/* TAXEL WRAPPER FOR PWE
*****************************************************************/

    TaxelPWE::TaxelPWE() : Taxel(), Evnt()
    {
        Resp    = 0;
        RFangle = 40*M_PI/180;
    }

    TaxelPWE::TaxelPWE(const Vector &p,
                       const Vector &n) : Taxel(p,n), Evnt()
    {
        Resp    = 0;
        RFangle = 40*M_PI/180;
    };

    TaxelPWE::TaxelPWE(const Vector &p,
                       const Vector &n,
                       const int &i) : Taxel(p,n,i), Evnt()
    {
        Resp    = 0;
        RFangle = 40*M_PI/180;
    };

    bool TaxelPWE::addSample(IncomingEvent4TaxelPWE ie)
    {
        if (!insideFoRCheck(ie))
            return false;

        std::vector <double> x = ie.getNRMTTC();
        // printf("[TaxelPWE::addSample] x %g %g\n",x[0],x[1]);
        
        return pwe->addSample(x);
    }

    bool TaxelPWE::removeSample(IncomingEvent4TaxelPWE ie)
    {
        if (!insideFoRCheck(ie))
            return false;

        std::vector <double> x = ie.getNRMTTC();
        return pwe->removeSample(x);
    }

    bool TaxelPWE::insideFoRCheck(const IncomingEvent4TaxelPWE ie)
    {
        std::vector<double> binWidth = pwe->getBinWidth();
        double binLimit = 8*binWidth[0];

        // the x,y limit of the receptive field at the incoming event's Z
        double RFlimit = ie.Pos(2)/tan(RFangle);

        // the x,y limit of the receptive field in the first bin
        double RFlimit_cyl = binLimit/tan(RFangle);

        // yDebug("binLimit: %g RFlimit_cyl: %g RFangle: %g \n", binLimit, RFlimit_cyl, RFangle);
        // yDebug("ie.Pos\t%s\n", ie.Pos.toString(3,3).c_str());
        // yDebug("Hist:\n%s\n", pwe->getHist().toString(3,3).c_str());

        if (ie.Pos(0)*ie.Pos(0)+ie.Pos(1)*ie.Pos(1) < RFlimit*RFlimit )
        {
            return true;
        }
        // There are two ifs only to let me debug things
        if ( (abs(ie.Pos(2))<=binLimit) && (ie.Pos(0)*ie.Pos(0)+ie.Pos(1)*ie.Pos(1) < RFlimit_cyl*RFlimit_cyl) )
        {
            return true;
        }
        return false;
    }

    void TaxelPWE::print(int verbosity)
    {
        yDebug("[TAXEL] %s", iCub::skinDynLib::Taxel::toString(verbosity).c_str());
        if (verbosity > 3)
        {
            yDebug("[PWE] %s",pwe->toString(verbosity).c_str());
        }
    }

    string TaxelPWE::toString(int verbosity)
    {
        stringstream res;
        res << "[TAXEL] " << iCub::skinDynLib::Taxel::toString(verbosity);

        if (verbosity)
        {
            res << "[PWE] " << pwe->toString(verbosity);
        }
        return res.str();
    }

    bool TaxelPWE::resetParzenWindowEstimator()
    {
        pwe->resetAllHist();
        return true;
    }

    bool TaxelPWE::computeResponse()
    {
        if (!insideFoRCheck(Evnt))
        {
            Resp = 0;
            return false;
        }

        std::vector<double> In = Evnt.getNRMTTC();
        Resp = pwe->computeResponse(In);

        return true;
    }

    Bottle TaxelPWE::TaxelPWEIntoBottle()
    {
        Bottle res;
        res.clear();
        res.addInt(ID);

        Bottle &dataPH = res.addList();
        matrixOfIntIntoBottle(pwe->getPosHist(),dataPH);

        Bottle &dataNH = res.addList();
        matrixOfIntIntoBottle(pwe->getNegHist(),dataNH);

        return res;
    }

    TaxelPWE::~TaxelPWE()
    {
        if (pwe!=NULL)
        {
            delete pwe;
        }
    }

/****************************************************************/
/* TAXEL WRAPPER FOR PWE 1D                                     */
/****************************************************************/
    TaxelPWE1D::TaxelPWE1D() : TaxelPWE()
    {
        pwe = new parzenWindowEstimator1D();
    }

    TaxelPWE1D::TaxelPWE1D(const yarp::sig::Vector &p,
                           const yarp::sig::Vector &n) : TaxelPWE(p,n)
    {
        pwe = new parzenWindowEstimator1D();
    }

    TaxelPWE1D::TaxelPWE1D(const yarp::sig::Vector &p,
                           const yarp::sig::Vector &n,
                           const int &i) : TaxelPWE(p,n,i)
    {
        pwe = new parzenWindowEstimator1D();
    }

    TaxelPWE1D::TaxelPWE1D(const Taxel &_t)
    {
        *this = _t;
    }

    TaxelPWE1D::TaxelPWE1D(const TaxelPWE1D &_t)
    {
        *this = _t;
    }

    TaxelPWE1D & TaxelPWE1D::operator=(const Taxel &t)
    {
        if (this == &t)
        {
            return *this;
        }

        iCub::skinDynLib::Taxel::operator=(t);

        // if (pwe)
        // {
        //     delete pwe;
        // }
        pwe = new parzenWindowEstimator1D();

        return *this;
    }

    TaxelPWE1D & TaxelPWE1D::operator=(const TaxelPWE1D &t)
    {
        if (this == &t)
        {
            return *this;
        }

        iCub::skinDynLib::Taxel::operator=(t);

        Evnt = t.Evnt;

        if (pwe)
        {
            delete pwe;
        }

        parzenWindowEstimator1D* newpwe = dynamic_cast<parzenWindowEstimator1D*>(t.pwe);
        pwe = new parzenWindowEstimator1D(*(newpwe));

        return *this;
    }

/****************************************************************/
/* TAXEL WRAPPER FOR PWE 2D                                     */
/****************************************************************/
    TaxelPWE2D::TaxelPWE2D() : TaxelPWE()
    {
        pwe = new parzenWindowEstimator2D();
    }

    TaxelPWE2D::TaxelPWE2D(const yarp::sig::Vector &p,
                           const yarp::sig::Vector &n) : TaxelPWE(p,n)
    {
        pwe = new parzenWindowEstimator2D();
    }

    TaxelPWE2D::TaxelPWE2D(const yarp::sig::Vector &p,
                           const yarp::sig::Vector &n,
                           const int &i) : TaxelPWE(p,n,i)
    {
        pwe = new parzenWindowEstimator2D();
    }


    TaxelPWE2D::TaxelPWE2D(const Taxel &_t)
    {
        *this = _t;
    }

    TaxelPWE2D::TaxelPWE2D(const TaxelPWE2D &_t)
    {
        *this = _t;
    }

    TaxelPWE2D & TaxelPWE2D::operator=(const Taxel &t)
    {
        if (this == &t)
        {
            return *this;
        }

        iCub::skinDynLib::Taxel::operator=(t);

        // if (pwe)
        // {
        //     delete pwe;
        // }
        pwe = new parzenWindowEstimator2D();

        return *this;
    }

    TaxelPWE2D & TaxelPWE2D::operator=(const TaxelPWE2D &t)
    {
        if (this == &t)
        {
            return *this;
        }

        iCub::skinDynLib::Taxel::operator=(t);

        Evnt = t.Evnt;

        if (pwe)
        {
            delete pwe;
        }

        parzenWindowEstimator2D* newpwe = dynamic_cast<parzenWindowEstimator2D*>(t.pwe);
        pwe = new parzenWindowEstimator2D(*(newpwe));

        return *this;
    }

// /****************************************************************/
// /* SKINPART WRAPPER
// *****************************************************************/
//     skinPart::skinPart()
//     {
//         name = SKIN_PART_UNKNOWN;
//         size =                 0;
//     }

//     // skinPart::skinPart(const string _name)
//     // {
//     //     name = _name;
//     //     size =     0;
//     // }

//     skinPart & skinPart::operator=(const skinPart &spw)
//     {
//         name           = spw.name;
//         size           = spw.size;
//         Taxel2Repr     = spw.Taxel2Repr;
//         Repr2TaxelList = spw.Repr2TaxelList;
//         return *this;
//     }

//     void skinPart::print(int verbosity)
//     {
//         yDebug("**********\n");
//         yDebug("name: %s\t", SkinPart_s[name].c_str());
//         yDebug("size: %i\n", size);
//         yDebug("**********\n");
        
//         if (verbosity>=4)
//         {
//             yDebug("\nTaxel ID -> representative ID:\n");

//             for (size_t i=0; i<size; i++)
//             {
//                 yDebug("[ %lu -> %d ]\t",i,Taxel2Repr[i]);
//                 if (i % 8 == 7)
//                 {
//                     yDebug("\n");
//                 }
//             }
//             yDebug("\n");
            
//             yDebug("Representative ID -> Taxel IDs:\n");
//             for(map<unsigned int, list<unsigned int> >::const_iterator iter_map = Repr2TaxelList.begin(); iter_map != Repr2TaxelList.end(); ++iter_map)
//             {
//                 list<unsigned int> l = iter_map->second;
//                 yDebug("%d -> {",iter_map->first);
//                 for(list<unsigned int>::const_iterator iter_list = l.begin(); iter_list != l.end(); iter_list++)
//                 {
//                     yDebug("%u, ",*iter_list);
//                 }
//                 yDebug("}\n");
//             }    
//             yDebug("\n");
//         }
//         yDebug("**********\n");
//     }

//     string skinPart::toString(int precision)
//     {
//         stringstream res;
//         res << "**********\n" << "Name: " << SkinPart_s[name] << "\tSize: "<< size << endl;
//         return res.str();
//     }

// /****************************************************************/
// /* SKINPART TAXEL WRAPPER
// *****************************************************************/
//     skinPartTaxel & skinPartTaxel::operator=(const skinPartTaxel &spw)
//     {
//         skinPart::operator=(spw);
//         taxels     = spw.taxels;
//         return *this;
//     }

//     void skinPartTaxel::print(int verbosity)
//     {
//         skinPart::print(verbosity);
//         for (size_t i = 0; i < taxels.size(); i++)
//             taxels[i]->print(verbosity);
//         yDebug("**********\n");
//     }

//     string skinPartTaxel::toString(int precision)
//     {
//         stringstream res(skinPart::toString(precision));
//         for (size_t i = 0; i < taxels.size(); i++)
//             res << taxels[i]->toString(precision);
//         res << "**********\n";
//         return res.str();
//     }

//     skinPartTaxel::~skinPartTaxel()
//     {
//         // while(!taxels.empty())
//         // {
//         //     if (taxels.back())
//         //     {
//         //         delete taxels.back();
//         //     }
//         //     taxels.pop_back();
//         // }
//     }

/****************************************************************/
/* SKINPART TAXEL PWE WRAPPER
*****************************************************************/
    skinPartPWE::skinPartPWE(const std::string &_modality) :
                             skinPart(), modality(_modality)
    {
        clearTaxels();
    }

    skinPartPWE::skinPartPWE(const skinPartPWE &_spwe)
    {
        *this = _spwe;
    }

    skinPartPWE & skinPartPWE::operator=(const skinPartPWE &spw)
    {
        if (this == &spw)
        {
            return *this;
        }

        iCub::skinDynLib::skinPart::operator=(spw);

        modality  = spw.modality;

        clearTaxels();
        if (modality=="1D")
        {
            for (std::vector<Taxel*>::const_iterator it = spw.taxels.begin();
                 it != spw.taxels.end(); ++it)
            {
                taxels.push_back(new TaxelPWE1D(*(*it)));
            }
        }
        else if (modality=="2D")
        {
            for (std::vector<Taxel*>::const_iterator it = spw.taxels.begin();
                 it != spw.taxels.end(); ++it)
            {
                taxels.push_back(new TaxelPWE2D(*(*it)));
            }
        }
        
        return *this;
    }

    void skinPartPWE::print(int verbosity)
    {
        iCub::skinDynLib::skinPart::print(verbosity);
        for (size_t i = 0; i < taxels.size(); i++)
            taxels[i]->print(verbosity);
        yDebug("**********\n");
    }

    string skinPartPWE::toString(int precision)
    {
        stringstream res(iCub::skinDynLib::skinPart::toString(precision));
        for (size_t i = 0; i < taxels.size(); i++)
            res << taxels[i]->toString(precision);
        res << "**********\n";
        return res.str();
    }

    skinPartPWE::~skinPartPWE()
    {
        // printf("Taxelsize %lu %i\n", taxels.size(), get_taxelSize());
        // int i=0;

        // while(!taxels.empty())
        // {
        //     printf("i %i\n", i); i++;
        //     if (taxels.back())
        //     {
        //         delete taxels.back();
        //     }
        //     taxels.pop_back();
        // }
    }

/****************************************************************/
/* EYE WRAPPER
*****************************************************************/
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
