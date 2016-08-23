#include "iCub/periPersonalSpace/taxelPWE.h"

using namespace  yarp::os;
using namespace yarp::sig;
using namespace       std;

/****************************************************************/
/* TAXEL WRAPPER FOR PWE
*****************************************************************/

    TaxelPWE::TaxelPWE() : Taxel(), Evnt()
    {
        Resp    = 0.0;
        RFangle = 40*M_PI/180;
    }

    TaxelPWE::TaxelPWE(const Vector &p,
                       const Vector &n) : Taxel(p,n), Evnt()
    {
        Resp    = 0.0;
        RFangle = 40*M_PI/180;
    };

    TaxelPWE::TaxelPWE(const Vector &p,
                       const Vector &n,
                       const int &i) : Taxel(p,n,i), Evnt()
    {
        Resp    = 0.0;
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

    bool TaxelPWE::computeResponse(double stress_modulation)
    {
        if (!insideFoRCheck(Evnt))
        {
            Resp = 0.0;
            return false;
        }

        std::vector<double> In = Evnt.getNRMTTC();
        Resp = pwe->computeResponse(In);

        Resp = Resp + (Resp * min(1.0,Evnt.Threat + stress_modulation)); //with this amplification,
        //may come out of the range (which used to be <0,255>, now <0,1> after 9.8.2016)
        //- in fact double that range.
        //printf("TaxelPWE::computeResponse: %f\n",Resp);

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

// empty line to make gcc happy
