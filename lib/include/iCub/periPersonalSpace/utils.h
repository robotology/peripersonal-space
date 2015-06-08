/**
 * Copyright (C) 2013 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
**/

/**
 * \defgroup utilsLib utilsLib
 *  
 * @ingroup periPersonalSpace
 *  
 * Utilities used throughout the modules and libraries.
 *
 * \author Alessandro Roncone
 *  
 * Date: first release 30/10/2013
 *  
 * Copyright (C) 2013 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
**/ 

#ifndef __UTILS_H__
#define __UTILS_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Log.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/skinContact.h>
// #include <iCub/skinDynLib/skinPart.h>

#include <map>
#include <list> 
#include <sstream>

#include "parzenWindowEstimator.h"

/**
 * Converts a matrix to a yarp::sig::Vector with 16 elements
**/
yarp::sig::Vector toVector(yarp::sig::Matrix m);


/**
 * Closes properly a given port
**/
void closePort(yarp::os::Contactable *_port);

/**
 * Retrieves a matrix from a bottle:
 * @param b  is the bottle
 * @param in is the index from which start acquiring values
 * @param r  is the number of rows of the matrix
 * @param c  is the number of cols of the matrix
**/
yarp::sig::Matrix matrixFromBottle(const yarp::os::Bottle b, int in, const int r, const int c);

/**
 * Puts a matrix into a bottle, by cycling through its elements
 * and adding them as double
**/
void      matrixIntoBottle(const yarp::sig::Matrix m, yarp::os::Bottle &b);
void matrixOfIntIntoBottle(const yarp::sig::Matrix m, yarp::os::Bottle &b);

/**
 * Puts a Vector into a bottle, by cycling through its elements
 * and adding them as double
**/
void VectorIntoBottle(const yarp::sig::Vector v, yarp::os::Bottle &b);


/**
* Converts an int to a string
**/
std::string int_to_string( const int a );

/**
* Computes the factorial using a recursive method
**/
unsigned int factorial(unsigned int n);

/**
* Struct that encloses all the information related to a taxel.
**/
struct IncomingEvent
{
    yarp::sig::Vector Pos;
    yarp::sig::Vector Vel;
    double Radius;          // average radius of the object
    std::string Src;       // the source of information the event is coming from

    double NRM;
    double TTC;

    /**
    * Constructors
    **/    
    IncomingEvent();
    IncomingEvent(const yarp::sig::Vector &p, const yarp::sig::Vector &v,
                  const double r, const std::string &s);
    IncomingEvent(const yarp::os::Bottle &b);
    IncomingEvent(const IncomingEvent &e);

    /**
    * 
    **/    
    yarp::os::Bottle toBottle();

    /**
    * 
    **/
    bool fromBottle(const yarp::os::Bottle &b);

    /**
    * Copy Operator
    **/
    IncomingEvent &operator=(const IncomingEvent &e);

    /**
    * Print Method
    **/
    void print();

    /**
    * toString Method
    **/
    std::string toString() const;
};

/**
* It has only a couple more stuff
**/
struct IncomingEvent4TaxelPWE : public IncomingEvent
{
    double NRM;
    double TTC;

    /**
    * Constructors
    **/    
    IncomingEvent4TaxelPWE();
    IncomingEvent4TaxelPWE(const yarp::sig::Vector &p, const yarp::sig::Vector &v,
                           const double r, const std::string &s);
    IncomingEvent4TaxelPWE(const IncomingEvent &e);
    IncomingEvent4TaxelPWE(const IncomingEvent4TaxelPWE &e);

    /**
    * Copy Operators
    **/
    IncomingEvent4TaxelPWE &operator=(const IncomingEvent &e);
    IncomingEvent4TaxelPWE &operator=(const IncomingEvent4TaxelPWE &e);

    /**
    * Compute the NRM and TTC from Pos and Vel
    */
    void computeNRMTTC();

    /**
     * Return norm and TTC in a pwe-compliant way
    */
    std::vector<double> getNRMTTC();

    /**
    * Print Method
    **/
    void print();

    /**
    * toString Method
    **/
    std::string toString() const;
};

/**
* Struct that encloses all the information related to a taxel.
**/
class Taxel
{
  public:
    int ID;                    // taxels' ID
    yarp::sig::Vector px;      // (u,v) projection in the image plane
    yarp::sig::Vector Pos;     // taxel's position w.r.t. the limb
    yarp::sig::Vector WRFPos;  // taxel's position w.r.t. the root FoR
    yarp::sig::Vector Norm;    // taxel's normal   w.r.t. the limb
    yarp::sig::Matrix FoR;     // taxel's reference Frame (computed from Pos and Norm)

    /**
    * Constructors
    **/    
    Taxel();
    Taxel(const yarp::sig::Vector &p, const yarp::sig::Vector &n);
    Taxel(const yarp::sig::Vector &p, const yarp::sig::Vector &n, const int &i);

    /**
    * Copy Operator
    **/
    Taxel &operator=(const Taxel &t);

    /**
    * init function
    **/
    void init();

    /**
    * Compute and set the taxel's reference frame (from its position and its normal Vector)
    **/
    void setFoR();

    /**
    * Print Method
    **/
    virtual void print(int verbosity=0) {};

    /**
    * toString Method
    **/
    virtual std::string toString(int precision=0) {};

    /**
    * Resets the parzen window estimator
    **/
    virtual bool resetParzenWindowEstimator() {};

    /**
    * Computes the response of the taxel.
    **/
    virtual bool computeResponse() {};
};

class TaxelPWE : public Taxel
{
  public:
    int       Resp;                 // taxels' activation level (0-255)
    double RFangle;                 // angle of the receptive field [rad]

    IncomingEvent4TaxelPWE Evnt;    // 
    parzenWindowEstimator *pwe;     // 

    /**
    * Constructors
    **/    
    TaxelPWE();
    TaxelPWE(const yarp::sig::Vector &p, const yarp::sig::Vector &n);
    TaxelPWE(const yarp::sig::Vector &p, const yarp::sig::Vector &n, const int &i);

    /*
    * Destructor
    **/
    ~TaxelPWE();

    /**
    * init function
    **/
    void init() { Taxel::init(); };

    /**
    * Add or remove a sample from the pwe's histogram
    **/
    bool    addSample(IncomingEvent4TaxelPWE ie);
    bool removeSample(IncomingEvent4TaxelPWE ie);

    /**
    * Check if the input sample is inside the Receptive field (i.e. the cone)
    **/
    bool insideFoRCheck(const IncomingEvent4TaxelPWE ie);

    /**
    * Print Method
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    **/
    std::string toString(int precision=0);

    /**
    * Resets the parzen window estimator
    **/
    bool resetParzenWindowEstimator();

    /**
    * Computes the response of the taxel.
    **/
    bool computeResponse();

    /**
    * Convert the taxel into a bottle in order to be saved on file
    **/
    yarp::os::Bottle TaxelPWEIntoBottle();
};

class TaxelPWE1D : public TaxelPWE
{
  public:

    /**
    * Constructors
    **/    
    TaxelPWE1D() : TaxelPWE()                                                    { pwe = new parzenWindowEstimator1D();};
    TaxelPWE1D(const yarp::sig::Vector &p, const yarp::sig::Vector &n) : TaxelPWE(p,n)                 { pwe = new parzenWindowEstimator1D();};
    TaxelPWE1D(const yarp::sig::Vector &p, const yarp::sig::Vector &n, const int &i) : TaxelPWE(p,n,i) { pwe = new parzenWindowEstimator1D();};

    /**
    * init function
    **/
    void init() { TaxelPWE::init(); };
};

class TaxelPWE2D : public TaxelPWE
{
  public:

    /**
    * Constructors
    **/    
    TaxelPWE2D() : TaxelPWE()                                                    { pwe = new parzenWindowEstimator2D();};
    TaxelPWE2D(const yarp::sig::Vector &p, const yarp::sig::Vector &n) : TaxelPWE(p,n)                 { pwe = new parzenWindowEstimator2D();};
    TaxelPWE2D(const yarp::sig::Vector &p, const yarp::sig::Vector &n, const int &i) : TaxelPWE(p,n,i) { pwe = new parzenWindowEstimator2D();};

    /**
    * init function
    **/
    void init() { TaxelPWE::init(); };
};

/**
* Class that encloses all the information related to a skinpart.
**/
class skinPart
{
  public:
    iCub::skinDynLib::SkinPart name;
    int size;   // theoretical maximum size of the skinPart if the patches were full ~ corresponds to number of values on the respective port 
    //and number of columns in the .txt files in icub-main/app/skinGui/conf/positions
    //IMPORTANT: it differs from taxel.size(), that is the real size of the skinPart with "active" or valid taxels
             
    /**
    * Indexing variable used in the case of reducing the resolution - e.g. taking only triangle centers
    * The index into the Vector is the taxel ID, the value stored is its representative
    **/
    std::vector<int> Taxel2Repr; 

    /**
    * Mapping in the opposite direction
    * Indexed by representative taxel IDs, it stores lists of the taxels being represented 
    * e.g. all taxels of a triangle
    **/
    std::map<unsigned int, std::list<unsigned int> > Repr2TaxelList;

    /**
    * Constructor
    **/    
    skinPart();
    // skinPart(const std::string _name);

    /**
    * Copy Operator
    **/
    virtual skinPart &operator=(const skinPart &spw);

    /**
    * Print Method
    **/
    virtual void print(int verbosity=0);

    /**
    * toString Method
    **/
    virtual std::string toString(int precision=0);
};

class skinPartTaxel : public skinPart
{
  public:
    /**
    * List of taxels that belong to the skinPart.
    **/
    std::vector<Taxel*> txls;

    /**
    * Destructor
    **/
    ~skinPartTaxel();

    /**
    * Copy Operator
    **/
    skinPartTaxel &operator=(const skinPartTaxel &spw);

    /**
    * Print Method
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    **/
    std::string toString(int precision=0);
};

class skinPartPWE : public skinPart
{
  public:
    /**
    * Modality (either 1D or 2D)
    */
    std::string modality;

    /*
    * List of taxelsPWE that belong to the skinPart (either 1D or 2D)
    **/
    std::vector<TaxelPWE*> txls;

    /*
    * Constructor that assigns modality member
    **/
    skinPartPWE(const std::string &_modality) : skinPart(), modality(_modality) { txls.clear(); };

    /**
    * Destructor
    **/
    ~skinPartPWE();

    /**
    * Copy Operator
    **/
    skinPartPWE &operator=(const skinPartPWE &spw);

    /**
    * Print Method
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    **/
    std::string toString(int precision=0);

    int get_taxelSize() { return txls.size(); };
};

/**
* Struct that encloses all the information related to the eyes.
* It's used for the projections.
**/
class eyeWrapper
{
public:
    std::string name;

    iCub::iKin::iCubEye *eye;

    double headVersion;
    yarp::sig::Matrix *Prj;

public:
    /**
    * Constructor
    **/    
    eyeWrapper(std::string _name, double _hV, const yarp::os::ResourceFinder &_eyeCalibRF);

    /**
    * Copy Operator
    **/
    eyeWrapper &operator=(const eyeWrapper &ew);
};

#endif

// empty line to make gcc happy
