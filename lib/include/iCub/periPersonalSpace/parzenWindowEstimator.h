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
 * \defgroup parzenWindowEstimatorLib parzenWindowEstimatorLib
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

#ifndef __PARZENWINDOWESTIMATOR_H__
#define __PARZENWINDOWESTIMATOR_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

#include <vector>
#include <sstream>

#include <cmath>

// These are basically some semi-useless constants. I can remove them but
// I've preserved them in order for me to ease the process of future improvements
// to the library
#define X_DIM 0
#define Y_DIM 1
#define START 0
#define END 1

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::iKin;

using namespace std;

/**
* class for defining a 1-D parzen window 
**/
class parzenWindowEstimator1D
{
  private:
    double ext;    // the maximum extension of the Receptive Field
    int    binsNum;  // the number of partitions of the input space
    double binWidth;   // the extension of the single sampling unit

    Vector sigm;   // sigma of the gaussians (by default they're all equal)
    Vector hist;   // histogram of the 


  public:
    /**
    * Constructors
    **/
    parzenWindowEstimator1D();
    parzenWindowEstimator1D(const double e, const int hs);

    /**
    * Resize the estimator to a given extension and number of samples
    * The histogram changes accordingly (and it's cleared as well).
    **/
    bool resize(const double e, const int hs);

    /**
    * Add or remove a sample from the histogram
    **/
    bool addSample(const double x);
    bool removeSample(const double x);

    /**
    * Get the value of the receptive field at a certain x
    **/
    double getF_X(const double x);

    /**
    * Get the value of the receptive field at a certain x.
    * It differs from the previous because it's scaled
    * (i.e. its max is set to 255, the other values accordingly)
    **/
    double getF_X_scaled(const double x);

    /**
    * Print Function
    **/
    void print();

    /**
    * Self-explaining functions
    **/
    int  getHistSize()                       { return binsNum; };
    int  getHist(const int i)                { return int(hist[i]); };
    yarp::sig::Vector getHist()              { return hist; };
    void setHist(const int i, const int val) { hist[i] = val; };
    void setHist(const yarp::sig::Vector &v) { hist = v; }
    double getExt()                          { return ext; };
    void resetHist()                         { hist.zero(); };
};

/**
* class for defining a 2-D parzen window with a custom range (even negative)
**/
class parzenWindowEstimator2D
{
  private:
    std::vector<double> extX;   // the extension of the Receptive Field in the x dimension
    std::vector<double> extY;   // the extension of the Receptive Field in the y dimension

    std::vector<int>    binsNum;  // the number of partitions of the input space (x and y dimensions)
    std::vector<double> binWidth;   // the extension of the single sampling unit (x and y dimensions)

    std::vector<int>    firstPosBin; // the first bin for which we have positive values
    std::vector<double> firstPosBinShift; // the shift from zero to the start value of the firstPosBin

    std::vector<double> binStartsX; //these are initialed at startup to contain the star, midpoint and end of each bin in the x dim.
    std::vector<double> binStartsY; //these are initialed at startup to contain the star, midpoint and end of each bin in the y dim.
    
    double sigmX;   // sigma of the gaussians in the x dimension (by default they're all equal)
    double sigmY;   // sigma of the gaussians in the y dimension (by default they're all equal)

    Matrix posHist; // histogram for the parzening - positive examples 
    Matrix negHist; //negative examples 
  public:
    /**
    * Constructors
    **/
    parzenWindowEstimator2D();
    parzenWindowEstimator2D(const std::vector<double> eX, const std::vector<double> eY, const std::vector<int> hS);

    /**
    * Resize the estimator to a given extension and number of samples
    * The histogram changes accordingly (and it's cleared as well).
    **/
    bool resize(const std::vector<double> eX, const std::vector<double> eY, const std::vector<int> hS);

    /**
    * Add or remove a sample from the histogram
    **/
    bool addSample(const std::vector<double> x);
    bool removeSample(const std::vector<double> x);

    /**
    * Compute the response for a specific input sample
    **/
    int computeResponse(const std::vector<double> x);

    
    /**
    * Check an input is inside the receptive fields, and if so assigns
    * the proper histogram indexes that belong to that input vector.
    **/
    bool getIndexes(const std::vector<double> x, int &b0, int &b1);

    /**
    * Get the value of the receptive field at a certain x
    **/
    double getF_X(const std::vector<double> x);
    
    /**
    * Get the value of the receptive field at a certain x.
    * It differs from the previous because it's scaled
    * (i.e. its max is set to 255, the other values accordingly)
    **/
    double getF_X_scaled(const std::vector<double> x);

        
    /**
    * Print Function
    **/
    void print();

    /**
    * Self-explaining functions
    **/
    std::vector<int>    getHistSize()                           { return binsNum; };
    std::vector<double> getBinWidth()                           { return binWidth; };
    std::vector<double> getExtX()                               { return extX; };
    std::vector<double> getExtY()                               { return extY; };

    double getHist(const int i, const int j);
    int    getPosHist(const int i, const int j)                 { return int(posHist(i,j)); };
    int    getNegHist(const int i, const int j)                 { return int(negHist(i,j)); };

    yarp::sig::Matrix getPosHist()                              { return posHist; };
    yarp::sig::Matrix getNegHist()                              { return negHist; };
    yarp::sig::Matrix getHist();

    void setPosHist(const int i, const int j, const int val)    { posHist(i,j) = val; };
    void setNegHist(const int i, const int j, const int val)    { negHist(i,j) = val; };
    void setPosHist(const yarp::sig::Matrix &v)                 { posHist = v; };
    void setNegHist(const yarp::sig::Matrix &v)                 { negHist = v; };

    void resetAllHist()                                         { posHist.zero(); negHist.zero(); };
};

#endif

// empty line to make gcc happy
