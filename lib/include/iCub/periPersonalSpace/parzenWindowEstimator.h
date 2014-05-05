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

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::iKin;

using namespace std;

/**
* class for defining a 1-D parzen window 
**/
class parzenWindowEstimator
{
private:
    double ext;    // the maximum extension of the Receptive Field
    int    hSize;  // the number of partitions of the input space
    double sExt;   // the extension of the single sampling unit

    Vector sigm;   // sigma of the gaussians (by default they're all equal)
    Vector hist;   // histogram of the 


public:
    parzenWindowEstimator();
    parzenWindowEstimator(const double e, const int ns);

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
    * Resize the estimator to a given extension and number of samples
    * The histogram changes accordingly (and it's cleared as well).
    **/
    bool resize(const double e, const int ns);

    /**
    * Add or remove a sample from the histogram
    **/
    bool addSample(const double x);

    bool removeSample(const double x);

    /**
    *
    **/
    int  getHistSize() { return hSize; };
    int  getHist(const int i) { return int(hist[i]); };
    yarp::sig::Vector getHist() { return hist; };
    void setHist(const int i, const int val) { hist[i] = val; };
    void setHist(const yarp::sig::Vector &v) { hist = v; };
    void resetHist() { hist.zero(); };
    double getExt() { return ext; };

    /**
    * Print Function
    **/
    void print();
};

#endif

// empty line to make gcc happy
