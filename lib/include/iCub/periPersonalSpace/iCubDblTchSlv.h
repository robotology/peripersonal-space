/* 
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
*/

/**
 * \defgroup doubleTouchSlvLib doubleTouchSlvLib
 *  
 * @ingroup periPersonalSpace
 *  
 * Classes for solving the double touch, i.e. providing an inverse
 * kinematics solver able to achieve a successfull double touch.
 *  
 * \note <b>SI units adopted</b>: meters for lengths and radians
 *       for angles.
 *
 * \author Alessandro Roncone - Ugo Pattacini 
 *  
 * Date: first release 30/06/2013
 *  
 * Copyright (C) 2013 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */ 

#include "iKinFwdMod.h"

using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace std;

/**
* @ingroup doubleTouchSlvLib
*
* A struct for defining the problem's variables (from the optimizer's standpoint). 
* Theroretically, a part from the type of task (R2L or L2R or whatever), any 
* double touch problem should be self-contained in such a class (apart from the
* chain, that is not a parameter :P)
*  
*/
struct iCubDoubleTouch_Variables
{    
    yarp::sig::Vector ee;
    yarp::sig::Vector joints;

    yarp::sig::Matrix H;
    yarp::sig::Matrix H_0;

    yarp::sig::Vector z_hat;
    yarp::sig::Vector x_hat;

    double dot;

    /**
    * Constructor. 
    * @param dim is the number of links of the chain to be dealt with. Anything
    *            else should be fitted by hand by the operator (and it's fine,
    *            because there's no need to hide the members of such a simple
    *            struct to the final user)
    */
    iCubDoubleTouch_Variables(int dim);

    /**
    * Prints the state of the variable
    */
    void print();

    /**
    * Copy Operator
    */
    iCubDoubleTouch_Variables &operator=(const iCubDoubleTouch_Variables &v);

    /**
    * Clone Function
    */
    void clone (const iCubDoubleTouch_Variables &v);
};

/**
* @ingroup doubleTouchSlvLib
*
* A struct for defining a subProblem (right now, either "R2L" or "L2R"). It encloses everything
* is needed for the solver to solve the problem (also the chain!).
* 
*/
struct iCubDoubleTouch_SubProblem
{
    iCubCustomLimb              limb;
    iCubFinger                  index;
    iKinLinIneqConstr          *mLIC;
    iKinLinIneqConstr          *sLIC;
    int                         nJoints;
    int                         nVars;
    iCubDoubleTouch_Variables   guess;

    std::string        getType()    { return limb.getType(); }
    int                getNVars()   { return nVars; }
    iKinLinIneqConstr* getMLIC()    { return mLIC; }
    iKinLinIneqConstr* getSLIC()    { return sLIC; }
    iKinChainMod*      asChainMod() { return limb.asChainMod(); }
    iCubCustomLimb*    getLimb()    { return &limb; }

    iCubDoubleTouch_SubProblem(string _type, string _indextype);
    iCubDoubleTouch_SubProblem & operator=(const iCubDoubleTouch_SubProblem &sp);
};

/**
* @ingroup doubleTouchSlvLib
*
* A struct for defining a double touch problem in its entirety. It's a wrapper for the underlying
* subproblems. Hence, any subproblem can be run at any time and they can be run in parallel without
* reconfiguring the solver over and over again.
*  
*/
struct iCubDoubleTouch_Problem
{
    iCubDoubleTouch_SubProblem *R2L;
    iCubDoubleTouch_SubProblem *L2R;

    // iCubDoubleTouch_Problem(const iCubDoubleTouch_SubProblem &_R2L): R2L(*_R2L) {}
    // iCubDoubleTouch_Problem(const iCubDoubleTouch_SubProblem &_R2L,
    //                         const iCubDoubleTouch_SubProblem &_L2R): R2L(*_R2L), L2R(*_L2R) {}
    /**
    * Constructor. 
    * @param type is the type of subproblem that has to be instantiated. Right now, it can be
    *             either "R2L" or "L2R" or "both"
    */
    iCubDoubleTouch_Problem(std::string _type);

    /**
    * Destructor.
    */
    ~iCubDoubleTouch_Problem();
};

/**
* @ingroup doubleTouchSlvLib
*
* A class for implementing a solver for doubletouch. It what is needed, and nothing more: the
* problem, a pointer to the subproblem, a way to detect the problem under investigation.
*  
*/
class iCubDoubleTouch_Solver
{
    protected:
        iCubDoubleTouch_Problem     problem;
        iCubDoubleTouch_SubProblem *current_subproblem;
        string current_subproblem_type;

    public:
        /**
        * Constructor. 
        * @param type is the type of subproblem that has to be instantiated. Right now, it can be
        *             either "R2L" or "L2R" or "both"
        */
        iCubDoubleTouch_Solver(string _type);

        /**
        * Constructor. 
        * @param problem is a problem already instantiated somewhere else.
        */
        iCubDoubleTouch_Solver(iCubDoubleTouch_Problem &_problem);

        /**
        * Gets the a subproblem among the possible choices.
        * @param type is the type of subproblem that has to be retrieved.
        */
        iCubDoubleTouch_SubProblem* getSubProblem(string _type);

        /**
        * Sets a specific subproblem among the possible choices.
        * @param type is the type of subproblem that has to be set.
        */
        bool setSubProblem (string _type);

        /**
        * Sets the initial parameters from which starting the optimization.
        */
        void setInitialGuess(const iCubDoubleTouch_Variables &g);

        /**
        * Solves the Inverse Kinematics task associated with the double touch.
        */
        bool solve(iCubDoubleTouch_Variables &solution);
};

// empty line to make gcc happy