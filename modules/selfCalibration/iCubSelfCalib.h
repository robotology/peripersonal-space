#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/iKin/iKinSlv.h>

#include "iCub/periPersonalSpace/iKinFwdMod.h"

#include <vector>

using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace std;

/****************************************************************/
/* selfCalib_ITEM
/****************************************************************/
    struct selfCalib_Item
    {    
        yarp::sig::Vector encsS;    // encoders slave
        yarp::sig::Vector encsM;    // encoders master

        yarp::sig::Matrix actualRF;  // given ref frame (from the skin)
        yarp::sig::Vector actualPos; // given position (from the skin)

        yarp::sig::Vector joints;   // joint values (slave + master)
        yarp::sig::Matrix fingerHN; // transform matrix of the finger
        string type;                // type of the task

        yarp::sig::Vector estPos; // estimated position (from the model)

        /**
        * Constructor. 
        */
        selfCalib_Item();

        /**
        * Prints the state of the variable
        */
        void print(int verbosity=0);

        /**
        * Copy Operator
        */
        selfCalib_Item &operator=(const selfCalib_Item &v);

        /**
        * Clone Function
        */
        void clone (const selfCalib_Item &v);

        /**
        * fromString Function
        */
        void fromString(const string &line);

        /**
        * compute quantities from the values given
        */
        void computeQuantities();
    };

/****************************************************************/
/* selfCalib_DATASET
/****************************************************************/
    struct selfCalib_Dataset
    {
        std::vector<selfCalib_Item> data;

        /**
        * Copy Operator
        */
        selfCalib_Dataset &operator=(const selfCalib_Dataset &v);

        /**
        * Clone Function
        */
        void clone (const selfCalib_Dataset &v) {data=v.data;};

        /**
        * Prints the state of the variable
        */
        void print(int verbosity=0);

        /**
        * Returns the number of elements in the data vector
        */
        int size();
    };

/****************************************************************/
/* selfCalib_PARAMETER_VARIABLES
/****************************************************************/
    struct selfCalib_ParVar
    {
        Vector Phi;
        double obj_value;
        string name;

        /*
        * Constructor.
        * @param dim is the number of variables to use (4*DOF)
        */
        selfCalib_ParVar(string _name, int dim);

        /**
        * Copy Operator
        */
        selfCalib_ParVar &operator=(const selfCalib_ParVar &v);

        /**
        * Clone Function
        */
        void clone (const selfCalib_ParVar &v);

        /**
        * Prints the state of the variable
        */
        void print();
    };

/****************************************************************/
/* selfCalib_SUBPROBLEM
/****************************************************************/
    struct selfCalib_SubProblem
    {
        iCubCustomLimb              limb;
        int                         nJoints;
        int                         nVars;
        selfCalib_ParVar   guess;

        std::string     getType()    { return limb.getType(); }
        iKinChainMod*   asChainMod() { return limb.asChainMod(); }
        iCubCustomLimb* getLimb()    { return &limb; }

        selfCalib_SubProblem(string _type);
        selfCalib_SubProblem & operator=(const selfCalib_SubProblem &sp);
    };

/****************************************************************/
/* selfCalib_PROBLEM
/****************************************************************/
    struct selfCalib_Problem
    {
        selfCalib_SubProblem *R2L;
        selfCalib_SubProblem *L2R;

        selfCalib_Problem(std::string _type);
    };

/****************************************************************/
/* selfCalib_SOLVER
/****************************************************************/
    class selfCalib_Solver
    {
    protected:
        selfCalib_Problem     problem;
        selfCalib_SubProblem *current_subproblem;
        string                current_subproblem_type;
        selfCalib_Dataset     dataset;

        // Type of test: it can be CC (Calibrate Chain), or
        // CF (Calibrate Finger) or PC (Perturbate Chain)
        string                test_type;     

    public:
        selfCalib_Solver(string _type, selfCalib_Dataset _dataset, string _test_type);

        bool setSubProblem (string _type);

        selfCalib_SubProblem* getSubProblem(string _type)
        {
            if (_type == "R2L" || _type == "L2R")
            {
                setSubProblem(_type);
            }
            return current_subproblem;
        };

        double solve(selfCalib_ParVar &solution);

        void setInitialGuess(const selfCalib_ParVar &g)
        {
            current_subproblem->guess=g;
        };
    };
