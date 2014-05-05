#include "iCubSelfCalib.h"

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <iCub/iKin/iKinFwd.h>

#include <iostream>
#include <sstream>
#include <string>

using namespace iCub::iKin;

/****************************************************************/
/* UTILS FUNCTIONS
/****************************************************************/
    double string_to_double( const std::string& s )
    {
        std::istringstream i(s);
        double x;

        if (!(i >> x))
            return 0;
        return x;
    }

/****************************************************************/
/* selfCalib_SUBPROBLEM
/****************************************************************/
    selfCalib_SubProblem & selfCalib_SubProblem::operator=
                                    (const selfCalib_SubProblem &sp)
    {
        limb    = sp.limb;
        nJoints = sp.nJoints;
        nVars   = sp.nVars;
    }

    /****************************************************************/
    selfCalib_SubProblem::selfCalib_SubProblem(string _type)
            : limb(_type), guess("SubProblemGuess",limb.getDOF()*4)
    {
        nVars   = limb.getDOF()*4;
        nJoints = limb.getDOF();
    }

/****************************************************************/
/* selfCalib_PROBLEM
/****************************************************************/
    selfCalib_Problem::selfCalib_Problem(string _type)
    {   
        if (_type == "both")
        {
            R2L = new selfCalib_SubProblem("R2L");
            L2R = new selfCalib_SubProblem("L2R");
        }
        else if (_type == "R2L")
        {
            R2L = new selfCalib_SubProblem("R2L");
            L2R = NULL;
        }
        else if (_type == "L2R")
        {
            R2L = NULL;
            L2R = new selfCalib_SubProblem("L2R");
        }
        else
        {
            R2L = NULL;
            L2R = NULL;
        }
    }

/****************************************************************/
/* selfCalib_ITEM
/****************************************************************/
    selfCalib_Item::selfCalib_Item()
    {
        encsS.resize(5,0.0);     // encoders slave
        encsM.resize(7,0.0);     // encoders master
        joints.resize(12,0.0);   // joint values (slave + master)

        actualRF = eye(4);       // given ref frame (from the skin)
        actualPos.resize(3,0.0); // given position  (from the skin)

        fingerHN = eye(4);       // transform matrix of the finger
        type = "";               // type of the task

        estPos.resize(3,0.0);  // given position  (from the model)
    }

    void selfCalib_Item::clone(const selfCalib_Item &v)
    {
        encsS     = v.encsS;
        encsM     = v.encsM;
        actualRF  = v.actualRF;
        actualPos = v.actualPos;
        joints    = v.joints;
        fingerHN  = v.fingerHN;
        type      = v.type;
        estPos  = v.estPos;
    }

    void selfCalib_Item::fromString(const string &line)
    {
        /**
        * ITERATOR, ABSOLUTE TIME, YARP TIMESTAMP,
        * ROBOT, COLOR, TASKTYPE, ENCODERS SLAVE, ENCODERS MASTER, 
        * TARGET TAXEL REFERENCE FRAME, FINAL TAXEL REFERENCE FRAME,
        * INDEX HN
        */
        istringstream iss(line);

        string notused, used;
        Vector temp(4,0.0);

        for (int i = 0; i < 5; i++)
            iss >> notused;

        iss >> type;

        for (int i = 0; i < 13; i++)
        {
            iss >> used;
            encsS[i] = CTRL_DEG2RAD * string_to_double(used);
        }

        for (int i = 0; i < 13; i++)
        {
            iss >> used;
            encsM[i] = CTRL_DEG2RAD * string_to_double(used);
        }

        for (int i = 0; i < 16; i++)
            iss >> notused; 

        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                iss >> used;
                temp[i] = string_to_double(used);
            }
            actualRF.setSubrow(temp,j,0);
        }

        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                iss >> used;
                temp[i] = string_to_double(used);
            }
            fingerHN.setSubrow(temp,j,0);
        }

        computeQuantities();
    }

    void selfCalib_Item::computeQuantities()
    {
        // compute the position of the end-eff from the RF
        actualPos = actualRF.subcol(0,3,3);

        // compute the joints angles from the encoders
        joints[0] = -encsS[4];
        joints[1] = -encsS[3];
        joints[2] = -encsS[2];
        joints[3] = -encsS[1];
        joints[4] = -encsS[0];

        for (int i = 0; i < 7; i++)
            joints[5+i] = encsM[i];

        // compute the estimated position through the model
    }

    selfCalib_Item &selfCalib_Item::
                        operator=(const selfCalib_Item &v)
    {
        clone(v);
        return *this;
    }

    void selfCalib_Item::print(int verbosity)
    {
        printf("**************************\n");
        printf("encsS  [deg]  = (%s)\n",(CTRL_RAD2DEG*encsS).toString().c_str());
        printf("encsM  [deg]  = (%s)\n",(CTRL_RAD2DEG*encsM).toString().c_str());

        if (verbosity >= 1)
            printf("joints [deg]  = (%s)\n",(CTRL_RAD2DEG*joints).toString().c_str());
        
        printf("EE Pos Skin   = (%s)\n",actualPos.toString().c_str());
        printf("EE Pos Est    = (%s)\n",estPos.toString().c_str());

        if (verbosity >= 1)
            printf("fingerHN  = \n%s\n",fingerHN.toString().c_str());
    }

/****************************************************************/
/* selfCalib_DATASET
/****************************************************************/
    selfCalib_Dataset &selfCalib_Dataset::
                            operator=(const selfCalib_Dataset &v)
    {
        clone(v);
        return *this;
    }

    void selfCalib_Dataset::print(int verbosity)
    {
        printf("**************************\n");
        printf("**************************\n");
        cout << "Printing variables" << endl;
        for (int i = 0; i < data.size(); i++)
        {
            data[i].print(verbosity);
        }
        printf("**************************\n");
        printf("**************************\n");
    }

    int selfCalib_Dataset::size()
    {
        return data.size();
    }

/****************************************************************/
/* selfCalib_ParVar
/****************************************************************/
    selfCalib_ParVar::selfCalib_ParVar(string _name, int dim) :
                                        name(_name)
    {
        Phi.resize(dim,0.0);
        obj_value = -1;
    }

    selfCalib_ParVar &selfCalib_ParVar::
                            operator=(const selfCalib_ParVar &v)
    {
        clone(v);
        return *this;
    }

    void selfCalib_ParVar::clone (const selfCalib_ParVar &v)
    {
        Phi=v.Phi;
        obj_value=v.obj_value;
    }

    void selfCalib_ParVar::print()
    {
        printf("**************************\n");
        printf("%s:\n", name.c_str());
        printf("%s\n", Phi.toString().c_str());
        printf("obj_value: %g\n", obj_value);
        printf("**************************\n");
    }

/****************************************************************/
/* selfCalib_NLP
/****************************************************************/
    class selfCalib_NLP : public Ipopt::TNLP
    {
    protected:
        selfCalib_ParVar  guess;
        selfCalib_ParVar  solution;
        selfCalib_Dataset dataset;
        iKinChainMod     *chain;

        unsigned int dim;

        yarp::sig::Vector  Phi0;
        yarp::sig::Vector  Phi;

        bool   firstGo;

    public:
        /****************************************************************/
        void computeQuantities(const Ipopt::Number *x)
        {
            Vector new_Phi(dim);
            for (Ipopt::Index i=0; i<(int)dim; i++)
                new_Phi[i]=x[i];

            if (!(Phi==new_Phi) || firstGo)
            {
                firstGo=false;
                
                Phi=setVariables(new_Phi);
                // for (int i = 0; i < chain->getDOF(); i++)
                // {
                //     cout << "A    [" << i << "]: " << Phi[4*i+0] << "\t";
                //     cout << "D    [" << i << "]: " << Phi[4*i+1] << "\t";
                //     cout << "Alpha[" << i << "]: " << Phi[4*i+2] << "\t";
                //     cout << "Offs [" << i << "]: " << Phi[4*i+3] << "\n";
                // }
                /****************************************************************/
                /* Updates the estimated end-eff positions for the dataset.
                /****************************************************************/
                for (int i = 0; i < dataset.size(); i++)
                {
                    chain->setAng(dataset.data[i].joints);

                    Matrix estMat=chain->getH()*dataset.data[i].fingerHN;
                    dataset.data[i].estPos[0]=estMat(0,3);
                    dataset.data[i].estPos[1]=estMat(1,3);
                    dataset.data[i].estPos[2]=estMat(2,3);

                    // cout << "i: " << i << "\t PosEst: "
                    //      << dataset.data[i].estPos.toString() << "\t PosSkin: "
                    //      << dataset.data[i].actualPos.toString() << endl;
                }
            }
        }

        /****************************************************************/
        /* Sets the variables to the chain.
        /****************************************************************/
        Vector setVariables(const Vector &x)
        {
            Vector PhiRes(dim,0.0);
            for (int i = 0; i < chain->getDOF(); i++)
            {
                (*chain)(i).setA     (x[4*i+0]);
                (*chain)(i).setD     (x[4*i+1]);
                (*chain)(i).setAlpha (x[4*i+2]);
                (*chain)(i).setOffset(x[4*i+3]);
                PhiRes(4*i+0) = (*chain)(i).getA();
                PhiRes(4*i+1) = (*chain)(i).getD();
                PhiRes(4*i+2) = (*chain)(i).getAlpha();
                PhiRes(4*i+3) = (*chain)(i).getOffset();

                // cout << "A    [" << i << "]: " << PhiRes[4*i+0] << "\t";
                // cout << "D    [" << i << "]: " << PhiRes[4*i+1] << "\t";
                // cout << "Alpha[" << i << "]: " << PhiRes[4*i+2] << "\t";
                // cout << "Offs [" << i << "]: " << PhiRes[4*i+3] << "\n";
            }
            return PhiRes;
        }

        /****************************************************************/
        selfCalib_NLP(iKinChainMod *_chain, selfCalib_Dataset _dataset)
                    : chain(_chain),
                      guess("NLPGuess",_chain->getDOF()*4),
                      solution("NLPS",_chain->getDOF()*4)
        {
            dim=chain->getDOF()*4;

            Phi0.resize(dim,0.0);
            Phi.resize(dim,0.0);

            firstGo=true;

            dataset=_dataset;
        }

        /****************************************************************/
        virtual void setInitialGuess(selfCalib_ParVar &g)
        {
            guess=g;
            Phi0=guess.Phi;

            Ipopt::Number x[dim];
            for (Ipopt::Index i=0; i<dim; i++)
                x[i]=Phi0[i];

            computeQuantities(x);

            // Compute the starting point obj function
            setVariables(Phi0); 
            g.obj_value = 0.0;

            for (int i = 0; i < dataset.size(); i++)
            {
                g.obj_value = g.obj_value +
                        norm(dataset.data[i].actualPos-dataset.data[i].estPos);
            }
            g.obj_value /= dataset.size();
        }

        /****************************************************************/
        virtual selfCalib_ParVar getSolution() const
        {
            return solution;
        }

        /****************************************************************/
        bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                          Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
        {
            n = dim;
            m = nnz_jac_g = nnz_h_lag = 0;
            index_style=TNLP::C_STYLE;
            
            return true;
        }

        /****************************************************************/
        bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                             Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
        {
            for (Ipopt::Index i=0; i<chain->getDOF(); i++)
            {
                x_l[4*i+0]=Phi0[4*i+0]-0.1;             x_u[4*i+0]= Phi0[4*i+0]+0.1;
                x_l[4*i+1]=Phi0[4*i+1]-0.1;             x_u[4*i+1]= Phi0[4*i+1]+0.1;
                x_l[4*i+2]=Phi0[4*i+2]-10*CTRL_DEG2RAD;   x_u[4*i+2]= Phi0[4*i+2]+10*CTRL_DEG2RAD;
                x_l[4*i+3]=Phi0[4*i+3]-10*CTRL_DEG2RAD;   x_u[4*i+3]= Phi0[4*i+3]+10*CTRL_DEG2RAD;

                // for (int j = 0; j < 4; j++)
                // {
                //     cout << "x_l[" << 4*i+j << "]: " << x_l[4*i+j] << "\t";
                //     cout << "x_u[" << 4*i+j << "]: " << x_u[4*i+j] << "\n";
                // }
            }

            return true;
        }
        
        /****************************************************************/
        bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                bool init_z, Ipopt::Number *z_L,
                                Ipopt::Number *z_U, Ipopt::Index m,
                                bool init_lambda, Ipopt::Number *lambda)
        {
            for (Ipopt::Index i=0; i<n; i++)
                x[i]=Phi0[i];

            return true;
        }
        
        /****************************************************************/
        bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Number &obj_value)
        {
            computeQuantities(x);

            obj_value = 0.0;
            for (int i = 0; i < dataset.size(); i++)
            {
                obj_value = obj_value +
                    norm(dataset.data[i].actualPos-dataset.data[i].estPos);
            }
            obj_value /= dataset.size();

            return true;
        }

        
        /****************************************************************/
        bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                         Ipopt::Number *grad_f)
        {
            // computeQuantities(x);
            // yarp::sig::Matrix J_xyz = zeros(3,12);
            // yarp::sig::Matrix J1 = zeros(6,12);

            // J1   = chain->GeoJacobian();
            // submatrix(J1,J_xyz,0,2,0,11);

            // for (int j=0; j<n; j++)
            // {
            //     if (j%4==3)
            //     {
            //         grad_f[j]= 0.0;
            //         grad_f[j] = grad_f[j] - dot(J_xyz.subcol(0,j/4,3),dataset.data[j].estPos);
            //         grad_f[j] = grad_f[j] * 2/dataset.size();
            //     }
            //     else
            //         grad_f[j]=0.0;
            // }

            return true;
        }

        /****************************************************************/
        bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Number *g)
        {
            return true;
        }

        /****************************************************************/
        bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                        Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                        Ipopt::Index *jCol, Ipopt::Number *values)
        {
            return true;
        }

        /****************************************************************/
        bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Number obj_factor, Ipopt::Index m,
                    const Ipopt::Number *lambda, bool new_lambda,
                    Ipopt::Index nele_hess, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
        {
            return true;
        }

        /****************************************************************/
        bool get_scaling_parameters(Ipopt::Number &obj_scaling, bool &use_x_scaling,
                                    Ipopt::Index n, Ipopt::Number *x_scaling,
                                    bool &use_g_scaling, Ipopt::Index m,
                                    Ipopt::Number *g_scaling)
        {
            obj_scaling=10.0;

            use_x_scaling=true;
            use_g_scaling=false;

            for (int j=0; j<n; j++)
            {
                if (j%4==0 || j%4==1)
                {
                    x_scaling[j]=10.0;
                }
                else
                {
                    x_scaling[j]=1.0;
                }
            }

            for (int i=0; i<m; i++)
                g_scaling[i]=1.0;

            return true;
        }
        
        /****************************************************************/
        void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                               const Ipopt::Number *x, const Ipopt::Number *z_L,
                               const Ipopt::Number *z_U, Ipopt::Index m,
                               const Ipopt::Number *g, const Ipopt::Number *lambda,
                               Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                               Ipopt::IpoptCalculatedQuantities *ip_cq)
        {
            computeQuantities(x);
            
            for (Ipopt::Index i=0; i<dim; i++)
            {
                solution.Phi[i]=x[i];
            }
            solution.obj_value=obj_value;
        }
    };

/****************************************************************/
/* selfCalib_SOLVER
/****************************************************************/

    selfCalib_Solver::selfCalib_Solver(string _type, selfCalib_Dataset _dataset,
                    string _test_type = "Calibrate_Chain"): problem(_type)
    {
        dataset = _dataset;
        if (_type == "R2L" || _type == "L2R")
        {
            setSubProblem(_type);
        }

        if (_test_type == "Calibrate_Chain")
        {
            test_type = "CC";
        }
        else if (_test_type == "Calibrate_Finger")
        {
            test_type = "CF";
        }
        else if (_test_type == "Perturbate_Chain")
        {
            test_type = "PC";
        }

        cout << "*** iCubSelfCalibSlv - Test_type: " << test_type << endl;
    }

    bool selfCalib_Solver::setSubProblem(string _type)
    {
        if      (_type == "R2L")
        {
            current_subproblem_type = _type;
            current_subproblem = problem.R2L;
            return true;
        }
        else if (_type == "L2R")
        {
            current_subproblem_type = _type;
            current_subproblem = problem.L2R;
            return true;
        }
        current_subproblem_type = "";
        current_subproblem      = NULL;
        return false;
    };

    double selfCalib_Solver::solve(selfCalib_ParVar &solution)
    {
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",1e-08);
        app->Options()->SetNumericValue("acceptable_tol",1e-08);
        app->Options()->SetIntegerValue("max_iter",2000);
        app->Options()->SetStringValue("mu_strategy","adaptive");
        app->Options()->SetStringValue("nlp_scaling_method","user-scaling");
        // app->Options()->SetStringValue("jacobian_approximation", "exact");
        app->Options()->SetStringValue("jacobian_approximation", "finite-difference-values");
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetIntegerValue("print_level",4);
        app->Options()->SetStringValue("derivative_test","none");
        // app->Options()->SetStringValue("derivative_test","first-order");
        // app->Options()->SetStringValue("derivative_test_print_all","yes");
        // app->Options()->SetNumericValue("derivative_test_perturbation",1e-4);

        app->Initialize();
        
        Ipopt::SmartPtr<selfCalib_NLP> nlp = new selfCalib_NLP(
                                        current_subproblem->asChainMod(),
                                        dataset);

        nlp->setInitialGuess(current_subproblem->guess);

        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
        solution=nlp->getSolution();

        current_subproblem->guess.print();
        solution.print();

        if (solution.obj_value < current_subproblem->guess.obj_value)
        {
            cout << "SUCCESS! I should have improved the task." << endl;
        }
        else
        {
            cout << "FAIL! I have not improved :(" << endl;
        }

        selfCalib_ParVar differences("Differences between guess and solution",48);
        differences.obj_value=solution.obj_value - current_subproblem->guess.obj_value;
        differences.Phi=solution.Phi - current_subproblem->guess.Phi;

        differences.print();

        return status;
    }

// empty line to make gcc happy