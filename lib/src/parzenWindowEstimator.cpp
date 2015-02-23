#include "iCub/periPersonalSpace/parzenWindowEstimator.h"

double gauss(const double x_0, const double sigma, const double val)
{
    double res = (1/(sqrt(2*M_PI)*sigma))*
                 exp(-0.5*( ((val - x_0)*(val - x_0))/(sigma*sigma) ));

    //           (1/(sqrt(2*pi  )*sigma))*exp(-0.5*((x(i)-x_0)/sigma)^2);
    return res;
}

double gauss2D(const double x_0, const double y_0,
               const double sigmaX, const double sigmaY,
               const double valX, const double valY)
{
    double res = (1/(sqrt(2*M_PI*sigmaX*sigmaY)))*
                 exp(-0.5*( ((valX - x_0)*(valX - x_0))/(sigmaX*sigmaX) +
                            ((valY - y_0)*(valY - y_0))/(sigmaY*sigmaY)   ));
    return res;
}

/****************************************************************/
/* PARZEN WINDOW ESTIMATOR 1D
*****************************************************************/

    parzenWindowEstimator1D::parzenWindowEstimator1D()
    {
        std::vector<double> eX; eX.push_back(-0.1); eX.push_back(0.2);
        std::vector<int>    bN; bN.push_back(20);

        resize(eX,bN);
    }

    parzenWindowEstimator1D::parzenWindowEstimator1D(const std::vector<double> eX, const std::vector<int> bN)
    {
        resize(eX,bN);
    }

    bool parzenWindowEstimator1D::resize(const std::vector<double> eX, const std::vector<int> bN)
    {
        if (eX.size()!=2 || bN.size()!=1)
        {
            yError("Resize failed! eX size: %lu bN size: %lu\n",eX.size(), bN.size());
            return false;
        }

        extX    = eX;
        binsNum = bN;

        binWidth.clear();
        binWidth.push_back((extX[1]-extX[0])/binsNum[0]);

        // Let's find the first bin for which we have positive values.
        // The 0 value should not be inside it
        // and its shift from zero to the start value of the firstPosBin
        firstPosBin.clear();
        firstPosBinShift.clear();

        std::vector<double> binStart;
        binStart.clear();
        binStart.push_back(eX[0]);

        for (size_t i = 0; i < binStart.size(); i++)
        {
            int idx=0;
            while (binStart[i]<0)
            {
                idx++;
                binStart[i]+=binWidth[i];
            }
            firstPosBin.push_back(idx);
            firstPosBinShift.push_back(binStart[i]);
        }

        int j=0;
        for(j=0; j<binsNum[X_DIM];j++)
        {  
            binStartsX.push_back(extX[START]+(j*binWidth[X_DIM]));
        }
                      
        posHist.resize(binsNum[0],1); posHist.zero();
        negHist.resize(binsNum[0],1); negHist.zero();
       
        sigmX = binWidth[0];

        return true;
    }

    yarp::sig::Matrix parzenWindowEstimator1D::getHist()
    {
        yarp::sig::Matrix Hist(binsNum[0],1);
        Hist.zero();

        for (int i = 0; i < binsNum[0]; i++)
        {
            Hist(i,1)=getHist(i);
        }

        return Hist;
    }

    double parzenWindowEstimator1D::getHist(const int i)
    {
        if ( posHist(i,0)+negHist(i,0) < 5 )
            return 0;

        return posHist(i,0)/(posHist(i,0)+negHist(i,0));
    }


    bool parzenWindowEstimator1D::addSample(const std::vector<double> x)
    {
        int b0=-1;
        
        if (getIndexes(x,b0))
        {
            posHist(b0,0) += 1;
            return true;
        }
        else
            return false;
    }

    bool parzenWindowEstimator1D::removeSample(const std::vector<double> x)
    {
        int b0=-1;

        if (getIndexes(x,b0))
        {
            negHist(b0,1) += 1;
            return true;
        }
        else
            return false;
    }

    int parzenWindowEstimator1D::computeResponse(const std::vector<double> x)
    {
        int b0=-1;

        if (getIndexes(x,b0))
        {
            return int(getF_X_scaled(x));
        }
        else
            return 0;
    }
        
    bool parzenWindowEstimator1D::getIndexes(const std::vector<double> x, int &b0)
    {
        // yInfo("x\t%g\t%g\t",x[0],x[1]);
        if (x[0] >= extX[0] && x[0] <= extX[1])
        {
            b0 = int((x[0]-firstPosBinShift[0])/binWidth[0]+firstPosBin[0]);
            // yInfo("b0\t%i\tb1\t%i\t\n", b0, b1);
            return true;
        }
        else
        {
            return false;
        }
    }

    double parzenWindowEstimator1D::getF_X(const std::vector<double> x)
    {
        double f_x = 0;

        for (size_t i = 0; i < posHist.rows(); i++)
        {
            if ( posHist(i,0)>=0 )
            {
                double factor=(posHist(i,0)+negHist(i,0))>0?posHist(i,0)/(posHist(i,0)+negHist(i,0)):0;
                f_x += factor * gauss(extX[0]+i*binWidth[0],sigmX,x[0]);
            }
        }

        return f_x;
    }

    double parzenWindowEstimator1D::getF_X_scaled(const std::vector<double> x)
    {
        double max      =  0;
        int granularity = 10;        

        // the granularity has been introduced to increase the precision of the process.
        for (size_t i = 0; i < binsNum[0]*granularity; i++)
        {
            std::vector<double> xx;
            xx.push_back(extX[0]+i*binWidth[0]*granularity);
            double func = getF_X(xx);

            max  = func>max?func:max;
        }

        double scalingfactor_max = 255/max;

        return getF_X(x)*scalingfactor_max;
    }

    void parzenWindowEstimator1D::print()
    {
        yInfo("Extension (X1 X2): %g %g",extX[0],extX[1]);
        yInfo("binsNum(X) %i\t binWidth(X) %i",binsNum[0],binWidth[0]);
        yInfo("sigma(X) %g\n",sigmX);
        yInfo("Positive histogram:\n");
        yInfo("%s\n",posHist.toString().c_str());
        yInfo("Negative histogram:\n");
        yInfo("%s\n",negHist.toString().c_str());
    }

/****************************************************************/
/* PARZEN WINDOW ESTIMATOR 2D
*****************************************************************/

    parzenWindowEstimator2D::parzenWindowEstimator2D()
    {
        std::vector<double> eX; eX.push_back(-0.1); eX.push_back(0.2);
        std::vector<double> eY; eY.push_back(0.0);  eY.push_back(1.2);
        std::vector<int>    bN; bN.push_back(8);    bN.push_back(4);

        resize(eX,eY,bN);
    }

    parzenWindowEstimator2D::parzenWindowEstimator2D(const std::vector<double> eX, const std::vector<double> eY, const std::vector<int> bN)
    {
        resize(eX,eY,bN);
    }    

    bool parzenWindowEstimator2D::resize(const std::vector<double> eX, const std::vector<double> eY, const std::vector<int> bN)
    {
        if (eX.size()!=2 || eY.size()!=2 || bN.size()!=2)
        {
            yInfo("ERROR! Resize failed. eX size: %lu eY size: %lu bN size: %lu\n",eX.size(), eY.size(), bN.size());
            return false;
        }

        extX   = eX;
        extY   = eY;
        binsNum  = bN;

        binWidth.clear();
        binWidth.push_back((extX[1]-extX[0])/binsNum[0]);
        binWidth.push_back((extY[1]-extY[0])/binsNum[1]);

        // Let's find the first bin for which we have positive values.
        // The 0 value should not be inside it
        // and its shift from zero to the start value of the firstPosBin
        firstPosBin.clear();
        firstPosBinShift.clear();

        std::vector<double> binStart;
        binStart.clear();
        binStart.push_back(eX[0]);
        binStart.push_back(eY[0]);

        for (size_t i = 0; i < binStart.size(); i++)
        {
            int idx=0;
            while (binStart[i]<0)
            {
                idx++;
                binStart[i]+=binWidth[i];
            }
            firstPosBin.push_back(idx);
            firstPosBinShift.push_back(binStart[i]);
        }

        int j=0;
        for(j=0; j<binsNum[X_DIM];j++)
        {  
            binStartsX.push_back(extX[START]+(j*binWidth[X_DIM]));
        }

        for(j=0; j<binsNum[Y_DIM];j++)
        { 
            binStartsY.push_back(extY[START]+(j*binWidth[Y_DIM]));    
        }
                      
        posHist.resize(binsNum[0],binsNum[1]); posHist.zero();
        negHist.resize(binsNum[0],binsNum[1]); negHist.zero();
       
        sigmX = binWidth[0];
        sigmY = binWidth[1];

        return true;
    }

    yarp::sig::Matrix parzenWindowEstimator2D::getHist()
    {
        yarp::sig::Matrix Hist(binsNum[0],binsNum[1]);
        Hist.zero();

        for (int i = 0; i < binsNum[0]; i++)
        {
            for (int j = 0; j < binsNum[1]; j++)
            {
                Hist(i,j)=getHist(i,j);
            }
        }

        return Hist;
    }

    double parzenWindowEstimator2D::getHist(const int i, const int j)
    {
        if ( posHist(i,j)+negHist(i,j) < 5 )
            return 0;

        return posHist(i,j)/(posHist(i,j)+negHist(i,j));
    }

    bool parzenWindowEstimator2D::addSample(const std::vector<double> x)
    {
        int b0=-1;
        int b1=-1;
        
        if (getIndexes(x,b0,b1))
        {
            posHist(b0,b1) += 1;
            return true;
        }
        else
            return false;
    }

    bool parzenWindowEstimator2D::removeSample(const std::vector<double> x)
    {
        int b0=-1;
        int b1=-1;

        if (getIndexes(x,b0,b1))
        {
            negHist(b0,b1) += 1;
            return true;
        }
        else
            return false;
    }

    int parzenWindowEstimator2D::computeResponse(const std::vector<double> x)
    {
        int b0=-1;
        int b1=-1;

        if (getIndexes(x,b0,b1))
        {
            return int(getF_X_scaled(x));
        }
        else
            return 0;
    }
        
    bool parzenWindowEstimator2D::getIndexes(const std::vector<double> x, int &b0, int &b1)
    {
        // yInfo("x\t%g\t%g\t",x[0],x[1]);
        if (x[0] >= extX[0] && x[0] <= extX[1] && x[1] >= extY[0] && x[1] <= extY[1])
        {
            b0 = int((x[0]-firstPosBinShift[0])/binWidth[0]+firstPosBin[0]);
            b1 = int((x[1]-firstPosBinShift[1])/binWidth[1]+firstPosBin[1]);
            // yInfo("b0\t%i\tb1\t%i\t\n", b0, b1);

            return true;
        }
        else
        {
            return false;
        }
    }

    double parzenWindowEstimator2D::getF_X(const std::vector<double> x)
    {
        double f_x = 0;

        for (size_t i = 0; i < posHist.rows(); i++)
        {
            for (size_t j = 0; j < posHist.cols(); j++)
            {
                if ( posHist(i,j)>=0 )
                {
                    double factor=(posHist(i,j)+negHist(i,j))>0?posHist(i,j)/(posHist(i,j)+negHist(i,j)):0;
                    f_x += factor * gauss2D(extX[0]+i*binWidth[0],extY[0]+j*binWidth[1],sigmX,sigmY,x[0],x[1]);
                }
            }
        }

        return f_x;
    }
    
    double parzenWindowEstimator2D::getF_X_scaled(const std::vector<double> x)
    {
        double max      = 0;
        int granularity = 2;
        // double sum  =   0;
        // int cnt     = 0;

        // the granularity has been introduced to increase the precision of the process.
        for (size_t i = 0; i < binsNum[0]*granularity; i++)
        {
            for (size_t j = 0; j < binsNum[1]*granularity; j++)
            {
                std::vector<double> xx;
                xx.push_back(extX[0]+i*binWidth[0]*granularity);
                xx.push_back(extY[0]+j*binWidth[1]*granularity);
                double func = getF_X(xx);

                max  = func>max?func:max;
                // sum += func;
                // cnt++;
            }
        }
        // double avg = sum/cnt;
        // double scalingfactor_avg = 100/avg;

        double scalingfactor_max = 255/max;

        return getF_X(x)*scalingfactor_max;
    }
        
    void parzenWindowEstimator2D::print()
    {
        yInfo("Extension (X1 X2 Y1 Y2): %g %g %g %g",extX[0],extX[1],extY[0],extY[1]);
        yInfo("binsNum(X Y) %i %i\nbinWidth(X Y) %g %g",binsNum[0],binsNum[1],binWidth[0],binWidth[1]);
        yInfo("sigma(X Y) %g %g\n",sigmX,sigmY);
        yInfo("Positive histogram:\n");
        yInfo("%s\n",posHist.toString().c_str());
        yInfo("Negative histogram:\n");
        yInfo("%s\n",negHist.toString().c_str());
    }
    // pp = pp + (1/(2*M_PI*sigm))*exp(-0.5*(xxb -px(b)).^2/(sigm^2));

