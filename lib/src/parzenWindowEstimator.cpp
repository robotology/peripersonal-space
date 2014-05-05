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

    parzenWindowEstimator::parzenWindowEstimator()
    {
        resize(0.2,10);
    }

    parzenWindowEstimator::parzenWindowEstimator(const double e, const int hS)
    {
        resize(e,hS);
    }

    bool parzenWindowEstimator::resize(const double e, const int hS)
    {
        ext    =  e;
        binsNum  = hS;
        binWidth   = ext/binsNum;

        hist.resize(binsNum,0.0);
        sigm.resize(binsNum,0.75*binWidth);

        return true;
    }

    bool parzenWindowEstimator::addSample(const double x)
    {
        if (x <= ext)
        {
            hist[int(x/binWidth)] += 2; // positive samples get a bonus because they're less than the negative
            return true;
        }
        else
            return false;
    }

    bool parzenWindowEstimator::removeSample(const double x)
    {
        if (x <= ext)
        {     
            if (hist[int(x/binWidth)] > -10) // negative values cannot go under the -10 mark
            {
                //printf("I'm removing a sample\n");
                hist[int(x/binWidth)] -= 1;
            }

            return true;
        }
        else
            return false;
    }

    double parzenWindowEstimator::getF_X(const double x)
    {
        double f_x = 0;

        for (size_t i = 0; i < hist.size(); i++)
        {
            int multiplier = hist[i]>=0?int(hist[i]):0;
            f_x += multiplier * gauss(i*binWidth,sigm[i],x);
        }

        return f_x;
    }

    double parzenWindowEstimator::getF_X_scaled(const double x)
    {
        double max   =   0;
        // double sum   =   0;
        double where = ext;
        int granularity = 10;

        for (size_t i = 0; i < binsNum*granularity; i++)
        {
            double func = getF_X(i*binWidth/granularity);
            where = func>max?(i*binWidth/granularity):where;
            max   = func>max?func:max;
            // sum  += func;
        }
        // printf("max is %g at %g\t avg is %g \n",max,where,avg);
        // double avg = sum/(binsNum*granularity);
        // double scalingfactor_avg = 100/avg;
        double scalingfactor_max = 255/max;

        return getF_X(x)*scalingfactor_max;
    }

    void parzenWindowEstimator::print()
    {
        printf("Extension: %g binsNum %i binWidth %g sigma %g\tHistogram:\n",ext,binsNum,binWidth,sigm[0]);
        printf("%s\n",hist.toString().c_str());
    }

/****************************************************************/
/* PARZEN WINDOW ESTIMATOR 2D
*****************************************************************/

    parzenWindowEstimator2D::parzenWindowEstimator2D()
    {
        std::vector<double> eX; eX.push_back(-0.1); eX.push_back(0.2);
        std::vector<double> eY;  eY.push_back(0.0); eY.push_back(1.2);
        // std::vector<int>    hS;    hS.push_back(8);   hS.push_back(8);

        // Let's reduce the resolution in TTC domain
        std::vector<int>    hS;    hS.push_back(8);   hS.push_back(4);

        resize(eX,eY,hS);
    }

    parzenWindowEstimator2D::parzenWindowEstimator2D(const std::vector<double> eX, const std::vector<double> eY, const std::vector<int> hS)
    {
        resize(eX,eY,hS);
    }    

    bool parzenWindowEstimator2D::resize(const std::vector<double> eX, const std::vector<double> eY, const std::vector<int> hS)
    {
        if (eX.size()!=2 || eY.size()!=2 || hS.size()!=2)
        {
            printf("ERROR! Resize failed. eX size: %lu eY size: %lu hS size: %lu\n",eX.size(), eY.size(), hS.size());
            return false;
        }

        extX   = eX;
        extY   = eY;
        binsNum  = hS;

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
            // printf("idx: %g\n", firstPosBinShift[i]);
        }

        int j=0;
        //printf("Creating bins:\n");
        for(j=0; j<binsNum[X_DIM];j++)
        {  
            binStartsX.push_back(extX[START]+(j*binWidth[X_DIM]));
          //   printf("<%f %f %f> ",binStartsX[j],binStartsX[j]+0.5*binWidth[X_DIM],binStartsX[j]+binWidth[X_DIM]);
        }
        //printf("\n");
        for(j=0; j<binsNum[Y_DIM];j++)
        { 
            binStartsY.push_back(extY[START]+(j*binWidth[Y_DIM]));    
          //   printf("^\n%f\n\n%f\n\n%f\nv\n\n",binStartsY[j],binStartsY[j]+0.5*binWidth[Y_DIM],binStartsY[j]+binWidth[Y_DIM]);
        }
                      
        posHist.resize(binsNum[0],binsNum[1]); posHist.zero();
        negHist.resize(binsNum[0],binsNum[1]); negHist.zero();
       
        sigmX = 0.75*binWidth[0];
        sigmY = 0.75*binWidth[1];

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
            // return int(255.0* getF_X_interpolated(x));
        }
        else
            return 0;
    }
        
    bool parzenWindowEstimator2D::getIndexes(const std::vector<double> x, int &b0, int &b1)
    {
        printf("x\t%g\t%g\t",x[0],x[1]);
        if (x[0] >= extX[0] && x[0] <= extX[1] && x[1] >= extY[0] && x[1] <= extY[1])
        {
            b0 = int((x[0]-firstPosBinShift[0])/binWidth[0]+firstPosBin[0]);
            b1 = int((x[1]-firstPosBinShift[1])/binWidth[1]+firstPosBin[1]);
            printf("b0\t%i\tb1\t%i\t\n", b0, b1);

            return true;
        }
        else
        {
            printf("\n");
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

    double parzenWindowEstimator2D::getF_X_interpolated(const std::vector<double> x)
    {
        // See http://en.wikipedia.org/wiki/Bilinear_interpolation
        int indexX=-1;
        int indexY=-1;
        int q11_indexX = -1; //bottom-left bin from our target
        int q11_indexY = -1;
        int q12_indexX = -1; //top-left bin from our target
        int q12_indexY = -1;
        int q21_indexX = -1; //bottom-right bin from our target 
        int q21_indexY = -1;
        int q22_indexX = -1; //top-right bin from our target
        int q22_indexY = -1;
        double fR1 = 0.0; // value at (x, y1)
        double fR2 = 0.0;  //value at (x,y2)
        double f_x_interpolated = 0.0;
        
        double halfBinWidthX = 0.5*binWidth[X_DIM];
        double halfBinWidthY = 0.5*binWidth[Y_DIM];
        
        getIndexes(x,indexX,indexY);
        double binCenterX = binStartsX[indexX]+halfBinWidthX;
        double binCenterY = binStartsY[indexY]+halfBinWidthY;
           
        if(x[0]<binCenterX)
        {
            q11_indexX = indexX-1;   
            q12_indexX = indexX-1;
            q21_indexX = indexX;   
            q22_indexX = indexX;
        }
        else
        { //x[0]>=binCenterX
            q11_indexX = indexX;   
            q12_indexX = indexX;
            q21_indexX = indexX+1;   
            q22_indexX = indexX+1;
        }

        if(x[1]<binCenterY)
        {
            q11_indexY = indexY-1;   
            q12_indexY = indexY;
            q21_indexY = indexY-1;   
            q22_indexY = indexY;
        }
        else
        { //x[1]>=binCenterY
            q11_indexY = indexY;   
            q12_indexY = indexY+1;
            q21_indexY = indexY;   
            q22_indexY = indexY+1;
        }

        if(((indexX==0) || (indexX==(binsNum[X_DIM]-1)) || (indexY==0) || (indexY==(binsNum[Y_DIM]-1))) )
        {   //bin is on the border
            //if we are out of range - we were at the border - we simply set it to the last bin - in that dimension, there won't be any interpolation       
            if (q11_indexX<0) q11_indexX=0;
            if (q12_indexX<0) q12_indexX=0; //N.B. for q21 and q22, <0 cannot happen, they will be checked on the other side
            if (q21_indexX>(binsNum[X_DIM]-1)) q21_indexX = binsNum[X_DIM]-1;
            if (q22_indexX>(binsNum[X_DIM]-1)) q22_indexX = binsNum[X_DIM]-1;
            if (q11_indexY<0) q11_indexY=0;
            if (q21_indexY<0) q21_indexY=0;
            if (q12_indexY>(binsNum[Y_DIM]-1)) q12_indexY = binsNum[Y_DIM]-1;
            if (q22_indexY>(binsNum[Y_DIM]-1)) q22_indexY = binsNum[Y_DIM]-1;
        }

        if (q11_indexX != q21_indexX)
        { // we perform interpolation in x direction, see http://en.wikipedia.org/wiki/Bilinear_interpolation
            fR1 = (binStartsX[q21_indexX]+halfBinWidthX - x[0]) / binWidth[X_DIM]  * getHist(q11_indexX,q11_indexY) + 
                  (x[0] - (binStartsX[q11_indexX]+halfBinWidthX)) / binWidth[X_DIM]  * getHist(q21_indexX,q21_indexY);
              // N.B. for binStartsX[q21_indexX] - binStartsX[q11_indexX] the +  halfBinWidthX would cancel out
            fR2 = (binStartsX[q21_indexX]+halfBinWidthX - x[0]) / binWidth[X_DIM]  * getHist(q12_indexX,q12_indexY) + 
                  (x[0] - (binStartsX[q11_indexX]+halfBinWidthX)) / binWidth[X_DIM]  * getHist(q22_indexX,q22_indexY);
        }
        else
        {
            fR1 = getHist(q11_indexX,q11_indexY);  
            fR2 = getHist(q12_indexX,q12_indexY);  
        }
        //let's continue in the y direction
        if (q11_indexY != q12_indexY)\
        {
            f_x_interpolated = (binStartsY[q12_indexY]+halfBinWidthY - x[1]) / binWidth[Y_DIM]  * fR1 + 
                               (x[1] - (binStartsY[q11_indexY]+halfBinWidthY)) / binWidth[Y_DIM]  * fR2;
        }
        else
        {
            f_x_interpolated = fR1;   
        }
        //debug
        printf("Interpolation - looking for value at %f,%f which is bin with indices %d,%d.\n",x[0],x[1],indexX,indexY);
        printf("    interpolating from these bins - (indexX,indexY): value\n");
        printf( "Q11 (%d,%d): %f\n",q11_indexX,q11_indexY,getHist(q11_indexX,q11_indexY));
        printf( "Q21 (%d,%d): %f\n",q21_indexX,q21_indexY,getHist(q21_indexX,q21_indexY));
        printf( "Q12 (%d,%d): %f\n",q12_indexX,q12_indexY,getHist(q12_indexX,q12_indexY));
        printf( "Q22 (%d,%d): %f\n",q22_indexX,q22_indexY,getHist(q22_indexX,q22_indexY));
        printf("Final interpolated value is: %f\n",f_x_interpolated);
        return f_x_interpolated; 
        
    }
    
    double parzenWindowEstimator2D::getF_X_scaled(const std::vector<double> x)
    {
        int b0=-1;
        int b1=-1;
        if (!getIndexes(x,b0,b1))
            return 0;

        double max  =   0;
        // double sum  =   0;
        // int granularity = 2;
        // int cnt     = 0;

        // the granularity has been introduced to increase the precision of the process.
        for (size_t i = 0; i < binsNum[0]/**granularity*/; i++)
        {
            for (size_t j = 0; j < binsNum[1]/**granularity*/; j++)
            {
                std::vector<double> xx;
                xx.push_back(extX[0]+i*binWidth[0]/*/granularity*/);
                xx.push_back(extY[0]+j*binWidth[1]/*/granularity*/);
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
        printf("Extension (X1 X2 Y1 Y2): %g %g %g %g",extX[0],extX[1],extY[0],extY[1]);
        printf("binsNum(X Y) %i %i\nbinWidth(X Y) %g %g",binsNum[0],binsNum[1],binWidth[0],binWidth[1]);
        printf("sigma(X Y) %g %g\n",sigmX,sigmY);
        printf("Positive histogram:\n");
        printf("%s\n",posHist.toString().c_str());
        printf("Negative histogram:\n");
        printf("%s\n",negHist.toString().c_str());
    }
    // pp = pp + (1/(2*M_PI*sigm))*exp(-0.5*(xxb -px(b)).^2/(sigm^2));

