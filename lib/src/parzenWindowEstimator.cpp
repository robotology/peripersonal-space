#include "iCub/periPersonalSpace/parzenWindowEstimator.h"

/****************************************************************/
/* PARZEN WINDOW ESTIMATOR
/****************************************************************/

double gauss(const double x_0, const double sigma, const double val)
{
    double res = (1/(sqrt(2*M_PI)*sigma))*exp(-0.5*(((val - x_0)/sigma)*((val - x_0)/sigma)));
    //           (1/(sqrt(2*pi  )*sigma))*exp(-0.5*((x(i)-x_0)/sigma)^2);
    return res;
}

parzenWindowEstimator::parzenWindowEstimator()
{
    resize(0.2,10);
}

parzenWindowEstimator::parzenWindowEstimator(const double e, const int ns)
{
    resize(e,ns);
}

bool parzenWindowEstimator::resize(const double e, const int ns)
{
    ext    =  e;
    hSize  = ns;
    sExt   = ext/hSize;

    hist.resize(hSize,0.0);
    sigm.resize(hSize,0.75*sExt);

    return true;
}

double parzenWindowEstimator::getF_X(const double x)
{
    double f_x = 0;

    for (size_t i = 0; i < hist.size(); i++)
    {
        int multiplier = hist[i]>=0?int(hist[i]):0;
        f_x += multiplier * gauss(i*sExt,sigm[i],x);
    }

    return f_x;
}

double parzenWindowEstimator::getF_X_scaled(const double x)
{
    double max   =   0;
    double sum   =   0;
    double where = ext;
    int granularity = 10;

    for (int i = 0; i < hSize*granularity; i++)
    {
        double func = getF_X(i*sExt/granularity);
        where = func>max?(i*sExt/granularity):where;
        max   = func>max?func:max;
        sum  += func;
    }
    double avg = sum/(hSize*granularity);

    // printf("max is %g at %g\t avg is %g \n",max,where,avg);
    double scalingfactor_avg = 100/avg;
    double scalingfactor_max = 255/max;

    return getF_X(x)*scalingfactor_max;
}

bool parzenWindowEstimator::addSample(const double x)
{
    if (x <= ext)
    {
        hist[int(x/sExt)] += 2; // positive samples get a bonus because they're less than the negative
        return true;
    }
    else
        return false;
}

bool parzenWindowEstimator::removeSample(const double x)
{
    if (x <= ext)
    {     
        if (hist[int(x/sExt)] > -10) // negative values cannot go under the -10 mark
        {
            //printf("I'm removing a sample\n");
            hist[int(x/sExt)] -= 1;
        }

        return true;
    }
    else
        return false;
}

void parzenWindowEstimator::print()
{
    printf("Extension: %g hSize %i sExt %g sigma %g\tHistogram:\n",ext,hSize,sExt,sigm[0]);
    printf("%s\n",hist.toString().c_str());
}

// pp = pp + (1/(2*M_PI*sigm))*exp(-0.5*(xxb -px(b)).^2/(sigm^2));
