#include <cstdlib>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string>
#include <vector>
#include <yarp/os/Random.h>
#include <sys/time.h>
#include <cmath>

#include "iCubSelfCalib.h"

#define BATCH_NUMBER 2

using namespace std;

double absMaxVector(Vector v)
{
    double max=abs(v[0]);
    
    for(int i=1; i<v.length(); i++)
    {
        if(max<abs(v[i]))
        {
            max = abs(v[i]);
        }
    }

    return max;
}

void perturbateDH (selfCalib_ParVar &in, int noise)
{
    yarp::os::Random Noise;
    double perturb = 0;
    Matrix ParamsOrdered(12,4);
    ParamsOrdered.zero();

    for (int i = 0; i < 12; i++)
    {
        ParamsOrdered.setSubrow(in.Phi.subVector(i*4,i*4+3), i, 0);
    }

    double halfMaxA  = absMaxVector(ParamsOrdered.getCol(0))/4;
    double halfMaxD  = absMaxVector(ParamsOrdered.getCol(1))/4;
    double halfMaxAl = absMaxVector(ParamsOrdered.getCol(2))/4;
    double halfMaxOf = absMaxVector(ParamsOrdered.getCol(3))/4;

    struct timeval start;
    long mtime, secs, usecs;
    gettimeofday(&start, NULL);
    usecs = start.tv_usec;
    Noise.seed(usecs);
    double un=0;

    for (int i = 0; i < in.Phi.size(); i++)
    {   
        // if (i%4==1)
        // {
        //     un=Noise.uniform()*2-1;
        //     perturb = 0.01*noise*un*halfMaxA;
        // }
        // else if (i%4==2)
        // {
        //     un=Noise.uniform()*2-1;
        //     perturb = 0.01*noise*un*halfMaxD;
        // }
        // else if (i%4==3)
        // {
        //     un=Noise.uniform()*2-1;
        //     perturb = 0.01*noise*un*halfMaxAl;
        // }
        // else if (i%4==0)
        // {
        //     un=Noise.uniform()*2-1;
        //     perturb = 0.01*noise*un*halfMaxOf;
        // }
        // cout << "\tp: " << un;
        // in.Phi[i] = perturb+in.Phi[i];
        un=Noise.uniform()*2-1;
        in.Phi[i] = 0.01*noise*un*in.Phi[i]+in.Phi[i];
    }
}

std::vector<std::string> open(std::string path = ".")
{
    DIR*    dir;
    dirent* entry;
    std::vector<std::string> files;
    
    int pos=0;

    dir = opendir(path.c_str());

    while (entry = readdir(dir)) {
    	pos = strlen(entry->d_name)-15;
        
        if (! strcmp(&entry->d_name[pos], "calibration.txt"))
        {
            files.push_back(entry->d_name);
        }
    }
    
    return files;
}

string convertInt(int number)
{
    if (number == 0)
        return "0";
    string temp="";
    string returnvalue="";
    while (number>0)
    {
        temp+=number%10+48;
        number/=10;
    }
    for (int i=0;i<temp.length();i++)
        returnvalue+=temp[temp.length()-i-1];
    return returnvalue;
}

int main(int argc,char *argv[])
{
    // PROCESS INPUT PARAMETERS
        // Type of test: it can be Calibrate Chain, 
        // Calibrate Finger Perturbate Chain
        string test_type = "Calibrate_Chain";

        // The amount of noise of the
        // Perturbate Chain test (10 by default)
        int noise = 10;

        if (argc >= 2)
        {
            if (argv[1][0] == 'h' && argv[1][1] == 'e' &&
                argv[1][2] == 'l' && argv[1][3] == 'p')
            {   
                cout << "\n*** iCubSelfCalibration - HELP ***\n\n" <<
                "    This module will achieve self calibration.\n" <<
                "    It accepts the following parameters:" << endl << 
                "        Calibrate_Chain  (CC)\n" << 
                "        Calibrate_Finger (CF)\n" << 
                "        Perturbate_Chain (PC)\n\n";
                return 0;
            }
            else
            {
                string input_type(argv[1]);
                if (   input_type=="Calibrate_Chain"
                    || input_type=="Calibrate_Finger"
                    || input_type=="Perturbate_Chain")
                    test_type = input_type;
                else if (input_type=="CC")
                {
                    test_type = "Calibrate_Chain";
                }
                else if (input_type=="CF")
                {
                    test_type = "Calibrate_Finger";
                }
                else if (input_type=="PC")
                {
                    test_type = "Perturbate_Chain";
                }
                else
                {
                    cout << "\n*** iCubSelfCalibration - ERROR\n"
                         << "I got an input parameter but it's wrong! "
                         << "Using the default one (i.e. " << test_type << ").\n";
                }
            }
        }

        cout << "\n*** iCubSelfCalibration - Test Type: " << test_type << endl;
        if (test_type=="Perturbate_Chain")
        {
            if (argc >= 3)
                noise=atoi(argv[2]);
            cout << "*** iCubSelfCalibration - Noise: " << noise << endl;
        }

    // INITIALIZE VARIABLES
        std::vector<std::string> f;
        string folder = "../data/batch_#"+convertInt(BATCH_NUMBER)+"/";
        selfCalib_Dataset dataMat;
        selfCalib_Item var;
    	std::fstream file;
    	string line;

        selfCalib_ParVar guess("ProblemGuess",48);
            guess.Phi[0]=       0.0;   guess.Phi[2]=  -M_PI/2.0;
            guess.Phi[1]=   -0.1373;   guess.Phi[3]=   M_PI/2.0;

            guess.Phi[4]=     0.015;   guess.Phi[6]=  -M_PI/2.0;
            guess.Phi[5]=       0.0;   guess.Phi[7]=        0.0;

            guess.Phi[8] =   -0.015;   guess.Phi[10]=           M_PI/2.0;
            guess.Phi[9] = -0.15228;   guess.Phi[11]= -75.0*CTRL_DEG2RAD;

            guess.Phi[12]=      0.0;   guess.Phi[14]= -M_PI/2.0;
            guess.Phi[13]=      0.0;   guess.Phi[15]=  M_PI/2.0;

            guess.Phi[16]=      0.0;   guess.Phi[18]=  M_PI/2.0;
            guess.Phi[17]= -0.10774;   guess.Phi[19]= -M_PI/2.0;

            guess.Phi[20]=      0.0;   guess.Phi[22]=  M_PI/2.0;
            guess.Phi[21]= -0.10774;   guess.Phi[23]= -M_PI/2.0;

            guess.Phi[24]=      0.0;   guess.Phi[26]= -M_PI/2.0;
            guess.Phi[25]=      0.0;   guess.Phi[27]= -M_PI/2.0;

            guess.Phi[28]=   -0.015;   guess.Phi[30]=           -M_PI/2.0;
            guess.Phi[29]= -0.15228;   guess.Phi[31]= -105.0*CTRL_DEG2RAD;

            guess.Phi[32]=    0.015;   guess.Phi[34]=  M_PI/2.0;
            guess.Phi[33]=      0.0;   guess.Phi[35]=       0.0;

            guess.Phi[36]=      0.0;   guess.Phi[38]=  M_PI/2.0;
            guess.Phi[37]=  -0.1373;   guess.Phi[39]= -M_PI/2.0;

            guess.Phi[40]=      0.0;   guess.Phi[42]=  M_PI/2.0;
            guess.Phi[41]=      0.0;   guess.Phi[43]=  M_PI/2.0;

            guess.Phi[44]=   0.0625;   guess.Phi[46]=       0.0;
            guess.Phi[45]=    0.016;   guess.Phi[47]=      M_PI;

            
            selfCalib_ParVar  delta("Delta between guess and solution (solution - guess)",48);

    	// Test
            // line = "2	1378126263541	1378126263.542539	icub	white	R2L	-60.560440	 21.901099	 26.950971	 64.450549	-75.501012	 0.000204	 0.000327	 24.000000	 9.923092	 87.913333	 10.963627	 13.106693	 6.906068	-41.857143	 23.241758	 75.103954	 80.153846	 48.234254	-53.946814	 1.931545	 18.749986	 10.004070	 89.520476	 55.467458	-1.077209	 1.098966	-0.987242	-0.099239	 0.124520	-0.030514	 0.100017	-0.994986	 0.000000	-0.073056	 0.123895	 0.012454	 0.992217	 0.008972	 0.000000	 0.000000	 0.000000	 1.000000	-0.631515	-0.369699	 0.681551	-0.024000	 0.505212	-0.862995	 0.000000	-0.043000	 0.588175	 0.344328	 0.731771	 0.021000	 0.000000	 0.000000	 0.000000	 1.000000	 0.942724	 0.333571	 0.001786	 0.077402	-0.332465	 0.939138	 0.086526	-0.051738	 0.027185	-0.082164	 0.996248	-0.008660	 0.000000	 0.000000	 0.000000	 1.000000";
            // var.fromString(line);
            // cout << line << endl;
            // var.print();

        selfCalib_ParVar guess_noisy(guess);
        guess_noisy.name = "ProblemGuess_Noisy";
        if (test_type=="Perturbate_Chain")
        {   
            perturbateDH(guess_noisy,noise);
        }

    // OPEN THE FILES & CREATE THE DATABASE
        f = open(folder);

        for (int i = 0; i < f.size(); i++)
        {
            file.open((folder+f[i]).c_str());
        	if (file.is_open())
        	{
        		while (std::getline(file,line))
        		{
        			var.fromString(line);
        			dataMat.data.push_back(var);
        		}
        	}
            file.close();
        }

    // TEST THE CALIBRATE CHAIN CASE
    if (test_type == "Calibrate_Chain")
    {
        // SOLVE THE OPTIMIZATION
        // selfCalib_Solver *slv = new selfCalib_Solver("R2L", dataMat, test_type);
        selfCalib_ParVar  solution("ProblemSolution",48);
        // slv->setInitialGuess(guess);
        // cout << "IPOPT problem status: " << slv->solve(solution) << endl;

        // SAVE TO FILE
        string finalfile="../matlab/data/batch_#"+convertInt(BATCH_NUMBER)+".txt";
        
        ofstream outputfile;
        outputfile.open(finalfile.c_str());
        outputfile  << "g:\t" << guess.Phi.toString() << endl
                    << "s:\t" << solution.Phi.toString();
        outputfile.close();

        // delete slv;
        // slv = 0;
    }
    // TEST THE PERTURBATE CHAIN CASE
    if (test_type == "Perturbate_Chain")
    {
        // SOLVE THE OPTIMIZATION
        selfCalib_Solver *slv = new selfCalib_Solver("R2L", dataMat, test_type);
        selfCalib_ParVar  solution("ProblemSolution",48);
        slv->setInitialGuess(guess_noisy);
        cout << "IPOPT problem status: " << slv->solve(solution) << endl;
        guess.print();

        string finalfile="../matlab/data/batch_#"+convertInt(BATCH_NUMBER)+
                         "_PC_"+convertInt(noise)+".txt";
        ofstream outputfile;
        outputfile.open(finalfile.c_str());
        outputfile  << "g:\t" << guess.Phi.toString()    << endl
                    << "s:\t" << solution.Phi.toString() << endl
                    << "n:\t" << guess_noisy.Phi.toString();
        outputfile.close();
    }

    return 0;
}