// PURPOSE: Generate a minimum jerk trajectory between current (x,y,z) and final (x,y,z) at time stamps in t
// INPUT: x0[], xf[], time to get to final location (assumes current t=0), number of points in trajectory, preallocated 2D array traj (numPointsx3)
// OUTPUT: traj[]

#include <cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

void minJerk(const std::vector<double> *x0, const std::vector<double> *xf, int tf, int numPoints, std::vector<double> *traj) {
    std::vector <double> t;
    double dt = tf/(double)numPoints;

    //Build t vector
    for (int i=0; i<numPoints; i++) {
        t.push_back((double)i*dt);
    } 

    //For x,y,z
    for (int i=0; i<3; i++) {
        //Find Coefficents
        double a0 = x0[i];
        double a1 = 0;
        double a2 = 0;
        double a3 =-(20*x0[i] - 20*xf[i])/(2*pow(tf,3));
        double a4 =(30*x0[i] - 30*xf[i])/(2*pow(tf,4));
        double a5 =-(12*x0[i] - 12*xf[i])/(2*pow(tf,5));

        //Build Trajectory
        for (int j=0; j<numPoints; j++) {
            traj[j,i] =  a5*pow(t[j],5) + a4*pow(t[j],4) + a3*pow(t[j],3) + a2*pow(t[j],2) + a1*t[j] + a0;
        }
    }
}


int main(void) {
    int tf = 5;
    int numPoints = 100;
    
    std::vector <double> x0;
    std::vector <double> xf;
    std::vector < std::vector< double> > traj;

    x0.push_back(0);
    x0.push_back(0);
    x0.push_back(0);
    
    xf.push_back(1);
    xf.push_back(1);
    xf.push_back(1);

    minJerk(&x0,&xf,tf,numPoints,&traj);
    
    return 0;
}


