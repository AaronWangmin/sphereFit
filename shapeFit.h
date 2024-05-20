#ifndef SHAPE_FIT_H
#define SHAPE_FIT_H

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <Eigen/Dense>
// #include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <ceres/rotation.h>


#include "dataPrepare.h"

using namespace std;

#include "dataPrepare.h"

void putOutResultFile(const string& outFileDir, const vector<string>& resultVector);

class ShapeFit
{
public:
    void getObs() {}
    
    void setInitialParams() {} 

    void fitCompute(); 
    
    void computeAverageCenterAndRadius(Point& center,double& radius,vector<Point>& pointsVector);
    
    string getResultString() const { return this->resultString;}
    
    vector<Point>& getPointsVector() {return this->pointsVector;} 
    vector<double>& getParams() { return this->params;}
    
protected:
    
    vector<double> params;
    
    vector<Point> pointsVector;
    
    string resultString;
    
};

#endif
