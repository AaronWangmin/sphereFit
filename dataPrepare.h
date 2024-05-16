#ifndef DATA_PREPARE_H
#define DATA_PREPARE_H

#include <string>
#include <vector>
#include <tuple>

using namespace std;

struct Point
{
    Point(){}
    
    Point(double x, double y, double z, 
          double yaw = 0.0, double pitch = 0.0 ,string pointName = "", string prismName = "")
    :x(x),y(y),z(z),yaw(yaw),pitch(pitch),pointName(pointName),prismName(prismName) {}
    
    Point(string pointName, string prismName,
          double x, double y, double z, double yaw, double pitch)
    :pointName(pointName),prismName(prismName),x(x),y(y),z(z),yaw(yaw),pitch(pitch) {}
    
    string pointName;
    string prismName;
    double x;
    double y;
    double z;
    double yaw;
    double pitch;
    
};


class DataPrepare
{
public:

void readObsFile(const string dataFileDir);
    
void selectPointsByPrismAndAngle(vector<Point>& selectedObsVec ,Point& circleCenter, 
                    const string prismName,const double angle,const string fixTag);

// void selectPointsByPrismAndPitch(vector<Point>& selectedObsVec ,Point& circleCenter, 
//                     const string prismName,const double angle,const string fixTag);

void selectPointsByPrims();

void calculateMeanCenterAndRadium(Point& circleCenterInitail,double& radumInitial,
                                 const vector<Point>& pointsVec);

void prepareAllData();

vector<Point>& getAllObsVector();
vector<tuple<vector<Point>,Point, double>>& getHorienCircleDataVector();
vector<tuple<vector<Point>,Point, double>>& getVerticalCircleDataVector();

vector< tuple<string,vector<Point>>>& getSphereDataByPrims();

private:
    vector<Point> obsVector;
    vector<tuple<vector<Point>,Point, double>> horienCircleDataVector;
    vector<tuple<vector<Point>,Point, double>> verticalCircleDataVector;
    
    vector< tuple<string,vector<Point>>> spherePointsVector;
    
public:    
    vector<string> prismVector = {"A1","B1","C1","D1","A2","B2","C2","D2"};
    vector<double> yawVector = {0,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,270,285,300,315,330,345};
    vector<double> pitchVector = {5,15,25,35,45,55,65,75,85};    
    
};
    
#endif
