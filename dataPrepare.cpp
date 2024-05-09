#include <iostream>
#include <fstream>
#include <tuple>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <cmath>

#include "dataPrepare.h"

void DataPrepare::readObsFile(const string dataFileDir)
{
    ifstream file(dataFileDir);
    if(!file.is_open())
    {
        cerr << "cant open file: " << dataFileDir << endl;        
    }
    
    double yaw = 999999;
    double pitch = 999999;
    
    string lineStr;
    getline(file,lineStr);
    while(getline(file,lineStr))
    {
        vector<string> infoList;
        boost::split(infoList,lineStr,boost::is_any_of(",")); 
        
        if(infoList[2] == "" || infoList[4] == "" || infoList[5] == "" || infoList[6] == "" || infoList[7] == "")
        {
            continue;
        }
        
        string prismName = infoList[2];
        string pointName = infoList[4];
        double N = stod(infoList[5]);
        double E = stod(infoList[6]);
        double H = stod(infoList[7]);
        
        if(infoList[0] != "")
        {  
            yaw = stod(infoList[0]); 
//             yawVector.push_back(yaw);
        }
        
        if(infoList[1] != "")
        {
           pitch = stod(infoList[1]); 
//            pitchVector.push_back(pitch);
        }
        
        Point point(pointName,prismName,N,E,H,yaw,pitch);
        this->obsVector.push_back(point);
        
    }
    
    cout << " count of obsPoints: " << obsVector.size() << endl; 
    
}

void DataPrepare::selectObsCoords(vector<Point>& selectedObsVec ,Point& circleCenter,
                             const string prismName,const double angle,const string fixTag)
{
    //     pitch is constant    
    
    if(fixTag == "V")
    {
        for(const auto&  point : obsVector)
        {
            if(point.prismName == prismName && point.pitch == angle)
            {
                
                selectedObsVec.push_back(point);            
            }
        }

        if(selectedObsVec.size() > 3)
        {
            Point circleCenterInitail;
            double radiumInitial;
            calculateCircleInitialParam(circleCenterInitail,radiumInitial,selectedObsVec);
            
            tuple<vector<Point>,Point, double> horienCircleData(
                selectedObsVec, circleCenterInitail,radiumInitial);
            
            horienCircleDataVector.push_back(horienCircleData);   
            
//             cout << prismName << " pitch: " << angle << 
//                 " count of points: " << selectedObsVec.size() <<
//                 " circleCenterInitail x: "<<  circleCenterInitail.x << " , "  <<
//                                      "y: " << circleCenterInitail.y  << " , " << 
//                                      "z: " << circleCenterInitail.z  << " , " <<
//                " radiumInitail : " << radiumInitial << endl;
            
        } 
    }
    
    if(fixTag == "H")
    {
        for(const auto&  point : obsVector)
        {
            if(point.prismName == prismName && point.yaw == angle)
            { 
                selectedObsVec.push_back(point);            
            }
        }
 
        if(selectedObsVec.size() > 3)
            {
                Point circleCenterInitail;
                double radiumInitial;
                calculateCircleInitialParam(circleCenterInitail,radiumInitial,selectedObsVec);
                
                tuple<vector<Point>,Point, double> verticalCircleData(
                    selectedObsVec, circleCenterInitail,radiumInitial);
                
                verticalCircleDataVector.push_back(verticalCircleData);
                
//                 cout << prismName << " yaw: " << angle << 
//                 " count of points: " << selectedObsVec.size() <<
//                 " circleCenterInitail x: "<<  circleCenterInitail.x << " , "  <<
//                                      "y: " << circleCenterInitail.y  << " , " << 
//                                      "z: " << circleCenterInitail.z  << " , " <<
//                " radiumInitail : " << radiumInitial << endl;
            }
    }
    
}

void DataPrepare::calculateCircleInitialParam(Point& circleCenterInitail,double& radumInitial,
                                              const vector<Point>& pointsVec)
{
    int count = pointsVec.size();
    if(count > 0)
    {
        double sumN = 0, sumE = 0, sumH = 0;
        for(const auto& point : pointsVec)
        {
            sumN += point.x;
            sumE += point.y;
            sumH += point.z;            
        }
        
        circleCenterInitail = Point(sumN / count, sumE / count, sumH / count); 
        
        double raiumInitial_tatal = 0.0;
        for(const auto& point : pointsVec)
        {
            raiumInitial_tatal += sqrt(pow(point.x - circleCenterInitail.x,2) +
                pow(point.y - circleCenterInitail.y,2) +
                pow(point.z - circleCenterInitail.z,2) );
        }
        
        radumInitial = raiumInitial_tatal / count;
        
//          radumInitial = 13;
    }
}

void DataPrepare::prepareAllData()
{
    //  circle on horizen 
    cout << "************ horizen circle *************************" << endl;
    for(const auto& prism : prismVector)
    {        
        
        vector<Point> selectedObsVec;        
        Point circleCenter;
        
        for(const auto& pitch : pitchVector)
        {
            selectObsCoords(selectedObsVec,circleCenter,prism,pitch,"V"); 
            selectedObsVec.clear();
        }
    }
    
     //  circle on horizen 
    cout << "************ Vertical circle *************************" << endl;
    for(const auto& yaw : yawVector)
    {   
        vector<Point> selectedObsVec;        
        Point circleCenter;
        
        for(const auto& prism : prismVector)
        {
            selectObsCoords(selectedObsVec,circleCenter,prism,yaw,"H"); 
            selectedObsVec.clear();
        }
    }
}

vector<tuple<vector<Point>,Point, double>>& DataPrepare::getHorienCircleDataVector()
{
  return horienCircleDataVector;  
}


// int main ( int argc,char** argv )
// {
//     DataPrepare dp;
//     
//     string dataFileDir = "/home/vboxuser/projects/ceres_hello_202404/data/20240507-ZB60.csv";
//     dp.readObsFile(dataFileDir);
//     
//     dp.prepareAllData();
    
//     vector<Point> selectedObsVec;
//     Point circleCenter;
//     dp.selectObsCoords(selectedObsVec, circleCenter,"C", 15,"H"); 

//     dp.calculateCircleCenter(circleCenter,selectedObsVec);   
    

    
//     return 0;
// 
// }
