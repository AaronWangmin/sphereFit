#include "shapeFit.h"

void ShapeFit::computeAverageCenterAndRadius(Point& center,double& radius,vector<Point>& pointsVector)
{

    int count = pointsVector.size();
    if(count > 0)
    {
        double sumN = 0, sumE = 0, sumH = 0;        
        for(const auto& point : pointsVector)
        {
            sumN += point.x;
            sumE += point.y;
            sumH += point.z; 
        }        

        center.x = sumN / count;
        center.y = sumE / count;
        center.z = sumH / count; 
        
        double sumRadius = 0;
        for(const auto& point : pointsVector)
        {
           double diff = std::sqrt(std::pow( (point.x - center.x),2) + 
                                        pow( (point.y - center.y),2) +
                                        pow( (point.z - center.z),2) );
           sumRadius += diff;
        }
        
        radius = sumRadius / count;
        
    }
}

void putOutResultFile(const string& outFileDir, const vector<string>& resultVector)
{
    int countFittedSucceed = 0;
    for(string s : resultVector)
    {
        if(s.size() != 0)
        {
           countFittedSucceed++; 
        }
        cout << "test...";
    }
    
    ofstream outfile(outFileDir);
    
    if(!outfile)
    {
        cerr << "cant open file for writting: " << outFileDir << endl; 
    }

    outfile << "   count of succeed-fitted:  " << countFittedSucceed << endl;
    outfile << "prism, countOfPoints, updated_x,updated_y,updated_z,updated_r, sigma,,,,,,," << endl;    
    
    for(auto& result : resultVector)
    {
        outfile << result;
    }
    
    outfile.close();
}

void angleAxisToEuro(vector<double>& rotation)
{
    Eigen::Vector3d r(rotation[0], rotation[1], rotation[2]);
    
    double angle = r.norm();
    
    r.normalize();
    
    Eigen::AngleAxisd angleAxis(angle, r);
    
    Eigen::Matrix3d rotationMatrix = angleAxis.toRotationMatrix();
    
    Eigen::Vector3d eulerAngle = rotationMatrix.eulerAngles(2,1,0);
    
    
    cout << "angleAxis: " << endl;
    for(int i= 0; i < r.size(); i++)
    {
        cout << r(i) << "," ;
    }
    
    cout << endl <<  "angle: " << angle * 180.0 / EIGEN_PI << endl;
    
    cout << endl << "euler angle(z-y-x):" << endl <<
                    "yaw: "   << eulerAngle(0) * 180.0 / EIGEN_PI << " , "
                    "pitch: " << eulerAngle(1) * 180.0 / EIGEN_PI << " , "
                    "roll: "  << eulerAngle(2) * 180.0 / EIGEN_PI << " , " << endl;
    
    
    
    
}
