#ifndef POINT_H
#define POINT_H

#include <string>

class point
{
    point(string pointName = "", double x = 0.0, double y = 0.0, double z = 0.0, 
          double yaw = 0.0, double pitch =0.0) 
    : pointName(pointName),x(x),y(y),z(z),yaw(yaw),pitch(pitch)
          
    string pointName;
    double x,
    double y,
    double z,
    
    double yaw;
    double pitch;
    
};

#endif
