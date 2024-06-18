#ifndef ENCENTRICITY_AND_INCLINATION_H
#define ENCENTRICITY_AND_INCLINATION_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "shapeFit.h"

template <typename T>
T degreeToRadians(T degrees)
{
   return degrees * (M_PI / 180.0); 
};

class EccentricityAndInclinationFit : public ShapeFit
{
public:
    
    EccentricityAndInclinationFit() {}
    
    void getObs(const vector<Point>& pointsVector);
    
    void setInitialParams(); 
    
    void fitCompute(); 
    
    void getParamsIndex(Point& point, int& index_a, int& index_b, int& index_OE, int& index_yaw, int& index_pitch); 
    
    void putOutResultFile(const string& outFileDir, const vector<string>& resultVector);
    
private:
    
    
};

// Unknown-fixed(8) : 
//                          : X_rp, Y_rp, Z_rp, 
//     eccentricity         : e
//     inclination          : alpha, belta,
//     nor-orthogonality    : gammer, 
//     OritentionCorrection : OA

// target_depended(24): 3 * m: 
//      a , b
//      eleventionCorrection: OE

// poits : 2 * n = 1820
 
// const int dimensionParams = 20; 
const int dimensionParams = 32; 
// const int dimensionParams = 1852;  

struct EccentricityCost
{
public:
    EccentricityCost(const Point& point, 
                      const int& index_a, 
                      const int& index_b, 
                      const int& index_OE,
                      const int& index_yaw,
                      const int& index_pitch
                     ) 
    : point(point), index_a(index_a), index_b(index_b), index_OE(index_OE), index_yaw(index_yaw), index_pitch(index_pitch) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {    
        T x_rp = params[0];
        T y_rp = params[1];
        T z_rp = params[2];
        
        T e = params[3];        
                
        T alpha = params[4];
        T belta = params[5];
        T gammer = params[6];
        
        T O_A = degreeToRadians(T(point.yaw));
//         T O_A = params[index_yaw];
        T A_p = params[7] + O_A;
        
        T a = params[index_a];
        T b = params[index_b];
        
        T O_E = degreeToRadians(T(point.pitch));
//         T O_E = params[index_pitch];
        T E_p = params[index_OE] + O_E;
        
        T x_target,y_target,z_target;  
        
        x_target = x_rp 
            + ceres::cos(alpha) * ceres::cos(A_p) * ceres::cos(gammer) * b 
            + ceres::cos(alpha) * ceres::cos(A_p) * ceres::sin(gammer) * a * ceres::sin(E_p)
            + ceres::cos(alpha) * ceres::sin(A_p) * e
            + ceres::cos(alpha) * ceres::sin(A_p) * a * ceres::cos(E_p)
            - ceres::sin(alpha) * ceres::sin(gammer) * b 
            + ceres::sin(alpha) * ceres::cos(gammer) * a * ceres::sin(E_p);
            
        y_target = y_rp  
            + ceres::sin(belta) * ceres::sin(alpha) * ceres::cos(A_p) * ceres::cos(gammer) * b  
            + ceres::sin(belta) * ceres::sin(alpha) * ceres::cos(A_p) * ceres::sin(gammer) * a * ceres::sin(E_p) 
            + ceres::sin(belta) * ceres::sin(alpha) * ceres::sin(A_p) * e  
            + ceres::sin(belta) * ceres::sin(alpha) * ceres::sin(A_p) * a * ceres::cos(E_p)  
            - ceres::cos(belta) * ceres::sin(A_p) * ceres::cos(gammer) * b  
            - ceres::cos(belta) * ceres::sin(A_p) * ceres::sin(gammer) * a * ceres::sin(E_p)
            + ceres::cos(belta) * ceres::cos(A_p) * e
            + ceres::cos(belta) * ceres::cos(A_p) * a * ceres::cos(E_p)
            + ceres::sin(belta) * ceres::cos(alpha) * ceres::sin(gammer) * b 
            - ceres::sin(belta) * ceres::cos(alpha) * ceres::cos(gammer) * a * ceres::sin(E_p);
            
        z_target = z_rp 
            - ceres::cos(belta) * ceres::sin(alpha) * ceres::cos(A_p) * ceres::cos(gammer) * b
            - ceres::cos(belta) * ceres::sin(alpha) * ceres::cos(A_p) * ceres::sin(gammer) * a * ceres::sin(E_p)
            - ceres::cos(belta) * ceres::sin(alpha) * ceres::sin(A_p) * e
            - ceres::cos(belta) * ceres::sin(alpha) * ceres::sin(A_p) * a * ceres::cos(E_p)
            - ceres::sin(belta) * ceres::sin(A_p) * ceres::cos(gammer) * b
            - ceres::sin(belta) * ceres::sin(A_p) * ceres::sin(gammer) * a * ceres::sin(E_p)
            + ceres::sin(belta) * ceres::cos(A_p) * e 
            + ceres::sin(belta) * ceres::cos(A_p) * a * ceres::cos(E_p)
            - ceres::cos(belta) * ceres::cos(alpha) * ceres::sin(gammer) * b
            + ceres::cos(belta) * ceres::cos(alpha) * ceres::cos(gammer) * a * ceres::sin(E_p);
        
        residual[0] = x_target - T(point.x);
        residual[1] = y_target - T(point.y);
        residual[2] = z_target - T(point.z);
                      
       return true;
    }
    
private:
        Point point;  
        int index_a;
        int index_b;
        int index_OE;
        int index_yaw;
        int index_pitch;        
};

struct SphereFittingCost
{
public:
    SphereFittingCost(const Point& point, 
                      const int& index_a, 
                      const int& index_b, 
                      const int& index_OE,
                      const int& index_yaw,
                      const int& index_pitch
                     ) 
    : point(point), index_a(index_a), index_b(index_b), index_OE(index_OE), index_yaw(index_yaw), index_pitch(index_pitch) {}
    
    template <typename T> 
    bool operator()(const T* const params, T* residual) const
    {
        T p1[3];
        p1[0] = params[index_b];
        p1[1] = params[index_a];
        p1[2] = T(0.0);
        
        T eleventionAngleAxis[3];
        T pitch = degreeToRadians(T(point.pitch)); 
        eleventionAngleAxis[0]= params[index_OE] + pitch;
        eleventionAngleAxis[1]= T(0.0);
        eleventionAngleAxis[2]= T(0.0); 
        
        T p2[3];        
        ceres::AngleAxisRotatePoint(eleventionAngleAxis, p1, p2);
        
        T p3[3];
        p3[0] = p2[0];
        p3[1] = p2[1] + params[3];
        p3[2] = p2[2];
        
        T nonOrthogonalityAngleAxis[3];
        nonOrthogonalityAngleAxis[0]= T(0.0); 
        nonOrthogonalityAngleAxis[1]= params[6];        
        nonOrthogonalityAngleAxis[2]= T(0.0); 
        
        T p4[3];
        ceres::AngleAxisRotatePoint(nonOrthogonalityAngleAxis, p3, p4);
        
        T azimuthAngleAxis[3];
        azimuthAngleAxis[0]= T(0.0); 
        azimuthAngleAxis[1]= T(0.0);   
        T yaw = degreeToRadians(T(point.yaw)); 
        azimuthAngleAxis[2]= -(params[7] + yaw);
        
        T p5[3];
        ceres::AngleAxisRotatePoint(azimuthAngleAxis, p4, p5);
        
        T inclinationAlphaAngleAxis[3];
        inclinationAlphaAngleAxis[0]= T(0.0); 
        inclinationAlphaAngleAxis[1]= params[4];
        inclinationAlphaAngleAxis[2]= T(0.0);
        
        T p6[3];
        ceres::AngleAxisRotatePoint(inclinationAlphaAngleAxis, p5, p6);
        
        T inclinationBeltaAngleAxis[3];
        inclinationBeltaAngleAxis[0]= params[5]; 
        inclinationBeltaAngleAxis[1]= T(0.0);
        inclinationBeltaAngleAxis[2]= T(0.0);
        
        T p7[3];
        ceres::AngleAxisRotatePoint(inclinationBeltaAngleAxis, p6, p7);
        
        T p8[3];
        p8[0] = p7[0] + params[0];
        p8[1] = p7[1] + params[1];
        p8[2] = p7[2] + params[2];
        
        residual[0] = p8[0] - T(point.x);
        residual[1] = p8[1] - T(point.y);
        residual[2] = p8[2] - T(point.z);
                      
       return true;
    }
    
private:
        Point point;  
        int index_a;
        int index_b;
        int index_OE;
        int index_yaw;
        int index_pitch;        
};


#endif
