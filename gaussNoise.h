#ifndef GAUSS_NOISE_H
#define GAUSS_NOISE_H

#include <iostream>

#include <cmath>
#include <random>

using namespace std;

class GaussNoise
{
public:
    GaussNoise(double mean = 0, double variance = 1) : mean(mean),variance(variance) {};
    
//     double noise = 0.0
    
    double generateGaussNoise()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        
        std::normal_distribution<double> distribute(mean,variance);
        
        return distribute(gen);
    };
    
private:
    double mean;
    double variance;
};


#endif
