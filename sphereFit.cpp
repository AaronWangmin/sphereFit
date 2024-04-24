#include <iostream>
#include <cmath>
#include <random>

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <chrono>

using namespace std;


// sphere funtion: (x - a)^2 + (y - b)^2 + (z - c)^2 = r^2

// h = (x - a)^2 + (y - b)^2 + (z - c)^2 - r^2

// linear at a0,b0,c0,r0

// h = -2(x-a0) * delta_a - -2(y-b0) * delta_b -2(z-c0) * delta_c - 2r0 * delta_r +
//     (x-a0)^2 + (y-b0)^2 + (z-c0)^2 - r0^2



struct SphereFittingCost
{
    SphereFittingCost(const double x,const double y,const double z) : _x(x),_y(y),_z(z) {}

    template <typename T> 
    bool operator()(const T* const abcr, T* residual) const
    {
        residual[0] = 
            ceres::pow((T(_x) - abcr[0]),2) + 
            ceres::pow((T(_y) - abcr[1]),2) + 
            ceres::pow((T(_z) - abcr[2]),2) - 
            abcr[3] * abcr[3] ;            
            
        return true;
    }

    private:
        const double _x,_y,_z;
};

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

int main(int argc,char** argv)
{
    // the earth truth: use for simulate the sphere data  
    double as = 100.0, bs = 200.0, cs = 300.0, rs = 27.0;  
    
    // the count of point for simulate the sphere   
    int N = 628;
    
    // simulate the sphere data ,added the GaussNoise 
    vector<double> x_data, y_data,z_data;
    for (int i = 0; i < N; i++)
    {
        GaussNoise gauseNoise(0,1);
        double noise = gauseNoise.generateGaussNoise() / 200.0;
        
        double theta = i / 100.0;
        double phi = i / 100.0;        
        
        double x = as + rs * std::cos( theta ) * std::sin(phi) + noise;
        double y = bs + rs * std::sin( theta ) * std::sin(phi) + noise;      
        double z = cs + rs * std::cos(phi) + noise; 
        
        x_data.push_back(x);
        y_data.push_back(y);
        z_data.push_back(z);
        
    }
    
    // the iniatial value     
    double ae = 90, be = 210, ce = 280, re = 26;
    double abcr[4] = {ae,be,ce,re};
    

    ceres::Problem problem;
    for (int i = 0; i < N; i++)
    {
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<SphereFittingCost,1,4>(
                new SphereFittingCost(x_data[i],y_data[i],z_data[i]));
        problem.AddResidualBlock(costFunction,new ceres::CauchyLoss(0.5),abcr);        
        
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options,&problem,&summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 -t1);
    cout << "solve time coat = " << time_used.count() << " seconds." << endl;
    cout << summary.BriefReport() << endl;
    cout << "estimated a,b,c,r = ";
    for (auto a : abcr)
    {
         cout << a << " ";        
    } 
    cout << endl;
    
    //  covariance compute
    ceres::Covariance::Options covarianceOptions;
    ceres::Covariance covariance(covarianceOptions);
    
    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.push_back(make_pair(abcr, abcr));

    CHECK(covariance.Compute(covariance_blocks, &problem));
    
    Eigen::Matrix<double,4,4,Eigen::RowMajor> covariance_abcr = 
        Eigen::Matrix<double, 4,4,Eigen::RowMajor>::Zero();
//     double covariance_abcr[4 * 4];   

    covariance.GetCovarianceBlock(abcr, abcr, covariance_abcr.data());
    
    std::cout << endl << "covariance of abcr: " << endl;
    std::cout << covariance_abcr << endl;
    
    return 0;

}
