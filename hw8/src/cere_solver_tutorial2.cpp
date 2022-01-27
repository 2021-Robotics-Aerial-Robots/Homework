#include <ros/ros.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <queue>

// f1 = x1 + 10 * x2
struct F1 {
  template <typename T>
  bool operator()(const T* const x1, const T* const x2, T* residual) const {
    residual[0] = (x1[0] + T(10.0) * x2[0]);
    return true;
  }
};

// f2 = 5^0.5 * (x3 - x4)
struct F2 {
  template <typename T>
  bool operator()(const T* const x3, const T* const x4, T* residual) const {
    residual[0] = T(sqrt(5.0)) * (x3[0] - x4[0]);
    return true;
  }
};

// f3 = (x2 - 2 * x3)^2
struct F3 {
  template <typename T>
  bool operator()(const T* const x2, const T* const x3, T* residual) const {
    residual[0] = (x2[0] - T(2) * x3[0]) * (x2[0] - T(2) * x3[0]);
    return true;
  }
};

// f4 = 10^0.5 * (x1 - x4)^2
struct F4 {
  template <typename T>
  bool operator()(const T* const x1, const T* const x4, T* residual) const {
    residual[0] = T(sqrt(10.0)) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
    return true;
  }
};

void moreCostfunc_test(){
    double x1, x2, x3, x4;
    x1 = 1;
    x2 = 2;
    x3 = 5;
    x4 = 6;
    ceres::Problem problem;
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<F1, 1, 1, 1>(new F1), NULL, &x1, &x2);
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<F2, 1, 1, 1>(new F2), NULL, &x3, &x4);
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<F3, 1, 1, 1>(new F3), NULL, &x2, &x3);
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<F4, 1, 1, 1>(new F4), NULL, &x1, &x4);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << x1 << " " << x2 << " " << x3 << " " << x4 << std::endl;
}

void showState(double v1[4], double v2[3]){
   std::cout << "v1 : " << std::endl;
   for (int i = 0; i < 4; i++){
       std::cout << v1[i] << std::endl;
   }
   std::cout << "v2 : " << std::endl;
   for (int i = 0; i < 3; i++){
       std::cout << v2[i] << std::endl;
   }
}

void vector_test(){
    double v[2][7];
    for (int i = 0; i < 7; i++){
        v[0][i] = i + 1;
        v[1][i] = 2 * i + 2;
    }
    showState(v[0], v[0]+4);
    std::cout << std::endl;
    showState(v[1]+3, v[1]);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cere_solver_tutorial2");
    ros::NodeHandle nh;
//    moreCostfunc_test();
    vector_test();
}
