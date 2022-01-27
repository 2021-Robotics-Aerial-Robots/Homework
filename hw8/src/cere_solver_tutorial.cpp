#include <ros/ros.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <queue>

/*
自動微分：對於AutoDiffCostrFunction類型的costFunction,我們建構一個結構體,重寫template operator()
數值微分：對於NumericDiffCostFunction類型的costFunction,將結構體的template改成double
解析微分：不使用AutoDiffCostrFunction,需自行計算cost function和Jacobian
cf: https://blog.csdn.net/weixin_41394379/article/details/90084314
cf: https://blog.csdn.net/weixin_41394379/article/details/90084314
*/
/*
Problem::AddResidualBlock(CostFunction, LossFunction, Parameters)
Problem::AddParameterBlock,額外檢查參數是否正確
*/
using namespace std;

// addResidualBlock have most 10 param(x,y,z....)
// ==== first cere-solver test ====
struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     //resdiual = (10-x) find mini number
     residual[0] = T(10.0) - x[0];
     return true;
   }
};

struct CostFunctorMulti {
   template <typename T>
    //const T* const x is template grammar
    bool operator()(const T* const x, const T* const y, T* residual) const {
     //costfunction (10-x)
     //costfunction (5-y)
     residual[0] = T(10.0) - x[0];
     residual[1] = T(5) - y[0];
     return true;
   }
};

// ==== second cere-solver test ====
struct NumericDiffCostFunctor {
  bool operator()(const double* const x, double* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

// ==== third cere-solver test ====
//
class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1> {
 public:
//  virtual ~QuadraticCostFunction() {}
  //parameters 做為輸入
  //residuals 做為輸出
  //jacobians 做為輸出
  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    const double x = parameters[0][0];
    residuals[0] = 10 - x;

    // Compute the Jacobian if asked for.
    if (jacobians != NULL && jacobians[0] != NULL) {
      jacobians[0][0] = -1;
    }
    return true;
  }
};

// ==== first cere-solver test ====
void AutoDiff_test(){
    cout << "=== AutoDiff test ===" << endl;
    // The variable to solve for with its initial value.
    double initial_x = 5.0;
    double x = initial_x;

    // Build the problem.
    ceres::Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    // 1, 1, reprensent as 1 resdual 1 state
    ceres::CostFunction* cost_function =
    new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);

    //problem.AddResidualBlock(cost_function, NULL, &x);
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    //loss_function detect error beteween real and predection
    //底下這個寫法跟cres_solver_tutorial2把各行加到problem.AddResidualBlock一樣
    problem.AddResidualBlock(cost_function, loss_function, &x);

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    // summary records the detail during iteration
    ceres::Solve(options, &problem, &summary);
    //summary.BriefReport show all records
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
}

void AutoDiffWithMultiState(){
    cout << "=== AutoDiffWithMultiState test ===" << endl;
    // The variable to solve for with its initial value.
    double initial_x = 10;
    double initial_y = 999999;
    double x = initial_x;
    double y = initial_y;
    // Build the problem.
    ceres::Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    // 1, 1, reprensent as 1 resdual 1 state
    ceres::CostFunction* cost_function =
    new ceres::AutoDiffCostFunction<CostFunctorMulti, 2, 1, 1>(new CostFunctorMulti);

    //problem.AddResidualBlock(cost_function, NULL, &x);
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    problem.AddResidualBlock(cost_function, loss_function, &x, &y);

    // Run the solver!
//    ceres::Solver::Options options;
//    options.linear_solver_type = ceres::DENSE_QR;
//    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
//    options.max_num_iterations = NUM_ITERATIONS;
    options.max_num_iterations = 4;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;

    // summary records the detail during iteration
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "cost function : \n10 - x = residual\n5 - y = residual" << std::endl;
    std::cout << "x from " << initial_x
              << " -> " << x << "\n";
    std::cout << "y from " << initial_y
              << " -> " << y << "\n";
}


// ==== second cere-solver test ====
void NumericDiff_test(){
    cout << "=== NumericDiff test ===" << endl;
    double initial_x = 5.0;
    double x = initial_x;
    ceres::Problem problem;
    ceres::CostFunction* cost_function =
    new ceres::NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>(new NumericDiffCostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
}

// ==== third cere-solver test ====
void sizeCost_test(){
    cout << "=== sizeCost test ===" << endl;
    double initial_x = 5.0;
    double x = initial_x;
    ceres::Problem problem;
    QuadraticCostFunction* cost_function = new QuadraticCostFunction();
    problem.AddResidualBlock(cost_function, NULL, &x);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";

}

void queue_test(){
    // ===== queue test =====
    // http://larry850806.github.io/2016/06/06/STL1/#queue

    // only operate front and end
    queue<int> q;   // 一個空的 queue
    q.push(10);
    q.push(20);
    q.push(30);     // [10, 20, 30]

    cout << "queue front : " << q.front() << endl; // 10
    cout << "queue back : " << q.back() << endl; // 30
    // eliminate the first place
    q.pop(); // [20, 30]
    cout << "queue pop size : " << q.size() << endl;   // 2
    // ===== queue test =====
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cere_solver_tutorial");
  ros::NodeHandle nh;

//  AutoDiff_test();
  AutoDiffWithMultiState();
//  NumericDiff_test();
//  sizeCost_test();
  ros::spin();
}
