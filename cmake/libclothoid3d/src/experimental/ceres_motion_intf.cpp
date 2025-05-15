#include "ceres_motion_intf.h"
#include <ceres/ceres.h>
#include <ceres/types.h>
#include <iostream>
#include <cstring>
#include "residual_clothoid_motion_fit.h"
#include "../segment.h"
#include "../gcode/fit.h"

class ceres_motion_intf {
public:
    ceres_motion_intf() {}

    ~ceres_motion_intf() {}

    void init(void* parameters, const void* user_data) {
        std::memcpy(m_parameters, parameters, sizeof(double) * 3);
        std::memcpy(m_user_data, user_data, sizeof(double) * 7);
        // std::cout << "ceres init ok." << std::endl;
    }

    // Residual functor for numeric differentiation
    struct ResidualFunctor {
        ResidualFunctor( double* user_data) : m_user_data(user_data) {}

        bool operator()(const double* const kappa, const double* const y21, const double* const s1, double* residuals) const {
            // Call your existing residual function

            // Pack the parameters into an array
            const double params[3] = {*kappa, *y21, *s1};

            // Call the residual function
            residual_motion_function(params, residuals, m_user_data);
            return true;
        }

        double* m_user_data; // Store user data (parameters)
    };

    // Find endpoint of clothoid spline.
    void solve(double *kappa, double* y21, double* s1, int solver_type, double s1_lower_limit, double s1_upper_limit) {

            // Create the Ceres problem
            ceres::Problem problem;
            ceres::CostFunction* cost_function = nullptr;

            //                 3 = dimensions.
            //                    1 = var.
            //                       1 = var.
            // ceres::CENTRAL, 3, 1, 1, 1

            if(solver_type==0){
                cost_function =
                        new ceres::NumericDiffCostFunction<ResidualFunctor, ceres::CENTRAL, 3, 1, 1, 1>(
                            new ResidualFunctor(m_user_data) // Pass the functor properly
                            );
                // std::cout << "solving with ceres central diff." << std::endl;
            }
            if(solver_type==1){
                cost_function =
                        new ceres::NumericDiffCostFunction<ResidualFunctor, ceres::FORWARD, 3, 1, 1, 1>(
                            new ResidualFunctor(m_user_data) // Pass the functor properly
                            );
                // std::cout << "solving with ceres forward diff." << std::endl;
            }
            if(solver_type==2){
                cost_function =
                        new ceres::NumericDiffCostFunction<ResidualFunctor, ceres::RIDDERS, 3, 1, 1, 1>(
                            new ResidualFunctor(m_user_data) // Pass the functor properly
                            );
                // std::cout << "solving with ceres ridders diff." << std::endl;
            }

            // Add the residual block to the problem
            problem.AddResidualBlock(cost_function, nullptr, y21, s1, kappa);

            // Set lower and upper bounds for s
            problem.SetParameterLowerBound(s1, 0, s1_lower_limit);
            problem.SetParameterUpperBound(s1, 0, s1_upper_limit);

            // Set up solver options
            ceres::Solver::Options options;
            options.minimizer_progress_to_stdout = false; // Print progress to stdout
            options.max_num_iterations = 200;           // Maximum number of iterations
            options.function_tolerance = 1e-6;          // Tolerance for cost function value

            // Solve the problem
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // Print the solver summary
            std::cout << summary.BriefReport() << "\n";
            std::cout << "Final parameters: y21 = " << *y21 << ", s1 = " << *s1 << "\n";
    }

    double* result() {
        return m_parameters;
    }

    double m_parameters[3];      // Parameters to optimize
    double m_user_data[7];       // User data passed to residual function
};

// C interface functions
extern "C" ceres_motion_intf* init_cpp_motion_ptr() {
    return new ceres_motion_intf();
}

extern "C" void ceres_cpp_motion_init(ceres_motion_intf* ptr, void* parameters, const void* user_data) {
    ptr->init(parameters, user_data);
}

extern "C" void ceres_cpp_motion_solve(ceres_motion_intf* ptr,
                                 double *kappa,
                                double* y21,
                                double* s1,
                                int solver_type,
                                double s1_lower_limit,
                                double s1_upper_limit) {
    ptr->solve(kappa, y21, s1, solver_type, s1_lower_limit, s1_upper_limit);
}

extern "C" void ceres_cpp_motion_result(ceres_motion_intf* ptr, double* result) {
    std::memcpy(result, ptr->m_parameters, sizeof(double) * 3);
}

extern "C" void ceres_cpp_motion_free_ptr(ceres_motion_intf *ptr) {
    if (ptr) {              // Check if the pointer is not null
        delete ptr;         // Free the memory
        ptr = nullptr;      // Optional: Set the pointer to null to avoid dangling pointers
    }
}
