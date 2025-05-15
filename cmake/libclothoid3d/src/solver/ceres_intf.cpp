#include "ceres_intf.h"
#include <ceres/ceres.h>
#include <ceres/types.h>
#include <iostream>
#include <cstring>
#include "residual_clothoid_fit.h"
#include "residual_fillet_fit.h"
#include "../segment.h"
#include "../gcode/fit.h"

class ceres_intf {
public:
    ceres_intf() {}

    ~ceres_intf() {}

    void init(void* parameters, const void* user_data) {
        std::memcpy(m_parameters, parameters, sizeof(double) * 3);
        std::memcpy(m_user_data, user_data, sizeof(double) * 18);
        // std::cout << "ceres init ok." << std::endl;
    }

    // Residual functor for numeric differentiation
    struct ResidualFunctor {
        ResidualFunctor( double* user_data) : m_user_data(user_data) {}

        bool operator()(const double* const y11, const double* const y21, const double* const s1, double* residuals) const {
            // Call your existing residual function

            // Pack the parameters into an array
            const double params[3] = {*y11, *y21, *s1};

            // Call the residual function
            residual_function(params, residuals, m_user_data);
            return true;
        }

        double* m_user_data; // Store user data (parameters)
    };

    // Find endpoint of clothoid spline.
    void solve(double* y11, double* y21, double* s1, int solver_type, double s1_lower_limit, double s1_upper_limit) {

      //  bool has_self_intersection = true;

      //  while (has_self_intersection) {
            // Create the Ceres problem
            ceres::Problem problem;
            ceres::CostFunction* cost_function = nullptr;

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
            problem.AddResidualBlock(cost_function, nullptr, y11, y21, s1);

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
            // std::cout << summary.BriefReport() << "\n";
            // std::cout << "Final parameters: y11 = " << *y11 << ", y21 = " << *y21 << ", s1 = " << *s1 << "\n";
    }

    double* result() {
        return m_parameters;
    }

    double m_parameters[3];      // Parameters to optimize
    double m_user_data[18];      // User data passed to residual function

    struct ResidualFunctorAlgorithm1 {
        ResidualFunctorAlgorithm1(const segment data, const double *Ptraj){
            m_data = data;
            for (int i = 0; i < 3; ++i) {
                m_Ps[i] = Ptraj[i];
            }
        }

        bool operator()(const double* const s, double* residual) const {
            // Interpolate the clothoid spline at parameter s
            double Pi_interpolated[3]={0,0,0};
            eq14_interpolate_clothoid(&m_data, *s, Pi_interpolated);

            // std::cout<< "distance on clothoid, s:"<<*s<<std::endl;

            // Compute the Euclidean distance between Ps and Pi_interpolated
            double dx = m_Ps[0] - Pi_interpolated[0];
            double dy = m_Ps[1] - Pi_interpolated[1];
            double dz = m_Ps[2] - Pi_interpolated[2];
            residual[0] = std::sqrt(dx * dx + dy * dy + dz * dz);

            return true;
        }

    private:
        segment m_data; // Clothoid data
        double m_Ps[3]; // Point in space.
    };

    // Find closest point in space to clothoid.
    void solve_algorithm1(double* Ptraj, segment data, double eps, double *s) {
        // Create the Ceres problem
        ceres::Problem problem;

        // Create the cost function using numeric differentiation
        ceres::CostFunction* cost_function =
                new ceres::NumericDiffCostFunction<ResidualFunctorAlgorithm1, ceres::CENTRAL, 1, 1>(
                    new ResidualFunctorAlgorithm1(data, Ptraj)
                    );

        // std::cout << "solving closest point on clothoid." << std::endl;

        // Add the residual block to the problem
        problem.AddResidualBlock(cost_function, nullptr, s);

        // Set lower and upper bounds for s
        problem.SetParameterLowerBound(s, 0, 0); // s >= s_min
        problem.SetParameterUpperBound(s, 0, data.s1*4); // s <= s_max

        // Set up solver options
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = false; // Disable progress output
        options.max_num_iterations = 200;             // Maximum number of iterations
        options.function_tolerance = eps;             // Tolerance for cost function value

        // Solve the problem
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Print the solver summary
        // std::cout << summary.BriefReport() << "\n";
    }

    // Fit clothoid between 2 segments, given max deviation.
    struct ResidualFunctorFilletFit {
        ResidualFunctorFilletFit(segment *seg0, segment *seg1, segment *seg2, double *max_deviation){
            m_seg0 = *seg0;
            m_seg1 = *seg1;
            m_seg2 = *seg2;
            m_max_deviation = *max_deviation;
        }

        // S0 = trim distance first segment. S1 = trim distance second segment.
        bool operator()(const double* const s, double* residual) const {

            double m_deviation=0;
            residual_fillet_fit(m_seg0, m_seg1, m_seg2, s, &m_deviation);
            residual[0] = m_max_deviation - m_deviation;
            // std::cout<<"deviation:"<<m_deviation<<std::endl;
            return true;
        }

    private:

        double m_max_deviation=0;
        segment m_seg0;
        segment m_seg1;
        segment m_seg2;
    };

    void solve_fillet_fit(segment *seg0, segment *seg1, segment *seg2, double *max_deviation, double *s) {
        // Create the Ceres problem
        ceres::Problem problem;

        // Create the cost function using numeric differentiation
        ceres::CostFunction* cost_function =
                new ceres::NumericDiffCostFunction<ResidualFunctorFilletFit, ceres::CENTRAL,
                1 /* Nr of residuals*/,
                1 /* First param is a single double value*/>(
                    new ResidualFunctorFilletFit(seg0,seg1,seg2,max_deviation)
                    );

        // Add the residual block to the problem
        problem.AddResidualBlock(cost_function, nullptr, s);

        // Set lower and upper bounds for s
        problem.SetParameterLowerBound(s, 0, 1e-6); // Note: 1e-9 will show no clothoid fillet anymore.

        // problem.SetParameterUpperBound(s, 0, fmin(seg0->length*0.5, seg1->length*0.5));
        problem.SetParameterUpperBound(s, 0, fmin(seg0->original_length*0.499, seg1->original_length*0.499));

        // Set up solver options
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = false; // Disable progress output
        options.max_num_iterations = 200;             // Maximum number of iterations
        options.function_tolerance = 1e-6;            // Tolerance for cost function value

        // Solve the problem
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Print the solver summary
         std::cout << summary.BriefReport() << "\n";
    }
};

// C interface functions
extern "C" ceres_intf* init_cpp_ptr() {
    return new ceres_intf();
}

extern "C" void ceres_cpp_init(ceres_intf* ptr, void* parameters, const void* user_data) {
    ptr->init(parameters, user_data);
}

extern "C" void ceres_cpp_solve(ceres_intf* ptr,
                                double* y11,
                                double* y21,
                                double* s1,
                                int solver_type,
                                double s1_lower_limit,
                                double s1_upper_limit) {
    ptr->solve(y11, y21, s1, solver_type, s1_lower_limit, s1_upper_limit);
}

extern "C" void ceres_cpp_result(ceres_intf* ptr, double* result) {
    std::memcpy(result, ptr->m_parameters, sizeof(double) * 3);
}

extern "C" void ceres_cpp_solve_algorithm1(ceres_intf *ptr, double *Ptraj, struct segment data, double eps, double *s) {
    ptr->solve_algorithm1(Ptraj,data,eps,s);
}

extern "C" void ceres_cpp_fit(ceres_intf *ptr,
                              struct segment *seg0,
                              struct segment *seg1,
                              struct segment *seg2,
                              double *deviation,
                              double *s) {
    ptr->solve_fillet_fit(seg0,seg1,seg2,deviation,s);
}

extern "C" void ceres_cpp_free_ptr(ceres_intf *ptr) {
    if (ptr) {              // Check if the pointer is not null
        delete ptr;         // Free the memory
        ptr = nullptr;      // Optional: Set the pointer to null to avoid dangling pointers
    }
}
