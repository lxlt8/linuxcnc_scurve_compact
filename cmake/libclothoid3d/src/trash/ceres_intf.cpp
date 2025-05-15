#include "ceres_intf.h"
#include "ceres/c_api.h"
#include <ceres/ceres.h>
#include <iostream>
#include <cstring>
#include "residual_function_jacobian.h"

class ceres_intf {
public:
    ceres_intf() {}

    ~ceres_intf() {}

    void init(void* parameters, const void* user_data) {
        std::memcpy(m_parameters, parameters, sizeof(double) * 3);
        std::memcpy(m_user_data, user_data, sizeof(double) * 18);
        std::cout << "ceres init ok." << std::endl;
    }

    void solve(double *y11, double *y21, double *s1) {
        std::cout << "solving with ceres c++" << std::endl;

        double *parameter_pointers[] = { y11, y21, s1 };
        int parameter_sizes[] = { 1, 1, 1 };
        int num_observations = 1000;

        ceres_problem_t* problem;

        /* Create the Ceres problem */
        problem = ceres_create_problem();
        if (problem == NULL) {
            fprintf(stderr, "Failed to create Ceres problem.\n");
            return;
        }

        /* Add all the residuals. */
        for (int i = 0; i < num_observations; ++i) {
            ceres_problem_add_residual_block(
                        problem,
                        residual_function_jacobian,  // Cost function
                        &m_user_data,            // Points to the user data
                        NULL,                  // No loss function
                        NULL,                  // No loss function user data
                        3,                     // Number of residuals
                        3,                     // Number of parameter blocks
                        parameter_sizes,
                        parameter_pointers);
        }

        /* Solve the problem */
        ceres_solve(problem);

        /* Free the problem */
        ceres_free_problem(problem);

    }

    double* result() {
        return m_parameters;
    }

    double m_parameters[3];
    double m_user_data[18];
};

extern "C" ceres_intf* init_cpp_ptr() {
    return new ceres_intf();
}

extern "C" void ceres_cpp_init(ceres_intf* ptr, void* parameters, const void* user_data) {
    ptr->init(parameters, user_data);
}

extern "C" void ceres_cpp_solve(ceres_intf* ptr, double *y11, double *y21, double *s1) {
    ptr->solve(y11,y21,s1);
}

extern "C" void ceres_cpp_result(ceres_intf* ptr, double* result) {
    std::memcpy(result, ptr->m_parameters, sizeof(double) * 3);
}
