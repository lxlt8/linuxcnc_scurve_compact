#include <dogleg.h>
#include <stdio.h>

// Residual function for dense Jacobian
void residual_function(const double* p, double* x, double* J, void* cookie) {
    double x_val = p[0];
    double y_val = p[1];
    double z = *((double*)cookie);

    // Residual x[0] has to be zero finally. Then left hand side = right hand side.
    // The right hand side here is a example function to be solved.
    x[0] = x_val * x_val + y_val * y_val - z;

    // Jacobian (gradient of the residual)
    J[0] = 2 * x_val; // df/dx
    J[1] = 2 * y_val; // df/dy
}

int powel_example() {
    // Initial guess for x and y
    double p[2] = {1.0, 1.0};

    // Right-hand side result
    double z = 5.0;

    // Solver parameters
    dogleg_parameters2_t parameters;
    dogleg_getDefaultParameters(&parameters);

    // Set termination thresholds
    parameters.Jt_x_threshold = 1e-6;
    parameters.update_threshold = 1e-6;
    parameters.trustregion_threshold = 1e-6;

    // Solver context
    dogleg_solverContext_t* ctx = NULL;

    // Call the dense optimizer
    double residual_norm = dogleg_optimize_dense2(
        p,                // Initial guess and output solution
        2,                // Number of state variables (x and y)
        1,                // Number of measurements (residuals)
        residual_function, // Residual function
        &z,               // Cookie (user data)
        &parameters,      // Solver parameters
        &ctx              // Solver context (optional)
    );

    // Check if the solver succeeded
    if (residual_norm >= 0) {
        printf("\nPowel Example: \n");
        printf("Solution found: x = %f, y = %f\n", p[0], p[1]);
        printf("Residual norm: %f\n", residual_norm);
    } else {
        printf("Solver failed.\n");
    }

    // Free the solver context if it was created
    if (ctx) {
        dogleg_freeContext(&ctx);
    }

    return 0;
}
