#ifndef JACOBIAN_H
#define JACOBIAN_H

// Compute the Jacobian using finite differences
void jacobian(const double *p, double *xyz_vector, double* J, void* cookie, double *residual){

    // Casting cookie to access the parameters
    double* params = (double*)cookie;

    // Extract the current guess for gamma20 and s1
    double gamma10 = p[0];
    double gamma20 = p[1];
    double s1 = p[2];

    // Extract the fixed parameters from the cookie
    double theta10 = params[0];
    double kappa10 = params[1];
    double sharpness10 = params[2];

    double theta20 = params[3];
    double kappa20 = params[4];
    double sharpness20 = params[5];

    double xs = params[6];
    double ys = params[7];
    double zs = params[8];

    double xe = params[9];
    double ye = params[10];
    double ze = params[11];

    // Variables for the computed point
    double xi, yi, zi, ltot;

    // Call the clothoid equation function to compute the point (xi, yi, zi)
    eq14_gauss(s1, xs, ys, zs,
               theta10, theta20,
               kappa10, kappa20,
               sharpness10, sharpness20,
               gamma10, gamma20,
               &xi, &yi, &zi, &ltot);

    printf("xi: %f, y: %f, zi: %f \n", xi, yi, zi);

    // Compute the residuals (errors)
    double dx = xe - xi;
    double dy = ye - yi;
    double dz = ze - zi;

    // Calculate the Euclidean error
    *residual = sqrt(dx * dx + dy * dy + dz * dz);

    // Optionally print the error for debugging
    // printf("error: %f \n", error);

    // Return the residuals in x[] array (to be minimized by the optimizer)
    xyz_vector[0] = dx;
    xyz_vector[1] = dy;
    xyz_vector[2] = dz;  // Include the third component (dz) for the residual

    // Compute the Jacobian (partial derivatives)
    double delta = 1e-6;  // Small perturbation for numerical differentiation

    // Derivatives of residuals with respect to each parameter
    // dx/dgamma10
    double p1 = p[0] + delta;
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, p1, gamma20, &xi, &yi, &zi, &ltot);
    double dx_dgamma10 = (xe - xi) - dx;

    // dx/dgamma20
    double p2 = p[1] + delta;
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, p2, &xi, &yi, &zi, &ltot);
    double dx_dgamma20 = (xe - xi) - dx;

    // dx/ds1
    double p3 = p[2] + delta;
    eq14_gauss(p3, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, gamma20, &xi, &yi, &zi, &ltot);
    double dx_ds1 = (xe - xi) - dx;

    // Assign the Jacobian values
    J[0] = dx_dgamma10 / delta;  // d(dx)/d(gamma10)
    J[1] = dx_dgamma20 / delta;  // d(dx)/d(gamma20)
    J[2] = dx_ds1 / delta;       // d(dx)/d(s1)

    printf("J0 %f \n",J[0]);
    printf("J1 %f \n",J[1]);
    printf("J2 %f \n",J[2]);

    // Repeat the same process for dy and dz
    // dy/dgamma10
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, p1, gamma20, &xi, &yi, &zi, &ltot);
    double dy_dgamma10 = (ye - yi) - dy;

    // dy/dgamma20
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, p2, &xi, &yi, &zi, &ltot);
    double dy_dgamma20 = (ye - yi) - dy;

    // dy/ds1
    eq14_gauss(p3, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, gamma20, &xi, &yi, &zi, &ltot);
    double dy_ds1 = (ye - yi) - dy;

    J[3] = dy_dgamma10 / delta;
    J[4] = dy_dgamma20 / delta;
    J[5] = dy_ds1 / delta;

    printf("J3 %f \n",J[3]);
    printf("J4 %f \n",J[4]);
    printf("J5 %f \n",J[5]);

    // dz/dgamma10
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, p1, gamma20, &xi, &yi, &zi, &ltot);
    double dz_dgamma10 = (ze - zi) - dz;

    // dz/dgamma20
    eq14_gauss(s1, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, p2, &xi, &yi, &zi, &ltot);
    double dz_dgamma20 = (ze - zi) - dz;

    // dz/ds1
    eq14_gauss(p3, xs, ys, zs, theta10, theta20, kappa10, kappa20, sharpness10, sharpness20, gamma10, gamma20, &xi, &yi, &zi, &ltot);
    double dz_ds1 = (ze - zi) - dz;

    J[6] = dz_dgamma10 / delta;
    J[7] = dz_dgamma20 / delta;
    J[8] = dz_ds1 / delta;

    printf("J6 %f \n",J[6]);
    printf("J7 %f \n",J[7]);
    printf("J8 %f \n",J[8]);
}

#endif // JACOBIAN_H
