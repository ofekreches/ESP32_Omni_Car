#ifndef COVARIANCE_MATRIX_H
#define COVARIANCE_MATRIX_H


// pos x , pos y , pos angular , velocity x , velocity y ,velocity angular
const double covariance_matrix[6][6] = {
    {2.2705591e-5,          0,              0,              0,              0,              0},
    {0,              1.23080771e-4,      0,              0,              0,              0},
    {0,              0,              0.0012185,      0,              0,              0},
    {0,              0,              0,              2.7012e-8,      0,              0},
    {0,              0,              0,              0,              2.7012e-8,      0},
    {0,              0,              0,              0,              0,              3.05e-6}
};

#endif // COVARIANCE_MATRIX_H
