#include "mex.h"
#include <vector>
#include <cmath>
#include <algorithm>
void getEulerJacobian(double* Q, double nx, double ny, double nz, double gamma, double A[5][5], double& lambda) {
    // Core math function calculating the exact 5x5 Flux Jacobian matrix (A = dF/dQ) for the Euler equations
    double r = Q[0]; double u = Q[1]/r; double v = Q[2]/r; double w = Q[3]/r; double E = Q[4]/r;
    double Vsq = u*u + v*v + w*w;
    double p = (gamma - 1.0) * (Q[4] - 0.5 * r * Vsq); // Ideal Gas state equation
    double H = (Q[4] + p) / r; // Total Enthalpy
    double U = u*nx + v*ny + w*nz; // Contravariant face velocity
    double phi = 0.5 * (gamma - 1.0) * Vsq;
    double c = std::sqrt(std::max(gamma * p / r, 1e-8)); // Local speed of sound
    lambda = std::abs(U) + c; // Spectral radius (maximum eigenvalue) used for Rusanov stabilization
    // Analytically derived derivatives of Mass, Momentum, and Energy fluxes
    A[0][0] = 0.0; A[0][1] = nx; A[0][2] = ny; A[0][3] = nz; A[0][4] = 0.0;
    A[1][0] = -u*U + nx*phi; A[1][1] = U - (gamma-2.0)*u*nx; A[1][2] = u*ny - (gamma-1.0)*v*nx; A[1][3] = u*nz - (gamma-1.0)*w*nx; A[1][4] = (gamma-1.0)*nx;
    A[2][0] = -v*U + ny*phi; A[2][1] = v*nx - (gamma-1.0)*u*ny; A[2][2] = U - (gamma-2.0)*v*ny; A[2][3] = v*nz - (gamma-1.0)*w*ny; A[2][4] = (gamma-1.0)*ny;
    A[3][0] = -w*U + nz*phi; A[3][1] = w*nx - (gamma-1.0)*u*nz; A[3][2] = w*ny - (gamma-1.0)*v*nz; A[3][3] = U - (gamma-2.0)*w*nz; A[3][4] = (gamma-1.0)*nz;
    A[4][0] = U*(phi - H); A[4][1] = H*nx - (gamma-1.0)*u*U; A[4][2] = H*ny - (gamma-1.0)*v*U; A[4][3] = H*nz - (gamma-1.0)*w*U; A[4][4] = gamma*U;
}
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // MEX entry point acting as a bridge between MATLAB memory and C++ pointers
    if(nrhs != 9) mexErrMsgTxt("Requires: N_cells, owner, neighbor, nx, ny, nz, area, Q, Gamma");
    int N_cells = (int)mxGetScalar(prhs[0]);
    double* owner = mxGetPr(prhs[1]);
    double* neighbor = mxGetPr(prhs[2]);
    double* fnx = mxGetPr(prhs[3]);
    double* fny = mxGetPr(prhs[4]);
    double* fnz = mxGetPr(prhs[5]);
    double* area = mxGetPr(prhs[6]);
    double* Q = mxGetPr(prhs[7]);
    double gamma = mxGetScalar(prhs[8]);
    int N_faces = mxGetNumberOfElements(prhs[1]);
    std::vector<double> idx_i, idx_j, val;
    idx_i.reserve(N_faces*100); idx_j.reserve(N_faces*100); val.reserve(N_faces*100); // Preallocate C++ vectors
    for(int f=0; f<N_faces; f++) {
        int o = (int)owner[f] - 1; int n = (int)neighbor[f] - 1;
        if(n < 0) continue; // Skip boundaries. Boundaries handled explicitly in MATLAB RHS loop
        double QL[5], QR[5];
        // Extract 5-state vector from interleaved 7-state array memory format
        for(int m=0; m<5; m++) { QL[m] = Q[o*7 + m]; QR[m] = Q[n*7 + m]; }
        double AL[5][5], AR[5][5], lamL, lamR;
        getEulerJacobian(QL, fnx[f], fny[f], fnz[f], gamma, AL, lamL); // Compute Left Face Jacobians
        getEulerJacobian(QR, fnx[f], fny[f], fnz[f], gamma, AR, lamR); // Compute Right Face Jacobians
        for(int m=0; m<5; m++) {
            for(int k=0; k<5; k++) {
                double diag = (m==k) ? 1.0 : 0.0;
                // Rusanov / Lax-Friedrichs numeric flux Jacobian splitting (Guarantees diagonal dominance)
                double dF_dL = 0.5*AL[m][k] + 0.5*lamL*diag;
                double dF_dR = 0.5*AR[m][k] - 0.5*lamR*diag;
                // Owner accumulation (RHS_o = RHS_o - Flux)
                idx_i.push_back(5*o + m + 1); idx_j.push_back(5*o + k + 1); val.push_back( dF_dL * area[f] );
                idx_i.push_back(5*o + m + 1); idx_j.push_back(5*n + k + 1); val.push_back( dF_dR * area[f] );
                // Neighbor accumulation (RHS_n = RHS_n + Flux)
                idx_i.push_back(5*n + m + 1); idx_j.push_back(5*o + k + 1); val.push_back( -dF_dL * area[f] );
                idx_i.push_back(5*n + m + 1); idx_j.push_back(5*n + k + 1); val.push_back( -dF_dR * area[f] );
            }
        }
    }
    // Construct returned MATLAB sparse-builder arrays (I, J, V)
    plhs[0] = mxCreateDoubleMatrix(1, idx_i.size(), mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, idx_j.size(), mxREAL);
    plhs[2] = mxCreateDoubleMatrix(1, val.size(), mxREAL);
    std::copy(idx_i.begin(), idx_i.end(), mxGetPr(plhs[0]));
    std::copy(idx_j.begin(), idx_j.end(), mxGetPr(plhs[1]));
    std::copy(val.begin(), val.end(), mxGetPr(plhs[2]));
}