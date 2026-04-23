#include "mex.h"
#include <vector>
#include <cmath>
#include <algorithm>
void getEulerJacobian(double* Q, double nx, double ny, double nz, double gamma, double A[7][7], double& lambda) {
    double r = std::max(Q[0], 1e-5); double u = Q[1]/r; double v = Q[2]/r; double w = Q[3]/r; double E = Q[4]/r;
    double Vsq = u*u + v*v + w*w;
    double p = std::max((gamma - 1.0) * (Q[4] - 0.5 * r * Vsq), 1.0);
    double H = (Q[4] + p) / r;
    double U = u*nx + v*ny + w*nz;
    double phi = 0.5 * (gamma - 1.0) * Vsq;
    double c = std::sqrt(gamma * p / r);
    lambda = std::abs(U) + c;
    for(int i=0; i<7; ++i) for(int j=0; j<7; ++j) A[i][j] = 0.0;
    A[0][0] = 0.0; A[0][1] = nx; A[0][2] = ny; A[0][3] = nz; A[0][4] = 0.0;
    A[1][0] = -u*U + nx*phi; A[1][1] = U - (gamma-2.0)*u*nx; A[1][2] = u*ny - (gamma-1.0)*v*nx; A[1][3] = u*nz - (gamma-1.0)*w*nx; A[1][4] = (gamma-1.0)*nx;
    A[2][0] = -v*U + ny*phi; A[2][1] = v*nx - (gamma-1.0)*u*ny; A[2][2] = U - (gamma-2.0)*v*ny; A[2][3] = v*nz - (gamma-1.0)*w*ny; A[2][4] = (gamma-1.0)*ny;
    A[3][0] = -w*U + nz*phi; A[3][1] = w*nx - (gamma-1.0)*u*nz; A[3][2] = w*ny - (gamma-1.0)*v*nz; A[3][3] = U - (gamma-2.0)*w*nz; A[3][4] = (gamma-1.0)*nz;
    A[4][0] = U*(phi - H); A[4][1] = H*nx - (gamma-1.0)*u*U; A[4][2] = H*ny - (gamma-1.0)*v*U; A[4][3] = H*nz - (gamma-1.0)*w*U; A[4][4] = gamma*U;
    double beta_star = 0.09; double dest_w = 0.075;
    A[5][5] = U + beta_star * Q[6];
    A[5][6] = beta_star * Q[5];
    A[6][5] = 0.0;
    A[6][6] = U + 2.0 * dest_w * Q[6];
}
bool invert7x7(double M[7][7], double Inv[7][7]) {
    double A[7][14];
    for(int i=0; i<7; ++i) { for(int j=0; j<7; ++j) { A[i][j] = M[i][j]; A[i][j+7] = (i==j)?1.0:0.0; } }
    for(int i=0; i<7; ++i) {
        double maxEl = std::abs(A[i][i]); int maxRow = i;
        for(int k=i+1; k<7; ++k) { if(std::abs(A[k][i]) > maxEl) { maxEl = std::abs(A[k][i]); maxRow = k; } }
        if(maxEl < 1e-14) return false;
        if(maxRow != i) { for(int k=0; k<14; ++k) std::swap(A[i][k], A[maxRow][k]); }
        double pivot = A[i][i];
        for(int k=0; k<14; ++k) A[i][k] /= pivot;
        for(int k=0; k<7; ++k) {
            if(k != i) {
                double factor = A[k][i];
                for(int j=0; j<14; ++j) A[k][j] -= factor * A[i][j];
            }
        }
    }
    for(int i=0; i<7; ++i) { for(int j=0; j<7; ++j) { Inv[i][j] = A[i][j+7]; } }
    return true;
}
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if(nrhs != 10) mexErrMsgTxt("Requires: N_cells, owner, neighbor, nx, ny, nz, area, Q, dt_inv, Gamma");
    int N_cells = (int)mxGetScalar(prhs[0]);
    double* owner = mxGetPr(prhs[1]); double* neighbor = mxGetPr(prhs[2]);
    double* fnx = mxGetPr(prhs[3]); double* fny = mxGetPr(prhs[4]); double* fnz = mxGetPr(prhs[5]);
    double* area = mxGetPr(prhs[6]); double* Q = mxGetPr(prhs[7]);
    double* dt_inv = mxGetPr(prhs[8]); double gamma = mxGetScalar(prhs[9]);
    int N_faces = mxGetNumberOfElements(prhs[1]);
    std::vector<double> idx_i, idx_j, val;
    idx_i.reserve(N_faces*150); idx_j.reserve(N_faces*150); val.reserve(N_faces*150);
    std::vector<std::vector<std::vector<double>>> Diag(N_cells, std::vector<std::vector<double>>(7, std::vector<double>(7, 0.0)));
    for(int c=0; c<N_cells; c++) { for(int m=0; m<7; m++) Diag[c][m][m] = dt_inv[m*N_cells + c]; }
    for(int f=0; f<N_faces; f++) {
        int o = (int)owner[f] - 1; int n = (int)neighbor[f] - 1;
        double QL[7]; for(int m=0; m<7; m++) QL[m] = Q[o*7 + m];
        if(n < 0) {
            double AL[7][7], lamL;
            getEulerJacobian(QL, fnx[f], fny[f], fnz[f], gamma, AL, lamL);
            for(int m=0; m<7; m++) {
                for(int k=0; k<7; k++) {
                    double diag_term = (m==k) ? 1.0 : 0.0;
                    double dF_dL = 0.5*AL[m][k] + 0.5*lamL*diag_term;
                    Diag[o][m][k] += dF_dL * area[f];
                }
            }
            continue;
        }
        double QR[7]; for(int m=0; m<7; m++) QR[m] = Q[n*7 + m];
        double AL[7][7], AR[7][7], lamL, lamR;
        getEulerJacobian(QL, fnx[f], fny[f], fnz[f], gamma, AL, lamL);
        getEulerJacobian(QR, fnx[f], fny[f], fnz[f], gamma, AR, lamR);
        for(int m=0; m<7; m++) {
            for(int k=0; k<7; k++) {
                double diag_term = (m==k) ? 1.0 : 0.0;
                double dF_dL = 0.5*AL[m][k] + 0.5*lamL*diag_term;
                double dF_dR = 0.5*AR[m][k] - 0.5*lamR*diag_term;
                Diag[o][m][k] += dF_dL * area[f]; Diag[n][m][k] -= dF_dR * area[f];
                idx_i.push_back(7*o + m + 1); idx_j.push_back(7*n + k + 1); val.push_back( dF_dR * area[f] );
                idx_i.push_back(7*n + m + 1); idx_j.push_back(7*o + k + 1); val.push_back( -dF_dL * area[f] );
            }
        }
    }
    std::vector<double> M_inv_i, M_inv_j, M_inv_v;
    for(int c=0; c<N_cells; c++) {
        double M[7][7], Minv[7][7];
        for(int i=0; i<7; i++) for(int j=0; j<7; j++) M[i][j] = Diag[c][i][j];
        if(invert7x7(M, Minv)) {
            for(int m=0; m<7; m++) { for(int k=0; k<7; k++) {
                M_inv_i.push_back(7*c + m + 1); M_inv_j.push_back(7*c + k + 1); M_inv_v.push_back(Minv[m][k]);
            }}
        } else {
            for(int m=0; m<7; m++) { M_inv_i.push_back(7*c + m + 1); M_inv_j.push_back(7*c + m + 1); M_inv_v.push_back(1.0/std::max(M[m][m], 1e-12)); }
        }
        for(int m=0; m<7; m++) { for(int k=0; k<7; k++) {
            idx_i.push_back(7*c + m + 1); idx_j.push_back(7*c + k + 1); val.push_back(M[m][k]);
        }}
    }
    plhs[0] = mxCreateDoubleMatrix(1, idx_i.size(), mxREAL); plhs[1] = mxCreateDoubleMatrix(1, idx_j.size(), mxREAL); plhs[2] = mxCreateDoubleMatrix(1, val.size(), mxREAL);
    plhs[3] = mxCreateDoubleMatrix(1, M_inv_i.size(), mxREAL); plhs[4] = mxCreateDoubleMatrix(1, M_inv_j.size(), mxREAL); plhs[5] = mxCreateDoubleMatrix(1, M_inv_v.size(), mxREAL);
    std::copy(idx_i.begin(), idx_i.end(), mxGetPr(plhs[0])); std::copy(idx_j.begin(), idx_j.end(), mxGetPr(plhs[1])); std::copy(val.begin(), val.end(), mxGetPr(plhs[2]));
    std::copy(M_inv_i.begin(), M_inv_i.end(), mxGetPr(plhs[3])); std::copy(M_inv_j.begin(), M_inv_j.end(), mxGetPr(plhs[4])); std::copy(M_inv_v.begin(), M_inv_v.end(), mxGetPr(plhs[5]));
}