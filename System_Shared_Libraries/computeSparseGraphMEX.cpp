#include "mex.h"
#include <vector>
#include <cmath>
#include <algorithm>
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if(nrhs != 6) mexErrMsgTxt("Requires 6 inputs: N_cells, owner, neighbor, CX, CY, CZ");
    int N_cells = (int)mxGetScalar(prhs[0]);
    double* owner = mxGetPr(prhs[1]);
    double* neighbor = mxGetPr(prhs[2]);
    int N_faces = mxGetNumberOfElements(prhs[1]);
    double* CX = mxGetPr(prhs[3]);
    double* CY = mxGetPr(prhs[4]);
    double* CZ = mxGetPr(prhs[5]);
    std::vector<std::vector<int>> adj(N_cells);
    for(int f=0; f<N_faces; f++) {
        int o = (int)owner[f] - 1; int n = (int)neighbor[f] - 1;
        if(n >= 0) { adj[o].push_back(n); adj[n].push_back(o); }
    }
    std::vector<double> idx_i, idx_j, vx, vy, vz;
    idx_i.reserve(N_cells*10); idx_j.reserve(N_cells*10);
    for(int c=0; c<N_cells; c++) {
        double Ixx=0, Iyy=0, Izz=0;
        std::vector<double> dx_l, dy_l, dz_l, w_l;
        for(size_t m=0; m<adj[c].size(); m++) {
            int n = adj[c][m];
            double dx = CX[n] - CX[c]; double dy = CY[n] - CY[c]; double dz = CZ[n] - CZ[c];
            double dist = std::max(std::sqrt(dx*dx + dy*dy + dz*dz), 1e-8);
            double w = 1.0 / dist;
            Ixx += w*dx*dx; Iyy += w*dy*dy; Izz += w*dz*dz;
            dx_l.push_back(dx); dy_l.push_back(dy); dz_l.push_back(dz); w_l.push_back(w);
        }
        double invIxx = 1.0/std::max(Ixx, 1e-10); double invIyy = 1.0/std::max(Iyy, 1e-10); double invIzz = 1.0/std::max(Izz, 1e-10);
        double cx_s=0, cy_s=0, cz_s=0;
        for(size_t m=0; m<adj[c].size(); m++) {
            int n = adj[c][m]; double w = w_l[m];
            double cx_j = w * invIxx * dx_l[m]; double cy_j = w * invIyy * dy_l[m]; double cz_j = w * invIzz * dz_l[m];
            idx_i.push_back(c+1); idx_j.push_back(n+1);
            vx.push_back(cx_j); vy.push_back(cy_j); vz.push_back(cz_j);
            cx_s -= cx_j; cy_s -= cy_j; cz_s -= cz_j;
        }
        idx_i.push_back(c+1); idx_j.push_back(c+1);
        vx.push_back(cx_s); vy.push_back(cy_s); vz.push_back(cz_s);
    }
    plhs[0] = mxCreateDoubleMatrix(1, idx_i.size(), mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, idx_j.size(), mxREAL);
    plhs[2] = mxCreateDoubleMatrix(1, vx.size(), mxREAL);
    plhs[3] = mxCreateDoubleMatrix(1, vy.size(), mxREAL);
    plhs[4] = mxCreateDoubleMatrix(1, vz.size(), mxREAL);
    std::copy(idx_i.begin(), idx_i.end(), mxGetPr(plhs[0]));
    std::copy(idx_j.begin(), idx_j.end(), mxGetPr(plhs[1]));
    std::copy(vx.begin(), vx.end(), mxGetPr(plhs[2]));
    std::copy(vy.begin(), vy.end(), mxGetPr(plhs[3]));
    std::copy(vz.begin(), vz.end(), mxGetPr(plhs[4]));
}