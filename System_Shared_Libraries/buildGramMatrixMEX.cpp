#include "mex.h"
#include <vector>
#include <cmath>
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    int N_cells = (int)mxGetScalar(prhs[0]); int N_faces = (int)mxGetScalar(prhs[1]);
    double *owner = mxGetPr(prhs[2]); double *neighbor = mxGetPr(prhs[3]);
    double *cx = mxGetPr(prhs[4]); double *cy = mxGetPr(prhs[5]); double *cz = mxGetPr(prhs[6]);
    std::vector<std::vector<int>> n_lists(N_cells);
    for(int f=0; f<N_faces; ++f) {
        int o = (int)owner[f] - 1; int n = (int)neighbor[f] - 1;
        if(o >= 0 && n >= 0) { n_lists[o].push_back(n); n_lists[n].push_back(o); }
    }
    std::vector<double> idx_i, idx_j, val_x, val_y, val_z;
    idx_i.reserve(N_cells * 7); idx_j.reserve(N_cells * 7); val_x.reserve(N_cells * 7); val_y.reserve(N_cells * 7); val_z.reserve(N_cells * 7);
    for(int c=0; c<N_cells; ++c) {
        double Ixx=0, Iyy=0, Izz=0, Ixy=0, Ixz=0, Iyz=0;
        int n_cnt = n_lists[c].size();
        std::vector<double> W_list(n_cnt), dx_list(n_cnt), dy_list(n_cnt), dz_list(n_cnt);
        for(int m=0; m<n_cnt; ++m) {
            int n = n_lists[c][m]; double dx = cx[n]-cx[c]; double dy = cy[n]-cy[c]; double dz = cz[n]-cz[c];
            double w = 1.0 / std::max(dx*dx + dy*dy + dz*dz, 1e-12);
            W_list[m] = w; dx_list[m] = dx; dy_list[m] = dy; dz_list[m] = dz;
            Ixx += w*dx*dx; Iyy += w*dy*dy; Izz += w*dz*dz; Ixy += w*dx*dy; Ixz += w*dx*dz; Iyz += w*dy*dz;
        }
        double detI = Ixx*(Iyy*Izz - Iyz*Iyz) - Ixy*(Ixy*Izz - Ixz*Iyz) + Ixz*(Ixy*Iyz - Iyy*Ixz);
        if(detI < 1e-14) { Ixx += 1e-8; Iyy += 1e-8; Izz += 1e-8; detI = Ixx*(Iyy*Izz - Iyz*Iyz) - Ixy*(Ixy*Izz - Ixz*Iyz) + Ixz*(Ixy*Iyz - Iyy*Ixz); }
        double invIxx = (Iyy*Izz - Iyz*Iyz)/detI; double invIxy = (Ixz*Iyz - Ixy*Izz)/detI; double invIxz = (Ixy*Iyz - Iyy*Ixz)/detI;
        double invIyy = (Ixx*Izz - Ixz*Ixz)/detI; double invIyz = (Ixy*Ixz - Ixx*Iyz)/detI; double invIzz = (Ixx*Iyy - Ixy*Ixy)/detI;
        double cx_s=0, cy_s=0, cz_s=0;
        for(int m=0; m<n_cnt; ++m) {
            int n = n_lists[c][m]; double w = W_list[m]; double dx = dx_list[m], dy = dy_list[m], dz = dz_list[m];
            double c_xj = w*(invIxx*dx + invIxy*dy + invIxz*dz); double c_yj = w*(invIxy*dx + invIyy*dy + invIyz*dz); double c_zj = w*(invIxz*dx + invIyz*dy + invIzz*dz);
            idx_i.push_back(c+1); idx_j.push_back(n+1); val_x.push_back(c_xj); val_y.push_back(c_yj); val_z.push_back(c_zj);
            cx_s -= c_xj; cy_s -= c_yj; cz_s -= c_zj;
        }
        idx_i.push_back(c+1); idx_j.push_back(c+1); val_x.push_back(cx_s); val_y.push_back(cy_s); val_z.push_back(cz_s);
    }
    plhs[0] = mxCreateDoubleMatrix(1, idx_i.size(), mxREAL); plhs[1] = mxCreateDoubleMatrix(1, idx_j.size(), mxREAL);
    plhs[2] = mxCreateDoubleMatrix(1, val_x.size(), mxREAL); plhs[3] = mxCreateDoubleMatrix(1, val_y.size(), mxREAL); plhs[4] = mxCreateDoubleMatrix(1, val_z.size(), mxREAL);
    std::copy(idx_i.begin(), idx_i.end(), mxGetPr(plhs[0])); std::copy(idx_j.begin(), idx_j.end(), mxGetPr(plhs[1]));
    std::copy(val_x.begin(), val_x.end(), mxGetPr(plhs[2])); std::copy(val_y.begin(), val_y.end(), mxGetPr(plhs[3])); std::copy(val_z.begin(), val_z.end(), mxGetPr(plhs[4]));
}