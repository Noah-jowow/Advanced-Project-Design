#include "mex.h"
#include <cmath>
#include <limits>
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    double *nx = mxGetPr(prhs[0]); double *ny = mxGetPr(prhs[1]); double *nz = mxGetPr(prhs[2]);
    double *tets = mxGetPr(prhs[3]); int N_cells = mxGetM(prhs[3]);
    double *fcx = mxGetPr(prhs[4]); double *fcy = mxGetPr(prhs[5]); double *fcz = mxGetPr(prhs[6]);
    double *fnx = mxGetPr(prhs[7]); double *fny = mxGetPr(prhs[8]); double *fnz = mxGetPr(prhs[9]);
    int N_stl = mxGetNumberOfElements(prhs[4]);
    plhs[0] = mxCreateLogicalMatrix(N_cells, 1); bool *valid = mxGetLogicals(plhs[0]);
    plhs[1] = mxCreateDoubleMatrix(1, N_cells, mxREAL); plhs[2] = mxCreateDoubleMatrix(1, N_cells, mxREAL);
    plhs[3] = mxCreateDoubleMatrix(1, N_cells, mxREAL); plhs[4] = mxCreateDoubleMatrix(1, N_cells, mxREAL);
    plhs[5] = mxCreateDoubleMatrix(1, N_cells, mxREAL);
    double *cx_out = mxGetPr(plhs[1]); double *cy_out = mxGetPr(plhs[2]); double *cz_out = mxGetPr(plhs[3]);
    double *vol_out = mxGetPr(plhs[4]); double *wd_out = mxGetPr(plhs[5]);
    for(int c=0; c<N_cells; ++c) {
        int n1 = (int)tets[c] - 1; int n2 = (int)tets[c + N_cells] - 1;
        int n3 = (int)tets[c + 2*N_cells] - 1; int n4 = (int)tets[c + 3*N_cells] - 1;
        double cx = (nx[n1]+nx[n2]+nx[n3]+nx[n4])*0.25; double cy = (ny[n1]+ny[n2]+ny[n3]+ny[n4])*0.25; double cz = (nz[n1]+nz[n2]+nz[n3]+nz[n4])*0.25;
        cx_out[c] = cx; cy_out[c] = cy; cz_out[c] = cz;
        double v1x = nx[n2]-nx[n1]; double v1y = ny[n2]-ny[n1]; double v1z = nz[n2]-nz[n1];
        double v2x = nx[n3]-nx[n1]; double v2y = ny[n3]-ny[n1]; double v2z = nz[n3]-nz[n1];
        double v3x = nx[n4]-nx[n1]; double v3y = ny[n4]-ny[n1]; double v3z = nz[n4]-nz[n1];
        vol_out[c] = std::abs(v1x*(v2y*v3z - v2z*v3y) + v1y*(v2z*v3x - v2x*v3z) + v1z*(v2x*v3y - v2y*v3x)) / 6.0;
        double min_dist_sq = std::numeric_limits<double>::max(); int best_f = -1;
        for(int f=0; f<N_stl; ++f) {
            double dsq = (cx-fcx[f])*(cx-fcx[f]) + (cy-fcy[f])*(cy-fcy[f]) + (cz-fcz[f])*(cz-fcz[f]);
            if(dsq < min_dist_sq) { min_dist_sq = dsq; best_f = f; }
        }
        wd_out[c] = std::sqrt(min_dist_sq);
        if(best_f >= 0) { valid[c] = ((cx-fcx[best_f])*fnx[best_f] + (cy-fcy[best_f])*fny[best_f] + (cz-fcz[best_f])*fnz[best_f] > -1e-5); }
        else { valid[c] = true; }
    }
}