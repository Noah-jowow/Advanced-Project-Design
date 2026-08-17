#include "mex.h"
#include <vector>
#include <cmath>
#include <algorithm>
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if(nrhs != 3) mexErrMsgTxt("Requires N, A_scalar, theta");
    int N = (int)mxGetScalar(prhs[0]);
    const mxArray* A = prhs[1];
    double theta = mxGetScalar(prhs[2]);
    mwIndex *jc = mxGetJc(A); mwIndex *ir = mxGetIr(A); double *pr = mxGetPr(A);
    std::vector<double> max_a(N, 0.0);
    for(int j=0; j<N; ++j) {
        for(mwIndex k=jc[j]; k<jc[j+1]; ++k) {
            int i = ir[k];
            if(i != j) {
                double val = std::abs(pr[k]);
                if(val > max_a[i]) max_a[i] = val;
            }
        }
    }
    std::vector<std::vector<int>> S(N);
    for(int j=0; j<N; ++j) {
        for(mwIndex k=jc[j]; k<jc[j+1]; ++k) {
            int i = ir[k];
            if(i != j) {
                if(std::abs(pr[k]) >= theta * max_a[i]) {
                    S[i].push_back(j);
                }
            }
        }
    }
    std::vector<int> aggr_id(N, 0);
    std::vector<bool> is_aggr(N, false);
    int num_aggr = 0;
    for(int i=0; i<N; ++i) {
        if(!is_aggr[i]) {
            num_aggr++; aggr_id[i] = num_aggr; is_aggr[i] = true;
            for(size_t n=0; n<S[i].size(); ++n) {
                int neighbor = S[i][n];
                if(!is_aggr[neighbor]) {
                    aggr_id[neighbor] = num_aggr; is_aggr[neighbor] = true;
                }
            }
        }
    }
    plhs[0] = mxCreateDoubleMatrix(1, N, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, N, mxREAL);
    double *out_i = mxGetPr(plhs[0]); double *out_j = mxGetPr(plhs[1]);
    for(int i=0; i<N; ++i) {
        out_i[i] = i + 1; out_j[i] = aggr_id[i];
    }
    plhs[2] = mxCreateDoubleScalar(num_aggr);
}