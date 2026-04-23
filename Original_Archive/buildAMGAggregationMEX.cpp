#include "mex.h"
#include <vector>
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    int N = (int)mxGetScalar(prhs[0]);
    double* o = mxGetPr(prhs[1]); double* n = mxGetPr(prhs[2]);
    int N_f = mxGetNumberOfElements(prhs[1]);
    std::vector<int> head(N+1, -1), to(N_f*2), next(N_f*2);
    int edge_cnt=0;
    for(int i=0; i<N_f; ++i) {
        int u=(int)o[i]-1, v=(int)n[i]-1;
        if(u>=0 && v>=0) {
            to[edge_cnt]=v; next[edge_cnt]=head[u]; head[u]=edge_cnt++;
            to[edge_cnt]=u; next[edge_cnt]=head[v]; head[v]=edge_cnt++;
        }
    }
    std::vector<bool> is_aggr(N, false); std::vector<int> aggr_id(N, 0);
    int num_aggr = 0;
    for(int i=0; i<N; ++i) {
        if(!is_aggr[i]) {
            num_aggr++; aggr_id[i] = num_aggr; is_aggr[i] = true;
            for(int e=head[i]; e!=-1; e=next[e]) {
                int v = to[e];
                if(!is_aggr[v]) { aggr_id[v] = num_aggr; is_aggr[v] = true; }
            }
        }
    }
    std::vector<double> P_i, P_j; P_i.reserve(N*7); P_j.reserve(N*7);
    for(int c=0; c<N; ++c) {
        for(int d=1; d<=7; ++d) {
            P_i.push_back(7*c + d); P_j.push_back(7*(aggr_id[c]-1) + d);
        }
    }
    plhs[0]=mxCreateDoubleMatrix(1,P_i.size(),mxREAL); double* out_i=mxGetPr(plhs[0]);
    plhs[1]=mxCreateDoubleMatrix(1,P_j.size(),mxREAL); double* out_j=mxGetPr(plhs[1]);
    plhs[2]=mxCreateDoubleScalar((double)num_aggr);
    for(size_t i=0; i<P_i.size(); ++i) { out_i[i]=P_i[i]; out_j[i]=P_j[i]; }
}