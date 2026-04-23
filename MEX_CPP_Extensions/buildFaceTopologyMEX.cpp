#include "mex.h"
#include <vector>
#include <algorithm>
struct Face { int n[3]; int cell; };
bool cmp(const Face& a, const Face& b) {
    if(a.n[0]!=b.n[0]) return a.n[0]<b.n[0];
    if(a.n[1]!=b.n[1]) return a.n[1]<b.n[1];
    return a.n[2]<b.n[2];
}
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    double* tets = mxGetPr(prhs[0]);
    int N = mxGetM(prhs[0]);
    std::vector<Face> F(N*4);
    for(int i=0; i<N; ++i) {
        int n1=(int)tets[i], n2=(int)tets[i+N], n3=(int)tets[i+2*N], n4=(int)tets[i+3*N];
        int f1[3]={n1,n2,n3}, f2[3]={n1,n2,n4}, f3[3]={n1,n3,n4}, f4[3]={n2,n3,n4};
        std::sort(f1,f1+3); std::sort(f2,f2+3); std::sort(f3,f3+3); std::sort(f4,f4+3);
        F[i*4]={f1[0],f1[1],f1[2], i+1}; F[i*4+1]={f2[0],f2[1],f2[2], i+1};
        F[i*4+2]={f3[0],f3[1],f3[2], i+1}; F[i*4+3]={f4[0],f4[1],f4[2], i+1};
    }
    std::sort(F.begin(), F.end(), cmp);
    std::vector<int> o, n, n_a, n_b, n_c;
    o.reserve(N*2); n.reserve(N*2); n_a.reserve(N*2); n_b.reserve(N*2); n_c.reserve(N*2);
    for(size_t i=0; i<F.size(); ++i) {
        if(i>0 && F[i].n[0]==F[i-1].n[0] && F[i].n[1]==F[i-1].n[1] && F[i].n[2]==F[i-1].n[2]) {
            n.back() = F[i].cell;
        } else {
            o.push_back(F[i].cell); n.push_back(-1);
            n_a.push_back(F[i].n[0]); n_b.push_back(F[i].n[1]); n_c.push_back(F[i].n[2]);
        }
    }
    plhs[0]=mxCreateDoubleMatrix(1,o.size(),mxREAL); double* out_o=mxGetPr(plhs[0]);
    plhs[1]=mxCreateDoubleMatrix(1,n.size(),mxREAL); double* out_n=mxGetPr(plhs[1]);
    plhs[2]=mxCreateDoubleMatrix(o.size(),3,mxREAL); double* out_nodes=mxGetPr(plhs[2]);
    size_t numF = o.size();
    for(size_t i=0; i<numF; ++i) {
        out_o[i]=o[i]; out_n[i]=n[i];
        out_nodes[i]=n_a[i]; out_nodes[i+numF]=n_b[i]; out_nodes[i+2*numF]=n_c[i];
    }
}