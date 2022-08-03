#include <iostream>
double A[10*10];
double B[10*10];
double R[10*10];
int m,n,c;
//m*n   X n*c
struct mutrix{
    double A[10*10];
    int m,n;
};
void f(double *A,double *B,double  *R,int m,int n,int c){
    for(int i=0;i<m;i++)
        for(int j=0;j<c;j++){
            for(int k=0;k<n;k++){
                R[i*c+j]+= A[i*n+k]*B[k*c+j];

            }
        }
}
int main() {
    m=2,n=2,c=1;
    A[0]=1,A[1]=1.0/100,A[2]=0,A[3]=1;
    B[0]=0,B[1]=3;
    f(A,B,R,m,n,c);
    for(int i=0;i<2;i++)
        printf("%f \n",R[i]);

    return 0;
}
