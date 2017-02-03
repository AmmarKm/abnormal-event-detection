//---------------------------------------------------------------------------

#ifndef MyMathH
#define MyMathH
//---------------------------------------------------------------------------

//void jacobi(float **a, int n, float d[], float **v, int *nrot);

void jacobi1(double **a, int n, double d[], double **v, int *nrot);
void jacobi2(double **a, int n, double d[], double **v, int *nrot);
void jacobi3(double **a, int n, double d[], double **v, int *nrot);
void eigsrt(double d[], double **v, int n);

int argmin(double *dist, int length, double *value=0);
int Normalize(double *arr, int length, double newMin=0, double newMax=1);

#endif
