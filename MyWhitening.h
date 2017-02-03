//---------------------------------------------------------------------------

#ifndef MyWhiteningH
#define MyWhiteningH
//---------------------------------------------------------------------------

/* aaaaa */
void getCovariance(	double **src, double **dst,	int size, int dim );

void getWhiteningTransform(double **src, double **transMat, int size, int dim, int zero_mean);
void doWhitenPatch(double **src, double **dst, double** transMat, int n, int dim);

void myWhitening(double **src, double** dst, int size, int dim );
void myWhitening2(double **src, double** dst, int size, int dim );
void myWhitening3(double **src, double** dst, double** transMat, int size, int dim );

#endif
