//---------------------------------------------------------------------------

#ifndef MyFunctionH
#define MyFunctionH
//---------------------------------------------------------------------------


void output(String fname, double **src, int row, int col, int type);

void myCreateArray(double *(*array), int rows);
void myCreateArray(int *(*array), int rows);

void myCreateArray2D(double ***array, int rows, int cols);
void myCreateArray2D(int ***array, int rows, int cols);

void myDeleteArray(double *(*array));
void myDeleteArray(int *(*array));

void myDeleteArray2D(double ***array, int rows);
void myDeleteArray2D(int ***array, int rows);

void myCopyArray2D(double **src, double **dst, int rows, int cols);
void myCopyArray2D(int **src, int **dst, int rows, int cols);

void myZerosArray(double *arr, int length);
void myZerosArray(int *arr, int length);

void myZerosArray2D(double **arr, int rows, int cols);
void myZerosArray2D(int **arr, int rows, int cols);

#endif
