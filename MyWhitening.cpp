//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "MyWhitening.h"
#include "cxcore.h"
#include "math.h"
#include "Unit1.h"

#include "MyFunction.h"
#include "MyMath.h"

//---------------------------------------------------------------------------

#pragma package(smart_init)

/* return the covariance matrix of a matrix which is (size x dim) */
//This function did'nt include zero-mean process,
//if needed, you have to do zero-mean before calling the function.
void getCovariance(
	double **src,	//input data, n by dim; n is numbers of data
	double **dst,   //the covariance matrix for output
	int size,       //data size, more than 0
	int dim			//data dimension, more than 0
	){

	if (size <= 0 || dim <= 0 )
		return;

	double sum = 0;

	for(int i=0;i<dim;i++){                // The size of covariance matrix is dim x dim
		for(int j=0;j<dim;j++){
			sum = 0;
			for(int k=0;k<size;k++){
				sum = sum + (src[k][i])*(src[k][j]);
			}
			dst[i][j] = sum/size;
		}
	}
}

/* Whitening process, using OpenCV to solve eigen vector and eigen value */
void myWhitening(double **src, double** dst, int size, int dim ){

	if( size < dim )	// this process is use to solve a matrix which data size is more than dimension
		return;
	if (size <= 0 || dim <= 0 )
		return;

	int n = size ;
	double **pa = src;
	double **wb = dst;                         //whitened patch
	double **pb = new double*[dim];			   //pb is covariance of pa
	double **w_trans_tmp = new double*[dim];   //be used to calculate w_trans
	double **w_trans = new double*[dim];       //whitening transform

	for(int i=0;i<dim;i++){
		pb[i] = new double[dim];
		w_trans_tmp[i] = new double[dim];
		w_trans[i] = new double[dim];
	}

	/* step1, zero-mean */
//	Form1->Memo1->Lines->Add("step1") ;
	double *cov_mean = new double[dim];
//	for(int i=0;i<dim;i++)
//		cov_mean[i] = 0;
//	for(int i=0;i<dim;i++){
//		for(int j=0;j<n;j++){
//			cov_mean[i] += pa[j][i];
//		}
//		cov_mean[i] /= n;		// n > 0
//
//		for(int j=0;j<n;j++){
//			pa[j][i] -= cov_mean[i];
//		}
//	}

	/* step2, CALCULATE COVARIANCE MATRIX */
	Form1->Memo1->Lines->Add("step2")  ;
	getCovariance(pa,pb,n,dim);

	output("cov.txt", pb, dim, dim, 2);

	//return;

	/* step3 */
	//% DETERMINE EIGENECTORS & EIGENVALUES
	//% OF COVARIANCE MATRIX
	Form1->Memo1->Lines->Add("step3")  ;

	//1D datapoints into CvMat
	double *cov_1D = new double[dim*dim];
	int k = 0;
	for(int i=0;i<dim;i++){
		for(int j=0;j<dim;j++){
			cov_1D[k++] = pb[i][j];


//			Memo1->Lines->Add(pb[i][j]);
//			Memo2->Lines->Add(cov_1D[k-1]);
		}
	}
	Form1->Memo1->Lines->Add("step3-1")  ;
	CvMat* mat = cvCreateMat(dim,dim,CV_64FC1); //eigenvectors
	cvSetData(mat,cov_1D,mat->step);
//	CvMat mat = cvMat(dim,dim,CV_64FC1, cov_1D);	//double -> CV_64F

	//Form1->Memo2->Lines->Clear();
//	for(int i=0;i<dim;i++)
//		for(int j=0;j<dim;j++)
//			Form1->Memo2->Lines->Add(mat->data.db[i*dim+j]);



	#define FLT_EPSILON 1.19209290E-07F
	#define DBL_EPSILON 2.2204460492503131E-16
//	#define DBL_EPSILON 2.2204460492503131E-10


	Form1->Memo1->Lines->Add("step3-2")  ;
	/* Eigenvalue specific code begins */
	CvScalar scal;
	CvMat* eig_vec = cvCreateMat(dim,dim,CV_64FC1); //eigenvectors
	CvMat* eig_val = cvCreateMat(1,dim,CV_64FC1);   //eigenvalues (1xN)
	cvZero(eig_vec);
	cvZero(eig_val);
	Form1->Memo1->Lines->Add("step3-3")  ;
//	cvEigenVV(mat, eig_vec, eig_val, 1, -1, -1);
	cvEigenVV(mat, eig_vec, eig_val, DBL_EPSILON);
	Form1->Memo1->Lines->Add("step3-4")  ;


	/* step4 */
	//% CALCULATE D^(-1/2)
	//  d = diag(D2);
	//	d = real(d.^-.5);
	//	WD2 = diag(d);
	Form1->Memo1->Lines->Add("step4")   ;

	double *vec_data = eig_vec->data.db;
	double *val_data = eig_val->data.db;

	for(int i=0;i<eig_val->cols;i++){
		//Memo1->Lines->Add(eig_val->data.db[i]);
		double tmp = val_data[i];
		if(tmp>0)
			tmp = 1.0/sqrt(tmp);
		else
			tmp = 0;
		eig_val->data.db[i] = tmp;
	}

	/* step5 */
	Form1->Memo1->Lines->Add("step5")   ;
	//% CALCULATE WHITENING TRANSFORM
	//W2 = E2*WD2*E2';
	for(int i=0;i<eig_vec->rows;i++){			//here, rows == cols == dim
		for(int j=0;j<eig_vec->cols;j++){
			w_trans_tmp[i][j] = vec_data[i*dim+j]*val_data[i];
			//Memo1->Lines->Add(w_trans_tmp[i][j]);
		}
	}
	for(int i=0;i<eig_vec->rows;i++){
		for(int j=0;j<eig_vec->cols;j++){
			double sum = 0;
			for(int k=0;k<eig_vec->cols;k++){
				sum += w_trans_tmp[k][i] * vec_data[k*dim+j];
			}
			w_trans[i][j] = sum;
//			Form1->Memo1->Lines->Add(w_trans[i][j]);
		}
	}

	/* step6 */
	Form1->Memo1->Lines->Add("step6")   ;
	//% WHITEN THE PATCHES
	//wb = W2*b;					//因為W2 是dim by dim, b是 n by dim
									//所以實際運算為 b*W2' 使得wb是 n by dim
	for(int i=0;i<n;i++){
		for(int j=0;j<dim;j++){
			double sum = 0;
			for(int k=0;k<dim;k++){
				sum += pa[i][k] * w_trans[j][k];
			}
			wb[i][j] = sum;
//			Form1->Memo1->Lines->Add(wb[i][j]);
		}
	}

	/***** end whitening process ******/


	//zero-mean back
//	for(int i=0;i<n;i++){
//		for(int j=0;j<dim;j++){
//			wb[i][j] += cov_mean[j];
//		}
//	}

	//test for checking covariance after whitening
//	for(int i=0;i<dim;i++){
//		for(int j=0;j<dim;j++){
//			int tmp = 0;
//			double sum = 0;
//			for(int k=0;k<n;k++){
//				sum += wb[k][i] * wb[k][j] / n;
//			}
//			if( sum != 0){
//				double tmp_ratio = pow(10,5); //overflow if power over by 8
//				tmp = sum * tmp_ratio +0.5 ;
//				sum = (tmp / tmp_ratio );
//			}
//			pb[i][j] = sum;
//
//			Memo2->Lines->Add(pb[i][j]);
//		}
//	}


    cvReleaseMat(&mat);
	cvReleaseMat(&eig_vec); //house-cleaning :P
	cvReleaseMat(&eig_val);

	delete[] cov_mean;
	delete[] cov_1D;

	for(int i=0;i<dim;i++){
		delete[] pb[i];
		delete[] w_trans[i];
		delete[] w_trans_tmp[i];
	}
	delete[] pb;
	delete[] w_trans;
	delete[] w_trans_tmp;
	//================================

}

void myWhitening2(double **src, double** dst, int size, int dim ){

	if( size < dim )	// this process is use to solve a matrix which data size is more than dimension
		return;
	if (size <= 0 || dim <= 0 )
		return;

	int n = size ;
	double **pa = src;
	double **wb = dst;                         //whitened patch
	double **pb = new double*[dim];			   //pb is covariance of pa
	double **w_trans_tmp = new double*[dim];   //be used to calculate w_trans
	double **w_trans = new double*[dim];       //whitening transform

	for(int i=0;i<dim;i++){
		pb[i] = new double[dim];
		w_trans_tmp[i] = new double[dim];
		w_trans[i] = new double[dim];
	}

	/* step1, zero-mean */
//	Form1->Memo1->Lines->Add("step1") ;
	double *cov_mean = new double[dim];
//	for(int i=0;i<dim;i++)
//		cov_mean[i] = 0;
//	for(int i=0;i<dim;i++){
//		for(int j=0;j<n;j++){
//			cov_mean[i] += pa[j][i];
//		}
//		cov_mean[i] /= n;		// n > 0
//
//		for(int j=0;j<n;j++){
//			pa[j][i] -= cov_mean[i];
//		}
//	}

	/* step2, CALCULATE COVARIANCE MATRIX */
	Form1->Memo1->Lines->Add("step2")  ;
	getCovariance(pa,pb,n,dim);

	output("cov.txt", pb, dim, dim, 2);

	//return;

	/* step3 */
	//% DETERMINE EIGENECTORS & EIGENVALUES
	//% OF COVARIANCE MATRIX
	Form1->Memo1->Lines->Add("step3")  ;

	//1D datapoints into CvMat
	double *cov_1D = new double[dim*dim];
	int k = 0;
	for(int i=0;i<dim;i++){
		for(int j=0;j<dim;j++){
			cov_1D[k++] = pb[i][j];


//			Memo1->Lines->Add(pb[i][j]);
//			Memo2->Lines->Add(cov_1D[k-1]);
		}
	}
	Form1->Memo1->Lines->Add("step3-1")  ;
	CvMat* mat = cvCreateMat(dim,dim,CV_64FC1); //eigenvectors
	cvSetData(mat,cov_1D,mat->step);
//	CvMat mat = cvMat(dim,dim,CV_64FC1, cov_1D);	//double -> CV_64F

	//Form1->Memo2->Lines->Clear();
//	for(int i=0;i<dim;i++)
//		for(int j=0;j<dim;j++)
//			Form1->Memo2->Lines->Add(mat->data.db[i*dim+j]);



	#define FLT_EPSILON 1.19209290E-07F
	#define DBL_EPSILON 2.2204460492503131E-16
//	#define DBL_EPSILON 2.2204460492503131E-10


	Form1->Memo1->Lines->Add("step3-2")  ;
	/* Eigenvalue specific code begins */
	CvScalar scal;
	CvMat* eig_vec = cvCreateMat(dim,dim,CV_64FC1); //eigenvectors
	CvMat* eig_val = cvCreateMat(1,dim,CV_64FC1);   //eigenvalues (1xN)
	cvZero(eig_vec);
	cvZero(eig_val);
	Form1->Memo1->Lines->Add("step3-3")  ;
//	cvEigenVV(mat, eig_vec, eig_val, 1, -1, -1);
	cvEigenVV(mat, eig_vec, eig_val, DBL_EPSILON);
	Form1->Memo1->Lines->Add("step3-4")  ;


	/* step4 */
	//% CALCULATE D^(-1/2)
	//  d = diag(D2);
	//	d = real(d.^-.5);
	//	WD2 = diag(d);
	Form1->Memo1->Lines->Add("step4")   ;

	double *vec_data = eig_vec->data.db;
	double *val_data = eig_val->data.db;

	for(int i=0;i<eig_val->cols;i++){
		//Memo1->Lines->Add(eig_val->data.db[i]);
		double tmp = val_data[i];
		if(tmp>0)
			tmp = 1.0/sqrt(tmp);
		else
			tmp = 0;
		eig_val->data.db[i] = tmp;
	}

	/* step5 */
	Form1->Memo1->Lines->Add("step5")   ;
	//% CALCULATE WHITENING TRANSFORM
	//W2 = E2*WD2*E2';
	for(int i=0;i<eig_vec->rows;i++){			//here, rows == cols == dim
		for(int j=0;j<eig_vec->cols;j++){
			w_trans_tmp[i][j] = vec_data[i*dim+j]*val_data[i];
			//Memo1->Lines->Add(w_trans_tmp[i][j]);
		}
	}
	for(int i=0;i<eig_vec->rows;i++){
		for(int j=0;j<eig_vec->cols;j++){
			double sum = 0;
			for(int k=0;k<eig_vec->cols;k++){
				sum += w_trans_tmp[k][i] * vec_data[k*dim+j];
			}
			w_trans[i][j] = sum;
//			Form1->Memo1->Lines->Add(w_trans[i][j]);
		}
	}

	/* step6 */
	Form1->Memo1->Lines->Add("step6")   ;
	//% WHITEN THE PATCHES
	//wb = W2*b;					//因為W2 是dim by dim, b是 n by dim
									//所以實際運算為 b*W2' 使得wb是 n by dim
	for(int i=0;i<n;i++){
		for(int j=0;j<dim;j++){
			double sum = 0;
			for(int k=0;k<dim;k++){
				sum += pa[i][k] * w_trans[j][k];
			}
			wb[i][j] = sum;
//			Form1->Memo1->Lines->Add(wb[i][j]);
		}
	}

	/***** end whitening process ******/


	//zero-mean back
//	for(int i=0;i<n;i++){
//		for(int j=0;j<dim;j++){
//			wb[i][j] += cov_mean[j];
//		}
//	}

	//test for checking covariance after whitening
//	for(int i=0;i<dim;i++){
//		for(int j=0;j<dim;j++){
//			int tmp = 0;
//			double sum = 0;
//			for(int k=0;k<n;k++){
//				sum += wb[k][i] * wb[k][j] / n;
//			}
//			if( sum != 0){
//				double tmp_ratio = pow(10,5); //overflow if power over by 8
//				tmp = sum * tmp_ratio +0.5 ;
//				sum = (tmp / tmp_ratio );
//			}
//			pb[i][j] = sum;
//
//			Memo2->Lines->Add(pb[i][j]);
//		}
//	}


    cvReleaseMat(&mat);
	cvReleaseMat(&eig_vec); //house-cleaning :P
	cvReleaseMat(&eig_val);

	delete[] cov_mean;
	delete[] cov_1D;

	for(int i=0;i<dim;i++){
		delete[] pb[i];
		delete[] w_trans[i];
		delete[] w_trans_tmp[i];
	}
	delete[] pb;
	delete[] w_trans;
	delete[] w_trans_tmp;
	//================================

}

void doZeroMean(double **src, int size, int dim){

	if (size < 1) {
		return;
	}

	double *mean = new double[dim];
	for(int i=0;i<dim;i++)
		mean[i] = 0;

	//caculate mean then minus mean
	int n = size;
	for(int i=0;i<dim;i++){
		for(int j=0;j<n;j++){
			mean[i] += src[j][i];
		}
		mean[i] /= n;		// n > 0
		for(int j=0;j<n;j++){
			src[j][i] = src[j][i] - mean[i];
		}
	}

	delete[] mean;
}

void getWhiteningTransform(double **src, double **transMat, int size, int dim, int zero_mean=0){

	double** mydata = src;
	int n = size;

	// dim x dim
	double **cov = new double*[dim]; 		//covariance of mydata
	double **vec = new double*[dim];		//eigen vector
	double *val = new double[dim];			//eigen value
	double **w_trans = transMat; 			//transform matrix as output
	double **w_trans_tmp = new double*[dim];
	for(int i=0;i<dim;i++){
		cov[i] = new double[dim];
		vec[i] = new double[dim];
		w_trans_tmp[i] = new double[dim];
	}
	int *nrot = new int;


	/** To start Whitening processing **/

	//% do Zero-mean if needed
	if(zero_mean == 1){
		doZeroMean(mydata, n, dim);
	}

	//% CALCULATE COVARIANCE MATRIX
	getCovariance(mydata, cov, n, dim);
//	output("cov.txt", cov, dim, dim, 2);

	//% DETERMINE EIGENECTORS & EIGENVALUES
	//% OF COVARIANCE MATRIX
	jacobi3(cov, dim, val, vec, nrot);             //solve eigen of covariance matrix
	eigsrt(val, vec, dim);
//	output("vec.txt", vec, dim, dim, 2);

	//% step4
	//% CALCULATE D^(-1/2)
	//  d = diag(D);
	//	d = real(d.^-.5);
	//	WD = diag(d);
	for(int i=0;i<dim;i++){
		double tmp = val[i];
		if(tmp>0)
			tmp = 1.0/sqrt(tmp);
		else
			tmp = 0;
		val[i] = tmp;
	}

	//% step5
	//% CALCULATE WHITENING TRANSFORM
	//W = E*WD*E';
	for(int i=0;i<dim;i++){
		for(int j=0;j<dim;j++){
			w_trans_tmp[i][j] = vec[i][j]*val[j];
		}
	}
	for(int i=0;i<dim;i++){
		for(int j=0;j<dim;j++){
			double sum = 0;
			for(int k=0;k<dim;k++){
				sum += w_trans_tmp[i][k] * vec[j][k];
			}
			w_trans[i][j] = sum;
		}
	}

	for(int i=0;i<dim;i++){
		delete[] cov[i];
		delete[] vec[i];
		delete[] w_trans_tmp[i];
	}
	delete[] cov;
	delete[] vec;
	delete[] val;
	delete[] w_trans_tmp;

	delete nrot;
}

// transform data using transMat
void doWhitenPatch(double **src, double **dst, double** transMat, int n, int dim){

	/* step6 */
	//% WHITEN THE PATCHES
	//wb = W2*b;					//因為W2 是dim by dim, b是 n by dim
									//所以實際運算為 b*W2' 使得wb是 n by dim
	double **mydata = src;
	double **wb = new double*[n];
	for(int i=0;i<n;i++){
		wb[i] = new double[dim];
	}

	for(int i=0;i<n;i++){
		for(int j=0;j<dim;j++){
			double sum = 0;
			for(int k=0;k<dim;k++){
				sum += mydata[i][k] * transMat[j][k];
			}
			wb[i][j] = sum;
		}
	}

	myCopyArray2D(wb, dst, n, dim);


	for(int i=0;i<n;i++){
		delete[] wb[i];
	}
	delete[] wb;

}

void myWhitening3(double **src, double** dst, double** transMat, int size, int dim){


	double** mydata = src;
	String s = "";

	// n x dim
	int n = size;

	/** start **/
	getWhiteningTransform(mydata, transMat, n, dim);		//get transform matrix as "transMat"
	output("w_trans2.txt", transMat, dim, dim, 2);

	/* step6 */
	Form1->Memo1->Lines->Add("step6")   ;
	//% WHITEN THE PATCHES
	//wb = W2*b;					//因為W2 是dim by dim, b是 n by dim
									//所以實際運算為 b*W2' 使得wb是 n by dim
   	doWhitenPatch(mydata, dst, transMat, n, dim);
//	output("wb2.txt", wb, n, dim, 2);

	//=========== test /////////////

//	//test for covariance
//	for(int i=0;i<dim;i++){
//		for(int j=0;j<dim;j++){
//			int tmp = 0;
//			double sum = 0;
//			for(int k=0;k<n;k++){
//				sum += wb[k][i] * wb[k][j] / n;
//			}
//			if( sum != 0){
//				double tmp_ratio = pow(10,5); //overflow if power over by 8
//				tmp = sum * tmp_ratio +0.5 ;
//				sum = (tmp / tmp_ratio );
//			}
//			cov_wb[i][j] = sum;
//
////			Memo2->Lines->Add(cov_wb[i][j]);
//		}
//	}
//	output("cov_wb.txt", cov_wb, dim, dim, 2);


//	for(int i=0;i<n;i++){
//		delete[] wb[i];
//	}
//	delete[] wb;

}
