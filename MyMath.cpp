//---------------------------------------------------------------------------


#pragma hdrstop

#include "MyMath.h"
#include <math.h>

//---------------------------------------------------------------------------

#pragma package(smart_init)


//#include "nrutil.h"

#define ROTATE(a,i,j,k,l) g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);\
	a[k][l]=h+s*(g-h*tau);

/*  Computes all eigenvalues and eigenvectors of a real symmetric matrix a[1..n][1..n]. On
	output, elements of a above the diagonal are destroyed. d[1..n] returns the eigenvalues of a.
	v[1..n][1..n] is a matrix whose columns contain, on output, the normalized eigenvectors of
	a. nrot returns the number of Jacobi rotations that were required. */

void jacobi0(double **a, int n, double d[], double **v, int *nrot)    // n = dim
{
	int j,iq,ip,i;
	float tresh,theta,tau,t,sm,s,h,g,c,*b,*z;
//	b=vector(1,n);
//	z=vector(1,n);
	b = new float[n+1];
	z = new float[n+1];
	for (ip=1;ip<=n;ip++) {
		for (iq=1;iq<=n;iq++)
			v[ip][iq]=0.0;
		v[ip][ip]=1.0;
	}
	for (ip=1;ip<=n;ip++) {
		b[ip]=d[ip]=a[ip][ip];
		z[ip]=0.0;
	}
	*nrot=0;
	for (i=1;i<=50;i++) {
		sm=0.0;
		for (ip=1;ip<=n-1;ip++) {
			for (iq=ip+1;iq<=n;iq++)
				sm += fabs(a[ip][iq]);
		}
		if (sm == 0.0) {
//			free_vector(z,1,n);
//			free_vector(b,1,n);
			delete[] z;
			delete[] b;
			return;
		}
	if (i < 4)
		tresh=0.2*sm/(n*n);
	else
		tresh=0.0;
	for (ip=1;ip<=n-1;ip++) {
	for (iq=ip+1;iq<=n;iq++) {
		g=100.0*fabs(a[ip][iq]);
		//After four sweeps, skip the rotation if the o-diagonal element is small.
		if (i > 4 && (float)(fabs(d[ip])+g) == (float)fabs(d[ip])
				&& (float)(fabs(d[iq])+g) == (float)fabs(d[iq])){
			a[ip][iq]=0.0;
		}
		else if (fabs(a[ip][iq]) > tresh) {
			h=d[iq]-d[ip];
			if ((float)(fabs(h)+g) == (float)fabs(h))
				t=(a[ip][iq])/h;
			else {
				theta=0.5*h/(a[ip][iq]);
				t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
				if (theta < 0.0)
					t = -t;
			}
			c=1.0/sqrt(1+t*t);
			s=t*c;
			tau=s/(1.0+c);
			h=t*a[ip][iq];
			z[ip] -= h;
			z[iq] += h;
			d[ip] -= h;
			d[iq] += h;
			a[ip][iq]=0.0;
			for (j=1;j<=ip-1;j++) {
				ROTATE(a,j,ip,j,iq)
			}
			for (j=ip+1;j<=iq-1;j++) {
				ROTATE(a,ip,j,j,iq)
			}
			for (j=iq+1;j<=n;j++) {
				ROTATE(a,ip,j,iq,j)
			}
			for (j=1;j<=n;j++) {
				ROTATE(v,j,ip,j,iq)
			}
			++(*nrot);
		}
	}
	}
	for (ip=1;ip<=n;ip++) {
		b[ip] += z[ip];
		d[ip]=b[ip];
		z[ip]=0.0;
	}
}
//nrerror("Too many iterations in routine jacobi");

}


void jacobi1(double **a, int n, double d[], double **v, int *nrot)
{
	int j,iq,ip,i;
	double tresh,theta,tau,t,sm,s,h,g,c,*b,*z;
	//b=vector(1,n);
	//z=vector(1,n);
	b = new double[n];
	z = new double[n];

	for (ip=0;ip<n;ip++) { 		//Initialize to the identity matrix.
		for (iq=0;iq<n;iq++)
			v[ip][iq]=0.0;
		v[ip][ip]=1.0;
	}
	for (ip=0;ip<n;ip++) { 		//Initialize b and d to the diagonal
		b[ip]=d[ip]=a[ip][ip];		// of a.
		z[ip]=0.0;					// This vector will accumulate terms
									//of the form tapq as in equation
									//(11.1.14).
	}
	*nrot=0;
	for (i=1;i<=50;i++) {
		sm=0.0;
		for (ip=0;ip<n-1;ip++) {	// Sum o-diagonal elements.
		for (iq=ip+1;iq<n;iq++)
			sm += fabs(a[ip][iq]);
		}
		if (sm == 0.0) {				// The normal return, which relies
										//on quadratic convergence to
										//machine underflow.
			//free_vector(z,1,n);
			//free_vector(b,1,n);
			delete[] z;
			delete[] b;
			return;
		}
		if (i < 4)
			tresh=0.2*sm/(n*n); 		//...on the rst three sweeps.
		else
			tresh=0.0; 					//...thereafter.

		for (ip=0;ip<n-1;ip++) {
			for (iq=ip+1;iq<n;iq++) {
				g=100.0*fabs(a[ip][iq]);
				//After four sweeps, skip the rotation if the o-diagonal element is small.
				if (i > 4 && (double)(fabs(d[ip])+g) == (double)fabs(d[ip])
					&& (double)(fabs(d[iq])+g) == (double)fabs(d[iq]))
					a[ip][iq]=0.0;
				else if (fabs(a[ip][iq]) > tresh) {
					h=d[iq]-d[ip];
					if ((double)(fabs(h)+g) == (double)fabs(h))
						t=(a[ip][iq])/h;	 		//t = 1=(2*theta)
					else {
						theta=0.5*h/(a[ip][iq]); 	//Equation (11.1.10).
						t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
						if (theta < 0.0) t = -t;
					}
					c=1.0/sqrt(1+t*t);
					s=t*c;
					tau=s/(1.0+c);
					h=t*a[ip][iq];
					z[ip] -= h;
					z[iq] += h;
					d[ip] -= h;
					d[iq] += h;
					a[ip][iq]=0.0;
					for (j=0;j<ip-1;j++) { 		//Case of rotations 1  j < p.
						ROTATE(a,j,ip,j,iq)
					}
					for (j=ip+1;j<=iq-1;j++) { 		//Case of rotations p < j < q.
						ROTATE(a,ip,j,j,iq)
					}
					for (j=iq+1;j<n;j++) { 		//Case of rotations q < j n.
						ROTATE(a,ip,j,iq,j)
					}
					for (j=0;j<n;j++) {
						ROTATE(v,j,ip,j,iq)
					}
					++(*nrot);
				}
			}
		}
		for (ip=0;ip<n;ip++) {
			b[ip] += z[ip];
			d[ip]=b[ip]; 						//Update d with the sum of tapq,
			z[ip]=0.0; 							//and reinitialize z.
		}
	}
}





void jacobi2(double **a, int n, double d[], double **v, int *nrot)    // n = dim
{
	int j,iq,ip,i;
	double tresh,theta,tau,t,sm,s,h,g,c,*b,*z;
	//b=vector(1,n);
	//z=vector(1,n);
	b = new double[n+1];
	z = new double[n+1];

	for (ip=0;ip<=n;ip++) { 		//Initialize to the identity matrix.
		for (iq=0;iq<=n;iq++)
			v[ip][iq]=0.0;
		v[ip][ip]=1.0;
	}
	for (ip=1;ip<=n;ip++) { 		//Initialize b and d to the diagonal
		b[ip]=d[ip]=a[ip][ip];		// of a.
		z[ip]=0.0;					// This vector will accumulate terms
									//of the form tapq as in equation
									//(11.1.14).
	}
	*nrot=0;
	for (i=1;i<=50;i++) {
		sm=0.0;
		for (ip=1;ip<=n-1;ip++) {	// Sum o-diagonal elements.
		for (iq=ip+1;iq<=n;iq++)
			sm += fabs(a[ip][iq]);
		}
		if (sm == 0.0) {				// The normal return, which relies
										//on quadratic convergence to
										//machine underflow.
			//free_vector(z,1,n);
			//free_vector(b,1,n);
			delete[] z;
			delete[] b;
			return;
		}
		if (i < 4)
			tresh=0.2*sm/(n*n); 		//...on the rst three sweeps.
		else
			tresh=0.0; 					//...thereafter.

		for (ip=1;ip<=n-1;ip++) {
			for (iq=ip+1;iq<=n;iq++) {
				g=100.0*fabs(a[ip][iq]);
				//After four sweeps, skip the rotation if the o-diagonal element is small.
				if (i > 4 && (double)(fabs(d[ip])+g) == (double)fabs(d[ip])
					&& (double)(fabs(d[iq])+g) == (double)fabs(d[iq]))
					a[ip][iq]=0.0;
				else if (fabs(a[ip][iq]) > tresh) {
					h=d[iq]-d[ip];
					if ((double)(fabs(h)+g) == (double)fabs(h))
						t=(a[ip][iq])/h;	 		//t = 1=(2*theta)
					else {
						theta=0.5*h/(a[ip][iq]); 	//Equation (11.1.10).
						t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
						if (theta < 0.0) t = -t;
					}
					c=1.0/sqrt(1+t*t);
					s=t*c;
					tau=s/(1.0+c);
					h=t*a[ip][iq];
					z[ip] -= h;
					z[iq] += h;
					d[ip] -= h;
					d[iq] += h;
					a[ip][iq]=0.0;
					for (j=1;j<=ip-1;j++) { 		//Case of rotations 1  j < p.
						ROTATE(a,j,ip,j,iq)
					}
					for (j=ip+1;j<=iq-1;j++) { 		//Case of rotations p < j < q.
						ROTATE(a,ip,j,j,iq)
					}
					for (j=iq+1;j<=n;j++) { 		//Case of rotations q < j n.
						ROTATE(a,ip,j,iq,j)
					}
					for (j=1;j<=n;j++) {
						ROTATE(v,j,ip,j,iq)
					}
					++(*nrot);
				}
			}
		}
		for (ip=1;ip<=n;ip++) {
			b[ip] += z[ip];
			d[ip]=b[ip]; 						//Update d with the sum of tapq,
			z[ip]=0.0; 							//and reinitialize z.
		}
	}

//	nrerror("Too many iterations in routine jacobi");

}

void jacobi3(double **a, int n, double d[], double **v, int *nrot)    // n = dim
{
	int j,iq,ip,i;
	double tresh,theta,tau,t,sm,s,h,g,c,*b,*z;
//	b=vector(1,n);
//	z=vector(1,n);

	b = new double[n];
	z = new double[n];

	//initial
	for (ip=0;ip<n;ip++) {
		for (iq=0;iq<n;iq++){
			v[ip][iq]=0.0;
		}
		v[ip][ip]=1.0;
	}
	for (ip=0;ip<n;ip++) {
		b[ip]=d[ip]=a[ip][ip];
		z[ip]=0.0;
	}
	*nrot=0;


	for (i=1;i<=50;i++) {
		sm=0.0;
		for (ip=0;ip<n-1;ip++) {
			for (iq=ip+1;iq<n;iq++){
				sm += fabs(a[ip][iq]);
			}
		}
		if (sm == 0.0) {
			delete[] z;
			delete[] b;
//			free_vector(z,1,n);
//			free_vector(b,1,n);
			return;
		}
		if (i < 4)
			tresh=0.2*sm/(n*n);
		else
			tresh=0.0;

		for (ip=0;ip<n-1;ip++) {
			for (iq=ip+1;iq<n;iq++) {
				g=100.0*fabs(a[ip][iq]);
				//After four sweeps, skip the rotation if the o-diagonal element is small.
				if (i > 4 && (double)(fabs(d[ip])+g) == (double)fabs(d[ip])
						&& (double)(fabs(d[iq])+g) == (double)fabs(d[iq])){
					a[ip][iq]=0.0;
				}
				else if (fabs(a[ip][iq]) > tresh) {
					h=d[iq]-d[ip];
					if ((double)(fabs(h)+g) == (double)fabs(h))
						t=(a[ip][iq])/h;
					else {
						theta=0.5*h/(a[ip][iq]);
						t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
						if (theta < 0.0)
							t = -t;
					}
					c=1.0/sqrt(1+t*t);
					s=t*c;
					tau=s/(1.0+c);
					h=t*a[ip][iq];
					z[ip] -= h;
					z[iq] += h;
					d[ip] -= h;
					d[iq] += h;
					a[ip][iq]=0.0;
					for (j=0;j<=ip-1;j++) {
						ROTATE(a,j,ip,j,iq)
					}
					for (j=ip+1;j<=iq-1;j++) {
						ROTATE(a,ip,j,j,iq)
					}
					for (j=iq+1;j<n;j++) {
						ROTATE(a,ip,j,iq,j)
					}
					for (j=0;j<n;j++) {
						ROTATE(v,j,ip,j,iq)
					}
					++(*nrot);
				}
			}
		}
		for (ip=0;ip<n;ip++) {
			b[ip] += z[ip];
			d[ip]=b[ip];
			z[ip]=0.0;
		}
	}
	//nrerror("Too many iterations in routine jacobi");

}


/*	Note that the above routine assumes that underflows are set to zero. On
	machines where this is not true, the program must be modified.
	The eigenvalues are not ordered on output. If sorting is desired, the following
	routine can be invoked to reorder the output of jacobi or of later routines in this
	chapter. (The method, straight insertion, is N2 rather than N logN; but since you
	have just done an N3 procedure to get the eigenvalues, you can afford yourself
	this little indulgence.)
*/

/*	Given the eigenvalues d[1..n] and eigenvectors v[1..n][1..n] as output from jacobi
	(x11.1) or tqli (x11.3), this routine sorts the eigenvalues into descending order, and rearranges
	the columns of v correspondingly. The method is straight insertion.
	*/
void eigsrt(double d[], double **v, int n){
	int k,j,i;
	float p;

	for (i=0;i<n-1;i++) {
		p=d[k=i];
		for (j=i+1;j<n;j++)
			if (d[j] >= p) p=d[k=j];
		if (k != i) {
			d[k]=d[i];
			d[i]=p;
			for (j=0;j<n;j++) {
				p=v[j][i];
				v[j][i]=v[j][k];
				v[j][k]=p;
			}
		}
	}
}

int argmin(double *dist, int length, double *value){

	double min;
	int index = 0;

	if (length < 0) {
		return -1;
	}
	else if(length == 0) {
		if(value!=0)
			*value = dist[0];
		return 0;
	}
	else{
		min = dist[0];
		for(int i=1;i<length;i++){
			if(dist[i] < min){
				min = dist[i];
				index = i;
			}
		}

		//return result
		if(value!=0)
			*value = dist[index];
		return index;
	}

}

//return -1 = successful, -2 = error
//other = same distribution(max = min)
//int Normalize(double *arr, int length, double newMin=0, double newMax=1);
int Normalize(double *arr, int length, double newMin, double newMax){
	if (length < 0) {
		return -2;
	}

	double max = arr[0];
	double min = arr[0];

	for(int i=1;i<length;i++){     		//search max & min
		if(arr[i] > max)
			max = arr[i];
		if(arr[i] < min)
			min = arr[i];
	}

	//error
	if( max == 0 ){
		return 0;
	}
	else if( max == min ){
		for(int i=0;i<length;i++){
			arr[i] = newMax;
			return max;
		}
	}

	//ok
	for(int i=0;i<length;i++){
		arr[i] = (arr[i]-min)*(newMax-newMin)/(max-min) + newMin;
	}
	return -1;
}
