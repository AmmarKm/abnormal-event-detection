//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "MyFunction.h"
#include "Unit1.h"

//---------------------------------------------------------------------------

#pragma package(smart_init)

void output(String fname, double **src, int row, int col, int type=1){

	String s, space;
	TStringList *list = new TStringList;
	list->Clear();

	switch(type){
		case 1:
			space = " ";
			break;
		case 2:
			space = "\t";
			break;
		default:
			space = " ";
			break;
	}

	int zero_count = 0;

	for(int i=0;i<row;i++){		//bk_size = 8, i = 0 to 7
		s = "";					//字串清空以供輸出
		s = s + src[i][0];		//放入第i行的第一筆數字

		if(src[i][0]==0) zero_count++;

		for(int j=1;j<col;j++){			//放入剩餘數字, j = 1 to 7
			s = s + space + src[i][j];	//以 'tab' 為間格將數字串連成同一行
			if(src[i][j]==0) zero_count++;

		}

		list->Add(s);
	}

	list->SaveToFile(fname);
//	Form1->Memo2->Lines->Add("Zero_count:");
//	Form1->Memo2->Lines->Add(zero_count);


	delete list;
}

// 1D ARRAY
void myCreateArray(double *(*array), int rows){

//	if (*array != NULL) {
//		myDeleteArray2D(array, rows);
//	}
	(*array) = new double[rows];
	for(int i=0;i<rows;i++){
		(*array)[i] = 0;
	}
}
void myCreateArray(int *(*array), int rows){

//	if (*array != NULL) {
//		myDeleteArray2D(array, rows);
//	}
	(*array) = new int[rows];
	for(int i=0;i<rows;i++){
		(*array)[i] = 0;
	}
}
void myDeleteArray(double *(*array)){
	if (*array == NULL)
		return;

	delete[] *array;
	*array = NULL;
}
void myDeleteArray(int *(*array)){
	if (*array == NULL)
		return;

	delete[] *array;
	*array = NULL;
}

// 2D ARRAY
void myCopyArray2D(double **src, double **dst, int rows, int cols){

	for(int i=0;i<rows;i++){
		for(int j=0;j<cols;j++){
			dst[i][j] = src[i][j];
		}
	}

}
void myCopyArray2D(int **src, int **dst, int rows, int cols){

	for(int i=0;i<rows;i++){
		for(int j=0;j<cols;j++){
			dst[i][j] = src[i][j];
		}
	}

}
void myCreateArray2D(double **(*array), int rows, int cols){

//	if (*array != NULL) {
//		myDeleteArray2D(array, rows);
//	}
	(*array) = new double*[rows];
	for(int i=0;i<rows;i++){
		(*array)[i] = new double[cols];
		for(int j=0;j<cols;j++){
			(*array)[i][j] = 0;
		}
	}

}
void myCreateArray2D(int **(*array), int rows, int cols){      //int
//	if (*array != NULL) {
//		myDeleteArray2D(array, rows);
//	}
	(*array) = new int*[rows];
	for(int i=0;i<rows;i++){
		(*array)[i] = new int[cols];
		for(int j=0;j<cols;j++){
			(*array)[i][j] = 0;
		}
	}

}
void myDeleteArray2D(double **(*array), int rows){             //double
	if (*array == NULL)
		return;

	for(int i=0;i<rows;i++){
		if ((*array)[i] == NULL)
			continue;

		delete[] (*array)[i];
		(*array)[i] = NULL;
	}
	delete[] *array;
	*array = NULL;
}
void myDeleteArray2D(int **(*array), int rows){             //double
	if (*array == NULL)
		return;

	for(int i=0;i<rows;i++){
		if ((*array)[i] == NULL)
			continue;

		delete[] (*array)[i];
		(*array)[i] = NULL;
	}
	delete[] *array;
	*array = NULL;
}

void myZerosArray(double *arr, int length){
	for(int i=0;i<length;i++)
		arr[i] = 0;
}
void myZerosArray(int *arr, int length){
	for(int i=0;i<length;i++)
		arr[i] = 0;
}

void myZerosArray2D(double **arr, int rows, int cols){
	for(int i=0;i<rows;i++)
		for(int j=0;j<cols;j++)
			arr[i][j] = 0;
}
void myZerosArray2D(int **arr, int rows, int cols){
	for(int i=0;i<rows;i++)
		for(int j=0;j<cols;j++)
			arr[i][j] = 0;
}

