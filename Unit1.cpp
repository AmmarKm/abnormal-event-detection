//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "Unit1.h"

#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
//#include "cvaux.h"
//#include "cvcam.h"

#include <math.h>
#include <time.h>
#include <fstream>

//#include "FileCtrl.hpp"
//#include "Shlobj.h"
//#include "windowsx.h"

#include "MyParticle.h"
#include "MyMath.h"
#include "MyWhitening.h"
#include "MyFunction.h"

using namespace std;
//using namespace cv;


/* --Sparse Optical Flow Demo Program--
* Written by David Stavens (dstavens@robotics.stanford.edu)
*/
#include <stdio.h>
//#include <vector>
//#include <cv.h>
//#include <highgui.h>
//#include <math.h>

//#define SWAP(x,y) {int t; t = x; x = y; y = t;}

//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma link "Chart"
#pragma link "Series"
#pragma link "TeEngine"
#pragma link "TeeProcs"
#pragma resource "*.dfm"
TForm1 *Form1;

CvSize cvGetSize( IplImage *img)
{
	 CvSize aa;
	 aa.width=img->width;
	 aa.height=img->height;
	 return aa;
}

//use for qsort -> quicksort
int compare (const void * a, const void * b){
	return ( *(int*)a - *(int*)b );
}
int dcompare (const void * a, const void * b){
	// Returns -1 if x < y
	//          0 if x == y
	//          1 if x > y

	double dx, dy;
	dx = *(double *)a;
	dy = *(double *)b;

	if (dx < dy) {
		return -1;
	} else if (dx > dy) {
		return 1;
	}
	return 0;
}


//void detectAndDraw( CvMat& img,
//				   CascadeClassifier& cascade, CascadeClassifier& nestedCascade,
//	 			   double scale);

//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner)
	: TForm(Owner)
{
	srand(time(NULL));
//	FileListBox1->Directory = "D:\\dataset\\UMN\\new";

	if(CheckBox6->Checked == true){
		isImageSrc = true;
		FileListBox1->Mask = "*.jpg;*.bmp;*.tif";
	}
	else{
		isImageSrc = false;
		FileListBox1->Mask = "*.wmv;*.avi;*.mpg";
	}

	//是否手動輸入K
	Edit4->Enabled = CheckBox5->Checked;

}
//---------------------------------------------------------------------------

static const double pi = 3.14159265358979323846;
inline static double square(int a)
{
	return a * a;
}
/* This is just an inline that allocates images. I did this to reduce clutter in the
* actual computer vision algorithmic code. Basically it allocates the requested image
* unless that image is already non-NULL. It always leaves a non-NULL image as-is even
* if that image's size, depth, and/or channels are different than the request.
*/
inline static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels)
{
	if ( *img != NULL )
		return;

	*img = cvCreateImage( size, depth, channels );

	if ( *img == NULL )
	{
		fprintf(stderr, "Error: Couldn't allocate image. Out of memory?\n");
		exit(-1);
	}
}





int win_size = 10;
const int MAX_COUNT = 500;
CvPoint2D32f* points[2] = {0,0}, *swap_points;
char* status = 0;
int count = 0;
int flags;
int init=1;
//int win_size = 10;
//const int MAX_COUNT = 500;
//CvPoint2D32f* points[2] = {0,0}, *swap_points;
//char* status = 0;
//int count = 0;
int need_to_init = 0;
int night_mode = 0;
//int flags = 0;
int add_remove_pt = 0;
CvPoint pt;

IplImage *image = 0, *grey = 0, *prev_grey = 0, *pyramid = 0, *prev_pyramid = 0, *swap_temp;
void on_mouse( int event, int x, int y, int flags, void* param )
{
	if( !image )
        return;

	if( image->origin )
		y = image->height - y;

	if( event == CV_EVENT_LBUTTONDOWN )
	{
		pt = cvPoint(x,y);
		add_remove_pt = 1;
	}
}
const int MAX_CORNERS = 20000 ;

// 2013.3.26 更動光流計算
// 先將影像切分成數個網格 3x3
// 單獨計算 3x3 內所有pixel之光流
// 重覆以完成所有網格計算
// 如此減少需記錄的點數 => MAX_COUNT將只需9
//

double **soc_data;
int soc_data_count;
//TStringList *dataList;

//const int MAX_FRAMES = 2000;
static const String eventt_state[2] = {"normal", "abnormal"};



int trDim = 64;          //資料的維度

void __fastcall TForm1::Button8Click(TObject *Sender)
{
	clock_t start, end, start2, end2;
	String s = "";

	String initDir = ExtractFilePath(Application->ExeName);
//	String newDir = "training\\";
	String newDir = Edit8->Text + "\\";

	String filename =
		initDir +
//		"traffic-junction-sample.avi";
//		"tree.avi";
		 "Crowd-Activity-All.avi";

	if(FileListBox1->ItemIndex > -1){		//有從listbox1中選擇檔名
		filename = FileListBox1->FileName;
	}
	String dataName = ExtractFileName(filename);
	dataName = ChangeFileExt(dataName, ""); //將Name去掉副檔名
	newDir = newDir + dataName +"\\";

	CvCapture *cam = cvCaptureFromFile(		//建立影片來源
		filename.t_str()
	);

	Memo1->Lines->Add(filename);
	start2 = clock();

	//影片資訊
	double frameWidth, frameHeight, frameFPS, frameCount, frameCurrent;

	Label9->Caption  = frameWidth 	= cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_WIDTH);
	Label10->Caption = frameHeight  = cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_HEIGHT);
	Label11->Caption = frameFPS 	= cvGetCaptureProperty(cam, CV_CAP_PROP_FPS);
	Label12->Caption = frameCount 	= cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_COUNT);
	Label15->Caption = frameCurrent = cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES);
//	ShowMessage(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

	assert( NULL != cam ) ;

	IplImage *frame = 0, *img_prev = 0, *frame_cur = 0, *img_curr = 0, *img_res = 0,
		*img_soc = 0, *img_off = 0,
		*img_eig = 0, *img_temp = 0, *pyr_prev = 0, *pyr_cur = 0,
		*img_back = 0;

	IplImage **frames = 0, **images = 0;
	MyParticle2D** optical_flow = 0; 	    //計算粒子光流所暫存的影像pixel之光流
	MyParticle2D** optical_flow_field = 0;  //根據3x3x3 median所求得之粒子光流場
	MyParticle2D** social_force_field = 0;  //根據粒子光流場所求得之社會力場

	//for trainging2
	double **hosf_data = 0;
	int hosf_size = 0;
	int data_dim = 0;
	double **visual_word = 0;
	int word_count = 0;		//Edit11->Text	//決定要取幾個words

	double **whiten_trans = 0;	//whiten trans mat

	//????
	int soc_count = 0;	//soc_field計算次數


	/* 初始參數設定 */
	int end_of_video = cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_COUNT);
//	int pscale = 3;   	//粒子大小, NxN
	int pscale = Edit3->Text.ToInt();   	//粒子大小, NxN
	int half_s = pscale/2;	// 3/2 = 1
	double motion_th = Edit2->Text.ToDouble();
	int frame_gap = Edit9->Text.ToInt();

	//若distance為6, 則初始frames使用0~5
	int distance = Edit1->Text.ToInt();	//gap between two frames for optical flow  //
	int cur = distance; 				//current frame count

	//for social force
	int range_k = 7;	//for getting social force
	int range_l = 7;    //for getting social force
	double range_b = sqrt(range_k*range_k + range_l*range_l);

	//for cuboid
	int cb_dim = 64;
	int cb_n = 4;	//cuboid size = 2n x 2n x 2m;   unit := 1 particle
	int cb_m = 4;
	TStringList *dataList = new TStringList;
	String dataFileString = "";
	int dataFileCount = 0;

	//output資料夾建立
	if(!DirectoryExists(initDir+newDir)){	 				//若output path不存在, 則建立dir
		if(!ForceDirectories(initDir+newDir)){				//建立多層目錄
			Memo1->Lines->Add("Cannot create directory!");      //若建立dir失敗
			Memo1->Lines->Add(initDir+newDir);
		}
	}

	//2014.11.02 updated
	ofstream wfs, wfs2;
	ifstream rfs;
	//記錄output filename
	String outFile1 = initDir + newDir + dataName + dataFileString + "_1.bin";
	String outFile2 = initDir + newDir + dataName + dataFileString + "_2.bin";
	String tmpFile = outFile1;
	String vwFilename = initDir + newDir + dataName + dataFileString + "_visual_words.vw";
	String BoF_Filename = initDir + newDir + dataName + dataFileString + "_BoF.bof";
	String whiteFilename = initDir + newDir + dataName + dataFileString + "_white_trans.wmat";

	//write header
	wfs.open(outFile1.c_str(), ios::out | ios::binary);
	wfs.write((const unsigned char*)&cb_dim, sizeof(cb_dim));		//write "dim", cb_dim = 64
	wfs.write((const unsigned char*)&cb_dim, sizeof(cb_dim));		//write "dim", cb_dim = 64


	/* get frames */
	frames = new IplImage*[frameCount];
	images = new IplImage*[frameCount];	//grey image of frames
	for(int i=0;i<frameCount;i++){
		frames[i] = 0;
		images[i] = 0;
	}

	frame = cvQueryFrame(cam) ;   		//initial, 將此frame count視為 -1
	CvSize img_sz = cvGetSize(frame);
	Label15->Caption = cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES);
//	ShowMessage(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

	//取得初始所需影像 pscale + dist
	// 3+6 = 9 frames
	//建立0~8 frames, images
	int init_frame_number = pscale+distance;	//第一張current frame的編號
	for(int i=0;i<init_frame_number;i++){

		//第二張frame開始跳格
		if(i>0){
			for(int tt=0;tt<frame_gap+1;tt++){
				frame = cvQueryFrame(cam);
			}
		}
		else{
			frame = cvQueryFrame(cam);
		}

		frames[i] = cvCloneImage(frame);

		//convert the image to grey image
		//轉灰階故images需以單通道建立
		images[i] = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
		cvCvtColor( frames[i], images[i], CV_BGR2GRAY);
	}
//	ShowMessage(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

	//1.建立0~2張影像之光流
	//2.建立第0個optical flow field, 存放計算好的初始光流( 2維(x,y)向量 )
	//3.建立第0個social force field, 存放計算好的SF( 2維(x,y)向量 )
	//cuboid, NxNxM
//	int m = 2000;		//預定處理之最大畫格數
	int m = frameCount;	//預定處理之最大畫格數
	int off_width = img_sz.width/pscale;
	int off_height= img_sz.height/pscale;
	int off_size = off_width*off_height; //存放粒子的空間大小

	optical_flow = new MyParticle2D*[pscale];   			//影像光流
	optical_flow_field = new MyParticle2D*[m/pscale];       //粒子之光流場
	social_force_field = new MyParticle2D*[m/pscale];       //粒子之社會力場

	// MyParticle2D 預設為0, 不需清空
	for(int i=0;i<pscale;i++){
		optical_flow[i] = new MyParticle2D[img_sz.width*img_sz.height];
	}
	for(int i=0;i<m/pscale;i++){
		optical_flow_field[i] = 0;
		social_force_field[i] = 0;
	}
	optical_flow_field[0] = new MyParticle2D[off_size];
//	social_force_field[0] = new MyParticle2D[off_size];

	MyParticle2D p1, p2;

	//method.1
//	int *r = new int[off_size*off_size];
//	for(int p1j=0;p1j<off_width;p1j++){
//	for(int p1k=0;p1k<off_height;p1k++){
//		for(int p2j=0;p2j<off_width;p2j++){
//		for(int p2k=0;p2k<off_height;p2k++){
//			//坐標還原 粒子->pixel
//			p1 = cvPoint(p1j*pscale+half_s, p1k*pscale+half_s);
//			p2 = cvPoint(p2j*pscale+half_s, p2k*pscale+half_s);
//
//			r[p1j*off_height*off_width*off_height
//			 +p1k*off_width*off_height
//			 +p2j*off_height
//			 +p2k] = sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y)) ;
//
//		}
//		}
//	}
//	}


	//method.2
	// ==此宣告尚未做delete==
	// 計算exp_r
	start = clock();
//	int **r = new int*[off_size];
	double **exp_r = 0;
	exp_r = new double*[off_size];
	for(int i=0;i<off_size;i++){
//		r[i] = new int[off_size];
		exp_r[i] = new double[off_size];
	}

	for(int p1j=0;p1j<off_width;p1j++){
	for(int p1k=0;p1k<off_height;p1k++){
		for(int p2j=0;p2j<off_width;p2j++){
		for(int p2k=0;p2k<off_height;p2k++){
			//座標還原 粒子unit->pixel unit
			p1 = MyParticle2D(p1j*pscale+half_s, p1k*pscale+half_s);
			p2 = MyParticle2D(p2j*pscale+half_s, p2k*pscale+half_s);

//			r[p1j*off_height+p1k][p2j*off_height+p2k] =
//				sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y))+0.5;

			//tmpdist為p1,p2之間的距離
			double tmpx = (p2.x-p1.x);
			double tmpy = (p2.y-p1.y);
			double tmpdist = sqrt( tmpx*tmpx + tmpy*tmpy );

			//算exp
			exp_r[p1j*off_height+p1k][p2j*off_height+p2k] =
				exp( 0-((tmpdist-pscale)/range_b) );
		}
		}
	}
	}
	/* end expr 計算 */

	end = clock();
	Memo1->Lines->Add("建expr所花時間: ");
	Memo1->Lines->Add(end-start);

	//output測試
//	int pscale = 3;   	//粒子大小, NxN
//	int half_s = pscale/2;	// 3/2 = 1
//	String s = "";
//	p1.x = 1*pscale+half_s;
//	p1.y = 1*pscale+half_s;
//	p2.x = 3*pscale+half_s;
//	p2.y = 3*pscale+half_s;

//	Memo1->Lines->Add(s + p1.x+", "+p1.y);
//	Memo1->Lines->Add(s + p2.x+", "+p2.y);
//	Memo1->Lines->Add(exp_r[1*off_height+1][3*off_height+3]);
//	Memo1->Lines->Add((4.2-pscale)/range_b );

//	int *test = new int[MAX_FRAMES*img_sz.width*img_sz.height];
//	int ***test = new int**[MAX_FRAMES];
//	for(int i=0;i<MAX_FRAMES;i++){
//		test[i] = new int*[img_sz.width];
//		 *(test+i) = new int*[img_sz.width];
//		for(int j=0;j<img_sz.width;j++){
//			test[i][j] = new int[img_sz.height];
//			*( *(test+i)+j ) = new int[img_sz.height];
//			*( *(test+i)+j ) = new int[img_sz.height];
//		}
//	}

	/** 設定結束 **/



	/** 開始影片處理 **/
	while ( 1 )
	{
//		if(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

		//use webcam
//      CvCapture* cam = cvCaptureFromCAM( CV_CAP_ANY ) ;

		/*初始設定*/
		const int win_size = 10 ;
		Label14->Caption = frameCurrent = cur;
		int prev = cur-distance;    //取得前一張影像的編號, 初始6-6=0

		/************/

		//create a imge for displaying result
		//清白以顯示向量場 optical flow field, social force field
		img_res = cvCreateImage(img_sz, IPL_DEPTH_8U, 3) ;
		for ( int y = 0 ; y < img_sz.height ; ++y )
		{
			uchar* ptr = (uchar*)( img_res->imageData + y * img_res->widthStep ) ;
			for ( int x = 0 ; x <img_res->widthStep; ++x )
			{
				ptr[x] = 0 ;
			}
		}
		img_off = cvCreateImage(img_sz, IPL_DEPTH_8U, 3) ;
		for ( int y = 0 ; y < img_sz.height ; ++y )
		{
			uchar* ptr = (uchar*)( img_off->imageData + y * img_off->widthStep ) ;
			for ( int x = 0 ; x <img_off->widthStep; ++x )
			{
				ptr[x] = 0 ;
			}
		}
		img_soc = cvCreateImage(img_sz, IPL_DEPTH_8U, 3) ;
		for ( int y = 0 ; y < img_sz.height ; ++y )
		{
			uchar* ptr = (uchar*)( img_soc->imageData + y * img_soc->widthStep ) ;
			for ( int x = 0 ; x <img_soc->widthStep; ++x )
			{
				ptr[x] = 0 ;
			}
		}
		img_back = cvCreateImage(img_sz, IPL_DEPTH_8U, 3) ;
		for ( int y = 0 ; y < img_sz.height ; ++y )
		{
			uchar* ptr = (uchar*)( img_back->imageData + y * img_back->widthStep ) ;
			for ( int x = 0 ; x <img_back->widthStep; ++x )
			{
				ptr[x] = 0 ;
			}
		}


		/* optical flow 計算 */

		//get previous image
//		frame_prev = frames[cur-distance];	//default := 6-6 = 0
//		img_prev = images[cur-distance];   	//default := 6-6 = 0
//		img_prev = cvCloneImage(images[cur-distance]);   	//default := 6-6 = 0
		img_prev = cvCreateImage(img_sz, IPL_DEPTH_8U, 1) ;
		cvCvtColor( frames[prev], img_prev, CV_BGR2GRAY);

		//get current image
//		frame_cur = frames[cur];	//default := 6
//		img_curr = images[cur];		//default := 6
//		img_curr = cvCloneImage(images[cur]);		//default := 6
		img_curr = cvCreateImage(img_sz, IPL_DEPTH_8U, 1) ;
		cvCvtColor( frames[cur], img_curr, CV_BGR2GRAY);

		//get good features
		img_eig = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
		img_temp = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
		int corner_count = MAX_CORNERS ;
		CvPoint2D32f*  features_prev = new CvPoint2D32f[MAX_CORNERS] ;

		char feature_found[ MAX_CORNERS ] ;
		float feature_errors[ MAX_CORNERS ] ;

		//case.1 傳統特徵萃取
		if(CheckBox2->Checked == false){
			cvGoodFeaturesToTrack(
				img_prev,
				img_eig,
				img_temp,
				features_prev,
				&corner_count,
				0.01,
				5.0,
				0,
				3,
				0,
				0.4
			);

			cvFindCornerSubPix(
				img_prev,
				features_prev,
				corner_count,
				cvSize(win_size,win_size),
				cvSize(-1,-1),
				cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,20,0.03)
			);

			// L-K
		   //	CvSize pyr_sz = cvSize( frame->width + 8 ,frame->height / 3 ) ;
			pyr_prev = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
			pyr_cur = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
			CvPoint2D32f*  features_cur = new CvPoint2D32f[ MAX_CORNERS ] ;

			cvCalcOpticalFlowPyrLK(
				img_prev,
				img_curr,
				pyr_prev,
				pyr_cur,
				features_prev,
				features_cur,
				corner_count,
				cvSize(win_size,win_size),
				3,
				feature_found,
				feature_errors,
				cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,20,0.3),
				0
			);
			delete[] features_cur;
		} // end case1

////////////////////////////////
//砍掉重練
//每一行各做一次LK
//h行則做h次
//優點: 可避免coners過多, 且分多次尋找效率相同
//
//case.2 get our own features
//3x3 patch
//		int pscale = 3;
//		int half_s = pscale/2;	// 3/2 = 1

clock_t t1, t2;


		//my method 正式處理
		if(CheckBox2->Checked == true){

			int h = frame->height;
			int w = frame->width;
			int imgsize = w*h;

			// L-K
			pyr_prev = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
			pyr_cur = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
			CvPoint2D32f* features_cur = new CvPoint2D32f[ MAX_CORNERS ] ;

			//for 3 frames, 每3 frames做1次 (clip)
			start = clock();
			for(int t=0;t<pscale;t++){				//particle scale=3
				//取相對之新影像, clip size=3
				if(t>0){
					if( img_prev ) cvReleaseImage( &img_prev );
					if( img_curr ) cvReleaseImage( &img_curr );

					img_prev = cvCreateImage(img_sz, IPL_DEPTH_8U, 1) ;
					cvCvtColor( frames[prev+t], img_prev, CV_BGR2GRAY);

					img_curr = cvCreateImage(img_sz, IPL_DEPTH_8U, 1) ;
					cvCvtColor( frames[cur+t], img_curr, CV_BGR2GRAY);
				}

				t1 = clock();
				//60*320=20000;

				//取第i行各點計算LK
				for(int i=0 ; i<frame->height ; i++ ){
					corner_count = 0;

					//取得第i行之座標點放入features_prev
					for(int j=0 ; j<frame->width ; j++ ){
						float x = j;
						float y = i;

						//features_prev[corner_count] = cvPoint2D32f(x, y);
						features_prev[corner_count].x = x;
						features_prev[corner_count].y = y;
						corner_count++;

						if(corner_count > MAX_CORNERS)
							break;
					}

					//對第i行做LK
					cvCalcOpticalFlowPyrLK(
						img_prev,     			//前影像
						img_curr,               //目前影像
						pyr_prev,
						pyr_cur,
						features_prev,          //前特徵點
						features_cur,           //所找出的對應點
						corner_count,
						cvSize(win_size,win_size),
						3,
						feature_found,			//是否找到
						feature_errors,         //errors rate
						cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,20,0.3),
						0
					);

					CvPoint pt_prev;
					CvPoint pt_cur;
					int row, col;
					//check & show
					for ( int k = 0 ; k < corner_count ; k++)
					{
						if ( 0 == feature_found[k] || feature_errors[k] > 550 )
						{
//							printf("error is %f \n" , feature_errors[k] ) ;
							continue ;
						}
//						printf("find it !\n") ;

						//找到並存入optical_flow[]
//						pt_prev = cvPoint( features_prev[k].x , features_prev[k].y ) ;
//						pt_cur = cvPoint( features_cur[k].x , features_cur[k].y ) ;
						pt_prev.x = features_prev[k].x;
						pt_prev.y = features_prev[k].y;
						pt_cur.x = features_cur[k].x;
						pt_cur.y = features_cur[k].y;

						row = pt_prev.y;
						col = pt_prev.x;
						optical_flow[t][row*w+col].x = pt_cur.x - pt_prev.x;
						optical_flow[t][row*w+col].y = pt_cur.y - pt_prev.y;

						//show optical flow
						if( CheckBox1->Checked == true ){
							if( (pt_prev.x == pt_cur.x) && (pt_prev.y == pt_cur.y) ){
								continue;
							}
						}

						optical_flow[t][row*w+col].CalculateLength();
						int c = optical_flow[t][row*w+col].length/5.0 * 255;
						if(c>255)
						  c=255;

//						cvLine( img_res,pt_prev,pt_cur,CV_RGB( 255,0,0),1 );
//						cvLine( img_res,pt_prev,pt_prev,CV_RGB( 64,0,0),1 );
						cvLine( img_res,pt_cur,pt_cur,CV_RGB( c,0,0),1 );

					}//end save optical flow

					if(corner_count > MAX_CORNERS)
						break;
				}
				t2 = clock();

				Memo1->Lines->Add(s + "影像[" + prev + "-" + t + "] 光流所花時間: " + String(t2-t1));
//				Memo1->Lines->Add(t2-t1);

			}//end 3 time (pscale)
			end = clock();
			Memo1->Lines->Add("影像光流總花時間: "+ String(end-start));
//			Memo1->Lines->Add(end-start);

			delete[] features_cur;
			// End L-K


			start = clock();
			/* median by 3x3x3 into optical flow filed */
			int *median_x = new int[pscale*pscale*pscale];
			int *median_y = new int[pscale*pscale*pscale];
			int median_count = 0;
			int row=0, col=0;
			int pp = prev/pscale;	//因粒子大小為pscale, pp為影像所對應的粒子場編號
									//即粒子在時間軸上之編號
			optical_flow_field[pp] = new MyParticle2D[off_size];		//ps. off可改為一項即可, 因計算完soc即無需使用


			for( int y=0 ; y<h ; y+=pscale ){			//y+=3
			for( int x=0 ; x<w ; x+=pscale ){			//x+=3
				if( y>h || x>w )
					continue;

				median_count = 0;
				//以(y+1, x+1, t+1)為中心將3x3x3個pixel之光流放入陣列, 以qsort排序
				for( int t=0 ; t<pscale ; t++ ){
				for( int i=y ; i<y+pscale ; i++ ){
				for( int j=x ; j<x+pscale ; j++ ){
					median_x[median_count] = optical_flow[t][i*w+j].x;
					median_y[median_count] = optical_flow[t][i*w+j].y;
					median_count++;
				}
				}
				}
				qsort (median_x, median_count, sizeof(int), compare);
				qsort (median_y, median_count, sizeof(int), compare);

				//計算&&存放 optical flow into field
				//以粒子為中心, 將算出的中間值光流和放入所對應的粒子光流場位置(row, col)
				// x=>col
				// 0=>0, 3=>1, 6=>2;
				row = y/pscale;
				col = x/pscale;
				optical_flow_field[pp][row*off_width+col].x = median_x[median_count/2];
				optical_flow_field[pp][row*off_width+col].y = median_y[median_count/2];

//				String s = "";
//				Memo1->Lines->Add(s+row+", "+col);

				//根據門檻值去除motion較小的粒子
				optical_flow_field[pp][row*off_width+col].CalculateLength();
				double magnitude = optical_flow_field[pp][row*off_width+col].length;
				if(magnitude < motion_th){
					optical_flow_field[pp][row*off_width+col].x = 0;
					optical_flow_field[pp][row*off_width+col].y = 0;
				}
				else{
					int tmpx = optical_flow_field[pp][row*off_width+col].x;
					int tmpy = optical_flow_field[pp][row*off_width+col].y;

	//				if( abs(tmpx) >= 2 || abs(tmpy)>=2 )
	//					Memo1->Lines->Add(s+optical_flow_field[prev][row*off_width+col].x+", "
	//									+optical_flow_field[prev][row*off_width+col].y);

					CvPoint pt_prev = cvPoint(x+half_s, y+half_s);
					CvPoint pt_cur = cvPoint(x+half_s+tmpx, y+half_s+tmpy);
					cvLine( img_off,pt_prev,pt_cur,CV_RGB( 255,0,0),1 );
					cvLine( img_off,pt_prev,pt_prev,CV_RGB( 128,128,128),1 );
				}


			}
			}

			/* end median */
			end = clock();
			Memo1->Lines->Add("3影像median總花時間: " + String(end-start));
//			Memo1->Lines->Add(end-start);




			delete[] median_x;
			delete[] median_y;


		}//end if checked

		/************** end optical flow ********************/

		///////////// social force field //////////////
		start = clock();

		//以粒子為中心的其他粒子之相對位置
		int pp = prev/pscale;	//因粒子大小為pscale, pp為影像所對應的粒子場編號
								//即粒子在時間軸上之編號

//		social_force_field[pp] = new MyParticle2D[4108];
		social_force_field[pp] = new MyParticle2D[off_size];             //當掉

		double mass = 1;
		int disp_k = ((2*range_k+1)/pscale); // ((2*7+1)/3) = 5
		int disp_l = ((2*range_l+1)/pscale);
		//相對位置範圍 -2~2
		int start_k = 0-disp_k/2;			 // 從-2開始
		int start_l = 0-disp_l/2;
		int end_k = start_k+disp_k;			 // 到2為止
		int end_l = start_l+disp_l;

		double f_sum_x = 0;  // sum of force
		double f_sum_y = 0;  // sum of force
		for(int row=0;row<off_height;row++){
		for(int col=0;col<off_width;col++){
			//inactive之粒子不計算soc
			if(optical_flow_field[pp][row*off_width+col].x == 0
				&& optical_flow_field[pp][row*off_width+col].y == 0){
				continue;
			}



			//對active particle計算其soc
			f_sum_x = 0;
			f_sum_y = 0;
			for(int i=start_l;i<=end_l;i++){	//-2~2
			for(int j=start_k;j<=end_k;j++){
				//若為中心粒子則不計算, 因距離影響為0
				if(i==0&&j==0)
					continue;

				//取相對粒子之真實座標
				int ii = row+i;
				int jj = col+j;
				//超過影像範圍之不存在的粒子, 不計算(視為0) //暫定
				if( ii<0 || ii>=off_height || jj<0 || jj>=off_width ){
					continue;
				}
				//所對應的動量太小之particle不列入soc計算 ( 存off時已設為(0,0) )
				if(optical_flow_field[pp][ii*off_width+jj].x == 0
					&& optical_flow_field[pp][ii*off_width+jj].y == 0 ){
					continue;
				}

				//排除上述粒子, 計算以(row, col)為中心之social force
				//exp_r[p1j*off_height+p1k][p2j*off_height+p2k]
				double tmp_exp = exp_r[row*off_height+col][ii*off_height+jj];
				f_sum_x +=
					mass*tmp_exp*
					(
						optical_flow_field[pp][ii*off_width+jj].x
						-optical_flow_field[pp][row*off_width+col].x
					);
				f_sum_y +=
					mass*tmp_exp*
					(
						optical_flow_field[pp][ii*off_width+jj].y
						-optical_flow_field[pp][row*off_width+col].y
					);
			}
			}


			social_force_field[pp][row*off_width+col].x = (int)(f_sum_x+0.5);
			social_force_field[pp][row*off_width+col].y = (int)(f_sum_y+0.5);
//			Memo1->Lines->Add(s + social_force_field[prev][row*off_width+col].x
//				+", "+social_force_field[prev][row*off_width+col].y);
//
//			Memo1->Lines->Add(s + (int)f_sum_x
//				+", "+(int)f_sum_y);


			//output result
			CvPoint pt_prev = cvPoint( col*pscale+half_s, row*pscale+half_s ) ;
			CvPoint pt_cur = cvPoint( pt_prev.x+social_force_field[pp][row*off_width+col].x,
										pt_prev.y+social_force_field[pp][row*off_width+col].y ) ;

//			if(abs(optical_flow_field[prev][row*off_width+col].x) >= 2
//			 || abs(optical_flow_field[prev][row*off_width+col].y) >=2 ){

				cvLine( img_soc,pt_prev,pt_cur,CV_RGB( 255,0,0),1 );
				cvLine( img_soc,pt_prev,pt_prev,CV_RGB( 128,128,128),1 );
//			}
		}
		}
		soc_count++;

		end = clock();
		Memo1->Lines->Add("social force所花時間: "+String(end-start));
//		Memo1->Lines->Add(end-start);
//		cvSub(img_prev, img_curr, img_back);

		/****************** end social force *************************/


		/////////////////////// HoSF of Cuobdid ////////////////////
		//建cuboid&data
		//檢查是否有足夠的soc_count去建立cuboid

		/***********  overlapping case  *******************
		case		spatial	temporal    (0/1 := false/true)
		0              0       0
		1              0       1
		2              1       0
		3              1       1
		原則上目前spatial都是overlapping, 故只控制temporal是否overlapping
		****************************************************/
		bool b_soc = false;
		int b_overlapping_case = 3;		//2 or 3

		switch (b_overlapping_case) {
			//non-overlapping
			case 0:
			case 2:
				b_soc = (soc_count%(2*cb_m)==0) && (soc_count>0) ;		//當soc數量為cuboid倍數時才做
				break;
			//overlapping
			case 1:
			case 3:
				b_soc = (soc_count >= 2*cb_m);                          //只要soc足夠建立出cuboid就做
				break;

			default:
				b_soc = false;
		}

////	舊設定
//		if( b_overlapping == true)
//			b_soc = (soc_count%(2*cb_m)==0) && (soc_count>0) ;				//non-overlapping
//		else
//			b_soc = (soc_count >= 2*cb_m);									//overlapping

		if( b_soc ){           								//boolean_soc
			start = clock();
			int t_center = soc_count-cb_m;	//cb中心
			int cb_start_x, cb_start_y, cb_start_z;  				//計算cb之起始點
			double soc_direct;
			int bin;
			double sx, sy;
			int cc=0;
			double *cb_hist = new double[cb_dim];        //2014.11.20改為動態  cb_dim = 64

//			double cb_test[512];

			//在粒子光流場中找active particle
			for(int row=0;row<off_height;row++){
			for(int col=0;col<off_width;col++){
				//inactive particle跳過
				if(optical_flow_field[t_center][row*off_width+col].x == 0
					&& optical_flow_field[t_center][row*off_width+col].y == 0){
					continue;
				}
				//範圍不足以建cuboid之particle
				if(row<cb_n || row>off_height-cb_n
					|| col<cb_n || col>off_width-cb_n)
					continue;

				//找到active並建立8 sub-coboid
				cb_start_x = col-cb_n;
				cb_start_y = row-cb_n;
				cb_start_z = t_center-cb_m;
				static int locx[8]={0,1,0,1,0,1,0,1};
				static int locy[8]={0,0,1,1,0,0,1,1};
				static int locz[8]={0,0,0,0,1,1,1,1};
				int xx, yy, zz;

				//陣列歸0
				for(int i=0;i<cb_dim;i++){
					cb_hist[i] = 0;
				}

				for(int scb=0;scb<8;scb++){
					for(int k=0;k<cb_m;k++){
					for(int j=0;j<cb_n;j++){
					for(int i=0;i<cb_n;i++){
						zz = cb_start_z+locz[scb]*cb_m+k;
						yy = cb_start_y+locy[scb]*cb_n+j;
						xx = cb_start_x+locx[scb]*cb_n+i;

						sx = social_force_field[zz][yy*off_width+xx].x;
						sy = social_force_field[zz][yy*off_width+xx].y;
						//計算soc向量方向
						soc_direct = cvFastArctan(sy, sx);
						//cvFastArctan之誤差當(1,1)(-1,-1), accurate is about 0.3	by OpenCV 2.0
						if( sx == sy )
							soc_direct += 0.3;
						//判斷bin
						if( soc_direct < 45 )
							bin = 0;
						else if( soc_direct < 90 )
							bin = 1;
						else if( soc_direct < 135 )
							bin = 2;
						else if( soc_direct < 180 )
							bin = 3;
						else if( soc_direct < 225 )
							bin = 4;
						else if( soc_direct < 270 )
							bin = 5;
						else if( soc_direct < 315 )
							bin = 6;
						else
							bin = 7;

						cb_hist[scb*8+bin] += sqrt(sx*sx+sy*sy);

					}
					}
					}

				}
				/* normalization */
//				Series1->Clear();
//				Series1->AddArray(cb_hist, 64);
				double cb_max = cb_hist[0];		//max magnitude of cuboid
				double cb_min = cb_hist[0];		//min magnitude of cuboid
				for(int i=1;i<cb_dim;i++){     		//search max
					if(cb_hist[i] > cb_max)
						cb_max = cb_hist[i];
					if(cb_hist[i] < cb_min)
						cb_min = cb_hist[i];                                    //2014.11.02 updated
				}

//				if(cb_max == 0){
//					Memo2->Lines->Add(bbb);
//					for(int i=0;i<64;i++){     		//search max
//						Memo2->Lines->Add(cb_hist[i]);
//					}
//					Memo2->Lines->Add("===");
//					for(int i=0;i<512;i++){     		//search max
//						Memo2->Lines->Add(cb_test[i]);
//					}
//					Memo2->Lines->Add(row);
//					Memo2->Lines->Add(col);
//					Memo2->Lines->Add("--");
//					Memo2->Lines->Add(optical_flow_field[t_center][row*off_width+col].x);
//					Memo2->Lines->Add(optical_flow_field[t_center][row*off_width+col].y);
//					Memo2->Lines->Add(social_force_field[t_center][row*off_width+col].x);
//					Memo2->Lines->Add(social_force_field[t_center][row*off_width+col].y);
//					cvWaitKey();
//				}
				//normalize to 0~1
				if(cb_max != 0){
					for(int i=0;i<cb_dim;i++){
						cb_hist[i] = (cb_hist[i] - cb_min)/(cb_max - cb_min);	//2014.11.02 updated
					}
				}
				/*end normalize*/


				//write data
				wfs.write((unsigned char*)cb_hist, sizeof(cb_hist[0])*cb_dim);	//需改為sizeof(cb_hist[0])*64

				//記錄data	//stack
				String s = "";
				s = s + cb_hist[0];
				for(int i=1;i<cb_dim;i++){
					s = s + " " + cb_hist[i];
				}
				dataList->Add(s);

				cc++;
			}
			}

			//當ap數超過10萬, 建新檔案
//			if(dataList->Count > 100000){
//				dataList->SaveToFile(initDir + newDir + dataName + dataFileString + ".txt");			//data資料輸出
//				dataList->Clear();
//				dataFileString = "_(" + WideString(++dataFileCount) + ")";
//			}

			//release
			delete[] cb_hist;

			end = clock();

			Memo1->Lines->Add("****");
			Memo1->Lines->Add(s+"clip: " + t_center + " 之cuboid總數: " + cc);
			Memo1->Lines->Add("建data所花時間: " + String(end-start));
//			Memo1->Lines->Add("============");
			Memo1->Lines->SaveToFile(initDir + newDir + dataName + "_readme.txt");
		}

		// end of HoSF of Cuboid per 3 frames


		// change 11.14
		/***** training step 2 + 3 ******/
		int training_type = RadioGroup1->ItemIndex; // 0=old, 1=new
		isWhitening = CheckBox7->Checked;

		int BoF_count;		//幾筆BoF資料 (part 2 實做 n clips, 則 BoF_count = n)

		if( training_type == 1){
			//判斷時間點
			Memo2->Lines->Add("" + WideString(frameCurrent) +"/"+ WideString(frameCount));

			if( frameCurrent > frameCount*0.5 ){		// if >: 做training step 2 + 3
				/* training step 2 */
				if( visual_word == 0 ){     //initial     //11.20因visual_word需用到結束, 故改為visual_word判斷initial
					Memo2->Lines->Add("training step 2 initial:");
					//write dataSize of header for "_1.bin"       //2014.11.02
					int dataSize = dataList->Count;
					wfs.seekp(0, wfs.beg);
					wfs.write((const unsigned char*)&dataSize, sizeof(dataSize));		//write "dataSize", dataSize = 64
					wfs.close();

					dataList->SaveToFile(initDir + newDir + dataName + dataFileString + ".txt");
					dataList->Clear();

					/** load "*_1.bin" **/
					//read data of part1
//					outFile1 = initDir + newDir + dataName + dataFileString + "_1.bin";
					ifstream sf_rs(outFile1.c_str(), ios::in | ios::binary);
					//read data size & dim
					sf_rs.read((unsigned char*)&hosf_size, sizeof(hosf_size));
					sf_rs.read((unsigned char*)&data_dim, sizeof(data_dim));

					hosf_data = new double*[hosf_size];  //size
					for (int i=0; i < hosf_size; i++) {
						hosf_data[i] = new double[data_dim];
					}

					/** bin => hosf_data **/
					for (int i = 0; i < hosf_size; i++) {
						sf_rs.read((unsigned char*)hosf_data[i], sizeof(hosf_data[i][0])*data_dim);
					}
					sf_rs.close();

					/** hosf_data-> whitening */
					if(isWhitening == true){
						Memo1->Lines->Add("Whitening Transform...");
						if(whiten_trans)
							myDeleteArray2D(&whiten_trans, data_dim);
						myCreateArray2D(&whiten_trans, data_dim, data_dim);

						//whitening
						myWhitening3(hosf_data, hosf_data, whiten_trans, hosf_size, data_dim);

						//test covariance
						double** cov_output;
						myCreateArray2D(&cov_output, data_dim, data_dim);
						getCovariance(hosf_data, cov_output, hosf_size, data_dim);
						output("hosf_cv.txt", cov_output, data_dim, data_dim, 2);
						myDeleteArray2D(&cov_output, data_dim);
						Memo1->Lines->Add("....end whitening");

						ofstream white_ws;
						white_ws.open(whiteFilename.c_str(), ios::out | ios::binary);				//"white_trans.wmat"  matrix of dim by dim
						white_ws.write((const unsigned char*)&data_dim, sizeof(data_dim));          //rows
						white_ws.write((const unsigned char*)&data_dim, sizeof(data_dim));          //cols
						for(int i=0;i<data_dim;i++){
							white_ws.write((const unsigned char*)whiten_trans[i], sizeof(whiten_trans[i][0])*data_dim);		//write "words"
						}
						white_ws.close();
					}

					/** hosf_data => k-mean **/
					//initial double *seed
//					word_count = Edit11->Text.ToInt();
					if(word_count > hosf_size){
						word_count = hosf_size*0.1;
					}

					word_count = hosf_size*0.1;
					visual_word = new double*[word_count];			//visual_word留到結束時realese by 2014.11.20
					for (int i = 0; i < word_count; i++) {
						visual_word[i] = new double[data_dim];
						for(int j=0;j<data_dim;j++)
							visual_word[i][j] = 0;
					}
					//select seeds for visual words
					int sd_temp;
					int *sd_temp2 = new int[hosf_size];//挑選SEED時,紀錄RANDOM LABEL是否出現
					for (int i = 0; i < hosf_size; i++) {
						 sd_temp2[i]=0;
					}
					//start random select
					for (int i = 0; i < word_count; i++) {
						  sd_temp = random(hosf_size);
						  if(sd_temp2[sd_temp]==0) {              //be selected
							 sd_temp2[sd_temp]=1;
							 for (int j = 0; j < data_dim; j++) {
								visual_word[i][j] = hosf_data[sd_temp][j];      //copy data to seed
							 }
						  }
						  else {
							i--;          //re-select
							continue;
						  }
					}
					delete[] sd_temp2;


					//===============================
					//debug
//					String tstring = initDir + newDir + dataName + dataFileString + "_test.bin";
//					ofstream ts(tstring.c_str(), ios::out | ios::binary);
//
//					//point to data size & dim
//					ts.write((unsigned char*)&hosf_size, sizeof(hosf_size));
//					ts.write((unsigned char*)&data_dim, sizeof(data_dim));
//
//					Memo7->Lines->Add(hosf_size);
//					Memo7->Lines->Add(data_dim);
//					for (int i = 0; i < hosf_size; i++) {
//						Memo7->Lines->Add("i ======== " + WideString(i));
//						for(int j=0;j<data_dim;j++){
//							Memo7->Lines->Add("" + WideString(hosf_data[i][j]));
//						}
//
//					   ts.write((unsigned char*)hosf_data[i], sizeof(hosf_data[i][0])*data_dim);
//					}
//					Memo7->Lines->SaveToFile("_bin.txt");
//					ts.close();
					//===============================


					/** start k_mean for visual word **/
					Memo1->Lines->Add("");
					Memo1->Lines->Add("*********************");
					Memo1->Lines->Add("Visual words建立....");
					Memo1->Lines->Add("hosf data size(part1) = "+WideString(hosf_size));
					Memo1->Lines->Add("word count = "+WideString(word_count));

					int *hosf_result = new int[hosf_size];
					int *word_size = new int[word_count];
					int c_count=0;
					int c_max_count=50;

					//return;		//debug

					//建 visual words, (from hosf_data)
					 my_k_means(
						hosf_data,		//input data
						hosf_size,     //number of data
						data_dim,		//dimension of data
						word_count, //K clustering

						visual_word,      	//Seed, input and output, := codewords
						hosf_result,//result
						word_size,

						c_count,
						c_max_count
					//	int cov_threshold;		//wcss變動小於此值則視為未變動(變動過小)
					//	int cov_count;
					//	int cov_max_count;		//wcss變動過小之次數超過此值時視為收斂
					) ;

					//output visual words as dictionary
					ofstream vw_ws(vwFilename.c_str(), ios::out | ios::binary);			//"_visualword.vw"
					vw_ws.seekp(0, vw_ws.beg);
					vw_ws.write((const unsigned char*)&word_count, sizeof(word_count));		//write "word_count"
					vw_ws.write((const unsigned char*)&data_dim, sizeof(data_dim));
					for(int i=0;i<word_count;i++){
						vw_ws.write((const unsigned char*)visual_word[i], sizeof(visual_word[i][0])*data_dim);		//write "words"
					}
					vw_ws.close();
					//release memory for visual word's tool
					for (int i = 0; i < hosf_size; i++) {
						 delete[] hosf_data[i];
						 hosf_data[i] = NULL;
					}
					delete[] hosf_data;
					hosf_data = NULL;
					delete[] hosf_result;
					delete[] word_size;

					/** end k-mean for visual words */

					Memo1->Lines->Add("Visual words建立完成!");
					Memo1->Lines->Add("*********************");
					Memo1->Lines->Add("");
				/* end training step 2 */

					//use for trainging step 3 for data
//					outFile2 = initDir + newDir + dataName + dataFileString + "_2.bin";
					wfs.open(outFile2.c_str(), ios::out | ios::binary);
					wfs.seekp(0, wfs.beg);
					wfs.write((const unsigned char*)&cb_dim, sizeof(cb_dim));		//write "dim", cb_dim = 64
					wfs.write((const unsigned char*)&cb_dim, sizeof(cb_dim));		//write "dim", cb_dim = 64

					//output
					//use for trainging step 3 for BoF (output)
					BoF_count = 0;
//					String BoF_Filename = initDir + newDir + dataName + dataFileString + "_BoF.bin";
					wfs2.open(BoF_Filename.c_str(), ios::out | ios::binary);
					wfs2.seekp(0, wfs2.beg);
					wfs2.write((const unsigned char*)&BoF_count, sizeof(BoF_count));		//write "BoF_count"
					wfs2.write((const unsigned char*)&word_count, sizeof(word_count));		//write "word_count", word_count = 100?;
				}
				/* training step 3 */
				else{
					Memo1->Lines->Add("training step 3: ");
					int dataSize = dataList->Count;
					wfs.seekp(0, wfs.beg);
					wfs.write((const unsigned char*)&dataSize, sizeof(dataSize));		//write dataSze (已知 datalist->count)
					wfs.close();

					//======== debug ===============
//					String tmpFile2 = initDir + newDir + dataName + "\\" + dataName + dataFileString + "_frame_" + frameCurrent + ".bin";
//					String tmpFile2t = initDir + newDir + dataName + "\\" + dataName + dataFileString + "_frame_" + frameCurrent + ".txt";
//					if(!DirectoryExists(initDir + newDir + dataName)){	 				//若output path不存在, 則建立dir
//						if(!ForceDirectories(initDir + newDir + dataName)){				//建立多層目錄
//							Memo1->Lines->Add("Cannot create directory!");      //若建立dir失敗
//							Memo1->Lines->Add(initDir + newDir + dataName);
//						}
//					}

					//===============================================
					//debug
					//文本data轉入陣列
//					TStringList *fdataList = dataList;
//					int d_size = fdataList->Count;
//					Memo1->Lines->Add("N = "+WideString(d_size));
//
//					double **mydata = new double*[d_size];
//					for(int i=0;i<d_size;i++)
//						mydata[i] = 0;
//
//					TStringList *tmpList = new TStringList;          //轉換用暫存data
//					tmpList->Delimiter = ' ';   		//設定切割符號
//
//					TStringList *tdataList = new TStringList;        //訓練好之輸出data
//
//					int dim;
//					for(int i=0;i<d_size;i++){
//						tmpList->Clear();
//						tmpList->DelimitedText = fdataList->Strings[i];		//將fdataList切割放入tmpList
//
//						dim = tmpList->Count;			 //取得此筆資料維度 => 64維
//						mydata[i] = new double[dim];
//						for(int j=0;j<dim;j++){
//							mydata[i][j] = tmpList->Strings[j].ToDouble();
//				//			tdataList->Add(mydata[i][j]);
//						}
//					}
					//=================================================


					dataList->Clear();

					//read data per frame
					rfs.open(outFile2.c_str(), ios::in | ios::binary);

					//read data size & dim
					int dataDim;
					rfs.read((unsigned char*)&dataSize, sizeof(dataSize));		//read "data size"
					rfs.read((unsigned char*)&dataDim, sizeof(dataDim));		//read "data dimension"

					//每讀一筆data做分類並記錄出現次數
					double* data = new double[dataDim];		//tmp data

					int wordCount = word_count;
					double* dist = new double[wordCount];		//??? = size of visual words = 1000?
					double* BoF_DES = new double[wordCount];		//bag of feature descriptor
//					myZerosArray(BoF_DES, wordCount);
					for(int i=0;i<wordCount;i++)
						BoF_DES[i] = 0;

					for(int i=0;i<dataSize;i++){
						rfs.read((unsigned char*)data, sizeof(data[0])*dataDim);		//read a "data" (64-dim)

						//whitening transform
						if(isWhitening == true){
							doWhitenPatch(&data, &data, whiten_trans, 1, dataDim);		//一次做1筆data, 故n:=1; data為1維, 故input := &data
						}

						//計算data與各words的距離
						for(int j=0;j<wordCount;j++){
							dist[j] = GetDist(data, visual_word[j], dataDim);
//							dist[j] = GetDist(mydata[i], visual_word[j], dataDim);			//debug
						}
						//取得距離最近的分類
						int index_min = argmin(dist, wordCount);
						//記錄分類次數
						BoF_DES[index_min]++;
					}
					rfs.close();

					Series1->Clear();
					Series1->AddArray(BoF_DES, (wordCount-1));
					//normalization
					int tmp = Normalize(BoF_DES, wordCount);
 //					Memo2->Lines->Add("normalize = " + WideString(tmp));
					BoF_count++;

					//記錄BoF descriptor
					//use for trainging step 3
					wfs2.write((unsigned char*)BoF_DES, sizeof(BoF_DES[0])*wordCount);

					//finsih per frame
					Memo1->Lines->Add("dataSize = "+WideString(dataSize));
//					Memo2->Lines->Add("dataSize = "+WideString(dataSize));
//					Memo2->Lines->Add("dataDim = "+WideString(dataDim));
//					Memo1->Lines->Add("============");
					delete[] data; data=NULL;
					delete[] dist; dist=NULL;
					delete[] BoF_DES; BoF_DES=NULL;

					//關檔留到video結束

					//===============================

					//write next frame's data
//					outFile2 = initDir + newDir + dataName + dataFileString + "_2.bin";
					wfs.open(outFile2.c_str(), ios::out | ios::binary);
					wfs.seekp(0, wfs.beg);
					wfs.write((const unsigned char*)&cb_dim, sizeof(cb_dim));		//write "dim", cb_dim = 64
					wfs.write((const unsigned char*)&cb_dim, sizeof(cb_dim));		//write "dim", cb_dim = 64

					// end training step 3
				}
			}
		}

		/***** end training step 2 +3 ******/




		//////////////// show ////////////////
		const char* window_prev = "img_prev" ;
		const char* window_curr = "img_curr" ;
		const char* window_res = "result" ;
		const char* window_soc = "soc" ;
		const char* window_off = "optical_flow_field" ;
//		const char* window_dist = "dist" ;

		cvNamedWindow( window_prev );
		cvNamedWindow( window_curr,CV_WINDOW_AUTOSIZE );
		cvNamedWindow( window_res );
		cvNamedWindow( window_soc );
		cvNamedWindow( window_off );
//		cvNamedWindow( window_dist );
		cvShowImage( window_prev, img_prev );
		cvShowImage( window_curr, img_curr );
		cvShowImage( window_res, img_res );
		cvShowImage( window_soc, img_soc );
		cvShowImage( window_off, img_off );
//		cvShowImage( window_dist, img_back );

		int wt = 33;
		if(CheckBox3->Checked==true)
			wt = 0;

		char opt = cvWaitKey( wt ) ;
		if ( 27 == opt )
		{
			break ;
		}

		cvReleaseImage( &img_curr );
		cvReleaseImage( &img_prev );
		cvReleaseImage( &img_res );
		cvReleaseImage( &img_soc );
		cvReleaseImage( &img_off );

		cvReleaseImage( &img_eig );
		cvReleaseImage( &img_temp );

		cvReleaseImage( &frames[prev] );
		cvReleaseImage( &images[prev] );

		delete[] features_prev;

		//取得新進影像
		//get next n frame, n=pscale
		//case.1
		cur = cur + pscale;
		frameCurrent = cur;

		for(int t=0;t<pscale;t++){
			cvReleaseImage( &frames[prev+t] );
			cvReleaseImage( &images[prev+t] );

			//frame跳格
			for(int tt=0;tt<frame_gap+1;tt++){
				frame = cvQueryFrame(cam);
				if( !frame ){
					break;
				}
			}
			if( !frame ){
				break;
			}

			frames[cur+t] = cvCloneImage(frame);
			//convert the image to grey image
			images[cur+t] = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
			cvCvtColor( frames[cur+t], images[cur+t], CV_BGR2GRAY);
		}
		//if end of video
		if( !frame ){
			end2 = clock();
			if( training_type == 0){
				dataList->SaveToFile(initDir + newDir + dataName + dataFileString + ".txt");			//剩餘data資料輸出
			}

			Memo1->Lines->Add("end of video!");
			Memo1->Lines->Add( "total cost time: " + WideString(end2-start2) + " (ms)." );
			Memo1->Lines->SaveToFile(initDir + newDir + dataName + "_readme.txt");

			if( training_type == 1){
				wfs.close();
				//記錄BoF Count;
				wfs2.seekp(0, wfs2.beg);
				wfs2.write((const unsigned char*)&BoF_count, sizeof(BoF_count));		//write "BoF_count"
				wfs2.close();

				//release visual word
				myDeleteArray2D(&visual_word, word_count);

				if(isWhitening == true){
					myDeleteArray2D(&whiten_trans, data_dim);
				}
				Memo2->Lines->SaveToFile(initDir + newDir + dataName + "_readme2.txt");

			}

			break;
		}

	}//end while

	for(int i=0;i<frameCount;i++){
		if( frames[i] )
			cvReleaseImage( &frames[i] );
		if( images[i] )
			cvReleaseImage( &images[i] );
	}
	delete[] frames;
	delete[] images;

	//release all
	for(int i=0;i<m/pscale;i++){
		if(optical_flow_field[i] == NULL)
			continue;
		delete[] optical_flow_field[i];
		delete[] social_force_field[i];
	}
	delete[] optical_flow_field;
	delete[] social_force_field;

	for(int i=0;i<off_size;i++){
		if(!exp_r[i]){
			continue;
//		    ShowMessage(i);
		}
		delete[] exp_r[i];
	}
	delete[] exp_r;

	delete dataList;




	cvReleaseCapture( &cam );
	cvDestroyAllWindows() ;

}
//---------------------------------------------------------------------------


//double aaa[2];

CvPoint pp(CvPoint p, int x, int y){
	p.x = x;
	p.y = y;

	return p;
}


double GetDist(double *a, double *b, int data_length){
	double dist = 0;

	for(int i=0;i<data_length;i++){
		dist = dist + (a[i]-b[i])*(a[i]-b[i]);
	}
	dist = sqrt(dist);

	return dist;
}

void RefreshDist(
	double *Dist, 		//更新後的各data與最近seed之Distance
	double **Data,
	int d_size,			//numbers of data, data數量
	int dimension,
	int *Seed_label,    //chk if Data is a Seed
	double **Seed,
	int k				//numbers of Seed, 目前取得seed數量
	){

	int dim = dimension;    // 8x8 = 64 dimention

	for(int i=0;i<d_size;i++){

		/* Dist[i] = 0 if Seed_label[i] = 1; */		//若該data為seed則距離=0, 即機率為0
		if(Seed_label[i] == 1){
			Dist[i]=0;
		}

		/* Dist[i] = min( || Data[i]-Seed[j] || ) */
		else{
			//先取第一個當基準
			double m = GetDist( Data[i], Seed[0], dim );

			//求最小距離 := 選取機率
			for(int j=1;j<k;j++){
			   double tmp = GetDist( Data[i], Seed[j], dim );
			   if(tmp < m)
				   m = tmp;
			}
			Dist[i] = m;

		}
	}

}
/** 2014.11.23 發現特例 **
 * 當 data數量不足 or data重覆時, 導致挑選的新seed可能重覆位置(dist=0)而不被挑選,
 * 會導致 k (seed數量)不足, 且 i >= d_size, 使得 *k_cur < K;

 * 應將 k_cur 視為實際挑選之 seed_count
 */
void k_means_plus(
	double **Data,		//input data
	int *Seed_label,    //input seed_label of data
	int d_size,         //number of data
	int dimension,		//dimension of data
	int K,              //K clustering
	double **Seed,      //Seed be getting
	int *k_cur          //k cluster currently
	){

	double *Dist = new double[d_size];
	double *proba_Dist = new double[d_size];
	int k = *k_cur;

	int dim = dimension;

	/** K-means++ **/
	RefreshDist(Dist,Data,d_size,dim,Seed_label,Seed,k);

	/* 取剩下的seed */
	int total_dist;
	int new_seed_dist;
	while( k < K ){
		if( k >= d_size) break;			//若種子數已超過data數, 則中止

		total_dist = 0;
		new_seed_dist = 0;

		//計算seed機率
		for(int i=0;i<d_size;i++){
			//若Data[i]已為Seed則跳過, RefreshDist() 中已將機率設為0
			proba_Dist[i] = Dist[i]*Dist[i];
			total_dist += proba_Dist[i];		//機率與距離平方為正比
		}

		//挑下一個seed
		new_seed_dist = random(total_dist);		//根據機率所取的new seed
		int i;
		for(i=0;i<d_size;i++){
			//Data[i]已為Seed則跳過
			if(Seed_label[i] == 1){
				continue;
			}
			//找到新Seed為Data[i]
			if(new_seed_dist < proba_Dist[i]){
				Seed_label[i] = 1;
				Seed[k] = Data[i];
				k++;
				RefreshDist(Dist,Data,d_size,dim,Seed_label,Seed,k);
				break;
			}
			else{
				new_seed_dist = new_seed_dist - proba_Dist[i];
			}
		}
		if(i>=d_size){
			break;
		};	//當所有Data已為種子, 防錯.

		/** 2014.11.23 發現特例 **
		當 data數量不足 or data重覆時, 導致挑選的新seed可能重覆位置(dist=0)而不被挑選,
		會導致 k (seed數量)不足, 且 i >= d_size, 使得 *k_cur < K;

		應將 k_cur 視為實際挑選之 seed_count
		*/
	}

	*k_cur = k;

	delete[] Dist;
	delete[] proba_Dist;

}


//	my_k_means(
//		Data,		//input data
//		d_size,     //number of data
//		dim,		//dimension of data
//		seed_count, //K clustering
//		Seed,      	//Seed, input and output
//		c_result,
//		c_size,
//
//		c_count,
//		c_max_count
//	//	int cov_threshold;		//wcss變動小於此值則視為未變動(變動過小)
//	//	int cov_count;
//	//	int cov_max_count;		//wcss變動過小之次數超過此值時視為收斂
//	);

double my_k_means(
	double **Data,		//input data
	int d_size,         //number of data
	int dimension,		//dimension of data
	int Kc,             //K clustering

	double **Seed,      //Seed be getting
	int *c_result,
	int *c_size,

	int c_count,
	int c_max_count
//	int cov_threshold;		//wcss變動小於此值則視為未變動(變動過小)
//	int cov_count;
//	int cov_max_count;		//wcss變動過小之次數超過此值時視為收斂
	){

	/* 開始做k-means */
	double wcss = 0, prev_wcss = 0, dist = 0;
	int argmin_dist = 0;

	//跑質心收斂
	while(1){

		//initial
		for(int i=0;i<Kc;i++){
			c_size[i] = 0;
		}

		//step1. clustering;
		//做分群, 對每筆data找最小距離seed並配置
		for(int i=0; i<d_size; i++){
			dist = 0;
			argmin_dist = 0;

			//計算第i筆資料與所有種子之最小距離
			for(int j=0;j<Kc;j++){
				//算距離
				double dist_tmp = 0;
				double tmp;
				for(int k=0;k<dimension;k++){
					tmp = Data[i][k] - Seed[j][k];
					dist_tmp = dist_tmp + tmp*tmp ;
				}
	//			dist_tmp = sqrt(dist_tmp);			//目的是找最小目標, 故省下此步

				//找最小距離
				if(j == 0){
					dist = dist_tmp;
					argmin_dist = 0;
				}
				else{
					//找到更近的
					if( dist_tmp < dist ){
						dist = dist_tmp;
						argmin_dist = j;
					}
				}
			}//end for

			//找到之後做記錄
			c_result[i] = argmin_dist;
			c_size[argmin_dist] ++;			//cluster_size +1

		}//end clustering

	//	Form1->Memo1->Clear();
	//	for(int i=0;i<seed_count;i++){
	//		Form1->Memo1->Lines->Add(Seed[i][0]);
	//	}

		//step2. 更新群均值, 即計算各群之新質心
		for(int i=0;i<Kc;i++){
			for(int j=0;j<dimension;j++){
				Seed[i][j] = 0;
			}
		}

		for(int i=0;i<d_size;i++){
			int j = c_result[i];   //該筆資料的分群編號
			int size = c_size[j];  //該分群的大小
			for(int k=0;k<dimension;k++){
				Seed[j][k] = Seed[j][k] + Data[i][k]/size ;	 //分群中所有data的平均
			}
		}

		//step3. 計算 criterion, 看是否收斂
		//within-cluster sum of squares (WCSS):

		c_count++; 	//分群次數+1

		if(c_count > c_max_count ){		//超過分群最大次數, k-means結束
			Form1->Memo1->Lines->Add(c_count);
			Form1->Memo1->Lines->Add(wcss);
			Form1->Memo1->Lines->Add(prev_wcss - wcss);
			Form1->Memo1->Lines->Add("over");
			break;
		}
		else{
			//計算wcss
			wcss = 0;
			for(int i=0;i<d_size;i++){
				int j = c_result[i];   //該筆資料的分群編號
				double tmp;
				for(int k=0;k<dimension;k++){
					tmp = Data[i][k] - Seed[j][k];
					wcss = wcss + tmp*tmp;	 	 //目標資料與群質心之平方和
				}
			}

			Form1->Memo1->Lines->Add(c_count);
			Form1->Memo1->Lines->Add(wcss);
			Form1->Memo1->Lines->Add(prev_wcss - wcss);

			if(c_count == 1){				//第一次不撿查
				prev_wcss = wcss;
				continue;
			}
			else if(wcss == prev_wcss){		//收斂, k-means結束, break while
//				Form1->Memo1->Lines->Add(c_count);
//				Form1->Memo1->Lines->Add(wcss);
//				Form1->Memo1->Lines->Add(prev_wcss - wcss);
				Form1->Memo1->Lines->Add("end");
				break;
			}
	//		else if( abs(prev_wcss - wcss) <= cov_threshold ){	//若變動小於收斂門檻
	//			prev_wcss = wcss;
	//			cov_count++;				//收斂次數
	//			if( cov_count > cov_max_count ){
	//				Memo1->Lines->Add("ddd");
	//				break;
	//			}
	//		}
			else{                           //未收斂, 繼續算
				prev_wcss = wcss;
			}


		}

	}//end while

	/* end k-means++ */

	return wcss;

}


//int my_k_means2(
//	double **Data,		//input data
//	int d_size,         //number of data
//	int dimension,		//dimension of data
//	int Kc,             //maximum K clustering
//	int c_max_count
////	int cov_threshold;		//wcss變動小於此值則視為未變動(變動過小)
////	int cov_count;
////	int cov_max_count;		//wcss變動過小之次數超過此值時視為收斂
//	){
//
//	int dim = dimension;
//	double **mydata = Data;
//
//	/* 開始做k-means */
//	double wcss = 0, prev_wcss = 0, dist = 0;
//	int argmin_dist = 0;
//
//	//v0313新增
//	int initKc = Kc;
//	int min_wcss = 0;
//	int index_min_wcss = 0; // = argmin t
//
//	int Kcc_max = 11;	//3種中選最好
//	int Kcc_count = 0;	//3次中真正有分群的次數
//	// ( 1 + i*0.5 ), i = -1~1
//
//
//
//	for( int t=(0-Kcc_max/2) ; t<=(0+Kcc_max/2) ; t++){
//		//初始Kc限制
//		Kc = initKc * (1 + t*0.5);
//		if(Kc < 5){
//			Kc = 5;
//			if ( Kcc_count > 0) {
//				continue;		//K太小且已經算過分群, 就不做
//			}
//		}
//
//		int K = Kc;		//分K類
//		Form1->Memo1->Lines->Add("================");
//		Form1->Memo1->Lines->Add("K = "+WideString(K));
//
//		//initial Seed_label
//		int *Seed_label = 0;			//tmp using for k-means++ only
//		Seed_label = new int[d_size];
//		for(int i=0;i<d_size;i++){
//			Seed_label[i] = 0;			//set all label by 0
//		}
//
//		/** K-means++ **/
//		double** Seed = 0;
//		Seed = new double*[K];		//第k個種子之資料 pointer to Data
//		myZerosArray(*Seed, K);		//initlal Seed[i] = NULL
//
//		//找第一個seed
//		int seed_count = 0;      	//目前seed數
//		int first = random(d_size);
//		Seed_label[first] = 1;
//		Seed[seed_count] = mydata[first];
//		seed_count++;
//
//		//k-means++ 決定初始種子, 存放在Seed
//		k_means_plus(
//			mydata,			//input data
//			Seed_label,     //input seed_label of data
//			d_size,        	//size of data
//			dim,			//dimension of data
//			K,              //K clustering
//			Seed,      		//Seed be getting
//			&seed_count     //k cluster currently
//		);
//
//		/** K-means **/
//		//第一次時新配置記憶體tmp 供Seed[ ]存放種子資料(僅一次)
//		for(int i=0;i<seed_count;i++){
//			double *tmp = new double[dim];
//			for(int j=0;j<dim;j++){         //原Seed所指為Data資料, 不需delete
//				tmp[j] = Seed[i][j];		//將原指向的Seed[ ]之Data複製過去
//			}
//			Seed[i] = tmp;					//新配置
//		}
//		/******** 2014.11.23 補充 *******
//		* 當 seed_count < K 時
//		* 因 Seed 原配置為 double*[K],
//		* 故二維只配到 Seed[seed_count], Seed[seed_count]後面為NULL
//		* delete時, 當 i >= seed_count, Seed[i] 為NULL未配置第二維空間Seed[i][j], 故不需對Seed[i]做 delete
//		/********************************/
//
//		// 開始做k-means
//		int *c_size = new int[seed_count];		//實際分群大小 cluster_size, 不一定為K
//		int *d_result = new int[d_size];		//各data之分類結果(Seed編號)
//
//		int c_count = 0;      	   //分群次數
////		int c_max_count = 50;      //最大分群次數, 超過則強制收斂
//
//		int cov_threshold = 20;		//wcss變動小於此值則視為未變動(變動過小)
//		int cov_count = 0;
//	//	int cov_max_count = 10;		//wcss變動過小之次數超過此值時視為收斂
//
//		wcss = my_k_means(
//			mydata,		//input data
//			d_size,     //number of data
//			dim,		//dimension of data
//			seed_count, //K clustering
//			Seed,      	//Seed, input and output, := codewords
//			d_result,
//			c_size,
//
//			c_count,
//			c_max_count
//		//	int cov_threshold;		//wcss變動小於此值則視為未變動(變動過小)
//		//	int cov_count;
//		//	int cov_max_count;		//wcss變動過小之次數超過此值時視為收斂
//		);
//
//		/* end k-means++ */
//
//
//		//記錄資訊, 找最小wcss
//		if(Kcc_count == 0 || wcss < min_wcss){
//			min_wcss = wcss;
//			index_min_wcss = t;
//		}
//
//		Kcc_count++;
//
//		//release tmp array
//		delete[] c_size;
//		delete[] d_result;
//		delete[] Seed_label;
//		for(int i=0;i<seed_count;i++){
//			delete[] Seed[i];
//		}
//		delete[] Seed;
//	}
//
//
//	Kc = initKc * (1 + index_min_wcss*0.5);
//	return Kc;
//
//}



void __fastcall TForm1::Exit1Click(TObject *Sender)
{
//	ShowMessage(Application->Title);
	Application->Terminate();
}
//---------------------------------------------------------------------------



void __fastcall TForm1::Button16Click(TObject *Sender)
{
/*
My Documents
explorer ::{450D8FBA-AD25-11D0-98A8-0800361B1103}

Network Neighborhood
explorer ::{208D2C60-3AEA-1069-A2D7-08002B30309D}

Recycle Bin
explorer ::{645FF040-5081-101B-9F08-00AA002F954E}

My Computer
explorer ::{20D04FE0-3AEA-1069-A2D8-08002B30309D}

Control Panel
explorer ::{20D04FE0-3AEA-1069-A2D8-08002B30309D}\::{21EC2020-3AEA-1069-A2DD-08002B30309D}

Dial-Up Networking
explorer ::{20D04FE0-3AEA-1069-A2D8-08002B30309D}\::{992CFFA0-F557-101A-88EC-00DD010CCC48}

Printers
explorer ::{20D04FE0-3AEA-1069-A2D8-08002B30309D}\::{2227A280-3AEA-1069-A2DE-08002B30309D}

Scheduled Tasks
explorer ::{20D04FE0-3AEA-1069-A2D8-08002B30309D}\::{D6277990-4C6A-11CF-8D87-00AA0060F5BF}

Fonts
explorer ::{20D04FE0-3AEA-1069-A2D8-08002B30309D}\::{21EC2020-3AEA-1069-A2DD-08002B30309D}\::{D20EA4E1-3957-11d2-A40B-0C5020524152}
*/
	String Dir;
	if (SelectDirectory("選擇目錄", "", Dir))
		FileListBox1->Directory = Dir;
}
//---------------------------------------------------------------------------

//int trDim = 64;          //資料的維度           	//丟到.h

double **trData = 0;     //訓練資料的來源
int tr_d_size = 0;       //訓練資料的數量
int tr_d_dim = 0;        //訓練資料的維度
int *tr_d_result = 0;    //資料訓練後的分類

double **trCodeword = 0; //根據trData訓練好的codeword  (dim = tr_d_dim)
int tr_c_count = 0;		 //seed_count := codeword 共有幾個
int *tr_c_size = 0;      //第i群中的資料筆數
double *tr_c_std = 0;    //第i群中的距離標準差, 距離:=資料與codeword的距離
double *tr_c_mean = 0;   //第i群中的距離平均值, 距離:=資料與codeword的距離
double tr_c_max = 0;	 //任意兩codeword之最大距離

int tr_d_count = 0;	     //同tr_d_size, 但會update

double tr_dic_std = 0;
double tr_dic_mean = 0;
//TStringList *tdataList = 0;

//for whitening
double **tr_transMat = 0;
int tr_transMat_dim = 0;

//for new method
double **trVisualWord = 0;
double tr_word_count = 0;
double tr_word_dim = 0;

/************* 清空舊訓練資料 ***************/
void DataClear(){
	if(trData){				//訓練資料
		myDeleteArray2D(&trData, tr_d_size);
	}
	tr_d_size = 0;
	tr_d_dim = 0;
	myDeleteArray(&tr_d_result);

	if(trCodeword){			//codewords
		myDeleteArray2D(&trCodeword, tr_c_count);
	}
	tr_c_count = 0;
	myDeleteArray(&tr_c_size);
	myDeleteArray(&tr_c_std);
	myDeleteArray(&tr_c_mean);
	tr_c_max = 0;

	tr_d_count = 0;

	tr_dic_std = 0;
	tr_dic_mean = 0;

	if(tr_transMat){
		myDeleteArray2D(&tr_transMat, tr_transMat_dim);
	}
	tr_transMat_dim = 0;

	//清空舊訓練資料 for new method
	if(trVisualWord){			//codewords
		myDeleteArray2D(&trVisualWord, tr_word_count);
	}
	tr_word_count = 0;
	tr_word_dim = 0;
}

/************** training button *******************/
void __fastcall TForm1::Button18Click(TObject *Sender)
{
	clock_t start, end;
	OpenDialog1->InitialDir = ExtractFilePath(Application->ExeName);
	SaveDialog1->DefaultExt = "txt";
	TStringList *fdataList = 0, *tmpList = 0;
	String loadName, saveName, loadName2, loadName3;

	int training_type = RadioGroup1->ItemIndex ;
	double** mydata = 0;
	int d_size = 0, dim = 0;

	String s = "";

	//讀檔 initlal
	OpenDialog1->Filter = "txt|*.txt|bag of feature descriptor (*.bof)|*.bof|all file|*.*";
	OpenDialog2->Filter = "txt|*.txt|visual word dictionary (*.vw)|*.vw|all file|*.*|whitening trans mat (*.wmat)|*.wmat";
	if( training_type == 1){
		OpenDialog1->FilterIndex = 2;
		OpenDialog2->FilterIndex = 2;
	}
	else{
		OpenDialog1->FilterIndex = 1;
	}

	//讀檔
	if(OpenDialog1->Execute()){
		loadName = OpenDialog1->FileName;
	}
	else{
		loadName = NULL;
	}
	if(!FileExists(loadName))
		return;

	//讀檔2
	if(training_type == 1){
		if(OpenDialog2->Execute())
			loadName2 = OpenDialog2->FileName;
		else
			loadName2 = NULL;
		if(!FileExists(loadName2))
			return;
		//if whitening
		isWhitening = CheckBox7->Checked;
		if(isWhitening == true){
			OpenDialog2->FilterIndex = 4;

			if(OpenDialog2->Execute())
				loadName3 = OpenDialog2->FileName;
			else
				loadName3 = NULL;
			if(!FileExists(loadName3))
				return;
		}
	}

	//設定存檔
//	if(SaveDialog1->Execute()){
//		saveName = SaveDialog1->FileName;
//	}
//	else{
//		saveName = ExtractFilePath(Application->ExeName)+"train_data.txt";
//	}

	start = clock();
	Memo1->Lines->Add("檔案前置處理中...");
	Memo1->Lines->Add(loadName);

	/************* 清空舊訓練資料 ***************/
	DataClear();

	//testing button -> unenable
	Button21->Enabled = false;

	/** start */
	if( training_type == 0){
		//文本data轉入陣列
		fdataList = new TStringList;		//存放來源data
		fdataList->LoadFromFile(loadName);
		d_size = fdataList->Count;
		Memo1->Lines->Add("N = "+WideString(d_size));

		mydata = new double*[d_size];
		for(int i=0;i<d_size;i++)
			mydata[i] = 0;

		tmpList = new TStringList;          //轉換用暫存data
		tmpList->Delimiter = ' ';   		//設定切割符號

		for(int i=0;i<d_size;i++){
			tmpList->Clear();
			tmpList->DelimitedText = fdataList->Strings[i];		//將fdataList切割放入tmpList

			dim = tmpList->Count;			 //取得此筆資料維度 => 64維
			mydata[i] = new double[dim];
			for(int j=0;j<dim;j++){
				mydata[i][j] = tmpList->Strings[j].ToDouble();
			}
		}
		//文本轉換結束

		end = clock();
		Memo1->Lines->Add("...done! 前處理耗時): " + WideString(start-end));
		start = clock();
		Memo1->Lines->Add("\n訓練開始..");

		/** training1 - whitening */
		isWhitening = CheckBox7->Checked;
		if(isWhitening == true){
			Memo1->Lines->Add("Whitening Transform...");
			if(tr_transMat){
				myDeleteArray2D(&tr_transMat, tr_transMat_dim);
				tr_transMat_dim = 0;
			}
			myCreateArray2D(&tr_transMat, dim, dim);        			//2014.11.20 修正為 dim by dim
			tr_transMat_dim = dim;										//store "dim" as global

			//whitening
			myWhitening3(mydata, mydata, tr_transMat, d_size, dim);

			//test covariance
			double** cov_output;
			myCreateArray2D(&cov_output, dim, dim);
			getCovariance(mydata, cov_output, d_size, dim);
			output("cv.txt", cov_output, dim, dim, 2);
			myDeleteArray2D(&cov_output, dim);
			Memo1->Lines->Add("....end whitening");
		}
	}
	//read binary data
	else if( training_type == 1){
		/**
		*	data於建立BoF前已做whitening, 而目前不需對 BoF 做whitening, 故此button無需whitening處理
		*	但需讀取whitening transform mat 供testing使用
		*/
		ifstream rfs;

		/** (1). read BoF descriptor **/
		int BoF_count, BoF_dim;		//BoF_dim =: word_count

		rfs.open(loadName.c_str(), ios::in | ios::binary);
		//read header
		rfs.read((unsigned char*)&BoF_count, sizeof(BoF_count));	//read "BoF_count" as "d_size"
		rfs.read((unsigned char*)&BoF_dim, sizeof(BoF_dim));		//read "BoF_dim" as "ddim"
		//read data
		myCreateArray2D(&mydata, BoF_count, BoF_dim);
		for(int i=0;i<BoF_count;i++){
			rfs.read((unsigned char*)mydata[i], sizeof(mydata[i][0])*BoF_dim);
		}
		rfs.close();

		/** (2). read visual words **/   // ext = "*.vw"		//for testing, never use in this button
		int word_count, word_dim;

		rfs.open(loadName2.c_str(), ios::in | ios::binary);
		//read header
		rfs.read((unsigned char*)&word_count, sizeof(word_count));          //word_count = 1000?
		rfs.read((unsigned char*)&word_dim, sizeof(word_dim));              //word_dim = hosf_dim = 64
		//read data
		double** visual_word;
		myCreateArray2D(&visual_word, word_count, word_dim);
		for(int i=0;i<word_count;i++){
			rfs.read((unsigned char*)visual_word[i], sizeof(visual_word[i][0])*word_dim);
		}
		rfs.close();

		/** (3). read whiten trans mat **/   // ext = "*.wmat"		//for testing, never use in this button
		int wmt_dim;
		rfs.open(loadName3.c_str(), ios::in | ios::binary);
		//read header
		rfs.read((unsigned char*)&wmt_dim, sizeof(wmt_dim));	//read "BoF_count" as "d_size"
		rfs.read((unsigned char*)&wmt_dim, sizeof(wmt_dim));		//read "BoF_dim" as "ddim"
		//read mat
		double **wmat;		// 'wmt_dim' by 'wmt_dim' matrix
		myCreateArray2D(&wmat, wmt_dim, wmt_dim);
		for(int i=0;i<wmt_dim;i++){
			rfs.read((unsigned char*)wmat[i], sizeof(wmat[i][0])*wmt_dim);
		}
		rfs.close();

		/** (4). update global visual words for testing button **/
		trVisualWord = visual_word;
		tr_word_count = word_count;
		tr_word_dim = word_dim;

		tr_transMat = wmat;
		tr_transMat_dim = wmt_dim;									//store "mat" as global

		/** 提供參數與資料供後續k_means處理
		*	將新的BoF descritpor視為舊的cuboid descriptor去做分群
		*/
		d_size = BoF_count;
		dim = BoF_dim;          //BoF_dim := word_count = 1000?
	}

	/** training2 **/				//change 2014.11.22
	int K;
	if( training_type == 0){
		if(CheckBox5->Checked == true)
			K = Form1->Edit4->Text.ToInt();		//分K類
		else
			K = 1.5*sqrt((double)d_size);
	}
	else if( training_type == 1){
		//暫時保持一樣方法取K for BoF //2014.11.22
		if(CheckBox5->Checked == true)
			K = Form1->Edit4->Text.ToInt();		//分K類
		else
			K = 1.5*sqrt((double)d_size);
	}

	/** K-means++ **/
	double** Seed = 0;
	Seed = new double*[K];		//第k個種子之資料 pointer to Data, 只配置1維
	for(int i=0;i<K;i++){
		Seed[i] = 0;            //initlal Seed[i] = NULL
	}
	//建立Seed_label供k-means++使用
	int *Seed_label = 0;			//tmp using for k-means++ only
	myCreateArray(&Seed_label, d_size);
	//找第一個seed
	int seed_count = 0;      	//目前seed數
	int first = random(d_size);
	Seed_label[first] = 1;
	Seed[seed_count] = mydata[first];
	seed_count++;

	//k-means++ 決定初始種子, 存在Seed
	k_means_plus(
		mydata,			//input data
		Seed_label,     //input seed_label of data
		d_size,        	//size of data
		dim,			//dimension of data
		K,              //K clustering
		Seed,      		//Seed be getting
		&seed_count     //k cluster currently
	);
	myDeleteArray(&Seed_label);

	Memo1->Lines->Add("data_count = "+WideString(d_size));
	Memo1->Lines->Add("data_dim = "+WideString(dim));
	Memo1->Lines->Add("init K = "+WideString(K));
	Memo1->Lines->Add("seed_count = "+WideString(seed_count));

  /** k means **/
	//第一次時新配置記憶體tmp 供Seed[ ]存放種子資料(僅一次)		//2014.11.23 將tmp改為2維
	double **tmp_arr = new double*[seed_count];
	for(int i=0;i<seed_count;i++){
		tmp_arr[i] = new double[dim];
		for(int j=0;j<dim;j++){
			tmp_arr[i][j] = Seed[i][j];		//將原指向的Seed[i][j]之Data複製過去
		}
	}
	delete[] Seed;				//將上面找Seed時的1維配置delete掉
								//原Seed[i]所指為Data資料, 不需delete Seed[i]

	//set 'Seed' point to 'tmp'
	Seed = tmp_arr;					//新配置, 至此Seed與mydata為 '獨立記憶體空間', 不再共用記憶體
	tmp_arr = NULL;


	/******** 2014.11.23 補充 *******
	* 當 seed_count < K 時
	* 因 Seed 原配置為 double*[K],
	* 故二維只配到 Seed[seed_count], Seed[seed_count]後面為NULL
	* delete時, 當 i >= seed_count, Seed[i] 為NULL未配置第二維空間Seed[i][j], 故不需對Seed[i]做 delete
	*
	* *最新更新: 將tmp改為2維宣告, 使Seed不多佔空間
	/********************************/

	/** 開始做k-means **/
	int *c_size = new int[seed_count];		//分群大小 cluster_size
	int *d_result = new int[d_size];		//各data之分類結果(Seed編號)

	int c_count = 0;      	   //分群次數
	int c_max_count = 50;      //最大分群次數, 超過則強制收斂

	int cov_threshold = 20;		//wcss變動小於此值則視為未變動(變動過小)
	int cov_count = 0;
//	int cov_max_count = 10;		//wcss變動過小之次數超過此值時視為收斂

	my_k_means(
		mydata,		//input data
		d_size,     //number of data
		dim,		//dimension of data
		seed_count, //K clustering
		Seed,      	//Seed, input and output, := codewords
		d_result,
		c_size,

		c_count,
		c_max_count
	//	int cov_threshold;		//wcss變動小於此值則視為未變動(變動過小)
	//	int cov_count;
	//	int cov_max_count;		//wcss變動過小之次數超過此值時視為收斂
	);

	
	//群之標準差計算
	double *c_std = new double[seed_count];         //seed_count := 分K群
	double *sigma_dist = new double[seed_count];	//該群之距離合
	double *sigma_x2 = new double[seed_count];      //該群之距離平方合
	double *c_mean = new double[seed_count];
	for(int i=0;i<seed_count;i++){
		c_std[i] = 0;
		sigma_dist[i] = 0;
		sigma_x2[i] = 0;
		c_mean[i] = 0;
	}
	double tmp=0;
	int r;
	double dist2 = 0;
	for(int i=0;i<d_size;i++){
		r = d_result[i];
		dist2 = 0;
		for(int j=0;j<dim;j++){
			tmp = mydata[i][j]-Seed[r][j];
			tmp = tmp*tmp;                //tmp := Xk^2
			dist2 += tmp;

			sigma_x2[r] += tmp;           //第r群之距離平方合sigma(Xk^2), Xk為第k筆資料與群中心之距離
		}
		sigma_dist[r] += sqrt(dist2);	  //第r群之距離合
	}

	//codeword之平均&標準差
	for(int i=0;i<seed_count;i++){

		c_mean[i] = sigma_dist[i]/c_size[i];
//		Memo6->Lines->Add(sigma_x2[i]/c_size[i] - c_mean[i]*c_mean[i]);
//		Memo6->Lines->Add(sigma_x2[i]/c_size[i]);
//		Memo6->Lines->Add(c_mean[i]*c_mean[i]);
		double sigma_tmp = sigma_x2[i]/c_size[i] - c_mean[i]*c_mean[i];
		sigma_tmp = sigma_tmp >= 0 ? sigma_tmp : 0;		//避免float overflow造成負數
		c_std[i] = sqrt(sigma_tmp);

	}

	//v0309新增    //2014.08.21 updated
	//1. c_std = 0 去掉
	//2. c_size < 10 去掉
	//以 c_size = -1 表示被去除的分群
	int aaa = Edit10->Text.ToInt();
	if(CheckBox4->Checked == true){
		for(int i=0;i<seed_count;i++){
//			//若std為0
//			if( c_std[i] == 0){
//				c_size[i] = 0;
//			}
			//cluster size太小
			if( c_size[i] < aaa ){
				c_size[i] = -1;
				c_std[i] = -1;
			}
		}
	}



	//計算typical mean&std, 排序後取中間50%;
	double *tmp_mean = new double[seed_count];	//暫存資料以供排序
	double *tmp_std = new double[seed_count];
	double dmean = 0, dstd = 0;
	int tmp3 = 0;
	for(int i=0;i<seed_count;i++){              //複製到暫存陣列
		tmp_mean[i] = c_mean[i];
		tmp_std[i] = c_std[i];
	}
	qsort(tmp_mean, seed_count, sizeof(double), dcompare);
	qsort(tmp_std, seed_count, sizeof(double), dcompare);
	for(int i=seed_count*0.25;i<seed_count*0.75;i++){		//取中間50%之平均
		dmean = dmean + tmp_mean[i];
		dstd = dstd + tmp_std[i];
		tmp3++;
	}
	if(tmp3==0){
		dmean = 0;
		dstd = 0;
	}
	else{
		dmean = dmean/tmp3;
		dstd = dstd/tmp3;
	}
	delete[] tmp_mean;
	delete[] tmp_std;

	//計算tr_c_max
	double max_dst = 0;
	double tmp2;
	for(int i=0;i<seed_count-1;i++){
	for(int j=i+1;j<seed_count;j++){
		tmp2 = GetDist(Seed[i], Seed[j], dim);
		if(tmp2 > max_dst)
			max_dst = tmp2;
	}
	}



	Memo2->Clear();
	Memo2->Lines->Add("c_size: ");
	for(int i=0;i<seed_count;i++){
		Form1->Memo2->Lines->Add(c_size[i]);
	}
	Memo2->Lines->Add("mean: ");
	for(int i=0;i<seed_count;i++){
		Form1->Memo2->Lines->Add(c_mean[i]);
	}
	Memo2->Lines->Add("std: ");
	for(int i=0;i<seed_count;i++){
		Form1->Memo2->Lines->Add(c_std[i]);
	}

//	Form1->Memo2->Lines->Add("");
//	Form1->Memo2->Lines->Add("c_result");
//	for(int i=0;i<d_size;i++){
//		Form1->Memo2->Lines->Add(c_result[i]);
//	}

	//Seed(訓練好之群質心) 輸出
//	output(saveName, Seed, seed_count, dim);

  /* end k-means++ */


	//存放新訓練至全域變數, for testing
	trData = mydata;
//	trDim = dim;
	tr_d_size = d_size;
	tr_d_dim = dim;
	tr_d_result = d_result;

	tr_c_count = seed_count;	//實際分群大小K

	trCodeword = Seed;
	tr_c_std = c_std;
	tr_c_mean = c_mean;
	tr_c_size = c_size;
	tr_c_max = max_dst;

	tr_dic_std = dstd;
	tr_dic_mean = dmean;

	Button21->Enabled = true;	//testing button
	Button22->Enabled = false;   //refine button

	end = clock();
	Memo1->Lines->Add("training done! cost time: " + WideString(end-start));


	delete[] sigma_dist;
	delete[] sigma_x2;
//	delete[] c_std;
//	delete[] c_mean;

	//release all

	delete tmpList;
	delete fdataList;
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Button19Click(TObject *Sender)
{
	FileListBox1->Update();
}
//---------------------------------------------------------------------------


bool isTesting = false;
typedef enum {Undefined, Normal, Abnormal} Normality;
void __fastcall TForm1::Button21Click(TObject *Sender)
{
	static const String strNormality[3] = {"Undefined", "Normal", "Abnormal"};
	static CvScalar colorNormality[3];
	colorNormality[0] = CV_RGB(0,255,0);
	colorNormality[1] = CV_RGB(0,255,0);
	colorNormality[2] = CV_RGB(255,0,0);

	clock_t start, end, start2, end2;
	String s = "";


	//get filename and output_filename
	String initDir = ExtractFilePath(Application->ExeName);
	String filename =
		initDir +
//		"traffic-junction-sample.avi";
//		"tree.avi";
		 "Crowd-Activity-All.avi";

	if(FileListBox1->ItemIndex > -1){		//有從listbox1中選擇檔名
		filename = FileListBox1->FileName;
	}
	String outputPath, outputName, outputExt, outputFilename;

	String newDir = Edit7->Text + "\\";
//	String newDir = "output2\\";
	//	outputPath = initDir;
	outputPath = "D:\\myOutput\\";
	outputExt = ".bmp";  					    //輸出之副檔名

	outputName = ExtractFileName(filename);     //輸出Name
	outputName = ChangeFileExt(outputName, ""); //將Name去掉副檔名

	newDir = newDir + outputName + "\\";

	CvCapture *cam = cvCaptureFromFile(		//建立影片來源
		filename.t_str()
	);

	Memo1->Lines->Add(filename);
	start2 = clock();

	//影片資訊
	double frameWidth, frameHeight, frameFPS, frameCount, frameCurrent;

	Label9->Caption  = frameWidth 	= cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_WIDTH);
	Label10->Caption = frameHeight  = cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_HEIGHT);
	Label11->Caption = frameFPS 	= cvGetCaptureProperty(cam, CV_CAP_PROP_FPS);
	Label12->Caption = frameCount 	= cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_COUNT);
	Label15->Caption = frameCurrent = cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES);
//	ShowMessage(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

	assert( NULL != cam ) ;

	IplImage *frame = 0,
		*img_prev = 0,*frame_cur = 0, *img_curr = 0, *img_res = 0,
		*img_output = 0,
		*img_soc = 0, *img_off = 0,
		*img_eig = 0, *img_temp = 0, *pyr_prev = 0, *pyr_cur = 0,
		*img_back = 0;

	IplImage **frames = 0, **images = 0;
	MyParticle2D** optical_flow = 0; 	    //計算粒子光流所暫存的影像pixel之光流
	MyParticle2D** optical_flow_field = 0;  //根據3x3x3 median所求得之粒子光流場
	MyParticle2D** social_force_field = 0;  //根據粒子光流場所求得之社會力場

	//2014.11.23 new method
	double **visual_word = trVisualWord;
	int word_count = tr_word_count;		//Edit11->Text	//決定要取幾個words

//	double **whiten_trans = tr_transMat;	//whiten trans mat


	int soc_count = 0;	//soc_field計算次數


	/* 初始參數設定 */
	int end_of_video = cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_COUNT);
//	int pscale = 3;   	//粒子大小, NxN
	int pscale = Edit3->Text.ToInt();   	//粒子大小, NxN
	int half_s = pscale/2;	// 3/2 = 1
	double motion_th = Edit2->Text.ToDouble();
	int frame_gap = Edit9->Text.ToInt();

	//若distance為6, 則初始frames使用0~5
	int distance = Edit1->Text.ToInt();	//distance between two frames
	int cur = distance; 				//current frame count

	//for social force
	int range_k = 7;	//for getting social force
	int range_l = 7;    //for getting social force
	double range_b = sqrt(range_k*range_k + range_l*range_l);

	//for cuboid
	int cb_dim = 64;
	int cb_n = 4;	//cuboid size = 2n x 2n x 2m
	int cb_m = 4;
	TStringList *dataList = new TStringList;

	/* get frames */
	frames = new IplImage*[frameCount];
	images = new IplImage*[frameCount];			//grey image of frames
	for(int i=0;i<frameCount;i++){
		frames[i] = 0;
		images[i] = 0;
	}

	frame = cvQueryFrame(cam) ;   		//initial, 將此frame count視為 -1
	CvSize img_sz = cvGetSize(frame);
	Label15->Caption = cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES);

	//取得初始所需影像 pscale + dist
	// 3+6 = 9 frames
	//建立0~8 frames, images
	int init_frame_number = pscale+distance;
	for(int i=0;i<init_frame_number;i++){
		//第二張frame開始跳格
		if(i>0){
			for(int tt=0;tt<frame_gap+1;tt++){
				frame = cvQueryFrame(cam);
			}
		}
		else{
			frame = cvQueryFrame(cam);
		}

		frames[i] = cvCloneImage(frame);

		//convert the image to grey image
		//轉灰階故images需以單通道建立
		images[i] = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
		cvCvtColor( frames[i], images[i], CV_BGR2GRAY);
	}
//	ShowMessage(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

	//1.建立0~2張影像之光流
	//2.建立第0個optical flow field, 存放計算好的初始光流( 2維(x,y)向量 )
	//3.建立第0個social force field, 存放計算好的SF( 2維(x,y)向量 )
	//cuboid, NxNxM
	int m = 2000;	//預定處理之最大畫格數
	int off_width = img_sz.width/pscale;
	int off_height= img_sz.height/pscale;
	int off_size = off_width*off_height; //存放粒子的空間大小

	optical_flow = new MyParticle2D*[pscale];   			//影像光流
	optical_flow_field = new MyParticle2D*[m/pscale];       //粒子之光流場
	social_force_field = new MyParticle2D*[m/pscale];       //粒子之社會力場

	//for testing
	double Th_N = Edit5->Text.ToDouble(); 		//Threshold_Normal
	int *ap_mapping = new int[off_size];		//做testing之8鄰居時mapping之用
	TStringList *zList = new TStringList;	  	//記錄ap的z-value
	TStringList *zheadList = new TStringList; 	//記錄ap的z-value-head
	TStringList *z_average_List = new TStringList;

	//for output;
	Normality normality = Undefined;
	int output_count = cb_m*pscale;		//當前偵測clip之起始frame
	int output_clip = 0;  	 			//當前偵測clip (build by 24 frames)
	CvPoint label_p1 = cvPoint(5,5);	//label size
	CvPoint label_p2 = cvPoint(40,12);
	newDir = newDir + WideString(tr_c_count) + "\\" + WideString(Th_N) + "\\";
	outputFilename = outputPath + newDir + outputName;






	// MyParticle2D 預設為0, 不需清空
	for(int i=0;i<pscale;i++){
		optical_flow[i] = new MyParticle2D[img_sz.width*img_sz.height];
	}
	for(int i=0;i<m/pscale;i++){
		optical_flow_field[i] = 0;
		social_force_field[i] = 0;
	}
	optical_flow_field[0] = new MyParticle2D[off_size];		//ps. off可改為一項即可, 因計算完soc即無需使用
//	social_force_field[0] = new MyParticle2D[off_size];

	MyParticle2D p1, p2;

	// 建立exp_r
	start = clock();
//	int **r = new int*[off_size];
	double **exp_r = 0;
	exp_r = new double*[off_size];
	for(int i=0;i<off_size;i++){
//		r[i] = new int[off_size];
		exp_r[i] = new double[off_size];
	}

	for(int p1j=0;p1j<off_width;p1j++){
	for(int p1k=0;p1k<off_height;p1k++){
		for(int p2j=0;p2j<off_width;p2j++){
		for(int p2k=0;p2k<off_height;p2k++){
			//座標還原 粒子unit->pixel unit
			p1 = MyParticle2D(p1j*pscale+half_s, p1k*pscale+half_s);
			p2 = MyParticle2D(p2j*pscale+half_s, p2k*pscale+half_s);

//			r[p1j*off_height+p1k][p2j*off_height+p2k] =
//				sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y))+0.5;

			//tmpdist為p1,p2之間的距離
			double tmpx = (p2.x-p1.x);
			double tmpy = (p2.y-p1.y);
			double tmpdist = sqrt( tmpx*tmpx + tmpy*tmpy );

			//算exp
			exp_r[p1j*off_height+p1k][p2j*off_height+p2k] =
				exp( 0-((tmpdist-pscale)/range_b) );
		}
		}
	}
	}
	/* end expr 計算 */

	end = clock();
	Memo1->Lines->Add("建expr所花時間: ");
	Memo1->Lines->Add(end-start);

	/** 設定結束 **/



	/** 開始處理 **/
	while ( 1 )
	{
//		if(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

		//use webcam
//      CvCapture* cam = cvCaptureFromCAM( CV_CAP_ANY ) ;

		/*初始設定*/
		const int win_size = 10 ;
		Label14->Caption = cur;
		int prev = cur-distance;    //取得前一張影像的編號, 初始6-6=0

		/************/

		//create a imge for displaying result
		//清白以顯示向量場
		img_res = cvCreateImage(img_sz, IPL_DEPTH_8U, 3) ;
		for ( int y = 0 ; y < img_sz.height ; ++y )
		{
			uchar* ptr = (uchar*)( img_res->imageData + y * img_res->widthStep ) ;
			for ( int x = 0 ; x <img_res->widthStep; ++x )
			{
				ptr[x] = 0 ;
			}
		}
		img_off = cvCreateImage(img_sz, IPL_DEPTH_8U, 3) ;
		for ( int y = 0 ; y < img_sz.height ; ++y )
		{
			uchar* ptr = (uchar*)( img_off->imageData + y * img_off->widthStep ) ;
			for ( int x = 0 ; x <img_off->widthStep; ++x )
			{
				ptr[x] = 0 ;
			}
		}
		img_soc = cvCreateImage(img_sz, IPL_DEPTH_8U, 3) ;
		for ( int y = 0 ; y < img_sz.height ; ++y )
		{
			uchar* ptr = (uchar*)( img_soc->imageData + y * img_soc->widthStep ) ;
			for ( int x = 0 ; x <img_soc->widthStep; ++x )
			{
				ptr[x] = 0 ;
			}
		}
		img_back = cvCreateImage(img_sz, IPL_DEPTH_8U, 3) ;
		for ( int y = 0 ; y < img_sz.height ; ++y )
		{
			uchar* ptr = (uchar*)( img_back->imageData + y * img_back->widthStep ) ;
			for ( int x = 0 ; x <img_back->widthStep; ++x )
			{
				ptr[x] = 0 ;
			}
		}

		/* optical flow 計算 */

		//get previous image
		img_prev = cvCreateImage(img_sz, IPL_DEPTH_8U, 1) ;
		cvCvtColor( frames[prev], img_prev, CV_BGR2GRAY);

		//get current image
		img_curr = cvCreateImage(img_sz, IPL_DEPTH_8U, 1) ;
		cvCvtColor( frames[cur], img_curr, CV_BGR2GRAY);

		//get good features
		img_eig = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
		img_temp = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
		int corner_count = MAX_CORNERS ;
		CvPoint2D32f*  features_prev = new CvPoint2D32f[MAX_CORNERS] ;

		char feature_found[ MAX_CORNERS ] ;
		float feature_errors[ MAX_CORNERS ] ;

		//case.1 傳統特徵萃取
		if(CheckBox2->Checked == false){
			cvGoodFeaturesToTrack(
				img_prev,
				img_eig,
				img_temp,
				features_prev,
				&corner_count,
				0.01,
				5.0,
				0,
				3,
				0,
				0.4
			);

			cvFindCornerSubPix(
				img_prev,
				features_prev,
				corner_count,
				cvSize(win_size,win_size),
				cvSize(-1,-1),
				cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,20,0.03)
			);

			// L-K
		   //	CvSize pyr_sz = cvSize( frame->width + 8 ,frame->height / 3 ) ;
			pyr_prev = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
			pyr_cur = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
			CvPoint2D32f*  features_cur = new CvPoint2D32f[ MAX_CORNERS ] ;

			cvCalcOpticalFlowPyrLK(
				img_prev,
				img_curr,
				pyr_prev,
				pyr_cur,
				features_prev,
				features_cur,
				corner_count,
				cvSize(win_size,win_size),
				3,
				feature_found,
				feature_errors,
				cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,20,0.3),
				0
			);
			delete[] features_cur;
		} // end case1

////////////////////////////////
//砍掉重練
//每一行各做一次LK
//h行則做h次
//優點: 可避免coners過多, 且分多次尋找效率相同
//
//case.2 get our own features
//3x3 patch
//		int pscale = 3;
//		int half_s = pscale/2;	// 3/2 = 1

clock_t t1, t2;

		if(CheckBox2->Checked == true){

			int h = frame->height;
			int w = frame->width;
			int imgsize = w*h;

			// L-K
			pyr_prev = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
			pyr_cur = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
			CvPoint2D32f* features_cur = new CvPoint2D32f[ MAX_CORNERS ] ;

			//for 3 frames
			start = clock();
			for(int t=0;t<pscale;t++){
				//取相對之新影像, clip size=3
				if(t>0){
					if( img_prev ){
						cvReleaseImage( &img_prev );
					}
					if( img_curr ){
						cvReleaseImage( &img_curr );
					}

					img_prev = cvCreateImage(img_sz, IPL_DEPTH_8U, 1) ;
					cvCvtColor( frames[prev+t], img_prev, CV_BGR2GRAY);

					img_curr = cvCreateImage(img_sz, IPL_DEPTH_8U, 1) ;
					cvCvtColor( frames[cur+t], img_curr, CV_BGR2GRAY);
				}

				t1 = clock();
				//60*320=20000;

				//取第i行各點計算LK
				int ignore_height = Edit6->Text.ToInt();
				for(int i=0 ; i<frame->height ; i++ ){

					//忽視某高度以內光流 -> 忽視normal&abnormal label
					if(i<=ignore_height){
//						optical_flow[t][row*w+col].x = 0;	//MyParticle預設為0可忽略
//						optical_flow[t][row*w+col].y = 0;
						continue;
					}

					corner_count = 0;

					//取得第i行之座標點放入features_prev
					for(int j=0 ; j<frame->width ; j++ ){
						float x = j;
						float y = i;

						//features_prev[corner_count] = cvPoint2D32f(x, y);
						features_prev[corner_count].x = x;
						features_prev[corner_count].y = y;
						corner_count++;

						if(corner_count > MAX_CORNERS)
							break;
					}

					//對第i行做LK
					cvCalcOpticalFlowPyrLK(
						img_prev,     			//前影像
						img_curr,               //目前影像
						pyr_prev,
						pyr_cur,
						features_prev,          //前特徵點
						features_cur,           //所找出的對應點
						corner_count,
						cvSize(win_size,win_size),
						3,
						feature_found,			//是否找到
						feature_errors,         //errors rate
						cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,20,0.3),
						0
					);

					CvPoint pt_prev;
					CvPoint pt_cur;
					int row, col;
					//check & show
					for ( int k = 0 ; k < corner_count ; k++)
					{
						if ( 0 == feature_found[k] || feature_errors[k] > 550 )
						{
//							printf("error is %f \n" , feature_errors[k] ) ;
							continue ;
						}
//						printf("find it !\n") ;

						//找到並存入optical_flow[]
//						pt_prev = cvPoint( features_prev[k].x , features_prev[k].y ) ;
//						pt_cur = cvPoint( features_cur[k].x , features_cur[k].y ) ;
						pt_prev.x = features_prev[k].x;
						pt_prev.y = features_prev[k].y;
						pt_cur.x = features_cur[k].x;
						pt_cur.y = features_cur[k].y;

						row = pt_prev.y;
						col = pt_prev.x;
						optical_flow[t][row*w+col].x = pt_cur.x - pt_prev.x;
						optical_flow[t][row*w+col].y = pt_cur.y - pt_prev.y;

						//show optical flow
						if( CheckBox1->Checked == true ){
							if( (pt_prev.x == pt_cur.x) && (pt_prev.y == pt_cur.y) ){
								continue;
							}
						}

						optical_flow[t][row*w+col].CalculateLength();
						int c = optical_flow[t][row*w+col].length/5.0 * 255;
						if(c>255)
						  c=255;

//						cvLine( img_res,pt_prev,pt_cur,CV_RGB( 255,0,0),1 );
//						cvLine( img_res,pt_prev,pt_prev,CV_RGB( 64,0,0),1 );
						cvLine( img_res,pt_cur,pt_cur,CV_RGB( c,0,0),1 );

					}//end save optical flow


				}
				t2 = clock();

				Memo1->Lines->Add(s + "影像[" + prev + "-" + t + "] 光流所花時間: " + String(t2-t1));
//				Memo1->Lines->Add(t2-t1);

			}//end 3 time (pscale)
			end = clock();
			Memo1->Lines->Add("影像光流總花時間: "+ String(end-start));
//			Memo1->Lines->Add(end-start);

			delete[] features_cur;
			// End L-K


			start = clock();
			/* median by 3x3x3 into optical flow filed */
			int *median_x = new int[pscale*pscale*pscale];
			int *median_y = new int[pscale*pscale*pscale];
			int median_count = 0;
			int row=0, col=0;
			int pp = prev/pscale;	//因粒子大小為pscale, pp為影像所對應的粒子場編號
									//即粒子在時間軸上之編號
			optical_flow_field[pp] = new MyParticle2D[off_size];


			for( int y=0 ; y<h ; y+=pscale ){			//y+=3
			for( int x=0 ; x<w ; x+=pscale ){			//x+=3
				if( y>h || x>w )
					continue;

				median_count = 0;
				//以(y+1, x+1, t+1)為中心將3x3x3個pixel之光流放入陣列, 以qsort排序
				for( int t=0 ; t<pscale ; t++ ){
				for( int i=y ; i<y+pscale ; i++ ){
				for( int j=x ; j<x+pscale ; j++ ){
					median_x[median_count] = optical_flow[t][i*w+j].x;
					median_y[median_count] = optical_flow[t][i*w+j].y;
					median_count++;
				}
				}
				}
				qsort (median_x, median_count, sizeof(int), compare);
				qsort (median_y, median_count, sizeof(int), compare);

				//計算&&存放 optical flow into field
				//以粒子為中心, 將算出的中間值光流和放入所對應的粒子光流場位置(row, col)
				// x=>col
				// 0=>0, 3=>1, 6=>2;
				row = y/pscale;
				col = x/pscale;
				optical_flow_field[pp][row*off_width+col].x = median_x[median_count/2];
				optical_flow_field[pp][row*off_width+col].y = median_y[median_count/2];

//				String s = "";
//				Memo1->Lines->Add(s+row+", "+col);

				//根據門檻值去除motion較小的粒子
				optical_flow_field[pp][row*off_width+col].CalculateLength();
				double magnitude = optical_flow_field[pp][row*off_width+col].length;
				if(magnitude < motion_th){
					optical_flow_field[pp][row*off_width+col].x = 0;
					optical_flow_field[pp][row*off_width+col].y = 0;
				}
				else{
					int tmpx = optical_flow_field[pp][row*off_width+col].x;
					int tmpy = optical_flow_field[pp][row*off_width+col].y;

	//				if( abs(tmpx) >= 2 || abs(tmpy)>=2 )
	//					Memo1->Lines->Add(s+optical_flow_field[prev][row*off_width+col].x+", "
	//									+optical_flow_field[prev][row*off_width+col].y);

					CvPoint pt_prev = cvPoint(x+half_s, y+half_s);
					CvPoint pt_cur = cvPoint(x+half_s+tmpx, y+half_s+tmpy);
					cvLine( img_off,pt_prev,pt_cur,CV_RGB( 255,0,0),1 );
					cvLine( img_off,pt_prev,pt_prev,CV_RGB( 128,128,128),1 );
				}


			}
			}

			/* end median */
			end = clock();
			Memo1->Lines->Add("3影像median總花時間: " + String(end-start));
//			Memo1->Lines->Add(end-start);


			delete[] median_x;
			delete[] median_y;

		}//end if checked



		///////////// social force field //////////////
		start = clock();

		//以粒子為中心的其他粒子之相對位置
		int pp = prev/pscale;	//因粒子大小為pscale, pp為影像所對應的粒子場編號
								//即粒子在時間軸上之編號

		social_force_field[pp] = new MyParticle2D[off_size];

		double mass = 1;
		int disp_k = ((2*range_k+1)/pscale); // ((2*7+1)/3) = 5
		int disp_l = ((2*range_l+1)/pscale);
		//相對位置範圍 -2~2
		int start_k = 0-disp_k/2;			 // 從-2開始
		int start_l = 0-disp_l/2;
		int end_k = start_k+disp_k;			 // 到2為止
		int end_l = start_l+disp_l;

		double f_sum_x = 0;  // sum of force
		double f_sum_y = 0;  // sum of force
		for(int row=0;row<off_height;row++){
		for(int col=0;col<off_width;col++){
			//inactive之粒子不計算soc
			if(optical_flow_field[pp][row*off_width+col].x == 0
				&& optical_flow_field[pp][row*off_width+col].y == 0){
				continue;
			}

			//對active particle計算其soc
			f_sum_x = 0;
			f_sum_y = 0;
			for(int i=start_l;i<=end_l;i++){	//-2~2
			for(int j=start_k;j<=end_k;j++){
				//若為中心粒子則不計算, 因距離影響為0
				if(i==0&&j==0)
					continue;

				//取相對粒子之真實座標
				int ii = row+i;
				int jj = col+j;
				//超過影像範圍之不存在的粒子, 不計算(視為0) //暫定
				if( ii<0 || ii>=off_height || jj<0 || jj>=off_width ){
					continue;
				}
				//所對應的動量太小之particle不列入soc計算 ( 存off時已設為(0,0) )
				if(optical_flow_field[pp][ii*off_width+jj].x == 0
					&& optical_flow_field[pp][ii*off_width+jj].y == 0 ){
					continue;
				}

				//排除上述粒子, 計算以(row, col)為中心之social force
				//exp_r[p1j*off_height+p1k][p2j*off_height+p2k]
				double tmp_exp = exp_r[row*off_height+col][ii*off_height+jj];
				f_sum_x +=
					mass*tmp_exp*
					(
						optical_flow_field[pp][ii*off_width+jj].x
						-optical_flow_field[pp][row*off_width+col].x
					);
				f_sum_y +=
					mass*tmp_exp*
					(
						optical_flow_field[pp][ii*off_width+jj].y
						-optical_flow_field[pp][row*off_width+col].y
					);
			}
			}


			social_force_field[pp][row*off_width+col].x = (int)(f_sum_x+0.5);
			social_force_field[pp][row*off_width+col].y = (int)(f_sum_y+0.5);
//			Memo1->Lines->Add(s + social_force_field[prev][row*off_width+col].x
//				+", "+social_force_field[prev][row*off_width+col].y);
//
//			Memo1->Lines->Add(s + (int)f_sum_x
//				+", "+(int)f_sum_y);


			//output result
			CvPoint pt_prev = cvPoint( col*pscale+half_s, row*pscale+half_s ) ;
			CvPoint pt_cur = cvPoint( pt_prev.x+social_force_field[pp][row*off_width+col].x,
										pt_prev.y+social_force_field[pp][row*off_width+col].y ) ;

//			if(abs(optical_flow_field[prev][row*off_width+col].x) >= 2
//			 || abs(optical_flow_field[prev][row*off_width+col].y) >=2 ){

				cvLine( img_soc,pt_prev,pt_cur,CV_RGB( 255,0,0),1 );
				cvLine( img_soc,pt_prev,pt_prev,CV_RGB( 128,128,128),1 );
//			}
		}
		}
		soc_count++;
		/* end social force */
		end = clock();
		Memo1->Lines->Add("social force所花時間: "+String(end-start));
//		Memo1->Lines->Add(end-start);

//		cvSub(img_prev, img_curr, img_back);

		//建cuboid&data
		bool b_soc = false;
		bool b_overlapping = true;	//true or false控制是否overlap

		//檢查是否有足夠的soc_count去建立cuboid
		if( b_overlapping )
			b_soc = (soc_count >= 2*cb_m);									//overlapping
		else
			b_soc = (soc_count%(2*cb_m)==0) && (soc_count>0) ;				//non-overlapping by temporal

		if( b_soc ){
			start = clock();
			int t_center = soc_count-cb_m;	//cb中心
			int cb_start_x, cb_start_y, cb_start_z;  				//計算cb之起始點
			double soc_direct;
			int bin;
			double sx, sy;
			int cb_count=0;
			double *cb_hist = new double[cb_dim];
			TStringList *ap_List = new TStringList;			//記錄ap的64維資料
			TStringList *ap_loc_List = new TStringList;		//記錄ap的座標
			double z_average = 0;

			//清空 index for AP mapping, 用來記錄有哪些粒子是AP, -1表示非AP
			for(int i=0;i<off_size;i++){
				ap_mapping[i] = -1;
			}

			//找active particle
			for(int row=0;row<off_height;row++){
			for(int col=0;col<off_width;col++){
				//inactive particle跳過
				if(optical_flow_field[t_center][row*off_width+col].x == 0
					&& optical_flow_field[t_center][row*off_width+col].y == 0){
					continue;
				}
				//範圍不足以建cuboid之particle
				if(row<cb_n || row>off_height-cb_n
					|| col<cb_n || col>off_width-cb_n)
					continue;

				//找到active並建立8 sub-coboid
				cb_start_x = col-cb_n;
				cb_start_y = row-cb_n;
				cb_start_z = t_center-cb_m;
				static int locx[8]={0,1,0,1,0,1,0,1};
				static int locy[8]={0,0,1,1,0,0,1,1};
				static int locz[8]={0,0,0,0,1,1,1,1};
				int xx, yy, zz;

				//陣列歸0
				for(int i=0;i<cb_dim;i++){
					cb_hist[i] = 0;
				}

				for(int scb=0;scb<8;scb++){
					for(int k=0;k<cb_m;k++){
					for(int j=0;j<cb_n;j++){
					for(int i=0;i<cb_n;i++){
						zz = cb_start_z+locz[scb]*cb_m+k;
						yy = cb_start_y+locy[scb]*cb_n+j;
						xx = cb_start_x+locx[scb]*cb_n+i;

						sx = social_force_field[zz][yy*off_width+xx].x;
						sy = social_force_field[zz][yy*off_width+xx].y;
//						Memo2->Lines->Add(WideString(":::")+WideString(sx));
						//計算soc向量方向
						soc_direct = cvFastArctan(sy, sx);
						//cvFastArctan之誤差當(1,1)(-1,-1), accurate is about 0.3
						if( sx == sy )
							soc_direct += 0.3;
						//判斷bin
						if( soc_direct < 45 )
							bin = 0;
						else if( soc_direct < 90 )
							bin = 1;
						else if( soc_direct < 135 )
							bin = 2;
						else if( soc_direct < 180 )
							bin = 3;
						else if( soc_direct < 225 )
							bin = 4;
						else if( soc_direct < 270 )
							bin = 5;
						else if( soc_direct < 315 )
							bin = 6;
						else
							bin = 7;

						cb_hist[scb*8+bin] += sqrt(sx*sx+sy*sy);
					}
					}
					}

				}
				/* normalization */
				double cb_max = cb_hist[0];		//max magnitude of cuboid
				double cb_min = cb_hist[0];		//max magnitude of cuboid
				for(int i=1;i<cb_dim;i++){     		//search max
					if(cb_hist[i] > cb_max)
						cb_max = cb_hist[i];
					if(cb_hist[i] < cb_min)
						cb_min = cb_hist[i];                                    //2014.11.02 updated
				}
				//normalize to 0~1
				if(cb_max != 0){
					for(int i=0;i<cb_dim;i++){
						cb_hist[i] = (cb_hist[i] - cb_min)/(cb_max - cb_min);	//2014.11.02 updated
					}
				}
				/*end normalize*/

				//記錄 ap之data
				String s = "";
				s = s + cb_hist[0];
				for(int i=1;i<cb_dim;i++){
					s = s + " " + cb_hist[i];
				}
				ap_List->Add(s);
				//記錄 ap之座標(col,row)
				ap_loc_List->Add(WideString(col)+WideString(",")+WideString(row));

				//a index of AP mapping, 存的值表示後方做鄰居處理時, 所存的陣列index
				ap_mapping[row*off_width+col] = cb_count;

				cb_count++;

			}
			}
			delete[] cb_hist;

			//若ap太少
			if(cb_count < 3){
				//NORMAL
				normality = Undefined;
				zList->Add(s+"clip: " + t_center + " 之cuboid總數: " + cb_count);
				zheadList->Add(s+"clip: " + t_center + " 之cuboid總數: " + cb_count);
				z_average = -1;
				Memo5->Lines->Add(z_average);
			}
			else{
				//取得frame中所有ap後, 再從List中存放回data陣列 (上述步驟是計算HoSF&統計AP數量)
				TStringList *tmpList = new TStringList;  //轉換用暫存data
				double **ap_data = new double*[cb_count];
				for(int i=0;i<cb_count;i++)
					ap_data[i] = 0;

				CvPoint *ap_loc = new CvPoint[cb_count];
				for(int i=0;i<cb_count;i++){
					ap_loc[i].x = 0;
					ap_loc[i].y = 0;
				}

				int *ap_result = new int[cb_count];

				int ap_dim = 0;
				//將ap_List轉換並存入ap_data
				tmpList->Delimiter = ' ';   		//設定切割符號
				for(int i=0;i<cb_count;i++){
					tmpList->Clear();
					tmpList->DelimitedText = ap_List->Strings[i];		//將ap_List切割放入tmpList

					ap_dim = tmpList->Count;			 //取得此筆資料維度 => 64維
					ap_data[i] = new double[ap_dim];
					for(int j=0;j<ap_dim;j++){
						ap_data[i][j] = tmpList->Strings[j].ToDouble();
					}
				}
				//將ap_loc_List轉換並存入ap_loc
				tmpList->Delimiter = ',';   		//設定切割符號
				for(int i=0;i<cb_count;i++){
					tmpList->Clear();
					tmpList->DelimitedText = ap_loc_List->Strings[i];		//將ap_loc_List切割放入tmpList

					ap_loc[i].x = tmpList->Strings[0].ToDouble();    //col
					ap_loc[i].y = tmpList->Strings[1].ToDouble();    //row
				}
//				Memo4->Lines->Add(ap_List->Text);
//				Memo4->Lines->SaveToFile(ExtractFilePath(Application->ExeName)+"apdata.txt");


				/************ whitening transform ************************
				 * 將 ap_data 做 whitening
				 **********************************************************/
				isWhitening = CheckBox7->Checked;
				if(isWhitening == true){
					//whitening transform
					doWhitenPatch(ap_data, ap_data, tr_transMat, cb_count, ap_dim);
				}

				int training_type = RadioGroup1->ItemIndex; // 0=old, 1=new

				if(training_type == 0){

					////// z-value-head //////////
					double *z_value = new double[cb_count];
					double *z_value_head = new double[cb_count];

					/****************************************************************************
					//training data (global)
					//double **trCodeword = 0; //訓練好的codeword
					//int tr_c_count = 0;	   //seed_count 共分成幾群
					//int *tr_c_size = 0;      //第i群中的資料筆數
					//double *tr_c_std = 0;    //第i群中的距離標準差, 距離:=資料與codeword的距離
					//double *tr_c_mean = 0;   //第i群中的距離平均值, 距離:=資料與codeword的距離
					*****************************************************************************/

					double z, min;
					int argcode;  		//argCodeword
					int neighbor_index, neighbor_count, mapx, mapy;
					int	dirx[8] = {-1, 0, 1,-1, 1,-1, 0, 1};	//8鄰居, 除了(0,0)以外
					int diry[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
					double median_z[9];
					//找AP最相似的codeword, 為AP做分類
					zList->Add(s+"clip: " + t_center + " 之cuboid總數: " + cb_count);
					for(int i=0;i<cb_count;i++){
						argcode = 0;
						min = GetDist(ap_data[i], trCodeword[0], ap_dim);	//ap_dim = 64
						double tmp;
						for(int j=1;j<tr_c_count;j++){
							if(tr_c_size[j] == -1)	//v0308, 若為被去除的分群則跳過
								continue;

							tmp = GetDist(ap_data[i], trCodeword[j], ap_dim);
							//找到最小
							if(tmp < min){
								argcode = j;
								min = tmp;
							}
						}
						ap_result[i] = argcode; //記錄所分類的群編號
	//					Memo3->Lines->Add(argcode);

						//calc. z-value
						if(tr_c_std[argcode] == 0){ 	 //若sigma == 0, 令其為0.001   //0821 updated
														 //v0308後, 此條件應永不成立, 因前面for時已篩選過, 除非分群的std結果為0
							//z = 0;
							tr_c_std[argcode] = 0.001;
						}
	//					else{
							z = fabs( (min-tr_c_mean[argcode]) / tr_c_std[argcode] );
	//					}

	//					if(tr_c_size[argcode] == 0 || tr_c_std[argcode] == 0)
	//						Memo6->Lines->Add("123");

						z_value[i] = z;
						zList->Add(z);
					}
					//ps. 上下兩段for不可合一起, 下段會出錯, 因鄰居控制會超出限制

					zheadList->Add(s+"clip: " + t_center + " 之cuboid總數: " + cb_count);

					//對所有AP計算z-value-head
					for(int i=0;i<cb_count;i++){

						z = z_value[i];
						//calc. z_head, 公式(13)	v0604.新修為median
						if( z >= Th_N){
							double tmp_sigma = 0;
							double pp_dist, cc_dist;

							double gamma = 0.5;
							double cc_max = tr_c_max;		//公式(13)中的 max

							neighbor_count = 0;				//AP之鄰居數

							//計算鄰居影響	//已改為8鄰居AP取median, by 2013.6.4
							for(int j=0;j<8;j++){
								mapx = ap_loc[i].x + dirx[j];   //col
								mapy = ap_loc[i].y + diry[j];	//row
								//check if neightbor out of range
								if( mapx<0 || mapx>=off_width || mapy<0 || mapy>=off_height )
									continue;

								//找8鄰居於ap陣列中的index
								neighbor_index = ap_mapping[mapy*off_width+mapx];		//取得mapping index

								//若鄰居是inactive point (非AP)
								if( neighbor_index == -1 )
									continue;

	//以下是舊方法
	//							//若兩點的codeword相同, 不列入計算 ( boolean Ci != Cj )
	//							if( argcode == ap_result[neighbor_index])
	//								continue;
	//							//計算兩點所屬codeword之距離
	//							cc_dist = GetDist(trCodeword[argcode], trCodeword[ap_result[neighbor_index]], trDim);
	//							//pp_dist以粒子距離為單位
	//							pp_dist = (ap_loc[i].x-ap_loc[neighbor_index].x)*(ap_loc[i].x-ap_loc[neighbor_index].x)
	//									+ (ap_loc[i].y-ap_loc[neighbor_index].y)*(ap_loc[i].y-ap_loc[neighbor_index].y);
	//							pp_dist = sqrt(pp_dist);
	//
	//							tmp_sigma = tmp_sigma + exp( cc_dist/(cc_max*gamma*pp_dist) );
	//end

								//v0604. 8-neightbor median
								//將鄰居值存入陣列, 等待排序
								median_z[neighbor_count] = z_value[neighbor_index];
								neighbor_count++;
							}

							//若無鄰居, 則z_head = ThN;
							if(neighbor_count == 0){
								z_value_head[i] = Th_N;
							}
							else{
	//old							z_value_head[i] = z * tmp_sigma / neighbor_count;


	//new						//加入自己後, 取median
								median_z[neighbor_count] = z;
								neighbor_count++;

								//median_z[0]是double, 故compare需用double compare
								qsort(median_z, neighbor_count, sizeof(double), dcompare);

								//若總數為even, 取中間兩位數之平均
								int mid = (neighbor_count >> 1);			//mid = neightbor_count/2
								if( (neighbor_count%2) == 0 ){
									z_value_head[i] = 0.5 * ( median_z[mid-1]+median_z[mid] );
								}
								else{
									z_value_head[i] = median_z[mid];
								}
							}

						}
						else{	 					  //if z<Th_N
							z_value_head[i] = z;
						}


						zheadList->Add(z_value_head[i]);

					}//end z-value-head for all AP per frame

					zList->SaveToFile(initDir + "test_z.txt");
					zheadList->SaveToFile(initDir + "test_zhead.txt");
	//				Memo2->Lines->Add(zheadList->Text);


					//Abnormal Detection
					z_average = 0;
					for(int k=0;k<cb_count;k++){
						z_average += z_value_head[k];
					}
					z_average = z_average/(double)cb_count;

					delete[] z_value;
					delete[] z_value_head;
				}
				// 2014.11.23 new method
				else if(training_type == 1){

					int dataSize = cb_count;
					int dataDim = ap_dim;
					int wordCount = word_count;

					//(1). ap_data 根據 visual word 生成 BoF per frame
					//每讀一筆data做分類並記錄出現次數
					double* dist = new double[wordCount];		//??? = size of visual words = 1000?
					double* BoF_DES = new double[wordCount];		//bag of feature descriptor
//					myZerosArray(BoF_DES, wordCount);
					for(int i=0;i<wordCount;i++)
						BoF_DES[i] = 0;

					for(int i=0;i<dataSize;i++){
						double *data = ap_data[i];

						//whitening transform
						//前面已對ap_data做過whitening故不需再做

						//計算data與各words的距離
						for(int j=0;j<wordCount;j++){
							dist[j] = GetDist(data, visual_word[j], dataDim);
						}
						//取得距離最近的分類
						int index_min = argmin(dist, wordCount);
						//記錄分類次數
						BoF_DES[index_min]++;
					}

					Series1->Clear();
					Series1->AddArray(BoF_DES, (wordCount-1));
					//normalization
					int tmpN = Normalize(BoF_DES, wordCount);
 //					Memo2->Lines->Add("normalize = " + WideString(tmpN));

					zList->Add(s+"clip: " + t_center + " 之cuboid總數: " + cb_count);

					//(2). bof -> zvalue
					//找BoF最相似的codeword, 為BoF做分類
					int index_min = 0;
					int c_dim = wordCount;	//cluster codeword dimension

					double dist_min = GetDist(BoF_DES, trCodeword[0], c_dim);
					double tmp;
					for(int j=1;j<tr_c_count;j++){
						if(tr_c_size[j] == -1)	//v0308, 若為被去除的分群則跳過
							continue;

						tmp = GetDist(BoF_DES, trCodeword[j], c_dim);
						//找到最小
						if(tmp < dist_min){
							index_min = j;       //記錄所分類的群編號
							dist_min = tmp;
						}
					}

					// calc. z-value
					if(tr_c_std[index_min] == 0){ 	 //若sigma == 0, 令其為0.001   //0821 updated
													 //v0308後, 此條件應永不成立, 因前面for時已篩選過, 除非分群的std結果為0
						//z = 0;
						tr_c_std[index_min] = 0.001;
					}
//					else{
						z_average = fabs( (dist_min-tr_c_mean[index_min]) / tr_c_std[index_min] );
//					}
//					if(tr_c_size[argcode] == 0 || tr_c_std[argcode] == 0)
//						Memo6->Lines->Add("123");


					//finsih per frame
					Memo1->Lines->Add("dataSize = "+WideString(dataSize));
					delete[] dist; dist=NULL;
					delete[] BoF_DES; BoF_DES=NULL;
				}

				/** Abnormal Detection */
				if(z_average < Th_N){
					//Normal
					normality = Normal;
				}
				else{
					//Abnormal
					normality = Abnormal;
				}

				Memo5->Lines->Add(z_average);

				//////////////////
				for(int i=0;i<cb_count;i++)
					delete[] ap_data[i];
				delete[] ap_data;
				delete[] ap_loc;
				delete[] ap_result;

			}
			end = clock();

			//output
			z_average_List->Add( WideString(output_clip) + WideString("\t") + WideString(z_average) );
			for(int t=0;t<pscale;t++){
				img_output = frames[output_count];
				if( img_output ){
					//check and creat output dir
					if(!DirectoryExists(outputPath+newDir)){	 				//若output path不存在, 則建立dir
						if(!ForceDirectories(outputPath+newDir)){				//建立多層目錄
							Memo1->Lines->Add("Cannot create directory!");      //若建立dir失敗
							Memo1->Lines->Add(outputPath+newDir);
						}
					}
//					else{
//						Memo1->Lines->Add("dir ok");
//					}

					//output and save images
					String out_tmp = outputFilename + "_" + output_clip
								 + "_" + output_count + "_" + strNormality[normality] + outputExt ;
					Memo1->Lines->Add(out_tmp.t_str());

					//save result image
					cvRectangle( img_output, label_p1, label_p2, colorNormality[normality], CV_FILLED);
					cvSaveImage( out_tmp.t_str(), img_output );

					//save result log
					z_average_List->SaveToFile(outputFilename + "_z_average.txt");

					//release image
					cvReleaseImage(&frames[output_count]);
					frames[output_count] = 0;
				}
				else{
					Memo1->Lines->Add("error output at frame "+WideString(output_count+t));
				}

				output_count++; 	 //無論output成功與否都要執行
			}
			output_clip++;


			Memo1->Lines->Add("****");
			Memo1->Lines->Add(s+"clip: " + t_center + " 之cuboid總數: " + cb_count);
//			Memo1->Lines->Add(cc);
			Memo1->Lines->Add("建data&testing所花時間: " + String(end-start));
//			Memo1->Lines->Add(end-start);
			Memo1->Lines->Add("============");
			Memo1->Lines->SaveToFile(outputFilename + "_readme.txt");
			Memo2->Lines->SaveToFile(outputFilename + "_readme2.txt");

			delete ap_List;
			delete ap_loc_List;
		}


		//////////////// show ////////////////
		const char* window_prev = "img_prev" ;
		const char* window_curr = "img_curr" ;
		const char* window_res = "result" ;
		const char* window_soc = "soc" ;
		const char* window_off = "optical_flow_field" ;
//		const char* window_dist = "dist" ;

		cvNamedWindow( window_prev );
		cvNamedWindow( window_curr,CV_WINDOW_AUTOSIZE );
		cvNamedWindow( window_res );
		cvNamedWindow( window_soc );
		cvNamedWindow( window_off );
//		cvNamedWindow( window_dist );
		cvShowImage( window_prev, img_prev );
		cvShowImage( window_curr, img_curr );
		cvShowImage( window_res, img_res );
		cvShowImage( window_soc, img_soc );
		cvShowImage( window_off, img_off );
//		cvShowImage( window_dist, img_back );

		int wt = 33;
		if(CheckBox3->Checked==true)
			wt = 0;

		char opt = cvWaitKey( wt ) ;
		if ( 27 == opt )
		{
			break ;
		}

		cvReleaseImage( &img_curr );
		cvReleaseImage( &img_prev );
		cvReleaseImage( &img_res );
		cvReleaseImage( &img_soc );
		cvReleaseImage( &img_off );

		cvReleaseImage( &img_eig );
		cvReleaseImage( &img_temp );

		delete[] features_prev;
//		delete[] features_cur;

		//取得新進影像
		//get next n frame, n=pscale
		//case.1
		cur = cur + pscale;

		for(int t=0;t<pscale;t++){
//			cvReleaseImage( &frames[prev+t] );		//改由output result時再釋放, 但會剩前幾張未釋放留到最後
			cvReleaseImage( &images[prev+t] );
			images[prev+t] = 0;

			//get new frames
			//frame跳格, 至少執行一次
			for(int tt=0;tt<frame_gap+1;tt++){      // +1 !!!!!!!!!
				frame = cvQueryFrame(cam);
				if( !frame ){
					break;
				}
			}
			if( !frame ){
				break;
			}

			frames[cur+t] = cvCloneImage(frame);
			//convert the image to grey image
			images[cur+t] = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
			cvCvtColor( frames[cur+t], images[cur+t], CV_BGR2GRAY);
		}

		if( !frame ){
			end2 = clock();
			Memo1->Lines->Add("end of video!");
			Memo1->Lines->Add( "total cost time: " + WideString(end2-start2) + " (ms)." );
			Memo1->Lines->SaveToFile(outputFilename + "_readme.txt");
			break;
		}

	}//end while


	//release all
	for(int i=0;i<frameCount;i++){			//避免漏網之魚, 因frames前少數幾張未釋放
//		Memo3->Lines->Add(i);
		if( frames[i] ){
//			Memo3->Lines->Add("f "+WideString(i));
			cvReleaseImage( &frames[i] );
		}
		if( images[i] ){
//			Memo3->Lines->Add("i "+WideString(i));
			cvReleaseImage( &images[i] );
		}
	}
	delete[] frames;
	delete[] images;

	for(int i=0;i<m/pscale;i++){
		if(optical_flow_field[i] == NULL)
			break;
		delete[] optical_flow_field[i];
		delete[] social_force_field[i];
	}
	delete[] optical_flow_field;
	delete[] social_force_field;
	delete[] ap_mapping;

	for(int i=0;i<off_size;i++){
		delete[] exp_r[i];
	}
	delete[] exp_r;

	delete dataList;
	delete zList;
	delete zheadList;
	delete z_average_List;


	cvReleaseCapture( &cam );
	cvDestroyAllWindows() ;
}
//---------------------------------------------------------------------------
double GetMaxDist( double **codeword, int count, int dim ){
	//計算tr_c_max
	double max_dst = 0;
	double tmp2;
	for(int i=0;i<count-1;i++){
	for(int j=i+1;j<count;j++){
		tmp2 = GetDist(codeword[i], codeword[j], dim);
		if(tmp2 > max_dst)
			max_dst = tmp2;
	}
	}

    return max_dst;
}

/***************************************************************
int trDim = 64;          //資料的維度
double **trData = 0;     //訓練的資料來源
int tr_d_size;           //訓練的資料數量
int *tr_d_result = 0;    //資料訓練後的分類

double **trCodeword = 0; //訓練好的codeword
int tr_c_count = 0;		 //seed_count 共分成幾群
int *tr_c_size = 0;      //第i群中的資料筆數
double *tr_c_std = 0;    //第i群中的距離標準差, 距離:=資料與codeword的距離
double *tr_c_mean = 0;   //第i群中的距離平均值, 距離:=資料與codeword的距離
double tr_c_max = 0;	 //任意兩codeword之最大距離
****************************************************************/

// Refine data
void __fastcall TForm1::Button22Click(TObject *Sender)
{
//	clock_t start, end;
//	OpenDialog1->InitialDir = ExtractFilePath(Application->ExeName);
//	SaveDialog1->DefaultExt = "txt";
//	TStringList *fdataList = 0, *tmpList = 0, *tdataList = 0;
//	String loadName, saveName;
//
//	double** mydata = 0;
//	int d_size;
//	int dim = 64;
//
//	String s = "";
//
//	//讀檔
//	if(OpenDialog1->Execute()){
//		loadName = OpenDialog1->FileName;
//	}
//	else{
//		loadName = NULL;
//	}
//	if(!FileExists(loadName))
//		return;
//
//	//設定存檔
//	if(SaveDialog1->Execute()){
//		saveName = SaveDialog1->FileName;
//	}
//	else{
//		saveName = ExtractFilePath(Application->ExeName)+"refine_data.txt";
//	}
//
//	start = clock();
//	Memo1->Lines->Add("檔案前置處理中...");
//
//	//清空舊訓練資料
////	if(trData){				//訓練資料
////		for(int i=0;i<tr_d_size;i++){
////			if(trData[i])
////				delete[] trData[i];
////		}
////		delete[] trData;
////	}
////	if(tr_d_result)         //分類結果
////		delete[] tr_d_result;
////
////	if(trCodeword){			//codewords
////		for(int i=0;i<tr_c_count;i++){
////			if(trCodeword[i])
////				delete[] trCodeword[i];
////		}
////		delete[] trCodeword;
////	}
////	if(tr_c_size)        	//分群大小
////		delete[] tr_c_size;
////	if(tr_c_std)        		//第i群中距離標準差
////		delete[] tr_c_std;
////	if(tr_c_mean)        		//
////		delete[] tr_c_mean;
//
//	/** start */
//
//	//文本data轉入陣列
//	fdataList = new TStringList;		//存放來源data
//	fdataList->LoadFromFile(loadName);
//	d_size = fdataList->Count;
//
//	mydata = new double*[d_size];
//	for(int i=0;i<d_size;i++)
//		mydata[i] = 0;
//
//	tmpList = new TStringList;          //轉換用暫存data
//	tmpList->Delimiter = ' ';   		//設定切割符號
//
//	tdataList = new TStringList;        //訓練好之輸出data
//
//	for(int i=0;i<d_size;i++){
//		tmpList->Clear();
//		tmpList->DelimitedText = fdataList->Strings[i];		//將fdataList切割放入tmpList
//
//		dim = tmpList->Count;			 //取得此筆資料維度 => 64維
//		mydata[i] = new double[dim];
//		for(int j=0;j<dim;j++){
//			mydata[i][j] = tmpList->Strings[j].ToDouble();
//		}
//	}
//	//文本轉換結束
//
//
//	int total = 0;	//total = tr_d_size;
//	for(int i=0;i<tr_c_count;i++){
//		total += tr_c_size[i];
//	}
//
//	int cb_count = d_size;
//	double **ap_data = mydata;
//
//	double Th_z = 2.0;	//Threshold_Normal
//	double z, min;
//	int argcode;  		//argCodeword
//
//	double* codeword;
//	double sigma_x2, std, mean, inver_size, beta;
//	int size, c_count;
//
//	bool case_c;
//	TStringList *rr = new TStringList;
//	//refine data
//	int ttt = 0;
//	Memo1->Lines->Add("Refine data processing...");
//	for(int i=0;i<cb_count;i++){
//		if(i%1000 == 0){
//			Memo1->Lines->Add(i);
//			Memo3->Lines->Add(ttt);
//			Memo2->Lines->Add(tr_c_count);
//			ttt = 0;
//		}
//		//找AP最相似的codeword, 計算z-value
//		argcode = 0;
//		min = GetDist(ap_data[i], trCodeword[0], trDim);	//trDim = 64
//		double tmp;
//		for(int j=1;j<tr_c_count;j++){
//			tmp = GetDist(ap_data[i], trCodeword[j], trDim);
//			//找到最小
//			if(tmp < min){
//				argcode = j;
//				min = tmp;
//			}
//		}
//		if(tr_c_std[argcode] == 0){ 	 //若sigma == 0
//			z = 0;
//		}
//		else{
//			z = fabs( (min-tr_c_mean[argcode]) / tr_c_std[argcode] );
//		}
//		rr->Add(z);
//		//end zvalue
//
//		/*****************
//		* refine step.
//		*****************/
//		case_c = false;
//
//		//case a, b, c共用
//		codeword = trCodeword[argcode];		//codeword is pointer to pointer
//		size = tr_c_size[argcode];
//		mean = tr_c_mean[argcode];
//		std = tr_c_std[argcode];
//		inver_size = 1.0/(size+1);			//  1/(n+1)
//		//update
//		sigma_x2 = size*(std*std+mean*mean);
//		sigma_x2 = sigma_x2 + min*min;
//		mean = (size*mean+min)*inver_size;				//mean_i
//		std =  sqrt(sigma_x2*inver_size-mean*mean); 	//std_i; mean is updated one
//
//		//case a.
//		if( z < Th_z ){
//			//update f(P) to Ci
//			for(int j=0;j<dim;j++){
//				codeword[j] = (size*codeword[j] + ap_data[i][j]) * inver_size;
//			}
//
//			if(size > sqrt(total)){
//				//直接更新
//				size = size + 1;
//			}
//			else{
//				//if 資料量過少, goto c
//				//argcode = argcode, then goto c.
//				case_c = true;
//			}
//		}
//		//case b.
//		else{
//			ttt++;
//			/***************************************************************
//			double **trCodeword = 0; //訓練好的codeword
//			int tr_c_count = 0;		 //seed_count 共分成幾群
//			int *tr_c_size = 0;      //第i群中的資料筆數
//			double *tr_c_std = 0;    //第i群中的距離標準差, 距離:=資料與codeword的距離
//			double *tr_c_mean = 0;   //第i群中的距離平均值, 距離:=資料與codeword的距離
//			double tr_c_max = 0;	 //任意兩codeword之最大距離
//			****************************************************************/
//			double **tmpCodeword = 0;
//			int *tmp_size = 0;
//			double *tmp_std = 0, *tmp_mean = 0;
//
//			//產生新的cluster k, goto c
//			//此段code是為了新增array[K+1]取代array[K]以達到新增data之目的
//			int new_count = tr_c_count + 1;			//new_K = K+1
//			tmpCodeword = trCodeword;               //原訓練資料做暫存
//			tmp_size = tr_c_size;
//			tmp_std = tr_c_std;
//			tmp_mean = tr_c_mean;
//
//			trCodeword = new double*[new_count];	//宣告新的陣列取代舊的
//			tr_c_size = new int[new_count];
//			tr_c_std = new double[new_count];
//			tr_c_mean = new double[new_count];
//
//			//複製舊資料
//			for(int j=0;j<new_count-1;j++){			//將舊值複製到新的陣列
//				trCodeword[j] = tmpCodeword[j];		//trCodeword為指標複製
//				tr_c_size[j] = tmp_size[j];         //以下三個為value複製
//				tr_c_std[j] = tmp_std[j];
//				tr_c_mean[j] = tmp_mean[j];
//			}
//			delete[] tmpCodeword;                   //delete 舊陣列
//			delete[] tmp_size;
//			delete[] tmp_std;
//			delete[] tmp_mean;
//
//			//newly added data at last index
//			argcode = new_count-1;	//j=k           //取得最後一筆codeword的index
//
//			trCodeword[argcode] = new double[dim];
//			for(int j=0;j<dim;j++){
//				trCodeword[argcode][j] = ap_data[i][j]; 		//update new codeword
//																//因為是新cluster, 故只有一筆data
//																//直接複製當做seed
//			}
//			tr_c_size[argcode] = 0;
//			tr_c_std[argcode] = 0;
//			tr_c_mean[argcode] = 0;
//
//			tr_c_count = new_count;					//update total number of data
//
//			//for goto c
//			case_c = true;			//goto c
//		}
//		//case c.
//		if( case_c ){
//			/***************************
//			//double tr_dic_std = 0;
//			//double tr_dic_mean = 0;
//			****************************/
//			size = tr_c_size[argcode] + 1;
//
//			beta = (double)size/sqrt(total);
//
//			mean = beta*mean + (1-beta)*tr_dic_mean;
//			std = beta*std + (1-beta)*tr_dic_std;
//		}
//
//		//update dictionary; codeword在case中已更新
//		tr_c_mean[argcode] = mean;
//		tr_c_std[argcode] = std;
//		tr_c_size[argcode] = size;
////		tr_c_max = GetMaxDist(trCodeword, tr_c_count, trDim);		//目前因codeword數量過大而過慢
//
//		/** end refine */
//	}
//	rr->SaveToFile(init+"rrr.txt");
//	delete rr;
//
//	for(int i=0;i<d_size;i++){
//		delete[] mydata[i];
//	}
//	delete[] mydata;
//
//	delete tdataList;
//	delete tmpList;
//	delete fdataList;


}
//---------------------------------------------------------------------------


void __fastcall TForm1::Button15Click(TObject *Sender)
{
	FileListBox1->Directory = "C:\\Users\\Ronnie\\Videos\\test4";
}
//---------------------------------------------------------------------------


void __fastcall TForm1::Button7Click(TObject *Sender)
{
//	FileListBox1->Directory = "C:\\Users\\Ronnie\\Videos\\test6_half";
	DirectoryListBox1->Directory = "C:\\Users\\Ronnie\\Videos\\test6_half";

}
//---------------------------------------------------------------------------

void __fastcall TForm1::Button25Click(TObject *Sender)
{

//	TMemo **memolist = new TMemo*[10];
	TMemo **memolist = new TMemo*[10];
	memolist[0] = Memo1;
	memolist[1] = Memo2;

	memolist[0]->Clear();
	memolist[1]->Clear();

//	Memo1->Clear();
//	Memo2->Clear();
	Memo3->Clear();
	Memo4->Clear();
	Memo5->Clear();
	Memo6->Clear();
	Memo7->Clear();

	Series1->Clear();

	delete[] memolist;
}
//---------------------------------------------------------------------------

void __fastcall TForm1::CheckBox5Click(TObject *Sender)
{
	if(CheckBox5->Checked)
		Edit4->Enabled = true;
	else
		Edit4->Enabled = false;
}
//---------------------------------------------------------------------------

void __fastcall TForm1::CheckBox6Click(TObject *Sender)
{
	if(CheckBox6->Checked == true)
		FileListBox1->Mask = "*.jpg;*.bmp;*.tif";
	else
		FileListBox1->Mask = "*.wmv;*.avi;*.mpg";
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Button26Click(TObject *Sender)
{
	DirectoryListBox1->Directory = "C:\\Users\\Ronnie\\Videos\\ucsd";
//	DirectoryListBox1->Directory = "D:\\dataset\\UCSD\\UCSD_Anomaly_Dataset\\UCSD_Anomaly_Dataset\\UCSDped1";
}
//---------------------------------------------------------------------------

//
//void getCovariance(double **src, double **dst, int dim, int n, double mean = 0){
//
//	double sum = 0;
//
//	for(int i=0;i<dim;i++){                // matrix size is dim x dim
//		for(int j=0;j<dim;j++){
//			sum = 0;
//			for(int k=0;k<n;k++){
//				sum = sum + (src[i][k]-mean)*(src[j][k]-mean);
//			}
//			dst[i][j] = sum/n;
//		}
//	}
//
//}
void myMatMul(double **src1, double **src2, double **dst, int a, int b, int c, double mean = 0){

	double sum = 0;

	for(int i=0;i<a;i++){                // matrix size is [a x b] [ b x c ] = [ a c ]
		for(int j=0;j<c;j++){
			sum = 0;
			for(int k=0;k<b;k++){
				sum = sum + src1[i][k]*src2[k][j];
			}
			dst[i][j] = sum;
		}
	}

}

double getMean(double *src, int n){

	if( n == 0 ) return 0;

	double sum = 0;
	for(int i=0;i<n;i++){
		sum += src[i];
	}
	sum /= n;

	return sum;

}

void myZeroMean2D(double **src, int size, int dim, double *mean = NULL){

	double *tmp_mean;

//	if(mean == NULL){
	if(true){
		tmp_mean = new double[dim];
		//initialization
		for(int j=0;j<dim;j++){
			tmp_mean[j] = 0;
		}
		//get sum by size
		for(int i=0;i<size;i++){
			for(int j=0;j<dim;j++){
				tmp_mean[j] += src[i][j];
			}
		}
		//get mean
		for(int j=0;j<dim;j++){
			tmp_mean[j] /= size;
		}
	}

	//zero-mean
	for(int i=0;i<size;i++){
		for(int j=0;j<dim;j++){
			src[i][j] -= tmp_mean[j];
		}
	}

	//copy tmp_mean back to mean[]
	for(int j=0;j<dim;j++){
		mean[j] = tmp_mean[j];
	}

	delete[] tmp_mean;

	return;
}
















void __fastcall TForm1::CheckBox7Click(TObject *Sender)
{
	isWhitening = CheckBox7->Checked;
}
//---------------------------------------------------------------------------



void __fastcall TForm1::Button1Click(TObject *Sender)
{
	double a[5] = {1,1,0.1,0.1,0.1};
//	double c[5] = {7,6,5,4,3};
	double b1[5];
	double b2[5];

	double f[5] = {5,7,0.3,0.4,0.6};

	double g[5] = {5,7,3,4,6};

	double c[12] = {0, 0, 0.150684931506849, 0, 0, 0, 0.123287671232877, 0, 0, 0, 1, 0,};
	double **c2 = new double*[3];
	for(int i=0;i<3;i++)
		c2[i] = new double[12];
	double *c3 = new double[12];

	for(int i=0;i<12;i++){
	  c2[0][i] = c[i];
	  c2[1][i] = c[11-i];
	  Memo1->Lines->Add(c2[1][i]);
	}
	Memo1->Lines->Add("===============");


//	double x = 0;
//	int m = argmin(f, 5, &x );
//	Memo2->Lines->Add(m);
//	Memo2->Lines->Add(x);

	Normalize(g, 5);
	for(int i=0;i<5;i++){
		Memo2->Lines->Add(g[i]);
	}



	int size = 0;
	int dim = 12;

			int dataCount = 12;
			ofstream wfs;//
			wfs.open("test222.bin", ios::out | ios::binary);
			wfs.seekp(0, ios::beg);
//			wfs.write((const unsigned char*)&size, sizeof(size));	//write "data size"
//			wfs.write((const unsigned char*)&dim, sizeof(dim));		//write "data dimension"

//			wfs.write((unsigned char*)a, sizeof(a));
			wfs.write((unsigned char*)c, sizeof(c[0])*dim);
			wfs.write((unsigned char*)c2[1], sizeof(c2[0][0])*dim);


//			wfs.seekp(0, ios::beg);
//			wfs.write((const unsigned char*)&size, sizeof(size));	//write "data size"

			wfs.close();

			ifstream rfs("test222.bin", ios::in | ios::binary);
			rfs.read((unsigned char*)c3, sizeof(c3[0])*dim);
			for(int i=0;i<12;i++)
				Memo1->Lines->Add(c3[i]);
			Memo1->Lines->Add("-----------------");

			rfs.read((unsigned char*)c3, sizeof(c3[0])*dim);
			for(int i=0;i<12;i++)
				Memo1->Lines->Add(c3[i]);
			Memo1->Lines->Add("-----------------");

				//
			rfs.close();



	int dd;
//	int ccc = pow(2,30);
//	Memo1->Lines->Add(ccc);

//			ifstream rs("test111.bin", ios::in | ios::binary);

			//read data size
//			rs.read((unsigned char*)&dd, sizeof(dd));		//write "data size"

			//get start position of data
//			int p1 = rs.tellg();

			//seek to end of data
//			rs.seekg(0, rs.end);

			//get data count
//			int size = (rs.tellg() - p1) / (sizeof(b1));
//			Memo1->Lines->Add(size);

//			rs.read((unsigned char*)b1, sizeof(b1));
//			rs.read((unsigned char*)b2, sizeof(b2));



//			rs.close();
//			Memo1->Lines->Add(dd);

//			for(int i=0;i<5;i++)
//				Memo1->Lines->Add(b1[i]);
//
//			for(int i=0;i<5;i++)
//				Memo1->Lines->Add(b2[i]);



}
//---------------------------------------------------------------------------


void __fastcall TForm1::FileListBox1DblClick(TObject *Sender)
{
		//   if(FileListBox1->ItemIndex > -1)
		//	   Memo1->Lines->Add(FileListBox1->ItemIndex);

}
//---------------------------------------------------------------------------







void __fastcall TForm1::Button2Click(TObject *Sender)
{

	clock_t start, end, start2, end2;
	Memo1->Clear();
	Memo2->Clear();

	start = clock();
	int a;
	int t = 100000000;
	for(int i=0;i<t;i++){

		a = sizeof(int);
//		int tmp = random(100);
//
//		if(tmp < 80){
//			Memo2->Lines->Add(tmp);
//			i--;
//
//			continue;
//		}
//		else
//			Memo3->Lines->Add(i);
//
//		Memo1->Lines->Add(tmp);

	}
	end = clock();
	Memo1->Lines->Add(a);
	Memo1->Lines->Add(end-start);
	Memo1->Lines->Add("===============");


	start = clock();

	for(int i=0;i<t;i++){

		a = sizeof(int)*64;
	}
	Memo1->Lines->Add(a);
	end = clock();
	Memo1->Lines->Add(end-start);


}
//---------------------------------------------------------------------------


void __fastcall TForm1::Button3Click(TObject *Sender)
{

	String loadName, saveName;

	//讀檔
	if(OpenDialog1->Execute()){
		loadName = OpenDialog1->FileName;
	}
	else{
		loadName = NULL;
		return;
	}

    int word_count, data_dim;

	ifstream sf_rs(loadName.c_str(), ios::in | ios::binary);
	sf_rs.seekg(0, sf_rs.beg);
	sf_rs.read((const unsigned char*)&word_count, sizeof(word_count));		//write "word_count"
	sf_rs.read((const unsigned char*)&data_dim, sizeof(data_dim));

	double **visual_word = new double*[word_count];			//visual_word留到結束時realese by 2014.11.20
	for (int i = 0; i < word_count; i++) {
		visual_word[i] = new double[data_dim];
		for(int j=0;j<data_dim;j++)
			visual_word[i][j] = 0;
	}

	TStringList *dataList = new TStringList;		//存放來源data
	for(int i=0;i<word_count;i++){
		sf_rs.read((const unsigned char*)visual_word[i], sizeof(visual_word[i][0])*data_dim);		//write "words"
		//記錄data	//stack
		String s = "";
		s = s + visual_word[i][0];
		for(int j=1;j<data_dim;j++){
			s = s + " " + visual_word[i][j];
		}
		dataList->Add(s);

	}
	dataList->SaveToFile("D:\\myProjects\\103\\bcb2010_cv2.1_baseProject_20141121\\Debug\\training_bin_1\\a1_training_5\\a1_visual_word.txt");

	sf_rs.close();
	delete dataList;

	//return;

	/** start */
	//讀檔
	if(OpenDialog1->Execute()){
		loadName = OpenDialog1->FileName;
	}
	else{
		loadName = NULL;
		return;
	}

	//文本data轉入陣列
	TStringList *fdataList = new TStringList;		//存放來源data
	fdataList->LoadFromFile(loadName);
	int d_size = fdataList->Count;
	Memo1->Lines->Add("N = "+WideString(d_size));

	double **mydata = new double*[d_size];
	for(int i=0;i<d_size;i++)
		mydata[i] = 0;

	TStringList *tmpList = new TStringList;          //轉換用暫存data
	tmpList->Delimiter = ' ';   		//設定切割符號

	TStringList *tdataList = new TStringList;        //訓練好之輸出data

	int dim;
	for(int i=0;i<d_size;i++){
		tmpList->Clear();
		tmpList->DelimitedText = fdataList->Strings[i];		//將fdataList切割放入tmpList

		dim = tmpList->Count;			 //取得此筆資料維度 => 64維
		mydata[i] = new double[dim];
		for(int j=0;j<dim;j++){
			mydata[i][j] = tmpList->Strings[j].ToDouble();
//			tdataList->Add(mydata[i][j]);
		}
	}
	//文本轉換結束

	int wordCount = word_count;
	int dataDim = data_dim;
	int dataSize = d_size;

	double* dist = new double[wordCount];		//??? = size of visual words = 1000?
	double* BoF_DES = new double[wordCount];		//bag of feature descriptor
	for(int i=0;i<wordCount;i++)
		BoF_DES[i] = 0;

	for(int i=0;i<dataSize;i++){
//			ifs.read((unsigned char*)data, sizeof(data[0])*dataDim);		//read a "data" (64-dim)

		//計算data與各words的距離
		for(int j=0;j<wordCount;j++){
			dist[j] = GetDist(mydata[i], visual_word[j], dataDim);
		}
		//取得距離最近的分類
		int index_min = argmin(dist, wordCount);
		//記錄分類次數
		BoF_DES[index_min]++;
	}
	Series1->Clear();
	Series1->AddArray(BoF_DES, (wordCount-1));
	//normalization
	int tmp = Normalize(BoF_DES, wordCount);
	Memo2->Lines->Add("normalize = " + WideString(tmp));
//	BoF_count++;

	//finsih per frame
	Memo2->Lines->Add("dataSize = "+WideString(dataSize));
	Memo2->Lines->Add("dataDim = "+WideString(dataDim));





	delete fdataList;
	delete tmpList;
	delete tdataList;

	for(int i=0;i<d_size;i++)
		delete[] mydata[i];
	delete[] mydata;


}
//---------------------------------------------------------------------------



void test1(double **src, double **dst, int n, int dim){

	double **mydata = src;
	double **wb = new double*[n];
	for(int i=0;i<n;i++){
		wb[i] = new double[dim];
		for(int j=0;j<dim;j++){
			wb[i][j] = mydata[i][j] + 2;
		}
	}

	myCopyArray2D(wb, dst, n, dim);

	for(int i=0;i<n;i++){
		delete[] wb[i];
	}
	delete[] wb;

}

void __fastcall TForm1::Button4Click(TObject *Sender)
{

	int n = 1;
	int dim = 10;
	double *data = new double[dim];
	for(int i=0;i<dim;i++){
		data[i] = i;
		Memo2->Lines->Add(data[i]);
	}

	test1(&data, &data, 1, dim);

	Memo2->Lines->Add("====");

	for(int i=0;i<dim;i++){

		Memo2->Lines->Add(data[i]);
	}




}
//---------------------------------------------------------------------------

