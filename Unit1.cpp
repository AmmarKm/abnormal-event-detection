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

	//�O�_��ʿ�JK
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

// 2013.3.26 ��ʥ��y�p��
// ���N�v���������ƭӺ��� 3x3
// ��W�p�� 3x3 ���Ҧ�pixel�����y
// ���ХH�����Ҧ�����p��
// �p����ֻݰO�����I�� => MAX_COUNT�N�u��9
//

double **soc_data;
int soc_data_count;
//TStringList *dataList;

//const int MAX_FRAMES = 2000;
static const String eventt_state[2] = {"normal", "abnormal"};



int trDim = 64;          //��ƪ�����

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

	if(FileListBox1->ItemIndex > -1){		//���qlistbox1������ɦW
		filename = FileListBox1->FileName;
	}
	String dataName = ExtractFileName(filename);
	dataName = ChangeFileExt(dataName, ""); //�NName�h�����ɦW
	newDir = newDir + dataName +"\\";

	CvCapture *cam = cvCaptureFromFile(		//�إ߼v���ӷ�
		filename.t_str()
	);

	Memo1->Lines->Add(filename);
	start2 = clock();

	//�v����T
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
	MyParticle2D** optical_flow = 0; 	    //�p��ɤl���y�ҼȦs���v��pixel�����y
	MyParticle2D** optical_flow_field = 0;  //�ھ�3x3x3 median�ҨD�o���ɤl���y��
	MyParticle2D** social_force_field = 0;  //�ھڲɤl���y���ҨD�o�����|�O��

	//for trainging2
	double **hosf_data = 0;
	int hosf_size = 0;
	int data_dim = 0;
	double **visual_word = 0;
	int word_count = 0;		//Edit11->Text	//�M�w�n���X��words

	double **whiten_trans = 0;	//whiten trans mat

	//????
	int soc_count = 0;	//soc_field�p�⦸��


	/* ��l�ѼƳ]�w */
	int end_of_video = cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_COUNT);
//	int pscale = 3;   	//�ɤl�j�p, NxN
	int pscale = Edit3->Text.ToInt();   	//�ɤl�j�p, NxN
	int half_s = pscale/2;	// 3/2 = 1
	double motion_th = Edit2->Text.ToDouble();
	int frame_gap = Edit9->Text.ToInt();

	//�Ydistance��6, �h��lframes�ϥ�0~5
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

	//output��Ƨ��إ�
	if(!DirectoryExists(initDir+newDir)){	 				//�Youtput path���s�b, �h�إ�dir
		if(!ForceDirectories(initDir+newDir)){				//�إߦh�h�ؿ�
			Memo1->Lines->Add("Cannot create directory!");      //�Y�إ�dir����
			Memo1->Lines->Add(initDir+newDir);
		}
	}

	//2014.11.02 updated
	ofstream wfs, wfs2;
	ifstream rfs;
	//�O��output filename
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

	frame = cvQueryFrame(cam) ;   		//initial, �N��frame count���� -1
	CvSize img_sz = cvGetSize(frame);
	Label15->Caption = cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES);
//	ShowMessage(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

	//���o��l�һݼv�� pscale + dist
	// 3+6 = 9 frames
	//�إ�0~8 frames, images
	int init_frame_number = pscale+distance;	//�Ĥ@�icurrent frame���s��
	for(int i=0;i<init_frame_number;i++){

		//�ĤG�iframe�}�l����
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
		//��Ƕ��Gimages�ݥH��q�D�إ�
		images[i] = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
		cvCvtColor( frames[i], images[i], CV_BGR2GRAY);
	}
//	ShowMessage(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

	//1.�إ�0~2�i�v�������y
	//2.�إ߲�0��optical flow field, �s��p��n����l���y( 2��(x,y)�V�q )
	//3.�إ߲�0��social force field, �s��p��n��SF( 2��(x,y)�V�q )
	//cuboid, NxNxM
//	int m = 2000;		//�w�w�B�z���̤j�e���
	int m = frameCount;	//�w�w�B�z���̤j�e���
	int off_width = img_sz.width/pscale;
	int off_height= img_sz.height/pscale;
	int off_size = off_width*off_height; //�s��ɤl���Ŷ��j�p

	optical_flow = new MyParticle2D*[pscale];   			//�v�����y
	optical_flow_field = new MyParticle2D*[m/pscale];       //�ɤl�����y��
	social_force_field = new MyParticle2D*[m/pscale];       //�ɤl�����|�O��

	// MyParticle2D �w�]��0, ���ݲM��
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
//			//�����٭� �ɤl->pixel
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
	// ==���ŧi�|����delete==
	// �p��exp_r
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
			//�y���٭� �ɤlunit->pixel unit
			p1 = MyParticle2D(p1j*pscale+half_s, p1k*pscale+half_s);
			p2 = MyParticle2D(p2j*pscale+half_s, p2k*pscale+half_s);

//			r[p1j*off_height+p1k][p2j*off_height+p2k] =
//				sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y))+0.5;

			//tmpdist��p1,p2�������Z��
			double tmpx = (p2.x-p1.x);
			double tmpy = (p2.y-p1.y);
			double tmpdist = sqrt( tmpx*tmpx + tmpy*tmpy );

			//��exp
			exp_r[p1j*off_height+p1k][p2j*off_height+p2k] =
				exp( 0-((tmpdist-pscale)/range_b) );
		}
		}
	}
	}
	/* end expr �p�� */

	end = clock();
	Memo1->Lines->Add("��expr�Ҫ�ɶ�: ");
	Memo1->Lines->Add(end-start);

	//output����
//	int pscale = 3;   	//�ɤl�j�p, NxN
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

	/** �]�w���� **/



	/** �}�l�v���B�z **/
	while ( 1 )
	{
//		if(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

		//use webcam
//      CvCapture* cam = cvCaptureFromCAM( CV_CAP_ANY ) ;

		/*��l�]�w*/
		const int win_size = 10 ;
		Label14->Caption = frameCurrent = cur;
		int prev = cur-distance;    //���o�e�@�i�v�����s��, ��l6-6=0

		/************/

		//create a imge for displaying result
		//�M�եH��ܦV�q�� optical flow field, social force field
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


		/* optical flow �p�� */

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

		//case.1 �ǲίS�x�Ѩ�
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
//�屼���m
//�C�@��U���@��LK
//h��h��h��
//�u�I: �i�קKconers�L�h, �B���h���M��Ĳv�ۦP
//
//case.2 get our own features
//3x3 patch
//		int pscale = 3;
//		int half_s = pscale/2;	// 3/2 = 1

clock_t t1, t2;


		//my method �����B�z
		if(CheckBox2->Checked == true){

			int h = frame->height;
			int w = frame->width;
			int imgsize = w*h;

			// L-K
			pyr_prev = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
			pyr_cur = cvCreateImage(img_sz,IPL_DEPTH_32F,1) ;
			CvPoint2D32f* features_cur = new CvPoint2D32f[ MAX_CORNERS ] ;

			//for 3 frames, �C3 frames��1�� (clip)
			start = clock();
			for(int t=0;t<pscale;t++){				//particle scale=3
				//���۹蠟�s�v��, clip size=3
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

				//����i��U�I�p��LK
				for(int i=0 ; i<frame->height ; i++ ){
					corner_count = 0;

					//���o��i�椧�y���I��Jfeatures_prev
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

					//���i�氵LK
					cvCalcOpticalFlowPyrLK(
						img_prev,     			//�e�v��
						img_curr,               //�ثe�v��
						pyr_prev,
						pyr_cur,
						features_prev,          //�e�S�x�I
						features_cur,           //�ҧ�X�������I
						corner_count,
						cvSize(win_size,win_size),
						3,
						feature_found,			//�O�_���
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

						//���æs�Joptical_flow[]
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

				Memo1->Lines->Add(s + "�v��[" + prev + "-" + t + "] ���y�Ҫ�ɶ�: " + String(t2-t1));
//				Memo1->Lines->Add(t2-t1);

			}//end 3 time (pscale)
			end = clock();
			Memo1->Lines->Add("�v�����y�`��ɶ�: "+ String(end-start));
//			Memo1->Lines->Add(end-start);

			delete[] features_cur;
			// End L-K


			start = clock();
			/* median by 3x3x3 into optical flow filed */
			int *median_x = new int[pscale*pscale*pscale];
			int *median_y = new int[pscale*pscale*pscale];
			int median_count = 0;
			int row=0, col=0;
			int pp = prev/pscale;	//�]�ɤl�j�p��pscale, pp���v���ҹ������ɤl���s��
									//�Y�ɤl�b�ɶ��b�W���s��
			optical_flow_field[pp] = new MyParticle2D[off_size];		//ps. off�i�אּ�@���Y�i, �]�p�⧹soc�Y�L�ݨϥ�


			for( int y=0 ; y<h ; y+=pscale ){			//y+=3
			for( int x=0 ; x<w ; x+=pscale ){			//x+=3
				if( y>h || x>w )
					continue;

				median_count = 0;
				//�H(y+1, x+1, t+1)�����߱N3x3x3��pixel�����y��J�}�C, �Hqsort�Ƨ�
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

				//�p��&&�s�� optical flow into field
				//�H�ɤl������, �N��X�������ȥ��y�M��J�ҹ������ɤl���y����m(row, col)
				// x=>col
				// 0=>0, 3=>1, 6=>2;
				row = y/pscale;
				col = x/pscale;
				optical_flow_field[pp][row*off_width+col].x = median_x[median_count/2];
				optical_flow_field[pp][row*off_width+col].y = median_y[median_count/2];

//				String s = "";
//				Memo1->Lines->Add(s+row+", "+col);

				//�ھڪ��e�ȥh��motion���p���ɤl
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
			Memo1->Lines->Add("3�v��median�`��ɶ�: " + String(end-start));
//			Memo1->Lines->Add(end-start);




			delete[] median_x;
			delete[] median_y;


		}//end if checked

		/************** end optical flow ********************/

		///////////// social force field //////////////
		start = clock();

		//�H�ɤl�����ߪ���L�ɤl���۹��m
		int pp = prev/pscale;	//�]�ɤl�j�p��pscale, pp���v���ҹ������ɤl���s��
								//�Y�ɤl�b�ɶ��b�W���s��

//		social_force_field[pp] = new MyParticle2D[4108];
		social_force_field[pp] = new MyParticle2D[off_size];             //��

		double mass = 1;
		int disp_k = ((2*range_k+1)/pscale); // ((2*7+1)/3) = 5
		int disp_l = ((2*range_l+1)/pscale);
		//�۹��m�d�� -2~2
		int start_k = 0-disp_k/2;			 // �q-2�}�l
		int start_l = 0-disp_l/2;
		int end_k = start_k+disp_k;			 // ��2����
		int end_l = start_l+disp_l;

		double f_sum_x = 0;  // sum of force
		double f_sum_y = 0;  // sum of force
		for(int row=0;row<off_height;row++){
		for(int col=0;col<off_width;col++){
			//inactive���ɤl���p��soc
			if(optical_flow_field[pp][row*off_width+col].x == 0
				&& optical_flow_field[pp][row*off_width+col].y == 0){
				continue;
			}



			//��active particle�p���soc
			f_sum_x = 0;
			f_sum_y = 0;
			for(int i=start_l;i<=end_l;i++){	//-2~2
			for(int j=start_k;j<=end_k;j++){
				//�Y�����߲ɤl�h���p��, �]�Z���v�T��0
				if(i==0&&j==0)
					continue;

				//���۹�ɤl���u��y��
				int ii = row+i;
				int jj = col+j;
				//�W�L�v���d�򤧤��s�b���ɤl, ���p��(����0) //�ȩw
				if( ii<0 || ii>=off_height || jj<0 || jj>=off_width ){
					continue;
				}
				//�ҹ������ʶq�Ӥp��particle���C�Jsoc�p�� ( �soff�ɤw�]��(0,0) )
				if(optical_flow_field[pp][ii*off_width+jj].x == 0
					&& optical_flow_field[pp][ii*off_width+jj].y == 0 ){
					continue;
				}

				//�ư��W�z�ɤl, �p��H(row, col)�����ߤ�social force
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
		Memo1->Lines->Add("social force�Ҫ�ɶ�: "+String(end-start));
//		Memo1->Lines->Add(end-start);
//		cvSub(img_prev, img_curr, img_back);

		/****************** end social force *************************/


		/////////////////////// HoSF of Cuobdid ////////////////////
		//��cuboid&data
		//�ˬd�O�_��������soc_count�h�إ�cuboid

		/***********  overlapping case  *******************
		case		spatial	temporal    (0/1 := false/true)
		0              0       0
		1              0       1
		2              1       0
		3              1       1
		��h�W�ثespatial���Ooverlapping, �G�u����temporal�O�_overlapping
		****************************************************/
		bool b_soc = false;
		int b_overlapping_case = 3;		//2 or 3

		switch (b_overlapping_case) {
			//non-overlapping
			case 0:
			case 2:
				b_soc = (soc_count%(2*cb_m)==0) && (soc_count>0) ;		//��soc�ƶq��cuboid���Ʈɤ~��
				break;
			//overlapping
			case 1:
			case 3:
				b_soc = (soc_count >= 2*cb_m);                          //�u�nsoc�����إߥXcuboid�N��
				break;

			default:
				b_soc = false;
		}

////	�³]�w
//		if( b_overlapping == true)
//			b_soc = (soc_count%(2*cb_m)==0) && (soc_count>0) ;				//non-overlapping
//		else
//			b_soc = (soc_count >= 2*cb_m);									//overlapping

		if( b_soc ){           								//boolean_soc
			start = clock();
			int t_center = soc_count-cb_m;	//cb����
			int cb_start_x, cb_start_y, cb_start_z;  				//�p��cb���_�l�I
			double soc_direct;
			int bin;
			double sx, sy;
			int cc=0;
			double *cb_hist = new double[cb_dim];        //2014.11.20�אּ�ʺA  cb_dim = 64

//			double cb_test[512];

			//�b�ɤl���y������active particle
			for(int row=0;row<off_height;row++){
			for(int col=0;col<off_width;col++){
				//inactive particle���L
				if(optical_flow_field[t_center][row*off_width+col].x == 0
					&& optical_flow_field[t_center][row*off_width+col].y == 0){
					continue;
				}
				//�d�򤣨��H��cuboid��particle
				if(row<cb_n || row>off_height-cb_n
					|| col<cb_n || col>off_width-cb_n)
					continue;

				//���active�ëإ�8 sub-coboid
				cb_start_x = col-cb_n;
				cb_start_y = row-cb_n;
				cb_start_z = t_center-cb_m;
				static int locx[8]={0,1,0,1,0,1,0,1};
				static int locy[8]={0,0,1,1,0,0,1,1};
				static int locz[8]={0,0,0,0,1,1,1,1};
				int xx, yy, zz;

				//�}�C�k0
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
						//�p��soc�V�q��V
						soc_direct = cvFastArctan(sy, sx);
						//cvFastArctan���~�t��(1,1)(-1,-1), accurate is about 0.3	by OpenCV 2.0
						if( sx == sy )
							soc_direct += 0.3;
						//�P�_bin
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
				wfs.write((unsigned char*)cb_hist, sizeof(cb_hist[0])*cb_dim);	//�ݧאּsizeof(cb_hist[0])*64

				//�O��data	//stack
				String s = "";
				s = s + cb_hist[0];
				for(int i=1;i<cb_dim;i++){
					s = s + " " + cb_hist[i];
				}
				dataList->Add(s);

				cc++;
			}
			}

			//��ap�ƶW�L10�U, �طs�ɮ�
//			if(dataList->Count > 100000){
//				dataList->SaveToFile(initDir + newDir + dataName + dataFileString + ".txt");			//data��ƿ�X
//				dataList->Clear();
//				dataFileString = "_(" + WideString(++dataFileCount) + ")";
//			}

			//release
			delete[] cb_hist;

			end = clock();

			Memo1->Lines->Add("****");
			Memo1->Lines->Add(s+"clip: " + t_center + " ��cuboid�`��: " + cc);
			Memo1->Lines->Add("��data�Ҫ�ɶ�: " + String(end-start));
//			Memo1->Lines->Add("============");
			Memo1->Lines->SaveToFile(initDir + newDir + dataName + "_readme.txt");
		}

		// end of HoSF of Cuboid per 3 frames


		// change 11.14
		/***** training step 2 + 3 ******/
		int training_type = RadioGroup1->ItemIndex; // 0=old, 1=new
		isWhitening = CheckBox7->Checked;

		int BoF_count;		//�X��BoF��� (part 2 �갵 n clips, �h BoF_count = n)

		if( training_type == 1){
			//�P�_�ɶ��I
			Memo2->Lines->Add("" + WideString(frameCurrent) +"/"+ WideString(frameCount));

			if( frameCurrent > frameCount*0.5 ){		// if >: ��training step 2 + 3
				/* training step 2 */
				if( visual_word == 0 ){     //initial     //11.20�]visual_word�ݥΨ쵲��, �G�אּvisual_word�P�_initial
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
					visual_word = new double*[word_count];			//visual_word�d�쵲����realese by 2014.11.20
					for (int i = 0; i < word_count; i++) {
						visual_word[i] = new double[data_dim];
						for(int j=0;j<data_dim;j++)
							visual_word[i][j] = 0;
					}
					//select seeds for visual words
					int sd_temp;
					int *sd_temp2 = new int[hosf_size];//�D��SEED��,����RANDOM LABEL�O�_�X�{
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
					Memo1->Lines->Add("Visual words�إ�....");
					Memo1->Lines->Add("hosf data size(part1) = "+WideString(hosf_size));
					Memo1->Lines->Add("word count = "+WideString(word_count));

					int *hosf_result = new int[hosf_size];
					int *word_size = new int[word_count];
					int c_count=0;
					int c_max_count=50;

					//return;		//debug

					//�� visual words, (from hosf_data)
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
					//	int cov_threshold;		//wcss�ܰʤp�󦹭ȫh�������ܰ�(�ܰʹL�p)
					//	int cov_count;
					//	int cov_max_count;		//wcss�ܰʹL�p�����ƶW�L���Ȯɵ�������
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

					Memo1->Lines->Add("Visual words�إߧ���!");
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
					wfs.write((const unsigned char*)&dataSize, sizeof(dataSize));		//write dataSze (�w�� datalist->count)
					wfs.close();

					//======== debug ===============
//					String tmpFile2 = initDir + newDir + dataName + "\\" + dataName + dataFileString + "_frame_" + frameCurrent + ".bin";
//					String tmpFile2t = initDir + newDir + dataName + "\\" + dataName + dataFileString + "_frame_" + frameCurrent + ".txt";
//					if(!DirectoryExists(initDir + newDir + dataName)){	 				//�Youtput path���s�b, �h�إ�dir
//						if(!ForceDirectories(initDir + newDir + dataName)){				//�إߦh�h�ؿ�
//							Memo1->Lines->Add("Cannot create directory!");      //�Y�إ�dir����
//							Memo1->Lines->Add(initDir + newDir + dataName);
//						}
//					}

					//===============================================
					//debug
					//�奻data��J�}�C
//					TStringList *fdataList = dataList;
//					int d_size = fdataList->Count;
//					Memo1->Lines->Add("N = "+WideString(d_size));
//
//					double **mydata = new double*[d_size];
//					for(int i=0;i<d_size;i++)
//						mydata[i] = 0;
//
//					TStringList *tmpList = new TStringList;          //�ഫ�μȦsdata
//					tmpList->Delimiter = ' ';   		//�]�w���βŸ�
//
//					TStringList *tdataList = new TStringList;        //�V�m�n����Xdata
//
//					int dim;
//					for(int i=0;i<d_size;i++){
//						tmpList->Clear();
//						tmpList->DelimitedText = fdataList->Strings[i];		//�NfdataList���Ω�JtmpList
//
//						dim = tmpList->Count;			 //���o������ƺ��� => 64��
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

					//�CŪ�@��data�������ðO���X�{����
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
							doWhitenPatch(&data, &data, whiten_trans, 1, dataDim);		//�@����1��data, �Gn:=1; data��1��, �Ginput := &data
						}

						//�p��data�P�Uwords���Z��
						for(int j=0;j<wordCount;j++){
							dist[j] = GetDist(data, visual_word[j], dataDim);
//							dist[j] = GetDist(mydata[i], visual_word[j], dataDim);			//debug
						}
						//���o�Z���̪񪺤���
						int index_min = argmin(dist, wordCount);
						//�O����������
						BoF_DES[index_min]++;
					}
					rfs.close();

					Series1->Clear();
					Series1->AddArray(BoF_DES, (wordCount-1));
					//normalization
					int tmp = Normalize(BoF_DES, wordCount);
 //					Memo2->Lines->Add("normalize = " + WideString(tmp));
					BoF_count++;

					//�O��BoF descriptor
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

					//���ɯd��video����

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

		//���o�s�i�v��
		//get next n frame, n=pscale
		//case.1
		cur = cur + pscale;
		frameCurrent = cur;

		for(int t=0;t<pscale;t++){
			cvReleaseImage( &frames[prev+t] );
			cvReleaseImage( &images[prev+t] );

			//frame����
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
				dataList->SaveToFile(initDir + newDir + dataName + dataFileString + ".txt");			//�Ѿldata��ƿ�X
			}

			Memo1->Lines->Add("end of video!");
			Memo1->Lines->Add( "total cost time: " + WideString(end2-start2) + " (ms)." );
			Memo1->Lines->SaveToFile(initDir + newDir + dataName + "_readme.txt");

			if( training_type == 1){
				wfs.close();
				//�O��BoF Count;
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
	double *Dist, 		//��s�᪺�Udata�P�̪�seed��Distance
	double **Data,
	int d_size,			//numbers of data, data�ƶq
	int dimension,
	int *Seed_label,    //chk if Data is a Seed
	double **Seed,
	int k				//numbers of Seed, �ثe���oseed�ƶq
	){

	int dim = dimension;    // 8x8 = 64 dimention

	for(int i=0;i<d_size;i++){

		/* Dist[i] = 0 if Seed_label[i] = 1; */		//�Y��data��seed�h�Z��=0, �Y���v��0
		if(Seed_label[i] == 1){
			Dist[i]=0;
		}

		/* Dist[i] = min( || Data[i]-Seed[j] || ) */
		else{
			//�����Ĥ@�ӷ���
			double m = GetDist( Data[i], Seed[0], dim );

			//�D�̤p�Z�� := ������v
			for(int j=1;j<k;j++){
			   double tmp = GetDist( Data[i], Seed[j], dim );
			   if(tmp < m)
				   m = tmp;
			}
			Dist[i] = m;

		}
	}

}
/** 2014.11.23 �o�{�S�� **
 * �� data�ƶq���� or data���Ю�, �ɭP�D�諸�sseed�i�୫�Ц�m(dist=0)�Ӥ��Q�D��,
 * �|�ɭP k (seed�ƶq)����, �B i >= d_size, �ϱo *k_cur < K;

 * ���N k_cur ������ڬD�蠟 seed_count
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

	/* ���ѤU��seed */
	int total_dist;
	int new_seed_dist;
	while( k < K ){
		if( k >= d_size) break;			//�Y�ؤl�Ƥw�W�Ldata��, �h����

		total_dist = 0;
		new_seed_dist = 0;

		//�p��seed���v
		for(int i=0;i<d_size;i++){
			//�YData[i]�w��Seed�h���L, RefreshDist() ���w�N���v�]��0
			proba_Dist[i] = Dist[i]*Dist[i];
			total_dist += proba_Dist[i];		//���v�P�Z�����謰����
		}

		//�D�U�@��seed
		new_seed_dist = random(total_dist);		//�ھھ��v�Ҩ���new seed
		int i;
		for(i=0;i<d_size;i++){
			//Data[i]�w��Seed�h���L
			if(Seed_label[i] == 1){
				continue;
			}
			//���sSeed��Data[i]
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
		};	//��Ҧ�Data�w���ؤl, ����.

		/** 2014.11.23 �o�{�S�� **
		�� data�ƶq���� or data���Ю�, �ɭP�D�諸�sseed�i�୫�Ц�m(dist=0)�Ӥ��Q�D��,
		�|�ɭP k (seed�ƶq)����, �B i >= d_size, �ϱo *k_cur < K;

		���N k_cur ������ڬD�蠟 seed_count
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
//	//	int cov_threshold;		//wcss�ܰʤp�󦹭ȫh�������ܰ�(�ܰʹL�p)
//	//	int cov_count;
//	//	int cov_max_count;		//wcss�ܰʹL�p�����ƶW�L���Ȯɵ�������
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
//	int cov_threshold;		//wcss�ܰʤp�󦹭ȫh�������ܰ�(�ܰʹL�p)
//	int cov_count;
//	int cov_max_count;		//wcss�ܰʹL�p�����ƶW�L���Ȯɵ�������
	){

	/* �}�l��k-means */
	double wcss = 0, prev_wcss = 0, dist = 0;
	int argmin_dist = 0;

	//�]��ߦ���
	while(1){

		//initial
		for(int i=0;i<Kc;i++){
			c_size[i] = 0;
		}

		//step1. clustering;
		//�����s, ��C��data��̤p�Z��seed�ðt�m
		for(int i=0; i<d_size; i++){
			dist = 0;
			argmin_dist = 0;

			//�p���i����ƻP�Ҧ��ؤl���̤p�Z��
			for(int j=0;j<Kc;j++){
				//��Z��
				double dist_tmp = 0;
				double tmp;
				for(int k=0;k<dimension;k++){
					tmp = Data[i][k] - Seed[j][k];
					dist_tmp = dist_tmp + tmp*tmp ;
				}
	//			dist_tmp = sqrt(dist_tmp);			//�ت��O��̤p�ؼ�, �G�٤U���B

				//��̤p�Z��
				if(j == 0){
					dist = dist_tmp;
					argmin_dist = 0;
				}
				else{
					//�����
					if( dist_tmp < dist ){
						dist = dist_tmp;
						argmin_dist = j;
					}
				}
			}//end for

			//��줧�ᰵ�O��
			c_result[i] = argmin_dist;
			c_size[argmin_dist] ++;			//cluster_size +1

		}//end clustering

	//	Form1->Memo1->Clear();
	//	for(int i=0;i<seed_count;i++){
	//		Form1->Memo1->Lines->Add(Seed[i][0]);
	//	}

		//step2. ��s�s����, �Y�p��U�s���s���
		for(int i=0;i<Kc;i++){
			for(int j=0;j<dimension;j++){
				Seed[i][j] = 0;
			}
		}

		for(int i=0;i<d_size;i++){
			int j = c_result[i];   //�ӵ���ƪ����s�s��
			int size = c_size[j];  //�Ӥ��s���j�p
			for(int k=0;k<dimension;k++){
				Seed[j][k] = Seed[j][k] + Data[i][k]/size ;	 //���s���Ҧ�data������
			}
		}

		//step3. �p�� criterion, �ݬO�_����
		//within-cluster sum of squares (WCSS):

		c_count++; 	//���s����+1

		if(c_count > c_max_count ){		//�W�L���s�̤j����, k-means����
			Form1->Memo1->Lines->Add(c_count);
			Form1->Memo1->Lines->Add(wcss);
			Form1->Memo1->Lines->Add(prev_wcss - wcss);
			Form1->Memo1->Lines->Add("over");
			break;
		}
		else{
			//�p��wcss
			wcss = 0;
			for(int i=0;i<d_size;i++){
				int j = c_result[i];   //�ӵ���ƪ����s�s��
				double tmp;
				for(int k=0;k<dimension;k++){
					tmp = Data[i][k] - Seed[j][k];
					wcss = wcss + tmp*tmp;	 	 //�ؼи�ƻP�s��ߤ�����M
				}
			}

			Form1->Memo1->Lines->Add(c_count);
			Form1->Memo1->Lines->Add(wcss);
			Form1->Memo1->Lines->Add(prev_wcss - wcss);

			if(c_count == 1){				//�Ĥ@�����߬d
				prev_wcss = wcss;
				continue;
			}
			else if(wcss == prev_wcss){		//����, k-means����, break while
//				Form1->Memo1->Lines->Add(c_count);
//				Form1->Memo1->Lines->Add(wcss);
//				Form1->Memo1->Lines->Add(prev_wcss - wcss);
				Form1->Memo1->Lines->Add("end");
				break;
			}
	//		else if( abs(prev_wcss - wcss) <= cov_threshold ){	//�Y�ܰʤp���Ī��e
	//			prev_wcss = wcss;
	//			cov_count++;				//���Ħ���
	//			if( cov_count > cov_max_count ){
	//				Memo1->Lines->Add("ddd");
	//				break;
	//			}
	//		}
			else{                           //������, �~���
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
////	int cov_threshold;		//wcss�ܰʤp�󦹭ȫh�������ܰ�(�ܰʹL�p)
////	int cov_count;
////	int cov_max_count;		//wcss�ܰʹL�p�����ƶW�L���Ȯɵ�������
//	){
//
//	int dim = dimension;
//	double **mydata = Data;
//
//	/* �}�l��k-means */
//	double wcss = 0, prev_wcss = 0, dist = 0;
//	int argmin_dist = 0;
//
//	//v0313�s�W
//	int initKc = Kc;
//	int min_wcss = 0;
//	int index_min_wcss = 0; // = argmin t
//
//	int Kcc_max = 11;	//3�ؤ���̦n
//	int Kcc_count = 0;	//3�����u�������s������
//	// ( 1 + i*0.5 ), i = -1~1
//
//
//
//	for( int t=(0-Kcc_max/2) ; t<=(0+Kcc_max/2) ; t++){
//		//��lKc����
//		Kc = initKc * (1 + t*0.5);
//		if(Kc < 5){
//			Kc = 5;
//			if ( Kcc_count > 0) {
//				continue;		//K�Ӥp�B�w�g��L���s, �N����
//			}
//		}
//
//		int K = Kc;		//��K��
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
//		Seed = new double*[K];		//��k�Ӻؤl����� pointer to Data
//		myZerosArray(*Seed, K);		//initlal Seed[i] = NULL
//
//		//��Ĥ@��seed
//		int seed_count = 0;      	//�ثeseed��
//		int first = random(d_size);
//		Seed_label[first] = 1;
//		Seed[seed_count] = mydata[first];
//		seed_count++;
//
//		//k-means++ �M�w��l�ؤl, �s��bSeed
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
//		//�Ĥ@���ɷs�t�m�O����tmp ��Seed[ ]�s��ؤl���(�Ȥ@��)
//		for(int i=0;i<seed_count;i++){
//			double *tmp = new double[dim];
//			for(int j=0;j<dim;j++){         //��Seed�ҫ���Data���, ����delete
//				tmp[j] = Seed[i][j];		//�N����V��Seed[ ]��Data�ƻs�L�h
//			}
//			Seed[i] = tmp;					//�s�t�m
//		}
//		/******** 2014.11.23 �ɥR *******
//		* �� seed_count < K ��
//		* �] Seed ��t�m�� double*[K],
//		* �G�G���u�t�� Seed[seed_count], Seed[seed_count]�᭱��NULL
//		* delete��, �� i >= seed_count, Seed[i] ��NULL���t�m�ĤG���Ŷ�Seed[i][j], �G���ݹ�Seed[i]�� delete
//		/********************************/
//
//		// �}�l��k-means
//		int *c_size = new int[seed_count];		//��ڤ��s�j�p cluster_size, ���@�w��K
//		int *d_result = new int[d_size];		//�Udata���������G(Seed�s��)
//
//		int c_count = 0;      	   //���s����
////		int c_max_count = 50;      //�̤j���s����, �W�L�h�j���
//
//		int cov_threshold = 20;		//wcss�ܰʤp�󦹭ȫh�������ܰ�(�ܰʹL�p)
//		int cov_count = 0;
//	//	int cov_max_count = 10;		//wcss�ܰʹL�p�����ƶW�L���Ȯɵ�������
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
//		//	int cov_threshold;		//wcss�ܰʤp�󦹭ȫh�������ܰ�(�ܰʹL�p)
//		//	int cov_count;
//		//	int cov_max_count;		//wcss�ܰʹL�p�����ƶW�L���Ȯɵ�������
//		);
//
//		/* end k-means++ */
//
//
//		//�O����T, ��̤pwcss
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
	if (SelectDirectory("��ܥؿ�", "", Dir))
		FileListBox1->Directory = Dir;
}
//---------------------------------------------------------------------------

//int trDim = 64;          //��ƪ�����           	//���.h

double **trData = 0;     //�V�m��ƪ��ӷ�
int tr_d_size = 0;       //�V�m��ƪ��ƶq
int tr_d_dim = 0;        //�V�m��ƪ�����
int *tr_d_result = 0;    //��ưV�m�᪺����

double **trCodeword = 0; //�ھ�trData�V�m�n��codeword  (dim = tr_d_dim)
int tr_c_count = 0;		 //seed_count := codeword �@���X��
int *tr_c_size = 0;      //��i�s������Ƶ���
double *tr_c_std = 0;    //��i�s�����Z���зǮt, �Z��:=��ƻPcodeword���Z��
double *tr_c_mean = 0;   //��i�s�����Z��������, �Z��:=��ƻPcodeword���Z��
double tr_c_max = 0;	 //���N��codeword���̤j�Z��

int tr_d_count = 0;	     //�Ptr_d_size, ���|update

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

/************* �M���°V�m��� ***************/
void DataClear(){
	if(trData){				//�V�m���
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

	//�M���°V�m��� for new method
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

	//Ū�� initlal
	OpenDialog1->Filter = "txt|*.txt|bag of feature descriptor (*.bof)|*.bof|all file|*.*";
	OpenDialog2->Filter = "txt|*.txt|visual word dictionary (*.vw)|*.vw|all file|*.*|whitening trans mat (*.wmat)|*.wmat";
	if( training_type == 1){
		OpenDialog1->FilterIndex = 2;
		OpenDialog2->FilterIndex = 2;
	}
	else{
		OpenDialog1->FilterIndex = 1;
	}

	//Ū��
	if(OpenDialog1->Execute()){
		loadName = OpenDialog1->FileName;
	}
	else{
		loadName = NULL;
	}
	if(!FileExists(loadName))
		return;

	//Ū��2
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

	//�]�w�s��
//	if(SaveDialog1->Execute()){
//		saveName = SaveDialog1->FileName;
//	}
//	else{
//		saveName = ExtractFilePath(Application->ExeName)+"train_data.txt";
//	}

	start = clock();
	Memo1->Lines->Add("�ɮ׫e�m�B�z��...");
	Memo1->Lines->Add(loadName);

	/************* �M���°V�m��� ***************/
	DataClear();

	//testing button -> unenable
	Button21->Enabled = false;

	/** start */
	if( training_type == 0){
		//�奻data��J�}�C
		fdataList = new TStringList;		//�s��ӷ�data
		fdataList->LoadFromFile(loadName);
		d_size = fdataList->Count;
		Memo1->Lines->Add("N = "+WideString(d_size));

		mydata = new double*[d_size];
		for(int i=0;i<d_size;i++)
			mydata[i] = 0;

		tmpList = new TStringList;          //�ഫ�μȦsdata
		tmpList->Delimiter = ' ';   		//�]�w���βŸ�

		for(int i=0;i<d_size;i++){
			tmpList->Clear();
			tmpList->DelimitedText = fdataList->Strings[i];		//�NfdataList���Ω�JtmpList

			dim = tmpList->Count;			 //���o������ƺ��� => 64��
			mydata[i] = new double[dim];
			for(int j=0;j<dim;j++){
				mydata[i][j] = tmpList->Strings[j].ToDouble();
			}
		}
		//�奻�ഫ����

		end = clock();
		Memo1->Lines->Add("...done! �e�B�z�Ӯ�): " + WideString(start-end));
		start = clock();
		Memo1->Lines->Add("\n�V�m�}�l..");

		/** training1 - whitening */
		isWhitening = CheckBox7->Checked;
		if(isWhitening == true){
			Memo1->Lines->Add("Whitening Transform...");
			if(tr_transMat){
				myDeleteArray2D(&tr_transMat, tr_transMat_dim);
				tr_transMat_dim = 0;
			}
			myCreateArray2D(&tr_transMat, dim, dim);        			//2014.11.20 �ץ��� dim by dim
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
		*	data��إ�BoF�e�w��whitening, �ӥثe���ݹ� BoF ��whitening, �G��button�L��whitening�B�z
		*	����Ū��whitening transform mat ��testing�ϥ�
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

		/** ���ѰѼƻP��ƨѫ���k_means�B�z
		*	�N�s��BoF descritpor�����ª�cuboid descriptor�h�����s
		*/
		d_size = BoF_count;
		dim = BoF_dim;          //BoF_dim := word_count = 1000?
	}

	/** training2 **/				//change 2014.11.22
	int K;
	if( training_type == 0){
		if(CheckBox5->Checked == true)
			K = Form1->Edit4->Text.ToInt();		//��K��
		else
			K = 1.5*sqrt((double)d_size);
	}
	else if( training_type == 1){
		//�ȮɫO���@�ˤ�k��K for BoF //2014.11.22
		if(CheckBox5->Checked == true)
			K = Form1->Edit4->Text.ToInt();		//��K��
		else
			K = 1.5*sqrt((double)d_size);
	}

	/** K-means++ **/
	double** Seed = 0;
	Seed = new double*[K];		//��k�Ӻؤl����� pointer to Data, �u�t�m1��
	for(int i=0;i<K;i++){
		Seed[i] = 0;            //initlal Seed[i] = NULL
	}
	//�إ�Seed_label��k-means++�ϥ�
	int *Seed_label = 0;			//tmp using for k-means++ only
	myCreateArray(&Seed_label, d_size);
	//��Ĥ@��seed
	int seed_count = 0;      	//�ثeseed��
	int first = random(d_size);
	Seed_label[first] = 1;
	Seed[seed_count] = mydata[first];
	seed_count++;

	//k-means++ �M�w��l�ؤl, �s�bSeed
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
	//�Ĥ@���ɷs�t�m�O����tmp ��Seed[ ]�s��ؤl���(�Ȥ@��)		//2014.11.23 �Ntmp�אּ2��
	double **tmp_arr = new double*[seed_count];
	for(int i=0;i<seed_count;i++){
		tmp_arr[i] = new double[dim];
		for(int j=0;j<dim;j++){
			tmp_arr[i][j] = Seed[i][j];		//�N����V��Seed[i][j]��Data�ƻs�L�h
		}
	}
	delete[] Seed;				//�N�W����Seed�ɪ�1���t�mdelete��
								//��Seed[i]�ҫ���Data���, ����delete Seed[i]

	//set 'Seed' point to 'tmp'
	Seed = tmp_arr;					//�s�t�m, �ܦ�Seed�Pmydata�� '�W�߰O����Ŷ�', ���A�@�ΰO����
	tmp_arr = NULL;


	/******** 2014.11.23 �ɥR *******
	* �� seed_count < K ��
	* �] Seed ��t�m�� double*[K],
	* �G�G���u�t�� Seed[seed_count], Seed[seed_count]�᭱��NULL
	* delete��, �� i >= seed_count, Seed[i] ��NULL���t�m�ĤG���Ŷ�Seed[i][j], �G���ݹ�Seed[i]�� delete
	*
	* *�̷s��s: �Ntmp�אּ2���ŧi, ��Seed���h���Ŷ�
	/********************************/

	/** �}�l��k-means **/
	int *c_size = new int[seed_count];		//���s�j�p cluster_size
	int *d_result = new int[d_size];		//�Udata���������G(Seed�s��)

	int c_count = 0;      	   //���s����
	int c_max_count = 50;      //�̤j���s����, �W�L�h�j���

	int cov_threshold = 20;		//wcss�ܰʤp�󦹭ȫh�������ܰ�(�ܰʹL�p)
	int cov_count = 0;
//	int cov_max_count = 10;		//wcss�ܰʹL�p�����ƶW�L���Ȯɵ�������

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
	//	int cov_threshold;		//wcss�ܰʤp�󦹭ȫh�������ܰ�(�ܰʹL�p)
	//	int cov_count;
	//	int cov_max_count;		//wcss�ܰʹL�p�����ƶW�L���Ȯɵ�������
	);

	
	//�s���зǮt�p��
	double *c_std = new double[seed_count];         //seed_count := ��K�s
	double *sigma_dist = new double[seed_count];	//�Ӹs���Z���X
	double *sigma_x2 = new double[seed_count];      //�Ӹs���Z������X
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

			sigma_x2[r] += tmp;           //��r�s���Z������Xsigma(Xk^2), Xk����k����ƻP�s���ߤ��Z��
		}
		sigma_dist[r] += sqrt(dist2);	  //��r�s���Z���X
	}

	//codeword������&�зǮt
	for(int i=0;i<seed_count;i++){

		c_mean[i] = sigma_dist[i]/c_size[i];
//		Memo6->Lines->Add(sigma_x2[i]/c_size[i] - c_mean[i]*c_mean[i]);
//		Memo6->Lines->Add(sigma_x2[i]/c_size[i]);
//		Memo6->Lines->Add(c_mean[i]*c_mean[i]);
		double sigma_tmp = sigma_x2[i]/c_size[i] - c_mean[i]*c_mean[i];
		sigma_tmp = sigma_tmp >= 0 ? sigma_tmp : 0;		//�קKfloat overflow�y���t��
		c_std[i] = sqrt(sigma_tmp);

	}

	//v0309�s�W    //2014.08.21 updated
	//1. c_std = 0 �h��
	//2. c_size < 10 �h��
	//�H c_size = -1 ��ܳQ�h�������s
	int aaa = Edit10->Text.ToInt();
	if(CheckBox4->Checked == true){
		for(int i=0;i<seed_count;i++){
//			//�Ystd��0
//			if( c_std[i] == 0){
//				c_size[i] = 0;
//			}
			//cluster size�Ӥp
			if( c_size[i] < aaa ){
				c_size[i] = -1;
				c_std[i] = -1;
			}
		}
	}



	//�p��typical mean&std, �Ƨǫ������50%;
	double *tmp_mean = new double[seed_count];	//�Ȧs��ƥH�ѱƧ�
	double *tmp_std = new double[seed_count];
	double dmean = 0, dstd = 0;
	int tmp3 = 0;
	for(int i=0;i<seed_count;i++){              //�ƻs��Ȧs�}�C
		tmp_mean[i] = c_mean[i];
		tmp_std[i] = c_std[i];
	}
	qsort(tmp_mean, seed_count, sizeof(double), dcompare);
	qsort(tmp_std, seed_count, sizeof(double), dcompare);
	for(int i=seed_count*0.25;i<seed_count*0.75;i++){		//������50%������
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

	//�p��tr_c_max
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

	//Seed(�V�m�n���s���) ��X
//	output(saveName, Seed, seed_count, dim);

  /* end k-means++ */


	//�s��s�V�m�ܥ����ܼ�, for testing
	trData = mydata;
//	trDim = dim;
	tr_d_size = d_size;
	tr_d_dim = dim;
	tr_d_result = d_result;

	tr_c_count = seed_count;	//��ڤ��s�j�pK

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

	if(FileListBox1->ItemIndex > -1){		//���qlistbox1������ɦW
		filename = FileListBox1->FileName;
	}
	String outputPath, outputName, outputExt, outputFilename;

	String newDir = Edit7->Text + "\\";
//	String newDir = "output2\\";
	//	outputPath = initDir;
	outputPath = "D:\\myOutput\\";
	outputExt = ".bmp";  					    //��X�����ɦW

	outputName = ExtractFileName(filename);     //��XName
	outputName = ChangeFileExt(outputName, ""); //�NName�h�����ɦW

	newDir = newDir + outputName + "\\";

	CvCapture *cam = cvCaptureFromFile(		//�إ߼v���ӷ�
		filename.t_str()
	);

	Memo1->Lines->Add(filename);
	start2 = clock();

	//�v����T
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
	MyParticle2D** optical_flow = 0; 	    //�p��ɤl���y�ҼȦs���v��pixel�����y
	MyParticle2D** optical_flow_field = 0;  //�ھ�3x3x3 median�ҨD�o���ɤl���y��
	MyParticle2D** social_force_field = 0;  //�ھڲɤl���y���ҨD�o�����|�O��

	//2014.11.23 new method
	double **visual_word = trVisualWord;
	int word_count = tr_word_count;		//Edit11->Text	//�M�w�n���X��words

//	double **whiten_trans = tr_transMat;	//whiten trans mat


	int soc_count = 0;	//soc_field�p�⦸��


	/* ��l�ѼƳ]�w */
	int end_of_video = cvGetCaptureProperty(cam, CV_CAP_PROP_FRAME_COUNT);
//	int pscale = 3;   	//�ɤl�j�p, NxN
	int pscale = Edit3->Text.ToInt();   	//�ɤl�j�p, NxN
	int half_s = pscale/2;	// 3/2 = 1
	double motion_th = Edit2->Text.ToDouble();
	int frame_gap = Edit9->Text.ToInt();

	//�Ydistance��6, �h��lframes�ϥ�0~5
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

	frame = cvQueryFrame(cam) ;   		//initial, �N��frame count���� -1
	CvSize img_sz = cvGetSize(frame);
	Label15->Caption = cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES);

	//���o��l�һݼv�� pscale + dist
	// 3+6 = 9 frames
	//�إ�0~8 frames, images
	int init_frame_number = pscale+distance;
	for(int i=0;i<init_frame_number;i++){
		//�ĤG�iframe�}�l����
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
		//��Ƕ��Gimages�ݥH��q�D�إ�
		images[i] = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
		cvCvtColor( frames[i], images[i], CV_BGR2GRAY);
	}
//	ShowMessage(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

	//1.�إ�0~2�i�v�������y
	//2.�إ߲�0��optical flow field, �s��p��n����l���y( 2��(x,y)�V�q )
	//3.�إ߲�0��social force field, �s��p��n��SF( 2��(x,y)�V�q )
	//cuboid, NxNxM
	int m = 2000;	//�w�w�B�z���̤j�e���
	int off_width = img_sz.width/pscale;
	int off_height= img_sz.height/pscale;
	int off_size = off_width*off_height; //�s��ɤl���Ŷ��j�p

	optical_flow = new MyParticle2D*[pscale];   			//�v�����y
	optical_flow_field = new MyParticle2D*[m/pscale];       //�ɤl�����y��
	social_force_field = new MyParticle2D*[m/pscale];       //�ɤl�����|�O��

	//for testing
	double Th_N = Edit5->Text.ToDouble(); 		//Threshold_Normal
	int *ap_mapping = new int[off_size];		//��testing��8�F�~��mapping����
	TStringList *zList = new TStringList;	  	//�O��ap��z-value
	TStringList *zheadList = new TStringList; 	//�O��ap��z-value-head
	TStringList *z_average_List = new TStringList;

	//for output;
	Normality normality = Undefined;
	int output_count = cb_m*pscale;		//��e����clip���_�lframe
	int output_clip = 0;  	 			//��e����clip (build by 24 frames)
	CvPoint label_p1 = cvPoint(5,5);	//label size
	CvPoint label_p2 = cvPoint(40,12);
	newDir = newDir + WideString(tr_c_count) + "\\" + WideString(Th_N) + "\\";
	outputFilename = outputPath + newDir + outputName;






	// MyParticle2D �w�]��0, ���ݲM��
	for(int i=0;i<pscale;i++){
		optical_flow[i] = new MyParticle2D[img_sz.width*img_sz.height];
	}
	for(int i=0;i<m/pscale;i++){
		optical_flow_field[i] = 0;
		social_force_field[i] = 0;
	}
	optical_flow_field[0] = new MyParticle2D[off_size];		//ps. off�i�אּ�@���Y�i, �]�p�⧹soc�Y�L�ݨϥ�
//	social_force_field[0] = new MyParticle2D[off_size];

	MyParticle2D p1, p2;

	// �إ�exp_r
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
			//�y���٭� �ɤlunit->pixel unit
			p1 = MyParticle2D(p1j*pscale+half_s, p1k*pscale+half_s);
			p2 = MyParticle2D(p2j*pscale+half_s, p2k*pscale+half_s);

//			r[p1j*off_height+p1k][p2j*off_height+p2k] =
//				sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y))+0.5;

			//tmpdist��p1,p2�������Z��
			double tmpx = (p2.x-p1.x);
			double tmpy = (p2.y-p1.y);
			double tmpdist = sqrt( tmpx*tmpx + tmpy*tmpy );

			//��exp
			exp_r[p1j*off_height+p1k][p2j*off_height+p2k] =
				exp( 0-((tmpdist-pscale)/range_b) );
		}
		}
	}
	}
	/* end expr �p�� */

	end = clock();
	Memo1->Lines->Add("��expr�Ҫ�ɶ�: ");
	Memo1->Lines->Add(end-start);

	/** �]�w���� **/



	/** �}�l�B�z **/
	while ( 1 )
	{
//		if(cvGetCaptureProperty(cam, CV_CAP_PROP_POS_FRAMES));

		//use webcam
//      CvCapture* cam = cvCaptureFromCAM( CV_CAP_ANY ) ;

		/*��l�]�w*/
		const int win_size = 10 ;
		Label14->Caption = cur;
		int prev = cur-distance;    //���o�e�@�i�v�����s��, ��l6-6=0

		/************/

		//create a imge for displaying result
		//�M�եH��ܦV�q��
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

		/* optical flow �p�� */

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

		//case.1 �ǲίS�x�Ѩ�
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
//�屼���m
//�C�@��U���@��LK
//h��h��h��
//�u�I: �i�קKconers�L�h, �B���h���M��Ĳv�ۦP
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
				//���۹蠟�s�v��, clip size=3
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

				//����i��U�I�p��LK
				int ignore_height = Edit6->Text.ToInt();
				for(int i=0 ; i<frame->height ; i++ ){

					//�����Y���ץH�����y -> ����normal&abnormal label
					if(i<=ignore_height){
//						optical_flow[t][row*w+col].x = 0;	//MyParticle�w�]��0�i����
//						optical_flow[t][row*w+col].y = 0;
						continue;
					}

					corner_count = 0;

					//���o��i�椧�y���I��Jfeatures_prev
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

					//���i�氵LK
					cvCalcOpticalFlowPyrLK(
						img_prev,     			//�e�v��
						img_curr,               //�ثe�v��
						pyr_prev,
						pyr_cur,
						features_prev,          //�e�S�x�I
						features_cur,           //�ҧ�X�������I
						corner_count,
						cvSize(win_size,win_size),
						3,
						feature_found,			//�O�_���
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

						//���æs�Joptical_flow[]
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

				Memo1->Lines->Add(s + "�v��[" + prev + "-" + t + "] ���y�Ҫ�ɶ�: " + String(t2-t1));
//				Memo1->Lines->Add(t2-t1);

			}//end 3 time (pscale)
			end = clock();
			Memo1->Lines->Add("�v�����y�`��ɶ�: "+ String(end-start));
//			Memo1->Lines->Add(end-start);

			delete[] features_cur;
			// End L-K


			start = clock();
			/* median by 3x3x3 into optical flow filed */
			int *median_x = new int[pscale*pscale*pscale];
			int *median_y = new int[pscale*pscale*pscale];
			int median_count = 0;
			int row=0, col=0;
			int pp = prev/pscale;	//�]�ɤl�j�p��pscale, pp���v���ҹ������ɤl���s��
									//�Y�ɤl�b�ɶ��b�W���s��
			optical_flow_field[pp] = new MyParticle2D[off_size];


			for( int y=0 ; y<h ; y+=pscale ){			//y+=3
			for( int x=0 ; x<w ; x+=pscale ){			//x+=3
				if( y>h || x>w )
					continue;

				median_count = 0;
				//�H(y+1, x+1, t+1)�����߱N3x3x3��pixel�����y��J�}�C, �Hqsort�Ƨ�
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

				//�p��&&�s�� optical flow into field
				//�H�ɤl������, �N��X�������ȥ��y�M��J�ҹ������ɤl���y����m(row, col)
				// x=>col
				// 0=>0, 3=>1, 6=>2;
				row = y/pscale;
				col = x/pscale;
				optical_flow_field[pp][row*off_width+col].x = median_x[median_count/2];
				optical_flow_field[pp][row*off_width+col].y = median_y[median_count/2];

//				String s = "";
//				Memo1->Lines->Add(s+row+", "+col);

				//�ھڪ��e�ȥh��motion���p���ɤl
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
			Memo1->Lines->Add("3�v��median�`��ɶ�: " + String(end-start));
//			Memo1->Lines->Add(end-start);


			delete[] median_x;
			delete[] median_y;

		}//end if checked



		///////////// social force field //////////////
		start = clock();

		//�H�ɤl�����ߪ���L�ɤl���۹��m
		int pp = prev/pscale;	//�]�ɤl�j�p��pscale, pp���v���ҹ������ɤl���s��
								//�Y�ɤl�b�ɶ��b�W���s��

		social_force_field[pp] = new MyParticle2D[off_size];

		double mass = 1;
		int disp_k = ((2*range_k+1)/pscale); // ((2*7+1)/3) = 5
		int disp_l = ((2*range_l+1)/pscale);
		//�۹��m�d�� -2~2
		int start_k = 0-disp_k/2;			 // �q-2�}�l
		int start_l = 0-disp_l/2;
		int end_k = start_k+disp_k;			 // ��2����
		int end_l = start_l+disp_l;

		double f_sum_x = 0;  // sum of force
		double f_sum_y = 0;  // sum of force
		for(int row=0;row<off_height;row++){
		for(int col=0;col<off_width;col++){
			//inactive���ɤl���p��soc
			if(optical_flow_field[pp][row*off_width+col].x == 0
				&& optical_flow_field[pp][row*off_width+col].y == 0){
				continue;
			}

			//��active particle�p���soc
			f_sum_x = 0;
			f_sum_y = 0;
			for(int i=start_l;i<=end_l;i++){	//-2~2
			for(int j=start_k;j<=end_k;j++){
				//�Y�����߲ɤl�h���p��, �]�Z���v�T��0
				if(i==0&&j==0)
					continue;

				//���۹�ɤl���u��y��
				int ii = row+i;
				int jj = col+j;
				//�W�L�v���d�򤧤��s�b���ɤl, ���p��(����0) //�ȩw
				if( ii<0 || ii>=off_height || jj<0 || jj>=off_width ){
					continue;
				}
				//�ҹ������ʶq�Ӥp��particle���C�Jsoc�p�� ( �soff�ɤw�]��(0,0) )
				if(optical_flow_field[pp][ii*off_width+jj].x == 0
					&& optical_flow_field[pp][ii*off_width+jj].y == 0 ){
					continue;
				}

				//�ư��W�z�ɤl, �p��H(row, col)�����ߤ�social force
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
		Memo1->Lines->Add("social force�Ҫ�ɶ�: "+String(end-start));
//		Memo1->Lines->Add(end-start);

//		cvSub(img_prev, img_curr, img_back);

		//��cuboid&data
		bool b_soc = false;
		bool b_overlapping = true;	//true or false����O�_overlap

		//�ˬd�O�_��������soc_count�h�إ�cuboid
		if( b_overlapping )
			b_soc = (soc_count >= 2*cb_m);									//overlapping
		else
			b_soc = (soc_count%(2*cb_m)==0) && (soc_count>0) ;				//non-overlapping by temporal

		if( b_soc ){
			start = clock();
			int t_center = soc_count-cb_m;	//cb����
			int cb_start_x, cb_start_y, cb_start_z;  				//�p��cb���_�l�I
			double soc_direct;
			int bin;
			double sx, sy;
			int cb_count=0;
			double *cb_hist = new double[cb_dim];
			TStringList *ap_List = new TStringList;			//�O��ap��64�����
			TStringList *ap_loc_List = new TStringList;		//�O��ap���y��
			double z_average = 0;

			//�M�� index for AP mapping, �ΨӰO�������ǲɤl�OAP, -1��ܫDAP
			for(int i=0;i<off_size;i++){
				ap_mapping[i] = -1;
			}

			//��active particle
			for(int row=0;row<off_height;row++){
			for(int col=0;col<off_width;col++){
				//inactive particle���L
				if(optical_flow_field[t_center][row*off_width+col].x == 0
					&& optical_flow_field[t_center][row*off_width+col].y == 0){
					continue;
				}
				//�d�򤣨��H��cuboid��particle
				if(row<cb_n || row>off_height-cb_n
					|| col<cb_n || col>off_width-cb_n)
					continue;

				//���active�ëإ�8 sub-coboid
				cb_start_x = col-cb_n;
				cb_start_y = row-cb_n;
				cb_start_z = t_center-cb_m;
				static int locx[8]={0,1,0,1,0,1,0,1};
				static int locy[8]={0,0,1,1,0,0,1,1};
				static int locz[8]={0,0,0,0,1,1,1,1};
				int xx, yy, zz;

				//�}�C�k0
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
						//�p��soc�V�q��V
						soc_direct = cvFastArctan(sy, sx);
						//cvFastArctan���~�t��(1,1)(-1,-1), accurate is about 0.3
						if( sx == sy )
							soc_direct += 0.3;
						//�P�_bin
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

				//�O�� ap��data
				String s = "";
				s = s + cb_hist[0];
				for(int i=1;i<cb_dim;i++){
					s = s + " " + cb_hist[i];
				}
				ap_List->Add(s);
				//�O�� ap���y��(col,row)
				ap_loc_List->Add(WideString(col)+WideString(",")+WideString(row));

				//a index of AP mapping, �s���Ȫ�ܫ�谵�F�~�B�z��, �Ҧs���}�Cindex
				ap_mapping[row*off_width+col] = cb_count;

				cb_count++;

			}
			}
			delete[] cb_hist;

			//�Yap�Ӥ�
			if(cb_count < 3){
				//NORMAL
				normality = Undefined;
				zList->Add(s+"clip: " + t_center + " ��cuboid�`��: " + cb_count);
				zheadList->Add(s+"clip: " + t_center + " ��cuboid�`��: " + cb_count);
				z_average = -1;
				Memo5->Lines->Add(z_average);
			}
			else{
				//���oframe���Ҧ�ap��, �A�qList���s��^data�}�C (�W�z�B�J�O�p��HoSF&�έpAP�ƶq)
				TStringList *tmpList = new TStringList;  //�ഫ�μȦsdata
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
				//�Nap_List�ഫ�æs�Jap_data
				tmpList->Delimiter = ' ';   		//�]�w���βŸ�
				for(int i=0;i<cb_count;i++){
					tmpList->Clear();
					tmpList->DelimitedText = ap_List->Strings[i];		//�Nap_List���Ω�JtmpList

					ap_dim = tmpList->Count;			 //���o������ƺ��� => 64��
					ap_data[i] = new double[ap_dim];
					for(int j=0;j<ap_dim;j++){
						ap_data[i][j] = tmpList->Strings[j].ToDouble();
					}
				}
				//�Nap_loc_List�ഫ�æs�Jap_loc
				tmpList->Delimiter = ',';   		//�]�w���βŸ�
				for(int i=0;i<cb_count;i++){
					tmpList->Clear();
					tmpList->DelimitedText = ap_loc_List->Strings[i];		//�Nap_loc_List���Ω�JtmpList

					ap_loc[i].x = tmpList->Strings[0].ToDouble();    //col
					ap_loc[i].y = tmpList->Strings[1].ToDouble();    //row
				}
//				Memo4->Lines->Add(ap_List->Text);
//				Memo4->Lines->SaveToFile(ExtractFilePath(Application->ExeName)+"apdata.txt");


				/************ whitening transform ************************
				 * �N ap_data �� whitening
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
					//double **trCodeword = 0; //�V�m�n��codeword
					//int tr_c_count = 0;	   //seed_count �@�����X�s
					//int *tr_c_size = 0;      //��i�s������Ƶ���
					//double *tr_c_std = 0;    //��i�s�����Z���зǮt, �Z��:=��ƻPcodeword���Z��
					//double *tr_c_mean = 0;   //��i�s�����Z��������, �Z��:=��ƻPcodeword���Z��
					*****************************************************************************/

					double z, min;
					int argcode;  		//argCodeword
					int neighbor_index, neighbor_count, mapx, mapy;
					int	dirx[8] = {-1, 0, 1,-1, 1,-1, 0, 1};	//8�F�~, ���F(0,0)�H�~
					int diry[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
					double median_z[9];
					//��AP�̬ۦ���codeword, ��AP������
					zList->Add(s+"clip: " + t_center + " ��cuboid�`��: " + cb_count);
					for(int i=0;i<cb_count;i++){
						argcode = 0;
						min = GetDist(ap_data[i], trCodeword[0], ap_dim);	//ap_dim = 64
						double tmp;
						for(int j=1;j<tr_c_count;j++){
							if(tr_c_size[j] == -1)	//v0308, �Y���Q�h�������s�h���L
								continue;

							tmp = GetDist(ap_data[i], trCodeword[j], ap_dim);
							//���̤p
							if(tmp < min){
								argcode = j;
								min = tmp;
							}
						}
						ap_result[i] = argcode; //�O���Ҥ������s�s��
	//					Memo3->Lines->Add(argcode);

						//calc. z-value
						if(tr_c_std[argcode] == 0){ 	 //�Ysigma == 0, �O�䬰0.001   //0821 updated
														 //v0308��, ���������ä�����, �]�e��for�ɤw�z��L, ���D���s��std���G��0
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
					//ps. �W�U��qfor���i�X�@�_, �U�q�|�X��, �]�F�~����|�W�X����

					zheadList->Add(s+"clip: " + t_center + " ��cuboid�`��: " + cb_count);

					//��Ҧ�AP�p��z-value-head
					for(int i=0;i<cb_count;i++){

						z = z_value[i];
						//calc. z_head, ����(13)	v0604.�s�׬�median
						if( z >= Th_N){
							double tmp_sigma = 0;
							double pp_dist, cc_dist;

							double gamma = 0.5;
							double cc_max = tr_c_max;		//����(13)���� max

							neighbor_count = 0;				//AP���F�~��

							//�p��F�~�v�T	//�w�אּ8�F�~AP��median, by 2013.6.4
							for(int j=0;j<8;j++){
								mapx = ap_loc[i].x + dirx[j];   //col
								mapy = ap_loc[i].y + diry[j];	//row
								//check if neightbor out of range
								if( mapx<0 || mapx>=off_width || mapy<0 || mapy>=off_height )
									continue;

								//��8�F�~��ap�}�C����index
								neighbor_index = ap_mapping[mapy*off_width+mapx];		//���omapping index

								//�Y�F�~�Oinactive point (�DAP)
								if( neighbor_index == -1 )
									continue;

	//�H�U�O�¤�k
	//							//�Y���I��codeword�ۦP, ���C�J�p�� ( boolean Ci != Cj )
	//							if( argcode == ap_result[neighbor_index])
	//								continue;
	//							//�p����I����codeword���Z��
	//							cc_dist = GetDist(trCodeword[argcode], trCodeword[ap_result[neighbor_index]], trDim);
	//							//pp_dist�H�ɤl�Z�������
	//							pp_dist = (ap_loc[i].x-ap_loc[neighbor_index].x)*(ap_loc[i].x-ap_loc[neighbor_index].x)
	//									+ (ap_loc[i].y-ap_loc[neighbor_index].y)*(ap_loc[i].y-ap_loc[neighbor_index].y);
	//							pp_dist = sqrt(pp_dist);
	//
	//							tmp_sigma = tmp_sigma + exp( cc_dist/(cc_max*gamma*pp_dist) );
	//end

								//v0604. 8-neightbor median
								//�N�F�~�Ȧs�J�}�C, ���ݱƧ�
								median_z[neighbor_count] = z_value[neighbor_index];
								neighbor_count++;
							}

							//�Y�L�F�~, �hz_head = ThN;
							if(neighbor_count == 0){
								z_value_head[i] = Th_N;
							}
							else{
	//old							z_value_head[i] = z * tmp_sigma / neighbor_count;


	//new						//�[�J�ۤv��, ��median
								median_z[neighbor_count] = z;
								neighbor_count++;

								//median_z[0]�Odouble, �Gcompare�ݥ�double compare
								qsort(median_z, neighbor_count, sizeof(double), dcompare);

								//�Y�`�Ƭ�even, ���������Ƥ�����
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

					//(1). ap_data �ھ� visual word �ͦ� BoF per frame
					//�CŪ�@��data�������ðO���X�{����
					double* dist = new double[wordCount];		//??? = size of visual words = 1000?
					double* BoF_DES = new double[wordCount];		//bag of feature descriptor
//					myZerosArray(BoF_DES, wordCount);
					for(int i=0;i<wordCount;i++)
						BoF_DES[i] = 0;

					for(int i=0;i<dataSize;i++){
						double *data = ap_data[i];

						//whitening transform
						//�e���w��ap_data���Lwhitening�G���ݦA��

						//�p��data�P�Uwords���Z��
						for(int j=0;j<wordCount;j++){
							dist[j] = GetDist(data, visual_word[j], dataDim);
						}
						//���o�Z���̪񪺤���
						int index_min = argmin(dist, wordCount);
						//�O����������
						BoF_DES[index_min]++;
					}

					Series1->Clear();
					Series1->AddArray(BoF_DES, (wordCount-1));
					//normalization
					int tmpN = Normalize(BoF_DES, wordCount);
 //					Memo2->Lines->Add("normalize = " + WideString(tmpN));

					zList->Add(s+"clip: " + t_center + " ��cuboid�`��: " + cb_count);

					//(2). bof -> zvalue
					//��BoF�̬ۦ���codeword, ��BoF������
					int index_min = 0;
					int c_dim = wordCount;	//cluster codeword dimension

					double dist_min = GetDist(BoF_DES, trCodeword[0], c_dim);
					double tmp;
					for(int j=1;j<tr_c_count;j++){
						if(tr_c_size[j] == -1)	//v0308, �Y���Q�h�������s�h���L
							continue;

						tmp = GetDist(BoF_DES, trCodeword[j], c_dim);
						//���̤p
						if(tmp < dist_min){
							index_min = j;       //�O���Ҥ������s�s��
							dist_min = tmp;
						}
					}

					// calc. z-value
					if(tr_c_std[index_min] == 0){ 	 //�Ysigma == 0, �O�䬰0.001   //0821 updated
													 //v0308��, ���������ä�����, �]�e��for�ɤw�z��L, ���D���s��std���G��0
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
					if(!DirectoryExists(outputPath+newDir)){	 				//�Youtput path���s�b, �h�إ�dir
						if(!ForceDirectories(outputPath+newDir)){				//�إߦh�h�ؿ�
							Memo1->Lines->Add("Cannot create directory!");      //�Y�إ�dir����
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

				output_count++; 	 //�L��output���\�P�_���n����
			}
			output_clip++;


			Memo1->Lines->Add("****");
			Memo1->Lines->Add(s+"clip: " + t_center + " ��cuboid�`��: " + cb_count);
//			Memo1->Lines->Add(cc);
			Memo1->Lines->Add("��data&testing�Ҫ�ɶ�: " + String(end-start));
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

		//���o�s�i�v��
		//get next n frame, n=pscale
		//case.1
		cur = cur + pscale;

		for(int t=0;t<pscale;t++){
//			cvReleaseImage( &frames[prev+t] );		//���output result�ɦA����, ���|�ѫe�X�i������d��̫�
			cvReleaseImage( &images[prev+t] );
			images[prev+t] = 0;

			//get new frames
			//frame����, �ܤְ���@��
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
	for(int i=0;i<frameCount;i++){			//�קK�|������, �]frames�e�ּƴX�i������
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
	//�p��tr_c_max
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
int trDim = 64;          //��ƪ�����
double **trData = 0;     //�V�m����ƨӷ�
int tr_d_size;           //�V�m����Ƽƶq
int *tr_d_result = 0;    //��ưV�m�᪺����

double **trCodeword = 0; //�V�m�n��codeword
int tr_c_count = 0;		 //seed_count �@�����X�s
int *tr_c_size = 0;      //��i�s������Ƶ���
double *tr_c_std = 0;    //��i�s�����Z���зǮt, �Z��:=��ƻPcodeword���Z��
double *tr_c_mean = 0;   //��i�s�����Z��������, �Z��:=��ƻPcodeword���Z��
double tr_c_max = 0;	 //���N��codeword���̤j�Z��
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
//	//Ū��
//	if(OpenDialog1->Execute()){
//		loadName = OpenDialog1->FileName;
//	}
//	else{
//		loadName = NULL;
//	}
//	if(!FileExists(loadName))
//		return;
//
//	//�]�w�s��
//	if(SaveDialog1->Execute()){
//		saveName = SaveDialog1->FileName;
//	}
//	else{
//		saveName = ExtractFilePath(Application->ExeName)+"refine_data.txt";
//	}
//
//	start = clock();
//	Memo1->Lines->Add("�ɮ׫e�m�B�z��...");
//
//	//�M���°V�m���
////	if(trData){				//�V�m���
////		for(int i=0;i<tr_d_size;i++){
////			if(trData[i])
////				delete[] trData[i];
////		}
////		delete[] trData;
////	}
////	if(tr_d_result)         //�������G
////		delete[] tr_d_result;
////
////	if(trCodeword){			//codewords
////		for(int i=0;i<tr_c_count;i++){
////			if(trCodeword[i])
////				delete[] trCodeword[i];
////		}
////		delete[] trCodeword;
////	}
////	if(tr_c_size)        	//���s�j�p
////		delete[] tr_c_size;
////	if(tr_c_std)        		//��i�s���Z���зǮt
////		delete[] tr_c_std;
////	if(tr_c_mean)        		//
////		delete[] tr_c_mean;
//
//	/** start */
//
//	//�奻data��J�}�C
//	fdataList = new TStringList;		//�s��ӷ�data
//	fdataList->LoadFromFile(loadName);
//	d_size = fdataList->Count;
//
//	mydata = new double*[d_size];
//	for(int i=0;i<d_size;i++)
//		mydata[i] = 0;
//
//	tmpList = new TStringList;          //�ഫ�μȦsdata
//	tmpList->Delimiter = ' ';   		//�]�w���βŸ�
//
//	tdataList = new TStringList;        //�V�m�n����Xdata
//
//	for(int i=0;i<d_size;i++){
//		tmpList->Clear();
//		tmpList->DelimitedText = fdataList->Strings[i];		//�NfdataList���Ω�JtmpList
//
//		dim = tmpList->Count;			 //���o������ƺ��� => 64��
//		mydata[i] = new double[dim];
//		for(int j=0;j<dim;j++){
//			mydata[i][j] = tmpList->Strings[j].ToDouble();
//		}
//	}
//	//�奻�ഫ����
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
//		//��AP�̬ۦ���codeword, �p��z-value
//		argcode = 0;
//		min = GetDist(ap_data[i], trCodeword[0], trDim);	//trDim = 64
//		double tmp;
//		for(int j=1;j<tr_c_count;j++){
//			tmp = GetDist(ap_data[i], trCodeword[j], trDim);
//			//���̤p
//			if(tmp < min){
//				argcode = j;
//				min = tmp;
//			}
//		}
//		if(tr_c_std[argcode] == 0){ 	 //�Ysigma == 0
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
//		//case a, b, c�@��
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
//				//������s
//				size = size + 1;
//			}
//			else{
//				//if ��ƶq�L��, goto c
//				//argcode = argcode, then goto c.
//				case_c = true;
//			}
//		}
//		//case b.
//		else{
//			ttt++;
//			/***************************************************************
//			double **trCodeword = 0; //�V�m�n��codeword
//			int tr_c_count = 0;		 //seed_count �@�����X�s
//			int *tr_c_size = 0;      //��i�s������Ƶ���
//			double *tr_c_std = 0;    //��i�s�����Z���зǮt, �Z��:=��ƻPcodeword���Z��
//			double *tr_c_mean = 0;   //��i�s�����Z��������, �Z��:=��ƻPcodeword���Z��
//			double tr_c_max = 0;	 //���N��codeword���̤j�Z��
//			****************************************************************/
//			double **tmpCodeword = 0;
//			int *tmp_size = 0;
//			double *tmp_std = 0, *tmp_mean = 0;
//
//			//���ͷs��cluster k, goto c
//			//���qcode�O���F�s�Warray[K+1]���Narray[K]�H�F��s�Wdata���ت�
//			int new_count = tr_c_count + 1;			//new_K = K+1
//			tmpCodeword = trCodeword;               //��V�m��ư��Ȧs
//			tmp_size = tr_c_size;
//			tmp_std = tr_c_std;
//			tmp_mean = tr_c_mean;
//
//			trCodeword = new double*[new_count];	//�ŧi�s���}�C���N�ª�
//			tr_c_size = new int[new_count];
//			tr_c_std = new double[new_count];
//			tr_c_mean = new double[new_count];
//
//			//�ƻs�¸��
//			for(int j=0;j<new_count-1;j++){			//�N�­Ƚƻs��s���}�C
//				trCodeword[j] = tmpCodeword[j];		//trCodeword�����нƻs
//				tr_c_size[j] = tmp_size[j];         //�H�U�T�Ӭ�value�ƻs
//				tr_c_std[j] = tmp_std[j];
//				tr_c_mean[j] = tmp_mean[j];
//			}
//			delete[] tmpCodeword;                   //delete �°}�C
//			delete[] tmp_size;
//			delete[] tmp_std;
//			delete[] tmp_mean;
//
//			//newly added data at last index
//			argcode = new_count-1;	//j=k           //���o�̫�@��codeword��index
//
//			trCodeword[argcode] = new double[dim];
//			for(int j=0;j<dim;j++){
//				trCodeword[argcode][j] = ap_data[i][j]; 		//update new codeword
//																//�]���O�scluster, �G�u���@��data
//																//�����ƻs��seed
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
//		//update dictionary; codeword�bcase���w��s
//		tr_c_mean[argcode] = mean;
//		tr_c_std[argcode] = std;
//		tr_c_size[argcode] = size;
////		tr_c_max = GetMaxDist(trCodeword, tr_c_count, trDim);		//�ثe�]codeword�ƶq�L�j�ӹL�C
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

	//Ū��
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

	double **visual_word = new double*[word_count];			//visual_word�d�쵲����realese by 2014.11.20
	for (int i = 0; i < word_count; i++) {
		visual_word[i] = new double[data_dim];
		for(int j=0;j<data_dim;j++)
			visual_word[i][j] = 0;
	}

	TStringList *dataList = new TStringList;		//�s��ӷ�data
	for(int i=0;i<word_count;i++){
		sf_rs.read((const unsigned char*)visual_word[i], sizeof(visual_word[i][0])*data_dim);		//write "words"
		//�O��data	//stack
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
	//Ū��
	if(OpenDialog1->Execute()){
		loadName = OpenDialog1->FileName;
	}
	else{
		loadName = NULL;
		return;
	}

	//�奻data��J�}�C
	TStringList *fdataList = new TStringList;		//�s��ӷ�data
	fdataList->LoadFromFile(loadName);
	int d_size = fdataList->Count;
	Memo1->Lines->Add("N = "+WideString(d_size));

	double **mydata = new double*[d_size];
	for(int i=0;i<d_size;i++)
		mydata[i] = 0;

	TStringList *tmpList = new TStringList;          //�ഫ�μȦsdata
	tmpList->Delimiter = ' ';   		//�]�w���βŸ�

	TStringList *tdataList = new TStringList;        //�V�m�n����Xdata

	int dim;
	for(int i=0;i<d_size;i++){
		tmpList->Clear();
		tmpList->DelimitedText = fdataList->Strings[i];		//�NfdataList���Ω�JtmpList

		dim = tmpList->Count;			 //���o������ƺ��� => 64��
		mydata[i] = new double[dim];
		for(int j=0;j<dim;j++){
			mydata[i][j] = tmpList->Strings[j].ToDouble();
//			tdataList->Add(mydata[i][j]);
		}
	}
	//�奻�ഫ����

	int wordCount = word_count;
	int dataDim = data_dim;
	int dataSize = d_size;

	double* dist = new double[wordCount];		//??? = size of visual words = 1000?
	double* BoF_DES = new double[wordCount];		//bag of feature descriptor
	for(int i=0;i<wordCount;i++)
		BoF_DES[i] = 0;

	for(int i=0;i<dataSize;i++){
//			ifs.read((unsigned char*)data, sizeof(data[0])*dataDim);		//read a "data" (64-dim)

		//�p��data�P�Uwords���Z��
		for(int j=0;j<wordCount;j++){
			dist[j] = GetDist(mydata[i], visual_word[j], dataDim);
		}
		//���o�Z���̪񪺤���
		int index_min = argmin(dist, wordCount);
		//�O����������
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

