//---------------------------------------------------------------------------

#ifndef Unit1H
#define Unit1H
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <FileCtrl.hpp>
#include <Dialogs.hpp>
#include <ExtDlgs.hpp>
#include <ExtCtrls.hpp>
#include <Menus.hpp>
#include "Chart.hpp"
#include "Series.hpp"
#include "TeEngine.hpp"
#include "TeeProcs.hpp"
#include <Grids.hpp>
#include <ValEdit.hpp>
#include <ComCtrls.hpp>
#include <ToolWin.hpp>
#include <Buttons.hpp>
//---------------------------------------------------------------------------
class TForm1 : public TForm
{
__published:	// IDE-managed Components
	TOpenPictureDialog *OpenPictureDialog1;
	TImage *Image1;
	TCheckBox *CheckBox1;
	TCheckBox *CheckBox2;
	TButton *Button8;
	TEdit *Edit1;
	TCheckBox *CheckBox3;
	TLabel *Label4;
	TLabel *Label5;
	TLabel *Label6;
	TLabel *Label7;
	TLabel *Label8;
	TPanel *Panel1;
	TLabel *Label9;
	TLabel *Label10;
	TLabel *Label11;
	TLabel *Label12;
	TLabel *Label13;
	TLabel *Label14;
	TMemo *Memo1;
	TEdit *Edit2;
	TEdit *Edit3;
	TEdit *Edit4;
	TMemo *Memo2;
	TMainMenu *MainMenu1;
	TMenuItem *Menu1;
	TMenuItem *Exit1;
	TFileListBox *FileListBox1;
	TButton *Button16;
	TLabel *Label15;
	TButton *Button18;
	TOpenDialog *OpenDialog1;
	TSaveDialog *SaveDialog1;
	TButton *Button19;
	TButton *Button21;
	TMemo *Memo3;
	TMemo *Memo4;
	TMemo *Memo5;
	TButton *Button22;
	TMemo *Memo6;
	TEdit *Edit5;
	TLabel *Label16;
	TEdit *Edit6;
	TLabel *Label17;
	TLabel *Label18;
	TLabel *Label19;
	TButton *Button15;
	TButton *Button23;
	TEdit *Edit7;
	TEdit *Edit8;
	TEdit *Edit9;
	TButton *Button7;
	TButton *Button25;
	TCheckBox *CheckBox4;
	TEdit *Edit10;
	TCheckBox *CheckBox5;
	TCheckBox *CheckBox6;
	TDirectoryListBox *DirectoryListBox1;
	TButton *Button26;
	TPanel *Panel2;
	TCheckBox *CheckBox7;
	TButton *Button1;
	TRadioGroup *RadioGroup1;
	TButton *Button2;
	TEdit *Edit11;
	TMemo *Memo7;
	void __fastcall Button8Click(TObject *Sender);
	void __fastcall Exit1Click(TObject *Sender);
	void __fastcall Button16Click(TObject *Sender);
	void __fastcall Button18Click(TObject *Sender);
	void __fastcall Button19Click(TObject *Sender);
	void __fastcall Button21Click(TObject *Sender);
	void __fastcall Button22Click(TObject *Sender);
	void __fastcall Button15Click(TObject *Sender);
	void __fastcall Button7Click(TObject *Sender);
	void __fastcall Button25Click(TObject *Sender);
	void __fastcall CheckBox5Click(TObject *Sender);
	void __fastcall CheckBox6Click(TObject *Sender);
	void __fastcall Button26Click(TObject *Sender);
	void __fastcall CheckBox7Click(TObject *Sender);
	void __fastcall Button1Click(TObject *Sender);
	void __fastcall FileListBox1DblClick(TObject *Sender);
	void __fastcall Button2Click(TObject *Sender);

//	void __fastcall Button7Click(TObject *Sender);

private:	// User declarations
	bool isImageSrc;

	bool isWhitening;




public:		// User declarations

	__fastcall TForm1(TComponent* Owner);
};


//my function declarations
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
	);
double GetDist(double *a, double *b, int data_length);


//---------------------------------------------------------------------------
extern PACKAGE TForm1 *Form1;
//---------------------------------------------------------------------------
#endif
