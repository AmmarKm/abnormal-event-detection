//---------------------------------------------------------------------------


#pragma hdrstop

#include "MyParticle.h"
#include "math.h"

//---------------------------------------------------------------------------

#pragma package(smart_init)

//---------------------------------------------------------------------------

MyParticle2D::MyParticle2D(){

//	MyParticle2D(0, 0);
	this->x = 0;
	this->y = 0;
	this->length = 0;


}

MyParticle2D::MyParticle2D(int x, int y){

	this->x = x;
	this->y = y;
	this->length = 0;
//	this->length = sqrt(x*x+y*y);
}

void MyParticle2D::CalculateLength(){

	int x = this->x;
	int y = this->y;

	this->length = sqrt(x*x+y*y);

}


