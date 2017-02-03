//---------------------------------------------------------------------------

#ifndef MyParticleH
#define MyParticleH
//---------------------------------------------------------------------------
class MyParticle2D{
	public:
		int x;                 //方向, 粒子光流單位向量
		int y;
		double length;          //向量長度, 即粒子之光流大小

		MyParticle2D();
		MyParticle2D(int x, int y);

		void CalculateLength();

};

//---------------------------------------------------------------------------
#endif
