//---------------------------------------------------------------------------

#ifndef MyParticleH
#define MyParticleH
//---------------------------------------------------------------------------
class MyParticle2D{
	public:
		int x;                 //��V, �ɤl���y���V�q
		int y;
		double length;          //�V�q����, �Y�ɤl�����y�j�p

		MyParticle2D();
		MyParticle2D(int x, int y);

		void CalculateLength();

};

//---------------------------------------------------------------------------
#endif
