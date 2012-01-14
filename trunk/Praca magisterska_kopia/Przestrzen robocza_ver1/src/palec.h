#ifndef _LIBS
#define _LIBS

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <vector>
#include <iostream>
#include <string>
#define EPSILON 0.001
#endif

class Paliczek
{
private:
	double _Dlugosc;
	double _Dolna_Podstawa;
	double _Gorna_Podstawa;
public:
	Paliczek(): _Dlugosc(0), _Dolna_Podstawa(0), _Gorna_Podstawa(0) {}
	Paliczek(double Dl, double DP, double GP): _Dlugosc(Dl), _Dolna_Podstawa(DP), _Gorna_Podstawa(GP) {}
	double Dlugosc(){return _Dlugosc;}
	double Dolna_Podstawa(){return _Dolna_Podstawa;}
	double Gorna_Podstawa(){return _Gorna_Podstawa;}
};

class Palec
{
private:
	unsigned int _Liczba_Paliczkow;
	unsigned int _Liczba_Przegubow;
	KDL::Frame _Przeksztalcenie_Do_Poczatku;
	KDL::Chain _Struktura;
	KDL::Chain _Struktura3W;
	std::vector<Paliczek> _Paliczki;
	KDL::ChainFkSolverPos_recursive *_FKSolver;
	KDL::ChainIkSolverVel_pinv *_IKSolverV;
	KDL::ChainIkSolverPos_NR_JL *_IKSolver;
	KDL::ChainFkSolverPos_recursive *_FKSolver3W;
	KDL::ChainIkSolverVel_pinv *_IKSolverV3W;
	KDL::ChainIkSolverPos_NR_JL *_IKSolver3W;
	KDL::JntArray _Min_Przegubow;
	KDL::JntArray _Max_Przegubow;
	KDL::JntArray _Konf_Przegubow;

public:
  
	Palec(): _Liczba_Paliczkow(0), _Liczba_Przegubow(0) { }
	Palec(unsigned int Lpal, unsigned int Lprz): _Liczba_Paliczkow(Lpal), _Liczba_Przegubow(Lprz), _FKSolver(NULL), _IKSolverV(NULL),
				_IKSolver(NULL), _FKSolver3W(NULL), _IKSolverV3W(NULL), _IKSolver3W(NULL), 
				_Min_Przegubow(KDL::JntArray(Lprz)), _Max_Przegubow(KDL::JntArray(Lprz)),
				_Konf_Przegubow(KDL::JntArray(Lprz)) { }
	//Palec(const Palec &P);
	~Palec() { delete _FKSolver; delete _IKSolverV; delete _IKSolver; delete _FKSolver3W; delete _IKSolverV3W; delete _IKSolver3W; }
	unsigned int Liczba_Paliczkow(){return _Liczba_Paliczkow;}
	unsigned int Liczba_Przegubow(){return _Liczba_Przegubow;}
	void DodajPaliczek(Paliczek & _P);
	void DodajPrzegub(double Min, double Max, double Obecna, unsigned int i);
	KDL::JntArray Min_Przegubow() { return _Min_Przegubow; }
	KDL::JntArray Max_Przegubow() { return _Max_Przegubow; }
	KDL::JntArray Konf_Przegubow() { return _Konf_Przegubow; }
	KDL::Frame Przeksztalcenie_Do_Poczatku(){ return _Przeksztalcenie_Do_Poczatku;}
	void Przeksztalcenie_Do_Poczatku(KDL::Frame Cel){_Przeksztalcenie_Do_Poczatku=Cel;}
	std::vector<Paliczek> Paliczki() { return _Paliczki; }
	double KatRoznicyPodstawPaliczka(unsigned int Paliczek);
	double DluzszaPodstawa(unsigned int Paliczek);
	double SredniaZPodstaw(unsigned int Paliczek);
	
	KDL::ChainFkSolverPos_recursive* Fk() { return _FKSolver; }
	KDL::ChainIkSolverPos_NR_JL* Ik() { return _IKSolver; }
	KDL::ChainFkSolverPos_recursive* Fk3W() { return _FKSolver3W; }
	KDL::ChainIkSolverPos_NR_JL* Ik3W() { return _IKSolver3W; }
	
	void Wyznacz_Strukture();
	KDL::Frame Wyznacz_Polozenie(KDL::Frame P0, KDL::Frame Pk, KDL::Vector V0, KDL::Vector Vk, double s);
	bool ZaplanujSciezke(std::vector<KDL::Frame> Punkty_Chwytu, unsigned int Krokow, std::fstream &Plik_Wynikowy);
};