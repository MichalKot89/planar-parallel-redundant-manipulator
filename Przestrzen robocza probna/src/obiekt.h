#include "palec.h"

class Obiekt
{
private:
	KDL::Frame _Srodek;
	unsigned int _Typ;
public:
	Obiekt(): _Srodek(KDL::Frame::Identity()), _Typ(0)  {}
	Obiekt(KDL::Frame V, unsigned int T): _Srodek(V), _Typ(T) {}
	unsigned int Typ() { return _Typ; }
	KDL::Frame Srodek() { return _Srodek; }
	virtual std::vector<KDL::Frame> WyznaczPunktyChwytu(std::vector<Palec>* _Palce) = 0;
	virtual float Promien() { return 0; }
	virtual float Wysokosc() { return 0; }
};

class Cylindryczny : public Obiekt
{
private:
	float _Promien;
	float _Wysokosc;
public:
	Cylindryczny(): Obiekt(KDL::Frame::Identity(), 0), _Promien(0), _Wysokosc(0) {}
	Cylindryczny(KDL::Frame F, float P, float W): Obiekt(F, 0), _Promien(P), _Wysokosc(W) {}
	float Promien() { return _Promien; }
	float Wysokosc() { return _Wysokosc; }
	std::vector<KDL::Frame> WyznaczPunktyChwytu(std::vector<Palec>* _Palce);
};

class Szczypcowy : public Obiekt
{
private:
	float _Bok1;
	float _Bok2;
	float _Wysokosc;
public:
	Szczypcowy(): Obiekt(KDL::Frame::Identity(), 1), _Bok1(0), _Bok2(0), _Wysokosc(0) {}
	Szczypcowy(KDL::Frame F, float B1, float B2, float W): Obiekt(F, 1), _Bok1(B1), _Bok2(B2), _Wysokosc(W) {}
	float Bok1() { return _Bok1; }
	float Bok2() { return _Bok2; }
	float Wysokosc() { return _Wysokosc; }
	std::vector<KDL::Frame> WyznaczPunktyChwytu(std::vector<Palec>* _Palce);
};



class Hakowy : public Obiekt
{
private:
	float _Szerokosc;
	float _Dlugosc;
	float _Wysokosc;
public:
	Hakowy(): Obiekt(KDL::Frame::Identity(), 2), _Szerokosc(0), _Dlugosc(0), _Wysokosc(0) {}
	Hakowy(KDL::Frame F, float S, float D, float W): Obiekt(F, 2), _Szerokosc(S), _Dlugosc(D), _Wysokosc(W) {}
	float Szerokosc() { return _Szerokosc; }
	float Dlugosc() { return _Dlugosc; }
	float Wysokosc() { return _Wysokosc; }
	std::vector<KDL::Frame> WyznaczPunktyChwytu(std::vector<Palec>* _Palce);
};


class Dloniowy : public Obiekt
{
private:
	float _Promien;
	float _Dlugosc;
public:
	Dloniowy(): Obiekt(KDL::Frame::Identity(), 3), _Promien(0), _Dlugosc(0) {}
	Dloniowy(KDL::Frame F, float P, float D): Obiekt(F, 3), _Promien(P), _Dlugosc(D) {}
	float Promien() { return _Promien; }
	float Dlugosc() { return _Dlugosc; }
	std::vector<KDL::Frame> WyznaczPunktyChwytu(std::vector<Palec>* _Palce);
};



class Sferyczny : public Obiekt
{
private:
	float _Promien;
public:
	Sferyczny(): Obiekt(KDL::Frame::Identity(), 4), _Promien(0) {}
	Sferyczny(KDL::Frame F, float P): Obiekt(F, 4), _Promien(P) {}
	float Promien() { return _Promien; }
	std::vector<KDL::Frame> WyznaczPunktyChwytu(std::vector<Palec>* _Palce);
};


class Lateralny : public Obiekt
{
private:
	float _Grubosc;
public:
	Lateralny(): Obiekt(KDL::Frame::Identity(), 5), _Grubosc(0) {}
	Lateralny(KDL::Frame F, float G): Obiekt(F, 5), _Grubosc(G) {}
	float Grubosc() { return _Grubosc; }
	std::vector<KDL::Frame> WyznaczPunktyChwytu(std::vector<Palec>* _Palce);
};