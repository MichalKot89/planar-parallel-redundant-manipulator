#include "obiekt.h"


class Reka
{
private:
	unsigned int _Liczba_Palcow;
	std::vector<Palec> _Palce;
	std::fstream Plik_Wynikowy;
	Obiekt* _Obiekt;
public:
	Reka(): _Liczba_Palcow(0)  {}
	Reka(unsigned int Lpal): _Liczba_Palcow(Lpal) {}
	unsigned int Liczba_Palcow(){return _Liczba_Palcow;}
	std::vector<Palec> *Palce() { return &_Palce; }
	void DodajPalec(Palec & _P);

	void Pomin_Komentarze(std::istream & Plik);
	bool Wczytaj_Dane(std::string Nazwa);
	bool Wczytaj_Obiekt(std::string Nazwa);
	void Stworz_Struktury();
	void Test();
	void Planuj(unsigned int Liczba_Krokow);
};

std::ostream & operator << (std::ostream & StrWyj, KDL::JntArray Konfiguracja);
double OdlegloscDwochPunktow(KDL::Vector V1, KDL::Vector V2);
double PitagorasPrzyprostokatna(double Z1, double Z2);
double PitagorasPrzeciwprostokatna(double Z1, double Z2);
