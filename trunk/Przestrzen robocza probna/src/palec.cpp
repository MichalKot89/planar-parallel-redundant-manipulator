#include "reka.h"

using namespace KDL;
using namespace std;

/*
 * Metoda DodajPaliczek
 *
 * Słuzy do dodania paliczka do wektora paliczkow palca
 *
 */
void Palec::DodajPaliczek(Paliczek & _P)
{
	_Paliczki.push_back(_P);
}

/*
 * Metoda DodajPrzegub
 *
 * Słuzy do dodania przegubu i jego wartosci
 * minimalnych i maksymalnych
 *
 */
void Palec::DodajPrzegub(double Min, double Max, double Obecna, unsigned int i)
{

	_Min_Przegubow(i)=Min;
	_Max_Przegubow(i)=Max;
	_Konf_Przegubow(i)=Obecna;
}

/*
 * Metoda WyznaczStrukture
 *
 * Słuzy do dodania poszczegolnych segmentow
 * tworzac strukture manipulatora, a takze
 * tworzy solvery kinematyki dla danego lancucha
 *
 */
void Palec::Wyznacz_Strukture()
{
  _Struktura.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation::EulerZYX(0.0,0.0,PI/2))));
  for(unsigned int i=0;i<Liczba_Paliczkow();i++)
  {
    _Struktura.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(Paliczki().at(i).Dlugosc(),0.0,0.0))));
    _Struktura3W.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(Paliczki().at(i).Dlugosc(),0.0,0.0))));
  }
  _FKSolver = new ChainFkSolverPos_recursive(_Struktura);
  _IKSolverV = new ChainIkSolverVel_pinv(_Struktura);
  _IKSolver = new ChainIkSolverPos_NR_JL(_Struktura, _Min_Przegubow, _Max_Przegubow, *_FKSolver, *_IKSolverV, 1000, EPSILON);
  _FKSolver3W = new ChainFkSolverPos_recursive(_Struktura3W);
  _IKSolverV3W = new ChainIkSolverVel_pinv(_Struktura3W);
  _IKSolver3W = new ChainIkSolverPos_NR_JL(_Struktura3W, _Min_Przegubow, _Max_Przegubow, *_FKSolver3W, *_IKSolverV3W, 1000, 1e-2);
}


/*
 * Metoda Wyznacz_Polozenie
 *
 * sluzy do wyznaczenia polozenia i orientacji efektora
 * w zaleznosci od parametru s, ktory parametryzuje odcinek
 * na przedziale (0,1)
 *
 */

KDL::Frame Palec::Wyznacz_Polozenie(KDL::Frame P0, KDL::Frame Pk, KDL::Vector V0, KDL::Vector Vk, double s)
{
  Twist delta_twist;
  
  if(s<=0)
    return P0;
  else if(s>=1)
    return Pk;
  
  // wyznaczenie orientacji efektora
  // poprzez wyznaczenie roznicy katowej i wyzerowanie roznicy predkosciowej
  delta_twist=s*diff(P0, Pk);
  delta_twist.vel.x(0);
  delta_twist.vel.y(0);
  delta_twist.vel.z(0);
  P0=addDelta(P0, delta_twist);
  
  // obliczenie polozenia efektora
  P0.p = P0.p*(1-3*pow(s, 2)+2*pow(s, 3)) + Pk.p*(3*pow(s, 2)-2*pow(s,3)) + V0*(s-2*pow(s,2)+pow(s,3)) + Vk*(-pow(s,2)+pow(s,3));
  
  return P0;
  
}

/*
 * Metoda ZaplanujSciezke
 *
 * sluzy do wyznaczenia i zapisania do pliku konfiguracji przegubow
 * palca w zadanej liczbie krokow, na sciezce od polozenia aktualnego
 * do polozenia zadanego jako KDL::Frame Punkt_Chwytu
 *
 */


bool Palec::ZaplanujSciezke(std::vector<KDL::Frame> Punkty_Chwytu, unsigned int Krokow, std::fstream &Plik_Wynikowy)
{
  KDL::Frame ObecnePolozenie, TymczasowePolozenie;
  JntArray q(Liczba_Przegubow());
  JntArray qq(Liczba_Przegubow());

  for(unsigned int i=0;i<Punkty_Chwytu.size();i++)
  {
    // Wyznaczenie aktualnego polozenia efektora na podstawie konfiguracji przegubow w przypadku pierwszego punktu
    // lub przyjecie poprzedniego punktu jako obecne polozenie
    if(i==0)
      Fk()->JntToCart(Konf_Przegubow(), ObecnePolozenie);
    else
      ObecnePolozenie=Punkty_Chwytu.at(i-1);

    // Wyznaczenie odleglosci pomiedzy punktem poczatkowym a koncowym ruchu
    double Odleglosc=OdlegloscDwochPunktow(ObecnePolozenie.p, Punkty_Chwytu.at(i).p);


    // Jesli jest rotacja w osi z, nie ma rotacji w innych osiach
    if(fabs(diff(ObecnePolozenie, Punkty_Chwytu.at(i)).rot.z())>EPSILON)
    { 
      // wyliczenie konfiguracji przegubow w punkcie docelowym
      // zostaje ona zapisana do zmiennej qq
      if(Ik()->CartToJnt(Konf_Przegubow(),Punkty_Chwytu.at(i),qq)<0)
	return false;
      for(unsigned int j=0;j<=Krokow;j++)
      {
	q=qq;
	// wyznaczenie wartosci kata w pierwszym przegubie
	q(0)=Konf_Przegubow()(0)+((1/(double)Krokow)*(double)j)*(qq(0)-Konf_Przegubow()(0));
	if(j==Krokow)
	  Plik_Wynikowy << "*";
	// zapis do pliku wynikowego
	Plik_Wynikowy << q;
      }
    }
    // Obliczenie konfiguracji przegubow w rownooddalonych punktach
    // sciezki od poczatkowego punktu do koncowego
    else
    {
      for(unsigned int j=0;j<=Krokow;j++)
      {
	// okreslenie poczatkowego i koncowego wektora predkosci
	Vector V0(0, Odleglosc, 0);
	Vector Vk(0, Odleglosc, 0);
	// przeksztalcenie wektorow dla punktu poczatkowego i koncowego
	V0 = ObecnePolozenie.M*V0;
	Vk = Punkty_Chwytu.at(i).M*Vk;

	// wywolanie algorytmu de Casteljeu
	TymczasowePolozenie=Wyznacz_Polozenie(ObecnePolozenie, Punkty_Chwytu.at(i), V0, Vk, (1/(double)Krokow)*(double)j);

	while(Ik()->CartToJnt(Konf_Przegubow(),TymczasowePolozenie,q)<0)
	{
	  // jesli nie powiodla sie sciezka liniowa zwroc blad
	  if(V0.z()<=0)
	    return false;
	  // zmniejszanie wektorow o polowe
	  V0=V0/2;
	  Vk=Vk/2;
	  // zmiana wektorow predkosci na wektory zerowe
	  if(V0.z()<1)
	  {
	    V0 = Vector::Zero();
	    Vk = Vector::Zero();
	  }
	  // wywolanie algorytmu de Casteljeu
	  TymczasowePolozenie=Wyznacz_Polozenie(ObecnePolozenie, Punkty_Chwytu.at(i), V0, Vk, (1/(double)Krokow)*(double)j);
	}
	if(j==Krokow)
	  Plik_Wynikowy << "*";
	// zapis do pliku wynikowego
	Plik_Wynikowy << q;
      }
    }
  }
  /*
  Temp=Wyznacz_Polozenie(Obecne_Polozenie, Punkt_Chwytu, V0, Vk, 0.5);
  //std::cout << Obecne_Polozenie << std::endl << std::endl;
  std::cout << Punkt_Chwytu << std::endl << std::endl;
  std::cout << Temp << std::endl;
    
    int ret = Ik()->CartToJnt(Konf_Przegubow(),Temp,q);
    if(ret>=0)
      for(unsigned int i=0;i<Liczba_Przegubow();i++)
	  std::cout << 180*q(i)/PI <<std::endl;
  Plik_Wynikowy << q;*/
  return true;
}

double Palec::KatRoznicyPodstawPaliczka(unsigned int Paliczek)
{ return atan((Paliczki().at(Paliczek).Dolna_Podstawa()-Paliczki().at(Paliczek).Gorna_Podstawa()) /
	    Paliczki().at(Paliczek).Dlugosc()); }
	    
double Palec::DluzszaPodstawa(unsigned int Paliczek)
{ return max(Paliczki().at(Paliczek).Dolna_Podstawa(),Paliczki().at(Paliczek).Gorna_Podstawa()); }

double Palec::SredniaZPodstaw(unsigned int Paliczek)
{ return (Paliczki().at(Paliczek).Dolna_Podstawa()+Paliczki().at(Paliczek).Gorna_Podstawa())/2; }