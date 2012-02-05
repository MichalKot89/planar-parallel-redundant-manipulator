#include "reka.h"

using namespace KDL;
using namespace std;

/*
 * Metody WyznaczPunktyChwytu
 *
 * Słuza wyznaczeniu docelowych punktow chwytu dla
 * zadanych palcow
 *
 */

/*
 * Chwyt Cylindryczny
 *
 * Chwyt obiektu, ktorego jedynym używanym parametrem jest promien, 
 * natomiast wysokosc obiektu nie jest brana pod uwage (palce dla ktorych zabraknie miejsca zawisna w przestrzeni)
 * Korzystajac z metody znajdywania stycznej do okregu z punktu wyznaczamy katy nachylenia kolejnych przegubow palcow pomijajac
 * kciuka. Do tak wyznaczonego kata dodawany jest takze kat wynikajacy z roznicy dlugosci podstaw(w przypadku roznicy
 * powstawalaby przerwa pomiedzy obiektem a paliczkiem). Dzieki temu kazdy z paliczkow dotyka obiektu, dzieki czemu chwyt jest
 * pewniejszy. Nie wiemy, ktora czesc paliczka konkretnie dotyka przedmiotu, bazujemy jedynie na katach.
 * W przypadku kciuka okreslamy konkretnie punkt chwytu, dla uproszczenia we wspolrzednych bazowych, a nastepnie transformujemy
 * go do ukladu kciuka. Nastepnie wyznaczamy kat obrotu pierwszego przegubu w taki sposob, aby zadany punkt znalazl sie
 * na plaszczyznie zgiecia pozostalych przegubow (potrojne wahadlo). 
 * Kolejnym krokiem jest obranie kata, od ktorego bedziemy poszukiwac mozliwej do spelnienia orientacji efektora w zadanym punkcie.
 * Zmieniejac wartosc kata (w tym przypadku od 90 stopni co 3 stopnie do 0) probujemy obliczyc odwrotna kinematyke. Jezeli ta
 * operacja sie powiedze, znalezlismy zadana orientacje. Jesli nie, chwyt nie jest mozliwy, zostaje zwrocony komunikat o bledzie
 * i program konczy swoje dzialanie. 
 
 * Wyznaczone punkty chwytu w postaci kinematyki (orientacja + wektor) zostaja zapisane do wektora Punkty i zwrocone jako wynik
 * metody. W tym przypadku wyznaczamy po 3 punkty dla kazdego z palcow, przy czym punkt 1 jest niezmiennym, poczatkowym
 * punktem rozpoczynajacym chwyt.
 */
std::vector<KDL::Frame> Cylindryczny::WyznaczPunktyChwytu(std::vector<Palec>* _Palce)
{
  std::vector<KDL::Frame> Punkty;
  std::vector<KDL::Frame>::iterator it;
  Frame F;
  Frame PrzekszOdwrotne;
  Vector P1, Ps, O;
  Vector *Temp_Vector;
  Rotation *Temp_Rotation;
  JntArray Konfiguracja = JntArray(4);
  JntArray KonfiguracjaPrzegubow = JntArray(4);
  JntArray KonfiguracjaWynikowa = JntArray(4);
  double Dlugosc=0;
  double OdlegloscOdSrodka;
  double Kat, Alfa, Beta, Gamma;
  
  for(unsigned int i=0;i<_Palce->at(0).Liczba_Paliczkow();i++)
    Dlugosc+=_Palce->at(0).Paliczki().at(i).Dlugosc();
   if((Promien()>(Dlugosc/2)) || (Promien()<15)) // jesli palec wskazujacy jest krotszy niz 1/8 obwodu
     { std::cerr << "Nie da sie chwycic tego obiektu ta reka" << std::endl; exit(1); }
     
   // Tutaj wyjatkowo zrobimy najpierw palce
 // Wyznaczenie punktow chwytu dla kolejnych palcow
  for(unsigned int i=1;i<_Palce->size();i++)
  {
    
    // PALCE 1: Wyznaczenie konfiguracji w polozeniu 1
    // zakladamy kat +- 20 stopni w pierwszym przegubie w palcach poza srodkowym
    KonfiguracjaPrzegubow(0)=0*PI/180; 
    KonfiguracjaPrzegubow(1)=0*PI/180;
    KonfiguracjaPrzegubow(2)=10*PI/180;
    KonfiguracjaPrzegubow(3)=15*PI/180;
    
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
    
    // PALCE 2: Wyznaczenie konfiguracji w polozeniu 2
    // KAT 1 - wynikajacy z roznicy podstaw paliczka, wychodzimy z zalozenia, ze dolna podstawa pierwszego paliczka
    // zadnego z palcow nie jest wieksza niz u palca srodkowego
    Kat=_Palce->at(i).KatRoznicyPodstawPaliczka(0);
    KonfiguracjaPrzegubow(1)=Kat;
//     std::cout << "Kat1-" << i << ": " << Kat*180/PI << std::endl;

    // KAT 2:
    // Wartosc promienia powiekszona o dolna grubosc paliczka pierwszego palca srodkowego
    O = Vector(0, 0, Promien()+_Palce->at(2).Paliczki().at(0).Dolna_Podstawa()); // srodek okregu
    P1 = Vector(_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.x() // punkt w ktorym konczy sie poprzedni paliczek
		  + _Palce->at(i).Paliczki().at(0).Dlugosc()*cos(KonfiguracjaPrzegubow(1)), 
		0, 
		_Palce->at(i).Paliczki().at(0).Dlugosc()*sin(KonfiguracjaPrzegubow(1)));
    OdlegloscOdSrodka = OdlegloscDwochPunktow(P1, O); // odleglosc punkty od srodka okregu
    Alfa = atan2(P1.x()-O.x(), O.z()-P1.z());
    Beta = acos((Promien() + _Palce->at(i).SredniaZPodstaw(1))/OdlegloscOdSrodka); 
    Gamma = Alfa + Beta - PI/2;
    Kat = PI - (PI/2 - Gamma) - KonfiguracjaPrzegubow(1);
    
    // powiekszenie katu przegubu o kat wynikajacy z roznicy podstaw paliczka
    Kat+=_Palce->at(i).KatRoznicyPodstawPaliczka(1);

    // sprawdzenie czy wyznaczony kat nie przekracza dopuszczalnych wartosci
    // jesli przekracza, przyjecie wartosci przegowych
    if(Kat > _Palce->at(i).Max_Przegubow()(2))
      KonfiguracjaPrzegubow(2)=_Palce->at(i).Max_Przegubow()(2);
    else if(Kat < _Palce->at(i).Min_Przegubow()(2))
      KonfiguracjaPrzegubow(2)=_Palce->at(i).Min_Przegubow()(2);
    else
      KonfiguracjaPrzegubow(2)=Kat;
    
    // KAT 3:
    // Wartosc promienia powiekszona o grubosc paliczka
    // Wyznaczenie odleglosci dwoch punktow: srodka kola w plaszczyznie ruchu oraz punktu, w ktorym
    // konczy sie drugi paliczek
    P1 = Vector(_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.x()
		  + _Palce->at(i).Paliczki().at(1).Dlugosc()*cos(KonfiguracjaPrzegubow(2)+KonfiguracjaPrzegubow(1)) 
		  + _Palce->at(i).Paliczki().at(0).Dlugosc()*cos(KonfiguracjaPrzegubow(1)), // punkt w ktorym konczy sie pierwszy paliczek, 
		0, 
		_Palce->at(i).Paliczki().at(1).Dlugosc()*sin(KonfiguracjaPrzegubow(2)+KonfiguracjaPrzegubow(1)) 
		  + _Palce->at(i).Paliczki().at(0).Dlugosc()*sin(KonfiguracjaPrzegubow(1)));
//cout << endl << "P1: " << P1 << endl << endl;
    OdlegloscOdSrodka = OdlegloscDwochPunktow(P1, O); //cout << "OdlOdSrd: " << OdlegloscOdSrodka << endl;
    Alfa = atan2(P1.x()-O.x(), O.z()-P1.z()); //cout << "Alfa: " << Alfa << endl;
    Beta = acos((Promien() + _Palce->at(i).SredniaZPodstaw(2))/OdlegloscOdSrodka);// cout << "Beta: " << Beta << endl;
    Ps = Vector(O.x() + Promien()*sin(Alfa + Beta), 0, O.z() + Promien()*cos(Alfa + Beta));
    Gamma = Alfa + Beta - PI/2;// cout << "Gamma: " << Gamma << endl;
    Kat = PI - (PI/2 - Gamma) - KonfiguracjaPrzegubow(1) - KonfiguracjaPrzegubow(2);
//     if(Gamma >= 0)
      Kat = PI - (PI/2 - Gamma) - KonfiguracjaPrzegubow(1) - KonfiguracjaPrzegubow(2);  //cout << "Kat: " << Kat << endl;
//     else
//       Kat = (PI/2 - Gamma) - KonfiguracjaPrzegubow(1) - KonfiguracjaPrzegubow(2);
    KonfiguracjaPrzegubow(3)=Kat;

    // Ustawienie konfiguracji posredniej rozniacej sie jedynie dwoma ostatnimi przegubami
    // efektor bedzie ustawiony w poblizu punktu docelowego
    Konfiguracja(0) = KonfiguracjaPrzegubow(0);
    Konfiguracja(1) = KonfiguracjaPrzegubow(1);
    Konfiguracja(2) = KonfiguracjaPrzegubow(2)-5*PI/180;
    Konfiguracja(3) = KonfiguracjaPrzegubow(3)-10*PI/180;
    _Palce->at(i).Fk()->JntToCart(Konfiguracja,F);
    Punkty.push_back(F);
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);

cout << KonfiguracjaPrzegubow;
  } 

  // KCIUK 1: Wyznaczenie rotacji i wektora kciuka w polozeniu 1
  // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
  Temp_Vector = new Vector(-PitagorasPrzyprostokatna(Promien(), Promien()/2)
			    -_Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa()
			    -5, 
			   Promien() + Promien()/2 + _Palce->at(2).Paliczki().at(0).Dolna_Podstawa(),
			   0);

  // Wyznaczenie przeksztalcenia odwrotnego, tzn z ukladu kciuka do ukladu bazowego, tak, aby
  // wyznaczony w ukladzie bazowym wektor punktu chwytu przeksztalcic do ukladu kciuka 
  PrzekszOdwrotne = Frame(_Palce->at(0).Przeksztalcenie_Do_Poczatku().M.Inverse(),
			  _Palce->at(0).Przeksztalcenie_Do_Poczatku().M.Inverse()*(-_Palce->at(0).Przeksztalcenie_Do_Poczatku().p));
  (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);

  KonfiguracjaPrzegubow(0)=atan((*Temp_Vector).y()/(*Temp_Vector).x());
  KonfiguracjaPrzegubow(1)=0*PI/180;
  KonfiguracjaPrzegubow(2)=0*PI/180;
  KonfiguracjaPrzegubow(3)=0*PI/180;
  _Palce->at(0).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
  it=Punkty.begin();
  Punkty.insert(it, F);  
  
  // KCIUK 2: Wyznaczenie rotacji i wektora kciuka w polozeniu 2
  // Wyznaczenie odleglosci od reki na podstawie zadanej konfiguracji bazowej poczatkowej palca wskazkujacego
  
  
  Kat=180;
  Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
  (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
  (*Temp_Rotation).DoRotZ(Kat*PI/180);
     
  while(_Palce->at(0).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
  {
    Kat-=6;
    delete Temp_Rotation;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
    (*Temp_Rotation).DoRotZ(Kat*PI/180); 
    if(Kat<0)
    { 
      std::cerr << "Kciuk jest zbyt krotki i nie da sie chwycic tego obiektu ta reka" << std::endl; 
      exit(1); 
    }
  }

  // Ustawienie wynikowej konfiguracji jako bazowej
  KonfiguracjaPrzegubow=KonfiguracjaWynikowa;
  
  it=Punkty.begin();
  Punkty.insert(it+1, Frame(*Temp_Rotation, *Temp_Vector)); 
  // usuniecie pamieci tymczasowej
  delete Temp_Vector;
  delete Temp_Rotation;
  
  // KCIUK 3: Wyznaczenie orientacji i wektora kciuka w polozeniu 3
  // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
  Temp_Vector = new Vector(- PitagorasPrzyprostokatna(Promien(),Promien()/2)
			    - _Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa(), 
			   Promien() + Promien()/2 + _Palce->at(2).Paliczki().at(0).Dolna_Podstawa(), 
			   0);
  
  (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);
  
  Kat=180;
  Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
  (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
  (*Temp_Rotation).DoRotZ(Kat*PI/180);
  
  while(_Palce->at(0).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
  {
    Kat-=6;
    delete Temp_Rotation;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
    (*Temp_Rotation).DoRotZ(Kat*PI/180); 
    if(Kat<0)
    { 
      std::cerr << "Kciuk jest za krotki i nie da sie chwycic tego obiektu ta reka" << std::endl; 
      exit(1); 
    }
  }
  it=Punkty.begin();
  Punkty.insert(it+2, Frame(*Temp_Rotation, *Temp_Vector)); 
  // KONIEC KCIUKA
cout << KonfiguracjaWynikowa;
  return Punkty; 
}

/*
 * Chwyt Szczypcowy
 *
 * Chwyt obiektu, ktorego najistotniejszym parametrem jest jego wysokosc, natomiast sprawdzana jest
 * jeszcze dlugosc jednego z bokow, w celu zweryfikowania czy chwyt jest w ogole mozliwy (zbyt dlugi obiekt)
 * Po wyznaczeniu stalego, pierwszego ustawienia palcow poszukujemy ustawien 2 i 3. W tym celu okreslamy punkty chwytu
 * z dwoch stron przedmiotu (kciuk z jednej, reszta palcow z drugiej strony), wykorzystujac wysokosc obiektu
 * Jezeli wiemy, ze palec nie jest w stanie siegnac zadanego punktu pozostawiamy go w polozeniu poczatkowym
 * Kolejnym krokiem jest obranie kata, od ktorego bedziemy poszukiwac mozliwej do spelnienia orientacji efektora w zadanym punkcie.
 * Zmieniejac wartosc kata (w tym przypadku od 90 stopni co 3 stopnie do 0, zaczynamy od 90 stopni - najmocniejszy, 
 * prostopadly do obiektu chwyt) probujemy obliczyc odwrotna kinematyke. Jezeli ta
 * operacja sie powiedze, znalezlismy zadana orientacje. Jesli nie, chwyt nie jest mozliwy, zostaje zwrocony komunikat o bledzie
 * i program konczy swoje dzialanie. 
 
 * Wyznaczone punkty chwytu w postaci kinematyki (orientacja + wektor) zostaja zapisane do wektora Punkty i zwrocone jako wynik
 * metody. W tym przypadku wyznaczamy po 3 punkty dla kazdego z palcow, przy czym punkt 1 jest niezmiennym, poczatkowym
 * punktem rozpoczynajacym chwyt.
 */
std::vector<KDL::Frame> Szczypcowy::WyznaczPunktyChwytu(std::vector<Palec>* _Palce)
{
  std::vector<KDL::Frame> Punkty;
  Frame F;
  Frame PrzekszOdwrotne;
  Vector *Temp_Vector;
  Rotation *Temp_Rotation;
  JntArray Konfiguracja = JntArray(4);
  JntArray KonfiguracjaPrzegubow = JntArray(4);
  JntArray KonfiguracjaWynikowa = JntArray(4);
  double Dlugosc=0, Najdluzszy=0;
  double  OdlegloscOdSrodka;
  int Kat;
  
//   Kciuk
  for(unsigned int i=0;i<_Palce->at(1).Liczba_Paliczkow();i++)
    Dlugosc+=_Palce->at(1).Paliczki().at(i).Dlugosc();
   if((Bok1()>Dlugosc) || (Wysokosc()>60) || (Wysokosc()<5))
     { std::cerr << "Nie da sie chwycic tego obiektu ta reka" << std::endl; exit(1); }
  
  // Wyznaczenie odleglosci od reki na podstawie zadanej konfiguracji bazowej poczatkowej palca wskazkujacego
  Konfiguracja(0)=0;
  Konfiguracja(1)=30*PI/180;
  Konfiguracja(2)=(90-Wysokosc()/2)*PI/180;
  Konfiguracja(3)=70*PI/180;
  _Palce->at(1).Fk()->JntToCart(Konfiguracja,F);
  OdlegloscOdSrodka=F.p.z();
  
  // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
  Temp_Vector = new Vector(-Wysokosc()/2 - _Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa()-5, 
			   OdlegloscOdSrodka, 
			   0);
  
  PrzekszOdwrotne = Frame(_Palce->at(0).Przeksztalcenie_Do_Poczatku().M.Inverse(),
			  _Palce->at(0).Przeksztalcenie_Do_Poczatku().M.Inverse()*(-_Palce->at(0).Przeksztalcenie_Do_Poczatku().p));
  (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);
  
  // KCIUK 1: Wyznaczenie rotacji i wektora kciuka w polozeniu 1
  KonfiguracjaPrzegubow(0)=atan((*Temp_Vector).y()/(*Temp_Vector).x());
  KonfiguracjaPrzegubow(1)=0*PI/180;
  KonfiguracjaPrzegubow(2)=0*PI/180;
  KonfiguracjaPrzegubow(3)=0*PI/180;
    
  _Palce->at(0).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
  Punkty.push_back(F);  
  
  // KCIUK 2: Wyznaczenie rotacji i wektora kciuka w polozeniu 2
  
  Kat=0;
  Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
  (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
  (*Temp_Rotation).DoRotZ(Kat*PI/180);
     
  while(_Palce->at(0).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
  {
    Kat+=6;
    delete Temp_Rotation;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
    (*Temp_Rotation).DoRotZ(Kat*PI/180); 
    if(Kat>180)
    { 
      std::cerr << "Kciuk jest zbyt krotki i nie da sie chwycic tego obiektu ta reka" << std::endl; 
      exit(1); 
    }
  }
  // Ustawienie wynikowej konfiguracji jako bazowej
  KonfiguracjaPrzegubow=KonfiguracjaWynikowa;
  
  Punkty.push_back(Frame(*Temp_Rotation, *Temp_Vector)); 
  // usuniecie pamieci tymczasowej
  delete Temp_Vector;
  delete Temp_Rotation;
  
  // KCIUK 3: Wyznaczenie orientacji i wektora kciuka w polozeniu 3
  // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
  Temp_Vector = new Vector(-Wysokosc()/2 - _Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa(), 
			   OdlegloscOdSrodka, 
			   0);

  (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);
  
  Kat=0;
  Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
  (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
  (*Temp_Rotation).DoRotZ(Kat*PI/180);
  
  while(_Palce->at(0).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
  {
    Kat+=6;
    delete Temp_Rotation;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
    (*Temp_Rotation).DoRotZ(Kat*PI/180); 
    if(Kat>180)
    { 
      std::cerr << "Kciuk jest za krotki i nie da sie chwycic tego obiektu ta reka" << std::endl; 
      exit(1); 
    }
  }
  Punkty.push_back(Frame(*Temp_Rotation, *Temp_Vector)); 
cout << KonfiguracjaWynikowa;
  // KONIEC KCIUKA
 
 
  // Znalezienie najdluzszego palca
  for(unsigned int i=1;i<_Palce->size();i++)
  {
    Dlugosc=0;
    for(unsigned int j=0;j<_Palce->at(i).Liczba_Paliczkow();j++)
      Dlugosc+=_Palce->at(i).Paliczki().at(j).Dlugosc();
    if(Dlugosc>=Najdluzszy)
      Najdluzszy=Dlugosc;
  }
  
  // Wyznaczenie punktow chwytu dla kolejnych palcow
  for(unsigned int i=1;i<_Palce->size();i++)
  {
    
    // PALCE 1: Wyznaczenie rotacji i wektora palca w polozeniu 1
    KonfiguracjaPrzegubow(0)=0;
    KonfiguracjaPrzegubow(1)=30*PI/180;
    KonfiguracjaPrzegubow(2)=60*PI/180;
    KonfiguracjaPrzegubow(3)=30*PI/180;
    
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
    
    // Policzenie dlugosci palca w celu sprawdzenia czy chwyt w ogole ma szanse powodzenia
    Dlugosc=0;
    for(unsigned int j=0;j<_Palce->at(i).Liczba_Paliczkow();j++)
      Dlugosc+=_Palce->at(i).Paliczki().at(j).Dlugosc();
    if(Najdluzszy-10>Dlugosc)
    {
cout << KonfiguracjaPrzegubow;
      Punkty.push_back(F);
      Punkty.push_back(F);
      continue;
    }
    // PALEC 2: Wyznaczenie rotacji i wektora palca w polozeniu 2
    
    // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
    Temp_Vector = new Vector(Wysokosc()/2 + _Palce->at(i).Paliczki().at(_Palce->at(i).Liczba_Paliczkow()-1).Gorna_Podstawa() + 5,
			     OdlegloscOdSrodka, 
			     _Palce->at(i).Przeksztalcenie_Do_Poczatku().p.z());
    
    PrzekszOdwrotne = Frame(_Palce->at(i).Przeksztalcenie_Do_Poczatku().M.Inverse(),
			  _Palce->at(i).Przeksztalcenie_Do_Poczatku().M.Inverse()*(-_Palce->at(i).Przeksztalcenie_Do_Poczatku().p));
    (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);
    
    
    Kat=179;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotZ(Kat*PI/180);
    
    
    while(_Palce->at(i).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
    {
      Kat-=6;
      delete Temp_Rotation;
      Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
      (*Temp_Rotation).DoRotZ(Kat*PI/180); 
      if(Kat<90)
      { 
	std::cerr << "Palec " << i << " nie dał rady chwycic tego obiektu" << std::endl; 
	exit(1); 
      }
    }
    
    // Ustawienie wynikowej konfiguracji jako bazowej
    KonfiguracjaPrzegubow=KonfiguracjaWynikowa;
    
    Punkty.push_back(Frame(*Temp_Rotation, *Temp_Vector)); 
    // usuniecie pamieci tymczasowej
    delete Temp_Vector;
    delete Temp_Rotation;
  
    // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
    Temp_Vector = new Vector(Wysokosc()/2 + _Palce->at(i).Paliczki().at(_Palce->at(i).Liczba_Paliczkow()-1).Gorna_Podstawa(), 
			     OdlegloscOdSrodka,
			     _Palce->at(i).Przeksztalcenie_Do_Poczatku().p.z());

    (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);

    Kat=179;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotZ(Kat*PI/180);
    
    while(_Palce->at(i).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
    {
      Kat-=6;
      delete Temp_Rotation;
      Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
      (*Temp_Rotation).DoRotZ(Kat*PI/180); 
      if(Kat<90)
      { 
	std::cerr << "Palec " << i << " niestety nie dał rady chwycic tego obiektu" << std::endl; 
	exit(1); 
      }
    }
    Punkty.push_back(Frame(*Temp_Rotation, *Temp_Vector)); 
    // KONIEC PALCOW
cout << KonfiguracjaWynikowa;
    // usuniecie pamieci tymczasowej
    delete Temp_Vector;
    delete Temp_Rotation;
  }
  return Punkty; 
}

/*
 * Chwyt Hakowy
 *
 * Chwyt obiektu, w ktorym skupiamy sie nie na rozmiarach samego obiektu, a przestrzeni w ktora nalezy wlozyc palce
 * aby chwyt byl mozliwy. W tym wypadku jest to Wysokosc, Dlugosc i Szerokosc
 *
 * Na podstawie dlugosci pierwszego i grubosci drugiego paliczka palca wskazujacego wyznaczamy ustawienie wolnej przestrzeni
 * w stosunku do ukladu poczatkowego dloni. Nastepnie liczymy i sumujemy grubosci poszczegolnych palcow, w celu sprawdzenia
 * czy ten palec, a takze ile palcow lacznie sie w ta przestrzeni zmiesci. Palce nieuczestniczace w chwycie zostaja w polozeniu
 * poczatkowym. Musimy takze okreslic, czy palec "trafi" w przestrzen zginajac sie juz w drugim przegubie, czy dopiero
 * w trzecim badz kolejnych. Na tej podstawie druga pozycje okreslamy z palcami zgietymi o 90 stopni. 
 *
 * Wyjatkowo dla tego chwytu, w tym momencie palce musza poczekac na ruch dloni - ktora przesunie sie w kierunku obiektu, 
 * dzieki czemu palce znajda sie wewnatrz przestrzeni. 
 *
 * Kolejnym krokiem jest wyznaczenie ostatniej pozycji. Dokonujemy tego wykorzystujac znajomosc dlugosci ostatnich paliczkow, 
 * co umozliwia nam wyznaczenie punktu chwytu na krawedzi raczki obiektu, niekoniecznie koncem palca (moze to byc dowolne
 * miejsce ostatniego paliczka). Kat korygujemy o roznice w dlugosciach podstaw ostatniego paliczka.
 * Jesli dlugosc raczki jest dostatecznie mala, zginamy palce maksymalnie o 90 stopni
 *
 * Kciuk nie uczestniczy w tym chwycie
 *
 * Wyznaczone punkty chwytu w postaci kinematyki (orientacja + wektor) zostaja zapisane do wektora Punkty i zwrocone jako wynik
 * metody. W tym przypadku wyznaczamy po 2 punkty dla kazdego z palcow, przy czym punkt 1 jest niezmiennym, poczatkowym
 * punktem rozpoczynajacym chwyt.
 */
std::vector<KDL::Frame> Hakowy::WyznaczPunktyChwytu(std::vector<Palec>* _Palce)
{
  std::vector<KDL::Frame> Punkty;
  Frame F;
  JntArray Konfiguracja = JntArray(4);
  JntArray KonfiguracjaPrzegubow = JntArray(4);
  JntArray KonfiguracjaWynikowa = JntArray(4);
  double DlugoscPalca=0, Najdluzszy=0, Najgrubszy=0, NajgrubszyPalca=0, SzerokoscRazem=0;
  double PolozenieUchwytu, x, y, RoznicaDlugosci;
  double Kat;
  unsigned int OstatniPalec=3;

  // Dodanie punktow kciuka, ktory bedzie pozostawal w miejscu
  _Palce->at(0).Fk()->JntToCart(_Palce->at(0).Konf_Przegubow(),F);
  Punkty.push_back(F);
  Punkty.push_back(F);
//   std::cout << _Palce->at(0).Konf_Przegubow() << std::endl;
  // Wyznaczenie polozenia uchwytu na podstawie gabarytow pierwszego palca
  PolozenieUchwytu = _Palce->at(1).Paliczki().at(0).Dlugosc()-_Palce->at(1).DluzszaPodstawa(1);
// cout << _Palce->at(0).Konf_Przegubow();
  
//   Znalezienie najgrubszego z palcow oraz dlugosci dwoch ostatnich paliczkow w celu okreslenia czy chwyt jest mozliwy
  for(unsigned int i=1;i<_Palce->size();i++) // zaczynamy od 1 bo kciuk sie nie rusza
  {
    DlugoscPalca=0;
    NajgrubszyPalca=0;
    for(unsigned int j=1;j<_Palce->at(i).Liczba_Paliczkow();j++) // zaczynamy od 1 bo pierwszy paliczek sie nie zgina
    {
      if(_Palce->at(i).Paliczki().at(j).Gorna_Podstawa()>Najgrubszy)
	Najgrubszy=_Palce->at(i).Paliczki().at(j).Gorna_Podstawa();
      if(_Palce->at(i).Paliczki().at(j).Dolna_Podstawa()>Najgrubszy)
	Najgrubszy=_Palce->at(i).Paliczki().at(j).Dolna_Podstawa();
      if(_Palce->at(i).Paliczki().at(j).Gorna_Podstawa()>NajgrubszyPalca)// na podstawie najgrubszych paliczkow z poszczegolnych
	NajgrubszyPalca=_Palce->at(i).Paliczki().at(j).Gorna_Podstawa();	// palcow liczymy laczna ich szerokosc
      if(_Palce->at(i).Paliczki().at(j).Dolna_Podstawa()>NajgrubszyPalca)
	NajgrubszyPalca=_Palce->at(i).Paliczki().at(j).Dolna_Podstawa();
      DlugoscPalca+=_Palce->at(i).Paliczki().at(j).Dlugosc();    
    }
    // sprawdzenie czy ze wzgledu na roznice dlugosci pomiedzy palcem 1 i jakims innym palcem dlon nie zmiesci sie w uchwyt
    if(_Palce->at(i).Paliczki().at(0).Dlugosc() + _Palce->at(i).DluzszaPodstawa(1) > PolozenieUchwytu + Wysokosc())
      { std::cerr << "Ze wzgledu na roznice dlugosci palcow dlon nie zmiesci sie w uchwyt" << std::endl; exit(1); }
    if(DlugoscPalca>Najdluzszy)
      Najdluzszy=DlugoscPalca;
    SzerokoscRazem+=NajgrubszyPalca;
    if(SzerokoscRazem>Szerokosc())
      {
	OstatniPalec=i-1;
	break;
      }   
  }
   if((Dlugosc()>Najdluzszy) || (Wysokosc()<2*Najgrubszy) || (OstatniPalec<2)) // 2* bo srednica a nie promien
     { std::cerr << "Nie da sie chwycic tego obiektu ta reka" << std::endl; exit(1); }
   
  // Wyznaczenie punktow chwytu dla kolejnych palcow
  for(unsigned int i=1;i<_Palce->size();i++)
  {
    // PALCE 1: Wyznaczenie rotacji i wektora palca w polozeniu 1
    KonfiguracjaPrzegubow(0)=0;
    KonfiguracjaPrzegubow(1)=0*PI/180;
    if(_Palce->at(i).Paliczki().at(0).Dlugosc() < 
	PolozenieUchwytu+_Palce->at(i).DluzszaPodstawa(1))
    { // przypadek gdy paliczek za krotki, aby zgiac sie w trzecim przegubie
      KonfiguracjaPrzegubow(2)=0*PI/180;
      if(_Palce->at(i).Paliczki().at(0).Dlugosc() + _Palce->at(i).Paliczki().at(1).Dlugosc() < 
	  PolozenieUchwytu+_Palce->at(i).DluzszaPodstawa(2))
	// przypadek gdy paliczek za krotki, aby zgiac sie w ogole
	KonfiguracjaPrzegubow(3)=0*PI/180;
      else // zgiecie paliczka tylko w ostatnim przegubie o 90stopni
	KonfiguracjaPrzegubow(3)=90*PI/180;
    }
    else
    { // zgiecie tylko w trzecim przegubie o 90 stopni i oczekiwanie na wsuniecie
      KonfiguracjaPrzegubow(2)=90*PI/180;
      KonfiguracjaPrzegubow(3)=0*PI/180;
    }
    
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
    
    // Jezeli ostatni kat ma wartosc 90 stopni, to znaczy ze palec nie uczestniczy w chwycie
    if(KonfiguracjaPrzegubow(3)>=85*PI/180)
    {
      Punkty.push_back(F);
cout << KonfiguracjaPrzegubow;
//     std::cout << KonfiguracjaPrzegubow(0)*180/PI << " " << KonfiguracjaPrzegubow(1)*180/PI << " " <<
//     KonfiguracjaPrzegubow(2)*180/PI << " " << KonfiguracjaPrzegubow(3)*180/PI << " " << std::endl;
      continue;
    }

    // Jezeli dlugosc paliczka jest wystarczajaco dluga, ustawienie kata 90 stopni
    if((_Palce->at(i).Paliczki().at(1).Dlugosc() 
	  - _Palce->at(i).DluzszaPodstawa(2)
	  - _Palce->at(2).Paliczki().at(0).Dolna_Podstawa()) >= Dlugosc())
    {
      KonfiguracjaPrzegubow(0)=0;
      KonfiguracjaPrzegubow(1)=0;
      KonfiguracjaPrzegubow(2)=90*PI/180;
      KonfiguracjaPrzegubow(3)=89.9*PI/180;
      _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
      Punkty.push_back(F);
cout << KonfiguracjaPrzegubow;
//     std::cout << KonfiguracjaPrzegubow(0)*180/PI << " " << KonfiguracjaPrzegubow(1)*180/PI << " " <<
//     KonfiguracjaPrzegubow(2)*180/PI << " " << KonfiguracjaPrzegubow(3)*180/PI << " " << std::endl;
      continue;
    }
    // Wyznaczenie punktu chwytu, niekoniecznie koncem palca
    // Wyznaczamy wspolrzedne pomocnicze x, i y wyznaczajace odchylenie paliczka drugiego
    // bazujac na roznicy pomiedzy dlugosciami pierwszych paliczkow palcow pierwszego i obecnego
    // Roznica dlugosci pierwszego paliczka u palca obecnego i pierwszego
    RoznicaDlugosci = _Palce->at(i).Paliczki().at(0).Dlugosc()-_Palce->at(1).Paliczki().at(0).Dlugosc();
    
    x=(_Palce->at(i).Paliczki().at(1).Dlugosc()) * cos((20-(RoznicaDlugosci))*PI/180);
    y=(_Palce->at(i).Paliczki().at(1).Dlugosc()) * sin((20-(RoznicaDlugosci))*PI/180);
    
    // Wyznaczenie z arcusa tangensa kata ostatniego przegubu, poprawione o roznice w podstawach paliczka
    // dzieki czemu palec dotknie obiektu zawsze w tym samym miejscu ale roznym punktem ostatniego paliczka
    Kat = PI/2 
	  - atan((Dlugosc() + _Palce->at(i).SredniaZPodstaw(2) - x)
		/(y + RoznicaDlugosci)) 
	  + _Palce->at(i).KatRoznicyPodstawPaliczka(2);

    // Ustawiamy tylko dwa ostatnie przeguby, reszta nie byla zmieniana
    KonfiguracjaPrzegubow(2)=(70+(RoznicaDlugosci))*PI/180;
    KonfiguracjaPrzegubow(3)=Kat;
    
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
//     std::cout << KonfiguracjaPrzegubow(0)*180/PI << " " << KonfiguracjaPrzegubow(1)*180/PI << " " <<
//     KonfiguracjaPrzegubow(2)*180/PI << " " << KonfiguracjaPrzegubow(3)*180/PI << " " << std::endl;
cout << KonfiguracjaPrzegubow;
  }
  return Punkty; 
}

/*
 * Chwyt Dloniowy
 *
 * Chwyt obiektu, ktorego jedynym nam potrzebnym parametrem jest promien (np. dlugopisu)
 * 
 * Zakladamy, ze zbedna jest nam wiedza o dlugosci obiektu, 
 * skupiamy sie jedynie na chwycie w okreslonym punkcie
 * Palce (oprocz kciuka i palca wskazujacego) ustawiamy w stalym polozeniu, przydzielajac z gory zalozone katy w przegubach, kolejno:
 * 0, 60, 90 i 60 stopni
 *
 * Odleglosc punktu chwytu od dloni wyznaczamy orientacyjnie na podstawie przyjetej stalej konfiguracji palca wskazujacego
 * Ponadto przedmiot ustawiony jest na jednej wysokosci (w osi X ukladu bazowego) z poczatkiem palca wskazujacego,
 * natomiast w osi Z bazowego w taki sposob, aby przy zerowym kacie w pierwszym przegubie palca wskazujacego (brak obrotu w bok)
 * dotykal on palca z boku (chwyt nie opuszkiem, lecz bokiem palca). 
 *
 * Palec wskazujacy dotyka przedmiotu w polowie ostatniego paliczka. W tym przypadku rowniez musimy poszukiwac orientacji
 * palca dla ktorej chwyt powiedzie sie, jednakze tutaj musimy zmieniac rowniez wektor polozenia efektora (stale polozenie srodka
 * ostatniego paliczka, natomiast sam efektor zmienia polozenie w zaleznosci od kata).
 *
 * Polozenie kciuka rowniez zalezy od wyznaczonej odleglosci punktu chwyt od dloni, natomiast w tym przypadku chwytamy
 * opuszkiem palca, w zwiazku z czym musimy uzmienniac jedynie orientacje efektora. Punkt chwytu kciuka ustawiony jest po
 * przeciwleglej stronie obiektu w stosunku do punktu chwytu palca wskazujacego.
 *
 * Wyznaczone punkty chwytu w postaci kinematyki (orientacja + wektor) zostaja zapisane do wektora Punkty i zwrocone jako wynik
 * metody. W tym przypadku wyznaczamy po 2 punkty dla kazdego z palcow, przy czym punkt 1 jest niezmiennym, poczatkowym
 * punktem rozpoczynajacym chwyt.
 */
std::vector<KDL::Frame> Dloniowy::WyznaczPunktyChwytu(std::vector<Palec>* _Palce)
{
  std::vector<KDL::Frame> Punkty;
  Frame F;
  Frame PrzekszOdwrotne;;
  Vector *Temp_Vector;
  Rotation *Temp_Rotation;
  JntArray Konfiguracja = JntArray(4);
  JntArray KonfiguracjaPrzegubow = JntArray(4);
  JntArray KonfiguracjaWynikowa = JntArray(4);
  double DlugoscPalca=0;
  double  OdlegloscOdSrodka;
  int Kat;
  
//   Kciuk
  for(unsigned int i=0;i<_Palce->at(1).Liczba_Paliczkow();i++)
    DlugoscPalca+=_Palce->at(1).Paliczki().at(i).Dlugosc();
   if((Promien()>DlugoscPalca/2))
     { std::cerr << "Nie da sie chwycic tego obiektu ta reka" << std::endl; exit(1); }
  
  // KCIUK 1: Wyznaczenie rotacji i wektora kciuka w polozeniu 1
  // Wyznaczenie odleglosci od reki na podstawie zadanej konfiguracji bazowej poczatkowej palca wskazkujacego
  Konfiguracja(0)=0;
  Konfiguracja(1)=70*PI/180;
  Konfiguracja(2)=(75+Promien())*PI/180;
  Konfiguracja(3)=15*PI/180;
  _Palce->at(1).Fk()->JntToCart(Konfiguracja,F);
  OdlegloscOdSrodka=F.p.z();
  

  // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
  Temp_Vector = new Vector(_Palce->at(1).Przeksztalcenie_Do_Poczatku().p.x(),
			   OdlegloscOdSrodka, 
			   2*Promien()
			    +_Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa()
			    +_Palce->at(1).Przeksztalcenie_Do_Poczatku().p.z()
			    +_Palce->at(1).SredniaZPodstaw(_Palce->at(0).Liczba_Paliczkow()-1)
			    +5);
   			   
  // Wyznaczenie przeksztalcenia odwrotnego, tzn z ukladu kciuka do ukladu bazowego, tak, aby
  // wyznaczony w ukladzie bazowym wektor punktu chwytu przeksztalcic do ukladu kciuka 
  PrzekszOdwrotne = Frame(_Palce->at(0).Przeksztalcenie_Do_Poczatku().M.Inverse(),
			  _Palce->at(0).Przeksztalcenie_Do_Poczatku().M.Inverse()*(-_Palce->at(0).Przeksztalcenie_Do_Poczatku().p));
  (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);
  

  // KCIUK 2: Wyznaczenie rotacji i wektora kciuka w polozeniu 1
  Konfiguracja(0)=atan((*Temp_Vector).y()/(*Temp_Vector).x());
  Konfiguracja(1)=0;
  Konfiguracja(2)=0;
  Konfiguracja(3)=0;
  _Palce->at(0).Fk()->JntToCart(Konfiguracja,F);
  Punkty.push_back(F);

  Kat=60;
  Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
  (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
  (*Temp_Rotation).DoRotZ(Kat*PI/180);
     
  while(_Palce->at(0).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
  {
    Kat-=3;
    delete Temp_Rotation;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
    (*Temp_Rotation).DoRotZ(Kat*PI/180); 
    if(Kat<0)
    { 
      std::cerr << "Kciuk jest zbyt krotki i nie da sie chwycic tego obiektu ta reka" << std::endl; 
      exit(1); 
    }
  }
  // Ustawienie wynikowej konfiguracji jako bazowej
  KonfiguracjaPrzegubow=KonfiguracjaWynikowa;
  
  Punkty.push_back(Frame(*Temp_Rotation, *Temp_Vector)); 
  // usuniecie pamieci tymczasowej
  delete Temp_Vector;
  delete Temp_Rotation;
  
  // KCIUK 3: Wyznaczenie orientacji i wektora kciuka w polozeniu 2
  // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
  Temp_Vector = new Vector(_Palce->at(1).Przeksztalcenie_Do_Poczatku().p.x(),
			   OdlegloscOdSrodka, 
			   2*Promien()
			    +_Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa()
			    +_Palce->at(1).Przeksztalcenie_Do_Poczatku().p.z()
			    +_Palce->at(1).SredniaZPodstaw(_Palce->at(0).Liczba_Paliczkow()-1));
  
  // Przeksztalcenie wektora do ukladu kciuka
  (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);
  
  Kat=60;
  Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
  (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
  (*Temp_Rotation).DoRotZ(Kat*PI/180);
  
  while(_Palce->at(0).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
  {
    Kat-=3;
    delete Temp_Rotation;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
    (*Temp_Rotation).DoRotZ(Kat*PI/180); 
    if(Kat<0)
    { 
      std::cerr << "Kciuk jest za krotki i nie da sie chwycic tego obiektu ta reka" << std::endl; 
      exit(1); 
    }
  }
  Punkty.push_back(Frame(*Temp_Rotation, *Temp_Vector)); 
  // KONIEC KCIUKA
// cout << KonfiguracjaWynikowa;
  // usuniecie pamieci tymczasowej
  delete Temp_Vector;
  delete Temp_Rotation;
  
  
    // Wyznaczenie punktow chwytu dla kolejnych palcow
  for(unsigned int i=1;i<_Palce->size();i++)
  {
    // Ustawienie konfiguracji palcow nie bioracych udzialu w chwycie
    if(i>1)
    {
    KonfiguracjaPrzegubow(0)=0;
    KonfiguracjaPrzegubow(1)=10*PI/180;
    KonfiguracjaPrzegubow(2)=45*PI/180;
    KonfiguracjaPrzegubow(3)=30*PI/180;
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
    KonfiguracjaPrzegubow(0)=0;
    KonfiguracjaPrzegubow(1)=20*PI/180;
    KonfiguracjaPrzegubow(2)=60*PI/180;
    KonfiguracjaPrzegubow(3)=30*PI/180;
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
    KonfiguracjaPrzegubow(0)=0;
    KonfiguracjaPrzegubow(1)=30*PI/180;
    KonfiguracjaPrzegubow(2)=90*PI/180;
    KonfiguracjaPrzegubow(3)=45*PI/180;
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
// cout << KonfiguracjaPrzegubow;
    continue;
    }
    
    // PALEC WSKAZUJACY 1: Wyznaczenie orientacji i polozenia efektora palca w polozeniu 1
    KonfiguracjaPrzegubow(0)=0;
    KonfiguracjaPrzegubow(1)=15*PI/180;
    KonfiguracjaPrzegubow(2)=60*PI/180;
    KonfiguracjaPrzegubow(3)=30*PI/180;
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
    // PALEC WSKAZUJACY 2: Wyznaczenie orientacji i polozenia efektora palca w polozeniu 1

    // Wyznaczenie przeksztalcenia odwrotnego, tzn z ukladu kciuka do ukladu bazowego, tak, aby
    // wyznaczony w ukladzie bazowym wektor punktu chwytu przeksztalcic do ukladu kciuka 
    PrzekszOdwrotne = Frame(_Palce->at(1).Przeksztalcenie_Do_Poczatku().M.Inverse(),
			  _Palce->at(1).Przeksztalcenie_Do_Poczatku().M.Inverse()*(-_Palce->at(1).Przeksztalcenie_Do_Poczatku().p));
    
    Kat=179;
    // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
    Temp_Vector = new Vector(_Palce->at(1).Przeksztalcenie_Do_Poczatku().p.x()
			      -((_Palce->at(1).Paliczki().at(_Palce->at(1).Liczba_Paliczkow()-1).
					    Dlugosc()/2)*cos((180-Kat)*PI/180))
			      +5, 
			     OdlegloscOdSrodka
			      +((_Palce->at(1).Paliczki().at(_Palce->at(1).Liczba_Paliczkow()-1).
					    Dlugosc()/2)*sin((180-Kat)*PI/180)),
			     _Palce->at(1).Przeksztalcenie_Do_Poczatku().p.z());

    (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);		     

    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotZ(Kat*PI/180);
    
    
    while(_Palce->at(i).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
    {
      Kat-=6;
      delete Temp_Vector;
      delete Temp_Rotation;
      Temp_Vector = new Vector(_Palce->at(1).Przeksztalcenie_Do_Poczatku().p.x()
			      -((_Palce->at(1).Paliczki().at(_Palce->at(1).Liczba_Paliczkow()-1).
					    Dlugosc()/2)*cos((180-Kat)*PI/180))
			      +5, 
			     OdlegloscOdSrodka
			      +((_Palce->at(1).Paliczki().at(_Palce->at(1).Liczba_Paliczkow()-1).
					    Dlugosc()/2)*sin((180-Kat)*PI/180)),
			     _Palce->at(1).Przeksztalcenie_Do_Poczatku().p.z());
      (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);
	
      Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
      (*Temp_Rotation).DoRotZ(Kat*PI/180); 
      if(Kat<90)
      { 
	std::cerr << "Palec " << i << " nie dał rady chwycic tego obiektu" << std::endl; 
	exit(1); 
      }
    }
    // Ustawienie wynikowej konfiguracji jako bazowej
    KonfiguracjaPrzegubow=KonfiguracjaWynikowa;
    
    Punkty.push_back(Frame(*Temp_Rotation, *Temp_Vector)); 
    // usuniecie pamieci tymczasowej
    delete Temp_Vector;
    delete Temp_Rotation;
  
   // PALEC WSKAZUJACY 3: Wyznaczenie orientacji i polozenia efektora palca w polozeniu 2
   // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
    Kat=179;
    // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
    Temp_Vector = new Vector(_Palce->at(1).Przeksztalcenie_Do_Poczatku().p.x()
			      -((_Palce->at(1).Paliczki().at(_Palce->at(1).Liczba_Paliczkow()-1).
					    Dlugosc()/2)*cos((180-Kat)*PI/180)), 
			     OdlegloscOdSrodka
			      +((_Palce->at(1).Paliczki().at(_Palce->at(1).Liczba_Paliczkow()-1).
					    Dlugosc()/2)*sin((180-Kat)*PI/180)),
			     _Palce->at(1).Przeksztalcenie_Do_Poczatku().p.z());
    (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);		     

    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotZ(Kat*PI/180);
    
    
    while(_Palce->at(i).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
    {
      Kat-=6;
      delete Temp_Vector;
      delete Temp_Rotation;
      Temp_Vector = new Vector(_Palce->at(1).Przeksztalcenie_Do_Poczatku().p.x()
			      -((_Palce->at(1).Paliczki().at(_Palce->at(1).Liczba_Paliczkow()-1).
					    Dlugosc()/2)*cos((180-Kat)*PI/180)), 
			     OdlegloscOdSrodka
			      +((_Palce->at(1).Paliczki().at(_Palce->at(1).Liczba_Paliczkow()-1).
					    Dlugosc()/2)*sin((180-Kat)*PI/180)),
			     _Palce->at(1).Przeksztalcenie_Do_Poczatku().p.z());
      (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);	
      Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
      (*Temp_Rotation).DoRotZ(Kat*PI/180); 
      if(Kat<90)
      { 
	std::cerr << "Palec " << i << " niestety nie dał rady chwycic tego obiektu" << std::endl; 
	exit(1); 
      }
    }
    Punkty.push_back(Frame(*Temp_Rotation, *Temp_Vector)); 
    // KONIEC PALCOW
// cout << KonfiguracjaWynikowa;
    // usuniecie pamieci tymczasowej
    delete Temp_Vector;
    delete Temp_Rotation;
  }
  
  return Punkty;
}

/*
 * Chwytanie obiektu Sferycznego
 *
 * Chwyt obiektu, ktorego jedynym parametrem jest promien (kula)
 * W przypadku tego chwytu obracamy wszystkie palce poza kciukiem i palcem srodkowym o staly kat 12 stopni
 * Nastepnie wyznaczamy promien kuli w plaszczyznie zgiecia palca, a na tej podstawie
 * szukajac stycznych do kola opisanego przez ten "NowyPromien" wyznaczamy katy przegubow tak,
 * aby kazdy z paliczkow bezposrednio dotykal obiektu. Do tego kata dodajemy takze kat
 * wynikajacy z roznicy w dlugosciach podstaw. 
 * Chwyt kciuka odbywa sie w zadanym punkcie, ktory znajduje sie na wysokosci (Z w bazowym) ulozenia palca
 * wskazujacego. Znajac ta wysokosc (ZDlaKciuka), wyznaczamy promien kola w tym miejscu i szukajac odpowiedniego
 * kata ustawiamy kciuk. 
 *
 * Wyznaczone punkty chwytu w postaci kinematyki (orientacja + wektor) zostaja zapisane do wektora Punkty i zwrocone jako wynik
 * metody. W tym przypadku wyznaczamy po 3 punkty dla kazdego z palcow, przy czym punkt 1 jest niezmiennym, poczatkowym
 * punktem rozpoczynajacym chwyt.
 */
std::vector<KDL::Frame> Sferyczny::WyznaczPunktyChwytu(std::vector<Palec>* _Palce)
{
  std::vector<KDL::Frame> Punkty;
  std::vector<KDL::Frame>::iterator it;
  Frame F;
  Frame PrzekszOdwrotne;
  Vector *Temp_Vector;
  Vector O, P1;
  Rotation *Temp_Rotation;
  JntArray Konfiguracja = JntArray(4);
  JntArray KonfiguracjaPrzegubow = JntArray(4);
  JntArray KonfiguracjaWynikowa = JntArray(4);
  double Dlugosc=0, NowyPromien, OdlegloscOdSrodkaWOsiZ, x, zm, ZDlaKciuka, PromienDlaKciuka;
  double OdlegloscOdSrednicy, OdlegloscOdSrodka;
  double Kat, Alfa, Beta, Gamma;
  
  for(unsigned int i=0;i<_Palce->at(0).Liczba_Paliczkow();i++)
    Dlugosc+=_Palce->at(0).Paliczki().at(i).Dlugosc();
   if((Promien()>(Dlugosc/2)) || (Promien()<20)) // jesli palec wskazujacy jest krotszy niz 1/8 obwodu
     { std::cerr << "Nie da sie chwycic tego obiektu ta reka" << std::endl; exit(1); }
     
   // Tutaj wyjatkowo zrobimy najpierw palce
 // Wyznaczenie punktow chwytu dla kolejnych palcow
  for(unsigned int i=1;i<_Palce->size();i++)
  {
    
    // PALCE 1: Wyznaczenie konfiguracji w polozeniu 1
    // zakladamy kat +- 20 stopni w pierwszym przegubie w palcach poza srodkowym
    if(_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.z()>0)
      KonfiguracjaPrzegubow(0)=-12*PI/180;
    else if(_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.z()<0)
      KonfiguracjaPrzegubow(0)=12*PI/180;
    else
      KonfiguracjaPrzegubow(0)=0*PI/180; 
    KonfiguracjaPrzegubow(1)=0*PI/180;
    KonfiguracjaPrzegubow(2)=0*PI/180;
    KonfiguracjaPrzegubow(3)=0*PI/180;
    
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
    
    // PALCE 2: Wyznaczenie konfiguracji w polozeniu 2
    // Obliczenie promienia kola kuli w plaszczyznie zgiecia palca
    x = -_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.x()*tan(12*PI/180);
    Kat = atan(x/(Promien() + _Palce->at(2).Paliczki().at(0).Dolna_Podstawa()));
    OdlegloscOdSrednicy = fabs(_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.z()*cos(Kat));


    // jesli palec nie da rady nie bierze udzialu w chwycie
    if(OdlegloscOdSrednicy > Promien())
    {
    KonfiguracjaPrzegubow(1)=10*PI/180;
    KonfiguracjaPrzegubow(2)=30*PI/180;
    KonfiguracjaPrzegubow(3)=15*PI/180;
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
    KonfiguracjaPrzegubow(1)=20*PI/180;
    KonfiguracjaPrzegubow(2)=60*PI/180;
    KonfiguracjaPrzegubow(3)=30*PI/180;
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
cout << KonfiguracjaPrzegubow;
    continue;
    }
    if(i==1)
    {
      ZDlaKciuka=OdlegloscOdSrednicy*cos(Kat); // Kciuk ma znalezc sie w takiej samej odleglosci Z(bazowy) co palec wsk.
    cout << "PALEC " << i << " " << ZDlaKciuka << endl;
      zm = tan(Kat) *(OdlegloscOdSrednicy*sin(Kat));
      PromienDlaKciuka=PitagorasPrzyprostokatna(Promien(), (-_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.z())-zm);
    }

    NowyPromien = PitagorasPrzyprostokatna(Promien(), OdlegloscOdSrednicy);
    
    // Liczymy katy na podstawie promienia
    // KAT 1:
    if(NowyPromien==Promien())
    {
      OdlegloscOdSrodkaWOsiZ=Promien();
      O = Vector(-_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.x(), 0, OdlegloscOdSrodkaWOsiZ); // srodek okregu
      Kat=0;
    }
    else
    {
      OdlegloscOdSrodkaWOsiZ = (fabs(_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.z())*sin(Kat)) + x/sin(Kat);
      O = Vector(-_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.x() , 0, OdlegloscOdSrodkaWOsiZ); // srodek okregu
      P1 = Vector(_Palce->at(i).Przeksztalcenie_Do_Poczatku().p.x(), 0, 0); // punkt w ktorym konczy sie pierwszy paliczek
      OdlegloscOdSrodka = OdlegloscDwochPunktow(P1, O); //cout << "OdlOdSrd: " << OdlegloscOdSrodka << endl;

//       Kat = atan((OdlegloscOdSrodka-NowyPromien)/PitagorasPrzyprostokatna(OdlegloscOdSrodka, NowyPromien));
  //cout << endl << "P1: " << P1 << endl << endl;

      Alfa = atan2(P1.x()-O.x(), O.z()-P1.z()); //cout << "Alfa: " << Alfa << endl;
      Beta = acos((NowyPromien + _Palce->at(i).SredniaZPodstaw(0))/OdlegloscOdSrodka); //cout << "Beta: " << Beta << endl;
      Gamma = Alfa + Beta - PI/2; //cout << "Gamma: " << Gamma << endl;
      Kat = PI - (PI/2 - Gamma);
    }
    Kat+=_Palce->at(i).KatRoznicyPodstawPaliczka(0);
    if(Kat > _Palce->at(i).Max_Przegubow()(1))
      KonfiguracjaPrzegubow(1)=_Palce->at(i).Max_Przegubow()(1);
    else if(Kat < _Palce->at(i).Min_Przegubow()(1))
      KonfiguracjaPrzegubow(1)=_Palce->at(i).Min_Przegubow()(1);
    else
	KonfiguracjaPrzegubow(1)=Kat;
    // KAT 2:
    // Wartosc promienia powiekszona o grubosc paliczka
      // punkt w ktorym konczy sie pierwszy paliczek
    P1 = Vector(_Palce->at(i).Paliczki().at(0).Dlugosc()*cos(KonfiguracjaPrzegubow(1)),
		0, 
		_Palce->at(i).Paliczki().at(0).Dlugosc()*sin(KonfiguracjaPrzegubow(1)));
    OdlegloscOdSrodka = OdlegloscDwochPunktow(P1, O); //cout << "OdlOdSrd: " << OdlegloscOdSrodka << endl;

    Alfa = atan2(P1.x()-O.x(), O.z()-P1.z()); //cout << "Alfa: " << Alfa << endl;
    Beta = acos((NowyPromien + _Palce->at(i).SredniaZPodstaw(1))/OdlegloscOdSrodka); //cout << "Beta: " << Beta << endl;
    Gamma = Alfa + Beta - PI/2; //cout << "Gamma: " << Gamma << endl;
    Kat = PI - (PI/2 - Gamma) - KonfiguracjaPrzegubow(1);
      
    Kat+=_Palce->at(i).KatRoznicyPodstawPaliczka(1);
    if(Kat > _Palce->at(i).Max_Przegubow()(2))
      KonfiguracjaPrzegubow(2)=_Palce->at(i).Max_Przegubow()(2);
    else if(Kat < _Palce->at(i).Min_Przegubow()(2))
      KonfiguracjaPrzegubow(2)=_Palce->at(i).Min_Przegubow()(2);
    else
      KonfiguracjaPrzegubow(2)=Kat;				      

    // KAT 3:
    // Wartosc promienia powiekszona o grubosc paliczka
    // Wyznaczenie odleglosci dwoch punktow: srodka kola w plaszczyznie ruchu oraz punktu, w ktorym
    // konczy sie drugi paliczek
    P1 = Vector(_Palce->at(i).Paliczki().at(0).Dlugosc()*cos(KonfiguracjaPrzegubow(1))
		  + _Palce->at(i).Paliczki().at(1).Dlugosc()*cos(KonfiguracjaPrzegubow(2)+KonfiguracjaPrzegubow(1)),
		0, 
		_Palce->at(i).Paliczki().at(0).Dlugosc()*sin(KonfiguracjaPrzegubow(1)) 
		  + _Palce->at(i).Paliczki().at(1).Dlugosc()*sin(KonfiguracjaPrzegubow(2)+KonfiguracjaPrzegubow(1)));
    OdlegloscOdSrodka = OdlegloscDwochPunktow(P1, O); //cout << "OdlOdSrd: " << OdlegloscOdSrodka << endl;

    Alfa = atan2(P1.x()-O.x(), O.z()-P1.z()); //cout << "Alfa: " << Alfa << endl;
    Beta = acos((NowyPromien + _Palce->at(i).SredniaZPodstaw(1))/OdlegloscOdSrodka); //cout << "Beta: " << Beta << endl;
    Gamma = Alfa + Beta - PI/2; //cout << "Gamma: " << Gamma << endl;
    Kat = PI - (PI/2 - Gamma) - KonfiguracjaPrzegubow(1) - KonfiguracjaPrzegubow(2);
      
    Kat+=_Palce->at(i).KatRoznicyPodstawPaliczka(2);
    if(Kat > _Palce->at(i).Max_Przegubow()(3))
      KonfiguracjaPrzegubow(3)=_Palce->at(i).Max_Przegubow()(3);
    else if(Kat < _Palce->at(i).Min_Przegubow()(3))
      KonfiguracjaPrzegubow(3)=_Palce->at(i).Min_Przegubow()(3);
    else
      KonfiguracjaPrzegubow(3)=Kat;

    // Ustawienie konfiguracji posredniej rozniacej sie jedynie dwoma ostatnimi przegubami
    // efektor bedzie ustawiony w poblizu punktu docelowego
    Konfiguracja(0) = KonfiguracjaPrzegubow(0);
    Konfiguracja(1) = KonfiguracjaPrzegubow(1);
    Konfiguracja(2) = KonfiguracjaPrzegubow(2)-5*PI/180;
    Konfiguracja(3) = KonfiguracjaPrzegubow(3)-10*PI/180;
    _Palce->at(i).Fk()->JntToCart(Konfiguracja,F);
    Punkty.push_back(F);
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
cout << KonfiguracjaPrzegubow;
  } 

  
  // KCIUK 1: Wyznaczenie rotacji i wektora kciuka w polozeniu 1
  NowyPromien=PromienDlaKciuka;
  // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
  Temp_Vector = new Vector(-NowyPromien/2
			      -_Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa()*sin(45*PI/180)
			      -5, 
			   PitagorasPrzyprostokatna(NowyPromien, NowyPromien/2)
			      +Promien()
			      +_Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa()*cos(45*PI/180)
			      +_Palce->at(2).Paliczki().at(0).Dolna_Podstawa()
			      +5, 
			   ZDlaKciuka);

  // Wyznaczenie przeksztalcenia odwrotnego, tzn z ukladu kciuka do ukladu bazowego, tak, aby
  // wyznaczony w ukladzie bazowym wektor punktu chwytu przeksztalcic do ukladu kciuka 
  PrzekszOdwrotne = Frame(_Palce->at(0).Przeksztalcenie_Do_Poczatku().M.Inverse(),
			  _Palce->at(0).Przeksztalcenie_Do_Poczatku().M.Inverse()*(-_Palce->at(0).Przeksztalcenie_Do_Poczatku().p));
  (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);
  KonfiguracjaPrzegubow(0)=atan((*Temp_Vector).y()/(*Temp_Vector).x());
  KonfiguracjaPrzegubow(1)=0*PI/180;
  KonfiguracjaPrzegubow(2)=0*PI/180;
  KonfiguracjaPrzegubow(3)=0*PI/180;
  _Palce->at(0).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
  it=Punkty.begin();
  Punkty.insert(it, F);  
  
  // KCIUK 2: Wyznaczenie rotacji i wektora kciuka w polozeniu 2
  // Wyznaczenie odleglosci od reki na podstawie zadanej konfiguracji bazowej poczatkowej palca wskazkujacego
  
  
  Kat=180;
  Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
  (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
  (*Temp_Rotation).DoRotZ(Kat*PI/180);
     
  while(_Palce->at(0).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
  {
    Kat-=6;
    delete Temp_Rotation;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
    (*Temp_Rotation).DoRotZ(Kat); 
    if(Kat<0)
    { 
      std::cerr << "Kciuk jest zbyt krotki i nie da sie chwycic tego obiektu ta reka" << std::endl; 
      exit(1); 
    }
  }
  // Ustawienie wynikowej konfiguracji jako bazowej
  KonfiguracjaPrzegubow=KonfiguracjaWynikowa;
  it=Punkty.begin();
  Punkty.insert(it+1, Frame(*Temp_Rotation, *Temp_Vector)); 
  // usuniecie pamieci tymczasowej
  delete Temp_Vector;
  delete Temp_Rotation;
  
  // KCIUK 3: Wyznaczenie orientacji i wektora kciuka w polozeniu 3
  // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
  Temp_Vector = new Vector(-NowyPromien/2
			      -_Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa()*sin(45*PI/180),
			   PitagorasPrzyprostokatna(NowyPromien, NowyPromien/2)
			      +Promien()
			      +_Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa()*cos(45*PI/180)
			      +_Palce->at(2).Paliczki().at(0).Dolna_Podstawa(),
			   ZDlaKciuka);
  
  (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);
  
  Kat=180;
  Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
  (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
  (*Temp_Rotation).DoRotZ(Kat*PI/180);
  
  while(_Palce->at(0).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
  {
    Kat-=6;
    delete Temp_Rotation;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
    (*Temp_Rotation).DoRotZ(Kat*PI/180); 
    if(Kat<0)
    { 
      std::cerr << "Kciuk jest za krotki i nie da sie chwycic tego obiektu ta reka" << std::endl; 
      exit(1); 
    }
  }
  it=Punkty.begin();
  Punkty.insert(it+2, Frame(*Temp_Rotation, *Temp_Vector)); 
  // KONIEC KCIUKA
cout << KonfiguracjaWynikowa;
  return Punkty;
}

/*
 * Chwyt Lateralny
 *
 * Chwyt obiektu, ktorego jedynym parametrem jest grubosc
 * Zakladamy, ze zbedna jest nam wiedza o szerokosci i dlugosci obiektu, 
 * skupiamy sie jedynie na chwycie w okreslonym punkcie
 * Palce (oprocz kciuka) ustawiamy w stalym polozeniu, przydzielajac z gory zalozone katy w przegubach, kolejno:
 * 0, 90, 80 i 60 stopni
 * Polozenie kciuka dostosowujemy na podstawie dlugosci pierwszego paliczka palca wskazujacego - na wysokosci jego zakonczenia 
 * nastapi docisk kciuka i chwyt obiektu. Odleglosc punktu chwytu od tego paliczka zalezy od grubosci obiektu.
 *
 * Wyznaczone punkty chwytu w postaci kinematyki (orientacja + wektor) zostaja zapisane do wektora Punkty i zwrocone jako wynik
 * metody. W tym przypadku wyznaczamy po 3 punkty dla kazdego z palcow, przy czym punkt 1 jest niezmiennym, poczatkowym
 * punktem rozpoczynajacym chwyt.
 */
std::vector<KDL::Frame> Lateralny::WyznaczPunktyChwytu(std::vector<Palec>* _Palce)
{
  std::vector<KDL::Frame> Punkty;
  Frame F;
  Frame PrzekszOdwrotne;
  Vector *Temp_Vector;
  Rotation *Temp_Rotation;
  JntArray Konfiguracja = JntArray(4);
  JntArray KonfiguracjaPrzegubow = JntArray(4);
  JntArray KonfiguracjaWynikowa = JntArray(4);
  double DlugoscPalca=0;
  int Kat;
  
//   Kciuk
  for(unsigned int i=0;i<_Palce->at(0).Liczba_Paliczkow();i++)
    DlugoscPalca+=_Palce->at(0).Paliczki().at(i).Dlugosc();
   if(Grubosc()>DlugoscPalca/2)
     { std::cerr << "Nawet nie ma co probowac chwycic tego za grubego obiektu" << std::endl; exit(1); }

  // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
  Temp_Vector = new Vector(_Palce->at(1).Przeksztalcenie_Do_Poczatku().p.x(),
			   _Palce->at(1).Paliczki().at(0).Dlugosc(), 
			   _Palce->at(1).Przeksztalcenie_Do_Poczatku().p.z()
			    +_Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa()
			    +_Palce->at(1).Paliczki().at(0).Gorna_Podstawa()
			    +Grubosc()
			    +5);
			   
  // Wyznaczenie przeksztalcenia odwrotnego, tzn z ukladu kciuka do ukladu bazowego, tak, aby
  // wyznaczony w ukladzie bazowym wektor punktu chwytu przeksztalcic do ukladu kciuka 
  PrzekszOdwrotne = Frame(_Palce->at(0).Przeksztalcenie_Do_Poczatku().M.Inverse(),
			  _Palce->at(0).Przeksztalcenie_Do_Poczatku().M.Inverse()*(-_Palce->at(0).Przeksztalcenie_Do_Poczatku().p));
  (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);


  // KCIUK 1: Wyznaczenie orientacji i polozenia efektora palca w polozeniu 1
  KonfiguracjaPrzegubow(0)=atan((*Temp_Vector).y()/(*Temp_Vector).x());
  KonfiguracjaPrzegubow(1)=0;
  KonfiguracjaPrzegubow(2)=0;
  KonfiguracjaPrzegubow(3)=0;
  _Palce->at(0).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
  Punkty.push_back(F);

  // KCIUK 2: Wyznaczenie rotacji i wektora kciuka w polozeniu 1
  // Wyznaczenie odleglosci od reki na podstawie zadanej konfiguracji bazowej poczatkowej palca wskazkujacego  
  
  
  Kat=60;
  Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
  (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
  (*Temp_Rotation).DoRotZ(Kat*PI/180);
     
  while(_Palce->at(0).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
  {
    Kat-=3;
    delete Temp_Rotation;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
    (*Temp_Rotation).DoRotZ(Kat*PI/180); 
    if(Kat<0)
    { 
      std::cerr << "Kciuk jest zbyt krotki i nie da sie chwycic tego obiektu ta reka" << std::endl; 
      exit(1); 
    }
  }
  // Ustawienie wynikowej konfiguracji jako bazowej
  KonfiguracjaPrzegubow=KonfiguracjaWynikowa;
  
  Punkty.push_back(Frame(*Temp_Rotation, *Temp_Vector)); 
  // usuniecie pamieci tymczasowej
  delete Temp_Vector;
  delete Temp_Rotation;
  
  // KCIUK 3: Wyznaczenie orientacji i wektora kciuka w polozeniu 2
  // Wyznaczenie polozenia punktu chwytu w ukladzie bazowym
  Temp_Vector = new Vector(_Palce->at(1).Przeksztalcenie_Do_Poczatku().p.x(),
			   _Palce->at(1).Paliczki().at(0).Dlugosc(), 
			   _Palce->at(1).Przeksztalcenie_Do_Poczatku().p.z()
			    +_Palce->at(0).Paliczki().at(_Palce->at(0).Liczba_Paliczkow()-1).Gorna_Podstawa()
			    +_Palce->at(1).Paliczki().at(0).Gorna_Podstawa()
			    +Grubosc());
  
  (*Temp_Vector)=PrzekszOdwrotne*(*Temp_Vector);
  
  Kat=60;
  Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
  (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
  (*Temp_Rotation).DoRotZ(Kat);
  
  while(_Palce->at(0).Ik()->CartToJnt(KonfiguracjaPrzegubow,Frame(*Temp_Rotation, *Temp_Vector),KonfiguracjaWynikowa)<0)
  {
    Kat-=3;
    delete Temp_Rotation;
    Temp_Rotation = new Rotation(Rotation::EulerZYX(0, 0, 90*PI/180)); // w stopniach
    (*Temp_Rotation).DoRotY(atan((*Temp_Vector).y()/(*Temp_Vector).x()));
    (*Temp_Rotation).DoRotZ(Kat); 
    if(Kat<0)
    { 
      std::cerr << "Kciuk jest za krotki i nie da sie chwycic tego obiektu ta reka" << std::endl; 
      exit(1); 
    }
  }
//     std::cout << KonfiguracjaWynikowa(0)*180/PI << " " << KonfiguracjaWynikowa(1)*180/PI << " " <<
//     KonfiguracjaWynikowa(2)*180/PI << " " << KonfiguracjaWynikowa(3)*180/PI << " " << std::endl;
  Punkty.push_back(Frame(*Temp_Rotation, *Temp_Vector)); 
  // KONIEC KCIUKA
// cout << KonfiguracjaWynikowa;
  // usuniecie pamieci tymczasowej
  delete Temp_Vector;
  delete Temp_Rotation;
  
  
    // Wyznaczenie punktow chwytu dla kolejnych palcow
  for(unsigned int i=1;i<_Palce->size();i++)
  {
    // Ustawienie stalej konfiguracji pozostalych palcow
    // PALCE 1: Wyznaczenie orientacji i polozenia efektora palca w polozeniu 1
    KonfiguracjaPrzegubow(0)=0;
    KonfiguracjaPrzegubow(1)=15*PI/180;
    KonfiguracjaPrzegubow(2)=45*PI/180;
    KonfiguracjaPrzegubow(3)=30*PI/180;
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
    // PALCE 2: Wyznaczenie orientacji i polozenia efektora palca w polozeniu 1
    KonfiguracjaPrzegubow(0)=0;
    KonfiguracjaPrzegubow(1)=30*PI/180;
    KonfiguracjaPrzegubow(2)=60*PI/180;
    KonfiguracjaPrzegubow(3)=30*PI/180;
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
   // PALCE 3: Wyznaczenie orientacji i polozenia efektora palca w polozeniu 2
    KonfiguracjaPrzegubow(0)=0;
    KonfiguracjaPrzegubow(1)=90*PI/180;
    KonfiguracjaPrzegubow(2)=60*PI/180;
    KonfiguracjaPrzegubow(3)=0*PI/180;
    _Palce->at(i).Fk()->JntToCart(KonfiguracjaPrzegubow,F);
    Punkty.push_back(F);
//     std::cout << KonfiguracjaPrzegubow(0)*180/PI << " " << KonfiguracjaPrzegubow(1)*180/PI << " " <<
//     KonfiguracjaPrzegubow(2)*180/PI << " " << KonfiguracjaPrzegubow(3)*180/PI << " " << std::endl;
// cout << KonfiguracjaPrzegubow;
  }
  return Punkty;
}