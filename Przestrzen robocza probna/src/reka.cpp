#include "reka.h"

using namespace KDL;
using namespace std;

/*
 * Metoda zwracajaca dlugosc drugiej przyprostokatnej trojkata
 * korzystajac z twierdzenia pitagorasa.
 *
 */
double PitagorasPrzyprostokatna(double Z1, double Z2)
{ return sqrt(fabs(pow(Z1, 2)-pow(Z2,2))); }

/*
 * Metoda zwracajaca dlugosc przeciwprostokatnej trojkata
 * korzystajac z twierdzenia pitagorasa.
 *
 */
double PitagorasPrzeciwprostokatna(double Z1, double Z2)
{ return sqrt(pow(Z1, 2)+pow(Z2,2)); }

/*
 * Metoda zwracajaca odleglosc dwoch zadanych punktow w przestrzeni
 *
 */
double OdlegloscDwochPunktow(KDL::Vector V1, KDL::Vector V2)
  { return sqrt(pow(V1.x()-V2.x(), 2)+pow(V1.y()-V2.y(), 2)+pow(V1.z()-V2.z(), 2)); }

/*
 * Metoda umozliwiajaca pominiecie komentarzy
 * w pliku, do ktorego uchwyt zostal przekazany jako argument
 */
void Reka::Pomin_Komentarze(std::istream & Plik)
{
	char Komentarz[1024];
	while(Plik.peek()=='\n') // pobranie znaku nowej linii
	  Plik.get();
  	while(Plik.peek()=='#')
	  Plik.getline(Komentarz, 1024);
}

/*
 * Metoda wczytuje dane o palcach z pliku,
 * tworzy obiekty klasy Palec i dodaje
 * je do wektora palcow rekid
 */
bool Reka::Wczytaj_Dane(std::string Nazwa)
{
	std::ifstream Plik(Nazwa.c_str());
	double Tab[3];
	Vector *Temp_Vector;
	Rotation *Temp_Rotation;
	Palec * Temp_Palec;
	Paliczek * Temp_Paliczek;

	if (!Plik.is_open())
	{
	  std::cout << "Blad: Nie podano uchwytu do pliku z danymi reki." << std::endl << std::endl;
	  return false;
	}
	Pomin_Komentarze(Plik);	// pominiecie komentarzy
	Plik >> _Liczba_Palcow;
	Pomin_Komentarze(Plik);
	
	// Wczytanie palcow
	for(unsigned int i=0;i<Liczba_Palcow();i++)
	{
	  // wczytanie wektora przesuniecia
	  Plik >> Tab[0]; 
	  Plik >> Tab[1];
	  Plik >> Tab[2];
	  

	  Temp_Vector = new Vector(Tab[0],Tab[1],Tab[2]);	
	  Pomin_Komentarze(Plik);
	  
	  // wczytanie macierzy rotacji
	  Plik >> Tab[0];
	  Plik >> Tab[1];
	  Plik >> Tab[2];
	  Temp_Rotation = new Rotation(Rotation::EulerZYX(Tab[0]*PI/180,0,0)); // w stopniach
	  Temp_Rotation->DoRotY(Tab[1]*PI/180);
	  Temp_Rotation->DoRotX(Tab[2]*PI/180);
	  // Temp_Rotation = new Rotation(Rotation::EulerZYX(Tab[0],Tab[1],Tab[2])); // w radianach
	  Pomin_Komentarze(Plik);
	  
	  // wczytanie liczby paliczkow
	  Plik >> Tab[0];
	  
	  // stworzenie palca
	  Temp_Palec = new Palec(Tab[0], Tab[0]+1);
	  Pomin_Komentarze(Plik);
	  
	  // wczytanie paliczkow palca
	  for(unsigned int j=0; j<Temp_Palec->Liczba_Paliczkow(); j++)
	  {
	    Plik >> Tab[0];
	    Plik >> Tab[1];
	    Plik >> Tab[2];
	    Temp_Paliczek = new Paliczek(Tab[0],Tab[1],Tab[2]);
	    Temp_Palec->DodajPaliczek(*Temp_Paliczek);
	    delete Temp_Paliczek;
	    Pomin_Komentarze(Plik);
	  }
	  
	  // wczytanie parametrow przegubow palca
	  for(unsigned int j=0; j<Temp_Palec->Liczba_Przegubow(); j++)
	  {
	    Plik >> Tab[0];
	    Plik >> Tab[1];
	    Plik >> Tab[2];
	    Temp_Palec->DodajPrzegub(Tab[0]*PI/180,Tab[1]*PI/180, Tab[2]*PI/180, j);
	    Pomin_Komentarze(Plik);
	  }

	  // stworzenie przeksztalcenia do punktu bazowego palca
	  Temp_Palec->Przeksztalcenie_Do_Poczatku(Frame(*Temp_Rotation, *Temp_Vector));
	  delete Temp_Vector;
	  delete Temp_Rotation;

	  DodajPalec(*Temp_Palec);
	  delete Temp_Palec;
	  
	}

	return true;
}


/*
 * Metoda wczytuje informacje o obiekcie
 * ktory ma byc chwytany
 */
bool Reka::Wczytaj_Obiekt(std::string Nazwa)
{
	std::ifstream Plik(Nazwa.c_str());
	double Tab[3];
	unsigned int Typ;
	Vector *Temp_Vector;
	Rotation *Temp_Rotation;

	if (!Plik.is_open())
	{
	  std::cout << "Blad: Nie podano uchwytu do pliku z danymi obiektu." << std::endl << std::endl;
	  return false;
	}
	Pomin_Komentarze(Plik);	// pominiecie komentarzy
	Plik >> Typ;
	Pomin_Komentarze(Plik);
	
	// Wczytanie wspolrzednych srodka obiektu
	Plik >> Tab[0]; 
	Plik >> Tab[1];
	Plik >> Tab[2];
	
	Temp_Vector = new Vector(Tab[0],Tab[1],Tab[2]);	
	Pomin_Komentarze(Plik);
	  
	// wczytanie macierzy rotacji
	Plik >> Tab[0];
	Plik >> Tab[1];
	Plik >> Tab[2];
	Temp_Rotation = new Rotation(Rotation::EulerZYX(Tab[0]*PI/180,Tab[1]*PI/180,Tab[2]*PI/180)); // w stopniach
	// Temp_Rotation = new Rotation(Rotation::EulerZYX(Tab[0],Tab[1],Tab[2])); // w radianach
	Pomin_Komentarze(Plik);
	switch (Typ)
	{
	  case 1:	Plik >> Tab[0];
			Plik >> Tab[1];
			Plik >> Tab[2];
			_Obiekt = new Szczypcowy(Frame(*Temp_Rotation, *Temp_Vector), Tab[0], Tab[1], Tab[2]);
			break;
			
	  case 2:	Plik >> Tab[0];
			Plik >> Tab[1];
			Plik >> Tab[2];
			_Obiekt = new Hakowy(Frame(*Temp_Rotation, *Temp_Vector), Tab[0], Tab[1], Tab[2]);
			break;
			
	  case 3:	Plik >> Tab[0];
			Plik >> Tab[1];
			_Obiekt = new Dloniowy(Frame(*Temp_Rotation, *Temp_Vector), Tab[0], Tab[1]);
			break;

	  case 4:	Plik >> Tab[0];
			_Obiekt = new Sferyczny(Frame(*Temp_Rotation, *Temp_Vector), Tab[0]);
			break;
	  
			
	  case 5:	Plik >> Tab[0];
			_Obiekt = new Lateralny(Frame(*Temp_Rotation, *Temp_Vector), Tab[0]);
			break;
			
	  default:	Plik >> Tab[0];
			Plik >> Tab[1];
			_Obiekt = new Cylindryczny(Frame(*Temp_Rotation, *Temp_Vector), Tab[0], Tab[1]);
			break;
	}
	delete Temp_Vector;
	delete Temp_Rotation;

	return true;
}


/*
 * Metoda DodajPalec
 *
 * Słuzy do dodania palca do wektora palcow reki
 *
 */
void Reka::DodajPalec(Palec & _P)
{
	_Palce.push_back(_P);
}


/*
 * Metoda StworzStruktury
 *
 * Słuzy do stworzenia struktur poszczegolnych
 * palcow laczac je za pomoca przegubow
 *
 */
void Reka::Stworz_Struktury()
{
  for(unsigned int i=0;i<Liczba_Palcow();i++)
    Palce()->at(i).Wyznacz_Strukture();
}

/* 
 * Metoda Planuj
 *
 * Definiuje nazwe pliku wyjsciowego i otwiera go, a ponadto
 * sluzy do wywolywania metody wyznaczania punktow chwytu,
 * a nastepnie metody planowania sciezki
 * dla kazdego z palcow, przekazujac im
 * uchwyt do pliku wyjsciowego
 * na koncu metoda zamyka polaczenie z plikiem
 */

void Reka::Planuj(unsigned int Liczba_Krokow)
{
  std::vector<KDL::Frame> PunktyDlaPalca;
  std::vector<KDL::Frame> WszystkiePunkty = _Obiekt->WyznaczPunktyChwytu(&_Palce);
  // Zmienna zawierajaca liczbe zadanych punktow dla jednego palca
  unsigned int PunktowDlaPalca = WszystkiePunkty.size()/Liczba_Palcow();

  Plik_Wynikowy.open("wynik.dat");
  Plik_Wynikowy << "# Liczba krokow:\n" << Liczba_Krokow << "\n";
  
  
  for(unsigned int i=0;i<Liczba_Palcow();i++)
  {
    cout << "PALEC " << i << endl;
    Plik_Wynikowy << "# Palec nr " << i << "\n";
    // Wyciagniecie punktow dla tego, i-tego, konkretnego palca
    PunktyDlaPalca.clear();
    for(unsigned int j=0;j<PunktowDlaPalca;j++)
      PunktyDlaPalca.push_back(WszystkiePunkty.at(i*PunktowDlaPalca+j));
    
//     Frame F_Docelowe=Frame(Rotation::EulerZYX(0, -PI/2, PI/2), Vector(50, 0, 40)); 
    if(!Palce()->at(i).ZaplanujSciezke(PunktyDlaPalca, Liczba_Krokow, Plik_Wynikowy))
    { std::cerr << "Nie udalo sie zaplanowac sciezki" << std::endl; exit(1); }
    
  }
  Plik_Wynikowy.close();
}

/*
 * Przeciazenie operatora wyjscia dla klasy KDL::JntArray
 *
 * Operator wypisuje w jednej linijce konfiguracje dla
 * kazdego z przegubow, oddzielone spacjami, a nastepnie
 * przenosi wskaznik do nastepnej linii
 */

std::ostream & operator << (std::ostream & StrWyj, KDL::JntArray Konfiguracja)
{
  for(unsigned int i=0;i<Konfiguracja.rows();i++)
    StrWyj << Konfiguracja(i) << " ";
  StrWyj << "\n";
  return StrWyj;
}




void Reka::Test()
{
    // Create joint array
    unsigned int nj = Palce()->at(1).Liczba_Przegubow();
    KDL::JntArray jointpositions = JntArray(nj);
 
    // Assign some values to the joint positions
    for(unsigned int i=0;i<nj;i++){
        double myinput;
        printf ("Enter the position of joint %i: ",i);
        scanf ("%lf",&myinput);
        jointpositions(i)=(double)myinput*PI/180;
    }
    // Create the frame that will contain the results
    KDL::Frame cartpos;   
    
    //Creation of jntarrays:
    JntArray q(nj);
 
    
    //Set destination frame
	  
    Palce()->at(1).Fk()->JntToCart(jointpositions,cartpos);
    std::cout << Palce()->at(1).Przeksztalcenie_Do_Poczatku()*cartpos << std::endl << cartpos << std::endl << std::endl;
 
    jointpositions(1)=0;
    Frame F_dest=cartpos;//Frame(Rotation::EulerZYX(0, -PI/2, PI/2), Vector(0, 0, 90));//Rotation::EulerZYX(PI/2, 0, 0), 
    std::cout << Palce()->at(1).Przeksztalcenie_Do_Poczatku() <<std::endl << std::endl << std::endl << F_dest << std::endl;
    int ret = Palce()->at(1).Ik()->CartToJnt(jointpositions,F_dest,q);
    std::cout << ret <<std::endl;
    if(ret>=0)
      for(unsigned int i=0;i<Palce()->at(1).Liczba_Przegubow();i++)
	  std::cout << 180*q(i)/PI << std::endl;
}

