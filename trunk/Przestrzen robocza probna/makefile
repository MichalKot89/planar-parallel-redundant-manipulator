# plik makefile do kompilacji planowania sciezki robotycznej reki

__start__: obj ./program
	rm -f core.* core; ./program

# Tworzy katalog "obj" gdy go nie ma
# 
obj:
	mkdir -p obj

program: obj/main.o obj/modul.o src/modul.h
# obiekt program jest zależy od powyższych plików
	g++ -o program obj/main.o obj/modul.o -Wall -pedantic -I/usr/local/include/kdl -I/usr/include/eigen2 -L/usr/local/lib -lorocos-kdl
# Do stworzenia tego obiektu używa się wyżej napisanego polecenia
obj/main.o: src/main.cpp src/modul.h
	g++ -c src/main.cpp -o obj/main.o -Wall -pedantic -I/usr/local/include/kdl -I/usr/include/eigen2 -L/usr/local/lib -lorocos-kdl

obj/modul.o: src/modul.cpp src/modul.h
	g++ -c src/modul.cpp -o obj/modul.o -Wall -pedantic -I/usr/local/include/kdl -I/usr/include/eigen2 -L/usr/local/lib -lorocos-kdl

clean:
	rm -f obj/* ./prog core.* core
