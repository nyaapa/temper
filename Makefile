all:	pcsensor

pcsensor:	pcsensor.cpp
	clang++  -O3 -Wall -W -Werror --std=c++1z -DNDEBUG -o $@ $^ -lusb

clean:		
	rm -f pcsensor *.o

rules-install:
	sudo cp 99-tempsensor.rules /etc/udev/rules.d
