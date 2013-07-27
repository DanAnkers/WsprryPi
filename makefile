prefix=/usr/local

all: wspr gpioclk

wspr: wspr.cpp
	g++ -Wall -lm wspr.cpp -owspr

gpioclk: gpioclk.cpp
	g++ -Wall -lm gpioclk.cpp -ogpioclk

.PHONY: install
install: wspr
	install -m 0755 wspr $(prefix)/bin
	install -m 0755 gpioclk $(prefix)/bin

.PHONY: uninstall
uninstall:
	rm -f $(prefix)/bin/wspr
	rm -f $(prefix)/bin/gpioclk

