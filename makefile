prefix=/usr/local

wspr: wspr.cpp
	g++ -Wall -lm wspr.cpp -owspr

.PHONY: install
install: wspr
	install -m 0755 wspr $(prefix)/bin

.PHONY: uninstall
uninstall:
	rm $(prefix)/bin/wspr

