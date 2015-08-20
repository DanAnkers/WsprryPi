prefix=/usr/local

archis = $(if $(findstring $(1),$(shell uname -m)),$(2))
peribase = $(if $(call archis,armv7,dummy-text),0x3F000000,0x20000000)

all: wspr gpioclk

wspr: wspr.cpp
	g++ -Wall -lm -DBCM2708_PERI_BASE=$(peribase) wspr.cpp -owspr

gpioclk: gpioclk.cpp
	g++ -Wall -lm -DBCM2708_PERI_BASE=$(peribase) gpioclk.cpp -ogpioclk

clean:
	rm gpioclk
	rm wspr

.PHONY: install
install: wspr
	install -m 0755 wspr $(prefix)/bin
	install -m 0755 gpioclk $(prefix)/bin

.PHONY: uninstall
uninstall:
	rm -f $(prefix)/bin/wspr
	rm -f $(prefix)/bin/gpioclk

