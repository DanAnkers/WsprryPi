prefix=/usr/local

archis = $(if $(findstring $(1),$(shell uname -m)),$(2))
pi_version_flag = $(if $(call archis,armv7,dummy-text),-DRPI2,-DRPI1)

all: wspr gpioclk

wspr: wspr.cpp
	g++ -Wall -lm $(pi_version_flag) mailbox.c wspr.cpp -owspr

gpioclk: gpioclk.cpp
	g++ -Wall -lm $(pi_version_flag) gpioclk.cpp -ogpioclk

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

