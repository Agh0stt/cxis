CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -std=c11 -Iinclude
TARGETS = cxas cxld cxvm cxdis

all: $(TARGETS)

cxas: cxas.c include/cxis.h include/cxo.h
	$(CC) $(CFLAGS) -o $@ $<

cxld: cxld.c include/cxis.h include/cxo.h include/cxe.h
	$(CC) $(CFLAGS) -o $@ $<

cxvm: cxvm.c include/cxis.h include/cxe.h
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGETS) *.cxo *.cxe

test: all tests/hello.cxis
	./cxas tests/hello.cxis -o tests/hello.cxo
	./cxld tests/hello.cxo  -o tests/hello.cxe
	./cxvm tests/hello.cxe

test-suite: all
	python3 tests/run_suite.py

test77: test-suite

.PHONY: all clean test test-suite test77
cxdis: cxdis.c include/cxis.h include/cxe.h include/cxo.h
	$(CC) $(CFLAGS) -o $@ $<
