
.PHONY: all
all:	test

test:	test.cpp dma.cpp dma.h mailbox.c
	g++ -Wall -g -O2 -L/opt/vc/lib -I/opt/vc/include -o test test.cpp dma.cpp mailbox.c -lm -lbcm_host -Wunused-variable -Wunused-function

clean:
	rm -f test

