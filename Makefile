#
# Makefile for the cm23 program
#
CFLAGS = -g3 -O3 
#
cm23	: cm23.o
	gcc $(CFLAGS) -o cm23 cm23.o -lm
cm23.o : cm23.c
	gcc $(CFLAGS) -c cm23.c


