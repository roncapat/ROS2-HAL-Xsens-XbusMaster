CC=g++
CFLAGS=-I. -Wall -Wno-attributes
LFLAGS=-lrt
DEPS = cmt1.h
OBJSTATIC = cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o

all: static

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<

static: $(OBJSTATIC)
	ar rcs libcmt.a $^

clean: clean_temp
	rm -f libcmt.a

clean_temp:
	rm -f *.o

