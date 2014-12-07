#!/usr/bin/make -f

CC = gcc
CFLAGS = -std=c99 -O2 -Wall -Wextra -g
CXX = g++
CXXFLAGS = -std=c++11 -O2 -Wall -Wextra -g
LDLIBS = \
	-lboost_program_options

EXEC = qksim

FPU_SOURCES = \
	      fadd.c fcmp.c fdiv.c ffloor.c finv.c float.c \
	      fmul.c fsqrt.c ftoi.c itof.c

SOURCES = \
	  native_fpu.cpp \
	  options.cpp ils.cpp jit.cpp cas.cpp main.cpp

all: $(EXEC)

clean:
	$(RM) $(EXEC) *.o *.d tmp-qksim-*

$(FPU_SOURCES:%.c=fpu/C/%.o): $(FPU_SOURCES:%.c=fpu/C/%.c)
	$(MAKE) -C fpu/C/

%.o %.d: %.c
	$(CC) -MMD $(CFLAGS) $(CPPFLAGS) -c -o $*.o $*.c
%.o %.d: %.cpp
	$(CXX) -MMD $(CXXFLAGS) $(CPPFLAGS) -c -o $*.o $*.cpp

$(EXEC): $(SOURCES:.cpp=.o) $(FPU_SOURCES:%.c=fpu/C/%.o)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

-include $(wildcard *.d)
