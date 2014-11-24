#!/usr/bin/make -f

CXX = g++
CXXFLAGS = -std=c++11 -O2 -Wall -Wextra -g
LDLIBS = \
	-lboost_program_options

EXEC = qksim

SOURCES = \
	  ils.cpp jit.cpp main.cpp

all: $(EXEC)

clean:
	$(RM) $(EXEC) *.o *.d tmp-qksim-*

%.o %.d: %.cpp
	$(CXX) -MMD $(CXXFLAGS) $(CPPFLAGS) -c -o $*.o $*.cpp

$(EXEC): $(SOURCES:.cpp=.o)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

-include $(wildcard *.d)
