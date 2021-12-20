EXECUTABLE_NAME=pathdemo
CPP=clang
INC=-I/usr/include/SDL2
CPPFLAGS=-Wall -Wextra -Ofast -march=skylake -mno-vzeroupper
LIBS=-lSDL2 -lstdc++ -lm
CPPSOURCES=path-demo.cpp

.PHONY : all
all: $(CPPSOURCES) $(EXECUTABLE_NAME)

$(EXECUTABLE_NAME): $(CPPSOURCES)
	$(CPP) $(CPPFLAGS) $(CPPSOURCES) $(INC) -o $@ $(LIBS)

.PHONY : clean
clean:
	rm $(EXECUTABLE_NAME)
