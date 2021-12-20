EXECUTABLE_NAME=pathchallenge
CPP=clang
INC=
CPPFLAGS=-Wall -Wextra -Ofast -march=skylake -mno-vzeroupper
LIBS=-lstdc++ -lm
CPPSOURCES=path-challenge.cpp path-challenge-impl.cpp

.PHONY : all
all: $(CPPSOURCES) $(EXECUTABLE_NAME)

$(EXECUTABLE_NAME): $(CPPSOURCES)
	$(CPP) $(CPPFLAGS) $(CPPSOURCES) $(INC) -o $@ $(LIBS)

.PHONY : clean
clean:
	rm $(EXECUTABLE_NAME)
