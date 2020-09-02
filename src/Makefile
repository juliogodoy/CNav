.SUFFIXES:
.SUFFIXES: .cpp .o

AR = ar
ARFLAGS = cru
CXX = g++
CXXFLAGS = -g -O2
RANLIB = ranlib
RM = rm -f
INCLUDES = -I.
OBJECTS = Agent.o KdTree.o Obstacle.o RVOSimulator.o

all: libRVO.a

libRVO.a: $(OBJECTS) 
	$(RM) libRVO.a
	$(AR) $(ARFLAGS) libRVO.a $(OBJECTS)
	$(RANLIB) libRVO.a

.cpp.o:
	$(CXX) $(INCLUDES) $(CXXFLAGS) -c -o $@ $<

clean:
	$(RM) libRVO.a
	$(RM) *.o

.PHONY: all clean

.NOEXPORT:
