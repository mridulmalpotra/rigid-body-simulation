#GCOMP = nvcc
#CCOMP = g++
#CPPFLAGS =-std=c++11 -g -lm -w

#all:
#	${GCOMP} -o rbcu simulatorutils.cu ${CPPFLAGS}
#	${CCOMP} -o rbcpp rigidbody.cpp rigidbodysimulator.cpp vertex.cpp triple.cpp ${CPPFLAGS}
#	${GCOMP} -o rbody rbcpp rbcu

#clean:
#	rm -f rbody
#	rm -f rbcpp
#	rm -f rbcu
	
	
objects = rigidbodysimulator.o rigidbody.o vertex.o triple.o simulatorutils.o bodyutils.o

all: $(objects)
	nvcc -arch=sm_35 $(objects) -w -o app

%.o: %.cpp
	nvcc -x cu -w -arch=sm_35 -I. -dc $< -o $@

clean:
	rm -f *.o app
