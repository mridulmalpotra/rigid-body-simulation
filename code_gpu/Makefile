GCOMP = nvcc
CCOMP = g++
CPPFLAGS =-std=c++11 -g -lm -w

all:
	${GCOMP} -o rbcu simulatorutils.cu ${CPPFLAGS}
	${CCOMP} -o rbcpp rigidbody.cpp rigidbodysimulator.cpp vertex.cpp triple.cpp ${CPPFLAGS}
	${GCOMP} -o rbody rbcpp rbcu

clean:
	rm -f rbody
	rm -f rbcpp
	rm -f rbcu
