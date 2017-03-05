CC=g++
CFLAGS=-c -Wall
LIBPATH = -I/usr/local/include/Eigen/
SOURCES=algorithm.cpp
LDFLAGS=
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=algorithm

all: $(SOURCES)  $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS)  $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(LIBPATH) $(CFLAGS) $< -o $@

clean:
	rm *o $(OBJECTS) $(EXECUTABLE)

run: ./$(EXECUTABLE)
	./$(EXECUTABLE) ${ARGS}