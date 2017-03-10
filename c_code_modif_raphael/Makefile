CC=g++
CFLAGS=-c -Wall -g
LIBPATH = -I/usr/local/include/Eigen/
EXE_SOURCES=main.cpp
SOURCES=photogram.cpp
TEST_SOURCES=test_algorithm.cpp
LDFLAGS=-lCppUTest -lCppUTestExt
OBJECTS=$(SOURCES:.cpp=.o)
EXE_OBJECTS=$(EXE_SOURCES:.cpp=.o)
TEST_OBJECTS=$(TEST_SOURCES:.cpp=.o)
EXECUTABLE=photogram
UNITTESTS=test_algorithm

all: $(SOURCES)  $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) $(EXE_OBJECTS)
	$(CC) $(LDFLAGS)  $(OBJECTS) $(EXE_OBJECTS) -o $@

$(UNITTESTS): $(OBJECTS) $(TEST_OBJECTS)
	$(CC) $(LDFLAGS)  $(OBJECTS) $(TEST_OBJECTS) -o $@

.cpp.o:
	$(CC) $(LIBPATH) $(CFLAGS) $< -o $@

clean:
	rm *o $(OBJECTS) $(EXECUTABLE)

run: $(EXECUTABLE)
	./$(EXECUTABLE) ${ARGS}

check: $(UNITTESTS)
	./$(UNITTESTS)