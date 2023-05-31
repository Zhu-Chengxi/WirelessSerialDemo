CC=g++ -std=c++11
CFLAGS= -c -Wall
LDFLAGS_NVC= -L/usr/local/lib 
LDFLAGS= -L/usr/local/lib 
INCLUDE= -I/usr/include
#SOURCES=main.cpp
#OBJECTS=$(SOURCES:.cpp=.o)
SRC_PATH = src
OBJ_PATH = build

SRC_DIRS := $(shell find $(SRC_PATH)/* -type d)
OBJ_DIRS := $(OBJ_PATH)
OBJ_DIRS += $(SRC_DIRS:$(SRC_PATH)/%=$(OBJ_PATH)/%)

SOURCES := $(shell find $(SRC_PATH) -name *.cpp)
OBJECTS := $(SOURCES:$(SRC_PATH)/%.cc=$(OBJ_PATH)/%.o)
EXECUTABLE = main

all: $(SOURCES) $(EXECUTABLE)
    
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(INCLUDE) $(OBJECTS) -o $@ $(LDFLAGS) -lpthread
	
.cpp.o:
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@ -lpthread

clean:
	rm -rf main *.o .so *~
