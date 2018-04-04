PROJ = ai
SCS = libscs
SCS_SRC_DIR = ../scs/src
SCS_INC_DIR = ../scs/include
AI_SRC_DIR = ../ai/src
AI_INC_DIR = ../ai/include
VPATH = $(SCS_SRC_DIR):$(SCS_INC_DIR):$(AI_SRC_DIR):$(AI_INC_DIR)

CXX = g++
CXXFLAGS = -I$(SCS_INC_DIR) -I$(AI_INC_DIR) -I/usr/include/eigen3 -g -DdDOUBLE -DQVGA
SCSOBJ = simulation.o draw.o car.o track.o api.o ui.o
AIOBJ = main.o
LIB = -lscs -lode -lglut -lGLU -lm -lGL

all: $(SCS).a $(PROJ)

$(SCS).a: $(SCSOBJ)
	ar rcs $@ $^

$(PROJ): $(AIOBJ) $(SCS).a
	$(CXX) -o $@ $< -L./ $(LIB)

run: ${PROJ}
	cd ../ai; ../build/$(PROJ)

main.o: scs.h
simulation.o: car.h track.h draw.h api.h simulation.h
draw.o: mytime.h simulation.h key.h car.h track.h common.h api.h draw.h
car.o: car.h draw.h
api.o: simulation.h draw.h common.h car.h mytime.h api.h track.h
track.o: track.h common.h car.h draw.h
ui.o: ui.h draw.h car.h key.h api.h simulation.h

clean:
	$(RM) *.o
	$(RM) $(SCS).a
	$(RM) $(PROJ)
