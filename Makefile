CXX       ?= g++
CXX_FLAGS += -std=c++17 -ggdb -fPIC -g -pedantic -Wall -Wextra

BIN     := ./
LIB     := ./
TESTS	:= tests
SRC     := src

INCLUDES := -I$(SRC)

LIBRARIES   := -L$(LIB)

NAME	:= lib$(shell basename $(CURDIR)).so
TESTNAME:= test_ICM20948

all: $(LIB)/$(NAME) $(BIN)/$(TESTNAME)

fresh: clean all

$(LIB)/$(NAME): $(SRC)/*.cpp
	$(CXX) $(CXX_FLAGS) -shared $(INCLUDES) $^ -o $@ $(LIBRARIES)

$(BIN)/$(TESTNAME): $(TESTS)/*.cpp
	$(CXX) $(CXX_FLAGS) $(INCLUDES) $^ -o $@ $(LIBRARIES) -l$(shell basename $(CURDIR))

clean:
	-rm -f $(LIB)/$(NAME)
	-rm -f $(BIN)/$(TESTNAME)
