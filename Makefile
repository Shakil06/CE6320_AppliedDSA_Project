CXX = g++
CXXFLAGS = -std=c++11

SRC = dijkstraISCAS.cpp
TARGET = iscas

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f $(TARGET)