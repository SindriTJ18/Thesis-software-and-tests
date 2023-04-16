BUILD_DIR := objects
SRC_DIR := source
INC_DIR := header

TARGET_EXEC := runner
SRC := $(wildcard $(SRC_DIR)/*.cpp)
OBJ := $(SRC:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)
HEAD := $(wildcard $(INC_DIR)/*.hpp)

CXX = g++
CXXFLAGS = -Wall
LDLIBS = -lpigpio -lpthread -lrt


.PHONY: all clean

all: $(TARGET_EXEC)

#to link objs into executable
$(TARGET_EXEC): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) -o $(TARGET_EXEC) $(LDLIBS)

#to build binaries
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp $(HEAD)
	$(CXX) $(CXXFLAGS) -c $< -o $@
	
clean:
	@$(RM) -rv $(BIN_DIR)/mpu9250 $(BUILD_DIR)/*.o