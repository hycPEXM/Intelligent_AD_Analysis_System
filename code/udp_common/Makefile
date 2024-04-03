# ******************************************************
# Author       : Joey Qiu
# Last modified: 2023-07-06 11:32
# Email        : 2650982628@qq.com
# Filename     : Makefile
# Description  : 
# ******************************************************

CPP = g++
CPPFLAGS = -std=c++11 -Wall -Wextra

BUILD_DIR := build
TARGET := udp_test

SRCS := $(wildcard *.cpp)
OBJS := $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.o))

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CPP) $(CPPFLAGS) $^ -o $@

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(BUILD_DIR)
	$(CPP) $(CPPFLAGS) -c $< -o $@

clean:
	$(RM) -r $(BUILD_DIR) $(TARGET)

.PHONY: all clean

