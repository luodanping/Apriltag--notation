CC = gcc
CXX = g++

CPPFLAGS = -I..
CFLAGS = -g -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -O4
CXXFLAGS = -g -Wall -O4
LDFLAGS = -lpthread -lm

TARGETS := apriltag_demo opencv_demo play_video

.PHONY: all
all: apriltag_demo opencv_demo play_video

apriltag_demo: apriltag_demo.o ../libapriltag.a
	@echo "   [$@]"
	@$(CC) -o $@ $^ $(LDFLAGS)

opencv_demo: opencv_demo.o ../libapriltag.a 
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS)  `pkg-config --libs opencv`

play_video: play_video.o
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS)  `pkg-config --libs opencv`

%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS) $(CPPFLAGS)

%.o: %.cc
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS) `pkg-config --cflags opencv`
	#we need to add the header file here for its compilation

.PHONY: clean
clean:
	@rm -rf *.o $(TARGETS)
