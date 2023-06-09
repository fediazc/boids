SARGS := -sUSE_SDL=2 -sUSE_SDL_GFX=2 -sALLOW_MEMORY_GROWTH
DEMO_SHELL_FILE := ./template/shell_minimal.html
DEMO_BUILD_DIR := ./build
DEMO_TARGET_EXEC := $(DEMO_BUILD_DIR)/demoapp.html
DEMO_OBJS := $(DEMO_BUILD_DIR)/demo.o $(DEMO_BUILD_DIR)/boids.o
OPTS := -O3 -flto -Wall
BUILD_FILES := $(DEMO_BUILD_DIR)/demoapp.*

$(DEMO_TARGET_EXEC): $(DEMO_OBJS) $(DEMO_SHELL_FILE) | $(DEMO_BUILD_DIR)
	emcc $(OPTS) --closure 1 $(DEMO_OBJS) -o $(DEMO_TARGET_EXEC) --shell-file $(DEMO_SHELL_FILE) $(SARGS)

$(DEMO_BUILD_DIR)/demo.o: demo.c boids.h | $(DEMO_BUILD_DIR)
	emcc $(OPTS) -c demo.c -o $(DEMO_BUILD_DIR)/demo.o

$(DEMO_BUILD_DIR)/boids.o: boids.c boids.h | $(DEMO_BUILD_DIR)
	emcc $(OPTS) -c boids.c -o $(DEMO_BUILD_DIR)/boids.o

$(DEMO_BUILD_DIR):
	mkdir -p $(DEMO_BUILD_DIR)

clean:
	rm -f $(BUILD_FILES) $(DEMO_OBJS)