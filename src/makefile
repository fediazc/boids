SARGS := -sUSE_SDL=2 -sUSE_SDL_GFX=2 -sALLOW_MEMORY_GROWTH
WP_SARGS := -sSINGLE_FILE
DEMO_SHELL_FILE := ./template/shell_minimal.html
WP_SHELL_FILE := ./template/shell_wallpaper.html
BUILD_DIR := ./build
DEMO_TARGET := $(BUILD_DIR)/demoapp.html
WP_TARGET := $(BUILD_DIR)/wallpaper.html
DEMO_OBJS := $(BUILD_DIR)/demo.o $(BUILD_DIR)/boids.o
WP_OBJS := $(BUILD_DIR)/wallpaper.o $(BUILD_DIR)/boids.o
OPTS := -O3 -flto -Wall
BUILD_FILES := $(BUILD_DIR)/demoapp.* $(BUILD_DIR)/wallpaper.html $(DEMO_OBJS) $(WP_OBJS)

all: $(DEMO_TARGET) $(WP_TARGET)

$(DEMO_TARGET): $(DEMO_OBJS) $(DEMO_SHELL_FILE) | $(BUILD_DIR)
	emcc $(OPTS) --closure 1 $(DEMO_OBJS) -o $(DEMO_TARGET) --shell-file $(DEMO_SHELL_FILE) $(SARGS)

$(WP_TARGET): $(WP_OBJS) $(WP_SHELL_FILE) | $(BUILD_DIR)
	emcc $(OPTS) $(WP_OBJS) -o $(WP_TARGET) --shell-file $(WP_SHELL_FILE) $(SARGS) $(WP_SARGS) --pre-js ./wallpaper/wallpaper.js

$(BUILD_DIR)/wallpaper.o: demo.c boids.h | $(BUILD_DIR)
	emcc $(OPTS) -c demo.c -o $(BUILD_DIR)/wallpaper.o -D WALLPAPER

$(BUILD_DIR)/demo.o: demo.c boids.h | $(BUILD_DIR)
	emcc $(OPTS) -c demo.c -o $(BUILD_DIR)/demo.o

$(BUILD_DIR)/boids.o: boids.c boids.h | $(BUILD_DIR)
	emcc $(OPTS) -c boids.c -o $(BUILD_DIR)/boids.o

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/*.wasm $(BUILD_DIR)/*.js $(DEMO_TARGET) $(WP_TARGET)