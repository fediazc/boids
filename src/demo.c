#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <emscripten.h>
#include <emscripten/html5.h>
#include <SDL2/SDL.h>
#include "boids.h"

#define INIT_CANVAS_WIDTH  1200
#define INIT_CANVAS_HEIGHT 512
#define INIT_BOID_COUNT 400
#define MAX_BOID_COUNT 4000
#define FLOCK_MARGIN 40

SDL_Window *window;
SDL_Renderer *renderer;

void run_simulation(Flock *flock) 
{
    update_boids(flock);

    SDL_SetRenderDrawColor(renderer, 0, 20, 64, 255);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawColor(renderer, 225, 235, 191, 255);

    for (int i = 0; i < flock->boid_count; i++) {
        SDL_Rect rect = {
            .x = flock->boids[i].x,
            .y = flock->boids[i].y,
            .w = 6,
            .h = 6
        };
        SDL_RenderFillRect(renderer, &rect);
    }
    SDL_RenderPresent(renderer);
}

void update_flock_params(Flock *flock) 
{
    double cohesion_value = EM_ASM_DOUBLE({
        return document.getElementById('cohesion-input').value;
    });

    double alignment_value = EM_ASM_DOUBLE({
        return document.getElementById('alignment-input').value;
    });

    double separation_value = EM_ASM_DOUBLE({
        return document.getElementById('separation-input').value;
    });

    double max_speed_value = EM_ASM_DOUBLE({
        return document.getElementById('max-speed-input').value;
    });

    double visual_range_value = EM_ASM_DOUBLE({
        return document.getElementById('visual-range-input').value;
    });

    double boid_count_value = EM_ASM_DOUBLE({
        return document.getElementById('boid-count-input').value;
    });

    flock->cohesion_factor = cohesion_value * 0.06;
    flock->alignment_factor = alignment_value * 0.3;
    flock->avoidance_factor = separation_value;

    flock->max_speed = max_speed_value;
    flock->visual_range = visual_range_value;
    flock->boid_count = boid_count_value;
}

void handle_events(void *arg) 
{
    Flock *flock = (Flock *)arg;

    SDL_Event event;
    SDL_PollEvent(&event);

    update_flock_params(flock);
    run_simulation(flock);
}

void set_boid_canvas_size(int w, int h, Flock *flock) 
{
    emscripten_set_canvas_element_size("#canvas", w, h);
    SDL_SetWindowSize(window, w, h);

    flock->flight_area.x2 = w - FLOCK_MARGIN;
    flock->flight_area.y2 = h - FLOCK_MARGIN;
}

EM_BOOL window_resize_callback(int eventType, const EmscriptenUiEvent *uiEvent, 
                                void *userData) 
{
    Flock *flock = (Flock *)userData;

    int w = uiEvent->windowInnerWidth;
    int h = uiEvent->windowInnerHeight;

    set_boid_canvas_size(w, h, flock);

    return EM_FALSE;
}

int main() 
{
    SDL_Init(SDL_INIT_VIDEO);

    SDL_CreateWindowAndRenderer(INIT_CANVAS_WIDTH, INIT_CANVAS_HEIGHT, 
            SDL_WINDOW_RESIZABLE, &window, &renderer);

    Flock flock = {
        .boids = (Boid *)malloc(sizeof(Boid) * MAX_BOID_COUNT),
        .boid_count = INIT_BOID_COUNT,
        .max_boid_count = MAX_BOID_COUNT,
        .flight_area = {
            .x1 = FLOCK_MARGIN,
            .y1 = FLOCK_MARGIN,
            .x2 = INIT_CANVAS_WIDTH-FLOCK_MARGIN,
            .y2 = INIT_CANVAS_HEIGHT-FLOCK_MARGIN
        },
        .visual_range = 32,
        .protected_range = 6,
        .cohesion_factor = 0.0005,
        .alignment_factor = 0.1,
        .avoidance_factor = 0.05,
        .turn_factor = 0.15,
        .min_speed = 2,
        .max_speed = 5
    };

    srand(time(NULL));

    for (int i = 0; i < flock.max_boid_count; i++) {
        flock.boids[i].x = (rand() % (int)flock.flight_area.x2) + 1.0;
        flock.boids[i].y = (rand() % (int)flock.flight_area.y2) + 1.0;
        flock.boids[i].vx = -1 + (rand() % (3));
        flock.boids[i].vy = -1 + (rand() % (3));
    }

    flock.boid_count = EM_ASM_DOUBLE({
        return document.getElementById('boid-count-input').value;
    });

    int window_width = EM_ASM_INT({ return window.innerWidth; });
    int window_height = EM_ASM_INT({ return window.innerHeight; });

    set_boid_canvas_size(window_width, window_height, &flock);

    emscripten_set_resize_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, &flock, 
            EM_FALSE, window_resize_callback);
    emscripten_set_main_loop_arg(handle_events, &flock, 0, true);

    return 0;
}
