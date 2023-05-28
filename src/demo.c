#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <emscripten.h>
#include <SDL2/SDL.h>
#include "boids.h"

#define SCREEN_WIDTH  1200
#define SCREEN_HEIGHT 512
#define INIT_BOID_COUNT 400

SDL_Window *window;
SDL_Renderer *renderer;

void run_simulation(Flock *flock) 
{
    update_boids(flock);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawColor(renderer, 120, 20, 180, 255);

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
        return document.getElementById('cohesion_input').value;
    });

    double alignment_value = EM_ASM_DOUBLE({
        return document.getElementById('alignment_input').value;
    });

    double separation_value = EM_ASM_DOUBLE({
        return document.getElementById('separation_input').value;
    });

    double max_speed_value = EM_ASM_DOUBLE({
        return document.getElementById('max_speed_input').value;
    });

    double visual_range_value = EM_ASM_DOUBLE({
        return document.getElementById('visual_range_input').value;
    });

    flock->cohesion_factor = cohesion_value * 0.06;
    flock->alignment_factor = alignment_value * 0.3;
    flock->avoidance_factor = separation_value;

    flock->max_speed = max_speed_value;
    flock->visual_range = visual_range_value;
}

void handle_events(void *arg) 
{
    Flock *flock = (Flock *)arg;

    SDL_Event event;
    SDL_PollEvent(&event);

    update_flock_params(flock);
    run_simulation(flock);
}

int main() 
{
    SDL_Init(SDL_INIT_VIDEO);

    SDL_CreateWindowAndRenderer(SCREEN_WIDTH, SCREEN_HEIGHT, 
            SDL_WINDOW_RESIZABLE, &window, &renderer);

    Flock flock = {
        .boids = (Boid *)malloc(sizeof(Boid) * INIT_BOID_COUNT),
        .boid_count = INIT_BOID_COUNT,
        .flight_area = {
            .x1 = 40,
            .y1 = 40,
            .x2 = SCREEN_WIDTH-40,
            .y2 = SCREEN_HEIGHT-40
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

    for (int i = 0; i < flock.boid_count; i++) {
        flock.boids[i].x = (rand() % (int)flock.flight_area.x2) + 1.0;
        flock.boids[i].y = (rand() % (int)flock.flight_area.y2) + 1.0;
        flock.boids[i].vx = -1 + (rand() % (3));
        flock.boids[i].vy = -1 + (rand() % (3));
    }

    emscripten_set_main_loop_arg(*handle_events, &flock, 0, true);

    return 0;
}
