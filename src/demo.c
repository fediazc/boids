#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <emscripten.h>
#include <emscripten/html5.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include "boids.h"

#define INIT_CANVAS_WIDTH  1200
#define INIT_CANVAS_HEIGHT 512
#define INIT_BOID_COUNT 400
#define MAX_BOID_COUNT 5000
#define FLOCK_MARGIN 0

SDL_Window *window;
SDL_Renderer *renderer;

void rot(double *x, double *y, double pivotx, double pivoty, double rads)
{
    double nx = ((*x - pivotx)*cos(rads) - (*y - pivoty)*sin(rads)) + pivotx;
    double ny = ((*x - pivotx)*sin(rads) + (*y - pivoty)*cos(rads)) + pivoty;
    *x = nx;
    *y = ny;
}  

void draw_boid(const Boid *boid, double h1, double h2, double w, 
                const double *radius, uint8_t r, uint8_t g, uint8_t b) 
{
    double x1 = boid->x - h1;
    double y1 = boid->y - w;
    double x2 = x1;
    double y2 = boid->y + w;
    double x3 = boid->x + h2;
    double y3 = boid->y;

    rot(&x1, &y1, boid->x, boid->y, atan2(boid->vy, boid->vx));
    rot(&x2, &y2, boid->x, boid->y, atan2(boid->vy, boid->vx));
    rot(&x3, &y3, boid->x, boid->y, atan2(boid->vy, boid->vx));

    filledTrigonRGBA(renderer, x1, y1, x2, y2, x3, y3, r, g, b, 255);

    if (radius != NULL) {
        circleRGBA(renderer, boid->x, boid->y, *radius, 255, 0, 0, 255);
    }
}

void connect_boids(const Boid *boid, const Boid *neighbor, void *data)
{
    SDL_Renderer *renderer = (SDL_Renderer *)data;
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_RenderDrawLineF(renderer, boid->x, boid->y, neighbor->x, neighbor->y);
}

void draw_qt(const BoidQuadTree *tree, double depth, double *rootlen, void *data)
{
    SDL_Renderer *renderer = (SDL_Renderer *)data;
    double hd = QT_half_dimension(depth, rootlen);
    SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
    SDL_FRect rect = {.x = tree->x - hd, .y = tree->y - hd, .w = hd*2, .h = hd*2};
    SDL_RenderDrawRectF(renderer, &rect);

    if (tree->northWest == NULL) {
        return;
    }

    draw_qt(tree->northWest, depth + 1, rootlen, data);
    draw_qt(tree->northEast, depth + 1, rootlen, data);
    draw_qt(tree->southWest, depth + 1, rootlen, data);
    draw_qt(tree->southEast, depth + 1, rootlen, data);
}

void run_simulation(Flock *flock)
{
    int draw_vision = EM_ASM_INT({
        return document.getElementById('show-vision-input').checked;
    });

    int draw_radius = EM_ASM_INT({
         return document.getElementById('show-range-input').checked;
    });

    int draw_tree = EM_ASM_INT({
        return document.getElementById('show-grid-input').checked;
    });

    int color_flag = EM_ASM_INT({
        return document.getElementById('color-boids-input').checked;
    });

    int wrap = EM_ASM_INT({
        return document.getElementById('wrap-behavior-input').checked;
    });

    SDL_SetRenderDrawColor(renderer, 225, 225, 225, 255);
    SDL_RenderClear(renderer);

    neighbor_cb neighbor_func_ptr = NULL;
    if (draw_vision) {
        neighbor_func_ptr = connect_boids;
    }

    tree_cb tree_func_ptr = NULL;
    if (draw_tree) {
        tree_func_ptr = draw_qt;
    }

    double *radius = NULL; 
    if (draw_radius) {
        radius = &(flock->visual_range);
    }

    update_boids(flock, wrap, neighbor_func_ptr, tree_func_ptr, renderer);

    uint8_t r, g, b;
    double speed;
    Boid *curr_boid;
    for (int i = 0; i < flock->boid_count; i++) {
        curr_boid = &(flock->boids[i]);
        speed = ambm(curr_boid->vx, curr_boid->vy);
        r = color_flag ? (255/flock->max_speed)*speed : 30;
        g = color_flag ? 0 : 30;
        b = color_flag ? (-255/flock->max_speed)*speed + 255 : 30;
        draw_boid(curr_boid, 5.0, 7.0, 5.0, radius, r, g, b);
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

    double speed_value = EM_ASM_DOUBLE({
        return document.getElementById('speed-input').value;
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

    flock->min_speed = speed_value;
    flock->max_speed = speed_value + 5;
    flock->visual_range = visual_range_value;
    flock->boid_count = boid_count_value;
}

void handle_events(void *arg) 
{
    Flock *flock = (Flock *)arg;

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

EM_BOOL mousedown_callback(int eventType, const EmscriptenMouseEvent *mouseEvent, 
                            void *userData)
{
    Flock *flock = (Flock *)userData;

    if (mouseEvent->button == 0) {
        flock->follow_flag = 1;
    }

    return EM_FALSE;
}

EM_BOOL touchstart_callback(int eventType, const EmscriptenTouchEvent *touchEvent, 
                            void *userData)
{
    Flock *flock = (Flock *)userData;

    if (touchEvent->numTouches > 0) {
        flock->follow_flag = 1;
    }

    return EM_FALSE;
}

EM_BOOL touchend_callback(int eventType, const EmscriptenTouchEvent *touchEvent, 
                            void *userData)
{
    Flock *flock = (Flock *)userData;

    int num_changed = 0;
    for (int i = 0; i < touchEvent->numTouches; i++) {
        if (touchEvent->touches[i].isChanged) {
            num_changed++;
        }
    } 

    if (touchEvent->numTouches == num_changed) {
        flock->follow_flag = 0;
    }

    return EM_FALSE;
}

EM_BOOL touchmove_callback(int eventType, const EmscriptenTouchEvent *touchEvent, 
                            void *userData)
{
    Flock *flock = (Flock *)userData;

    flock->follow_target_count = touchEvent->numTouches;
    for (int i = 0; i < touchEvent->numTouches && i < 32; i++) {
        flock->follow_x[i] = touchEvent->touches[i].targetX;
        flock->follow_y[i] = touchEvent->touches[i].targetY;
    }

    return EM_FALSE;
}

EM_BOOL mouseup_callback(int eventType, const EmscriptenMouseEvent *mouseEvent, 
                            void *userData)
{
    Flock *flock = (Flock *)userData;

    if (mouseEvent->button == 0) {
        flock->follow_flag = 0;
    }

    return EM_FALSE;
}

EM_BOOL mousemove_callback(int eventType, const EmscriptenMouseEvent *mouseEvent, 
                            void *userData)
{
    Flock *flock = (Flock *)userData;

    flock->follow_target_count = 1;

    flock->follow_x[0] = mouseEvent->targetX;
    flock->follow_y[0] = mouseEvent->targetY;
    
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
        .turn_factor = 0.06,
        .min_speed = 2,
        .max_speed = 7,
        .follow_target_count = 0,
        .follow_flag = 0
    };

    srand(time(NULL));

    flock.boid_count = EM_ASM_INT({
        return document.getElementById('boid-count-input').value;
    });

    int window_width = EM_ASM_INT({ return window.innerWidth; });
    int window_height = EM_ASM_INT({ return window.innerHeight; });

    set_boid_canvas_size(window_width, window_height, &flock);

    for (int i = 0; i < flock.max_boid_count; i++) {
        flock.boids[i].x = (rand() % (int)flock.flight_area.x2) + 1.0;
        flock.boids[i].y = (rand() % (int)flock.flight_area.y2) + 1.0;
        flock.boids[i].vx = -1 + (rand() % (3));
        flock.boids[i].vy = -1 + (rand() % (3));
    }

    emscripten_set_resize_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, &flock, 
            EM_FALSE, window_resize_callback);

    emscripten_set_mousedown_callback("#canvas", &flock, 
        EM_FALSE, mousedown_callback);

    emscripten_set_mousemove_callback("#canvas", &flock, 
        EM_FALSE, mousemove_callback);

    emscripten_set_mouseup_callback("#canvas", &flock, 
        EM_FALSE, mouseup_callback);

    emscripten_set_touchstart_callback("#canvas", &flock, 
        EM_FALSE, touchstart_callback);

    emscripten_set_touchmove_callback("#canvas", &flock, 
        EM_FALSE, touchmove_callback);

    emscripten_set_touchend_callback("#canvas", &flock, 
        EM_FALSE, touchend_callback);

    emscripten_set_touchcancel_callback("#canvas", &flock, 
        EM_FALSE, touchend_callback);

    emscripten_set_main_loop_arg(handle_events, &flock, 0, 1);

    return 0;
}
