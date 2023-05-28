#ifndef BOIDS_H
#define BOIDS_H

typedef struct Boid {
    double x, y, vx, vy;
} Boid;

typedef struct BoidArea {
    double x1, y1, x2, y2;
} BoidArea;

typedef struct Flock {
    Boid *boids;
    int boid_count;
    BoidArea flight_area;
    double visual_range;
    double protected_range;
    double cohesion_factor;
    double alignment_factor;
    double avoidance_factor;
    double turn_factor;
    double min_speed;
    double max_speed;
} Flock;

void update_boids(Flock *flock);

#endif