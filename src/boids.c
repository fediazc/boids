#include <math.h>
#include "boids.h"

void update_boids(Flock *flock) 
{
    Boid *init_ptr = flock->boids;
    Boid *boid = init_ptr;
    for (int i = 0; i < flock->boid_count; i++, boid++) {
        double xpos_avg = 0.0,
               ypos_avg = 0.0,
               xvel_avg = 0.0,
               yvel_avg = 0.0,
               close_dx = 0.0,
               close_dy = 0.0;
               
        int neighbor_count = 0;

        Boid *other_boid = init_ptr;
        for (int j = 0; j < flock->boid_count; j++, other_boid++) {
            if (i != j) {
                double dx = boid->x - other_boid->x;
                double dy = boid->y - other_boid->y;
                double square_distance = dx*dx + dy*dy;

                if (square_distance < flock->protected_range * 
                        flock->protected_range) {
                    close_dx += boid->x - other_boid->x;
                    close_dy += boid->y - other_boid->y;
                }
                if (square_distance < flock->visual_range * 
                        flock->visual_range) {
                    xpos_avg += other_boid->x;
                    ypos_avg += other_boid->y;
                    xvel_avg += other_boid->vx;
                    yvel_avg += other_boid->vy;
                    neighbor_count++;
                }
            }
        }

        if (neighbor_count > 0) {
            xpos_avg /= neighbor_count;
            ypos_avg /= neighbor_count;
            xvel_avg /= neighbor_count;
            yvel_avg /= neighbor_count;

            boid->vx += (xpos_avg - boid->x)*flock->cohesion_factor;
            boid->vx += (xvel_avg - boid->vx)*flock->alignment_factor;

            boid->vy += (ypos_avg - boid->y)*flock->cohesion_factor;
            boid->vy += (yvel_avg - boid->vy)*flock->alignment_factor;
        }

        boid->vx += close_dx*flock->avoidance_factor;
        boid->vy += close_dy*flock->avoidance_factor;

        if (boid->y < flock->flight_area.y1) {
            boid->vy += flock->turn_factor;
        }
        if (boid->x > flock->flight_area.x2) {
            boid->vx -= flock->turn_factor;
        }
        if (boid->x < flock->flight_area.x1) {
            boid->vx += flock->turn_factor;
        }
        if (boid->y > flock->flight_area.y2) {
            boid->vy -= flock->turn_factor;
        }

        double speed = sqrt(boid->vx*boid->vx + boid->vy*boid->vy);

        if (speed < flock->min_speed) {
            boid->vx = (boid->vx / speed)*flock->min_speed;
            boid->vy = (boid->vy / speed)*flock->min_speed;
        }
        
        if (speed > flock->max_speed) {
            boid->vx = (boid->vx / speed)*flock->max_speed;
            boid->vy = (boid->vy / speed)*flock->max_speed;
        }

        boid->x += boid->vx;
        boid->y += boid->vy;
    }
}
