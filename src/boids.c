#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "boids.h"

void update_boids(Flock *flock) 
{
    Boid *init_ptr = flock->boids;
    Boid *boid = init_ptr;

    AABB boundary = {
        .center = {
            .x = flock->flight_area.x2 / 2, 
            .y = flock->flight_area.y2 / 2
        },
        .half_dimension = flock->flight_area.x2 / 2
    };

    BoidQuadTree *tree = construct_quadtree(15, boundary);
    for (int i = 0; i < flock->boid_count; i++) {
        QT_insert_boid(tree, &(flock->boids[i]));
    }

    for (int i = 0; i < flock->boid_count; i++, boid++) {
        double xpos_avg = 0.0,
               ypos_avg = 0.0,
               xvel_avg = 0.0,
               yvel_avg = 0.0,
               close_dx = 0.0,
               close_dy = 0.0;
               
        int neighbor_count = 0;

        AABB range = {
            .center = {
                .x = flock->boids[i].x,
                .y = flock->boids[i].y
            },
            .half_dimension = flock->visual_range*2
        };

        BoidNode *other_boid_head = QT_boids_in_range(range, tree);
        BoidNode *other_boid = other_boid_head;
        while (other_boid != NULL) {
            double dx = boid->x - other_boid->boid.x;
            double dy = boid->y - other_boid->boid.y;
            if (other_boid->boid.x != flock->boids[i].x && 
                other_boid->boid.y != flock->boids[i].y &&
                fabs(dx) < flock->visual_range &&
                fabs(dy) < flock->visual_range) {
                
                double square_dist = dx*dx + dy*dy;

                if (square_dist < flock->visual_range*flock->visual_range) {
                    close_dx += boid->x - other_boid->boid.x;
                    close_dy += boid->y - other_boid->boid.y;
                    xpos_avg += other_boid->boid.x;
                    ypos_avg += other_boid->boid.y;
                    xvel_avg += other_boid->boid.vx;
                    yvel_avg += other_boid->boid.vy;

                    neighbor_count++;
                }
            }

            other_boid = other_boid->next;
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

            double mag = sqrt(close_dx*close_dx + close_dy*close_dy);

            close_dx /= mag;
            close_dy /= mag;

            boid->vx += close_dx*flock->avoidance_factor;
            boid->vy += close_dy*flock->avoidance_factor;
        }

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

        list_cleanup(other_boid_head);
    }
    QT_cleanup(tree);
}

int contains_point(AABB box, XY p)
{
    double cx = box.center.x;
    double cy = box.center.y;
    double h = box.half_dimension;
    return (p.x < cx + h) && (p.x > cx - h) && (p.y < cy + h) && (p.y > cy - h);
}

int intersects_AABB(AABB box, AABB other)
{
    double cx = box.center.x;
    double cy = box.center.y;
    double h = box.half_dimension;
    XY p1 = {.x = cx - h, .y = cy - h};
    XY p2 = {.x = cx + h, .y = cy - h};
    XY p3 = {.x = cx - h, .y = cy + h};
    XY p4 = {.x = cx + h, .y = cy + h};

    return contains_point(other, p1) ||
            contains_point(other, p2) ||
            contains_point(other, p3) ||
            contains_point(other, p4);
}

BoidQuadTree *construct_quadtree(int cap, AABB box)
{
    BoidQuadTree *qt = (BoidQuadTree *)malloc(sizeof(BoidQuadTree));
    
    qt->NODE_CAPACITY = cap;
    qt->boundary = box;
    qt->points = (XY *)malloc(sizeof(XY) * cap);
    qt->boids = (Boid *)malloc(sizeof(Boid) * cap);
    qt->size = 0;
    qt->northEast = NULL;
    qt->northWest = NULL;
    qt->southEast = NULL;
    qt->southWest = NULL;

    return qt;
}

void QT_subdivide(BoidQuadTree *qt)
{
    double qd = qt->boundary.half_dimension / 2.0;
    double cx = qt->boundary.center.x;
    double cy = qt->boundary.center.y;
    XY c1 = {.x = cx - qd, .y = cy - qd};
    XY c2 = {.x = cx + qd, .y = cy - qd};
    XY c3 = {.x = cx - qd, .y = cy + qd};
    XY c4 = {.x = cx + qd, .y = cy + qd};

    AABB box1 = {.center = c1, .half_dimension=qd};
    AABB box2 = {.center = c2, .half_dimension=qd};
    AABB box3 = {.center = c3, .half_dimension=qd};
    AABB box4 = {.center = c4, .half_dimension=qd};

    qt->northWest = construct_quadtree(qt->NODE_CAPACITY, box1);
    qt->northEast = construct_quadtree(qt->NODE_CAPACITY, box2);
    qt->southWest = construct_quadtree(qt->NODE_CAPACITY, box3);
    qt->southEast = construct_quadtree(qt->NODE_CAPACITY, box4);
}

void push_boid(BoidNode **headRef, Boid newBoid)
{
    BoidNode *newNode = (BoidNode *)malloc(sizeof(BoidNode));

    newNode->boid = newBoid;
    newNode->next = *headRef;
    *headRef = newNode;
}

void push_all(BoidNode **listHeadRef, BoidNode *otherHeadRef)
{
    if (*listHeadRef != NULL && otherHeadRef != NULL) {
        BoidNode *last = otherHeadRef;
        while (last->next != NULL) {
            last = last->next;
        }
        last->next = (*listHeadRef);
        *listHeadRef = otherHeadRef;
    } else if (*listHeadRef == NULL) {
        *listHeadRef = otherHeadRef;
    }
}

int QT_insert_boid(BoidQuadTree *qt, Boid *boid)
{
    XY p = {.x = boid->x, .y = boid->y};
    if (!contains_point(qt->boundary, p)) {
        return 0;
    }

    if (qt->size < qt->NODE_CAPACITY && qt->northWest == NULL) {
        qt->points[qt->size] = p;
        qt->boids[qt->size] = *boid;
        qt->size++;
        return 1;
    }

    if (qt->northWest == NULL) {
        QT_subdivide(qt);
    }

    if (QT_insert_boid(qt->northWest, boid)) return 1;
    if (QT_insert_boid(qt->northEast, boid)) return 1;
    if (QT_insert_boid(qt->southWest, boid)) return 1;
    if (QT_insert_boid(qt->southEast, boid)) return 1;

    return 0;
}

BoidNode *QT_boids_in_range(AABB range, BoidQuadTree *qt)
{
    BoidNode *boids_in_range = NULL;

    if(!intersects_AABB(range, qt->boundary)) {
        return boids_in_range;
    }

    for (int i = 0; i < qt->size; i++) {
        if (contains_point(range, qt->points[i])) {
            push_boid(&boids_in_range, qt->boids[i]);
        }
    }

    if (qt->northWest == NULL) {
        return boids_in_range;
    }

    push_all(&boids_in_range, QT_boids_in_range(range, qt->northWest));
    push_all(&boids_in_range, QT_boids_in_range(range, qt->northEast));
    push_all(&boids_in_range, QT_boids_in_range(range, qt->southWest));
    push_all(&boids_in_range, QT_boids_in_range(range, qt->southEast));

    return boids_in_range;
}

void QT_cleanup(BoidQuadTree *qt)
{
    if (qt->northWest == NULL) {
        free(qt->points);
        free(qt->boids);
        free(qt);
        return;
    }
    
    QT_cleanup(qt->northWest);
    QT_cleanup(qt->northEast);
    QT_cleanup(qt->southWest);
    QT_cleanup(qt->southEast);

    free(qt->points);
    free(qt->boids);
    free(qt);
    return;
}

void list_cleanup(BoidNode *head)
{
    BoidNode *tmp;
    while (head != NULL) {
        tmp = head;
        head = head->next;
        free(tmp);
    }
}
