#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "boids.h"

void update_boids(Flock *flock) 
{
    Boid *init_ptr = flock->boids;
    Boid *boid = init_ptr;

    double qtx = flock->flight_area.x2 / 2;
    double qty =  flock->flight_area.y2 / 2;
    double rootlen = flock->flight_area.x2 / 2;

    BoidQuadTree *tree = construct_quadtree(15, qtx, qty);
    for (int i = 0; i < flock->boid_count; i++) {
        QT_insert_boid(tree, &(flock->boids[i]), 0, &rootlen);
    }

    for (int i = 0; i < flock->boid_count; i++, boid++) {
        double xpos_avg = 0.0,
               ypos_avg = 0.0,
               xvel_avg = 0.0,
               yvel_avg = 0.0,
               close_dx = 0.0,
               close_dy = 0.0;
               
        int neighbor_count = 0;

        double bbx = flock->boids[i].x;
        double bby = flock->boids[i].y;
        double bbhd = flock->visual_range*2;

        BoidNode *other_boid_head = NULL;
        if (boid->x < flock->flight_area.x2 || boid->y < flock->flight_area.y2) {
            other_boid_head = QT_boids_in_range(tree, bbx, bby, bbhd, 0, &rootlen);
        }
        
        BoidNode *other_boid = other_boid_head;
        while (other_boid != NULL) {
            double dx = boid->x - other_boid->boid.x;
            double dy = boid->y - other_boid->boid.y;
            if (other_boid->boid.x != boid->x && 
                other_boid->boid.y != boid->y &&
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

            double mag = ambm(close_dx, close_dy);

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

        double speed = ambm(boid->vx, boid->vy);

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

double QT_half_dimension(int depth, const double *rootlen)
{
    return depth ? *rootlen / (1 << depth) : *rootlen;
}

int contains_point(double bbx, double bby, double bbhd, double x, double y)
{
    return (x < bbx + bbhd) && (x > bbx - bbhd) && (y < bby + bbhd) && (y > bby - bbhd);
}

int intersects_AABB(double bbx, double bby, double bbhd, double qtx, double qty, double qhd)
{
    return contains_point(qtx, qty, qhd, bbx - bbhd, bby - bbhd) ||
            contains_point(qtx, qty, qhd, bbx + bbhd, bby - bbhd) ||
            contains_point(qtx, qty, qhd, bbx - bbhd, bby + bbhd) ||
            contains_point(qtx, qty, qhd, bbx + bbhd, bby + bbhd);
}

BoidQuadTree *construct_quadtree(int cap, double cx, double cy)
{
    BoidQuadTree *qt = (BoidQuadTree *)malloc(sizeof(BoidQuadTree));
    
    qt->NODE_CAPACITY = cap;
    qt->x = cx;
    qt->y = cy;
    qt->boids = (Boid *)malloc(sizeof(Boid) * cap);
    qt->size = 0;
    qt->northEast = NULL;
    qt->northWest = NULL;
    qt->southEast = NULL;
    qt->southWest = NULL;

    return qt;
}

void QT_subdivide(BoidQuadTree *qt, int depth, const double *rootlen)
{
    double qd = QT_half_dimension(depth + 1, rootlen);

    qt->northWest = construct_quadtree(qt->NODE_CAPACITY, qt->x - qd, qt->y - qd);
    qt->northEast = construct_quadtree(qt->NODE_CAPACITY, qt->x + qd, qt->y - qd);
    qt->southWest = construct_quadtree(qt->NODE_CAPACITY, qt->x - qd, qt->y + qd);
    qt->southEast = construct_quadtree(qt->NODE_CAPACITY, qt->x + qd, qt->y + qd);
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
    if (*listHeadRef == otherHeadRef) {
        return;
    }
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

int QT_insert_boid(BoidQuadTree *qt, Boid *boid, int depth, const double *rootlen)
{
    double hd = QT_half_dimension(depth, rootlen);
    if (!contains_point(qt->x, qt->y, hd, boid->x, boid->y)) {
        return 0;
    }

    if (qt->size < qt->NODE_CAPACITY && qt->northWest == NULL) {
        qt->boids[qt->size] = *boid;
        qt->size++;
        return 1;
    }

    if (qt->northWest == NULL) {
        QT_subdivide(qt, depth, rootlen);
    }

    if (QT_insert_boid(qt->northWest, boid, depth + 1, rootlen)) return 1;
    if (QT_insert_boid(qt->northEast, boid, depth + 1, rootlen)) return 1;
    if (QT_insert_boid(qt->southWest, boid, depth + 1, rootlen)) return 1;
    if (QT_insert_boid(qt->southEast, boid, depth + 1, rootlen)) return 1;

    return 0;
}

BoidNode *QT_boids_in_range(BoidQuadTree *qt, double bbx, double bby, double bbhd, int depth, const double *rootlen)
{
    BoidNode *boids_in_range = NULL;

    double hd = QT_half_dimension(depth, rootlen);
    if(!intersects_AABB(bbx, bby, bbhd, qt->x, qt->y, hd)) {
        return boids_in_range;
    }

    for (int i = 0; i < qt->size; i++) {
        if (contains_point(bbx, bby, bbhd, qt->boids[i].x, qt->boids[i].y)) {
            push_boid(&boids_in_range, qt->boids[i]);
        }
    }

    if (qt->northWest == NULL) {
        return boids_in_range;
    }

    push_all(&boids_in_range, QT_boids_in_range(qt->northWest, bbx, bby, bbhd, depth + 1, rootlen));
    push_all(&boids_in_range, QT_boids_in_range(qt->northEast, bbx, bby, bbhd, depth + 1, rootlen));
    push_all(&boids_in_range, QT_boids_in_range(qt->southWest, bbx, bby, bbhd, depth + 1, rootlen));
    push_all(&boids_in_range, QT_boids_in_range(qt->southEast, bbx, bby, bbhd, depth + 1, rootlen));

    return boids_in_range;
}

void QT_cleanup(BoidQuadTree *qt)
{
    if (qt->northWest == NULL) {
        free(qt->boids);
        free(qt);
        return;
    }
    
    QT_cleanup(qt->northWest);
    QT_cleanup(qt->northEast);
    QT_cleanup(qt->southWest);
    QT_cleanup(qt->southEast);

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

double ambm(double a, double b)
{
    a = fabs(a);
    b = fabs(b);
    if (a > b) {
        return a*0.960433870103 + b*0.397824734759;
    } else {
        return b*0.960433870103 + a*0.397824734759;
    }
}