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
    int max_boid_count;
    BoidArea flight_area;
    double visual_range;
    double protected_range;
    double cohesion_factor;
    double alignment_factor;
    double avoidance_factor;
    double turn_factor;
    double min_speed;
    double max_speed;
    int follow_flag;
    int follow_target_count;
    double follow_x[32];
    double follow_y[32];
} Flock;

typedef struct BoidQuadTree {
    int NODE_CAPACITY;
    double x;
    double y;
    Boid *boids;
    int size;
    struct BoidQuadTree *northWest;
    struct BoidQuadTree *northEast;
    struct BoidQuadTree *southWest;
    struct BoidQuadTree *southEast;
} BoidQuadTree;

typedef struct BoidNode {
    Boid boid;
    struct BoidNode *next;
} BoidNode;

typedef void (*neighbor_cb)(const Boid *boid, const Boid *neighbor, void *data);
typedef void (*tree_cb)(const BoidQuadTree *tree, double depth, double *rootlen, void *data);

int contains_point(double bbx, double bby, double bbhd, double x, double y);
int intersects_AABB(double bbx, double bby, double bbhd, double qtx, double qty, double qhd);
BoidQuadTree *construct_quadtree(int cap, double cx, double cy);
double QT_half_dimension(int depth, const double *rootlen);
void QT_subdivide(BoidQuadTree *qt, int depth, const double *rootlen);
int QT_insert_boid(BoidQuadTree *qt, Boid *boid, int depth, const double *rootlen);
BoidNode *QT_boids_in_range(BoidQuadTree *qt, double bbx, double bby, double bbhd, int depth, const double *rootlen);
void QT_cleanup(BoidQuadTree *qt);
void list_cleanup(BoidNode *boid);
void push_boid(BoidNode **headRef, Boid newBoid);
void push_all(BoidNode **listHead, BoidNode *otherHead);
void update_boids(Flock *flock, int wrap, neighbor_cb process_neighbor, tree_cb process_tree, void *data);
double ambm(double a, double b); 

#endif