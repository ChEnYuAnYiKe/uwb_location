// -------------------------------------------------------------------------------------------------------------------
//
//  File: trilateration.h
//
//  Copyright 2016 (c) Decawave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//
// -------------------------------------------------------------------------------------------------------------------

#ifndef __TRILATERATION_H__
#define __TRILATERATION_H__

#include "stdio.h"
#include <Eigen/Core>

#define	TRIL_3SPHERES			  3
#define	TRIL_4SPHERES			  4
#define Kp                        2.0f
#define Ki                        0.2f
#define halfT                     0.05f
#define squa( Sq )                (((float)Sq)*((float)Sq))
#define MAX_ANCHOR_NUMBER         4

/* Largest nonnegative number still considered zero */
#define MAXZERO 0.001

#define ERR_TRIL_CONCENTRIC -1
#define ERR_TRIL_COLINEAR_2SOLUTIONS -2
#define ERR_TRIL_SQRTNEGNUMB -3
#define ERR_TRIL_NOINTERSECTION_SPHERE4 -4
#define ERR_TRIL_NEEDMORESPHERE -5

#define UWB_ANC_BELOW_THREE -6
#define UWB_LIN_DEP_FOR_THREE -7
#define UWB_ANC_ON_ONE_LEVEL -8
#define UWB_LIN_DEP_FOR_FOUR -9
#define UWB_RANK_ZERO -10
#define UWB_OK 1
#define CM_ERR_ADDED 10

typedef struct vec3d	vec3d;

struct vec3d {
	double	x;
	double	y;
	double	z;
};

typedef struct {
	int x, y, z; //axis in cm
} position_t; // Position of a device or target in 3D space

/* Return the difference of two vectors, (vector1 - vector2). */
vec3d vdiff(const vec3d vector1, const vec3d vector2);

/* Return the sum of two vectors. */
vec3d vsum(const vec3d vector1, const vec3d vector2);

/* Multiply vector by a number. */
vec3d vmul(const vec3d vector, const double n);

/* Divide vector by a number. */
vec3d vdiv(const vec3d vector, const double n);

/* Return the Euclidean norm. */
double vdist(const vec3d v1, const vec3d v2);

/* Return the Euclidean norm. */
double vnorm(const vec3d vector);

/* Return the dot product of two vectors. */
double dot(const vec3d vector1, const vec3d vector2);

/* Replace vector with its cross product with another vector. */
vec3d cross(const vec3d vector1, const vec3d vector2);

double gdoprate(const vec3d tag, const vec3d p1, const vec3d p2, const vec3d p3);

int sphereline(const vec3d p1, const vec3d p2, const vec3d sc, double r, double *const mu1, double *const mu2);

int trilateration(vec3d *const result1,
                  vec3d *const result2,
                  vec3d *const best_solution,
                  const vec3d p1, const double r1,
                  const vec3d p2, const double r2,
                  const vec3d p3, const double r3,
                  const vec3d p4, const double r4,
                  const double maxzero);

int deca_3dlocate ( vec3d     *const solution1,
                    vec3d     *const solution2,
                    vec3d     *const best_solution,
                    int       *const nosolution_count,
                    double    *const best_3derror,
                    double    *const best_gdoprate,
                    vec3d p1, double r1,
                    vec3d p2, double r2,
                    vec3d p3, double r3,
                    vec3d p4, double r4,
                    int *combination);

int leastSquaresMethod(vec3d *best_solution, Eigen::MatrixXd anchorArray, int *distanceArray);

int cmp(const void *m,const void *n);

// int GetLocation(vec3d *best_solution, Eigen::MatrixXd anchorArray, int *distanceArray, int mode);

#endif
