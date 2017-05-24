#ifndef MATRIX_H_
#define MATRIX_H_	1

#include <stdlib.h>
#include <stdint.h>

#if defined USE_FLOAT_DATATYPE
	#define element float
#else
	#define element double
#endif

#if defined USE_STATIC_ALLOC
	#define MATRIX_MAX_SIZE		25
#endif

typedef struct {
	#if defined USE_STATIC_ALLOC
		element data[MATRIX_MAX_SIZE];
	#else
		element *data;
	#endif
	int rows;
	int cols;
} mat;

#if defined USE_STATIC_ALLOC
	mat *matrix_init(mat* m, int rows, int cols);
#else
	mat *matrix_init(mat** m, int rows, int cols);
#endif

#if defined USE_STDIO_DEBUG
	#include <stdio.h>
	void print_matrix(mat *m);
#endif

int matadd(mat* res, mat* m1, mat* m2);
int matsub(mat* res, mat* m1, mat* m2);
int matmul(mat* res, mat* m1, mat* m2);

int matpopulate(mat* dst, element* data);
element get_value(mat m, int row, int col);

#endif