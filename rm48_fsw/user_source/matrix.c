#include "matrix.h"

#if defined USE_STATIC_ALLOC
	mat *matrix_init(mat* m, int rows, int cols)
	{
		m->rows = rows;
		m->cols = cols;
		int i = 0;
		int j = 0;
		for(i=0; i<m->rows; ++i)
		{
			for(j=0; j<m->cols; ++j)
			{
				m->data[i*m->cols + j] = (element)0.0f;
			}
		}
		return m;
	}
#else
	mat *matrix_init(mat** m, int rows, int cols)
	{
		if(*m == NULL)
		{
			*m = malloc(sizeof(mat));
			(*m)->data = malloc(rows*cols*sizeof(element));
			(*m)->rows = rows;
			(*m)->cols = cols;
			return *m;
		}
		if((*m)->data == NULL)
		{
			(*m)->data = malloc(rows*cols*sizeof(element));
			(*m)->rows = rows;
			(*m)->cols = cols;
			return *m;
		}
		element *temp = realloc((*m)->data, rows*cols*sizeof(element));
		if(temp != NULL)
		{
			(*m)->data = temp;
		}
		return *m;
	}
#endif

#if defined USE_STDIO_DEBUG
	void print_matrix(mat *m)
	{
		int i = 0;
		int j = 0;
		for(i=0; i <m->rows; ++i)
		{
			for(j=0; j<m->cols; ++j)
			{
				printf("%f ", m->data[i*m->cols + j]);
			}
			printf("\n");
		}
	}
#endif

int matadd(mat* res, mat* m1, mat* m2)
{
	if(m1->rows != m2->rows)
	{
		return -1;
	}
	if(m1->cols != m2->cols)
	{
		return -1;
	}
	if(res == NULL)
	{
		return -2;
	}
	if(res->data == NULL)
	{
		return -3;
	}
	if(res->rows != m1->rows)
	{
		return -4;
	}
	if(res->cols != m1->cols)
	{
		return -5;
	}
	int i = 0;
	for(i=0; i<res->rows*res->cols; ++i)
	{
		res->data[i] = m1->data[i] + m2->data[i];
	}
	return 0;
}

int matsub(mat* res, mat* m1, mat* m2)
{
	if(m1->rows != m2->rows)
	{
		return -1;
	}
	if(m1->cols != m2->cols)
	{
		return -1;
	}
	if(res == NULL)
	{
		return -2;
	}
	if(res->data == NULL)
	{
		return -3;
	}
	if(res->rows != m1->rows)
	{
		return -4;
	}
	if(res->cols != m1->cols)
	{
		return -5;
	}
	int i = 0;

	for(i=0; i<res->rows*res->cols; ++i)
	{
		res->data[i] = m1->data[i] - m2->data[i];
	}
	return 0;
}

int matmul(mat* res, mat* m1, mat* m2)
{
	if(m1->cols != m2->rows)
	{
		return -1;
	}
	if(res == NULL)
	{
		return -2;
	}
	if(res->data == NULL)
	{
		return -3;
	}
	if(res->rows != m1->rows)
	{
		return -4;
	}
	if(res->cols != m2->cols)
	{
		return -5;
	}

	int i = 0;
	int j = 0;
	int k = 0;

	for(i=0; i<res->rows; ++i)
	{
		for(j=0; j<res->cols; ++j)
		{
			res->data[i*res->cols + j] = 0.0f;
			for(k=0; k<m1->cols; ++k)
			{
				res->data[i*res->cols + j] += m1->data[i*m1->cols+k] * m2->data[k*m2->cols + j];
			}
		}
	}
	return 0;
}

int matpopulate(mat* dst, element* data)
{
	int i = 0;

	for(i=0; i<dst->rows * dst->cols; ++i)
	{
		dst->data[i] = data[i];
	}
	return 0;
}

element get_value(mat m, int row, int col)
{
	if(row > 0 && col > 0)
	{
		if(row <= m.rows && col <= m.cols)
		{
			return m.data[(row-1)*m.cols + (col-1)];
		}
		return (element)0.0f;
	}
	return (element)0.0f;
}