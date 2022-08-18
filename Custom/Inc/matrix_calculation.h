/*
 * matrix_calculation.h
 *
 *  Created on: 2022. 6. 3.
 *      Author: keonyoung
 */

#ifndef INC_MATRIX_CALCULATION_H_
#define INC_MATRIX_CALCULATION_H_

typedef struct Matrix {
    int rows;
    int cols;
    float **data;
}Matrix;

/* returns whether or not matrix indexing is valid. 1 if valid, 0 if invalid*/
int IsValidRange(Matrix* mat, int row, int col);

/* returns matrix element at certain index */
float *Elem(Matrix* mat, int row, int col);

/* getter and setter */
float GetElement(Matrix *mat, int row, int col);
int SetElement(Matrix *mat, int row, int col, float val);

/* construct a matrix and returns its pointer*/
struct Matrix* MatrixConstructor(int row, int col);

/*
 * initialize a constructed matrix and returns success/failure
 * dataList : 2 dimensional array of floats used to initialize matrix
 * listRows : # of rows of dataList
 * listCols : # of cols of dataList
 */
int MatrixInitializer(Matrix* mat, float* dataList, int listRows, int listCols);

/* destruct a matrix */
void MatrixDestructor(Matrix* mat);

/* X = A + B */
int MatrixSum(Matrix* X, Matrix* A, Matrix* B);

/* X = A - B */
int MatrixSubstract(Matrix* X, Matrix* A, Matrix*B);

/* X = A.*B */
int MatrixScalarMultiply(Matrix *X, Matrix* A, Matrix* B);

int MatrixScalarMultiply2(Matrix *X, Matrix* A, float k);

/* X = A * B */
int MatrixMultiply(Matrix* X, Matrix* A, Matrix* B);

/* X = transpose(A) */
int Transpose(Matrix* X, Matrix *A);

/* X = transpose(A)*A */
int MatrixTransposeMultiply(Matrix *X, Matrix *A);

/* X = inverse(A) */
int Inverse(Matrix *X, Matrix* A);

float Determinant(Matrix *A);

#endif /* INC_MATRIX_CALCULATION_H_ */
