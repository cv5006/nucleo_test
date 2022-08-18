/*
 * matrix_calculation.c
 *
 *  Created on: 2022. 6. 3.
 *      Author: keonyoung
 */


#include <stdio.h>
#include <stdlib.h>
#include "matrix_calculation.h"

int IsValidRange(Matrix* mat, int row, int col){
    return (row > 0) && (row <= mat->rows) && (col > 0) && (col <= mat->cols);
}

float *Elem(Matrix* mat, int row, int col){
	return &(mat->data)[row-1][col-1];
}

float GetElement(Matrix *mat, int row, int col){
	return (mat->data)[row-1][col-1];
}

int SetElement(Matrix *mat, int row, int col, float val){
	(mat->data)[row-1][col-1] = val;
	return 1;
}

struct Matrix* MatrixConstructor(int row, int col){
    struct Matrix* mat = (struct Matrix *)malloc(sizeof(struct Matrix));
    mat->rows = row;
    mat->cols = col;
    mat->data = (float **)malloc(sizeof(float *)*row);
    for(int i = 0; i < row; i++){
    	(mat->data)[i] = (float *)malloc(sizeof(float)*col);
    }

    return mat;
}

int MatrixInitializer(Matrix* mat, float* dataList, int listRows, int listCols){

	for(int i = 1; i <= mat->rows; i++){
		for(int j = 1; j <= mat->cols; j++){
			(mat->data)[i-1][j-1] = dataList[listCols*(i-1)+j-1];
		}
	}

    return 1;
}

void MatrixDestructor(Matrix* mat){
    free(mat->data);
    free(mat);
}

int MatrixSum(Matrix* X, Matrix* A, Matrix* B){

    for(int i=1; i<=X->rows; i++){
        for(int j=1; j<=X->cols; j++){
        	(X->data)[i-1][j-1] = (A->data)[i-1][j-1] + (B->data)[i-1][j-1];
        }
    }
    return 1;
}

int MatrixSubstract(Matrix* X, Matrix* A, Matrix*B){

    for(int i=1; i<=X->rows; i++){
        for(int j=1; j<=X->cols; j++){
        	(X->data)[i-1][j-1] = (A->data)[i-1][j-1] - (B->data)[i-1][j-1];
        }
    }
    return 1;
}

int MatrixScalarMultiply(Matrix *X, Matrix* A, Matrix* B){

    for(int i=1; i<=X->rows; i++){
        for(int j=1; j<=X->cols; j++){
        	(X->data)[i-1][j-1] = (A->data)[i-1][j-1] * (B->data)[i-1][j-1];
        }
    }
    return 1;
}

int MatrixScalarMultiply2(Matrix *X, Matrix* A, float k){
	for(int i=1; i<=X->rows; i++){
		for(int j=1; j<=X->cols; j++){
			(X->data)[i-1][j-1] = (A->data)[i-1][j-1]*k;
		}
	}
	return 1;
}

int MatrixMultiply(Matrix* X, Matrix* A, Matrix* B){

    float saveElem;
    for(int i=1; i<=X->rows; i++){
        for(int j=1; j<=X->cols; j++){
        	saveElem = 0;
            for(int k=1; k<=A->cols; k++){
            	saveElem += (A->data)[i-1][k-1] * (B->data)[k-1][j-1];
            }
            (X->data)[i-1][j-1] = saveElem;

        }
    }
    return 1;
}

int Transpose(Matrix* X, Matrix *A){

    for(int i=1; i<=X->rows; i++){
        for(int j=1; j<=X->cols; j++){
        	(X->data)[i-1][j-1] = (A->data)[j-1][i-1];
        }
    }
    return 1;
}

int MatrixTransposeMultiply(Matrix *X, Matrix *A){

    float saveElem;
    for(int i=1; i<=X->rows; i++){
        for(int j=1; j<=X->cols; j++){
            saveElem = 0;
            for(int k=1; k<=A->rows; k++){
                saveElem += (A->data)[k-1][i-1] * (A->data)[k-1][j-1];
            }
            (X->data)[i-1][j-1] = saveElem;
        }
    }
    return 1;
}

int Inverse(Matrix *X, Matrix* A){
//    if(A->rows != 4 || A->cols != 4 || A->rows != X->rows || A->cols != X->cols){ return 0; }
    float **A_data = A->data;
    float a11 = A_data[0][0];
    float a12 = A_data[0][1];
    float a13 = A_data[0][2];
    float a14 = A_data[0][3];
    float a21 = A_data[1][0];
    float a22 = A_data[1][1];
    float a23 = A_data[1][2];
    float a24 = A_data[1][3];
    float a31 = A_data[2][0];
    float a32 = A_data[2][1];
    float a33 = A_data[2][2];
    float a34 = A_data[2][3];
    float a41 = A_data[3][0];
    float a42 = A_data[3][1];
    float a43 = A_data[3][2];
    float a44 = A_data[3][3];

    float A_determinant = a11*(a22*a33*a44 + a23*a34*a42 + a24*a32*a43 - a24*a33*a42 - a23*a32*a44 - a22*a34*a43)
                        - a21*(a12*a33*a44 + a13*a34*a42 + a14*a32*a43 - a14*a33*a42 - a13*a32*a44 - a12*a34*a43)
                        + a31*(a12*a23*a44 + a13*a24*a42 + a14*a22*a43 - a14*a23*a42 - a13*a22*a44 - a12*a24*a43)
                        - a41*(a12*a23*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32 - a13*a22*a34 - a12*a24*a33);

    /* Determinant should be checked by certain bound */
    if(A_determinant < 0.3 && A_determinant > -0.3) return 0;

    (X->data)[0][0] = (a22*a33*a44 + a23*a34*a42 + a24*a32*a43 - a24*a33*a42 - a23*a32*a44 - a22*a34*a43)/A_determinant;
    (X->data)[0][1] = (- a12*a33*a44 - a13*a34*a42 - a14*a32*a43 + a14*a33*a42 + a13*a32*a44 + a12*a34*a43)/A_determinant;
    (X->data)[0][2] = (a12*a23*a44 + a13*a24*a42 + a14*a22*a43 - a14*a23*a42 - a13*a22*a44 - a12*a24*a43)/A_determinant;
    (X->data)[0][3] = (- a12*a23*a34 - a13*a24*a32 - a14*a22*a33 + a14*a23*a32 + a13*a22*a34 + a12*a24*a33)/A_determinant;

    (X->data)[1][0] = (- a12*a33*a44 - a23*a34*a41 - a24*a31*a43 + a24*a33*a41 + a23*a31*a44 + a21*a34*a43)/A_determinant;
    (X->data)[1][1] = (a11*a33*a44 + a13*a34*a41 + a14*a31*a43 - a14*a33*a41 - a13*a31*a44 - a11*a34*a43)/A_determinant;
    (X->data)[1][2] = (- a11*a23*a44 - a13*a24*a41 - a14*a21*a43 + a14*a23*a41 + a13*a21*a44 + a11*a24*a43)/A_determinant;
    (X->data)[1][3] = (a11*a23*a34 + a13*a24*a31 + a14*a21*a33 - a14*a23*a31 - a13*a21*a34 - a11*a24*a33)/A_determinant;

    (X->data)[2][0] = (a21*a32*a44 + a22*a34*a41 + a24*a31*a42 - a24*a32*a41 - a22*a31*a44 - a21*a34*a42)/A_determinant;
    (X->data)[2][1] = (- a11*a32*a44 - a12*a34*a41 - a14*a31*a42 + a14*a32*a41 + a12*a31*a44 + a11*a34*a42)/A_determinant;
    (X->data)[2][2] = (a11*a22*a44 + a12*a24*a41 + a14*a21*a42 - a14*a22*a41 - a12*a21*a44 - a11*a24*a42)/A_determinant;
    (X->data)[2][3] = (- a11*a22*a34 - a12*a24*a31 - a14*a21*a32 + a14*a22*a31 + a12*a21*a34 + a11*a24*a32)/A_determinant;

    (X->data)[3][0] = (- a21*a32*a43 - a22*a33*a41 - a23*a31*a42 + a23*a32*a41 + a22*a31*a43 + a21*a33*a42)/A_determinant;
    (X->data)[3][1] = (a11*a32*a43 + a12*a33*a41 + a13*a31*a42 - a13*a32*a41 - a12*a31*a43 - a11*a33*a42)/A_determinant;
    (X->data)[3][2] = (- a11*a22*a43 - a12*a23*a41 - a13*a21*a42 + a13*a22*a41 + a12*a21*a43 + a11*a23*a42)/A_determinant;
    (X->data)[3][3] = (a11*a22*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31 - a12*a21*a33 - a11*a23*a32)/A_determinant;
    return 1;
}

float Determinant(Matrix *A)
{
	float **A_data = A->data;
	float a11 = A_data[0][0];
	float a12 = A_data[0][1];
	float a13 = A_data[0][2];
	float a14 = A_data[0][3];
	float a21 = A_data[1][0];
	float a22 = A_data[1][1];
	float a23 = A_data[1][2];
	float a24 = A_data[1][3];
	float a31 = A_data[2][0];
	float a32 = A_data[2][1];
	float a33 = A_data[2][2];
	float a34 = A_data[2][3];
	float a41 = A_data[3][0];
	float a42 = A_data[3][1];
	float a43 = A_data[3][2];
	float a44 = A_data[3][3];

	float A_determinant = a11*(a22*a33*a44 + a23*a34*a42 + a24*a32*a43 - a24*a33*a42 - a23*a32*a44 - a22*a34*a43)
						- a21*(a12*a33*a44 + a13*a34*a42 + a14*a32*a43 - a14*a33*a42 - a13*a32*a44 - a12*a34*a43)
						+ a31*(a12*a23*a44 + a13*a24*a42 + a14*a22*a43 - a14*a23*a42 - a13*a22*a44 - a12*a24*a43)
						- a41*(a12*a23*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32 - a13*a22*a34 - a12*a24*a33);
	return A_determinant;
}
