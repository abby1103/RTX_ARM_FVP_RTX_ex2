#pragma once
#ifndef __singular_value_decomposition_H
#define __singular_value_decomposition_H

int Singular_Value_Decomposition(double* A, int nrows, int ncols, double* U,
	double* singular_values, double* V, double* dummy_array);
void Singular_Value_Decomposition_Solve(double* U, double* D, double* V,
	double tolerance, int nrows, int ncols, double* B, double* x);
void Singular_Value_Decomposition_Inverse(double* U, double* D, double* V,
	double tolerance, int nrows, int ncols, double* Astar);
int pinv_indexX3(double* Smat, double* ISmat_tem2, int index);

#endif //__singular_value_decomposition_H

