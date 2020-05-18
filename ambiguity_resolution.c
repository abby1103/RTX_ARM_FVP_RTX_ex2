#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "ambiguity_resolution.h"
#include "singular_value_decomposition.h"
#include "position.h"
#include "measure.h"
#include "constants.h"


/*static void output_matrix(double arr[], int n, int m) {
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) printf("%10f ", arr[j + m * i]);
		printf("\n");
	}
	printf("\n");
}
*/

double det_3X3(double A[]) {

	double det_A = 0;
	det_A += A[0] * (A[4] * A[8] - A[5] * A[7]);
	det_A += A[1] * (A[5] * A[6] - A[3] * A[8]);
	det_A += A[2] * (A[3] * A[7] - A[4] * A[6]);

	return det_A;
}

int inverse2X2(double* A, double* IA) {
	double det_A = A[0] * A[3] - A[1] * A[2];
	if (fabs(det_A) < 10e-6) return 1;

	IA[0] = A[3] / det_A;
	IA[1] = -A[1] / det_A;
	IA[2] = -A[2] / det_A;
	IA[3] = A[0] / det_A;

	return 0;

}

// Gauss - Jordan reduced echelon form
void inverse_nXn(double* a_prt, double* ia_prt, int n) {
	// a[n][2*n]

	int i, j, k;
	double a[14][28] = { 0 };
	double  m;

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			a[i][j] = a_prt[i * n + j];
			a[i][n + i] = 1;
		}
	}

	for (i = 0; i < n - 1; i++) {
		for (j = i + 1; j < n; j++) {
			m = a[j][i] / a[i][i];
			for (k = i; k < n * 2; k++)
				a[j][k] -= m * a[i][k];
		}
	}

	for (i = n - 1; i > 0; i--) {
		for (j = i - 1; j >= 0; j--) {
			m = a[j][i] / a[i][i];
			for (k = i; k < n * 2; k++)
				a[j][k] -= m * a[i][k];
		}
	}

	for (i = 0; i < n; i++) {
		m = a[i][i];
		for (j = 0; j < n * 2; j++)
			a[i][j] /= m;
	}

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			ia_prt[i * n + j] = a[i][j + n];
		}
	}
}

static void transpose(double* a, double* at, int m, int n)
{
	int i, j;

	for (i = 0; i < m; ++i)
		for (j = 0; j < n; ++j) {
			at[j * m + i] = a[i * n + j];
		}
}

/* C <- A * B */
static void mulmat(double* a, double* b, double* c, int arows, int acols, int bcols)
{
	int i, j, l;

	for (i = 0; i < arows; ++i)
		for (j = 0; j < bcols; ++j) {
			c[i * bcols + j] = 0;
			for (l = 0; l < acols; ++l)
				c[i * bcols + j] += a[i * acols + l] * b[l * bcols + j];
		}
}

int maxv(int v[], int n) {
	int max = v[0], i;
	for (i = 1; i < n; i++) {
		if (max < v[i]) max = v[i];
	}
	return max;
}

int minv(int v[], int n) {
	int min = v[0], i;
	for (i = 1; i < n; i++) {
		if (min > v[i]) min = v[i];
	}
	return min;
}

/* 快速排序法 */
void quick_sort_struct(meau_model arr[], int first_index, int last_index) {
	// 宣告索引變數
	int pivotIndex, index_a, index_b;
	meau_model temp;

	if (first_index < last_index) {
		// 以第一個元素作為基準
		pivotIndex = first_index;
		index_a = first_index;
		index_b = last_index;

		// 以遞增方式排序
		while (index_a < index_b) {
			while (arr[index_a].range <= arr[pivotIndex].range && index_a < last_index) {
				index_a++;
			}
			while (arr[index_b].range > arr[pivotIndex].range) {
				index_b--;
			}

			if (index_a < index_b) {
				// 交換元素
				temp = arr[index_a];
				arr[index_a] = arr[index_b];
				arr[index_b] = temp;
			}
		}

		// 交換基準元素與 index_b 元素
		temp = arr[pivotIndex];
		arr[pivotIndex] = arr[index_b];
		arr[index_b] = temp;

		// 遞迴呼叫快速排序法函數
		quick_sort_struct(arr, first_index, index_b - 1);
		quick_sort_struct(arr, index_b + 1, last_index);
	}
}

void quick_sort_cands(double costfun[] , double cand_angle[], int first_index, int last_index, int cand_cols) {
	// 宣告索引變數
	int pivotIndex, index_a, index_b, i;
	double temp;

	if (first_index < last_index) {
		// 以第一個元素作為基準
		pivotIndex = first_index;
		index_a = first_index;
		index_b = last_index;

		// 以遞增方式排序
		while (index_a < index_b) {
			while (costfun[index_a] <= costfun[pivotIndex] && index_a < last_index) {
				index_a++;
			}
			while (costfun[index_b] > costfun[pivotIndex]){
				index_b--;
			}

			if (index_a < index_b) {
				// 交換元素
				temp = costfun[index_a];
				costfun[index_a] = costfun[index_b];
				costfun[index_b] = temp;

				for (i = 0; i < cand_cols; i++) {
					temp = cand_angle[index_a * cand_cols + i];
					cand_angle[index_a * cand_cols + i] = cand_angle[index_b * cand_cols + i];
					cand_angle[index_b * cand_cols + i] = temp;
				}

				
			}
		}

		// 交換基準元素與 index_b 元素
		temp = costfun[pivotIndex];
		costfun[pivotIndex] = costfun[index_b];
		costfun[index_b] = temp;

		for (i = 0; i < cand_cols; i++) {
			temp = cand_angle[pivotIndex * cand_cols + i];
			cand_angle[pivotIndex * cand_cols + i] = cand_angle[index_b * cand_cols + i];
			cand_angle[index_b * cand_cols + i] = temp;
		}

		// 遞迴呼叫快速排序法函數
		quick_sort_cands(costfun, cand_angle, first_index, index_b - 1, cand_cols);
		quick_sort_cands(costfun, cand_angle, index_b + 1, last_index, cand_cols);
	}
}


// setting cvariance matrix for single baseline
void cov_matrix1(double QY[], double sdstd, int n) {
	int i, j;

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			if (i == j) QY[i * n + j] = 2 * sdstd * sdstd;
			else QY[i * n + j] = sdstd * sdstd;
		}
	}
}

double* kron(double A[], int sizeA, double B[], int sizeB) {

	int n = sizeA * sizeB;
	double* M;
	int a1, a2, b1, b2;

	M = (double*)malloc(n * n * sizeof(double));
	if (M == NULL) {
		//printf(" No memory available for kron\n");
		return NULL;
	}
	for (a1 = 0; a1 < sizeA; a1++) {
		for (a2 = 0; a2 < sizeA; a2++) {
			for (b1 = 0; b1 < sizeB; b1++) {
				for (b2 = 0; b2 < sizeB; b2++) {
					M[(a1 * n * sizeB + b1 * n) + (a2 * sizeB + b2)] = A[a1 * sizeA + a2] * B[b1 * sizeB + b2];
				}
			}
		}
	}
	return M;
}

void ddcp_model(meau_model ant1_ptr[], meau_model ant2_ptr[], double sdcp[], double sdstd, int n )
{
	double LOS[21] = { 0 };	// 7 * 3
	double enu2ecef[3][3];
	int i;
	double lat, lon;

	// setting S matrix
		// setting light of sight matrix from SVi to ant0
	for (i = 0; i < n; i++) {
		LOS[i * 3 + 0] = (receiver_pvt.x - sat_position[i + 1].x ) / m_rho[i + 1] - (receiver_pvt.x - sat_position[0].x ) / m_rho[0];
		LOS[i * 3 + 1] = (receiver_pvt.y - sat_position[i + 1].y ) / m_rho[i + 1] - (receiver_pvt.y - sat_position[0].y ) / m_rho[0];
		LOS[i * 3 + 2] = (receiver_pvt.z - sat_position[i + 1].z ) / m_rho[i + 1] - (receiver_pvt.z - sat_position[0].z ) / m_rho[0];
	}
		// the end of setting light of sight matrix from SVi to ant0

		// setting coversion matrix of enu to ecef
	lat = receiver_llh.lat; // in rad
	lon = receiver_llh.lon; // in rad

	enu2ecef[0][0] = -sin(lon);
	enu2ecef[1][0] = cos(lon);
	enu2ecef[2][0] = 0;
	enu2ecef[0][1] = -cos(lon) * sin(lat);
	enu2ecef[1][1] = -sin(lon) * sin(lat);
	enu2ecef[2][1] = cos(lat);
	enu2ecef[0][2] = cos(lon) * cos(lat);
	enu2ecef[1][2] = sin(lon) * cos(lat);
	enu2ecef[2][2] = sin(lat);
		// the end of setting coversion matrix of enu to ecef

	for (i = 0; i < n; i++) {
		ant1_ptr[i].S[0] = LOS[i * 3 + 0] * enu2ecef[0][0] + LOS[i * 3 + 1] * enu2ecef[1][0] + LOS[i * 3 + 2] * enu2ecef[2][0];
		ant1_ptr[i].S[1] = LOS[i * 3 + 0] * enu2ecef[0][1] + LOS[i * 3 + 1] * enu2ecef[1][1] + LOS[i * 3 + 2] * enu2ecef[2][1];
		ant1_ptr[i].S[2] = LOS[i * 3 + 0] * enu2ecef[0][2] + LOS[i * 3 + 1] * enu2ecef[1][2] + LOS[i * 3 + 2] * enu2ecef[2][2];
	}

	for (int i = 0; i < n; i++) {
		ant2_ptr[i].S[0] = ant1_ptr[i].S[0];
		ant2_ptr[i].S[1] = ant1_ptr[i].S[1];
		ant2_ptr[i].S[2] = ant1_ptr[i].S[2];
	}
	// the end of setting S matrix 

	// setting ddcp
	for (i = 0; i < n; i++) {
		ant1_ptr[i].ddcp = sdcp[i + 1] - sdcp[0];
		ant2_ptr[i].ddcp = sdcp[i + 1 + (n + 1)] - sdcp[n + 1];
	}
	// the end of setting ddcp

}

void b1_NRange(meau_model ant1_ptr[], meau_model ant2_ptr[], int n, double old_an[], double small_an[]){

	double S_norm[7] = { 0 };
	int up_tem, low_tem;
	int i;

	for (i = 0; i < n; i++) {
		S_norm[i] = sqrt(ant1_ptr[i].S[0] * ant1_ptr[i].S[0] + ant1_ptr[i].S[1] * ant1_ptr[i].S[1] + ant1_ptr[i].S[2] * ant1_ptr[i].S[2]);	// 因 ant1_ptr[i].S = ant2_ptr[i].S[0] , 只要算一次即可
		up_tem = (int)floor(S_norm[i] * length / L1) + 1;
		low_tem = (int)ceil(- S_norm[i] * length / L1) - 1;
		ant1_ptr[i].up = up_tem;
		ant2_ptr[i].up = up_tem;
		ant1_ptr[i].low = low_tem;
		ant2_ptr[i].low = low_tem;
	}

	if (old_an != NULL) {
		double a, b;
		double delta_yaw, cri_piont_yaw, delta_pitch, cri_piont_pitch;
		double upbound_yaw, lowbound_yaw, upbound_pitch, lowbound_pitch;
		double b1_tem[3];
		int N_tem[5], index, max_N, min_N;

		for (i = 0; i < n; i++) {

			// finding out critical point of yaw
			a = ant1_ptr[i].S[1] / ant1_ptr[i].S[0];
			b = tan(old_an[2]);
			delta_yaw = atan((a - b) / (a * b + 1));
			cri_piont_yaw = old_an[2] + delta_yaw;
			// the end of finding out critical point of yaw

			// finding out critical point of pitch
			a = - ant1_ptr[i].S[2] / (ant1_ptr[i].S[0] * cos(cri_piont_yaw) + ant1_ptr[i].S[1] * sin(cri_piont_yaw));
			b = tan(old_an[1]);
			delta_pitch = atan((a - b) / (a * b + 1));
			cri_piont_pitch = old_an[1] + delta_pitch;
			// the end of finding out critical point of yaw


			// caculate bound of N
			if (fabs(delta_pitch) <= small_an[1] && fabs(delta_yaw) <= small_an[2]) {
				index = 5;
				b1_tem[0] = cos(cri_piont_yaw) * cos(cri_piont_pitch);
				b1_tem[1] = sin(cri_piont_yaw) * cos(cri_piont_pitch);
				b1_tem[2] = -sin(cri_piont_pitch);
				N_tem[4] = (int) round((ant1_ptr[i].S[0] * b1_tem[0] + ant1_ptr[i].S[1] * b1_tem[1] + ant1_ptr[i].S[2] * b1_tem[2]) / L1 - ant1_ptr[i].ddcp);
			}
			else {
				index = 4;
			}
			upbound_yaw = old_an[2] + small_an[2];
			lowbound_yaw = old_an[2] - small_an[2];
			upbound_pitch = old_an[1] + small_an[1];
			lowbound_pitch = old_an[1] - small_an[1];

			b1_tem[0] = cos(upbound_yaw) * cos(upbound_pitch);
			b1_tem[1] = sin(upbound_yaw) * cos(upbound_pitch);
			b1_tem[2] = -sin(upbound_pitch);
			N_tem[0] = (int)round((ant1_ptr[i].S[0] * b1_tem[0] + ant1_ptr[i].S[1] * b1_tem[1] + ant1_ptr[i].S[2] * b1_tem[2]) / L1 - ant1_ptr[i].ddcp);

			b1_tem[0] = cos(upbound_yaw) * cos(lowbound_pitch);
			b1_tem[1] = sin(upbound_yaw) * cos(lowbound_pitch);
			b1_tem[2] = -sin(lowbound_pitch);
			N_tem[1] = (int)round((ant1_ptr[i].S[0] * b1_tem[0] + ant1_ptr[i].S[1] * b1_tem[1] + ant1_ptr[i].S[2] * b1_tem[2]) / L1 - ant1_ptr[i].ddcp);

			b1_tem[0] = cos(lowbound_yaw) * cos(upbound_pitch);
			b1_tem[1] = sin(lowbound_yaw) * cos(upbound_pitch);
			b1_tem[2] = -sin(upbound_pitch);
			N_tem[2] = (int)round((ant1_ptr[i].S[0] * b1_tem[0] + ant1_ptr[i].S[1] * b1_tem[1] + ant1_ptr[i].S[2] * b1_tem[2]) / L1 - ant1_ptr[i].ddcp);

			b1_tem[0] = cos(lowbound_yaw) * cos(lowbound_pitch);
			b1_tem[1] = sin(lowbound_yaw) * cos(lowbound_pitch);
			b1_tem[2] = -sin(lowbound_pitch);
			N_tem[3] = (int)round((ant1_ptr[i].S[0] * b1_tem[0] + ant1_ptr[i].S[1] * b1_tem[1] + ant1_ptr[i].S[2] * b1_tem[2]) / L1 - ant1_ptr[i].ddcp);

			max_N = maxv(N_tem, index);
			min_N = minv(N_tem, index);

			if (ant1_ptr[i].up > max_N && max_N >= ant1_ptr[i].low)
				ant1_ptr[i].up = max_N;
			if (ant1_ptr[i].up >= min_N && min_N > ant1_ptr[i].low)
				ant1_ptr[i].low = min_N;
			ant1_ptr[i].range = ant1_ptr[i].up - ant1_ptr[i].low;
			// the end of caculate bound of N
		
		}
	}
	quick_sort_struct(ant1_ptr, 0, n - 1);
}

int std_baseline(meau_model ant_ptr[], error_std* ant_std, int n, double* QY_nsv, int ratio,
			double* Smat, double* pinvSmat, double* pinvSmat_tran, double* pinvSmat_tem, int index_b) {

	double ISmat3X3_tran[9];
	double Smat3X3[9];
	double M[49] = { 0 }; // 7 * 7
	double s_tem;
	int err;
	int i, j;

	// setting S matrix
	for (i = 0; i < n; i++) {
		for (j = 0; j < 3; j++) {
			Smat[i * 3 + j] = ant_ptr[i].S[j];
		}
	}
	// the end of setting S matrix
	//--------------------------------------------------------------//
	// caculate the std of the error of baseline for 4SV
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			Smat3X3[i * 3 + j] = ant_ptr[i].S[j];
		}
	}

	// doing inverse of S(1:3,1:3)
	inverse_nXn(Smat3X3, pinvSmat_tem, 3);
	transpose(pinvSmat_tem, ISmat3X3_tran, 3, 3);
	mulmat(ISmat3X3_tran, pinvSmat_tem, M, 3, 3, 3);

	s_tem = 0;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			s_tem = s_tem + QY_nsv[i * n + j] * M[i * 3 + j];
		}
	}
	if (index_b == 1) {
		ant_std->var_b1_4sv = s_tem;
		ant_std->sigma_b1_4sv = ratio * sqrt(L1 * L1 * s_tem);
	}
	else {
		ant_std->var_b2_4sv = s_tem;
		ant_std->sigma_b2_4sv = ratio * sqrt(L1 * L1 * s_tem);
	}
	// the end of caculate the std of the error of baseline for 4SV
	//--------------------------------------------------------------//

	if (n > 3) {
		//--------------------------------------------------------------//
		// caculate the std of the error of baseline for nSV
		// doing inverse of S(1:index,1:3)
		err = pinv_indexX3(Smat, pinvSmat, n);
		if (err != 0) return 1;
		// the end of doing inverse of S(1:index,1:3)

		transpose(pinvSmat, pinvSmat_tran, 3, n);
		mulmat(pinvSmat_tran, pinvSmat, M, n, 3, n);
		s_tem = 0;
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++) {
				s_tem = s_tem + QY_nsv[i * n + j] * M[i * n + j];
			}
		}
		if (index_b == 1) {
			ant_std->var_b1_nsv = s_tem;
			ant_std->sigma_b1_nsv = ratio * sqrt(L1 * L1 * s_tem);
		}
		else {
			ant_std->var_b2_nsv = s_tem;
			ant_std->sigma_b2_nsv = ratio * sqrt(L1 * L1 * s_tem);
		}
		// the end of caculate the std of the error of baseline for nSV
		//--------------------------------------------------------------//
	}

	return 0;
}

void std_b1b2(meau_model ant2_ptr[], error_std* ant_std, int n, int ratio,
	double* pinvS1mat, double* pinvS2mat_tran, double* pinvS2mat_tem) {

	double M[49] = { 0 };	//7 * 7
	double IS2mat3X3_tran[9];
	double s_tem = 0;
	int i, j;

	transpose(pinvS2mat_tem, IS2mat3X3_tran, 3, 3);
	mulmat(IS2mat3X3_tran, pinvS1mat, M, 3, 3, n);
	for (i = 0; i < 3; i++) {
		for (j = 0; j < n; j++) {
			s_tem += ant2_ptr[i].QY_pnt[j] * M[i * n + j];
		}
	}

	ant_std->sigma_b1b2_4sv = ratio * sqrt(ant_std->var_b2_4sv - 2 * s_tem + ant_std->var_b1_nsv) * L1;

	if (n > 3) {

		mulmat(pinvS2mat_tran, pinvS1mat, M, n, 3, n);
		s_tem = 0;
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++) {
				s_tem += ant2_ptr[i].QY_pnt[j] * M[i * n + j];
			}
		}

		ant_std->sigma_b1b2_nsv = ratio * sqrt(ant_std->var_b2_nsv - 2 * s_tem + ant_std->var_b1_nsv) * L1;
	}

}


int GSO(meau_model ant_ptr[], double sigma_b, int n, cand_list cand_b[]) {

	double v[9], a[9];
	double c2_bar;
	int n0, n1, n2, n2_tem[4];
	double c0, c1, c2;
	double c2_square_low, c2_square_up, c2_low, c2_up;
	double temp1, temp2, error;
	int num;
	int i;

	// 預設10組候選人
	int maxnum = 10;
	int* prt_tem0;
	double* prt_tem1, * prt_tem2;
	cand_b[0].Ncands = (int*)malloc(3 * maxnum * sizeof(int));	//因為先針對前4個衛星,每組有'3'個N
	cand_b[0].bcands = (double*)malloc(3 * maxnum * sizeof(double));	//每組有'3'個分量(bx,by,bz)
	cand_b[0].goodness = (double*)malloc(maxnum * sizeof(double));
	if (cand_b[0].Ncands == NULL || cand_b[0].bcands == NULL || cand_b[0].goodness == NULL) {
		//printf(" No memory available for cand_list in GSO\n");
		return 1;
	}
	cand_b[0].numofcand = 0;

	// Create a new three-axis coordinate
	// v(0:2) = S(0,0:2)
	v[0] = ant_ptr[0].S[0];		
	v[1] = ant_ptr[0].S[1];
	v[2] = ant_ptr[0].S[2];
	// a0 = v(0:2)' * v(0:2) 
	a[0] = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];

	// a[3*1+0] = S(1,0:2) * v(0:2) /  a0
	a[3] = (ant_ptr[1].S[0] * v[0] + ant_ptr[1].S[1] * v[1] + ant_ptr[1].S[2] * v[2]) / a[0];
	// v(3*1+0 : 3*1+2) = S(1,0:2) - a[3*1+0] * v(0:2);
	v[3] = ant_ptr[1].S[0] - a[3] * v[0];
	v[4] = ant_ptr[1].S[1] - a[3] * v[1];
	v[5] = ant_ptr[1].S[2] - a[3] * v[2];
	// a[3*1+1] = v(3:5)' * v(3:5) 
	a[4] = v[3] * v[3] + v[4] * v[4] + v[5] * v[5];

	// a[3*2+0] = S(2,0:2) * v(0:2) /  a0
	a[6] = (ant_ptr[2].S[0] * v[0] + ant_ptr[2].S[1] * v[1] + ant_ptr[2].S[2] * v[2]) / a[0];
	// a[3*1+0] = S(2,0:2) * v(3:5) /  a[3*1+1]
	a[7] = (ant_ptr[2].S[0] * v[3] + ant_ptr[2].S[1] * v[4] + ant_ptr[2].S[2] * v[5]) / a[4];
	// v(3*2+0 : 3*2+2) = S(2,0:2) - a[3*2+0] * v(0:2) - a[3*2+1] * v(3:5)
	v[6] = ant_ptr[2].S[0] - a[6] * v[0] - a[7] * v[3];
	v[7] = ant_ptr[2].S[1] - a[6] * v[1] - a[7] * v[4];
	v[8] = ant_ptr[2].S[2] - a[6] * v[2] - a[7] * v[5];
	// a[3*2+2] = v(6:8)' * v(6:8) 
	a[8] = v[6] * v[6] + v[7] * v[7] + v[8] * v[8];

	// a[1] = a[3*2+0] - a[3*2+1] * a[3*1+0]
	a[1] = a[6] - a[7] * a[3];
	// the end of Create a new three-axis coordinate

	// search n
	num = -1;
	c2_bar = ant_ptr[2].ddcp * L1 - a[7] * ant_ptr[1].ddcp * L1 - a[1] * ant_ptr[0].ddcp * L1;
	for (n0 = ant_ptr[0].low; n0 <= ant_ptr[0].up; n0++) {
		for (n1 = ant_ptr[1].low; n1 <= ant_ptr[1].up; n1++) {
			c0 = (ant_ptr[0].ddcp + n0) * L1;
			c1 = (ant_ptr[1].ddcp + n1) * L1 - a[3] * c0;

			c2_square_low = a[8] * ((length - sigma_b) * (length - sigma_b) - c0 * c0 / a[0] - c1 * c1 / a[4]);
			c2_square_up = a[8] * ((length + sigma_b) * (length + sigma_b) - c0 * c0 / a[0] - c1 * c1 / a[4]);

			if (c2_square_low < 0) c2_low = 0;			
			else c2_low = sqrt(c2_square_low);
			if (c2_square_up < 0) c2_up = 0;
			else c2_up = sqrt(c2_square_up);

			temp1 = -c2_bar / L1 + a[7] * n1 + a[1] * n0;
			n2_tem[0] = (int)round(-c2_up / L1 + temp1);
			n2_tem[1] = (int)round(-c2_low / L1 + temp1);
			n2_tem[2] = (int)round(c2_low / L1 + temp1);
			n2_tem[3] = (int)round(c2_up / L1 + temp1);

			for (i = 0; i < 4; i++) {
				if (n2_tem[i] < ant_ptr[2].low) n2_tem[i] = ant_ptr[2].low;
				if (n2_tem[i] > ant_ptr[2].up) n2_tem[i] = ant_ptr[2].up;
			}

			temp2 =(ant_ptr[2].ddcp - a[7] * (ant_ptr[1].ddcp + n1) - a[1] * (ant_ptr[0].ddcp + n0)) * L1;
			for (n2 = n2_tem[0]; n2 <= n2_tem[1]; n2++) {
				c2 = n2 * L1 + temp2;
				error = fabs(sqrt(c0 * c0 / a[0] + c1 * c1 / a[4] + c2 * c2 / a[8]) - length);
				if (error <= sigma_b) {

					num++;
					if ((num + 1) > maxnum) {
						// 每次增加10組候選人位子
						maxnum += 10;
						prt_tem0 = (int*)realloc(cand_b[0].Ncands, 3 * maxnum * sizeof(int));	//因為先針對前4個衛星,每組有'3'個N
						prt_tem1 = (double*)realloc(cand_b[0].bcands, 3 * maxnum * sizeof(double));	//每組有'3'個分量(bx,by,bz)
						prt_tem2 = (double*)realloc(cand_b[0].goodness, maxnum * sizeof(double));
						if (prt_tem0 == NULL || prt_tem1 == NULL || prt_tem2 == NULL) {
							//printf(" No memory available for cand_list in GSO\n");
							return 1;
						}
						else {
							cand_b[0].Ncands = prt_tem0;
							cand_b[0].bcands = prt_tem1;
							cand_b[0].goodness = prt_tem2;

						}
					}

					cand_b[0].bcands[num * 3 + 0] = c0 * v[0] / a[0] + c1 * v[3] / a[4] + c2 * v[6] / a[8];
					cand_b[0].bcands[num * 3 + 1] = c0 * v[1] / a[0] + c1 * v[4] / a[4] + c2 * v[7] / a[8];
					cand_b[0].bcands[num * 3 + 2] = c0 * v[2] / a[0] + c1 * v[5] / a[4] + c2 * v[8] / a[8];

					cand_b[0].Ncands[num * 3 + 0] = n0;
					cand_b[0].Ncands[num * 3 + 1] = n1;
					cand_b[0].Ncands[num * 3 + 2] = n2;

					cand_b[0].goodness[num] = error;
				}
			}

			if (n2_tem[2] == n2_tem[1]) n2_tem[2]++;

			for (n2 = n2_tem[2]; n2 <= n2_tem[3]; n2++) {
				c2 = n2 * L1 + temp2;
				error = fabs(sqrt(c0 * c0 / a[0] + c1 * c1 / a[4] + c2 * c2 / a[8]) - length);
				if (error <= sigma_b) {

					num++;
					if ((num + 1) > maxnum) {
						// 每次增加10組候選人位子
						maxnum += 10;
						prt_tem0 = (int*)realloc(cand_b[0].Ncands, 3 * maxnum * sizeof(int));	//因為先針對前4個衛星,每組有'3'個N
						prt_tem1 = (double*)realloc(cand_b[0].bcands, 3 * maxnum * sizeof(double));	//每組有'3'個分量(bx,by,bz)
						prt_tem2 = (double*)realloc(cand_b[0].goodness, maxnum * sizeof(double));
						if (prt_tem0 == NULL || prt_tem1 == NULL || prt_tem2 == NULL) {
							//printf(" No memory available for cand_list in GSO\n");
							return 1;
						}
						else {
							cand_b[0].Ncands = prt_tem0;
							cand_b[0].bcands = prt_tem1;
							cand_b[0].goodness = prt_tem2;

						}
					}

					cand_b[0].bcands[num * 3 + 0] = c0 * v[0] / a[0] + c1 * v[3] / a[4] + c2 * v[6] / a[8];
					cand_b[0].bcands[num * 3 + 1] = c0 * v[1] / a[0] + c1 * v[4] / a[4] + c2 * v[7] / a[8];
					cand_b[0].bcands[num * 3 + 2] = c0 * v[2] / a[0] + c1 * v[5] / a[4] + c2 * v[8] / a[8];

					cand_b[0].Ncands[num * 3 + 0] = n0;
					cand_b[0].Ncands[num * 3 + 1] = n1;
					cand_b[0].Ncands[num * 3 + 2] = n2;

					cand_b[0].goodness[num] = error;
				}
			}
		}
	}
	if ((num + 1) < maxnum && num >= 0) {
		// 去除多餘的候選人位子
		prt_tem0 = (int*)realloc(cand_b[0].Ncands, 3 * (num + 1) * sizeof(int));	//因為先針對前4個衛星,每組有'3'個N
		prt_tem1 = (double*)realloc(cand_b[0].bcands, 3 * (num + 1) * sizeof(double));	//每組有'3'個分量(bx,by,bz)
		prt_tem2 = (double*)realloc(cand_b[0].goodness, (num + 1) * sizeof(double));
		if (prt_tem0 == NULL || prt_tem1 == NULL || prt_tem2 == NULL) {
			//printf(" No memory available for cand_list in GSO\n");
			return 1;
		}
		else {
			cand_b[0].Ncands = prt_tem0;
			cand_b[0].bcands = prt_tem1;
			cand_b[0].goodness = prt_tem2;

		}
	}
	cand_b[0].numofcand = num + 1;

	return 0;
}


int search_moreN(meau_model ant_ptr[], double sigma_b, cand_list cand_b_pre, cand_list* cand_b_new,
		double* Smat, double* ISmat, double* ISmat_tem1, double* QY1_nsv, int n,int index, int ratio) {

	double s_index[3] = { Smat[(index - 1) * 3 + 0], Smat[(index - 1) * 3 + 1],  Smat[(index - 1) * 3 + 2] };
	double sigma_n4;
	double var_n4_prt[1] = {0};
	int n4_low, n4_up, n4;
	int num = -1, maxnum = 10;
	double temp3, b_tem[3], l_est, error_l;

	int i, j, k;
	double err;

	// ptn_tem for realloc
	int* prt_tem0;
	double* prt_tem1, * prt_tem2;

	double alpha[6] = { 0 };	// 7 - 1
	double temp1[7] = { 0 }, temp2[7] = { 0 };	// 7
	
	// alpha = Sindex * pinv(S(0:(index-1),:))
	mulmat(s_index, ISmat_tem1, alpha, 1, 3, index - 1);

	// var_n4 = [alpha -1] * QY1(1:index,1:indexx) * [alpha -1]'
	for (i = 0; i < (index - 1); i++) {
		temp1[i] = alpha[i];
	}
	temp1[index - 1] = -1;
	mulmat(temp1, QY1_nsv, temp2, 1, index, index);
	mulmat(temp2, temp1, var_n4_prt, 1, index, 1);
	sigma_n4 = ratio * sqrt(var_n4_prt[0]);

	cand_b_new->Ncands = (int*)malloc(index * maxnum * sizeof(int));	//因為先針對前4個衛星,每組有'3'個N
	cand_b_new->bcands = (double*)malloc(3 * maxnum * sizeof(double));	//每組有'3'個分量(bx,by,bz)
	cand_b_new->goodness = (double*)malloc(maxnum * sizeof(double));
	if (cand_b_new->Ncands == NULL || cand_b_new->bcands == NULL || cand_b_new->goodness == NULL) {
		//printf(" No memory available for cand_list in search_moreN\n");
		return 1;
	}
	cand_b_new->numofcand = 0;

	if (index != n) {
		err = pinv_indexX3(Smat, ISmat_tem1, index);
	}
	else {
		err = 0;
		ISmat_tem1 = ISmat;
	}

	if (err != 0) return 1;

	for (i = 0; i < cand_b_pre.numofcand; i++) {
		for (j = 0; j < (index - 1); j++) {
			temp1[j] = ant_ptr[j].ddcp + cand_b_pre.Ncands[i * (index - 1) + j];
			temp2[j] = temp1[j] * L1;
		}
		mulmat(alpha, temp1, &temp3, 1, index - 1, 1);
		n4_low = (int)round(temp3 - ant_ptr[index - 1].ddcp - sigma_n4);
		n4_up = (int)round(temp3 - ant_ptr[index - 1].ddcp + sigma_n4);
		if (n4_low < ant_ptr[index - 1].low) n4_low = ant_ptr[index - 1].low;
		if (n4_up > ant_ptr[index - 1].up) n4_up = ant_ptr[index - 1].up;

		for (n4 = n4_low; n4 <= n4_up; n4++) {
			temp2[index - 1] = (ant_ptr[index - 1].ddcp + n4) * L1;
			mulmat(ISmat_tem1, temp2, b_tem, 3, index, 1);
			l_est = sqrt(b_tem[0] * b_tem[0] + b_tem[1] * b_tem[1] + b_tem[2] * b_tem[2]);
			error_l = fabs(l_est - length);
			if (error_l <= sigma_b) {
				num++;
				if ((num + 1) > maxnum) {
					maxnum += 10;
					prt_tem0 = (int*)realloc(cand_b_new->Ncands, index * maxnum * sizeof(int));	//因為針對前index+1個衛星,每組有'index'個N
					prt_tem1 = (double*)realloc(cand_b_new->bcands, 3 * maxnum * sizeof(double));	//每組有'3'個分量(bx,by,bz)
					prt_tem2 = (double*)realloc(cand_b_new->goodness, maxnum * sizeof(double));
					if (prt_tem0 == NULL || prt_tem1 == NULL || prt_tem2 == NULL) {
						//printf(" No memory available for cand_list in search_moreN\n");
						return 1;
					}
					else {
						cand_b_new->Ncands = prt_tem0;
						cand_b_new->bcands = prt_tem1;
						cand_b_new->goodness = prt_tem2;

					}
				}

				cand_b_new->bcands[num * 3 + 0] = b_tem[0];
				cand_b_new->bcands[num * 3 + 1] = b_tem[1];
				cand_b_new->bcands[num * 3 + 2] = b_tem[2];
				cand_b_new->goodness[num] = error_l;

				for (k = 0; k < (index - 1); k++) {
					cand_b_new->Ncands[num * index + k] = cand_b_pre.Ncands[i * (index - 1) + k];
				}
				cand_b_new->Ncands[num * index + (index - 1)] = n4;
			}
		}
	}
	cand_b_new->numofcand = num + 1;

	if ((num + 1) < maxnum && num >= 0) {
		// 去除多餘的候選人位子
		prt_tem0 = (int*)realloc(cand_b_new->Ncands, index * (num + 1) * sizeof(int));	//因為先針對前4個衛星,每組有'3'個N
		prt_tem1 = (double*)realloc(cand_b_new->bcands, 3 * (num + 1) * sizeof(double));	//每組有'3'個分量(bx,by,bz)
		prt_tem2 = (double*)realloc(cand_b_new->goodness, (num + 1) * sizeof(double));
		if (prt_tem0 == NULL || prt_tem1 == NULL || prt_tem2 == NULL) {
			//printf(" No memory available for cand_list in search_moreN\n");
			return 1;
		}
		else {
			cand_b_new->Ncands = prt_tem0;
			cand_b_new->bcands = prt_tem1;
			cand_b_new->goodness = prt_tem2;

		}
	}

	return 0;
}

int b2_NRrange(meau_model ant2_ptr[], meau_model ant2_ptr_tem[], int n, double old_an[], double b1[]) {

	int i;
	double angle_sv_plane, length_on_sv;
	double dot_b1_b1, norm_b1, norm_sv, dot_sv_b1;
	int up_tem, low_tem;

	double cos_roll, sin_roll, cos_pitch, sin_pitch, cos_yaw, sin_yaw;
	double b2_old[3], v[3], p[3];
	double A[4], IA[4];
	int err;

	double a, b, c, check, t1, t2;
	double b2_bound1[3], b2_bound2[3];

	int N_tem[3], index = 2;
	double temp, check_an, p_sv[3], norm_p_sv;


	if (old_an == NULL) {
		for (i = 0; i < n; i++) {

			dot_sv_b1 = ant2_ptr_tem[i].S[0] * b1[0] + ant2_ptr_tem[i].S[1] * b1[1] + ant2_ptr_tem[i].S[2] * b1[2];
			norm_sv = sqrt(ant2_ptr_tem[i].S[0] * ant2_ptr_tem[i].S[0] + ant2_ptr_tem[i].S[1] * ant2_ptr_tem[i].S[1] + ant2_ptr_tem[i].S[2] * ant2_ptr_tem[i].S[2]);
			norm_b1 = sqrt(b1[0] * b1[0] + b1[1] * b1[1] + b1[2] * b1[2]);
			angle_sv_plane = PI / 2 - acos(dot_sv_b1 / (norm_b1 * norm_sv));
			length_on_sv = floor(fabs(norm_sv * length * cos(angle_sv_plane) / L1));

			up_tem = (int)length_on_sv + 1;
			low_tem = -(int)length_on_sv - 1;

			if (ant2_ptr[i].up > up_tem) {
				if (up_tem >= ant2_ptr[i].low)
					ant2_ptr_tem[i].up = up_tem;
				else
					return 1;
			}
			else
				ant2_ptr_tem[i].up = ant2_ptr[i].up;

			if (low_tem > ant2_ptr[i].low) {
				if (ant2_ptr[i].up >= low_tem)
					ant2_ptr_tem[i].low = low_tem;
				else
					return 1;
			}
			else
				ant2_ptr_tem[i].low = ant2_ptr[i].low;
			ant2_ptr_tem[i].range = ant2_ptr_tem[i].up - ant2_ptr_tem[i].low;
		}
	}
	else {
		cos_roll = cos(old_an[0]);	sin_roll = sin(old_an[0]);
		cos_pitch = cos(old_an[1]); sin_pitch = sin(old_an[1]);
		cos_yaw = cos(old_an[2]);	sin_yaw = sin(old_an[2]);

		//b2 = R_old * [0 1 0]'
		b2_old[0] = cos_yaw * sin_pitch * sin_roll - cos_roll * sin_yaw;
		b2_old[1] = cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll;
		b2_old[2] = cos_pitch * sin_roll;

		// v = cross(b1,b2_old) : the vector of the intersection of two planes
		v[0] = b1[1] * b2_old[2] - b2_old[1] * b1[2];
		v[1] = b1[2] * b2_old[0] - b2_old[2] * b1[0];
		v[2] = b1[0] * b2_old[1] - b2_old[0] * b1[1];

		// finding a point on the intersection of the two planes
		if (v[2] == 0) {
			if (v[1] == 0) {
				p[0] = 0;
				A[0] = b1[1];	A[1] = b1[2];
				A[2] = b2_old[1];		A[3] = b2_old[2];
				err = inverse2X2(A, IA);
				if (err != 0) return 1;
				p[1] = IA[1] * cos(0.4134); // 0.4134 = 23.6982 deg * PI/180
				p[2] = IA[3] * cos(0.4134);
			}
			else {
				p[1] = 0;
				A[0] = b1[0];	A[1] = b1[2];
				A[2] = b2_old[0];		A[3] = b2_old[2];
				err = inverse2X2(A, IA);
				if (err != 0) return 1;
				p[0] = IA[1] * cos(0.4134); // 0.4134 = 23.6982 deg * PI/180
				p[2] = IA[3] * cos(0.4134);
			}
		}
		else {
			p[2] = 0;
			A[0] = b1[0];	A[1] = b1[1];
			A[2] = b2_old[0];		A[3] = b2_old[1];
			err = inverse2X2(A, IA);
			if (err != 0) return 1;
			p[0] = IA[1] * cos(0.4134); // 0.4134 = 23.6982 deg * PI/180
			p[1] = IA[3] * cos(0.4134);
		}
		// the end of finding a point on the intersection of the two planes

		// finding two boundaries of the b2
		a = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
		b = 2 * (v[0] * p[0] + v[1] * p[1] + v[2] * p[2]);
		c = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]) - 1;
		check = b * b - 4 * a * c;
		if (check <= 0) return 1;
		
		t1 = (-b - sqrt(check)) / (2 * a);
		t2 = (-b + sqrt(check)) / (2 * a);
		b2_bound1[0] = p[0] + v[0] * t1;
		b2_bound1[1] = p[1] + v[1] * t1;
		b2_bound1[2] = p[2] + v[2] * t1;

		b2_bound2[0] = p[0] + v[0] * t2;
		b2_bound2[1] = p[1] + v[1] * t2;
		b2_bound2[2] = p[2] + v[2] * t2;
		// the end of finding two boundaries of the b2

		for (i = 0; i < n; i++) {

			// finding the boundary value of N[i]
			N_tem[0] = (int)round((ant2_ptr_tem[i].S[0] * b2_bound1[0] + ant2_ptr_tem[i].S[1] * b2_bound1[1]
				+ ant2_ptr_tem[i].S[2] * b2_bound1[2]) / L1 - ant2_ptr_tem[i].ddcp);
			N_tem[1] = (int)round((ant2_ptr_tem[i].S[0] * b2_bound2[0] + ant2_ptr_tem[i].S[1] * b2_bound2[1]
				+ ant2_ptr_tem[i].S[2] * b2_bound2[2]) / L1 - ant2_ptr_tem[i].ddcp);
			// the end of finding the boundary value of N[i]

			// finding the relative extremum of N[i] 
			dot_sv_b1 = ant2_ptr_tem[i].S[0] * b1[0] + ant2_ptr_tem[i].S[1] * b1[1] + ant2_ptr_tem[i].S[2] * b1[2];
			dot_b1_b1 = b1[0] * b1[0] + b1[1] * b1[1] + b1[2] * b1[2];
			temp = dot_sv_b1 / dot_b1_b1;

			p_sv[0] = ant2_ptr_tem[i].S[0] - b1[0] * temp;
			p_sv[1] = ant2_ptr_tem[i].S[1] - b1[1] * temp;
			p_sv[2] = ant2_ptr_tem[i].S[2] - b1[2] * temp;
			norm_p_sv = sqrt(p_sv[0] * p_sv[0] + p_sv[1] * p_sv[1] + p_sv[2] * p_sv[2]);
			p_sv[0] /= norm_p_sv;
			p_sv[1] /= norm_p_sv;
			p_sv[2] /= norm_p_sv;

			check_an = acos(p_sv[0] * b2_old[0] + p_sv[1] * b2_old[1] + p_sv[2] * b2_old[2]);
			if (check_an <= 0.4134) {
				N_tem[2] = (int)round((ant2_ptr_tem[i].S[0] * p_sv[0] + ant2_ptr_tem[i].S[1] * p_sv[1]
					+ ant2_ptr_tem[i].S[2] * p_sv[2]) / L1 - ant2_ptr_tem[i].ddcp);
				index = 3;
			}
			else {
				dot_sv_b1 = - (ant2_ptr_tem[i].S[0] * b1[0] + ant2_ptr_tem[i].S[1] * b1[1] + ant2_ptr_tem[i].S[2] * b1[2]);
				temp = dot_sv_b1 / dot_b1_b1;
				p_sv[0] = - ant2_ptr_tem[i].S[0] - b1[0] * temp;
				p_sv[1] = - ant2_ptr_tem[i].S[1] - b1[1] * temp;
				p_sv[2] = - ant2_ptr_tem[i].S[2] - b1[2] * temp;
				norm_p_sv = sqrt(p_sv[0] * p_sv[0] + p_sv[1] * p_sv[1] + p_sv[2] * p_sv[2]);
				p_sv[0] /= norm_p_sv;
				p_sv[1] /= norm_p_sv;
				p_sv[2] /= norm_p_sv;
				
				check_an = acos(p_sv[0] * b2_old[0] + p_sv[1] * b2_old[1] + p_sv[2] * b2_old[2]);
				if (check_an <= 0.4134) {
					N_tem[2] = (int)round((ant2_ptr_tem[i].S[0] * p_sv[0] + ant2_ptr_tem[i].S[1] * p_sv[1]
						+ ant2_ptr_tem[i].S[2] * p_sv[2]) / L1 - ant2_ptr_tem[i].ddcp);
					index = 3;
				}
				else
					index = 2;
			}
			// the end of finding the relative extremum of N[i] 

			// Determining the upperand lower bounds of N[i]
			up_tem = maxv(N_tem, index);
			low_tem = minv(N_tem, index);

			if (ant2_ptr[i].up > up_tem) {
				if (up_tem >= ant2_ptr[i].low)
					ant2_ptr_tem[i].up = up_tem;
				else
					return 1;
			}
			else
				ant2_ptr_tem[i].up = ant2_ptr[i].up;

			if (low_tem > ant2_ptr[i].low) {
				if (ant2_ptr[i].up >= low_tem)
					ant2_ptr_tem[i].low = low_tem;
				else
					return 1;
			}
			else
				ant2_ptr_tem[i].low = ant2_ptr[i].low;
			ant2_ptr_tem[i].range = ant2_ptr_tem[i].up - ant2_ptr_tem[i].low;
			// the end of determining the upperand lower bounds of N[i]
		}
	}
	quick_sort_struct(ant2_ptr_tem, 0, n - 1);
	return 0;
}

int pairing(cand_list cand_b2, cand_list cand_b2_paired[], double b1[], double sigma_b1b2_4sv, int index) {
	// Pairing b1 and b2
	double delta_length, delta_b[3], error_delta_length;
	int* prt_tem0;
	double* prt_tem1, * prt_tem2;
	int numPair_b1b2_4sv = 0;
	int i, j;

	cand_b2_paired[0].Ncands = (int*)malloc(index * cand_b2.numofcand * sizeof(int));	// b2 的前 3 個 N
	cand_b2_paired[0].bcands = (double*)malloc(3 * cand_b2.numofcand * sizeof(double));
	cand_b2_paired[0].goodness = (double*)malloc(cand_b2.numofcand * sizeof(double));
	if (cand_b2_paired[0].Ncands == NULL || cand_b2_paired[0].bcands == NULL || cand_b2_paired[0].goodness == NULL) {
		//printf(" No memory available for cand_b2_paired\n");
		return 1;
	}
	cand_b2_paired[0].numofcand = 0;

	for (i = 0; i < cand_b2.numofcand; i++) {
		delta_b[0] = b1[0] - cand_b2.bcands[i * 3 + 0];
		delta_b[1] = b1[1] - cand_b2.bcands[i * 3 + 1];
		delta_b[2] = b1[2] - cand_b2.bcands[i * 3 + 2];
		delta_length = sqrt(delta_b[0] * delta_b[0] + delta_b[1] * delta_b[1] + delta_b[2] * delta_b[2]);
		error_delta_length = fabs(delta_length - ideal_delta_l);

		if (error_delta_length <= sigma_b1b2_4sv) {

			for (j = 0; j < index; j++) {
				cand_b2_paired[0].Ncands[numPair_b1b2_4sv * index + j] = cand_b2.Ncands[i * index + j];
			}

			cand_b2_paired[0].bcands[numPair_b1b2_4sv * 3 + 0] = cand_b2.bcands[i * 3 + 0];
			cand_b2_paired[0].bcands[numPair_b1b2_4sv * 3 + 1] = cand_b2.bcands[i * 3 + 1];
			cand_b2_paired[0].bcands[numPair_b1b2_4sv * 3 + 2] = cand_b2.bcands[i * 3 + 2];

			cand_b2_paired[0].goodness[numPair_b1b2_4sv] = cand_b2.goodness[i] + error_delta_length;

			numPair_b1b2_4sv++;
		}
	}
	cand_b2_paired[0].numofcand = numPair_b1b2_4sv;
	if (numPair_b1b2_4sv == 0) {
		free(cand_b2_paired[0].Ncands);
		free(cand_b2_paired[0].bcands);
		free(cand_b2_paired[0].goodness);
	}
	else if (numPair_b1b2_4sv < cand_b2.numofcand) {
		// 去除多餘的候選人位子
		prt_tem0 = (int*)realloc(cand_b2_paired[0].Ncands, index * numPair_b1b2_4sv * sizeof(int));	//因為先針對前4個衛星,每組有'3'個N
		prt_tem1 = (double*)realloc(cand_b2_paired[0].bcands, 3 * numPair_b1b2_4sv * sizeof(double));	//每組有'3'個分量(bx,by,bz)
		prt_tem2 = (double*)realloc(cand_b2_paired[0].goodness, numPair_b1b2_4sv * sizeof(double));
		if (prt_tem0 == NULL || prt_tem1 == NULL || prt_tem2 == NULL) {
			//printf(" No memory available for cand_list in cand_b2_paired\n");
			return 1;
		}
		else {
			cand_b2_paired[0].Ncands = prt_tem0;
			cand_b2_paired[0].bcands = prt_tem1;
			cand_b2_paired[0].goodness = prt_tem2;
		}
	}

	free(cand_b2.Ncands);
	free(cand_b2.bcands);
	free(cand_b2.goodness);
	// the end of pairing b1 and b2
	return 0;
}

int rotation_matrix(double b1[], double b2[], double R[]) {

	double B[9] = {0}, det_U, det_V, d;
	int i, j;

	double V[3][3], V_arr[9];
	double V_tran[3][3], V_tran_arr[9];
	double U[3][3], U_arr[9];
	double singular_values[3];
	double dummy_array[3];
	int err;

	for (i = 0; i < 3; i++) {
		B[i * 3 + 0] = b1[i] / 2;
		B[i * 3 + 1] = b2[i] / 2;
	}

	// SVD
	err = Singular_Value_Decomposition(B, 3, 3, (double*)U, singular_values, (double*)V, dummy_array);

	if (err < 0) {
		//printf(" Failed to converge in rotation_matrix\n");
		return 1;
	}
	// the end of SVD

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			V_arr[i * 3 + j] = V[i][j];
			U_arr[i * 3 + j] = U[i][j];
			V_tran_arr[i * 3 + j] = V_tran[i][j];
		}
	}
	det_U = det_3X3(U_arr);
	det_V = det_3X3(V_arr);
	d = det_U * det_V;

	// U * [1 0 0; 0 1 0; 0 0 1]
	U_arr[0 * 3 + 2] *= d;
	U_arr[1 * 3 + 2] *= d;
	U_arr[2 * 3 + 2] *= d;

	transpose(V_arr, V_tran_arr, 3, 3);
	mulmat(U_arr, V_tran_arr, R, 3, 3, 3);

	return 0;
}

void attitude(double R[], double angle[]) {
	angle[0] = atan2(R[7], R[8]);	// roll
	angle[1] = asin(- R[6]);	// pitch
	angle[2] = atan2(R[3], R[0]);	// yaw
}

void cost_fun(meau_model ant1_ptr[], meau_model ant2_ptr_tem[], double S1[], double S2_tem[],
				int N[], double b[], double invQY[], double R[], double* costfunction, int n) {
	double Y_vec[14], L_vec[14], N_vec[14], residual[14], costfun_temp[14];	// L_vec = S*R*b_body
	double temp[3], temp2[2] = {0}, delta_b[6];
	int i;

	// || Y - SRb_body + N ||QY
	for (i = 0; i < n; i++) {
		Y_vec[i] = ant1_ptr[i].ddcp * L1;
		Y_vec[n + i] = ant2_ptr_tem[i].ddcp * L1;
		N_vec[i] = (double)N[i] *  L1;
		N_vec[n + i] = (double)N[n + i] * L1;
	}

	temp[0] = R[0];
	temp[1] = R[3];
	temp[2] = R[6];
	mulmat(S1, temp, L_vec, n, 3, 1);

	temp[0] = R[1];
	temp[1] = R[4];
	temp[2] = R[7];
	mulmat(S2_tem, temp, &L_vec[n], n, 3, 1);

	for (i = 0; i < n; i++) {
		residual[i] = Y_vec[i] - L_vec[i] + N_vec[i];
		residual[n + i] = Y_vec[n + i] - L_vec[n + i] + N_vec[n + i];
	}

	mulmat(residual, invQY, costfun_temp, 1, 2 * n, 2 * n);
	mulmat(costfun_temp, residual, costfunction, 1, 2 * n, 1);
	*costfunction = sqrt(*costfunction);
	// end of || Y - SRb_body + N ||QY

	// || b - R * b_body||
	for (i = 0; i < 3; i++) {
		delta_b[i] = b[i] - R[i * 3];
		temp2[0] += delta_b[i] * delta_b[i];

		delta_b[3 + i] = b[3 + i] - R[i * 3 + 1];
		temp2[1] += delta_b[3 + i] * delta_b[3 + i];
	}

	*costfunction += sqrt(temp2[0]) + sqrt(temp2[1]);
	// end of || b - R * b_body||
}
