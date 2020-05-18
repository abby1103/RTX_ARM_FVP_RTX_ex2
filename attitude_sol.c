#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "ambiguity_resolution.h"
#include "position.h"
#include "measure.h"
#include "constants.h"

/*static void output_arr(double arr[], int n) {
	for (int i = 0; i < n; ++i) { printf("%f ", arr[i]); }
	printf("\n");
}

static void output_matrix(double arr[], int n, int m) {
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) printf("%10f ", arr[j + m * i]);
		printf("\n");
	}
	printf("\n");
}*/

//extern double ddcp_noise[];
//extern int epoch;

int attitude_sol(int n, double sdcp[], double sdstd, double old_an[], double small_an[], double angle[]) {

	int i, j, k;
	meau_model ant1_ptr[7];
	meau_model ant2_ptr[7];

	double QY_nsv[49] = { 0 }, QY_tem[49] = { 0 }, QY_tem2[49] = { 0 };	// 7 * 7 (# of measument for one baseline)
	double QY[196] = { 0 }, invQY[196] = { 0 };	// 14 * 14 (# of measument for two baseline)

	int err;

	double S1mat[21] = { 0 }, pinvS1mat[21] = { 0 }, pinvS1mat_tran[21] = { 0 };	// 7(# of measument for one baseline) * 3(Sx,Sy,Sz)
	double pinvS1mat_tem[21] = { 0 };
	error_std ant_std;

	cand_list cand_b1[7 - 3];	// 假設最多收到7顆衛星,因此固定 cand_list cand_b1[7-3]

	int index;

	meau_model ant2_ptr_tem[7];
	double b1[3];
	int numOFb1b2 = 0, current_index = 0;
	int* prt_tem0;
	double* prt_tem1, * prt_tem2, * prt_tem3;
	double S2mat[21] = { 0 }, pinvS2mat[21] = { 0 }, pinvS2mat_tran[21] = { 0 };	// 7(# of measument for one baseline) * 3(Sx,Sy,Sz)
	double pinvS2mat_tem[21] = { 0 };

	cand_list cand_b2[7 - 3], cand_b2_paired[1];
	cand_list cand_b1b2[1];
	double R[9], costfunction;
	double* cand_angle;

	int pass;

	// setting ddcp measument model
	n--;
	ddcp_model(ant1_ptr, ant2_ptr, sdcp, sdstd, n);

	// 模擬用
	double ddcp_noise[8] = {-0.319249, -0.312203, -0.011918, 0.256023, 0.168796, 0.353649, 1.562073, 1.047918};
	for (i = 0; i < 4; i++) {
		ant1_ptr[i].ddcp = ddcp_noise[4 + i];
		ant2_ptr[i].ddcp = ddcp_noise[i];
	}
	// the end of setting ddcp measument model


	// setting covariance matrix of measument
	cov_matrix1(QY_nsv, sdstd, n);

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			QY[(i * 2 * n) + j] = QY_nsv[i * n + j];
			QY[((i + n) * 2 * n) + (j + n)] = QY_nsv[i * n + j];
		}
	}

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			QY_tem[(i * n) + j] = 0.5 * QY_nsv[(i * n) + j]; // QY(0:n-1,n:2n-1)
		}
	}
	for (i = 0; i < n; i++) {
		ant1_ptr[i].QY_pnt = &QY_tem[i * n]; //QY_pnt 指向QY的第[i][n]個位址,之後排序會用到
	}
	// the end of setting covariance matrix of measument

	// caculate n range of b1 and sort
	b1_NRange(ant1_ptr, ant2_ptr, n, old_an, small_an);
	// the end of caculate n range of b1 and sort

	// 將排序後的 QY 轉置後給 ant2_ptr
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			QY_tem2[(i * n) + j] = *(ant1_ptr[j].QY_pnt + i);
		}
	}
	for (i = 0; i < n; i++) {
		ant2_ptr[i].QY_pnt = &QY_tem2[i * n + 0]; //QY_pnt 指向QY的第[i][n]個位址,之後排序會用到
	}
	// the end of 將排序後的 QY 轉置後給 ant2_ptr

	// caculate error of std for b1
	err = std_baseline(ant1_ptr, &ant_std, n, QY_nsv, 2, S1mat, pinvS1mat, pinvS1mat_tran, pinvS1mat_tem, 1);
	if (err != 0) return 5;
	// the end of caculate error of std for b1

	// GSO
	err = GSO(ant1_ptr, ant_std.sigma_b1_4sv, n, cand_b1);
	if (err != 0) return 6;
	// the end of GSO

	//search_moreN
	for (index = 4; index <= n; index++) {
		if (cand_b1[index - 4].numofcand != 0) {
			err = search_moreN(ant1_ptr, ant_std.sigma_b1_nsv, cand_b1[index - 4], &cand_b1[index - 3], S1mat, pinvS1mat, pinvS1mat_tem, QY_nsv, n, index, 2);
			if (err != 0) return 7;
		}
		else {
			free(cand_b1[index - 4].Ncands);
			free(cand_b1[index - 4].bcands);
			free(cand_b1[index - 4].goodness);
			//printf("false to find any Ncands\n");
			return 1;
		}
		free(cand_b1[index - 4].Ncands);
		free(cand_b1[index - 4].bcands);
		free(cand_b1[index - 4].goodness);
	}
	if (cand_b1[n - 3].numofcand == 0) {
		free(cand_b1[n - 3].Ncands);
		free(cand_b1[n - 3].bcands);
		free(cand_b1[n - 3].goodness);
		//printf("false to find any Ncands\n");
		return 1;
	}

	// b2_NRrange
	cand_b1b2[0].Ncands = (int*)malloc((3 + n) * 1 * sizeof(int));	// b1 的 n個 N + b2 的 3個
	cand_b1b2[0].bcands = (double*)malloc(6 * 1 * sizeof(double));	// 兩條baseline 3 + 3
	cand_b1b2[0].goodness = (double*)malloc(1 * sizeof(double));
	if (cand_b1b2[0].Ncands == NULL || cand_b1b2[0].bcands == NULL || cand_b1b2[0].goodness == NULL) {
		//printf(" No memory available for cand_b1b2\n");
		return 2;
	}
	cand_b1b2[0].numofcand = 0;

	cand_angle = (double*)malloc(1 * 3 * sizeof(double));
	if (cand_angle == NULL) {
		//printf(" No memory available for SVD in std_baseline for cand_angle\n");
		return 3;
	}

	for (j = 0; j < cand_b1[n - 3].numofcand; j++) {

		// seting b2 measument model
		b1[0] = cand_b1[n - 3].bcands[3 * j + 0];
		b1[1] = cand_b1[n - 3].bcands[3 * j + 1];
		b1[2] = cand_b1[n - 3].bcands[3 * j + 2];
		for (i = 0; i < n; i++) {
			ant2_ptr_tem[i].S[0] = ant2_ptr[i].S[0];
			ant2_ptr_tem[i].S[1] = ant2_ptr[i].S[1];
			ant2_ptr_tem[i].S[2] = ant2_ptr[i].S[2];
			ant2_ptr_tem[i].ddcp = ant2_ptr[i].ddcp;
			ant2_ptr_tem[i].QY_pnt = &QY_tem2[i * n + 0]; //QY_pnt 指向QY的第[i + n][0]個位址,之後排序會用到
		}
		// the end of seting b2 measument model

		// caculate n range of b2 , sort and inverse QY
		err = b2_NRrange(ant2_ptr, ant2_ptr_tem, n, old_an, b1);

		if (err == 0) {
			for (i = 0; i < n; i++) {
				for (k = 0; k < n; k++) {
					QY[(i * 2 * n) + (k + n)] = *(ant2_ptr_tem[k].QY_pnt + i); // QY(0:n-1,n:2n-1)
					QY[((i + n) * 2 * n) + k] = *(ant2_ptr_tem[i].QY_pnt + k); // QY(n:2n-1,0:n-1)
				}
			}
			inverse_nXn(QY, invQY, 2 * n);
			// the end of caculate n range of b2 , sort and inverse QY

			// caculate error of std for the length of b2 and the delta length between b1 and b2
			err = std_baseline(ant2_ptr_tem, &ant_std, n, QY_nsv, 2, S2mat, pinvS2mat, pinvS2mat_tran, pinvS2mat_tem, 2);
			if (err != 0) return 1;

			std_b1b2(ant2_ptr_tem, &ant_std, n, 2, pinvS1mat, pinvS2mat_tran, pinvS2mat_tem);
			// the end of caculate error of std for the length of b2and the delta length between b1and b2

			// GSO for b2
			err = GSO(ant2_ptr_tem, ant_std.sigma_b2_4sv, n, cand_b2);
			if (err == 1) return 1;

			// pairing
			err = pairing(cand_b2[0], cand_b2_paired, b1, ant_std.sigma_b1b2_4sv, 3);
			if (err == 1) return 8;

			//search_moreN for b2
			if (cand_b2_paired[0].numofcand != 0) {
				for (index = 4; index <= n; index++) {
					search_moreN(ant2_ptr_tem, ant_std.sigma_b2_nsv, cand_b2_paired[0], &cand_b2[index - 3], S2mat, pinvS2mat, pinvS2mat_tem, QY_nsv, n, index, 2);
					free(cand_b2_paired[0].Ncands);
					free(cand_b2_paired[0].bcands);
					free(cand_b2_paired[0].goodness);

					// pairing
					err = pairing(cand_b2[index - 3], cand_b2_paired, b1, ant_std.sigma_b1b2_nsv, index);
					if (err == 1) return 8;
				}
			}

			numOFb1b2 += cand_b2_paired[0].numofcand;
			if (cand_b2_paired[0].numofcand != 0) {
				prt_tem0 = (int*)realloc(cand_b1b2[0].Ncands, (2 * n) * numOFb1b2 * sizeof(int));	// b1 的 n個 N + b2 的 3個
				prt_tem1 = (double*)realloc(cand_b1b2[0].bcands, 6 * numOFb1b2 * sizeof(double));	// 兩條baseline 3 + 3 = 6
				prt_tem2 = (double*)realloc(cand_b1b2[0].goodness, numOFb1b2 * sizeof(double));
				prt_tem3 = (double*)realloc(cand_angle, numOFb1b2 * 3 * sizeof(double));

				if (prt_tem0 == NULL || prt_tem1 == NULL || prt_tem2 == NULL || prt_tem3 == NULL) {
					//printf(" No memory available for cand_b1b2\n");
					return 2;
				}
				else {
					cand_b1b2[0].Ncands = prt_tem0;
					cand_b1b2[0].bcands = prt_tem1;
					cand_b1b2[0].goodness = prt_tem2;
					cand_angle = prt_tem3;
				}
			}


			for (i = 0; i < cand_b2_paired[0].numofcand; i++) {
				// Merging cand_b1 and cand_b2_paired into cand_b1b2
				for (k = 0; k < n; k++) {
					cand_b1b2[0].Ncands[(i + current_index) * (2 * n) + k] = cand_b1[n - 3].Ncands[j * n + k];
					cand_b1b2[0].Ncands[(i + current_index) * (2 * n) + k + n] = cand_b2_paired[0].Ncands[i * n + k];
				}
				for (k = 0; k < 3; k++) {
					cand_b1b2[0].bcands[(i + current_index) * 6 + k] = cand_b1[n - 3].bcands[j * 3 + k];
					cand_b1b2[0].bcands[(i + current_index) * 6 + k + 3] = cand_b2_paired[0].bcands[i * 3 + k];
				}
				cand_b1b2[0].goodness[(i + current_index)] = cand_b1[n - 3].goodness[j] + cand_b2_paired[0].goodness[i];
				cand_b1b2[0].numofcand = numOFb1b2;
				// the end of merging cand_b1 and cand_b2_paired into cand_b1b

				// caculating the rotation_matrix AND angle
				err = rotation_matrix(&(cand_b1b2[0].bcands[(i + current_index) * 6 + 0]), &(cand_b1b2[0].bcands[(i + current_index) * 6 + 3]), R);
				if (err == 1) return 9;
				attitude(R, &cand_angle[(i + current_index) * 3]);
				// the end of caculating the rotation_matrix
				cost_fun(ant1_ptr, ant2_ptr_tem, S1mat, S2mat, &(cand_b1b2[0].Ncands[(i + current_index) * (2 * n) + 0]), &(cand_b1b2[0].bcands[(i + current_index) * 6 + 0]), invQY, R, &costfunction, n);
				cand_b1b2[0].goodness[(i + current_index)] += costfunction;
			}

			if (cand_b2_paired[0].numofcand != 0) {
				free(cand_b2_paired[0].Ncands);
				free(cand_b2_paired[0].bcands);
				free(cand_b2_paired[0].goodness);
			}

			current_index = numOFb1b2;
		}
	}


	free(cand_b1[n - 3].Ncands);
	free(cand_b1[n - 3].bcands);
	free(cand_b1[n - 3].goodness);

	if (cand_b1b2[0].numofcand != 0) {

		quick_sort_cands(cand_b1b2[0].goodness, cand_angle, 0, cand_b1b2[0].numofcand - 1, 3);

		if (old_an == NULL) {
			angle[0] = cand_angle[0] * 180 / PI;
			angle[1] = cand_angle[1] * 180 / PI;
			angle[2] = cand_angle[2] * 180 / PI;
			//output_arr(angle, 3);
		}
		else
		{
			for (i = 0; i < cand_b1b2[0].numofcand; i++) {
				pass = fabs(cand_angle[i * 3 + 0] - old_an[0]) <= 0.3491 && fabs(cand_angle[i * 3 + 1] - old_an[1]) <= 0.3491 && fabs(cand_angle[i * 3 + 2] - old_an[2]) <= 0.3491;
				if (pass) {
					angle[0] = cand_angle[i * 3 + 0] * 180 / PI;
					angle[1] = cand_angle[i * 3 + 1] * 180 / PI;
					angle[2] = cand_angle[i * 3 + 2] * 180 / PI;
					//output_arr(angle, 3);
					break;
				}
				else if (i == (cand_b1b2[0].numofcand - 1)) {
					//printf("false to find any cands in small angle\n");
					return 4;
				}

			}
		}
	}
	else {
		//printf("false to find any cands\n");
		return 1;
	}

	free(cand_b1b2[0].Ncands);
	free(cand_b1b2[0].bcands);
	free(cand_b1b2[0].goodness);
	free(cand_angle);

	return 0;
}
