
#ifndef __ambiguity_resolution_H
#define __ambiguity_resolution_H

#define length 1				// meter
#define ideal_delta_l 1.4142	// meter

#define L1_WL 0.1903			// L1 WAVELENGTH

typedef struct{

	double x;
	double y;
	double z;

} ECEF_pos;

typedef struct{

	double lat;
	double lon;
	double hgt;

} llh_pos;

typedef struct{

	double b_body[3]; // meter
	double S[3];
	double ddcp;
	int low;
	int up;
	int range;
	double* QY_pnt;

} meau_model;

typedef struct {

	double var_b1_4sv;
	double sigma_b1_4sv;
	double var_b1_nsv;
	double sigma_b1_nsv;

	double var_b2_4sv;
	double sigma_b2_4sv;
	double var_b2_nsv;
	double sigma_b2_nsv;

	double sigma_b1b2_4sv;
	double sigma_b1b2_nsv;

} error_std;

typedef struct {

	int* Ncands;
	double* bcands;
	double* goodness;
	int numofcand ;

} cand_list;

void inverse_nXn(double* a_prt, double* ia_prt, int n);
int inverse2X2(double* A, double* IA);
double det_3X3(double A[]);
int maxv(int[], int N);
int minv(int[], int N);
void quick_sort_struct(meau_model arr[], int first_index, int last_index);
void quick_sort_cands(double costfun[], double cand_angle[], int first_index, int last_index, int cand_cols);
void cov_matrix1(double QY[], double sdstd, int n);
double* kron( double A[], int sizeA, double B[], int sizeB );
void ddcp_model(meau_model ant_ptr[], meau_model ant2_ptr[], ECEF_pos P_ant0, llh_pos ant0_llh, ECEF_pos P_sat[], double pseudo_range[], double sdcp[], double sdstd, int n );
void b1_NRange(meau_model ant1_ptr[], meau_model ant2_ptr[], int n, double old_an[], double small_an[]);
int std_baseline(meau_model ant_ptr[], error_std* ant_std, int n, double* QY_nsv, int ratio,
	double* Smat, double* pinvSmat, double* pinvSmat_tran, double* pinvSmat_tem, int index_b);
void std_b1b2(meau_model ant_ptr[], error_std* ant_std, int n, int ratio,
	double* pinvS1mat, double* pinvS2mat_tran, double* pinvS2mat_tem);
int GSO(meau_model ant_ptr[], double sigma_b, int n,volatile cand_list cand_b[]);
int search_moreN(meau_model ant_ptr[], double sigma_b,volatile cand_list cand_b_pre,volatile cand_list* cand_b_new,
	double* Smat, double* ISmat, double* ISmat_tem1, double* QY1_nsv, int n, int index, int ratio);
int b2_NRrange(meau_model ant2_ptr[], meau_model ant2_ptr_tem[], int n, double old_an[], double b1[]);
int pairing(volatile cand_list cand_b2,volatile cand_list cand_b2_paired[], double b1[], double sigma_b1b2_4sv, int index);
int rotation_matrix(double b1[], double b2[], double R[]);
void attitude(double R[], double angle[]);
void cost_fun(meau_model ant1_ptr[], meau_model ant2_ptr_tem[], double S1[], double S2_tem[],
	int N[], double b[], double invQY[], double R[], double* costfunction, int n);

void test(double** a);

#endif

