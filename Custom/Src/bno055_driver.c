/*
 * bno055_driver.c
 *
 *  Created on: Jun 20, 2022
 *      Author: keonyoung
 */

#include <math.h>

#include "bno055_driver.h"
#include "matrix_calculation.h"
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "stm32h7xx_hal_tim.h"

bno055_Object hbno;

uint8_t acc_buff[6];
uint8_t mag_buff[6];
uint8_t gyr_buff[6];

int16_t acc[3];
int16_t mag[3];
int16_t gyr[3];

int flag;

int measure_rate = 2;
float Ts = 0.0052;

float g = 9.80665;
float g_k;


float roll, pitch, yaw;

float roll_gyr = 0;
float pitch_gyr = 0;
float yaw_gyr = 0;

float t_roll;
float t_pitch;
float t_yaw;

float alpha = 0.98;
float Ax, Ay, Az;
float wx, wy, wz;
float mx, my, mz;


int timetimetime;

/* EKF */

Matrix *x_t; // 3 x 1 matrix
float x_t_datalist[3] = {0, 0, 0};
Matrix *F_t; // 3 x 1 matrix
float F_t_datalist[3] = {0, 0, 0};
Matrix *P_t; // 3 x 3 matrix
float P_t_datalist[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
Matrix *f_t; // 3 x 3 matrix
float f_t_datalist[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *Q_t; // 3 x 3 matrix
float Q_t_datalist[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *K_t; // 3 x 4 matrix
float K_t_datalist[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *h_t; // 4 x 3 matrix
float h_t_datalist[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *R_t; // 4 x 4 matrix
float R_t_datalist[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *y_t; // 4 x 1 matrix
float y_t_datalist[4] = {0, 0, 0, 0};
Matrix *H_t; // 4 x 1 matrix
float H_t_datalist[4] = {0, 0, 0, 0};
Matrix *f_t_T;
float f_t_T_datalist[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *h_t_T; // 3 x 4 matrix
float h_t_T_datalist[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *buff1; // 3 x 3 matrix
float buff1_datalist[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *buff2; // 4 x 3 matrix
float buff2_datalist[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *buff3; // 4 x 4 matrix
float buff3_datalist[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *buff4; // 3 x 4 matrix
float buff4_datalist[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *buff5;
float buff5_datalist[3] = {0, 0, 0};
Matrix *buff6; // 3 x 3 matrix
float buff6_datalist[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix *identity; // 3 x 3 matrix
float identity_datalist[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

int echo = 0;

uint8_t serial_roll[4];
uint8_t serial_pitch[4];

uint8_t str[] = "Hello World!\n\r";
uint8_t endstr[] = "\n";
uint8_t receivebuffer[1];

/* EKF Covariance Matrix Tuning Variables */
float Q_gain = 0.005;
float Q_offset = 0.00001;
float R_gain = 500;
float R_offset = 1000;

//float alpha = 0.98;

/* ESOQ2 */

//float roll_esoq2;
//float pitch_esoq2;
//float yaw_esoq2;

//float Ax_s, Ay_s, Az_s;
//float mx_s, my_s, mz_s;
//
//Matrix *B_matrix; // 3x3
//float B_matrix_datalist[9];
//
//Matrix *K_matrix; // 4x4
//float K_matrix_datalist[16];
//
//Matrix *z_matrix; // 3x1
//float z_matrix_datalist[3];
//-
//Matrix *e_matrix; // 3x1
//float e_matrix_datalist[3];
//
//float q0, q1, q2, q3;
//
//Matrix *S_matrix; // 3x3
//float S_matrix_datalist[9];
//
//Matrix *M_matrix;
//float M_matrix_datalist[9];
//
//Matrix *zzT_matrix;
//float zzT_matrix_datalist[9];
//
//float x_esoq2[4];
//
//Matrix *y_save_matrix; // 4x3
//float y_save_matrix_datalist[12];
//
//Matrix *y_matrix; // 4x1
//float y_matrix_datalist[4];
//
//float alpha1 = 0;
//float alpha2 = 1;
//
//float delta1 = 0;
//
////float m_earth_x = 0.57315;
////float m_earth_y = 0.0894;
////float m_earth_z = -0.81456;
//
//float m_earth_x, m_earth_y, m_earth_z;
//
//float lambdamax, lambda3;
//float b, d;
//float t_esoq2;
//
//float Ma, Mb, Mc, Mx, My, Mz;
//float sin_rotangle, cos_rotangle;
//
//float xTx;
//float delta2 = 0;
//
//float check;
//
//float x_max;
//
//int max_index1, max_index2;
//
//float max_x, max_y;


//static int32_t IMU_IO_Init(void)
//{
//	return BNO055_Init(&hbno);
//}
//
//static int32_t IMU_IO_DeInit(void)
//{
//	return BNO055_DeInit(&hbno);
//}

static int32_t IMU_IO_IsDevReady(uint16_t dev_addr, uint32_t trials, uint32_t timeout)
{
	if(HAL_I2C_IsDeviceReady(&hi2c1, dev_addr, trials, timeout) == HAL_OK) {
		return 0;
	} else {
		return -1;
	}
}

static int32_t IMU_IO_ReadReg(uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size)
{
	int32_t res;
	uint8_t reg = reg_addr;
	res = IOIF_I2C_Tx(dev_addr, &reg, 1);
	res = IOIF_I2C_Rx(dev_addr, data, size);
	return res;
}


static int32_t IMU_IO_WriteReg(uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size)
{
	return IOIF_I2C_MemTx(dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size);
}

static int32_t IMU_IO_Wait(uint32_t time)
{
	return IOIF_WaitASec(time);
}


static void IMU_ReadBuffData(){
	for(int i=0; i<3; i++){
	        acc[i] = acc_buff[2*i+1] << 8 | acc_buff[2*i];
	        gyr[i] = gyr_buff[2*i+1] << 8 | gyr_buff[2*i];
	        mag[i] = mag_buff[2*i+1] << 8 | mag_buff[2*i];
	}
}

/* convert float data to byte */
static void FloatToBytes(float val, uint8_t *bytes_array) {
	union {
		float float_variable;
		uint8_t temp_array[4];
	} u;
	u.float_variable = val;
	memcpy(bytes_array, u.temp_array, 4);
}

static void EKF_Init()
{
		/* scan imu data */
		BNO055_ScanAll(&hbno, acc_buff, gyr_buff, mag_buff);
	    IMU_ReadBuffData();

	    /* set roll, pitch, yaw initial values */
		roll = atan2f(acc[1], acc[2]);
		pitch = atan2f(-acc[0], (sqrtf((float)(acc[1]*acc[1]/100+acc[2]*acc[2]/100))*10));
		yaw = 0;

		/* initialize required matrices */
		x_t_datalist[0] = yaw;
		x_t_datalist[1] = pitch;
		x_t_datalist[2] = roll;

		x_t = MatrixConstructor(3, 1);
		MatrixInitializer(x_t, x_t_datalist, 3, 1);

		f_t = MatrixConstructor(3, 3);

		y_t = MatrixConstructor(4, 1);

		H_t = MatrixConstructor(4, 1);

		buff1 = MatrixConstructor(3, 3);
		MatrixInitializer(buff1, buff1_datalist, 3, 3);

		P_t = MatrixConstructor(3, 3);
		MatrixInitializer(P_t, P_t_datalist, 3, 3);

		f_t_T = MatrixConstructor(3, 3);
		MatrixInitializer(f_t_T, f_t_T_datalist, 3, 3);

		Q_t = MatrixConstructor(3, 3);

		h_t = MatrixConstructor(4, 3);

		h_t_T = MatrixConstructor(3, 4);

		K_t = MatrixConstructor(3, 4);

		R_t = MatrixConstructor(4, 4);

		buff2 = MatrixConstructor(4, 3);

		buff3 = MatrixConstructor(4, 4);

		buff4 = MatrixConstructor(3, 4);

		buff5 = MatrixConstructor(3, 1);

		buff6 = MatrixConstructor(3, 3);

		identity = MatrixConstructor(3, 3);
		MatrixInitializer(identity, identity_datalist, 3, 3);

}

/* send IMU data via UART */
static void SerialEcho(uint8_t *src1, uint8_t *src2, int length)
{
	while(HAL_UART_Receive_IT(&huart3, receivebuffer, 1) != HAL_OK) {
		osDelay(1);
	}
	HAL_UART_Transmit_IT(&huart3, src1, length);
	while(HAL_UART_Receive_IT(&huart3, receivebuffer, 1) != HAL_OK) {
			osDelay(1);
		}
	HAL_UART_Transmit_IT(&huart3, src2, length);
//	HAL_UART_Transmit_IT(&huart3, endstr, 1);
}

static void EKF()
{
	Ax = (float)acc[0]/(float)16384*16*g;
	Ay = (float)acc[1]/(float)16384*16*g;
	Az = (float)acc[2]/(float)16384*16*g;

	wx = gyr[0]/(float)65536*4000*3.141592/180;
	wy = gyr[1]/(float)65536*4000*3.141592/180;
	wz = gyr[2]/(float)65536*4000*3.141592/180;


	mx = (float)mag[0];
	my = (float)mag[1];
	mz = (float)mag[2];

	// initialize Xt+1 = F(Xt, wt)
	x_t_datalist[0] = yaw + sinf(roll)/cosf(pitch)*wy*Ts+cosf(roll)/cosf(pitch)*wz*Ts; // yaw
	x_t_datalist[1] = pitch + cosf(roll)*wy*Ts-sinf(roll)*wz*Ts; // pitch
	x_t_datalist[2] = roll + wx*Ts + sinf(roll)*tanf(pitch)*wy*Ts + cosf(roll)*tanf(pitch)*wz*Ts; // roll
	MatrixInitializer(x_t, x_t_datalist, 3, 1);

	// calculate estimated roll, pitch, yaw
	yaw = x_t_datalist[0];
	pitch = x_t_datalist[1];
	roll = x_t_datalist[2];

	// initialize ft, ftT(transpose of ft)
	f_t_datalist[0] = 1;
	f_t_datalist[1] = sinf(roll)/cosf(pitch)*tanf(pitch)*wy*Ts + cosf(roll)/cosf(pitch)*tanf(pitch)*wz*Ts;
	f_t_datalist[2] = cosf(roll)/cosf(pitch)*wy*Ts - sinf(roll)/cosf(pitch)*wz*Ts;
	f_t_datalist[3] = 0;
	f_t_datalist[4] = 1;
	f_t_datalist[5] = -sinf(roll)*wy*Ts - cosf(roll)*wz*Ts;
	f_t_datalist[6] = 0;
	f_t_datalist[7] = sinf(roll)/(cosf(pitch)*cosf(pitch))*wy*Ts + cosf(roll)/(cosf(pitch)*cosf(pitch))*wz*Ts;
	f_t_datalist[8] = 1 + cosf(roll)*tanf(pitch)*wy*Ts - sinf(roll)*tan(pitch)*wz*Ts;

	MatrixInitializer(f_t, f_t_datalist, 3, 3);
	Transpose(f_t_T, f_t);

	// initialize yt+1
	y_t_datalist[0] = atan2f(-(cosf(roll)*my-sinf(roll)*mz), (cosf(pitch)*mx+sinf(roll)*sinf(pitch)*my+cosf(roll)*sinf(pitch)*mz));
	y_t_datalist[1] = Ax;
	y_t_datalist[2] = Ay;
	y_t_datalist[3] = Az;
	MatrixInitializer(y_t, y_t_datalist, 4, 1);

	// initialize Ht+1
	H_t_datalist[0] = yaw;
	H_t_datalist[1] = -sinf(pitch)*g;
	H_t_datalist[2] = cosf(pitch)*sinf(roll)*g;
	H_t_datalist[3] = cosf(pitch)*cosf(roll)*g;
	MatrixInitializer(H_t, H_t_datalist, 4, 1);

	// initialize covariance matrix Qt
	Q_t_datalist[0] = Q_gain*sqrtf(wx*wx + wy*wy + wz*wz)+Q_offset;
	Q_t_datalist[4] = Q_gain*sqrtf(wx*wx + wy*wy + wz*wz)+Q_offset;
	Q_t_datalist[8] = Q_gain*sqrtf(wx*wx + wy*wy + wz*wz)+Q_offset;
	MatrixInitializer(Q_t, Q_t_datalist, 3, 3);

	// calculate Pt+1 = fPfT  + Q
	MatrixMultiply(buff1, f_t, P_t); // here !!
	MatrixMultiply(P_t, buff1, f_t_T);
	MatrixSum(P_t, P_t, Q_t);

	// initialize ht+1, ht+1T
	h_t_datalist[0] = 1;
	h_t_datalist[1] = 0;
	h_t_datalist[2] = 0;
	h_t_datalist[3] = 0;
	h_t_datalist[4] = -cosf(pitch)*g;
	h_t_datalist[5] = 0;
	h_t_datalist[6] = 0;
	h_t_datalist[7] = -sinf(pitch)*sinf(roll)*g;
	h_t_datalist[8] = cosf(pitch)*cosf(roll)*g;
	h_t_datalist[9] = 0;
	h_t_datalist[10] = -sinf(pitch)*cosf(roll)*g;
	h_t_datalist[11] = -cosf(pitch)*sinf(roll)*g;
	MatrixInitializer(h_t, h_t_datalist, 4, 3);
	Transpose(h_t_T, h_t);

	// initialize covariance matrix Rt
	R_t_datalist[0] = 0*sqrtf(mx*mx + my*my + mz*mz)+0;
	R_t_datalist[5] = R_gain*sqrtf(Ax*Ax + Ay*Ay + (Az-g)*(Az-g))+R_offset;
	R_t_datalist[10] = R_gain*sqrtf(Ax*Ax + Ay*Ay + (Az-g)*(Az-g))+R_offset;
	R_t_datalist[15] = R_gain*sqrtf(Ax*Ax + Ay*Ay + (Az-g)*(Az-g))+R_offset;
	MatrixInitializer(R_t, R_t_datalist, 4, 4);

	// calculate Kalman gain : Kt+1 = Ph(hphT + R)-1
	MatrixMultiply(buff2, h_t, P_t);
	MatrixMultiply(buff3, buff2, h_t_T);
	MatrixSum(buff3, buff3, R_t);
	flag = Inverse(buff3, buff3);
	for(int k = 0; k < 4; k++){
		for(int l = 0; l < 4; l++){
			buff3_datalist[4*k + l] = buff3->data[k][l];
		}
	}
	MatrixMultiply(buff4, P_t, h_t_T);
	MatrixMultiply(K_t, buff4, buff3);

	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 4; j++){
			K_t_datalist[4*i + j] = K_t->data[i][j];
		}
	}

	// calculate xt+1 = xt+1 + Kt+1(yt+1-Ht+1)
	MatrixSubstract(y_t, y_t, H_t);
	MatrixMultiply(buff5, K_t, y_t);
	MatrixSum(x_t, x_t, buff5);

	yaw = x_t->data[0][0];
	pitch = x_t->data[1][0];
	roll = x_t->data[2][0];

	t_yaw = yaw;
	t_pitch = pitch;
	t_roll = roll;

	// update P_t
	MatrixMultiply(buff1, K_t, h_t);
	MatrixMultiply(buff6, buff1, P_t);
	MatrixSubstract(P_t, P_t, buff6);

}

//static void ESOQ2_Init()
//{
//	B_matrix = matrixConstructor(3, 3);
//	z_matrix = matrixConstructor(3, 1);
//	K_matrix = matrixConstructor(4, 4);
//	S_matrix = matrixConstructor(3, 3);
//	y_save_matrix = matrixConstructor(4, 3);
//	y_matrix = matrixConstructor(4, 1);
//	e_matrix = matrixConstructor(3, 1);
//
//	BNO055_ScanAll(&hbno, acc_buff, gyr_buff, mag_buff);
//	IMU_ReadBuffData();
//
//	IOIF_WaitASec(2000);
//
//	BNO055_ScanAll(&hbno, acc_buff, gyr_buff, mag_buff);
//	IMU_ReadBuffData();
//
//	mx = (float)mag[0];
//	my = (float)mag[1];
//	mz = (float)mag[2];
//
//	float param3 = sqrtf(mx*mx + my*my + mz*mz);
//
////	m_earth_x = mx/param3;
////	m_earth_y = my/param3;
////	m_earth_z = mz/param3;
//
//	m_earth_x = 1/sqrtf(14);
//	m_earth_y = 2/sqrtf(14);
//	m_earth_z = 3/sqrtf(14);
////	m_earth_x = mx;
////	m_earth_y = my;
////	m_earth_z = mz;
//
//}

//static int max3(float aa, float bb, float cc)
//{
//	float max;
//	max = aa > bb ? aa : bb;
//	max = max > cc ? max : cc;
//	if(max == aa) return 0;
//	else if(max == bb) return 1;
//	else return 2;
//}
//
//static int max4(float aa, float bb, float cc, float dd)
//{
//	float max;
//	max = aa > bb ? aa : bb;
//	max = max > cc? max : cc;
//	max = max > dd ? max : dd;
//	if(max == aa) return 0;
//	else if(max == bb) return 1;
//	else if(max == cc) return 2;
//	else return 3;
//}

//static void ESOQ2()
//{
//	/* normalize accelerometer data */
//	Ax_s = (float)acc[0];
//	Ay_s = (float)acc[1];
//	Az_s = (float)acc[2];
//	float param1 = sqrtf(Ax_s*Ax_s + Ay_s*Ay_s + Az_s*Az_s);
////	Ax = Ax_s/param1;
////	Ay = Ay_s/param1;
////	Az = Az_s/param1; // uncomment
//	Ax = 0;
//	Ay = -sqrtf(0.5);
//	Az = sqrtf(0.5);
//
//	/* normalize magnetometer data */
//	mx_s = (float)mag[0];
//	my_s = (float)mag[1];
//	mz_s = (float)mag[2];
//	float param2 = sqrtf(mx_s*mx_s + my_s*my_s + mz_s*mz_s);
//	mx = m_earth_x;
//	my = sqrtf(0.5)*m_earth_y-sqrtf(0.5)*m_earth_z;
//	mz = sqrtf(0.5)*m_earth_y+sqrtf(0.5)*m_earth_z;
////	mx = mx_s/param2;
////	my = my_s/param2;
////	mz = mz_s/param2; uncomment
//
//	/* match unit of gyroscope data */
//	wx = gyr[0]/(float)65536*4000*3.141592/180;
//	wy = gyr[1]/(float)65536*4000*3.141592/180;
//	wz = gyr[2]/(float)65536*4000*3.141592/180;
//
//	/* initialize B, z matrix */
//	B_matrix_datalist[0] = alpha1*mx*m_earth_x;
//	B_matrix_datalist[1] = alpha1*mx*m_earth_y;
//	B_matrix_datalist[2] = alpha1*mx*m_earth_z + alpha2*Ax;
//	B_matrix_datalist[3] = alpha1*my*m_earth_x;
//	B_matrix_datalist[4] = alpha1*my*m_earth_y;
//	B_matrix_datalist[5] = alpha1*my*m_earth_z + alpha2*Ay;
//	B_matrix_datalist[6] = alpha1*mz*m_earth_x;
//	B_matrix_datalist[7] = alpha1*mz*m_earth_y;
//	B_matrix_datalist[8] = alpha1*mz*m_earth_z + alpha2*Az;
//	matrixInitializer(B_matrix, B_matrix_datalist, 3, 3);
//
////	z_matrix_datalist[0] = B_matrix_datalist[5] - B_matrix_datalist[7];
////	z_matrix_datalist[1] = B_matrix_datalist[6] - B_matrix_datalist[2];
////	z_matrix_datalist[2] = B_matrix_datalist[1] - B_matrix_datalist[3];
//	z_matrix_datalist[0] = B_matrix->data[1][2] - B_matrix->data[2][1];
//	z_matrix_datalist[1] = B_matrix->data[2][0] - B_matrix->data[0][2];
//	z_matrix_datalist[2] = B_matrix->data[0][1] - B_matrix->data[1][0];
//	matrixInitializer(z_matrix, z_matrix_datalist, 3, 1);
//
//	/* set S matrix as B + transpose(B) */
//	transpose(S_matrix, B_matrix);
//	matrixSum(S_matrix, S_matrix, B_matrix);
//
//	/* initialize K matrix */
//	float trace_B = B_matrix->data[0][0] + B_matrix->data[1][1] + B_matrix->data[2][2];
//	K_matrix_datalist[0] = S_matrix->data[0][0] - trace_B;
//	K_matrix_datalist[1] = S_matrix->data[0][1];
//	K_matrix_datalist[2] = S_matrix->data[0][2];
//	K_matrix_datalist[3] = z_matrix->data[0][0];
//	K_matrix_datalist[4] = S_matrix->data[1][0];
//	K_matrix_datalist[5] = S_matrix->data[1][1] - trace_B;
//	K_matrix_datalist[6] = S_matrix->data[1][2];
//	K_matrix_datalist[7] = z_matrix->data[1][0];
//	K_matrix_datalist[8] = S_matrix->data[2][0];
//	K_matrix_datalist[9] = S_matrix->data[2][1];
//	K_matrix_datalist[10] = S_matrix->data[2][2] - trace_B;
//	K_matrix_datalist[11] = z_matrix->data[2][0];
//	K_matrix_datalist[12] = z_matrix->data[0][0];
//	K_matrix_datalist[13] = z_matrix->data[1][0];
//	K_matrix_datalist[14] = z_matrix->data[2][0];
//	K_matrix_datalist[15] = trace_B;
//	matrixInitializer(K_matrix, K_matrix_datalist, 4, 4);
//
//	/* calculate eigenvalues of K */
//	d = determinant(K_matrix);
//	b = -2*trace_B + S_matrix->data[1][1]*S_matrix->data[2][2] - S_matrix->data[1][2]*S_matrix->data[2][1]
//				   + S_matrix->data[0][0]*S_matrix->data[2][2] - S_matrix->data[0][2]*S_matrix->data[2][0]
//				   + S_matrix->data[0][0]*S_matrix->data[1][1] - S_matrix->data[0][1]*S_matrix->data[1][0]
//				   - z_matrix_datalist[0]*z_matrix_datalist[0] - z_matrix_datalist[1]*z_matrix_datalist[1] - z_matrix_datalist[2]*z_matrix_datalist[2];
//	float g3 = sqrtf(2*sqrtf(d)-b);
//	float g4 = sqrtf(-2*sqrtf(d)-b);
//	lambdamax = (g3+g4)/2.0;
//	lambda3 = (g3-g4)/2.0;
//
//	/* calculate principal axis */
//	t_esoq2 = trace_B - lambdamax;
//	S_matrix->data[0][0] -= (trace_B + lambdamax);
//	S_matrix->data[1][1] -= (trace_B + lambdamax);
//	S_matrix->data[2][2] -= (trace_B + lambdamax);
////	matrixScalarMultiply2(S_matrix, S_matrix, t_esoq2);
//	Ma = S_matrix->data[0][0]*t_esoq2 - z_matrix_datalist[0]*z_matrix_datalist[0];
//	Mx = S_matrix->data[0][1]*t_esoq2 - z_matrix_datalist[0]*z_matrix_datalist[1];
//	My = S_matrix->data[0][2]*t_esoq2 - z_matrix_datalist[0]*z_matrix_datalist[2];
//	Mb = S_matrix->data[1][1]*t_esoq2 - z_matrix_datalist[1]*z_matrix_datalist[1];
//	Mz = S_matrix->data[1][2]*t_esoq2 - z_matrix_datalist[1]*z_matrix_datalist[2];
//	Mc = S_matrix->data[2][2]*t_esoq2 - z_matrix_datalist[2]*z_matrix_datalist[2];
//	max_index1 = max3(fabsf(Mb*Mc - Mz*Mz), fabsf(Ma*Mc - My*My), fabsf(Ma*Mb - Mx*Mx));
//	if(max_index1 == 0){
//		e_matrix_datalist[0] = Mb*Mc - Mz*Mz;
//		e_matrix_datalist[1] = My*Mz - Mx*Mc;
//		e_matrix_datalist[2] = Mx*Mz - My*Mb;
//	} else if(max_index1 == 1) {
//		e_matrix_datalist[0] = My*Mz - Mx*Mc;
//		e_matrix_datalist[1] = Ma*Mc - My*My;
//		e_matrix_datalist[2] = Mx*My - Mz*Ma;
//	} else {
//		e_matrix_datalist[0] = Mx*Mz - My*Mb;
//		e_matrix_datalist[1] = Mx*My - Mz*Ma;
//		e_matrix_datalist[2] = Ma*Mb - Mx*Mx;
//	}
//	float e_size = sqrtf(e_matrix_datalist[0]*e_matrix_datalist[0] + e_matrix_datalist[1]*e_matrix_datalist[1] + e_matrix_datalist[2]*e_matrix_datalist[2]);
//	for(int h = 0; h < 3; h++){
//		e_matrix_datalist[h] = e_matrix_datalist[h]/e_size;
//	}
//	matrixInitializer(e_matrix, e_matrix_datalist, 3, 1);
//
//	/* calculate principal angle */
//	for(int i = 0; i < 3; i++) {
//		x_esoq2[i] = z_matrix_datalist[i];
//
//	}
//	x_esoq2[3] = t_esoq2;
////	max_index2 = max4(fabsf(x_esoq2[0]), fabsf(x_esoq2[1]), fabsf(x_esoq2[2]), fabsf(x_esoq2[3]));
//
//	for(int j = 0; j < 3; j++) {
//		for(int k = 0; k < 3; k++) {
//			y_save_matrix->data[j][k] = -1*S_matrix->data[j][k];
//		}
//	}
//	for(int l = 0; l < 3; l++) {
//		y_save_matrix->data[3][l] = -1*z_matrix_datalist[l];
//	}
//	matrixMultiply(y_matrix, y_save_matrix, e_matrix);
//	for(int m = 0; m < 4; m++) {
//		y_matrix_datalist[m] = y_matrix->data[m][0];
//	}
//
//	max_index2 = max4(fabsf(y_matrix->data[0][0]), fabsf(y_matrix->data[1][0]), fabsf(y_matrix->data[2][0]), fabsf(y_matrix->data[3][0]));
//
//	max_x = x_esoq2[max_index2];
//	max_y = y_matrix->data[max_index2][0];
//	sin_rotangle = max_x/sqrtf(max_x*max_x + max_y*max_y);
//	cos_rotangle = max_y/sqrtf(max_x*max_x + max_y*max_y);
//
////	sin_rotangle = x_esoq2[max_index2]/sqrtf(x_esoq2[max_index2]*x_esoq2[max_index2] + y_matrix->data[max_index2][0]*y_matrix->data[max_index2][0]);
////	cos_rotangle = y_matrix->data[max_index2][0]/sqrtf(x_esoq2[max_index2]*x_esoq2[max_index2] + y_matrix->data[max_index2][0]*y_matrix->data[max_index2][0]);
//
//	/* calculate unit quaternion */
//	q0 = cos_rotangle;
//	q1 = e_matrix_datalist[0]*sin_rotangle;
//	q2 = e_matrix_datalist[1]*sin_rotangle;
//	q3 = e_matrix_datalist[2]*sin_rotangle;
//
//	/* calculate euler angle via unit quaternion conversion */
//	roll_esoq2 = atan2(2*(q0*q1 + q2*q3), (1-2*(q1*q1 + q2*q2)));
//	pitch_esoq2 = asinf(2*(q0*q2 - q3*q1));
//	yaw_esoq2 = atan2f(2*(q0*q3 + q1*q2), (1-2*(q2*q2 + q3*q3)));
//
//	roll_gyr = roll_gyr + wx*Ts;
//	pitch_gyr = pitch_gyr + wy*Ts;
//	yaw_gyr = yaw_gyr + wz*Ts;
//
//	t_roll = 0.9*roll_gyr + 0.1*roll_esoq2;
//	t_pitch = 0.9*pitch_gyr + 0.1*pitch_esoq2;
//	t_yaw = 0.9*yaw_gyr + 0.1*yaw_esoq2;
//}



void BNO_Init()
{
	bno055_IOctx ioctx = {
		NULL,
		NULL,
		IMU_IO_IsDevReady,
		IMU_IO_ReadReg,
		IMU_IO_WriteReg,
		IMU_IO_Wait
	};

	BNO055_RegisterIOContext(&hbno, &ioctx);

//	int32_t status = BNO055_Init(&hbno);
	BNO055_Init(&hbno);

	HAL_TIM_Base_Start_IT(&htim15);

	/* EKF */
	EKF_Init();

	/* ESOQ2 */
//	ESOQ2_Init(&hbno);
//	IOIF_WaitASec(1);
}




void BNO_Run()
{
	__HAL_TIM_SET_COUNTER(&htim15, 0);
	BNO055_ScanAll(&hbno, acc_buff, gyr_buff, mag_buff);
    IMU_ReadBuffData();

    /* complementary filter : for test */
//	float totVec1 = sqrtf((float)(acc[1]*acc[1	]/100+acc[2]*acc[2]/100))*10;
//	float totVec2 = sqrtf((float)(acc[0]*acc[0]/100+acc[2]*acc[2]/100))*10;
//
//	float rot1 = atan2(acc[0], totVec1);
//	float rot2 = atan2(acc[1], totVec2);
//
//	roll = alpha*(roll + (float)gyr[0]*0.004*0.001)+(1-alpha)*rot1*180/3.141592;
//	pitch = alpha*(pitch + (float)gyr[1]*0.004*0.001)+(1-alpha)*rot2*180/3.141592;
//	IOIF_WaitASec(1);

    /* EKF */

    EKF();

    /* ESOQ2 */
//    ESOQ2();

    IOIF_WaitASec(measure_rate);
    timetimetime = __HAL_TIM_GET_COUNTER(&htim15);

}

void Serial_Send()
{
	FloatToBytes(t_roll, serial_roll);
	FloatToBytes(t_pitch, serial_pitch);
	SerialEcho(serial_roll, serial_pitch, 4);
}

