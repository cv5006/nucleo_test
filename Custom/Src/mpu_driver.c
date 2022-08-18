///*
// * mpu_driver.c
// *
// *  Created on: 2022. 5. 31.
// *      Author: keonyoung
// */
//
//
//#include "mpu_driver.h"
//#include <math.h>
//#include "matrix_calculation.h"
//
//MPU_obj himu;
//
//int flag;
//
//int measure_rate = 8;
//float Ts = 0.01;
//
//float g = 9.80665;
//float g_k;
//
//float roll;
//float pitch;
//float yaw;
//
//float t_roll;
//float t_pitch;
//float t_yaw;
//
////float alpha = 0.98;
//float Ax, Ay, Az;
//float wx, wy, wz;
//float mx, my, mz;
//
//float temp1, temp2, temp3, temp4;
//
//float buff5_0, buff5_1, buff5_2;
//
//Matrix *x_t; // 3 x 1
//float x_t_datalist[3] = {0, 0, 0};
//Matrix *F_t; // 3 x 1
//float F_t_datalist[3] = {0, 0, 0};
//Matrix *P_t; // 3 x 3
//float P_t_datalist[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
//Matrix *f_t; // 3 x 3
//float f_t_datalist[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *Q_t; // 3 x 3
//float Q_t_datalist[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *K_t; // 3 x 4
//float K_t_datalist[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *h_t; // 4 x 3
//float h_t_datalist[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *R_t; // 4 x 4
//float R_t_datalist[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *y_t; // 4 x 1
//float y_t_datalist[4] = {0, 0, 0, 0};
//Matrix *H_t; // 4 x 1
//float H_t_datalist[4] = {0, 0, 0, 0};
//Matrix *f_t_T;
//float f_t_T_datalist[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *h_t_T; // 3 x 4
//float h_t_T_datalist[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *buff1; // 3 x 3
//float buff1_datalist[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *buff2; // 4 x 3
//float buff2_datalist[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *buff3; // 4 x 4
//float buff3_datalist[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *buff4; // 3 x 4
//float buff4_datalist[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *buff5;
//float buff5_datalist[3] = {0, 0, 0};
//Matrix *buff6; // 3 x 3
//float buff6_datalist[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
//Matrix *identity; // 3 x 3
//float identity_datalist[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
//
//
//
//
//static void Driver_IOWait(uint16_t dev_addr, uint32_t timeout)
//{
//    IOIF_I2C_WaitForI2C_Ready(dev_addr, timeout);
//}
//static int32_t Driver_IORegisterRx(uint16_t dev_addr, uint16_t reg_addr, uint8_t* rx, uint16_t size)
//{
//	return IOIF_I2C_RegisterRx(dev_addr, reg_addr, rx, size);
//}
//static int32_t Driver_IORegisterTx(uint16_t dev_addr, uint16_t reg_addr, uint8_t* rx, uint16_t size)
//{
//	return IOIF_I2C_RegisterTx(dev_addr, reg_addr, rx, size);
//}
//
//// can be called from freertos.c
//// seti2c + wait + configure
//void IMU_Init()
//{
//
//    MPU9250_IOctx ioctx = {
//        Driver_IOWait,
//        Driver_IORegisterRx,
//        Driver_IORegisterTx
//    };
//
//    // initialize himu with IOcontext
//    MPU9250_RegisterIOContext(&himu, &ioctx);
//
//    // seti2c + wait + configure -> seti2c no more use
//    MPU9250_Initialize(&himu);
//
//    // initial value for EKF
//    MPU9250_ReadData(&himu);
//	MPU9250_ReadBuffData(&himu);
//
////	g_k = g/(float)himu.acc[2];
//
//	roll = atan2f(himu.acc[1], himu.acc[2]);
//	pitch = atan2f(-himu.acc[0], (sqrtf((float)(himu.acc[1]*himu.acc[1]/100+himu.acc[2]*himu.acc[2]/100))*10));
////	yaw = atan2f(-(cosf(roll)*himu.mag[1]-sinf(roll)*himu.mag[2]), (cosf(pitch)*himu.mag[0]+sinf(roll)*sinf(pitch)*himu.mag[1]+cosf(roll)*sinf(pitch)*himu.mag[2]));
////	roll = 0;
////	pitch = 0;
//	yaw = 0;
//
//	x_t_datalist[0] = yaw;
//	x_t_datalist[1] = pitch;
//	x_t_datalist[2] = roll;
//
//	x_t = matrixConstructor(3, 1);
//	matrixInitializer(x_t, x_t_datalist, 3, 1);
//
//	f_t = matrixConstructor(3, 3);
//
//	y_t = matrixConstructor(4, 1);
//
//	H_t = matrixConstructor(4, 1);
//
//	buff1 = matrixConstructor(3, 3);
//	matrixInitializer(buff1, buff1_datalist, 3, 3);
//
//	P_t = matrixConstructor(3, 3);
//	matrixInitializer(P_t, P_t_datalist, 3, 3);
//
//	f_t_T = matrixConstructor(3, 3);
//	matrixInitializer(f_t_T, f_t_T_datalist, 3, 3);
//
//	Q_t = matrixConstructor(3, 3);
//
//	h_t = matrixConstructor(4, 3);
//
//	h_t_T = matrixConstructor(3, 4);
//
//	K_t = matrixConstructor(3, 4);
//
//	R_t = matrixConstructor(4, 4);
//
//	buff2 = matrixConstructor(4, 3);
//
//	buff3 = matrixConstructor(4, 4);
//
//	buff4 = matrixConstructor(3, 4);
//
//	buff5 = matrixConstructor(3, 1);
//
//	buff6 = matrixConstructor(3, 3);
//
//
//	identity = matrixConstructor(3, 3);
//	matrixInitializer(identity, identity_datalist, 3, 3);
//
//	IOIF_WaitForASec(measure_rate);
//
//
//}
//// readdata + readbuffdata()
//void IMU_Run()
//{
//    MPU9250_ReadData(&himu);
//    MPU9250_ReadBuffData(&himu);
//
//    /* complementary filter : for test */
////	float totVec1 = sqrtf((float)(himu.acc[1]*himu.acc[1]/100+himu.acc[2]*himu.acc[2]/100))*10;
////	float totVec2 = sqrtf((float)(himu.acc[0]*himu.acc[0]/100+himu.acc[2]*himu.acc[2]/100))*10;
////
////	float rot1 = atan2(himu.acc[0], totVec1);
////	float rot2 = atan2(himu.acc[1], totVec2);
////
////	roll = alpha*(roll + (float)himu.gyr[0]*0.004*0.001)+(1-alpha)*rot1*180/3.141592;
////	pitch = alpha*(pitch + (float)himu.gyr[1]*0.004*0.001)+(1-alpha)*rot2*180/3.141592;
//
//    /* EKF */
//    EKF();
//    IOIF_WaitForASec(measure_rate);
//
//}
//
//
//void EKF()
//{
////	calculation_time = xTaskGetTickCount();
//
//	Ax = (float)himu.acc[0]/(float)65536*4*g;
//	Ay = (float)himu.acc[1]/(float)65536*4*g;
//	Az = (float)himu.acc[2]/(float)65536*4*g;
//
////	Ax = himu.acc[0]*g_k;
////	Ay = himu.acc[1]*g_k;
////	Az = himu.acc[2]*g_k;
//
////	wx = himu.gyr[0]/(float)65536*500*3.141592/180;
////	wy = himu.gyr[1]/(float)65536*500*3.141592/180;
////	wz = himu.gyr[2]/(float)65536*500*3.141592/180;
//	wx = (float)himu.gyr[0]*0.004*3.141592/180;
//	wy = (float)himu.gyr[1]*0.004*3.141592/180;
//	wz = (float)himu.gyr[2]*0.004*3.141592/180;
//	mx = (float)himu.mag[0];
//	my = (float)himu.mag[1];
//	mz = (float)himu.mag[2];
//
////	if(fabs(pitch - 3.141592/2.0) < 0.1 || fabs(pitch + 3.141592/2.0) < 0.1){
////		pitch =
////	}
//
//	// Xt+1 = F(Xt, wt)
////	x_t_datalist[0] = yaw + sinf(roll)/cosf(pitch)*wy*Ts+cosf(roll)/cosf(pitch)*wz*Ts;
//	x_t_datalist[0] = 0;
//	x_t_datalist[1] = pitch + cosf(roll)*wy*Ts-sinf(roll)*wz*Ts;
//	x_t_datalist[2] = roll + wx*Ts + sinf(roll)*tanf(pitch)*wy*Ts + cosf(roll)*tanf(pitch)*wz*Ts;
//	matrixInitializer(x_t, x_t_datalist, 3, 1);
//
//	// ft, ftT
//	f_t_datalist[0] = 1;
//	f_t_datalist[1] = sinf(roll)/cosf(pitch)*tanf(pitch)*wy*Ts + cosf(roll)/cosf(pitch)*tanf(pitch)*wz*Ts;
//	f_t_datalist[2] = cosf(roll)/cosf(pitch)*wy*Ts - sinf(roll)/cosf(pitch)*wz*Ts;
//	f_t_datalist[3] = 0;
//	f_t_datalist[4] = 1;
//	f_t_datalist[5] = -sinf(roll)*wy*Ts - cosf(roll)*wz*Ts;
//	f_t_datalist[6] = 0;
//	f_t_datalist[7] = sinf(roll)/(cosf(pitch)*cosf(pitch))*wy*Ts + cosf(roll)/(cosf(pitch)*cosf(pitch))*wz*Ts;
//	f_t_datalist[8] = 1 + cosf(roll)*tanf(pitch)*wy*Ts - sinf(roll)*tan(pitch)*wz*Ts;
//
//	for(int a = 0; a < 3; a++){
//		if(x_t_datalist[a] >= 3.141592/2.0 - 0.1){
//			x_t_datalist[a] = 3.141592/2.0 - 0.1;
//		}
//		else if(x_t_datalist[a] <= -3.141592/2.0 + 0.1){
//			x_t_datalist[a] = -3.141592/2.0 + 0.1;
//		}
//	}
//
//	yaw = x_t_datalist[0];
//	pitch = x_t_datalist[1];
//	roll = x_t_datalist[2];
//
//	matrixInitializer(f_t, f_t_datalist, 3, 3);
//	transpose(f_t_T, f_t);
//
//
//
//	// yt+1
////	y_t_datalist[0] = atan2f(-(cosf(roll)*my-sinf(roll)*mz), (cosf(pitch)*mx+sinf(roll)*sinf(pitch)*my+cosf(roll)*sinf(pitch)*mz));
//	y_t_datalist[0] = yaw;
//	y_t_datalist[1] = Ax;
//	y_t_datalist[2] = Ay;
//	y_t_datalist[3] = Az;
//	matrixInitializer(y_t, y_t_datalist, 4, 1);
//
//
//	// Ht+1
//	H_t_datalist[0] = yaw;
//	H_t_datalist[1] = -sinf(pitch)*g;
//	H_t_datalist[2] = cosf(pitch)*sinf(roll)*g;
//	H_t_datalist[3] = cosf(pitch)*cosf(roll)*g;
////	H_t_datalist[1] = sinf(pitch)*g;
////	H_t_datalist[2] = -cosf(pitch)*sinf(roll)*g;
////	H_t_datalist[3] = -cosf(pitch)*sinf(roll)*g;
//	matrixInitializer(H_t, H_t_datalist, 4, 1);
//
//	temp1 = y_t_datalist[0] - H_t_datalist[0];
//	temp2 = y_t_datalist[1] - H_t_datalist[1];
//	temp3 = y_t_datalist[2] - H_t_datalist[2];
//	temp4 = y_t_datalist[3] - H_t_datalist[3];
//
//	// Qt
////	Q_t_datalist[0] = sqrtf(wx*wx + wy*wy + wz*wz);
////	Q_t_datalist[3] = sqrtf(wx*wx + wy*wy + wz*wz);
////	Q_t_datalist[6] = sqrtf(wx*wx + wy*wy + wz*wz);
//	Q_t_datalist[0] = 0;
//	Q_t_datalist[4] = sqrtf(wx*wx + wy*wy + wz*wz)+1;
//	Q_t_datalist[8] = sqrtf(wx*wx + wy*wy + wz*wz)+1;
//	matrixInitializer(Q_t, Q_t_datalist, 3, 3);
//
//	// Pt+1 = fPfT  + Q
//	matrixMultiply(buff1, f_t, P_t); // here !!
//	matrixMultiply(P_t, buff1, f_t_T);
//	matrixSum(P_t, P_t, Q_t);
//
//
//	// ht+1, ht+1T
//	h_t_datalist[0] = 1;
//	h_t_datalist[1] = 0;
//	h_t_datalist[2] = 0;
//	h_t_datalist[3] = 0;
//	h_t_datalist[4] = -cosf(pitch)*g;
////	h_t_datalist[4] = cosf(pitch)*g;
//	h_t_datalist[5] = 0;
//	h_t_datalist[6] = 0;
//	h_t_datalist[7] = -sinf(pitch)*sinf(roll)*g;
//	h_t_datalist[8] = cosf(pitch)*cosf(roll)*g;
////	h_t_datalist[7] = sinf(pitch)*sinf(roll)*g;
////	h_t_datalist[8] = -cosf(pitch)*cosf(roll)*g;
//	h_t_datalist[9] = 0;
//	h_t_datalist[10] = -sinf(pitch)*cosf(roll)*g;
//	h_t_datalist[11] = -cosf(pitch)*sinf(roll)*g;
////	h_t_datalist[10] = sinf(pitch)*cosf(roll)*g;
////	h_t_datalist[11] = cosf(pitch)*sinf(roll)*g;
//	matrixInitializer(h_t, h_t_datalist, 4, 3);
//	transpose(h_t_T, h_t);
////	flag = 0;
//
//	// Rt
////	R_t_datalist[0] = 0;
////	R_t_datalist[4] = sqrtf(Ax*Ax + Ay*Ay + (Az-g)*(Az-g));
////	R_t_datalist[8] = 0;
////	R_t_datalist[12] = 0;
//	R_t_datalist[0] = 1;
////	R_t_datalist[0] = sqrtf(mx*mx + my*my + mz*mz);s
//	R_t_datalist[5] = sqrtf(Ax*Ax + Ay*Ay + (Az-g)*(Az-g))+1;
//	R_t_datalist[10] = sqrtf(Ax*Ax + Ay*Ay + (Az-g)*(Az-g))+1;
//	R_t_datalist[15] = sqrtf(Ax*Ax + Ay*Ay + (Az-g)*(Az-g))+1;
//
//	matrixInitializer(R_t, R_t_datalist, 4, 4);
//
//	// Kt+1 = Ph(hphT + R)-1
//	matrixMultiply(buff2, h_t, P_t);
//	matrixMultiply(buff3, buff2, h_t_T);
//	matrixSum(buff3, buff3, R_t);
//	flag = inverse(buff3, buff3);
//	for(int k = 0; k < 4; k++){
//		for(int l = 0; l < 4; l++){
//			buff3_datalist[4*k + l] = buff3->data[k][l];
//		}
//	}
//	matrixMultiply(buff4, P_t, h_t_T);
//	matrixMultiply(K_t, buff4, buff3);
//
//	for(int i = 0; i < 3; i++){
//		for(int j = 0; j < 4; j++){
//			K_t_datalist[4*i + j] = K_t->data[i][j];
//		}
//	}
//
//	// xt+1 = xt+1 + Kt+1(yt+1-Ht+1)
//	matrixSubstract(y_t, y_t, H_t);
//	matrixMultiply(buff5, K_t, y_t);
//	matrixSum(x_t, x_t, buff5);
////	matrixSubstract(x_t, x_t, buff5);
//
//	for(int b = 3; b < 3; b++){
//		if(x_t->data[b][0] >= 3.141592/2.0 - 0.1){
//			x_t->data[b][0] = 3.141592/2.0 - 0.1;
//		}
//		else if(x_t->data[b][0] <= -3.141592/2.0 + 0.1){
//			x_t->data[b][0] = -3.141592/2.0 + 0.1;
//		}
//	}
//
//	buff5_0 = buff5->data[0][0];
//	buff5_1 = buff5->data[1][0];
//	buff5_2 = buff5->data[2][0];
//
//	yaw = x_t->data[0][0];
//	pitch = x_t->data[1][0];
//	roll = x_t->data[2][0];
//
//	t_yaw = yaw;
//	t_pitch = pitch;
//	t_roll = roll;
//
//	// Pt+1 = (I-Kt+1ht+1)Pt+1
//
//
//	matrixMultiply(buff1, K_t, h_t);
//	matrixMultiply(buff6, buff1, P_t);
//	matrixSubstract(P_t, P_t, buff6);
//
////	calculation_time2 = xTaskGetTickCount() - calculation_time;
//}
//
//
//
//
//
//
//
//
//
//
//
