/*
 * mpu9250_calibrate.c
 *
 *  Created on: Jul 1, 2016
 *      Author: art
 */

#include "mpu9250_calibrate.h"
#include "math.h"


void DLP(float *val_old, float val_new, float alpha){
  if (alpha < 0){alpha = 0;}
  if (alpha > 1){alpha = 1;}
  *val_old = *val_old*(1-alpha)+val_new*alpha;
}

void PreProcess(struct Sensor_Calibrate *sc){
//  float norm;

//  float R0 = 10000.0; // 荐庇筿竟把σ放筿
//  float beta = 3950.0; // 荐庇筿竟放╰计
//  float t0 = 25.0; // 把σ放
//  float r;
//  DLP(&sc->mx,(sc->myr - MYB)/MYS, 0.5);   // myb, mys is user defined const, mx is dym data
//  DLP(&sc->my,(sc->mxr - MXB)/MXS , 0.5);
//  DLP(&sc->mz,-(sc->mzr - MZB)/MZS , 0.5);

//  sc->gx = (sc->gxr-GXB)*DEG2RAD;   // gyb etc is used defined
//  sc->gy = (sc->gyr-GYB)*DEG2RAD;
//  sc->gz = (sc->gzr-GZB)*DEG2RAD;
//
//  sc->ax = sc->axr;
//  sc->ay = sc->ayr;
//  sc->az = sc->azr;
//
//  norm = sqrt(sc->ax*sc->ax + sc->ay*sc->ay + sc->az*sc->az);
//  sc->ax /= norm;
//  sc->ay /= norm;
//  sc->az /= norm;
  sc->gx = (float)sc->gxr * 0.030517;
  sc->gy = (float)sc->gyr * 0.030517;
  sc->gz = (float)sc->gzr * 0.030517;

  sc->ax = (float)sc->axr / 835.918;
  sc->ay = (float)sc->ayr / 835.918;
  sc->az = (float)sc->azr / 835.918;

  sc->Temp = ((float) sc->Tempr - 21.0) / 333.87 + 21.0;

//  sc->Temp = ((((float) sc->Tempr) + 521) / 340.0) + 21.0;

//  r = R0 * (32767.0 / sc->Tempr - 1.0);
//  sc->Temp = 1.0 / (1.0 / (t0 + 273.15) + log(r / R0) / beta) - 273.15;
//  norm = sqrt(sc->mx*sc->mx+sc->my*sc->my+sc->mz*sc->mz);
//  sc->mx /= norm;
//  sc->my /= norm;
//  sc->mz /= norm;
}

void UpdateGDFilter_MARG(struct Sensor_Calibrate *sc){
    float q1, q2, q3, q4;         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float j_11or24, j_12or23, j_13or22, j_14or21, j32, j33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float q2q4, q1q3, q1q2, q3q4, q2q2, q3q3;

    sc->dt = (float)sc->t_elapse * 0.000001;

    q1 = sc->SEq1;
    q2 = sc->SEq2;
    q3 = sc->SEq3;
    q4 = sc->SEq4;  // global with 'SE'

    norm = sqrt(sc->axr * sc->axr + sc->ayr * sc->ayr + sc->azr * sc->azr);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    sc->axr *= norm;
    sc->ayr *= norm;
    sc->azr *= norm;

    q2q4 = q2*q4;     //q2q4 = q2*q4/10000;

    q1q3 = q1*q3;
    q1q2 = q1*q2;
    q3q4 = q3*q4;
    q2q2 = q2*q2;
    q3q3 = q3*q3;


    // Compute the objective function and Jacobian

    f1 = 2*(q2q4-q1q3)-sc->axr;
    f2 = 2*(q1q2+q3q4)-sc->ayr;
    f3 = 2*(0.5-q2q2-q3q3)-sc->azr;

    j_11or24 = 2.0*q3;
    j_12or23 = 2.0*q4;
    j_13or22 = 2.0*q1;
    j_14or21 = 2.0*q2;

    j32 = -4*q2;
    j33 = -4*q3;

    // Compute the gradient (matrix multiplication)
    hatDot1 = j_14or21 * f2 - j_11or24 * f1;
    hatDot2 = j_12or23 * f1 + j_13or22 * f2 - j32 * f3;
    hatDot3 = j_12or23 * f2 - j33 *f3 - j_13or22 * f1;
    hatDot4 = j_14or21 * f1 + j_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope biases
    gerrx = 2*(q1*hatDot2 - q2*hatDot1 - q3*hatDot4 + q4*hatDot3);
    gerry = 2*(q1*hatDot3 + q2*hatDot4 - q3*hatDot1 - q4*hatDot2);
    gerrz = 2*(q1*hatDot4 - q2*hatDot3 + q3*hatDot2 - q4*hatDot1);

    // Compute and remove gyroscope biases
    sc->wbx += gerrx * sc->dt * sc->zeta;
    sc->wby += gerry * sc->dt * sc->zeta;
    sc->wbz += gerrz * sc->dt * sc->zeta;

    sc->gxr -= sc->wbx;
    sc->gyr -= sc->wby;
    sc->gzr -= sc->wbz;

    // Compute the quaternion derivative
    qDot1 = 0.5*(-q2*sc->gxr -q3*sc->gyr -q4*sc->gzr);
    qDot2 = 0.5*(q1*sc->gxr + q3*sc->gzr -q4*sc->gyr);
    qDot3 = 0.5*(q1*sc->gyr -q2*sc->gzr + q4*sc->gxr);
    qDot4 = 0.5*(q1*sc->gzr + q2*sc->gyr -q3*sc->gxr);

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(sc->beta * hatDot1)) * sc->dt;
    q2 += (qDot2 -(sc->beta * hatDot2)) * sc->dt;
    q3 += (qDot3 -(sc->beta * hatDot3)) * sc->dt;
    q4 += (qDot4 -(sc->beta * hatDot4)) * sc->dt;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q1 *=  norm;
    q2 *= norm;
    q3 *= norm;
    q4 *= norm;

    sc->SEq1 = q1;
    sc->SEq2 = q2;
    sc->SEq3 = q3;
    sc->SEq4 = q4;
}

float getPitch(float q0, float q1, float q2, float q3) {
  float arg1 = 2*(q1*q3 - q0*q2);
  return -atan(arg1)*RAD2DEG;
}

float getYaw(float q0, float q1, float q2, float q3) {
    float arg1 = 2*(q1*q2 + q0*q3);
    float arg2 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
  return atan2(arg1,arg2)*RAD2DEG;
}

float getRoll(float q0, float q1, float q2, float q3) {
    float arg1 = 2*(q2*q3 + q0*q1);
    float arg2 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  return atan2(arg1,arg2)*RAD2DEG;
}
