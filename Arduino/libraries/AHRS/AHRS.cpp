/*
AHRS.cpp
Roman Käslin
rkaeslin@student.ethz.ch
2017-05-14

adapted from

*/

#include "AHRS.h"

AHRS::AHRS(float bx1, float by1, float bz1, float bx2, float by2, float bz2) {
    x << 1, 0, 0, 0, bx1, by1, bz1, 1, 0, 0, 0, bx2, by2, bz2;
    P << 1.0/3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1.0/3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1.0/3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1.0/3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, gyroBias, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, gyroBias, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, gyroBias, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1.0/3.0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1.0/3.0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0/3.0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0/3.0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gyroBias, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gyroBias, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gyroBias;
    Q << gyroNoise, 0, 0, 0, 0, 0,
         0, gyroNoise, 0, 0, 0, 0,
         0, 0, gyroNoise, 0, 0, 0,
         0, 0, 0, gyroNoise, 0, 0,
         0, 0, 0, 0, gyroNoise, 0,
         0, 0, 0, 0, 0, gyroNoise;
    R << accelNoise, 0, 0, 0, 0, 0, 0,
         0, accelNoise, 0, 0, 0, 0, 0,
         0, 0, accelNoise, 0, 0, 0, 0,
         0, 0, 0, accelNoise, 0, 0, 0,
         0, 0, 0, 0, accelNoise, 0, 0,
         0, 0, 0, 0, 0, accelNoise, 0,
         0, 0, 0, 0, 0, 0, encoderNoise; 
    I = Eigen::MatrixXf::Identity(14,14);

    x2 << 1, 0, 0, 0, 1, 0, 0, 0;
    P2 << 1.0/3.0, 0, 0, 0, 0, 0, 0, 0,
          0, 1.0/3.0, 0, 0, 0, 0, 0, 0,
          0, 0, 1.0/3.0, 0, 0, 0, 0, 0, 
          0, 0, 0, 1.0/3.0, 0, 0, 0, 0, 
          0, 0, 0, 0, 1.0/3.0, 0, 0, 0,
          0, 0, 0, 0, 0, 1.0/3.0, 0, 0,
          0, 0, 0, 0, 0, 0, 1.0/3.0, 0,
          0, 0, 0, 0, 0, 0, 0, 1.0/3.0;
    I2 = Eigen::MatrixXf::Identity(8,8);
    bias1 << 0, bx1, by1, bz1;
    bias2 << 0, bx2, by2, bz2;

    lastUpdate = micros();
}

void AHRS::EKFupdate(float* ax1, float* ay1, float* az1, float* gx1, float* gy1, float* gz1, float* ax2, float* ay2, float* az2, float* gx2, float* gy2, float* gz2, float* psi) {
  // calculate magnitude of measured accelerations
  float a1 = sqrt(*ax1 * *ax1 + *ay1 * *ay1 + *az1 * *az1);
  float a2 = sqrt(*ax2 * *ax2 + *ay2 * *ay2 + *az2 * *az2);

  //if ( (fabs(a1-G) < G) && (fabs(a2-G) < G) ) {
    // Prediction
    // x_k+1 = x_k + 0.5*Quat(x_k)*(omega-bias)*delta_t

    // quaternion of IMU 1 (footsole) in matrix form
    Eigen::MatrixXf Quat1(4,4);
    Quat1 << x(0), -x(1), -x(2), -x(3), 
             x(1),  x(0), -x(3),  x(2), 
             x(2),  x(3),  x(0), -x(1), 
             x(3), -x(2),  x(1),  x(0);

    // quaternion of IMU 2 (shank) in matrix form
    Eigen::MatrixXf Quat2(4,4);
    Quat2 <<  x(7), -x(8),  -x(9), -x(10), 
              x(8),  x(7), -x(10),   x(9), 
              x(9), x(10),   x(7),  -x(8), 
             x(10), -x(9),   x(8),   x(7);

    // prediction update for quaternion of IMU 1 (footsole)
    omega1 << 0, *gx1, *gy1, *gz1;
    bias1 << 0, x(4), x(5), x(6);
    u1 = 0.5 * Quat1 * (omega1 - bias1) * sampleTime;

    // prediction update for quaternion of IMU 2 (shank)
    omega2 << 0, *gx2, *gy2, *gz2;
    bias2 << 0, x(11), x(12), x(13);
    u2 = 0.5 * Quat2 * (omega2 - bias2) * sampleTime;

    // calculate linearized jacobian matrix A = d (x_k + 0.5*Quat(x_k)*(omega-bias)*delta_t)/ dx_k
    float t = sampleTime / 2.0;
    Eigen::VectorXf d(6);
    d << bias1.block<3,1>(1,0) - omega1.block<3,1>(1,0), bias2.block<3,1>(1,0) - omega2.block<3,1>(1,0);

    A <<       1,  t*d(0),  t*d(1),  t*d(2),  t*x(1),  t*x(2),  t*x(3),       0,       0,       0,       0,        0,        0,        0,
         -t*d(0),       1, -t*d(2),  t*d(1), -t*x(0),  t*x(3), -t*x(2),       0,       0,       0,       0,        0,        0,        0,
         -t*d(1),  t*d(2),       1, -t*d(0), -t*x(3), -t*x(0),  t*x(1),       0,       0,       0,       0,        0,        0,        0,
         -t*d(2), -t*d(1),  t*d(0),       1,  t*x(2), -t*x(1), -t*x(0),       0,       0,       0,       0,        0,        0,        0,
               0,       0,       0,       0,       1,       0,       0,       0,       0,       0,       0,        0,        0,        0,
               0,       0,       0,       0,       0,       1,       0,       0,       0,       0,       0,        0,        0,        0,
               0,       0,       0,       0,       0,       0,       1,       0,       0,       0,       0,        0,        0,        0,
               0,       0,       0,       0,       0,       0,       0,       1,  t*d(3),  t*d(4),  t*d(5),   t*x(8),   t*x(9),  t*x(10),
               0,       0,       0,       0,       0,       0,       0, -t*d(3),       1, -t*d(5),  t*d(4),  -t*x(7),  t*x(10),  -t*x(9),
               0,       0,       0,       0,       0,       0,       0, -t*d(4),  t*d(5),       1, -t*d(3), -t*x(10),  -t*x(7),   t*x(8),
               0,       0,       0,       0,       0,       0,       0, -t*d(5), -t*d(4),  t*d(3),       1,   t*x(9),  -t*x(8),  -t*x(7),
               0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,        1,        0,        0,
               0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,        0,        1,        0,
               0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,        0,        0,        1;
    // A <<       1,  t*x(4),  t*x(5),  t*x(6),  t*x(1),  t*x(2),  t*x(3),        0,        0,        0,        0,        0,        0,        0,
    //      -t*x(4),       1, -t*x(6),  t*x(5), -t*x(0),  t*x(3), -t*x(2),        0,        0,        0,        0,        0,        0,        0,
    //      -t*x(5),  t*x(6),       1, -t*x(4), -t*x(3), -t*x(0),  t*x(1),        0,        0,        0,        0,        0,        0,        0,
    //      -t*x(6), -t*x(5),  t*x(4),       1,  t*x(2), -t*x(1), -t*x(0),        0,        0,        0,        0,        0,        0,        0,
    //            0,       0,       0,       0,       1,       0,       0,        0,        0,        0,        0,        0,        0,        0,
    //            0,       0,       0,       0,       0,       1,       0,        0,        0,        0,        0,        0,        0,        0,
    //            0,       0,       0,       0,       0,       0,       1,        0,        0,        0,        0,        0,        0,        0,
    //            0,       0,       0,       0,       0,       0,       0,        1,  t*x(11),  t*x(12),  t*x(13),   t*x(8),   t*x(9),  t*x(10),
    //            0,       0,       0,       0,       0,       0,       0, -t*x(11),        1, -t*x(13),  t*x(12),  -t*x(7),  t*x(10),  -t*x(9),
    //            0,       0,       0,       0,       0,       0,       0, -t*x(12),  t*x(13),        1, -t*x(11), -t*x(10),  -t*x(7),   t*x(8),
    //            0,       0,       0,       0,       0,       0,       0, -t*x(13), -t*x(12),  t*x(11),        1,   t*x(9),  -t*x(8),  -t*x(7),
    //            0,       0,       0,       0,       0,       0,       0,        0,        0,        0,        0,        1,        0,        0,
    //            0,       0,       0,       0,       0,       0,       0,        0,        0,        0,        0,        0,        1,        0,
    //            0,       0,       0,       0,       0,       0,       0,        0,        0,        0,        0,        0,        0,        1;

    // calculate the covariance of the gyroscope noise in the direction of the quaternions Q*R*transpose(Q), set the bias noise to 0
    L <<         Quat1.block<4,3>(0,1), Eigen::MatrixXf::Zero(4,3),
            Eigen::MatrixXf::Zero(3,6),
            Eigen::MatrixXf::Zero(4,3),      Quat2.block<4,3>(0,1),
            Eigen::MatrixXf::Zero(3,6);
    LQL << L.block<4,3>(0,0)*Q.block<3,3>(0,0)*(L.block<4,3>(0,0).transpose()), Eigen::MatrixXf::Zero(4,10),
           Eigen::MatrixXf::Zero(3,14),
           Eigen::MatrixXf::Zero(4,7),                                          L.block<4,3>(7,3)*Q.block<3,3>(3,3)*(L.block<4,3>(7,3).transpose()), Eigen::MatrixXf::Zero(4,3),
           Eigen::MatrixXf::Zero(3,14);

    APA << A.block<7,7>(0,0) * P.block<7,7>(0,0) * (A.block<7,7>(0,0).transpose()), A.block<7,7>(0,0) * P.block<7,7>(0,7) * (A.block<7,7>(7,7).transpose()),
           A.block<7,7>(7,7) * P.block<7,7>(7,0) * (A.block<7,7>(0,0).transpose()), A.block<7,7>(7,7) * P.block<7,7>(7,7) * (A.block<7,7>(7,7).transpose());

    // update the covarinace matrix with prediction
    // P = A * P * A.transpose() + 0.25 * L * Q * L.transpose();
    P = APA + 0.25 * LQL;

    // update states with prediction
    x(0) = x(0) + u1(0);
    x(1) = x(1) + u1(1);
    x(2) = x(2) + u1(2);
    x(3) = x(3) + u1(3);
    //x(4) = x(4);
    //x(5) = x(5);
    //x(6) = x(6);
    x(7)  =  x(7) + u2(0);
    x(8)  =  x(8) + u2(1);
    x(9)  =  x(9) + u2(2);
    x(10) = x(10) + u2(3);
    //x(11) = x(11);
    //x(12) = x(12);
    //x(13) = x(13);

    // Normalise quaternion of IMU 1 (foothold)
    float recipNorm = invSqrt(x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3));
    x(0) *= recipNorm;
    x(1) *= recipNorm;
    x(2) *= recipNorm;
    x(3) *= recipNorm;

    // Normalise quaternion of IMU 2 (shank)
    recipNorm = invSqrt(x(7) * x(7) + x(8) * x(8) + x(9) * x(9) + x(10) * x(10));
    x(7)  *= recipNorm;
    x(8)  *= recipNorm;
    x(9)  *= recipNorm;
    x(10) *= recipNorm;


    // Measurement Update
    // check if external accelerations affect the acceleration measurements 
    if ( (fabs(a1-G) < 0.1*G) && (fabs(a2-G) < 0.1*G) ) { // if only small external accelerations occur, perform the measurement update with the measured accelerations
      z << *ax1 / a1, *ay1 / a1, *az1 / a1, *ax2 / a2, *ay2 / a2, *az2 / a2, *psi;
    }
    else {  // avoid measurement update of acceleration by providing prediction of acceleration as measurement
      z << 2*(x(1)*x(3) - x(0)*x(2)), 2*(x(2)*x(3) + x(0)*x(1)), x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3),
           2*(x(8)*x(10) - x(7)*x(9)), 2*(x(9)*x(10) + x(7)*x(8)), x(7)*x(7) - x(8)*x(8) - x(9)*x(9) + x(10)*x(10),
           *psi;
    }

    // calculate the relative quaternion between IMU1 (footsole) and IMU2 (shank), q1*inv(q2) = qr
    float q0r = x(0)*x(7) + x(1)*x(8) + x(2)*x(9) + x(3)*x(10);
    float q1r = -x(0)*x(8) + x(1)*x(7) + x(2)*x(10) - x(3)*x(9);
    float q2r = -x(0)*x(9) - x(1)*x(10) + x(2)*x(7) + x(3)*x(8);
    float q3r = -x(0)*x(10) + x(1)*x(9) - x(2)*x(8) + x(3)*x(7);
    
    // predict the measurements, for acceleration z direction rotated with quaternion z_q = inv(q)*z_w*q, for angle psi of qr
    h <<  2*(x(1)*x(3) - x(0)*x(2)), 2*(x(2)*x(3) + x(0)*x(1)), x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3),
          2*(x(8)*x(10) - x(7)*x(9)), 2*(x(9)*x(10) + x(7)*x(8)), x(7)*x(7) - x(8)*x(8) - x(9)*x(9) + x(10)*x(10),
          -atan2(2*(q0r*q3r+q1r*q2r), (q0r*q0r + q1r*q1r - q2r*q2r - q3r*q3r));
          
    // calculate products nexessary to compute the linearized jacobian matrix H
    float pr  =  q0r*q0r + q1r*q1r - q2r*q2r - q3r*q3r;
    float p1  =  x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3);
    float p2  =  x(7)*x(7) + x(8)*x(8) - x(9)*x(9) - x(10)*x(10);
    float p2_0_12_3 = x(7)*x(7) - x(8)*x(8) + x(9)*x(9) - x(10)*x(10);

    float fr_0312  =    q0r*q3r +    q1r*q2r; 
    float f1_0312  =  x(0)*x(3) +  x(1)*x(2);
    float f1_02_13 =  x(0)*x(2) -  x(1)*x(3);
    float f2_0312  = x(7)*x(10) +  x(8)*x(9);
    float f2_03_12 = x(7)*x(10) -  x(8)*x(9);
    float f2_02_13 =  x(7)*x(9) - x(8)*x(10);
    float f2_0123  =  x(7)*x(8) + x(9)*x(10);

    float d01 = 4*( 2*f2_02_13*x(2) + 2*f2_0312*x(3) + p2*x(0))*fr_0312 + 2*( 2*f2_0123*x(2) + 2*f2_03_12*x(0) - p2_0_12_3*x(3))*pr;
    float d11 = 4*(-2*f2_02_13*x(3) + 2*f2_0312*x(2) + p2*x(1))*fr_0312 + 2*(-2*f2_0123*x(3) + 2*f2_03_12*x(1) - p2_0_12_3*x(2))*pr;
    float d21 = 4*( 2*f2_02_13*x(0) + 2*f2_0312*x(1) - p2*x(2))*fr_0312 + 2*( 2*f2_0123*x(0) - 2*f2_03_12*x(2) - p2_0_12_3*x(1))*pr;
    float d31 = 4*(-2*f2_02_13*x(1) + 2*f2_0312*x(0) - p2*x(3))*fr_0312 + 2*(-2*f2_0123*x(1) - 2*f2_03_12*x(3) - p2_0_12_3*x(0))*pr;
    
    float d02 = 4*(  2*f1_02_13*x(9) + 2*f1_0312*x(10) +  p1*x(7))*fr_0312 + 2*( 2*f1_02_13*x(8) -  2*f1_0312*x(7) + p1*x(10))*pr;
    float d12 = 4*(-2*f1_02_13*x(10) +  2*f1_0312*x(9) +  p1*x(8))*fr_0312 + 2*( 2*f1_02_13*x(7) +  2*f1_0312*x(8) - p1*x(9))*pr;
    float d22 = 4*(  2*f1_02_13*x(7) +  2*f1_0312*x(8) -  p1*x(9))*fr_0312 + 2*(2*f1_02_13*x(10) -  2*f1_0312*x(9) - p1*x(8))*pr;
    float d32 = 4*( -2*f1_02_13*x(8) +  2*f1_0312*x(7) - p1*x(10))*fr_0312 + 2*( 2*f1_02_13*x(9) + 2*f1_0312*x(10) + p1*x(7))*pr;
    
    float no = 4*fr_0312*fr_0312 + pr*pr;

    // calculate linearized jacobian matrix H = d ([ q1*z_w*inv(q1), q2*z_w*inv(q2), -atan( 2*(q0r*q3r+q1r*q2r)/(q0r*q0r + q1r*q1r - q2r*q2r - q3r*q3r) ) ]) / dx_k
    H << -2*x(2),  2*x(3), -2*x(0),  2*x(1), 0, 0, 0,       0,       0,       0,       0, 0, 0, 0,
          2*x(1),  2*x(0),  2*x(3),  2*x(2), 0, 0, 0,       0,       0,       0,       0, 0, 0, 0,
          2*x(0), -2*x(1), -2*x(2),  2*x(3), 0, 0, 0,       0,       0,       0,       0, 0, 0, 0,
               0,       0,       0,       0, 0, 0, 0, -2*x(9), 2*x(10), -2*x(7),  2*x(8), 0, 0, 0,
               0,       0,       0,       0, 0, 0, 0,  2*x(8),  2*x(7), 2*x(10),  2*x(9), 0, 0, 0,
               0,       0,       0,       0, 0, 0, 0,  2*x(7), -2*x(8), -2*x(9), 2*x(10), 0, 0, 0,
          d01/no,  d11/no,  d21/no,  d31/no, 0, 0, 0,  d02/no,  d12/no,  d22/no,  d32/no, 0, 0, 0;

    // calculate the Kalman gain matrix K
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // update the covariance matrix with measurement update
    P = (I - K * H) * P;

    // update states with measurement update (and make sure that encoder estimate and measuement stay within [-pi, pi])
    Eigen::MatrixXf diff(7,1);
    diff = z - h;
    if (diff(6) > M_PI) { diff(6) -= 2*M_PI; }
    if (diff(6) < -M_PI) { diff(6) += 2*M_PI; }
    x = x + K * diff;

    // Normalise quaternion of IMU 1 (footsole)
    recipNorm = invSqrt(x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3));
    x(0) *= recipNorm;
    x(1) *= recipNorm;
    x(2) *= recipNorm;
    x(3) *= recipNorm;

    // Normalise quaternion of IMU 2 (shank)
    recipNorm = invSqrt(x(7) * x(7) + x(8) * x(8) + x(9) * x(9) + x(10) * x(10));
    x(7)  *= recipNorm;
    x(8)  *= recipNorm;
    x(9)  *= recipNorm;
    x(10) *= recipNorm;
  //}
}

void AHRS::getQEKF(float* q1, float* q2, float* ax1, float* ay1, float* az1, float* gx1, float* gy1, float* gz1, float* ax2, float* ay2, float* az2, float* gx2, float* gy2, float* gz2, float* psi) {  
  now = micros();
  sampleTime = ((float)(now - lastUpdate)) / 1000000.0;
  lastUpdate = now;

  EKFupdate(ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2, psi);
  q1[0] = x(0);
  q1[1] = x(1);
  q1[2] = x(2);
  q1[3] = x(3);

  q2[0] = x(7);
  q2[1] = x(8);
  q2[2] = x(9);
  q2[3] = x(10);
}

void AHRS::CompUpdate(float* ax1, float* ay1, float* az1, float* gx1, float* gy1, float* gz1, float* ax2, float* ay2, float* az2, float* gx2, float* gy2, float* gz2, float* psi) {
  // calculate magnitude of measured accelerations
  float a1 = sqrt(*ax1 * *ax1 + *ay1 * *ay1 + *az1 * *az1);
  float a2 = sqrt(*ax2 * *ax2 + *ay2 * *ay2 + *az2 * *az2);

  //if ( (fabs(a1-G) < G) && (fabs(a2-G) < G) ) {
    // Prediction
    // x_k+1 = x_k + 0.5*Quat(x_k)*(omega-bias)*delta_t = x_k+1 = x_k + 0.5*(omega-bias)*q(x_k)*delta_t

    // quaternion of IMU 1 (footsole) in matrix form
    Eigen::MatrixXf Quat1(4,4);
    Quat1 << x2(0), -x2(1), -x2(2), -x2(3), 
             x2(1),  x2(0), -x2(3),  x2(2), 
             x2(2),  x2(3),  x2(0), -x2(1), 
             x2(3), -x2(2),  x2(1),  x2(0);

    // quaternion of IMU 2 (shank) in matrix form
    Eigen::MatrixXf Quat2(4,4);
    Quat2 <<  x2(4), -x2(5),  -x2(6),  -x2(7), 
              x2(5),  x2(4),  -x2(7),   x2(6), 
              x2(6),  x2(7),   x2(4),  -x2(5), 
              x2(7), -x2(6),   x2(5),   x2(4);

    // prediction update for quaternion of IMU 1 (footsole)
    omega1 << 0, *gx1, *gy1, *gz1;
    u1 = 0.5 * Quat1 * (omega1 - bias1) * sampleTime;

    // prediction update for quaternion of IMU 2 (shank)
    omega2 << 0, *gx2, *gy2, *gz2;
    u2 = 0.5 * Quat2 * (omega2 - bias2) * sampleTime;

    // update states with prediction
    x2(0) = x2(0) + u1(0);
    x2(1) = x2(1) + u1(1);
    x2(2) = x2(2) + u1(2);
    x2(3) = x2(3) + u1(3);
    x2(4) = x2(4) + u2(0);
    x2(5) = x2(5) + u2(1);
    x2(6) = x2(6) + u2(2);
    x2(7) = x2(7) + u2(3);

    // Normalise quaternion of IMU 1 (foothold)
    float recipNorm = invSqrt(x2(0) * x2(0) + x2(1) * x2(1) + x2(2) * x2(2) + x2(3) * x2(3));
    x2(0) *= recipNorm;
    x2(1) *= recipNorm;
    x2(2) *= recipNorm;
    x2(3) *= recipNorm;

    // Normalise quaternion of IMU 2 (shank)
    recipNorm = invSqrt(x2(4) * x2(4) + x2(5) * x2(5) + x2(6) * x2(6) + x2(7) * x2(7));
    x2(4) *= recipNorm;
    x2(5) *= recipNorm;
    x2(6) *= recipNorm;
    x2(7) *= recipNorm;


    // Measurement Update
    
    // check if external accelerations affect the acceleration measurements 
    if ( (a1 != 0) && (fabs(a1-G) < 0.1*G) && (fabs(a2-G) < 0.1*G) ) { // if only small external accelerations occur, perform the measurement update with the measured accelerations
      float q1[4] = {x2(0), x2(1), x2(2), x2(3)};
      float q2[4] = {x2(4), x2(5), x2(6), x2(7)};

      Eigen::VectorXf acc1(3);
      acc1 <<  *ax1/a1, *ay1/a1, *az1/a1;
      Eigen::VectorXf acc2(3);
      acc2 <<  *ax2/a2, *ay2/a2, *az2/a2;
      
      Eigen::MatrixXf Rot1(3,3);
      Eigen::VectorXf g1(3);
      quatToRotMat(q1, Rot1);
      g1 = Rot1.transpose()*acc1;

      Eigen::MatrixXf Rot2(3,3);
      Eigen::VectorXf g2(3);
      quatToRotMat(q2, Rot2);
      g2 = Rot2.transpose()*acc2;
  
      float qI[4] = {1,0,0,0};
      float qcorr1[4] = {sqrt((g1(2)+1)/2), g1(1)/sqrt(2*(g1(2)+1)), -g1(0)/sqrt(2*(g1(2)+1)), 0};

      float f1 = sin((1-alphag_)*qcorr1[0])/sin(qcorr1[0]);
      float f2 = sin(alphag_*qcorr1[0])/sin(qcorr1[0]);
      for (int j=0; j<4; j++) {
        qcorr1[j] = f1*qI[j] + f2*qcorr1[j];
      }

      float qup1[4];
      quatMult(q1, qcorr1, qup1);

      float qcorr2[4] = {sqrt((g2(2)+1)/2), g2(1)/sqrt(2*(g2(2)+1)), -g2(0)/sqrt(2*(g2(2)+1)), 0};

      f1 = sin((1-alphag_)*qcorr2[0])/sin(qcorr2[0]);
      f2 = sin(alphag_*qcorr2[0])/sin(qcorr2[0]);
      for (int j=0; j<4; j++) {
        qcorr2[j] = f1*qI[j] + f2*qcorr2[j];
      }

      float qup2[4];
      quatMult(q2, qcorr2, qup2);

      for (int j=0; j<4; j++) {
        x2(j) = qup1[j];
      }
      for (int j=4; j<8; j++) {
        x2(j) = qup2[j-4];
      }

      // Normalise quaternion of IMU 1 (foothold)
      recipNorm = invSqrt(x2(0) * x2(0) + x2(1) * x2(1) + x2(2) * x2(2) + x2(3) * x2(3));
      x2(0) *= recipNorm;
      x2(1) *= recipNorm;
      x2(2) *= recipNorm;
      x2(3) *= recipNorm;

      // Normalise quaternion of IMU 2 (shank)
      recipNorm = invSqrt(x2(4) * x2(4) + x2(5) * x2(5) + x2(6) * x2(6) + x2(7) * x2(7));
      x2(4) *= recipNorm;
      x2(5) *= recipNorm;
      x2(6) *= recipNorm;
      x2(7) *= recipNorm;
    }
    
    if ( (fabs(x2(4)*x2(4)+x2(7)*x2(7)-x2(5)*x2(5)-x2(6)*x2(6)) > 0.5) && (fabs(x2(0)*x2(0)+x2(3)*x2(3)-x2(1)*x2(1)-x2(2)*x2(2)) > 0.5) ) {
      float q1[4] = {x2(0), x2(1), x2(2), x2(3)};
      float q2[4] = {x2(4), x2(5), x2(6), x2(7)};

      float q21[4];
      float invq2[4];
      invertQuat(q2, invq2);
      quatMult(q1, invq2, q21);

      float epsi = -atan2(2*(q21[0]*q21[3] + q21[1]*q21[2]), (q21[0]*q21[0] + q21[1]*q21[1] - q21[2]*q21[2] - q21[3]*q21[3]));
      float deltapsi = *psi - epsi;
      // float halfdeltaqenc1[4] = {cos(-deltapsi/4), 0, 0, sin(-deltapsi/4)};
      // float halfdeltaqenc2[4] = {cos(deltapsi/4), 0, 0, sin(deltapsi/4)};
      float qI[4] = {1,0,0,0};
      float deltaqenc2[4] = {cos(deltapsi/2), 0, 0, sin(deltapsi/2)};

      float f1 = sin((1-alphaenc_)*deltaqenc2[0])/sin(deltaqenc2[0]);
      float f2 = sin(alphaenc_*deltaqenc2[0])/sin(deltaqenc2[0]);
      for (int j=0; j<4; j++) {
        deltaqenc2[j] = f1*qI[j] + f2*deltaqenc2[j];
      }

      // float qenc1[4];
      // float invq21[4];
      // invertQuat(q21, invq21);
      // quatMult(halfdeltaqenc1, invq21, halfdeltaqenc1);
      // quatMult(q21, halfdeltaqenc1, halfdeltaqenc1);
      // quatMult(halfdeltaqenc1, q1, qenc1);

      float qenc2[4];
      // quatMult(halfdeltaqenc2, q2, qenc2);
      quatMult(deltaqenc2, q2, qenc2);

      // Eigen::MatrixXf diff(8,1);
      // for (int j=0; j<4; j++) {
      //   diff(j) = qenc1[j] - q1[j];
      // }
      for (int j=4; j<8; j++) {
        x2(j) = qenc2[j-4];
      }
      // x2 += diff;

      // Normalise quaternion of IMU 1 (foothold)
      recipNorm = invSqrt(x2(0) * x2(0) + x2(1) * x2(1) + x2(2) * x2(2) + x2(3) * x2(3));
      x2(0) *= recipNorm;
      x2(1) *= recipNorm;
      x2(2) *= recipNorm;
      x2(3) *= recipNorm;

      // Normalise quaternion of IMU 2 (shank)
      recipNorm = invSqrt(x2(4) * x2(4) + x2(5) * x2(5) + x2(6) * x2(6) + x2(7) * x2(7));
      x2(4) *= recipNorm;
      x2(5) *= recipNorm;
      x2(6) *= recipNorm;
      x2(7) *= recipNorm;
    }
  //}
}

void AHRS::getComp(float* q1, float* q2, float* ax1, float* ay1, float* az1, float* gx1, float* gy1, float* gz1, float* ax2, float* ay2, float* az2, float* gx2, float* gy2, float* gz2, float* psi) {  
  now = micros();
  sampleTime = ((float)(now - lastUpdate)) / 1000000.0;
  lastUpdate = now;

  CompUpdate(ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2, psi);
  q1[0] = x2(0);
  q1[1] = x2(1);
  q1[2] = x2(2);
  q1[3] = x2(3);

  q2[0] = x2(4);
  q2[1] = x2(5);
  q2[2] = x2(6);
  q2[3] = x2(7);
}

float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}
