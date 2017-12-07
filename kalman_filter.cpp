#include "kalman_filter.h"

void TimeUpdate(const Matrix<1> &u, Matrix<1> &xhat, Matrix<1, 1> &Phat)
{
  // Tuning parameter
  float arrayQ[1][1]{{1e-4}};
  Matrix<1, 1> Q = arrayQ;

  // Sampling rate in seconds
  float Ts = 0.01;

  // System A&B-matrix
  float arrayA[1][1]{{1}};
  Matrix<1, 1> A = arrayA;
  float arrayB[1][1]{{Ts}};
  Matrix<1, 1> B = arrayB;

  // Evaluate discrete-time system dynamics
  xhat = A*xhat + B*u;

  // Update state covariance: P = APAt + Q
  Phat = A * Phat * A.Transpose() + Q;
}

void MeasurementUpdate(const Matrix<1> &y, Matrix<1> &xhat, Matrix<1, 1> &Phat)
{
  // Tuning parameter
  float arrayR[1][1]{{5e-2}};
  Matrix<1, 1> R = arrayR;

  // Compute Jacobian of measurement equation
  float arrayC[1][1]{{-1}};
  Matrix<1, 1> C = arrayC;

  // Compute innovation
  Matrix<1> nu = y;
  nu -= C*xhat;

  // Compute innovation covariance
  Matrix<1, 1> S = C * Phat * C.Transpose() + R;

  // Compute optimal Kalman filter gain
  Matrix<1, 1> L = Phat * C.Transpose() * S.Inverse();

  // Compute corrected system state estimate
  xhat += L * nu;

  // Compute corrected state estimate covariance
  Identity<1, 1> eye;
  Phat = (eye - L * C) * Phat;
}