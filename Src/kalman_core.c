/*
 * kalman_core.c
 *
 *  Created on: Jan 3, 2026
 *      Author: SAIED
 */
#include <kalman_core.h>
#include <arm_math.h>
#include "kalman_core_params_defaults.h"
#include "cf_assert.h"

static float A[KC_STATE_DIM][KC_STATE_DIM];

//float gx,gy,gz,ax,ay,az;



// #define DEBUG_STATE_CHECK

/**
 * Supporting and utility functions
 */

#ifdef DEBUG_STATE_CHECK
static void assertStateNotNaN(const kalmanCoreData_t* this) {
  if ((isnan(this->S[KC_STATE_X])) ||
      (isnan(this->S[KC_STATE_Y])) ||
      (isnan(this->S[KC_STATE_Z])) ||
      (isnan(this->S[KC_STATE_PX])) ||
      (isnan(this->S[KC_STATE_PY])) ||
      (isnan(this->S[KC_STATE_PZ])) ||
      (isnan(this->S[KC_STATE_D0])) ||
      (isnan(this->S[KC_STATE_D1])) ||
      (isnan(this->S[KC_STATE_D2])) ||
      (isnan(this->q[0])) ||
      (isnan(this->q[1])) ||
      (isnan(this->q[2])) ||
      (isnan(this->q[3])))
  {
    ASSERT(false);
  }

  for(int i=0; i<KC_STATE_DIM; i++) {
    for(int j=0; j<KC_STATE_DIM; j++)
    {
      if (isnan(this->P[i][j]))
      {
        ASSERT(false);
      }
    }
  }
}
#else
static void assertStateNotNaN(const kalmanCoreData_t* this)
{
  return;
}
#endif


// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Small number epsilon, to prevent dividing by zero
#define EPS (1e-6f)

__attribute__((used))
void kalmanCoreDefaultParams(kalmanCoreParams_t* params)
{
  *params = (kalmanCoreParams_t){
    KALMAN_CORE_DEFAULT_PARAMS_INIT
  };
}




void kalmanCoreInit(kalmanCoreData_t *this, const kalmanCoreParams_t *params, const uint32_t nowMs)
{
  // Reset all data to 0 (like upon system reset)
  memset(this, 0, sizeof(kalmanCoreData_t));

  this->S[KC_STATE_X] = params->initialX;
  this->S[KC_STATE_Y] = params->initialY;
  this->S[KC_STATE_Z] = params->initialZ;
//  this->S[KC_STATE_PX] = 0;
//  this->S[KC_STATE_PY] = 0;
//  this->S[KC_STATE_PZ] = 0;
//  this->S[KC_STATE_D0] = 0;
//  this->S[KC_STATE_D1] = 0;
//  this->S[KC_STATE_D2] = 0;

  // reset the attitude quaternion
  this->initialQuaternion[0] = arm_cos_f32(params->initialYaw / 2);
  this->initialQuaternion[1] = 0.0;
  this->initialQuaternion[2] = 0.0;
  this->initialQuaternion[3] = arm_sin_f32(params->initialYaw / 2);
  for (int i = 0; i < 4; i++) { this->q[i] = this->initialQuaternion[i]; }

  // then set the initial rotation matrix to the identity. This only affects
  // the first prediction step, since in the finalization, after shifting
  // attitude errors into the attitude state, the rotation matrix is updated.
  for(int i=0; i<3; i++) { for(int j=0; j<3; j++) { this->R[i][j] = i==j ? 1 : 0; }}

  for (int i=0; i< KC_STATE_DIM; i++) {
    for (int j=0; j < KC_STATE_DIM; j++) {
      this->P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }

  // initialize state variances
  this->P[KC_STATE_X][KC_STATE_X]  = powf(params->stdDevInitialPosition_xy, 2);
  this->P[KC_STATE_Y][KC_STATE_Y]  = powf(params->stdDevInitialPosition_xy, 2);
  this->P[KC_STATE_Z][KC_STATE_Z]  = powf(params->stdDevInitialPosition_z, 2);

  this->P[KC_STATE_PX][KC_STATE_PX] = powf(params->stdDevInitialVelocity, 2);
  this->P[KC_STATE_PY][KC_STATE_PY] = powf(params->stdDevInitialVelocity, 2);
  this->P[KC_STATE_PZ][KC_STATE_PZ] = powf(params->stdDevInitialVelocity, 2);

  this->P[KC_STATE_D0][KC_STATE_D0] = powf(params->stdDevInitialAttitude_rollpitch, 2);
  this->P[KC_STATE_D1][KC_STATE_D1] = powf(params->stdDevInitialAttitude_rollpitch, 2);
  this->P[KC_STATE_D2][KC_STATE_D2] = powf(params->stdDevInitialAttitude_yaw, 2);

  this->Pm.numRows = KC_STATE_DIM;
  this->Pm.numCols = KC_STATE_DIM;
  this->Pm.pData = (float*)this->P;

  this->baroReferenceHeight = 0.0;

  this->isUpdated = 0;//false;/////////////////////////////////////////////////////////////////////////////////////////////////////////
  this->lastPredictionMs = nowMs;
  this->lastProcessNoiseUpdateMs = nowMs;
}





//static
void predictDt(kalmanCoreData_t* this, const kalmanCoreParams_t *params, Axis3f *acc, Axis3f *gyro, float dt, uint8_t quadIsFlying)
{


	// The linearized update matrix
	//
	static __attribute__((aligned(4))) arm_matrix_instance_f32 Am = { KC_STATE_DIM, KC_STATE_DIM, (float *)A}; // linearized dynamics for covariance update;

	// Temporary matrices for the covariance updates
	static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

    static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

    float dt2 = dt*dt;



								//static __attribute__((aligned(4))) arm_matrix_instance_f32 Pm = { KC_STATE_DIM, KC_STATE_DIM, (float*)this->P};
								this->Pm.numRows = KC_STATE_DIM;
								this->Pm.numCols = KC_STATE_DIM;
								this->Pm.pData = (float*)this->P;

    ///////////////////////////////////////////////////////////////////////////////

    this->P[KC_STATE_X][KC_STATE_X] = 1;
    this->P[KC_STATE_Y][KC_STATE_Y] = 1;
    this->P[KC_STATE_Z][KC_STATE_Z] = 1;

    this->P[KC_STATE_PX][KC_STATE_PX] = 1;
    this->P[KC_STATE_PY][KC_STATE_PY] = 1;
    this->P[KC_STATE_PZ][KC_STATE_PZ] = 1;

    this->P[KC_STATE_D0][KC_STATE_D0] = 1;
    this->P[KC_STATE_D1][KC_STATE_D1] = 1;
    this->P[KC_STATE_D2][KC_STATE_D2] = 1;

    for (int i = 0; i < KC_STATE_DIM; i++) {
        for (int j = 0; j < KC_STATE_DIM; j++) {
            this->P[i][j] = 1.0f;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////


    // ====== DYNAMICS LINEARIZATION ======
    // Initialize as the identity
    A[KC_STATE_X][KC_STATE_X] = 1;
    A[KC_STATE_Y][KC_STATE_Y] = 1;
    A[KC_STATE_Z][KC_STATE_Z] = 1;

    A[KC_STATE_PX][KC_STATE_PX] = 1;
    A[KC_STATE_PY][KC_STATE_PY] = 1;
    A[KC_STATE_PZ][KC_STATE_PZ] = 1;

    A[KC_STATE_D0][KC_STATE_D0] = 1;
    A[KC_STATE_D1][KC_STATE_D1] = 1;
    A[KC_STATE_D2][KC_STATE_D2] = 1;

    // position from body-frame velocity
    A[KC_STATE_X][KC_STATE_PX] = 3;//this->R[0][0]*dt;
    A[KC_STATE_Y][KC_STATE_PX] = 7;//this->R[1][0]*dt;
    A[KC_STATE_Z][KC_STATE_PX] = 0;//this->R[2][0]*dt;

    A[KC_STATE_X][KC_STATE_PY] = 4;//this->R[0][1]*dt;
    A[KC_STATE_Y][KC_STATE_PY] = 3;//this->R[1][1]*dt;
    A[KC_STATE_Z][KC_STATE_PY] = 0;//this->R[2][1]*dt;

    A[KC_STATE_X][KC_STATE_PZ] = 3;//this->R[0][2]*dt;
    A[KC_STATE_Y][KC_STATE_PZ] = 0;//this->R[1][2]*dt;
    A[KC_STATE_Z][KC_STATE_PZ] = 0;//this->R[2][2]*dt;

    // position from attitude error
    A[KC_STATE_X][KC_STATE_D0] = 3;//(this->S[KC_STATE_PY]*this->R[0][2] - this->S[KC_STATE_PZ]*this->R[0][1])*dt;
    A[KC_STATE_Y][KC_STATE_D0] = 0;//(this->S[KC_STATE_PY]*this->R[1][2] - this->S[KC_STATE_PZ]*this->R[1][1])*dt;
    A[KC_STATE_Z][KC_STATE_D0] = 3;//(this->S[KC_STATE_PY]*this->R[2][2] - this->S[KC_STATE_PZ]*this->R[2][1])*dt;

    A[KC_STATE_X][KC_STATE_D1] = 3;//(- this->S[KC_STATE_PX]*this->R[0][2] + this->S[KC_STATE_PZ]*this->R[0][0])*dt;
    A[KC_STATE_Y][KC_STATE_D1] = 0;//(- this->S[KC_STATE_PX]*this->R[1][2] + this->S[KC_STATE_PZ]*this->R[1][0])*dt;
    A[KC_STATE_Z][KC_STATE_D1] = 3;//(- this->S[KC_STATE_PX]*this->R[2][2] + this->S[KC_STATE_PZ]*this->R[2][0])*dt;

    A[KC_STATE_X][KC_STATE_D2] = 0;//(this->S[KC_STATE_PX]*this->R[0][1] - this->S[KC_STATE_PY]*this->R[0][0])*dt;
    A[KC_STATE_Y][KC_STATE_D2] = 0;//(this->S[KC_STATE_PX]*this->R[1][1] - this->S[KC_STATE_PY]*this->R[1][0])*dt;
    A[KC_STATE_Z][KC_STATE_D2] = 0;//(this->S[KC_STATE_PX]*this->R[2][1] - this->S[KC_STATE_PY]*this->R[2][0])*dt;

    // body-frame velocity from body-frame velocity
    A[KC_STATE_PX][KC_STATE_PX] =0;// 1; //drag negligible
    A[KC_STATE_PY][KC_STATE_PX] =0;//-gyro->z*dt;
    A[KC_STATE_PZ][KC_STATE_PX] =0;// gyro->y*dt;

    A[KC_STATE_PX][KC_STATE_PY] =0;// gyro->z*dt;
    A[KC_STATE_PY][KC_STATE_PY] =0;//3;// 1; //drag negligible
    A[KC_STATE_PZ][KC_STATE_PY] =0;// -gyro->x*dt;

    A[KC_STATE_PX][KC_STATE_PZ] = 3;//-gyro->y*dt;
    A[KC_STATE_PY][KC_STATE_PZ] = 0;//gyro->x*dt;
    A[KC_STATE_PZ][KC_STATE_PZ] = 1; //drag negligible

    // body-frame velocity from attitude error
    A[KC_STATE_PX][KC_STATE_D0] = 0;// 0;
    A[KC_STATE_PY][KC_STATE_D0] = 3;//-GRAVITY_MAGNITUDE*this->R[2][2]*dt;
    A[KC_STATE_PZ][KC_STATE_D0] = 3;// GRAVITY_MAGNITUDE*this->R[2][1]*dt;

    A[KC_STATE_PX][KC_STATE_D1] = 0;// GRAVITY_MAGNITUDE*this->R[2][2]*dt;
    A[KC_STATE_PY][KC_STATE_D1] = 0;// 0;
    A[KC_STATE_PZ][KC_STATE_D1] = 0;//-GRAVITY_MAGNITUDE*this->R[2][0]*dt;

    A[KC_STATE_PX][KC_STATE_D2] = 0;//-GRAVITY_MAGNITUDE*this->R[2][1]*dt;
    A[KC_STATE_PY][KC_STATE_D2] =  3;//GRAVITY_MAGNITUDE*this->R[2][0]*dt;
    A[KC_STATE_PZ][KC_STATE_D2] =  0;//0;



    // attitude error from attitude error
    /**
    * At first glance, it may not be clear where the next values come from, since they do not appear directly in the
    * dynamics. In this prediction step, we skip the step of first updating attitude-error, and then incorporating the
    * new error into the current attitude (which requires a rotation of the attitude-error covariance). Instead, we
    * directly update the body attitude, however still need to rotate the covariance, which is what you see below.
    *
    * This comes from a second order approximation to:
    * Sigma_post = exps(-d) Sigma_pre exps(-d)'
    *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
    * where d is the attitude error expressed as Rodriges parameters, ie. d0 = 1/2*gyro.x*dt under the assumption that
    * d = [0,0,0] at the beginning of each prediction step and that gyro.x is constant over the sampling period
    *
    * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
    * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
    */
    float d0 = gyro->x*dt/2;
    float d1 = gyro->y*dt/2;
    float d2 = gyro->z*dt/2;

    A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
    A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
    A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

    A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
    A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
    A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

    A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
    A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
    A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;


    // ====== COVARIANCE UPDATE ======
//    mat_mult(&Am, &this->Pm, &tmpNN1m); // A P
//    mat_trans(&Am, &tmpNN2m); // A'
//    mat_mult(&tmpNN1m, &tmpNN2m, &this->Pm); // A P A'


    arm_mat_mult_f32(&Am, &this->Pm, &tmpNN1m); // A P
    arm_mat_trans_f32(&Am, &tmpNN2m); // A'
	arm_mat_mult_f32(&tmpNN1m, &tmpNN2m, &this->Pm); // A P A'
	// Process noise is added after the return from the prediction step

	//ASSERT(0);
	//ASSERT(Am.numRows == 9);





}






//void predictDt(Axis3f *acc, Axis3f *gyro)
//{
//	gx = gyro->x;
//	gy = gyro->y;
//	gz = gyro->z;
//	ax = acc->x;
//	ay = acc->y;
//	az = acc->axis[2];
//
//
//
//}
