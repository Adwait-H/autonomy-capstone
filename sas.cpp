/*#include <unistd.h>*/
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#include <Navio2/RCInput_Navio2.h>
#include <Navio+/RCInput_Navio.h>
#include <Navio2/ADC_Navio2.h>
#include <Navio+/ADC_Navio.h>
#include "Common/MPU9250.h"
#include "Navio2/LSM9DS1.h"
#include "Common/MS5611.h"
#include "Common/Util.h"
#include <unistd.h>
#include <string>
#include <memory>
#include <time.h>
#include "datalink/network.h"

/* SERVO STUFF */
#define SERVO_MIN 2150 /*mS*/
#define SERVO_MAX 1000 /*mS*/

#define PWM_OUTPUT 8
/* END SERVO STUFF */

/*FROM THERMAL CAMERA CODE*/
#include <stdint.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <chrono>
#include <thread>
#include <math.h>
#include "MLX90640_API.h"

#define MLX_I2C_ADDR 0x33

#define SENSOR_W 24
#define SENSOR_H 32

// Valid frame rates are 1, 2, 4, 8, 16, 32 and 64
// The i2c baudrate is set to 1mhz to support these
#define FPS 32

#define LOOP_HZ 100

bool running = true;
/* END FROM THERMAL CAMERA */


/* turn this on for debug prints */
#define DEBUG               0
#define DISP_E              0
#define DISP_MOCAP_READINGS 0
#define DISP_THERMALCAM     0

/* EKF configuration */
#define EKF_PRINT_STATE 0
#define EKF_GRAVITY_MPS2 9.80665
#define EKF_ACCEL_STD_MPS2 0.50
#define EKF_GYRO_STD_RAD_S 0.03
#define EKF_GYRO_BIAS_RW_RAD_S 0.002
#define EKF_ACCEL_BIAS_RW_MPS2 0.03
#define EKF_MOCAP_POS_STD_M 0.01
#define EKF_MOCAP_ATT_STD_RAD (2.0*3.14159265358979323846/180.0)
#define EKF_ACCEL_BIAS_LIMIT_MPS2 (4.0*EKF_GRAVITY_MPS2)
#define EKF_GYRO_BIAS_LIMIT_RAD_S (30.0*3.14159265358979323846/180.0)
#define EKF_MAX_PREDICT_DT_S 0.01
#define EKF_MAX_TOTAL_DT_S 0.25
// #define EKF_PRINT_STATE 0
// #define EKF_GRAVITY_MPS2 9.80665
// #define EKF_ACCEL_STD_MPS2 0.50
// #define EKF_GYRO_STD_RAD_S 0.03
// #define EKF_GYRO_BIAS_RW_RAD_S 0.002
// #define EKF_ACCEL_BIAS_RW_MPS2 0.03
// #define EKF_MOCAP_POS_STD_M 0.01
// #define EKF_MOCAP_ATT_STD_RAD (2.0*3.14159265358979323846/180.0)
// #define EKF_ACCEL_BIAS_LIMIT_MPS2 (4.0*EKF_GRAVITY_MPS2)

/*
 * Set to 0 if the incoming Mocap quaternion is world-to-body instead of
 * body-to-world. The EKF uses the same body axes already used by the SAS
 * controller: p=+wy, q=+wx, r=-wz and sx=-ay, sy=-ax, sz=+az.
 */
#define MOCAP_QUAT_BODY_TO_WORLD 0

/* gains/params */
#define KFF_X           (3.14/4)
#define KFF_Y           (3.14/4)
#define KFF_Z           3.14
#define KP_X            0.045
#define KP_Y            0.045
#define KP_Z            0.15

#define KI_X            0.25
#define KI_Y            0.25
#define KI_Z            0.3
#define TAUI            3
#define TAUM            0.025       /* cannot be zero */
#define TAUA            (TAUM)

/* simple autopilot waypoint follower */
#define AUTOPILOT_WAYPOINT_COUNT           3
#define AUTOPILOT_WAYPOINT_HOLD_TIME_S     3
#define AUTOPILOT_WAYPOINT_ACCEPT_RADIUS_M 0.20
#define AUTOPILOT_MAX_ANGLE_RAD            (15.0*EKF_PI/180.0)

#define AUTOPILOT_POS_KP_X                 0.0075
#define AUTOPILOT_POS_KI_X                 0.02
#define AUTOPILOT_POS_KD_X                 0.12

#define AUTOPILOT_POS_KP_Y                 0.0075
#define AUTOPILOT_POS_KI_Y                 0.02
#define AUTOPILOT_POS_KD_Y                 0.12

#define AUTOPILOT_POS_KP_Z                 0.10
#define AUTOPILOT_POS_KI_Z                 0.25
#define AUTOPILOT_POS_KD_Z                 1.00

#define AUTOPILOT_POS_INT_LIM_X            1.00
#define AUTOPILOT_POS_INT_LIM_Y            1.00
#define AUTOPILOT_POS_INT_LIM_Z            1.00

#define AUTOPILOT_ATT_KP_ROLL              0.3
#define AUTOPILOT_ATT_KI_ROLL              0.03
#define AUTOPILOT_ATT_KD_ROLL              0.10

#define AUTOPILOT_ATT_KP_PITCH             0.3
#define AUTOPILOT_ATT_KI_PITCH             0.03
#define AUTOPILOT_ATT_KD_PITCH             0.10

#define AUTOPILOT_ATT_KP_YAW               0.75
#define AUTOPILOT_ATT_KI_YAW               0.005
#define AUTOPILOT_ATT_KD_YAW               0.2

#define AUTOPILOT_ATT_INT_LIM              0.40

/* motors */
#define PWM_HZ          400 // also used for the timing of the whole loop
#define PWM_HZ_MOCAP    250 // only used for the mocap reading timing
#define PWM_MOTOR0      12
#define PWM_MOTOR1      13
#define PWM_MOTOR2      1
#define PWM_MOTOR3      0
// I'm actually not sure which motor is which PWM_MOTORX, but these are the directions
// front right ccw
// front left cw
// aft right cw
// aft left ccw
#define PWM_MIN         1025   /* mS */
#define PWM_MAX         1980 //1980   /* mS */ // capped below full scale for debugging
#define ARM_TIME        1.0   /* sec */

/* pilot input */

#define PWM_PAYLOAD_DOOR 6 // PWM_FTS
#define PAYLOAD_DOOR_THRESHOLD  1600

#define PWM_THRUST      2
#define PWM_ROLL        0
#define PWM_PITCH       1
#define PWM_YAW         3
#define PWM_AUTO        4
#define PWM_ARM         5
#define THRUST_MIN      1025
#define THRUST_MAX      1980
#define ROLL_MIN        1000
#define ROLL_MAX        1980
#define PITCH_MIN       2000  /* note direction reversal */
#define PITCH_MAX       1000
#define YAW_MIN         1010
#define YAW_MAX         1980
#define ARMED_THRESHOLD 1600
#define AUTO_THRESHOLD  1600
#define AUTO_ON         1900
#define AUTO_OFF        1050
#define THROTTLEMINTOL  0.2

/* IMU */
// for testing, try both of these
#define IMU_MPU9250     1
// this second IMU seems better when sitting still?
//#define IMU_LSM9DS1     1

/* A to D */
#define A2D_VOLTAGECHANNEL 2
#define A2D_CURRENTCHANNEL 3

#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))

using namespace Navio;

static const long long NSEC_PER_SEC = 1000000000LL;
static const double EKF_PI = 3.14159265358979323846;
static const int EKF_STATE_SIZE = 15;
static const int EKF_MAX_MEAS_SIZE = 6;

// enum EkfStateIndex {
//     EKF_POS_X = 0,
//     EKF_POS_Y,
//     EKF_POS_Z,
//     EKF_VEL_X,
//     EKF_VEL_Y,
//     EKF_VEL_Z,
//     EKF_ROLL,
//     EKF_PITCH,
//     EKF_YAW,
//     EKF_RATE_P,
//     EKF_RATE_Q,
//     EKF_RATE_R,
//     EKF_BIAS_X,
//     EKF_BIAS_Y,
//     EKF_BIAS_Z
// };

enum EkfStateIndex {
    EKF_POS_X = 0,
    EKF_POS_Y,
    EKF_POS_Z,
    EKF_VEL_X,
    EKF_VEL_Y,
    EKF_VEL_Z,
    EKF_ROLL,
    EKF_PITCH,
    EKF_YAW,
    EKF_GYRO_BIAS_P,
    EKF_GYRO_BIAS_Q,
    EKF_GYRO_BIAS_R,
    EKF_BIAS_X,
    EKF_BIAS_Y,
    EKF_BIAS_Z
};

std::unique_ptr <RCInput> get_rcin() {

    if( get_navio_version() == NAVIO2 ) {
        auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
        return ptr;
    } else {
        auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio() };
        return ptr;
    }

}

std::unique_ptr <RCOutput> get_rcout() {

    if( get_navio_version() == NAVIO2 ) {
        auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
        return ptr;
    } else {
        auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio() };
        return ptr;
    }

}

std::unique_ptr <InertialSensor> get_inertial_sensor() {

#ifdef IMU_MPU9250
        auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
        return ptr;
#endif
#ifdef IMU_LSM9DS1
        auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
        return ptr;
#endif
    printf("No IMU Identified\n");
    return nullptr; // This was in adwait's version but I don't like it

}

std::unique_ptr <ADC> get_converter() {

    if( get_navio_version() == NAVIO2 ) {
        auto ptr = std::unique_ptr <ADC>{ new ADC_Navio2() };
        return ptr;
    } else {
        auto ptr = std::unique_ptr <ADC>{ new ADC_Navio() };
        return ptr;
    }

}

timespec diff( timespec start, timespec end ) {

    timespec temp;
    if( ( end.tv_nsec - start.tv_nsec ) < 0 ) {
        temp.tv_sec  =              end.tv_sec  - start.tv_sec  - 1;
        temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
    } else {
        temp.tv_sec  =              end.tv_sec  - start.tv_sec;
        temp.tv_nsec =              end.tv_nsec - start.tv_nsec;
    }
    return temp;

}

static long long elapsed_nsec( const timespec& start, const timespec& end ) {

    return ( (long long)end.tv_sec - (long long)start.tv_sec ) * NSEC_PER_SEC
           + ( (long long)end.tv_nsec - (long long)start.tv_nsec );

}

static double elapsed_seconds( const timespec& start, const timespec& end ) {

    return (double)elapsed_nsec( start, end ) / (double)NSEC_PER_SEC;

}

static void advance_timespec( timespec& time, long long nsec ) {

    time.tv_sec  += nsec / NSEC_PER_SEC;
    time.tv_nsec += nsec % NSEC_PER_SEC;

    while( time.tv_nsec >= NSEC_PER_SEC ) {
        time.tv_sec++;
        time.tv_nsec -= NSEC_PER_SEC;
    }

}

static bool period_elapsed( timespec& previous, const timespec& current, long long period_nsec ) {

    const long long elapsed = elapsed_nsec( previous, current );
    if( elapsed < period_nsec ) {
        return false;
    }

    const long long periods = elapsed / period_nsec;
    advance_timespec( previous, periods * period_nsec );
    return true;

}

static void set_all_motors_min( RCOutput& motors ) {

    motors.set_duty_cycle( PWM_MOTOR0, PWM_MIN );
    motors.set_duty_cycle( PWM_MOTOR1, PWM_MIN );
    motors.set_duty_cycle( PWM_MOTOR2, PWM_MIN );
    motors.set_duty_cycle( PWM_MOTOR3, PWM_MIN );

}

static double clamp_value( double value, double lower, double upper ) {

    return std::max( lower, std::min( value, upper ) );

}

static double wrap_angle( double angle ) {

    while( angle > EKF_PI ) {
        angle -= 2.0*EKF_PI;
    }
    while( angle < -EKF_PI ) {
        angle += 2.0*EKF_PI;
    }
    return angle;

}

static double clamp_pitch( double pitch ) {

    const double pitchLimit = 0.5*EKF_PI - 1.0e-3;
    return clamp_value( pitch, -pitchLimit, pitchLimit );

}

class SasEkf {

public:
    SasEkf() {
        reset();
    }

    void reset() {

        std::fill( x_, x_ + EKF_STATE_SIZE, 0.0 );
        std::fill( &P_[0][0], &P_[0][0] + EKF_STATE_SIZE*EKF_STATE_SIZE, 0.0 );
        std::fill( lastCorrectedRate_, lastCorrectedRate_ + 3, 0.0 );

        const double initialAttVariance = (30.0*EKF_PI/180.0)*(30.0*EKF_PI/180.0);
        const double initialGyroBiasVariance = (10.0*EKF_PI/180.0)*(10.0*EKF_PI/180.0);
        for( int i=0; i<3; i++ ) {
            P_[EKF_POS_X + i][EKF_POS_X + i] = 25.0;
            P_[EKF_VEL_X + i][EKF_VEL_X + i] = 25.0;
            P_[EKF_ROLL  + i][EKF_ROLL  + i] = initialAttVariance;
            P_[EKF_GYRO_BIAS_P + i][EKF_GYRO_BIAS_P + i] = initialGyroBiasVariance;
            P_[EKF_BIAS_X + i][EKF_BIAS_X + i] = 4.0;
        }

        lastMocapFrame_ = -1;

    }

    void stepImu( double dt, double gyroP, double gyroQ, double gyroR,
                  double accelX, double accelY, double accelZ ) {

        if( !std::isfinite( dt ) || dt <= 0.0 ) {
            return;
        }
        const double gyroBody[3] = { gyroP, gyroQ, gyroR };
        const double accelBody[3] = { accelX, accelY, accelZ };
        for( int i=0; i<3; i++ ) {
            if( !std::isfinite( gyroBody[i] ) || !std::isfinite( accelBody[i] ) ) {
                return;
            }
        }

        dt = clamp_value( dt, 1.0e-4, EKF_MAX_TOTAL_DT_S );
        while( dt > 1.0e-9 ) {
            const double predictDt = std::min( dt, EKF_MAX_PREDICT_DT_S );
            predict( predictDt, gyroBody, accelBody );
            dt -= predictDt;
        }

        updateCorrectedRates( gyroBody );

    }

    bool updateMocap( const onboardMocapClient_ref& mocap ) {

        if( !mocap.valid ) {
            return false;
        }
        if( mocap.frameNum > 0 && mocap.frameNum == lastMocapFrame_ ) {
            return false;
        }

        double mocapEuler[3];
        if( !quaternionToEuler( mocap.qx, mocap.qy, mocap.qz, mocap.qw, mocapEuler ) ) {
            return false;
        }

        const double mocapPos[3] = { mocap.pos_x, mocap.pos_y, mocap.pos_z };
        for( int i=0; i<3; i++ ) {
            if( !std::isfinite( mocapPos[i] ) || !std::isfinite( mocapEuler[i] ) ) {
                return false;
            }
        }

        if( lastMocapFrame_ < 0 ) {
            for( int i=0; i<3; i++ ) {
                x_[EKF_POS_X + i] = mocapPos[i];
                x_[EKF_ROLL  + i] = mocapEuler[i];
            }
            normalizeState();
        }

        const int index[EKF_MAX_MEAS_SIZE] = {
            EKF_POS_X, EKF_POS_Y, EKF_POS_Z, EKF_ROLL, EKF_PITCH, EKF_YAW
        };
        double residual[EKF_MAX_MEAS_SIZE] = {
            mocapPos[0] - x_[EKF_POS_X],
            mocapPos[1] - x_[EKF_POS_Y],
            mocapPos[2] - x_[EKF_POS_Z],
            wrap_angle( mocapEuler[0] - x_[EKF_ROLL] ),
            wrap_angle( mocapEuler[1] - x_[EKF_PITCH] ),
            wrap_angle( mocapEuler[2] - x_[EKF_YAW] )
        };
        const double posVar = EKF_MOCAP_POS_STD_M*EKF_MOCAP_POS_STD_M;
        const double attVar = EKF_MOCAP_ATT_STD_RAD*EKF_MOCAP_ATT_STD_RAD;
        double rDiag[EKF_MAX_MEAS_SIZE] = {
            posVar, posVar, posVar, attVar, attVar, attVar
        };

        if( !updateIndexed( index, residual, rDiag, EKF_MAX_MEAS_SIZE ) ) {
            return false;
        }

        lastMocapFrame_ = mocap.frameNum;
        return true;

    }

    double state( int index ) const {

        return x_[index];

    }

    int lastMocapFrame() const {

        return lastMocapFrame_;

    }

    double bodyRate( int axis ) const {

        if( axis < 0 || axis >= 3 ) {
            return 0.0;
        }
        return lastCorrectedRate_[axis];

    }

    double covarianceSummary() const {

        double trace = 0.0;
        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            trace += P_[i][i];
        }
        return trace / EKF_STATE_SIZE;

    }

private:
    double x_[EKF_STATE_SIZE];
    double P_[EKF_STATE_SIZE][EKF_STATE_SIZE];
    double lastCorrectedRate_[3];
    int lastMocapFrame_;

    static bool quaternionToEuler( double qx, double qy, double qz, double qw,
                                   double euler[3] ) {

        const double norm = std::sqrt( qx*qx + qy*qy + qz*qz + qw*qw );
        if( !std::isfinite( norm ) || norm < 1.0e-9 ) {
            return false;
        }

        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;

#if !MOCAP_QUAT_BODY_TO_WORLD
        qx = -qx;
        qy = -qy;
        qz = -qz;
#endif

        const double sinrCosp = 2.0*( qw*qx + qy*qz );
        const double cosrCosp = 1.0 - 2.0*( qx*qx + qy*qy );
        // euler[0] = std::atan2( sinrCosp, cosrCosp );
        euler[1] = std::atan2( sinrCosp, cosrCosp );

        const double sinp = clamp_value( 2.0*( qw*qy - qz*qx ), -1.0, 1.0 );
        // euler[1] = std::asin( sinp );
        euler[0] = std::asin( sinp );

        const double sinyCosp = 2.0*( qw*qz + qx*qy );
        const double cosyCosp = 1.0 - 2.0*( qy*qy + qz*qz );
        euler[2] = std::atan2( sinyCosp, cosyCosp );

        euler[0] = wrap_angle( euler[0] );
        euler[1] = clamp_pitch( euler[1] );
        euler[2] = wrap_angle( euler[2] );
        // printf("roll, pitch, yaw: [%f, %f, %f]\n", euler[0], euler[1], euler[2]);
        return true;

    }

    static void eulerToRotation( double roll, double pitch, double yaw,
                                 double R[3][3] ) {

        const double sr = std::sin( roll );
        const double cr = std::cos( roll );
        const double sp = std::sin( pitch );
        const double cp = std::cos( pitch );
        const double sy = std::sin( yaw );
        const double cy = std::cos( yaw );

        R[0][0] = cy*cp;
        R[0][1] = cy*sp*sr - sy*cr;
        R[0][2] = cy*sp*cr + sy*sr;

        R[1][0] = sy*cp;
        R[1][1] = sy*sp*sr + cy*cr;
        R[1][2] = sy*sp*cr - cy*sr;

        R[2][0] = -sp;
        R[2][1] = cp*sr;
        R[2][2] = cp*cr;

    }

    static void eulerRates( double roll, double pitch, double p, double q, double r,
                            double rates[3] ) {

        const double sr = std::sin( roll );
        const double cr = std::cos( roll );
        const double sp = std::sin( pitch );
        double cp = std::cos( pitch );
        if( std::fabs( cp ) < 1.0e-3 ) {
            cp = std::copysign( 1.0e-3, cp );
        }
        const double tp = sp / cp;

        rates[0] = p + sr*tp*q + cr*tp*r;
        rates[1] = cr*q - sr*r;
        rates[2] = (sr/cp)*q + (cr/cp)*r;

    }

    void propagateState( const double stateIn[EKF_STATE_SIZE], double dt,
                         const double gyroBody[3],
                         const double accelBody[3],
                         double stateOut[EKF_STATE_SIZE] ) const {

        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            stateOut[i] = stateIn[i];
        }

        const double roll = stateIn[EKF_ROLL];
        const double pitch = clamp_pitch( stateIn[EKF_PITCH] );
        const double yaw = stateIn[EKF_YAW];

        double R[3][3];
        eulerToRotation( roll, pitch, yaw, R );

        const double specificForceBody[3] = {
            accelBody[0] - stateIn[EKF_BIAS_X],
            accelBody[1] - stateIn[EKF_BIAS_Y],
            accelBody[2] - stateIn[EKF_BIAS_Z]
        };
        const double correctedGyroBody[3] = {
            gyroBody[0] - stateIn[EKF_GYRO_BIAS_P],
            gyroBody[1] - stateIn[EKF_GYRO_BIAS_Q],
            gyroBody[2] - stateIn[EKF_GYRO_BIAS_R]
        };

        double accelWorld[3] = { 0.0, 0.0, -EKF_GRAVITY_MPS2 };
        for( int i=0; i<3; i++ ) {
            for( int j=0; j<3; j++ ) {
                accelWorld[i] += R[i][j]*specificForceBody[j];
            }
        }

        for( int i=0; i<3; i++ ) {
            stateOut[EKF_POS_X + i] = stateIn[EKF_POS_X + i]
                                    + stateIn[EKF_VEL_X + i]*dt
                                    + 0.5*accelWorld[i]*dt*dt;
            stateOut[EKF_VEL_X + i] = stateIn[EKF_VEL_X + i]
                                    + accelWorld[i]*dt;
        }

        double attitudeRates[3];
        eulerRates( roll, pitch,
                    correctedGyroBody[0], correctedGyroBody[1], correctedGyroBody[2],
                    attitudeRates );

        stateOut[EKF_ROLL]  = wrap_angle( stateIn[EKF_ROLL]  + attitudeRates[0]*dt );
        stateOut[EKF_PITCH] = clamp_pitch( stateIn[EKF_PITCH] + attitudeRates[1]*dt );
        stateOut[EKF_YAW]   = wrap_angle( stateIn[EKF_YAW]   + attitudeRates[2]*dt );

    }

    void predict( double dt, const double gyroBody[3], const double accelBody[3] ) {

        double statePred[EKF_STATE_SIZE];
        propagateState( x_, dt, gyroBody, accelBody, statePred );

        double F[EKF_STATE_SIZE][EKF_STATE_SIZE] = {};
        for( int col=0; col<EKF_STATE_SIZE; col++ ) {
            const double eps = finiteDifferenceStep( col );
            double xPlus[EKF_STATE_SIZE];
            double xMinus[EKF_STATE_SIZE];
            for( int i=0; i<EKF_STATE_SIZE; i++ ) {
                xPlus[i] = x_[i];
                xMinus[i] = x_[i];
            }

            xPlus[col] += eps;
            xMinus[col] -= eps;
            xPlus[EKF_ROLL] = wrap_angle( xPlus[EKF_ROLL] );
            xPlus[EKF_PITCH] = clamp_pitch( xPlus[EKF_PITCH] );
            xPlus[EKF_YAW] = wrap_angle( xPlus[EKF_YAW] );
            xMinus[EKF_ROLL] = wrap_angle( xMinus[EKF_ROLL] );
            xMinus[EKF_PITCH] = clamp_pitch( xMinus[EKF_PITCH] );
            xMinus[EKF_YAW] = wrap_angle( xMinus[EKF_YAW] );

            double yPlus[EKF_STATE_SIZE];
            double yMinus[EKF_STATE_SIZE];
            propagateState( xPlus, dt, gyroBody, accelBody, yPlus );
            propagateState( xMinus, dt, gyroBody, accelBody, yMinus );

            for( int row=0; row<EKF_STATE_SIZE; row++ ) {
                double diff = yPlus[row] - yMinus[row];
                if( row == EKF_ROLL || row == EKF_PITCH || row == EKF_YAW ) {
                    diff = wrap_angle( diff );
                }
                F[row][col] = diff / ( 2.0*eps );
            }
        }

        double FP[EKF_STATE_SIZE][EKF_STATE_SIZE] = {};
        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            for( int j=0; j<EKF_STATE_SIZE; j++ ) {
                for( int k=0; k<EKF_STATE_SIZE; k++ ) {
                    FP[i][j] += F[i][k]*P_[k][j];
                }
            }
        }

        double Pnew[EKF_STATE_SIZE][EKF_STATE_SIZE] = {};
        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            for( int j=0; j<EKF_STATE_SIZE; j++ ) {
                for( int k=0; k<EKF_STATE_SIZE; k++ ) {
                    Pnew[i][j] += FP[i][k]*F[j][k];
                }
            }
        }

        addProcessNoise( Pnew, dt );

        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            x_[i] = statePred[i];
            for( int j=0; j<EKF_STATE_SIZE; j++ ) {
                P_[i][j] = Pnew[i][j];
            }
        }

        normalizeState();
        conditionCovariance();

    }

    bool updateIndexed( const int index[], const double residual[],
                        const double rDiag[], int measurementSize ) {

        if( measurementSize <= 0 || measurementSize > EKF_MAX_MEAS_SIZE ) {
            return false;
        }

        double S[EKF_MAX_MEAS_SIZE][EKF_MAX_MEAS_SIZE] = {};
        for( int i=0; i<measurementSize; i++ ) {
            for( int j=0; j<measurementSize; j++ ) {
                S[i][j] = P_[index[i]][index[j]];
            }
            S[i][i] += rDiag[i];
        }

        double Sinv[EKF_MAX_MEAS_SIZE][EKF_MAX_MEAS_SIZE] = {};
        if( !invertSmallMatrix( S, Sinv, measurementSize ) ) {
            return false;
        }

        double K[EKF_STATE_SIZE][EKF_MAX_MEAS_SIZE] = {};
        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            for( int j=0; j<measurementSize; j++ ) {
                for( int k=0; k<measurementSize; k++ ) {
                    K[i][j] += P_[i][index[k]]*Sinv[k][j];
                }
            }
        }

        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            double dx = 0.0;
            for( int j=0; j<measurementSize; j++ ) {
                dx += K[i][j]*residual[j];
            }
            x_[i] += dx;
        }
        normalizeState();

        double A[EKF_STATE_SIZE][EKF_STATE_SIZE] = {};
        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            A[i][i] = 1.0;
            for( int j=0; j<measurementSize; j++ ) {
                A[i][index[j]] -= K[i][j];
            }
        }

        double AP[EKF_STATE_SIZE][EKF_STATE_SIZE] = {};
        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            for( int j=0; j<EKF_STATE_SIZE; j++ ) {
                for( int k=0; k<EKF_STATE_SIZE; k++ ) {
                    AP[i][j] += A[i][k]*P_[k][j];
                }
            }
        }

        double Pnew[EKF_STATE_SIZE][EKF_STATE_SIZE] = {};
        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            for( int j=0; j<EKF_STATE_SIZE; j++ ) {
                for( int k=0; k<EKF_STATE_SIZE; k++ ) {
                    Pnew[i][j] += AP[i][k]*A[j][k];
                }
                for( int k=0; k<measurementSize; k++ ) {
                    Pnew[i][j] += K[i][k]*rDiag[k]*K[j][k];
                }
            }
        }

        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            for( int j=0; j<EKF_STATE_SIZE; j++ ) {
                P_[i][j] = Pnew[i][j];
            }
        }

        conditionCovariance();
        return true;

    }

    static double finiteDifferenceStep( int index ) {

        if( index >= EKF_ROLL && index <= EKF_YAW ) {
            return 1.0e-6;
        }
        if( index >= EKF_GYRO_BIAS_P && index <= EKF_GYRO_BIAS_R ) {
            return 1.0e-6;
        }
        return 1.0e-5;

    }

    static bool invertSmallMatrix( const double A[EKF_MAX_MEAS_SIZE][EKF_MAX_MEAS_SIZE],
                                   double Ainv[EKF_MAX_MEAS_SIZE][EKF_MAX_MEAS_SIZE],
                                   int n ) {

        double aug[EKF_MAX_MEAS_SIZE][2*EKF_MAX_MEAS_SIZE] = {};
        for( int i=0; i<n; i++ ) {
            for( int j=0; j<n; j++ ) {
                aug[i][j] = A[i][j];
                aug[i][n+j] = ( i == j ) ? 1.0 : 0.0;
            }
        }

        for( int col=0; col<n; col++ ) {
            int pivot = col;
            double maxAbs = std::fabs( aug[col][col] );
            for( int row=col+1; row<n; row++ ) {
                const double candidate = std::fabs( aug[row][col] );
                if( candidate > maxAbs ) {
                    maxAbs = candidate;
                    pivot = row;
                }
            }

            if( maxAbs < 1.0e-12 || !std::isfinite( maxAbs ) ) {
                return false;
            }

            if( pivot != col ) {
                for( int j=0; j<2*n; j++ ) {
                    std::swap( aug[col][j], aug[pivot][j] );
                }
            }

            const double pivotValue = aug[col][col];
            for( int j=0; j<2*n; j++ ) {
                aug[col][j] /= pivotValue;
            }

            for( int row=0; row<n; row++ ) {
                if( row == col ) {
                    continue;
                }
                const double scale = aug[row][col];
                for( int j=0; j<2*n; j++ ) {
                    aug[row][j] -= scale*aug[col][j];
                }
            }
        }

        for( int i=0; i<n; i++ ) {
            for( int j=0; j<n; j++ ) {
                Ainv[i][j] = aug[i][n+j];
            }
        }

        return true;

    }

    static void addProcessNoise( double P[EKF_STATE_SIZE][EKF_STATE_SIZE], double dt ) {

        const double accelVar = EKF_ACCEL_STD_MPS2*EKF_ACCEL_STD_MPS2;
        const double gyroVar = EKF_GYRO_STD_RAD_S*EKF_GYRO_STD_RAD_S;
        const double gyroBiasRwVar = EKF_GYRO_BIAS_RW_RAD_S*EKF_GYRO_BIAS_RW_RAD_S;
        const double biasRwVar = EKF_ACCEL_BIAS_RW_MPS2*EKF_ACCEL_BIAS_RW_MPS2;
        const double dt2 = dt*dt;
        const double dt3 = dt2*dt;
        const double dt4 = dt2*dt2;

        for( int i=0; i<3; i++ ) {
            const int pos = EKF_POS_X + i;
            const int vel = EKF_VEL_X + i;
            const int att = EKF_ROLL + i;
            const int gyroBias = EKF_GYRO_BIAS_P + i;
            const int bias = EKF_BIAS_X + i;

            P[pos][pos] += 0.25*accelVar*dt4 + 1.0e-9;
            P[pos][vel] += 0.5*accelVar*dt3;
            P[vel][pos] += 0.5*accelVar*dt3;
            P[vel][vel] += accelVar*dt2 + 1.0e-8;
            P[att][att] += gyroVar*dt2 + 1.0e-10;
            P[gyroBias][gyroBias] += gyroBiasRwVar*dt + 1.0e-12;
            P[bias][bias] += biasRwVar*dt + 1.0e-10;
        }

    }

    void normalizeState() {

        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            if( !std::isfinite( x_[i] ) ) {
                x_[i] = 0.0;
            }
        }

        x_[EKF_ROLL] = wrap_angle( x_[EKF_ROLL] );
        x_[EKF_PITCH] = clamp_pitch( x_[EKF_PITCH] );
        x_[EKF_YAW] = wrap_angle( x_[EKF_YAW] );

        for( int i=0; i<3; i++ ) {
            const int gyroBiasIndex = EKF_GYRO_BIAS_P + i;
            x_[gyroBiasIndex] = clamp_value( x_[gyroBiasIndex],
                                             -EKF_GYRO_BIAS_LIMIT_RAD_S,
                                             EKF_GYRO_BIAS_LIMIT_RAD_S );

            const int biasIndex = EKF_BIAS_X + i;
            x_[biasIndex] = clamp_value( x_[biasIndex],
                                         -EKF_ACCEL_BIAS_LIMIT_MPS2,
                                         EKF_ACCEL_BIAS_LIMIT_MPS2 );
        }

    }

    void updateCorrectedRates( const double gyroBody[3] ) {

        for( int i=0; i<3; i++ ) {
            const double correctedRate = gyroBody[i] - x_[EKF_GYRO_BIAS_P + i];
            lastCorrectedRate_[i] = std::isfinite( correctedRate ) ? correctedRate : 0.0;
        }

    }

    void conditionCovariance() {

        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            for( int j=i; j<EKF_STATE_SIZE; j++ ) {
                double value = 0.5*( P_[i][j] + P_[j][i] );
                if( !std::isfinite( value ) ) {
                    value = 0.0;
                }
                P_[i][j] = value;
                P_[j][i] = value;
            }
        }

        for( int i=0; i<EKF_STATE_SIZE; i++ ) {
            if( P_[i][i] < 1.0e-12 ) {
                P_[i][i] = 1.0e-12;
            }
        }

    }

};

struct SimplePid {
    double kp;
    double ki;
    double kd;
    double integralLimit;
    double integral;

    void reset() {
        integral = 0.0;
    }

    double update( double error, double measuredRate, double dt ) {

        if( !std::isfinite( error ) || !std::isfinite( measuredRate ) ||
            !std::isfinite( dt ) || dt <= 0.0 ) {
            return 0.0;
        }

        integral += error*dt;
        integral = clamp_value( integral, -integralLimit, integralLimit );

        return kp*error + ki*integral - kd*measuredRate;

    }
};

struct AutopilotWaypoint {
    double x;
    double y;
    double z;
    double yaw;
};

struct AutopilotState {
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
    double roll;
    double pitch;
    double yaw;
    double p;
    double q;
    double r;
};

struct AutopilotControl {
    double rollStick = 0.0;
    double pitchStick = 0.0;
    double yawStick = 0.0;
    double thrustStick = -1.0;
    double delmx = 0.0;
    double delmy = 0.0;
    double delmz = 0.0;
    double delf = -1.0;
    int waypointIndex = -1;
    double waypointDistance = 0.0;
    bool valid = false;
};

class WaypointFollower {

public:
    WaypointFollower()
        : posXPid_{ AUTOPILOT_POS_KP_X, AUTOPILOT_POS_KI_X, AUTOPILOT_POS_KD_X,
                    AUTOPILOT_POS_INT_LIM_X, 0.0 },
          posYPid_{ AUTOPILOT_POS_KP_Y, AUTOPILOT_POS_KI_Y, AUTOPILOT_POS_KD_Y,
                    AUTOPILOT_POS_INT_LIM_Y, 0.0 },
          posZPid_{ AUTOPILOT_POS_KP_Z, AUTOPILOT_POS_KI_Z, AUTOPILOT_POS_KD_Z,
                    AUTOPILOT_POS_INT_LIM_Z, 0.0 },
          rollPid_{ AUTOPILOT_ATT_KP_ROLL, AUTOPILOT_ATT_KI_ROLL, AUTOPILOT_ATT_KD_ROLL,
                    AUTOPILOT_ATT_INT_LIM, 0.0 },
          pitchPid_{ AUTOPILOT_ATT_KP_PITCH, AUTOPILOT_ATT_KI_PITCH, AUTOPILOT_ATT_KD_PITCH,
                     AUTOPILOT_ATT_INT_LIM, 0.0 },
          yawPid_{ AUTOPILOT_ATT_KP_YAW, AUTOPILOT_ATT_KI_YAW, AUTOPILOT_ATT_KD_YAW,
                   AUTOPILOT_ATT_INT_LIM, 0.0 },
          waypointCount_( 0 ),
          activeWaypoint_( 0 ),
          timeAtWaypoint_( 0.0 ) {
    }

    void resetControllers() {
        posXPid_.reset();
        posYPid_.reset();
        posZPid_.reset();
        rollPid_.reset();
        pitchPid_.reset();
        yawPid_.reset();
        timeAtWaypoint_ = 0.0;
    }

    void clearMission() {
        waypointCount_ = 0;
        activeWaypoint_ = 0;
        timeAtWaypoint_ = 0.0;
        resetControllers();
    }

    void initializeMission( double referenceX, double referenceY, double referenceZ,
                            double referenceYaw ) {

        (void)referenceYaw;

        clearMission();
        waypointCount_ = AUTOPILOT_WAYPOINT_COUNT;

        // World-frame NED mission with yaw fixed to zero.
        // x = North, y = East, z = Down.
        waypoints_[0] = { referenceX, referenceY, referenceZ + 0.5, 0.0 };
        waypoints_[1] = { referenceX, referenceY + 0.5, referenceZ + 0.5, 0.0 };
        waypoints_[2] = { referenceX, referenceY + 0.5, referenceZ + 0.05, 0.0};
        // waypoints_[3] = { referenceX, referenceY, referenceZ + 0.7, 0.0 };
        // waypoints_[4] = { referenceX, referenceY, referenceZ + 0.6, 0.0 };
        // waypoints_[5] = { referenceX, referenceY, referenceZ + 0.5, 0.0 };

        printf( "Autopilot mission armed from EKF pose [%.2f %.2f %.2f] yaw=0.00 with %d waypoints", referenceX, referenceY, referenceZ, waypointCount_ );

    }

    bool missionReady() const {
        return waypointCount_ > 0;
    }

    int waypointCount() const {
        return waypointCount_;
    }

    const AutopilotWaypoint* currentWaypoint() const {
        if( !missionReady() ) {
            return nullptr;
        }
        return &waypoints_[activeWaypoint_];
    }

    AutopilotControl update( const AutopilotState& state, double dt ) {

        AutopilotControl output;
        if( !missionReady() ) {
            return output;
        }

        dt = clamp_value( dt, 1.0e-3, 0.05 );

        AutopilotWaypoint target = waypoints_[activeWaypoint_];
        double dx = target.x - state.x;
        double dy = target.y - state.y;
        double dz = target.z - state.z;
        double distance = std::sqrt( dx*dx + dy*dy + dz*dz );

        if( distance <= AUTOPILOT_WAYPOINT_ACCEPT_RADIUS_M &&
            activeWaypoint_ + 1 < waypointCount_ ) {
            timeAtWaypoint_ += dt;
            if( timeAtWaypoint_ >= AUTOPILOT_WAYPOINT_HOLD_TIME_S ) {
                activeWaypoint_++;
                resetControllers();
                target = waypoints_[activeWaypoint_];
                dx = target.x - state.x;
                dy = target.y - state.y;
                dz = target.z - state.z;
                distance = std::sqrt( dx*dx + dy*dy + dz*dz );
                timeAtWaypoint_ = 0.0;
            }
        } else {
            timeAtWaypoint_ = 0.0;
        }

        const double desiredYaw = 0.0;

        // World-frame NED horizontal control.
        // With yaw fixed at zero, +x (North) maps to pitch command and
        // +y (East) maps to roll command with the same sign convention as the
        // original controller at yaw = 0.
        // const double yError = dx;
        // const double xError  = dy;
        const double yError = dy;
        const double xError  = dx;
        const double xVel   = state.vx;
        const double yVel    = state.vy;

        // printf("y Error: %f Current: %f Desired: %f\n", yError, state.y, target.y);
        // printf("x Error:  %f Current: %f Desired: %f \n", xError, state.x, target.x);
        // printf("z Error:  %f Current: %f Desired: %f \n\n", dz, state.z, target.z);

        double desiredPitch = posXPid_.update( yError, yVel, dt );
        double desiredRoll  = -posYPid_.update( xError, xVel, dt );
        
        desiredPitch = clamp_value( desiredPitch, -AUTOPILOT_MAX_ANGLE_RAD, AUTOPILOT_MAX_ANGLE_RAD );
        desiredRoll  = clamp_value( desiredRoll,  -AUTOPILOT_MAX_ANGLE_RAD, AUTOPILOT_MAX_ANGLE_RAD );

        const double yawError = wrap_angle( desiredYaw - state.yaw );
        // printf("Roll Error: %f Roll: %f Desired: %f\n", wrap_angle( desiredRoll - state.roll ), state.roll, desiredRoll);
        // printf("Pitch Error: %f Pitch: %f Desired: %f \n", wrap_angle( desiredPitch - state.pitch ), state.pitch, desiredPitch);
        // printf("Yaw Error: %f Yaw: %f Desired: %f \n\n", yawError, state.yaw, desiredYaw);


        output.delmx = rollPid_.update( wrap_angle( desiredRoll - state.roll ), state.p, dt );
        output.delmy = pitchPid_.update( wrap_angle( desiredPitch - state.pitch ), state.q, dt );
        output.delmz = yawPid_.update( yawError, state.r, dt );
        // output.delf  = clamp_value( posZPid_.update( dz, state.vz, dt ), -0.2, 0.7 );
        output.delf  = clamp_value( posZPid_.update( dz, state.vz, dt ), -1.0, 0.7 );

        output.rollStick = clamp_value( desiredRoll / AUTOPILOT_MAX_ANGLE_RAD, -1.0, 1.0 );
        output.pitchStick = clamp_value( desiredPitch / AUTOPILOT_MAX_ANGLE_RAD, -1.0, 1.0 );
        output.yawStick = clamp_value( yawError / KFF_Z, -1.0, 1.0 );
        output.thrustStick = output.delf;
        output.waypointIndex = activeWaypoint_;
        output.waypointDistance = distance;
        output.valid = true;

        return output;

    }

private:
    SimplePid posXPid_;
    SimplePid posYPid_;
    SimplePid posZPid_;
    SimplePid rollPid_;
    SimplePid pitchPid_;
    SimplePid yawPid_;
    AutopilotWaypoint waypoints_[AUTOPILOT_WAYPOINT_COUNT];
    int waypointCount_;
    int activeWaypoint_;
    double timeAtWaypoint_;
};

/*THERMAL CAMERA STUFF*/

void standard_dev(const float* val, float& stan_dev, float& high_stan_dev) {
    float sum = 0.0f;
    float sum_square = 0.0f;

    for (int i = 0; i < 768; i++) {
        sum += val[i];
    }
    float mean = sum / 768;

    for (int i = 0; i < 768; i++) {
        sum_square += pow(val[i] - mean, 2);
    }
    stan_dev = sqrt(sum_square / 768);

    if (stan_dev > high_stan_dev) {
        high_stan_dev = stan_dev;
    }
}

struct Segment {
    int start;
    int end;
};

struct Result {
    int row;
    Segment seg_num[2];
};

struct Centroid {
    float col;
    float row;
    bool true_positive;
};

Centroid frame_centroid(const float* frame) {
    Centroid centroid = { 0.0f, 0.0f, false };
    Result results[24];
    int valid_rows = 0;

    for (int y = 0; y < SENSOR_W; y++) {
        Segment seg_noise[3];
        int seg_index = 0;
        bool in_segment = false;
        for (int x = 0; x < SENSOR_H; x++) {
            float val = frame[SENSOR_H * y + (SENSOR_H - 1 - x)];
            bool hot = val > 30.0f; // Threshold for "hot" pixels, adjust as needed
            if (hot && !in_segment) {
                // Start of a new segment
                if (seg_index < 3) {
                    seg_noise[seg_index].start = x;
                    seg_noise[seg_index].end = x;
                }
                in_segment = true;
            }
            else if (hot && in_segment) {
                seg_noise[seg_index].end = x;
            }
            else if (!hot && in_segment) {
                // End of the current segment
                in_segment = false;
                seg_index++;
            }
        }
        if (in_segment) seg_index++; // Close any open segment at the end of the row
        if (seg_index == 2 && valid_rows < 24) {
            results[valid_rows].row = y;
            results[valid_rows].seg_num[0] = seg_noise[0];
            results[valid_rows].seg_num[1] = seg_noise[1];
            valid_rows++;
        }
    }
    // end of two_pass

    if (valid_rows == 0) {
        #if DISP_THERMALCAM
            printf("No valid rows found for centroid calculation.\n");
        #endif
        return centroid;
    }

    float col_centroid = 0.0f;
    float row_centroid = 0.0f;
    int segment_count = 0;

    for (int i = 0; i < valid_rows; i++) {
        int row = results[i].row;
        for (int j = 0; j < 2; j++) {
            int start = results[i].seg_num[j].start;
            int end = results[i].seg_num[j].end;
            if (end >= start) {
                col_centroid += (start + end) / 2.0f;
                row_centroid += row;
                segment_count++;
            }
        }
    }

    if (segment_count > 0) {
        centroid.col = col_centroid / segment_count;
        centroid.row = row_centroid / segment_count;
        centroid.true_positive = true;
    }
    if (centroid.col > SENSOR_H || centroid.row > SENSOR_W || centroid.col < 0 || centroid.row < 0) {
        #if DISP_THERMALCAM
            printf("Centroid calculation failed. No valid segments found.\n");
        #endif
		centroid.true_positive = false;
        return centroid;
    }
    else {
        #if DISP_THERMALCAM
            printf("Centroid: (%.3f, %.3f)\n", centroid.col, centroid.row);
        #endif
        return centroid;
    }
}

int position_centroid(const float* frame) {
    Centroid hot_centroid = frame_centroid(frame);

    if (!hot_centroid.true_positive) {
        #if DISP_THERMALCAM
            printf("No hot centroid detected. Cannot determine position.\n");
        #endif
        return 0;
    }
    int frame_centroid_x = SENSOR_H / 2; //columns? == 16
    int frame_centroid_y = SENSOR_W / 2; //rows? == 12
    int offset_x = (int)(hot_centroid.col - frame_centroid_x);
    int offset_y = (int)(hot_centroid.row - frame_centroid_y);

    int tolerance = 3; // Adjust as needed
    if (offset_x > tolerance) {
        #if DISP_THERMALCAM
            printf("Move left by %d pixels\n", offset_x);
        #endif
    }
    else if (offset_x < -tolerance) {
        #if DISP_THERMALCAM
            printf("Move right by %d pixels\n", -offset_x);
        #endif
    }
    else {
        #if DISP_THERMALCAM
            printf("Horizontal position is centered.\n");
        #endif
    }
    if (offset_y > tolerance) {
        #if DISP_THERMALCAM
            printf("Move forward by %d pixels\n", offset_y);
        #endif
    }
    else if (offset_y < -tolerance) {
        #if DISP_THERMALCAM
            printf("Move backward by %d pixels\n", -offset_y);
        #endif
    }
    else {
        #if DISP_THERMALCAM
            printf("Vertical position is centered.\n");
        #endif
    }
    return 1;
}
char temp_to_char(float val, int minTemp, int maxTemp)
{
    // static const char ramp[] = {'1','2','3','4','5'};  // cold â†’ hot
    static const char ramp[] = { '.',';','&','#','@' };  // cold â†’ hot
    const int n = sizeof(ramp) - 1;

    if (maxTemp <= minTemp)
        return ramp[n / 2];

    float t = (val - minTemp) / (maxTemp - minTemp);

    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    // >>> non-linear contrast boost <<<
    float gamma = 0.8f;            // adjust to taste (0.4 â†’ strong, 0.8 â†’ subtle)
    t = std::pow(t, gamma);

    int idx = static_cast<int>(t * (n - 1) + 0.5f);
    return ramp[idx];
}
/* END THERMAL CAMERA STUFF */

extern onboardMocapClient_ref onboardMocapClient;
// extern mlx90640CentroidMessage_ref centroidMessage;

int main( int argc, char *argv[] ) {
    int count_open = 0;

    /*THERMAL CAMERA STUFF*/

    timespec newtime, oldtime;
	int startSec;
    int countThermalCam = 0;
    float stan_dev = 0;
    float high_stan_dev = 0;

	int minTemp = 100;
    int maxTemp = 0;

    char text_buffer[36];
    static uint16_t eeMLX90640[832];
    float emissivity = 1;
    uint16_t frame[834];
    static float mlx90640To[768];
    double sendFrame[768];
    float eTa;

    int count_frames = 0;
    
    /* END THERMAL CAMERA STUFF */
    ///////////////////
    float thermalCameraStd = 42;
	float posFireX = 42;
	float posFireY = 42;

    /*EKF Stuff*/
    SasEkf ekf;

    float ekf_x = 0;
    float ekf_y = 0;
    float ekf_z = 0;

    float ekf_vx = 0;
    float ekf_vy = 0;
    float ekf_vz = 0;

    float ekf_roll = 0;
    float ekf_pitch = 0;
    float ekf_yaw = 0;

    float ekf_p = 0;
    float ekf_q = 0;
    float ekf_r = 0;

    float ekf_bias_x = 0;
    float ekf_bias_y = 0;
    float ekf_bias_z = 0;

    float ekf_covariance = 0;
    int ekf_frameNum = -1;

    int pwm, armed = 0;
    int payloadDoor = 0;
    int payloadDoorSwitchInitialized = 0;
    int payloadDoorSwitchHigh = 0;
    int count = 0;
    double rollStick = 0, pitchStick = 0, yawStick = 0, thrustStick = -1;
    int throttleAllowed = 0;
    ///
    double servoControl = 0;
    ///
    float wx, wy, wz;
    float ax, ay, az;
    double wxTotal=0, wyTotal=0, wzTotal=0, axTotal=0, ayTotal=0, azTotal=0;
    double wxTotalAuto=0, wyTotalAuto=0, wzTotalAuto=0;
    double sx=0, sy=0, sz=0, ekfSx=0, ekfSy=0, ekfSz=0;
    double axTotalAuto=0, ayTotalAuto=0, azTotalAuto=0;
    int IMUsamples=0, IMUsamplesAuto=0;
    double p=0, q=0, r=0, pp=0, qq=0, rr=0;
    double delmx = 0, delmy = 0, delmz = 0, delf = -1;
    double dt = 1.0/PWM_HZ, time;

    double e, u1, x4x=0, x4y=0, x4z=0, x5x=0, x5y=0, x5z=0;
    double x4x_dot=0, x4y_dot=0, x4z_dot=0;
    double x5x_dot=0, x5y_dot=0, x5z_dot=0;
    double oldx4x_dot=0, oldx4y_dot=0, oldx4z_dot=0;
    double oldx5x_dot=0, oldx5y_dot=0, oldx5z_dot=0;
    double fom = 1;

    double motor0, motor1, motor2, motor3;
    int pwmMotor0, pwmMotor1, pwmMotor2, pwmMotor3;

    MS5611 barometer;
    float absPress;
    float mx, my, mz;
    char sampleBit = 0;

    int voltage = 12000, current = 0;
    short pilotPwm[7] = {1500,1500,1500,1500,1500,1500,1500};
    int autopilot = 0;
    double armTimeRemaining = 0;
    WaypointFollower waypointFollower;
    AutopilotControl autopilotControl;
    int pendingWaypointInitialization = 0;
    int lastAutopilotWaypointIndex = -1;
    ///
	int phaseOfFlight = 1;
	///

    int mocapDebugCount = 0;

    timespec now, startTime, controlTime, mocapTime;
    const long long controlPeriodNsec = NSEC_PER_SEC / PWM_HZ;
    const long long mocapPeriodNsec = NSEC_PER_SEC / PWM_HZ_MOCAP;

    auto reset_controller_states = [&]() {
        x4x = 0;
        x4y = 0;
        x4z = 0;
        x5x = 0;
        x5y = 0;
        x5z = 0;
        x4x_dot = 0;
        x4y_dot = 0;
        x4z_dot = 0;
        x5x_dot = 0;
        x5y_dot = 0;
        x5z_dot = 0;
        oldx4x_dot = 0;
        oldx4y_dot = 0;
        oldx4z_dot = 0;
        oldx5x_dot = 0;
        oldx5y_dot = 0;
        oldx5z_dot = 0;
    };

    auto reset_autopilot_controller = [&]() {
        waypointFollower.resetControllers();
        autopilotControl = AutopilotControl{};
        lastAutopilotWaypointIndex = -1;
    };

    auto clear_autopilot_mission = [&]() {
        waypointFollower.clearMission();
        reset_autopilot_controller();
        pendingWaypointInitialization = 0;
    };

    if( check_apm() ) {
        return EXIT_FAILURE;
    }

    /* rc input setup */

    printf( "Init rc input\n" );
    auto rcin = get_rcin();
    rcin->initialize();

    /* motor control setup */

    printf( "Init motors\n" );
    auto motors = get_rcout();
    /* INIT SERVO */
    auto servo = get_rcout();

    if( getuid() ) {
        printf( "Not root. Please launch like this: sudo %s\n", argv[0] );
        return EXIT_FAILURE;
    }
    if( !( motors->initialize(PWM_MOTOR0) ) ) { return EXIT_FAILURE; }
    if( !( motors->initialize(PWM_MOTOR1) ) ) { return EXIT_FAILURE; }
    if( !( motors->initialize(PWM_MOTOR2) ) ) { return EXIT_FAILURE; }
    if( !( motors->initialize(PWM_MOTOR3) ) ) { return EXIT_FAILURE; }
    usleep(10);

    motors->set_frequency( PWM_MOTOR0, PWM_HZ );
    motors->set_frequency( PWM_MOTOR1, PWM_HZ );
    motors->set_frequency( PWM_MOTOR2, PWM_HZ );
    motors->set_frequency( PWM_MOTOR3, PWM_HZ );

    // servo->set_frequency(PWM_OUTPUT, 50);
    usleep(10);

    if ( !( motors->enable( PWM_MOTOR0 ) ) ) { return EXIT_FAILURE; }
    if ( !( motors->enable( PWM_MOTOR1 ) ) ) { return EXIT_FAILURE; }
    if ( !( motors->enable( PWM_MOTOR2 ) ) ) { return EXIT_FAILURE; }
    if ( !( motors->enable( PWM_MOTOR3 ) ) ) { return EXIT_FAILURE; }
    set_all_motors_min( *motors );
    usleep(10);

    /* IMU measurement setup */

    printf( "Init IMU and Mag\n" );
    auto imu = get_inertial_sensor();
    if( !imu ) {
        printf( "IMU not found\n" );
        return EXIT_FAILURE;
    }
    if( !imu->probe() ) {
        printf( "IMU not enabled\n" );
        return EXIT_FAILURE;
    }
    imu->initialize();

    /* air data */

    printf( "Init air data\n" );
    barometer.initialize();
    barometer.refreshTemperature();
    usleep(10000); // Waiting for temperature data ready
    barometer.readTemperature();
    barometer.refreshPressure();

    /* A2D */

    printf( "Init a2d\n" );
    auto adc = get_converter();
    adc->initialize();

    /* get network ready */

    openPort();
    openPortMocap();
    openPortThermalCam();

    /* main loop */

    printf( "SAS starting\n" );

    clock_gettime( CLOCK_MONOTONIC, &startTime );
    controlTime = startTime;
    mocapTime = startTime;

    // THERMAL CAMERA
    clock_gettime( CLOCK_REALTIME, &oldtime );
	startSec = oldtime.tv_sec;
    switch(FPS){
        case 1:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b001);
            break;
        case 2:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b010);
            break;
        case 4:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b011);
            break;
        case 8:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b100);
            break;
        case 16:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b101);
            break;
        case 32:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b110);
            break;
        case 64:
            MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b111);
            break;
        default:
            printf("Unsupported framerate: %d", FPS);
            return 1;
    }
    MLX90640_SetChessMode(MLX_I2C_ADDR);
    printf("Thermal camera selected chess mode\n");

    paramsMLX90640 mlx90640;
    MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    printf("Thermal camera going for while\n");
    // END THERMAL CAMERA

    // servo->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
    
    while (true) {

        // /* THERMAL CAMERA STUFF */
        // clock_gettime( CLOCK_REALTIME, &newtime );
		// if( diff( oldtime, newtime ).tv_nsec > (1000000000/LOOP_HZ) ) {
		// 	oldtime.tv_nsec += (1000000000/LOOP_HZ);
		// 	if( oldtime.tv_nsec > 1000000000 ) {
		// 		oldtime.tv_sec--;
		// 		oldtime.tv_nsec -= 1000000000;
		// 	}
		// 	countThermalCam++;

        //     if ( 5 == (countThermalCam%10) ) {
        //         count_frames++;
        //         MLX90640_GetFrameData(MLX_I2C_ADDR, frame);

        //         eTa = MLX90640_GetTa(frame, &mlx90640);
        //         MLX90640_CalculateTo(frame, &mlx90640, emissivity, eTa, mlx90640To);

        //         MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, mlx90640To, 1, &mlx90640);
        //         MLX90640_BadPixelsCorrection((&mlx90640)->outlierPixels, mlx90640To, 1, &mlx90640);

        //         // standard_dev(mlx90640To,stan_dev, high_stan_dev);
		// 		// frame_centroid(mlx90640To);

        //         /* send data to onboard */
        //         for(int i = 0; i < SENSOR_W; i++){
        //             for(int j = 0; j < SENSOR_H; j++){
        //                 sendFrame[SENSOR_H * i + j] = (double)mlx90640To[SENSOR_H * i + (SENSOR_H - 1 - j)];
        //             }
        //         }

        //         // sendThermalData( &sendFrame[0] );

        //         minTemp = 100;
        //         maxTemp = 0;
        //         for(int i=0;i<768;i++){
        //             if(minTemp > mlx90640To[i]) minTemp = mlx90640To[i];
        //             if(maxTemp < mlx90640To[i]) maxTemp = mlx90640To[i];
        //         }

        //         for(int y = 0; y < SENSOR_W; y++){
        //             for(int x = 0; x < SENSOR_H; x++){
        //                 /* flips image */
        //                 // float val = mlx90640To[SENSOR_H * y + (SENSOR_H - 1 - x)];
        //                 float val = mlx90640To[SENSOR_H*(SENSOR_W - 1 - y) + x];
        //                 char c = temp_to_char(val, minTemp, maxTemp);
        //                 #if DISP_THERMALCAM
        //                     putchar(c);
        //                 #endif
        //             }
        //             #if DISP_THERMALCAM
        //                 putchar('\n'); // end of row
        //             #endif    
        //         }
        //         #if DISP_THERMALCAM
        //             printf("\n\n"); // end of frame
        //         #endif
        //         // first couple frames are sort of trash
        //         if (count_frames > 5) {
        //             standard_dev(mlx90640To, stan_dev, high_stan_dev);
        //             Centroid hot_centroid = frame_centroid(mlx90640To); // has row, column, and bool saying if it's valid or not
        //             position_centroid(mlx90640To);
        //             // printf("Row: %f, Col: %f\n", hot_centroid.row, hot_centroid.col);
        //             // if( hot_centroid.true_positive && hot_centroid.col >= 13.0f && hot_centroid.col <= 19.0f && hot_centroid.row >= 9.0f && hot_centroid.row <= 15.0f) {
        //             //     // payload door is closed, open it
        //             //     printf("open\n");
        //             //     servo->set_duty_cycle(PWM_OUTPUT, SERVO_MAX);
        //             //     servoControl = 1;
        //             //     count_open++;
        //             // }
        //             // else if (count_open > 10) {
        //             //     printf("close\n");
        //             //     servo->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
        //             //     servoControl = 0;
        //             //     count_open = 0;
        //             // }

        //             #if DISP_THERMALCAM
        //                 printf("Frame Num: %d\tMax:%d C\tMin:%d C\t Frame Std_Dev:%.6f\tHigh Std_Dev:%.6f\n",count_frames,maxTemp,minTemp, stan_dev, high_stan_dev);
        //             #endif    
        //         }
        //     }
        // }
        /* END THERMAL CAMERA STUFF */
        /* read IMU measurements */

        imu->update();
        imu->read_gyroscope( &wx, &wy, &wz );
        wxTotal += wx;
        wyTotal += wy;
        wzTotal += wz;
        wxTotalAuto += wx;
        wyTotalAuto += wy;
        wzTotalAuto += wz;
        imu->read_accelerometer( &ax, &ay, &az );
        axTotal += ax;
        ayTotal += ay;
        azTotal += az;
        axTotalAuto += ax;
        ayTotalAuto += ay;
        azTotalAuto += az;
        IMUsamples++;
        IMUsamplesAuto++;
        #if DEBUG
            printf( "wx=%f\twy=%f\twz=%f\ts%d\n", wz, wy, wz, IMUsamples );
        #endif
		// printf("%ld.%ld\n", (long)now.tv_sec, now.tv_nsec);
        /* read clock, is it time to update? */

        clock_gettime( CLOCK_MONOTONIC, &now );
		// 250Hz
        if( period_elapsed( mocapTime, now, mocapPeriodNsec ) ) {
			// printf("%ld.%ld\n", (long)now.tv_sec, now.tv_nsec);
            time = elapsed_seconds( startTime, now );
            //printf("\nreading optitrak \n");
            // const int gotMocap = readDatalink( );
            if( readDatalink( ) ) {
                //printf("\nOptitrak reading successful \n");
                ekf.updateMocap( onboardMocapClient );
            }

            #if DISP_MOCAP_READINGS
            if (onboardMocapClient.valid && 0 == (mocapDebugCount++ % 25)) {
                printf("Pos_x=%.2f\n",onboardMocapClient.pos_x);
                printf("Pos_y=%.2f\n",onboardMocapClient.pos_y);
                printf("Pos_z=%.2f\n",onboardMocapClient.pos_z);
                printf("qx=%.2f\n",onboardMocapClient.qx);
                printf("qy=%.2f\n",onboardMocapClient.qy);
                printf("qz=%.2f\n",onboardMocapClient.qz);
                printf("qw=%.2f\n",onboardMocapClient.qw);
                printf("frameNum=%d\n",onboardMocapClient.frameNum);
                printf("Valid=%d\n",onboardMocapClient.valid);
            }
            #else
            (void)mocapDebugCount;
            #endif
        }
        /// 400 Hz
        const long long controlElapsedNsec = elapsed_nsec( controlTime, now );
        const double controlElapsed = (double)controlElapsedNsec / (double)NSEC_PER_SEC;
        if( period_elapsed( controlTime, now, controlPeriodNsec ) ) {
			// printf("%ld.%ld\n", (long)now.tv_sec, now.tv_nsec);
            time = elapsed_seconds( startTime, now );
            #if DEBUG
            if( 0==(count%40) ) printf( "time = %.4f\n", time );
                //printf( "control elapsed = %lld\n", elapsed_nsec( controlTime, now ) );
                printf( "control elapsed = %lld\n", controlElapsedNsec );
            #endif
            count++;

            /* prepare IMU measurements */

            if( IMUsamples > 0 ) {
                pp = +wyTotal/IMUsamples;  
                qq = +wxTotal/IMUsamples;  /* rad/sec */
                rr = -wzTotal/IMUsamples;
                ekfSx = -ayTotal/IMUsamples;
                ekfSy = -axTotal/IMUsamples;
                ekfSz = +azTotal/IMUsamples;
                ekf.stepImu( controlElapsed, pp, qq, rr, ekfSx, ekfSy, ekfSz );
                // ekfSx = -ayTotal/IMUsamples;
                // ekfSy = -axTotal/IMUsamples;
                // ekfSz = +azTotal/IMUsamples;
                // ekf.stepImu( dt, pp, qq, rr, -ayTotal/IMUsamples, -axTotal/IMUsamples, +azTotal/IMUsamples );
            }
            #if DEBUG
                if( (count%40)<1 ) printf( "t=%.2f\tp=%.2f\tq=%.2f\tr=%.2f\ts%d\n", time, pp, qq, rr, IMUsamples );
            #endif

            IMUsamples = 0;
            wxTotal = 0;
            wyTotal = 0;
            wzTotal = 0;
            axTotal = 0;
            ayTotal = 0;
            azTotal = 0;

            ekf_x = (float)ekf.state( EKF_POS_X );
            ekf_y = (float)ekf.state( EKF_POS_Y );
            ekf_z = (float)ekf.state( EKF_POS_Z );
            ekf_vx = (float)ekf.state( EKF_VEL_X );
            ekf_vy = (float)ekf.state( EKF_VEL_Y );
            ekf_vz = (float)ekf.state( EKF_VEL_Z );
            ekf_roll = (float)ekf.state( EKF_ROLL );
            ekf_pitch = (float)ekf.state( EKF_PITCH );
            ekf_yaw = (float)ekf.state( EKF_YAW );
            ekf_p = (float)ekf.bodyRate( 0 );
            ekf_q = (float)ekf.bodyRate( 1 );
            ekf_r = (float)ekf.bodyRate( 2 );
            ekf_bias_x = (float)ekf.state( EKF_BIAS_X );
            ekf_bias_y = (float)ekf.state( EKF_BIAS_Y );
            ekf_bias_z = (float)ekf.state( EKF_BIAS_Z );
            ekf_covariance = (float)ekf.covarianceSummary();
            ekf_frameNum = ekf.lastMocapFrame();

            #if EKF_PRINT_STATE
                printf( "EKF t=%.4f frame=%d pos=(%.3f %.3f %.3f) vel=(%.3f %.3f %.3f) att=(%.3f %.3f %.3f) pqr=(%.3f %.3f %.3f) abias=(%.3f %.3f %.3f) cov=%.6g\n",
                        time, ekf_frameNum,
                        ekf_x, ekf_y, ekf_z,
                        ekf_vx, ekf_vy, ekf_vz,
                        ekf_roll, ekf_pitch, ekf_yaw,
                        ekf_p, ekf_q, ekf_r,
                        ekf_bias_x, ekf_bias_y, ekf_bias_z,
                        ekf_covariance );
            #endif

            if( 0 == (count%4) ) {

                /* send IMU data out */

                if( IMUsamplesAuto > 0 ) {
                    p  = +wyTotalAuto/IMUsamplesAuto;  /* get mounting orientation right here, raw is ENU */
                    q  = +wxTotalAuto/IMUsamplesAuto;  /* rad/sec */
                    r  = -wzTotalAuto/IMUsamplesAuto;
                    sx = -ayTotalAuto/IMUsamplesAuto; /* why is it different that the gyro? */
                    sy = -axTotalAuto/IMUsamplesAuto;
                    sz = +azTotalAuto/IMUsamplesAuto;
                }
                //if( (count%400)<3 ) printf( "t=%.2f\tp=%.2f\tq=%.2f\tr=%.2f\ts%d\n", time, p, q, r, IMUsamples );
                #if DEBUG
                    if( (count%400)<3 ) printf( "t=%.2f\tp=%.2f\tq=%.2f\tr=%.2f\ts%d\n", time, p, q, r, IMUsamples );
                #endif
                IMUsamplesAuto = 0;
                wxTotalAuto = 0;
                wyTotalAuto = 0;
                wzTotalAuto = 0;
                axTotalAuto = 0;
                ayTotalAuto = 0;
                azTotalAuto = 0;
   
                /* 10 Hz IMU and EKF data transmission */
                /// Could move EKF data transmission
                /// to where the data is updated?
                 if( (count%40)<3 ) {
					// printf("%ld.%ld\n", (long)now.tv_sec, now.tv_nsec);
                    // printf("Accel: [%f %f %f]\n", sx, sy, sz);
                    sendIMUdata( p, q, r, sx, sy, sz );
                    sendEKFdata( ekf_x, ekf_y, ekf_z, ekf_vx, ekf_vy, ekf_vz, ekf_roll, ekf_pitch, ekf_yaw, ekf_p, ekf_q, ekf_r, ekf_bias_x, ekf_bias_y, ekf_bias_z, ekf_covariance, ekf_frameNum );
                 }

                 if ( readThermalCentroidData() ) {
                    printf("hot_centroid_col:  %f\n", mlx90640Centroid.hot_centroid_col);
                    printf("hot_centroid_row:  %f\n", mlx90640Centroid.hot_centroid_row);
                    printf("hot_centroid_x:  %d\n", mlx90640Centroid.frame_centroid_x);
                    printf("hot_centroid_y:  %d\n", mlx90640Centroid.frame_centroid_y);
                    printf("hot_centroid_valid:  %d\n\n", mlx90640Centroid.hot_centroid_valid);
                 }

                /* read pilot inputs */

                pwm = rcin->read( PWM_ROLL );
                //printf("pwm_roll: %d \n", pwm);
                if( pwm <= 0 ) {
                    rollStick = 0;
                } else {
                    rollStick = ((double)( pwm - ROLL_MIN )*2)/( ROLL_MAX - ROLL_MIN ) - 1;
                    pilotPwm[PWM_ROLL] = pwm;
                    // printf("rollStick: %.2f \n", rollStick);
                }

                pwm = rcin->read( PWM_PITCH );
                //printf("pwm_pitch: %d \n", pwm);
                if( pwm <= 0 ) {
                    pitchStick = 0;
                } else {
                    pitchStick = ((double)( pwm - PITCH_MIN )*2)/( PITCH_MAX - PITCH_MIN ) - 1;
                    pilotPwm[PWM_PITCH] = pwm;
                }

                pwm = rcin->read( PWM_YAW );
                //printf("pwm_yaw: %d \n", pwm);
                if( pwm <= 0 ) {
                    yawStick = 0;
                } else {
                    yawStick = ((double)( pwm - YAW_MIN )*2)/( YAW_MAX - YAW_MIN ) - 1;
                    pilotPwm[PWM_YAW] = pwm;
                }

                pwm = rcin->read( PWM_THRUST );
                if( pwm <= 0 ) {
                    thrustStick = -1;
                } else {
                    thrustStick = ((double)( pwm - THRUST_MIN )*2)/( THRUST_MAX - THRUST_MIN ) - 1;
                    pilotPwm[PWM_THRUST] = pwm;
                }


                // PAYLOAD DOOR CONTROL
                // Edge-triggered toggle: one low-to-high switch transition changes the door state.
                // disabled for auto trigger testing
                #if 0
                pwm = rcin->read( PWM_PAYLOAD_DOOR );
                if( pwm <= 0 ) {
                    if( servoControl ) {
                        servo->set_duty_cycle(PWM_OUTPUT, SERVO_MAX);
                    }
                    servoControl = 0;
                } else {
                    if( pwm > PAYLOAD_DOOR_THRESHOLD ) {
                        // payload door is closed, open it
                        // printf("open\n");
                        servo->set_duty_cycle(PWM_OUTPUT, SERVO_MAX);
                        servoControl = 1;
                    }
                    else {
                        // printf("close\n");
                        servo->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
                        servoControl = 0;
                    }
                }
                #endif
                


                const int armedBeforeSwitch = armed;
                const int autopilotBeforeSwitch = autopilot;

                pwm = rcin->read( PWM_ARM );
                if( pwm <= 0 ) {
                    if( armed ) {
                        printf("Restting Controller States\n");
                        reset_controller_states();
                    }
                    // printf("Drone Disarmed\n");
                    armed = 0; /* maybe turn autopilot on? */
                    thrustStick = -1;
                    throttleAllowed = 0;
                    // concerned about this part, this isn't in the og code
                    // printf("Setting motors to min\n");
                    set_all_motors_min( *motors );
                } else {
                    if( pwm > ARMED_THRESHOLD ) {
                        if( 0 == armed ) {
                            armTimeRemaining = ARM_TIME;
                            throttleAllowed = 0;
                            // printf("Restting Controller States\n");
                            reset_controller_states();
                        }

                        armed = 1;

                        if( thrustStick <= -1 + THROTTLEMINTOL ) { /* throttle command near min */
                            throttleAllowed = 1; /* special logic to avoid accidental arming at high throttle */
                                                 /* forces one to move throttle to min first before motors will follow it */
                        }
                    } else {
                        if( 1 == armed ) {
                            // this function name is misleading, this sets the duty cycle
                            // this does NOT set the motor rpm to min
                            printf("Setting all motors to min\n");
                            set_all_motors_min( *motors );
                            printf("Resetting Controller States\n");
                            reset_controller_states();
                        }
                        armed = 0;
                        thrustStick = -1;
                        throttleAllowed = 0;
                    }
                    pilotPwm[PWM_ARM] = pwm;
                }

                pwm = rcin->read( PWM_AUTO );
                if( pwm <= 0 ) {
                    autopilot = 0;
                } else {
                    if( pwm > AUTO_THRESHOLD ) {
                        if( 0 == autopilot ) {
                            autopilot = 1;
                            //printf("\nAUTOPILOT ON");
                        }
                    } else {
                        if( 1 == autopilot ) {
                            autopilot = 0;
                        }
                    }
                }
                /* pilotPwm[PWM_AUTO] = pwm; this is handled below */

                if( armed != armedBeforeSwitch ) {
                    reset_autopilot_controller();
                    if( 1 == armed ) {
                        pendingWaypointInitialization = 1;
                    } else {
                        clear_autopilot_mission();
                    }
                }

                if( autopilot != autopilotBeforeSwitch ) {
                    printf( "Autopilot %s\n", autopilot ? "enabled" : "disabled" );
                    reset_controller_states();
                    reset_autopilot_controller();
                }

                #if DEBUG
                    // printf("thrustStick = %.2f \n rollStick = %.2f \n pitchStick = %.2f \n yawStick = %.2f \n", thrustStick, rollStick, pitchStick, yawStick);
                #endif
            }

            if( pendingWaypointInitialization && 1 == armed && ekf_frameNum >= 0 ) {
                waypointFollower.initializeMission( ekf_x, ekf_y, ekf_z, ekf_yaw );
                autopilotControl = AutopilotControl{};
                lastAutopilotWaypointIndex = -1;
                pendingWaypointInitialization = 0;
            }

            const double autopilotDt = clamp_value( controlElapsed > 0.0 ? controlElapsed : dt,
                                                    1.0e-3, 0.05 );
            const int autopilotMissionActive = ( 1 == autopilot ) &&
                                               ( 1 == armed ) &&
                                               waypointFollower.missionReady();
            if( autopilotMissionActive ) {
                const AutopilotState autopilotState = {
                    ekf_x, ekf_y, ekf_z,
                    ekf_vx, ekf_vy, ekf_vz,
                    ekf_roll, ekf_pitch, ekf_yaw,
                    ekf_p, ekf_q, ekf_r
                };
                autopilotControl = waypointFollower.update( autopilotState, autopilotDt );
                if( autopilotControl.valid ) {
                    rollStick = autopilotControl.rollStick;
                    pitchStick = autopilotControl.pitchStick;
                    yawStick = autopilotControl.yawStick;
                    thrustStick = autopilotControl.thrustStick;

                    if( autopilotControl.waypointIndex != lastAutopilotWaypointIndex ) {
                        const AutopilotWaypoint* waypoint = waypointFollower.currentWaypoint();
                        if( waypoint ) {
                            printf( "Autopilot tracking waypoint %d/%d at [%.2f %.2f %.2f], distance=%.2f m\n",
                                    autopilotControl.waypointIndex + 1,
                                    waypointFollower.waypointCount(),
                                    waypoint->x, waypoint->y, waypoint->z,
                                    autopilotControl.waypointDistance );
                        }
                        lastAutopilotWaypointIndex = autopilotControl.waypointIndex;
                    }
                }
            }

            /*10 Hz - send control inputs*/
            if( (count%40)<3 ) {
                sendControlInputs( rollStick, pitchStick, yawStick, thrustStick, servoControl );
                //printf("%f %f %f %f %f\n", rollStick, pitchStick, yawStick, thrustStick, servoControl);
            }

            if( autopilotMissionActive && autopilotControl.valid ) {
                delmx = autopilotControl.delmx;
                delmy = autopilotControl.delmy;
                delmz = autopilotControl.delmz;
                delf = autopilotControl.delf;
                // printf("delf: %f \n", delf);
            } else {
                /* thrust control */

                delf = thrustStick;

                /* feedforward and stability augmentation */
                /* use this to avoid windup issues when on the ground, etc. */

                fom = LIMIT( ( 1.0 + delf )/0.2, 0, 1 ); /* 0 when throttle all the way down, 1 once throttle at 10% */

                e = rollStick*KFF_X - x4x;
                u1 = -pp*KP_X + e*KI_X;
                //u1 = 0; // pitch testing
                #if DISP_E
                    printf("rollStick: e = %.2f \n", e);
                #endif
                // printf("roll u1: %+-.3f \n", u1);

                delmx = ( x5x*( TAUA - TAUM ) + u1*TAUM )/TAUA;
                if( 1 == armed ) {
                    x4x_dot = ( pp - rollStick*KFF_X/TAUI )*fom;
                } else {
                    x4x_dot = 0;
                }
                x5x_dot = ( u1 - x5x )/TAUA;
                x4x += ( x4x_dot*2 - oldx4x_dot )*dt;  oldx4x_dot = x4x_dot;
                x5x += ( x5x_dot*2 - oldx5x_dot )*dt;  oldx5x_dot = x5x_dot;

                e = pitchStick*KFF_Y - x4y;
                u1 = -qq*KP_Y + e*KI_Y;
                #if DISP_E
                    printf("pitchStick: e = %.2f \n", e);
                #endif
                // printf("Pitch u1: %+-.3f \n", u1);

                delmy = ( x5y*( TAUA - TAUM ) + u1*TAUM )/TAUA;
                if( 1 == armed ) {
                    x4y_dot = ( qq - pitchStick*KFF_Y/TAUI )*fom;
                } else {
                    x4y_dot = 0;
                }
                x5y_dot = ( u1 - x5y )/TAUA;
                x4y += ( x4y_dot*2 - oldx4y_dot )*dt;  oldx4y_dot = x4y_dot;
                x5y += ( x5y_dot*2 - oldx5y_dot )*dt;  oldx5y_dot = x5y_dot;

                e = yawStick*KFF_Z - rr;
                u1 = e*KP_Z + x4z*KI_Z;
                // u1 = 0; // pitch testing
                #if DISP_E
                    printf("yawStick: e = %.2f \n", e);
                #endif
                // printf("Yaw u1: %+-.3f \n", u1);

                delmz = ( x5z*( TAUA - TAUM ) + u1*TAUM )/TAUA;
                if( 1 == armed ) {
                    x4z_dot = e*fom;
                } else {
                    x4z_dot = 0;
                }
                x5z_dot = ( u1 - x5z )/TAUA;
                x4z += ( x4z_dot*2 - oldx4z_dot )*dt;  oldx4z_dot = x4z_dot;
                x5z += ( x5z_dot*2 - oldx5z_dot )*dt;  oldx5z_dot = x5z_dot;
            }

            #if DEBUG
                printf("delf, delmx, delmy, delmz: \n");
                printf( "f=%+-.3f mx=%+-.3f my=%+-.3f mz=%+-.3f\n", delf, delmx, delmy, delmz );
            #endif

            /* motor mixing */
            motor0 = - delmx + delmy + delmz + delf;
            motor1 = - delmx - delmy - delmz + delf;
            motor2 = + delmx - delmy + delmz + delf;
            motor3 = + delmx + delmy - delmz + delf;

            pwmMotor0 = LIMIT( PWM_MIN + ( PWM_MAX - PWM_MIN )*( 0.5 + 0.5*motor0 ), PWM_MIN, PWM_MAX );
            pwmMotor1 = LIMIT( PWM_MIN + ( PWM_MAX - PWM_MIN )*( 0.5 + 0.5*motor1 ), PWM_MIN, PWM_MAX );
            pwmMotor2 = LIMIT( PWM_MIN + ( PWM_MAX - PWM_MIN )*( 0.5 + 0.5*motor2 ), PWM_MIN, PWM_MAX );
            pwmMotor3 = LIMIT( PWM_MIN + ( PWM_MAX - PWM_MIN )*( 0.5 + 0.5*motor3 ), PWM_MIN, PWM_MAX );

            /* 10 Hz - sending motor pwm signals to ground station */
			if( (count%40)<3 ) {
				sendMotorPwm( pwmMotor0, pwmMotor1, pwmMotor2, pwmMotor3 );
			};

            #if DEBUG
                printf("motor0, motor1, motor2, motor3 \n");
                printf( "0=%.2f\t1=%.2f\t2=%.2f\t3=%.2f\n", motor0, motor1, motor2, motor3 );
                // if( (count%400) < 3 )
                printf("time, pwmMotor0, pwmMotor1, pwmMotor2, pwmMotor3 \n");
                printf( "\n\n%.2f\t0=%d\t1=%d\t2=%d\t3=%d\n", time, pwmMotor0, pwmMotor1, pwmMotor2, pwmMotor3 );
            #endif

            /* send out motor commands */
            if( 1 == armed ) {
                if( armTimeRemaining > 0 || 1 != throttleAllowed ) {
                    armTimeRemaining -= dt;
                    set_all_motors_min( *motors );  /* make sure all motors arm */
                } else {
                    motors->set_duty_cycle( PWM_MOTOR0, pwmMotor0 );
                    motors->set_duty_cycle( PWM_MOTOR1, pwmMotor1 );
                    motors->set_duty_cycle( PWM_MOTOR2, pwmMotor2 );
                    motors->set_duty_cycle( PWM_MOTOR3, pwmMotor3 );
                }
            } else {
                // This is new and should be tested
                set_all_motors_min( *motors );
            }

            /* 10 Hz stuff */
            if( 1 == (count%40) ) {
                imu->read_magnetometer( &mx, &my, &mz );
                if( sampleBit ) {
                    barometer.readPressure();
                    sampleBit = 0;
                } else {
                    barometer.readTemperature();
                    sampleBit = 1;
                }
                barometer.calculatePressureAndTemperature();
                absPress = barometer.getPressure()*0.0145038;
                // This is commented out since we don't use it
                // if( count > 1000 ) sendMagAirdata( mx, my, mz, absPress, voltage, current );
                #if DEBUG
                    printf( "Ps = %f\n", absPress );
                #endif
                if( sampleBit ) {
                    barometer.refreshPressure();
                } else {
                    barometer.refreshTemperature();
                }
            }
            /* 10Hz - sends control pwm values to the groundstation */
            if( 17 == (count%40) ) {
                if( 1 == autopilot ) {
                    pilotPwm[PWM_AUTO] = AUTO_ON;
                } else {
                    pilotPwm[PWM_AUTO] = AUTO_OFF;
                }
                sendPilotLink( pilotPwm );
            }

            /* 1 Hz stuff */
            if( 30 == (count%400) ) {
                voltage = (int)(adc->read( A2D_VOLTAGECHANNEL ) *15000/1378); // 1378 ADC read for 15.00V
                #if DEBUG
                    printf("Batt: %d\n",voltage);
                #endif
                current = (int)(adc->read( A2D_CURRENTCHANNEL ) * 200); // *200
                #if DEBUG
                    printf("Curr: %d\n",current);
                #endif
                sendMisc1Hz( voltage, current, phaseOfFlight, thermalCameraStd, posFireX, posFireY );
                //printf("%d %d\n", voltage, current);
            }
        }
        usleep(1);  /* release CPU */
    }
    return 0;
}