/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.2-dev */

#ifndef PB_IMU_PB_H_INCLUDED
#define PB_IMU_PB_H_INCLUDED
#include <pb.h>
#include "Vector3.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _Imu {
    bool has_angular_velocity;
    Vector3 angular_velocity;
    bool has_linear_acceleration;
    Vector3 linear_acceleration;
} Imu;


/* Initializer values for message structs */
#define Imu_init_default                         {false, Vector3_init_default, false, Vector3_init_default}
#define Imu_init_zero                            {false, Vector3_init_zero, false, Vector3_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define Imu_angular_velocity_tag                 1
#define Imu_linear_acceleration_tag              2

/* Struct field encoding specification for nanopb */
#define Imu_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  angular_velocity,   1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  linear_acceleration,   2)
#define Imu_CALLBACK NULL
#define Imu_DEFAULT NULL
#define Imu_angular_velocity_MSGTYPE Vector3
#define Imu_linear_acceleration_MSGTYPE Vector3

extern const pb_msgdesc_t Imu_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Imu_fields &Imu_msg

/* Maximum encoded size of messages (where known) */
#define Imu_size                                 34

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
