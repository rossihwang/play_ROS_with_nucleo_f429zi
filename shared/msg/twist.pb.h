/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.2-dev */

#ifndef PB_TWIST_PB_H_INCLUDED
#define PB_TWIST_PB_H_INCLUDED
#include <pb.h>
#include "Vector3.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _Twist {
    bool has_linear;
    Vector3 linear;
    bool has_angular;
    Vector3 angular;
} Twist;


/* Initializer values for message structs */
#define Twist_init_default                       {false, Vector3_init_default, false, Vector3_init_default}
#define Twist_init_zero                          {false, Vector3_init_zero, false, Vector3_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define Twist_linear_tag                         1
#define Twist_angular_tag                        2

/* Struct field encoding specification for nanopb */
#define Twist_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  linear,            1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  angular,           2)
#define Twist_CALLBACK NULL
#define Twist_DEFAULT NULL
#define Twist_linear_MSGTYPE Vector3
#define Twist_angular_MSGTYPE Vector3

extern const pb_msgdesc_t Twist_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Twist_fields &Twist_msg

/* Maximum encoded size of messages (where known) */
#define Twist_size                               34

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif