/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "TianJin2020"
 * 	found in "tianan2020.asn"
 * 	`asn1c -gen-PER`
 */

#ifndef	_PositionalAccuracy_H_
#define	_PositionalAccuracy_H_


#include <asn_application.h>

/* Including external dependencies */
#include "SemiMajorAxisAccuracy.h"
#include "SemiMinorAxisAccuracy.h"
#include "SemiMajorAxisOrientation.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PositionalAccuracy */
typedef struct PositionalAccuracy {
	SemiMajorAxisAccuracy_t	 semiMajor;
	SemiMinorAxisAccuracy_t	 semiMinor;
	SemiMajorAxisOrientation_t	 orientation;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PositionalAccuracy_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PositionalAccuracy;

#ifdef __cplusplus
}
#endif

#endif	/* _PositionalAccuracy_H_ */
#include <asn_internal.h>