/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "TianJin2020"
 * 	found in "tianan2020.asn"
 * 	`asn1c -gen-PER`
 */

#ifndef	_ReferencePathList_H_
#define	_ReferencePathList_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ReferencePath;

/* ReferencePathList */
typedef struct ReferencePathList {
	A_SEQUENCE_OF(struct ReferencePath) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ReferencePathList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ReferencePathList;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ReferencePath.h"

#endif	/* _ReferencePathList_H_ */
#include <asn_internal.h>
