/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "TianJin2020"
 * 	found in "tianan2020.asn"
 * 	`asn1c -gen-PER`
 */

#ifndef	_RTEList_H_
#define	_RTEList_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RTEData;

/* RTEList */
typedef struct RTEList {
	A_SEQUENCE_OF(struct RTEData) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RTEList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RTEList;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RTEData.h"

#endif	/* _RTEList_H_ */
#include <asn_internal.h>