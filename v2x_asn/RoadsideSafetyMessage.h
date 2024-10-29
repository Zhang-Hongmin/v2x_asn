/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "TianJin2020"
 * 	found in "tianan2020.asn"
 * 	`asn1c -gen-PER`
 */

#ifndef	_RoadsideSafetyMessage_H_
#define	_RoadsideSafetyMessage_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MsgCount.h"
#include <OCTET_STRING.h>
#include "Position3D.h"
#include "ParticipantList.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RoadsideSafetyMessage */
typedef struct RoadsideSafetyMessage {
	MsgCount_t	 msgCnt;
	OCTET_STRING_t	 id;
	Position3D_t	 refPos;
	ParticipantList_t	 participants;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RoadsideSafetyMessage_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RoadsideSafetyMessage;

#ifdef __cplusplus
}
#endif

#endif	/* _RoadsideSafetyMessage_H_ */
#include <asn_internal.h>
