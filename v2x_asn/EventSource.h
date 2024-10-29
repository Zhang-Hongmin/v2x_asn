/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "TianJin2020"
 * 	found in "tianan2020.asn"
 * 	`asn1c -gen-PER`
 */

#ifndef	_EventSource_H_
#define	_EventSource_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum EventSource {
	EventSource_unknown	= 0,
	EventSource_police	= 1,
	EventSource_government	= 2,
	EventSource_meteorological	= 3,
	EventSource_internet	= 4,
	EventSource_detection	= 5
	/*
	 * Enumeration is extensible
	 */
} e_EventSource;

/* EventSource */
typedef long	 EventSource_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_EventSource;
asn_struct_free_f EventSource_free;
asn_struct_print_f EventSource_print;
asn_constr_check_f EventSource_constraint;
ber_type_decoder_f EventSource_decode_ber;
der_type_encoder_f EventSource_encode_der;
xer_type_decoder_f EventSource_decode_xer;
xer_type_encoder_f EventSource_encode_xer;
per_type_decoder_f EventSource_decode_uper;
per_type_encoder_f EventSource_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _EventSource_H_ */
#include <asn_internal.h>
