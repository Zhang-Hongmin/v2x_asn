/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "TianJin2020"
 * 	found in "tianan2020.asn"
 * 	`asn1c -gen-PER`
 */

#include "PathHistoryPointList.h"

static asn_per_constraints_t asn_PER_type_PathHistoryPointList_constr_1 GCC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 5,  5,  1,  23 }	/* (SIZE(1..23)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_PathHistoryPointList_1[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_PathHistoryPoint,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		""
		},
};
static const ber_tlv_tag_t asn_DEF_PathHistoryPointList_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_PathHistoryPointList_specs_1 = {
	sizeof(struct PathHistoryPointList),
	offsetof(struct PathHistoryPointList, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
asn_TYPE_descriptor_t asn_DEF_PathHistoryPointList = {
	"PathHistoryPointList",
	"PathHistoryPointList",
	SEQUENCE_OF_free,
	SEQUENCE_OF_print,
	SEQUENCE_OF_constraint,
	SEQUENCE_OF_decode_ber,
	SEQUENCE_OF_encode_der,
	SEQUENCE_OF_decode_xer,
	SEQUENCE_OF_encode_xer,
	SEQUENCE_OF_decode_uper,
	SEQUENCE_OF_encode_uper,
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_PathHistoryPointList_tags_1,
	sizeof(asn_DEF_PathHistoryPointList_tags_1)
		/sizeof(asn_DEF_PathHistoryPointList_tags_1[0]), /* 1 */
	asn_DEF_PathHistoryPointList_tags_1,	/* Same as above */
	sizeof(asn_DEF_PathHistoryPointList_tags_1)
		/sizeof(asn_DEF_PathHistoryPointList_tags_1[0]), /* 1 */
	&asn_PER_type_PathHistoryPointList_constr_1,
	asn_MBR_PathHistoryPointList_1,
	1,	/* Single element */
	&asn_SPC_PathHistoryPointList_specs_1	/* Additional specs */
};

