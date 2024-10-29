/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "TianJin2020"
 * 	found in "tianan2020.asn"
 * 	`asn1c -gen-PER`
 */

#include "RoadSideInformation.h"

static int
memb_id_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const OCTET_STRING_t *st = (const OCTET_STRING_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	size = st->size;
	
	if((size == 8)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_per_constraints_t asn_PER_memb_id_constr_4 GCC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 0,  0,  8,  8 }	/* (SIZE(8..8)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_RoadSideInformation_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct RoadSideInformation, msgCnt),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MsgCount,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"msgCnt"
		},
	{ ATF_POINTER, 1, offsetof(struct RoadSideInformation, moy),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MinuteOfTheYear,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"moy"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RoadSideInformation, id),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		memb_id_constraint_1,
		&asn_PER_memb_id_constr_4,
		0,
		"id"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RoadSideInformation, refPos),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Position3D,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"refPos"
		},
	{ ATF_POINTER, 2, offsetof(struct RoadSideInformation, rtes),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RTEList,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"rtes"
		},
	{ ATF_POINTER, 1, offsetof(struct RoadSideInformation, rtss),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RTSList,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"rtss"
		},
};
static const int asn_MAP_RoadSideInformation_oms_1[] = { 1, 4, 5 };
static const ber_tlv_tag_t asn_DEF_RoadSideInformation_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RoadSideInformation_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* msgCnt */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* moy */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* id */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* refPos */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* rtes */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 } /* rtss */
};
static asn_SEQUENCE_specifics_t asn_SPC_RoadSideInformation_specs_1 = {
	sizeof(struct RoadSideInformation),
	offsetof(struct RoadSideInformation, _asn_ctx),
	asn_MAP_RoadSideInformation_tag2el_1,
	6,	/* Count of tags in the map */
	asn_MAP_RoadSideInformation_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	5,	/* Start extensions */
	7	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_RoadSideInformation = {
	"RoadSideInformation",
	"RoadSideInformation",
	SEQUENCE_free,
	SEQUENCE_print,
	SEQUENCE_constraint,
	SEQUENCE_decode_ber,
	SEQUENCE_encode_der,
	SEQUENCE_decode_xer,
	SEQUENCE_encode_xer,
	SEQUENCE_decode_uper,
	SEQUENCE_encode_uper,
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_RoadSideInformation_tags_1,
	sizeof(asn_DEF_RoadSideInformation_tags_1)
		/sizeof(asn_DEF_RoadSideInformation_tags_1[0]), /* 1 */
	asn_DEF_RoadSideInformation_tags_1,	/* Same as above */
	sizeof(asn_DEF_RoadSideInformation_tags_1)
		/sizeof(asn_DEF_RoadSideInformation_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_RoadSideInformation_1,
	6,	/* Elements count */
	&asn_SPC_RoadSideInformation_specs_1	/* Additional specs */
};

