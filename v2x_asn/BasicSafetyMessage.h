/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "TianJin2020"
 * 	found in "tianan2020.asn"
 * 	`asn1c -gen-PER`
 */

#ifndef	_BasicSafetyMessage_H_
#define	_BasicSafetyMessage_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MsgCount.h"
#include <OCTET_STRING.h>
#include "DSecond.h"
#include "TimeConfidence.h"
#include "Position3D.h"
#include "TransmissionState.h"
#include "Speed.h"
#include "Heading.h"
#include "SteeringWheelAngle.h"
#include "AccelerationSet4Way.h"
#include "BrakeSystemStatus.h"
#include "VehicleSize.h"
#include "VehicleClassification.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PositionalAccuracy;
struct PositionConfidenceSet;
struct MotionConfidenceSet;
struct VehicleSafetyExtensions;
struct VehicleEmergencyExtensions;

/* BasicSafetyMessage */
typedef struct BasicSafetyMessage {
	MsgCount_t	 msgCnt;
	OCTET_STRING_t	 id;
	DSecond_t	 secMark;
	TimeConfidence_t	*timeConfidence	/* OPTIONAL */;
	Position3D_t	 pos;
	struct PositionalAccuracy	*posAccuracy	/* OPTIONAL */;
	struct PositionConfidenceSet	*posConfidence	/* OPTIONAL */;
	TransmissionState_t	 transmission;
	Speed_t	 speed;
	Heading_t	 heading;
	SteeringWheelAngle_t	*angle	/* OPTIONAL */;
	struct MotionConfidenceSet	*motionCfd	/* OPTIONAL */;
	AccelerationSet4Way_t	 accelSet;
	BrakeSystemStatus_t	 brakes;
	VehicleSize_t	 size;
	VehicleClassification_t	 vehicleClass;
	struct VehicleSafetyExtensions	*safetyExt	/* OPTIONAL */;
	struct VehicleEmergencyExtensions	*emergencyExt	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BasicSafetyMessage_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_BasicSafetyMessage;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PositionalAccuracy.h"
#include "PositionConfidenceSet.h"
#include "MotionConfidenceSet.h"
#include "VehicleSafetyExtensions.h"
#include "VehicleEmergencyExtensions.h"

#endif	/* _BasicSafetyMessage_H_ */
#include <asn_internal.h>
