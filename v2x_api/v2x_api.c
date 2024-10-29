/********************************
func：v2x通信，asn编解码接口
author:zyh
data:2023.8.14
*********************************/
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include <unistd.h> 
#include "v2x_api.h"
#include "MessageFrame.h"



//消息帧编码接口
int v2x_encode_msgFrame(MessageFrame_t *messageFrame, void *buffer, size_t buffer_size)
{
	asn_enc_rval_t ec;
	int len = 0;
	
	if ( (messageFrame == NULL) || (buffer == NULL) ) {
		return -1;
	}

#if 0
	char err_buf[256] = {0};
	int err_len = 0;
	/** check the messageframe_decode_struct is valid */
	if (asn_check_constraints(&asn_DEF_MessageFrame, (void *)messageFrame, err_buf, (size_t*)&err_len)) {
		printf("Encode MessageFrame noncompliance, ERR_BUFF:%s, ERR_SIZE:%d.\n", err_buf, err_len);
		return -1;
	}
#endif

	/** uper encode */
#if 1//asnc1 9.28
	ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, (void *)messageFrame, (void *)buffer, (size_t)buffer_size);
#else//asnc1 9.29
		ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, NULL, (void *)messageFrame, (void *)buffer, (size_t)buffer_size);
#endif
	if (ec.encoded == -1) {//编码失败一般是因为messageFrame结构体错误，如内部指针申请空间大小不匹配或指针赋值错误等原因
		fprintf(stderr, "Could not encode MessageFrame (at %s)\n", ec.failed_type ? ec.failed_type->name : "unknown");
		return -1;
	}
	
	//len = ec.encoded;
	len = ((ec.encoded + 7) / 8);
	printf("UPER encoded MessageFrame ec.encoded[%ld], len:%d\n", ec.encoded, len);
	
	return len;
}


//消息帧解码接口, 调用者需释放messageframe内存
MessageFrame_t *v2x_decode_msgFrame(const void *buffer, size_t size)
{
	asn_dec_rval_t rc;
	MessageFrame_t *messageFrame = NULL;

	if (buffer == NULL) {
		return NULL;
	}
	/** decode */
	rc = uper_decode(NULL, &asn_DEF_MessageFrame, (void **)&messageFrame, (const void *)buffer, (size_t)size, 0, 0);
	if (rc.code != RC_OK) {
		printf("asn uper decode fail.\n");
	  	ASN_STRUCT_FREE(asn_DEF_MessageFrame, messageFrame);
		return NULL;
	}
	
#if 0
	char err_buf[256] = {0};
	int err_len = 0;
	/** check the messageframe_decode_struct is valid */
	if (asn_check_constraints(&asn_DEF_MessageFrame, (void *)messageFrame, err_buf, (size_t*)&err_len)) {
		printf("decode MessageFrame noncompliance, ERR_BUFF:%s, ERR_SIZE:%d.\n", err_buf, err_len);
	  	ASN_STRUCT_FREE(asn_DEF_MessageFrame, messageFrame);
		return NULL;
	}
#endif

	return messageFrame;
}

//消息帧解码释放messageFrame内存
int v2x_decode_free(MessageFrame_t *messageframe)
{
	ASN_STRUCT_FREE(asn_DEF_MessageFrame, messageframe);
	messageframe = NULL;
	return 0;
}


//填充数据之后，用于释放内存（全部检查释放）
void v2x_rsi_msg_free(RoadSideInformation_t *rsi)
{
	int i = 0, j = 0, k = 0;

	if (rsi->moy) {
		SAFE_FREE(rsi->moy);
	}
	if (rsi->id.buf) {
		SAFE_FREE(rsi->id.buf);
	}
	if (rsi->refPos.elevation) {
		SAFE_FREE(rsi->refPos.elevation);
	}
	
	//printf("000000000000\n");
	if (rsi->rtes) {
		if (rsi->rtes->list.array) {
			for (i = 0; i < rsi->rtes->list.count; i++) {
				RTEData_t *rte = rsi->rtes->list.array[i];
				if (rte->eventPos) {
					if (rte->eventPos->offsetV) {
						SAFE_FREE(rte->eventPos->offsetV);
					}
					SAFE_FREE(rte->eventPos);
				}
				if (rte->eventRadius) {
					SAFE_FREE(rte->eventRadius);
				}
				if (rte->description) {
					if (rte->description->choice.textString.buf) {
						SAFE_FREE(rte->description->choice.textString.buf);
					}
					if (rte->description->choice.textGB2312.buf) {
						SAFE_FREE(rte->description->choice.textGB2312.buf);
					}
					SAFE_FREE(rte->description);
				}
				if (rte->timeDetails) {
					if (rte->timeDetails->startTime) {
						SAFE_FREE(rte->timeDetails->startTime);
					}
					if (rte->timeDetails->endTime) {
						SAFE_FREE(rte->timeDetails->endTime);
					}
					if (rte->timeDetails->endTimeConfidence) {
						SAFE_FREE(rte->timeDetails->endTimeConfidence);
					}
					SAFE_FREE(rte->timeDetails);
				}
				if (rte->priority) {
					if (rte->priority->buf) {
						SAFE_FREE(rte->priority->buf);
					}
					SAFE_FREE(rte->priority);
				}
				//
				if (rte->referencePaths) {
					if (rte->referencePaths->list.array) {
						for (j = 0; j < rte->referencePaths->list.count; j++) {
							ReferencePath_t *refp = rte->referencePaths->list.array[j];
							if (refp->activePath.list.array) {
								for (k = 0; k < refp->activePath.list.count; k++) {
									PositionOffsetLLV_t *pos_llv = refp->activePath.list.array[k];
									if (pos_llv->offsetV) {
										SAFE_FREE(pos_llv->offsetV);
									}
									SAFE_FREE(pos_llv);
								}
								SAFE_FREE(refp->activePath.list.array);
							}
							if (refp) {
								SAFE_FREE(refp);
							}
						}
						SAFE_FREE(rte->referencePaths->list.array);
					}
					SAFE_FREE(rte->referencePaths);
				}
				//
				if (rte->referenceLinks) {
					if (rte->referenceLinks->list.array) {
						for (j = 0; j < rte->referenceLinks->list.count; j++) {
							ReferenceLink_t *refl = rte->referenceLinks->list.array[j];
							if (refl->upstreamNodeId.region) {
								SAFE_FREE(refl->upstreamNodeId.region);
							}
							if (refl->downstreamNodeId.region) {
								SAFE_FREE(refl->downstreamNodeId.region);
							}
							if (refl->referenceLanes) {
								if (refl->referenceLanes->buf) {
									SAFE_FREE(refl->referenceLanes->buf);
								}
								SAFE_FREE(refl->referenceLanes);
							}
							if (refl) {
								SAFE_FREE(refl);
							}
						}
						SAFE_FREE(rte->referenceLinks->list.array);
					}
					SAFE_FREE(rte->referenceLinks);
				}
				
				//
				if (rte->eventConfidence) {
					SAFE_FREE(rte->eventConfidence);
				}
				
				if (rte) {
					SAFE_FREE(rte);
				}
			}
			SAFE_FREE(rsi->rtes->list.array);
		}
	
		SAFE_FREE(rsi->rtes);
	}

//
	if (rsi->rtss) {
		if (rsi->rtss->list.array) {
			for (i = 0; i < rsi->rtss->list.count; i++) {
				RTSData_t *rts = rsi->rtss->list.array[i];
				if (rts->signPos) {
					if (rts->signPos->offsetV) {
						SAFE_FREE(rts->signPos->offsetV);
					}
					SAFE_FREE(rts->signPos);
				}
				if (rts->description) {
					if (rts->description->choice.textString.buf) {
						SAFE_FREE(rts->description->choice.textString.buf);
					}
					if (rts->description->choice.textGB2312.buf) {
						SAFE_FREE(rts->description->choice.textGB2312.buf);
					}
					SAFE_FREE(rts->description);
				}
				if (rts->timeDetails) {
					if (rts->timeDetails->startTime) {
						SAFE_FREE(rts->timeDetails->startTime);
					}
					if (rts->timeDetails->endTime) {
						SAFE_FREE(rts->timeDetails->endTime);
					}
					if (rts->timeDetails->endTimeConfidence) {
						SAFE_FREE(rts->timeDetails->endTimeConfidence);
					}
					SAFE_FREE(rts->timeDetails);
				}
				if (rts->priority) {
					if (rts->priority->buf) {
						SAFE_FREE(rts->priority->buf);
					}
					SAFE_FREE(rts->priority);
				}
				//
				if (rts->referencePaths) {
					if (rts->referencePaths->list.array) {
						for (j = 0; j < rts->referencePaths->list.count; j++) {
							ReferencePath_t *refp = rts->referencePaths->list.array[j];
							if (refp->activePath.list.array) {
								for (k = 0; k < refp->activePath.list.count; k++) {
									PositionOffsetLLV_t *pos_llv = refp->activePath.list.array[k];
									if (pos_llv->offsetV) {
										SAFE_FREE(pos_llv->offsetV);
									}
									SAFE_FREE(pos_llv);
								}
								SAFE_FREE(refp->activePath.list.array);
							}
							if (refp) {
								SAFE_FREE(refp);
							}
						}
						SAFE_FREE(rts->referencePaths->list.array);
					}
					SAFE_FREE(rts->referencePaths);
				}
				//
				if (rts->referenceLinks) {
					if (rts->referenceLinks->list.array) {
						for (j = 0; j < rts->referenceLinks->list.count; j++) {
							ReferenceLink_t *refl = rts->referenceLinks->list.array[j];
							if (refl->upstreamNodeId.region) {
								SAFE_FREE(refl->upstreamNodeId.region);
							}
							if (refl->downstreamNodeId.region) {
								SAFE_FREE(refl->downstreamNodeId.region);
							}
							if (refl->referenceLanes) {
								if (refl->referenceLanes->buf) {
									SAFE_FREE(refl->referenceLanes->buf);
								}
								SAFE_FREE(refl->referenceLanes);
							}
							if (refl) {
								SAFE_FREE(refl);
							}
						}
						SAFE_FREE(rts->referenceLinks->list.array);
					}
					SAFE_FREE(rts->referenceLinks);
				}

				//
				if (rts) {
					SAFE_FREE(rts);
				}
			}
			SAFE_FREE(rsi->rtss->list.array);
		}
		SAFE_FREE(rsi->rtss);
	}
	
}

void v2x_rsm_msg_free(RoadsideSafetyMessage_t *rsm)
{
	if (rsm->id.buf) {
		SAFE_FREE(rsm->id.buf);
	}
	if (rsm->refPos.elevation) {
		SAFE_FREE(rsm->refPos.elevation);
	}
	
	int i = 0;
	if (rsm->participants.list.array) {
		for (i = 0; i < rsm->participants.list.count; i++) {
			ParticipantData_t *ptc = rsm->participants.list.array[i];
			if (ptc->id) {
				SAFE_FREE(ptc->id->buf);
				SAFE_FREE(ptc->id);
			}
			if (ptc->pos.offsetV) {
				SAFE_FREE(ptc->pos.offsetV);
			}
			if (ptc->posConfidence.elevation) {
				SAFE_FREE(ptc->posConfidence.elevation);
			}
			if (ptc->transmission) {
				SAFE_FREE(ptc->transmission);
			}
			if (ptc->angle) {
				SAFE_FREE(ptc->angle);
			}
			if (ptc->motionCfd) {
				if (ptc->motionCfd->speedCfd) {
					SAFE_FREE(ptc->motionCfd->speedCfd);
				}
				if (ptc->motionCfd->headingCfd) {
					SAFE_FREE(ptc->motionCfd->headingCfd);
				}
				if (ptc->motionCfd->steerCfd) {
					SAFE_FREE(ptc->motionCfd->steerCfd);
				}
				SAFE_FREE(ptc->motionCfd);
			}
			if (ptc->accelSet) {
				SAFE_FREE(ptc->accelSet);
			}
			if (ptc->size.height) {
				SAFE_FREE(ptc->size.height);
			}
			if (ptc->vehicleClass) {
				if (ptc->vehicleClass->fuelType) {
					SAFE_FREE(ptc->vehicleClass->fuelType);
				}
				SAFE_FREE(ptc->vehicleClass);
			}
			
			SAFE_FREE(ptc);
		}
		SAFE_FREE(rsm->participants.list.array);
	}
}


void v2x_bsm_msg_free(BasicSafetyMessage_t *bsm)
{
	int i = 0;

	if (bsm->id.buf) {
		SAFE_FREE(bsm->id.buf);
	}
	if (bsm->timeConfidence) {
		SAFE_FREE(bsm->timeConfidence);
	}
	if (bsm->pos.elevation) {
		SAFE_FREE(bsm->pos.elevation);
	}
	if (bsm->posAccuracy) {
		SAFE_FREE(bsm->posAccuracy);
	}
	if (bsm->posConfidence) {
		if (bsm->posConfidence->elevation) {
			SAFE_FREE(bsm->posConfidence->elevation);
		}
		SAFE_FREE(bsm->posConfidence);
	}
	if (bsm->angle) {
		SAFE_FREE(bsm->angle);
	}
	
	if (bsm->motionCfd) {
		if (bsm->motionCfd->speedCfd) {
			SAFE_FREE(bsm->motionCfd->speedCfd);
		}
		if (bsm->motionCfd->headingCfd) {
			SAFE_FREE(bsm->motionCfd->headingCfd);
		}
		if (bsm->motionCfd->steerCfd) {
			SAFE_FREE(bsm->motionCfd->steerCfd);
		}
		SAFE_FREE(bsm->motionCfd);
	}
	
	if (bsm->brakes.brakePadel) {
		SAFE_FREE(bsm->brakes.brakePadel);
	}
	if (bsm->brakes.wheelBrakes) {
		SAFE_FREE(bsm->brakes.wheelBrakes);
	}
	if (bsm->brakes.traction) {
		SAFE_FREE(bsm->brakes.traction);
	}
	if (bsm->brakes.abs) {
		SAFE_FREE(bsm->brakes.abs);
	}
	if (bsm->brakes.scs) {
		SAFE_FREE(bsm->brakes.scs);
	}
	if (bsm->brakes.brakeBoost) {
		SAFE_FREE(bsm->brakes.brakeBoost);
	}
	if (bsm->brakes.auxBrakes) {
		SAFE_FREE(bsm->brakes.auxBrakes);
	}

	if (bsm->size.height) {
		SAFE_FREE(bsm->size.height);
	}
	
	if (bsm->vehicleClass.fuelType) {
		SAFE_FREE(bsm->vehicleClass.fuelType);
	}

	if (bsm->safetyExt) {
		if (bsm->safetyExt->events) {
			if (bsm->safetyExt->events->buf) {
				SAFE_FREE(bsm->safetyExt->events->buf);
			}
			SAFE_FREE(bsm->safetyExt->events);
		}
		//
		if (bsm->safetyExt->pathHistory) {
			if (bsm->safetyExt->pathHistory->initialPosition) {
				if (bsm->safetyExt->pathHistory->initialPosition->utcTime) {
					//printf("00000000000000000\n");
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->utcTime->year);
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->utcTime->month);
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->utcTime->day);
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->utcTime->hour);
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->utcTime->minute);
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->utcTime->second);
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->utcTime->offset);
					
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->utcTime);
				}
				if (bsm->safetyExt->pathHistory->initialPosition->pos.elevation) {
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->pos.elevation);
				}
				if (bsm->safetyExt->pathHistory->initialPosition->heading) {
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->heading);
				}
				if (bsm->safetyExt->pathHistory->initialPosition->transmission) {
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->transmission);
				}
				if (bsm->safetyExt->pathHistory->initialPosition->speed) {
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->speed);
				}
				if (bsm->safetyExt->pathHistory->initialPosition->posAccuracy) {
					if (bsm->safetyExt->pathHistory->initialPosition->posAccuracy->elevation) {
						SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->posAccuracy->elevation);
					}
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->posAccuracy);
				}
				if (bsm->safetyExt->pathHistory->initialPosition->timeConfidence) {
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->timeConfidence);
				}
				if (bsm->safetyExt->pathHistory->initialPosition->motionCfd) {
					if (bsm->safetyExt->pathHistory->initialPosition->motionCfd->speedCfd) {
						SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->motionCfd->speedCfd);
					}
					if (bsm->safetyExt->pathHistory->initialPosition->motionCfd->headingCfd) {
						SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->motionCfd->headingCfd);
					}
					if (bsm->safetyExt->pathHistory->initialPosition->motionCfd->steerCfd) {
						SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->motionCfd->steerCfd);
					}
					SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition->motionCfd);
				}
				
				SAFE_FREE(bsm->safetyExt->pathHistory->initialPosition);
				
			}
			
			if (bsm->safetyExt->pathHistory->currGNSSstatus) {
				if (bsm->safetyExt->pathHistory->currGNSSstatus->buf) {
					SAFE_FREE(bsm->safetyExt->pathHistory->currGNSSstatus->buf);
				}
				SAFE_FREE(bsm->safetyExt->pathHistory->currGNSSstatus);
			}
			//
			if (bsm->safetyExt->pathHistory->crumbData.list.array) {
				for (i = 0; i < bsm->safetyExt->pathHistory->crumbData.list.count; i++) {
					PathHistoryPoint_t *point = bsm->safetyExt->pathHistory->crumbData.list.array[i];
					if (point->llvOffset.offsetV) {
						SAFE_FREE(point->llvOffset.offsetV);
					}
					if (point->speed) {
						SAFE_FREE(point->speed);
					}
					if (point->posAccuracy) {
						if (point->posAccuracy->elevation) {
							SAFE_FREE(point->posAccuracy->elevation);
						}
						SAFE_FREE(point->posAccuracy);
					}
					if (point->heading) {
						SAFE_FREE(point->heading);
					}
					//
					if (point) {
						SAFE_FREE(point);
					}
				}
				SAFE_FREE(bsm->safetyExt->pathHistory->crumbData.list.array);
			}
			//
			
			SAFE_FREE(bsm->safetyExt->pathHistory);
		}
		
		//
		if (bsm->safetyExt->pathPrediction) {
			SAFE_FREE(bsm->safetyExt->pathPrediction);
		}
		if (bsm->safetyExt->lights) {
			if (bsm->safetyExt->lights->buf) {
				SAFE_FREE(bsm->safetyExt->lights->buf);
			}
			SAFE_FREE(bsm->safetyExt->lights);
		}
		
		//
		SAFE_FREE(bsm->safetyExt);
	}
	
	if (bsm->emergencyExt) {
		if (bsm->emergencyExt->responseType) {
			SAFE_FREE(bsm->emergencyExt->responseType);
		}
		if (bsm->emergencyExt->sirenUse) {
			SAFE_FREE(bsm->emergencyExt->sirenUse);
		}
		if (bsm->emergencyExt->lightsUse) {
			SAFE_FREE(bsm->emergencyExt->lightsUse);
		}
		SAFE_FREE(bsm->emergencyExt);
	}
	
}

void v2x_spat_msg_free(SPAT_t *spat)
{
	int i = 0, j = 0, k = 0;

	if (spat->moy) {
		SAFE_FREE(spat->moy);
	}
	if (spat->timeStamp) {
		SAFE_FREE(spat->timeStamp);
	}
	if (spat->name) {
		if (spat->name->buf) {
			SAFE_FREE(spat->name->buf);
		}
		SAFE_FREE(spat->name);
	}
	if (spat->intersections.list.array) {
		for (i = 0; i < spat->intersections.list.count; i++) {
			IntersectionState_t *interSecState = spat->intersections.list.array[i];
			if (interSecState->intersectionId.region) {
				SAFE_FREE(interSecState->intersectionId.region);
			}
			if (interSecState->status.buf) {
				SAFE_FREE(interSecState->status.buf);
			}
			if (interSecState->moy) {
				SAFE_FREE(interSecState->moy);
			}
			if (interSecState->timeStamp) {
				SAFE_FREE(interSecState->timeStamp);
			}
			if (interSecState->timeConfidence) {
				SAFE_FREE(interSecState->timeConfidence);
			}
			
			if (interSecState->phases.list.array) {
				for (j = 0; j < interSecState->phases.list.count; j++) {
					Phase_t *phase = interSecState->phases.list.array[j];
					
					if (phase->phaseStates.list.array) {
						for (k = 0; k < phase->phaseStates.list.count; k++) {
							PhaseState_t *phaseState = phase->phaseStates.list.array[k];
							
							if (phaseState->timing) {
								//
								if (phaseState->timing->choice.counting.minEndTime) {
									SAFE_FREE(phaseState->timing->choice.counting.minEndTime);
								}
								if (phaseState->timing->choice.counting.maxEndTime) {
									SAFE_FREE(phaseState->timing->choice.counting.maxEndTime);
								}
								if (phaseState->timing->choice.counting.timeConfidence) {
									SAFE_FREE(phaseState->timing->choice.counting.timeConfidence);
								}
								if (phaseState->timing->choice.counting.nextStartTime) {
									SAFE_FREE(phaseState->timing->choice.counting.nextStartTime);
								}
								if (phaseState->timing->choice.counting.nextDuration) {
									SAFE_FREE(phaseState->timing->choice.counting.nextDuration);
								}
								//
								if (phaseState->timing->choice.utcTiming.minEndUTCTime) {
									SAFE_FREE(phaseState->timing->choice.utcTiming.minEndUTCTime);
								}
								if (phaseState->timing->choice.utcTiming.maxEndUTCTime) {
									SAFE_FREE(phaseState->timing->choice.utcTiming.maxEndUTCTime);
								}
								if (phaseState->timing->choice.utcTiming.timeConfidence) {
									SAFE_FREE(phaseState->timing->choice.utcTiming.timeConfidence);
								}
								if (phaseState->timing->choice.utcTiming.nextStartUTCTime) {
									SAFE_FREE(phaseState->timing->choice.utcTiming.nextStartUTCTime);
								}
								if (phaseState->timing->choice.utcTiming.nextEndUTCTime) {
									SAFE_FREE(phaseState->timing->choice.utcTiming.nextEndUTCTime);
								}
								
								//
								SAFE_FREE(phaseState->timing);
							}
							
							//
							if (phaseState) {
								SAFE_FREE(phaseState);
							}
						}
						SAFE_FREE(phase->phaseStates.list.array);
					}
					
					//
					if (phase) {
						SAFE_FREE(phase);
					}
				}
				SAFE_FREE(interSecState->phases.list.array);
			}
			
			//
			if (interSecState) {
				SAFE_FREE(interSecState);
			}
		}
		SAFE_FREE(spat->intersections.list.array);
	}
}


void v2x_map_msg_free(MapData_t *map)
{
	int i = 0, j = 0, k = 0, l = 0;

	if (map->timeStamp) {
		SAFE_FREE(map->timeStamp);
	}
	if (map->nodes.list.array) {
		for (i = 0; i < map->nodes.list.count; i++) {
			Node_t *node = map->nodes.list.array[i];
			if (node->name) {
				if (node->name->buf) {
					SAFE_FREE(node->name->buf);
				}
				SAFE_FREE(node->name);
			}
			if (node->id.region) {
				SAFE_FREE(node->id.region);
			}
			if (node->refPos.elevation) {
				SAFE_FREE(node->refPos.elevation);
			}
			if (node->inLinks) {
				if (node->inLinks->list.array) {
					for (j = 0; j < node->inLinks->list.count; j++) {
						Link_t *link = node->inLinks->list.array[j];
						if (link->name) {
							if (link->name->buf) {
								SAFE_FREE(link->name->buf);
							}
							SAFE_FREE(link->name);
						}
						if (link->upstreamNodeId.region) {
							SAFE_FREE(link->upstreamNodeId.region);
						}
						//
						if (link->speedLimits) {
							if (link->speedLimits->list.array) {
								for (k = 0; k < link->speedLimits->list.count; k++) {
									RegulatorySpeedLimit_t *speedLimit = link->speedLimits->list.array[k];
									if (speedLimit) {
										SAFE_FREE(speedLimit);
									}
								}
								SAFE_FREE(link->speedLimits->list.array);
							}
							SAFE_FREE(link->speedLimits);
						}
						//
						if (link->points) {
							if (link->points->list.array) {
								for (k = 0; k < link->points->list.count; k++) {
									RoadPoint_t *point = link->points->list.array[k];
									if (point->posOffset.offsetV) {
										SAFE_FREE(point->posOffset.offsetV);
									}
									if (point) {
										SAFE_FREE(point);
									}
								}
								SAFE_FREE(link->points->list.array);
							}
							SAFE_FREE(link->points);
						}
						//
						if (link->movements) {
							if (link->movements->list.array) {
								for (k = 0; k < link->movements->list.count; k++) {
									Movement_t *movement = link->movements->list.array[k];
									if (movement->remoteIntersection.region) {
										SAFE_FREE(movement->remoteIntersection.region);
									}
									if (movement->phaseId) {
										SAFE_FREE(movement->phaseId);
									}
									if (movement) {
										SAFE_FREE(movement);
									}
								}
								SAFE_FREE(link->movements->list.array);
							}
							SAFE_FREE(link->movements);
						}
						//
						if (link->lanes.list.array) {
							for (k = 0; k < link->lanes.list.count; k++) {
								Lane_t *lane = link->lanes.list.array[k];
								if (lane->laneWidth) {
									SAFE_FREE(lane->laneWidth);
								}
								//
								if (lane->laneAttributes) {
									if (lane->laneAttributes->shareWith) {
										if (lane->laneAttributes->shareWith->buf) {
											SAFE_FREE(lane->laneAttributes->shareWith->buf);
										}
										SAFE_FREE(lane->laneAttributes->shareWith);
									}
									if (lane->laneAttributes->laneType.choice.vehicle.buf) {
										SAFE_FREE(lane->laneAttributes->laneType.choice.vehicle.buf);
									}
									if (lane->laneAttributes->laneType.choice.crosswalk.buf) {
										SAFE_FREE(lane->laneAttributes->laneType.choice.crosswalk.buf);
									}
									if (lane->laneAttributes->laneType.choice.bikeLane.buf) {
										SAFE_FREE(lane->laneAttributes->laneType.choice.bikeLane.buf);
									}
									if (lane->laneAttributes->laneType.choice.sidewalk.buf) {
										SAFE_FREE(lane->laneAttributes->laneType.choice.sidewalk.buf);
									}
									if (lane->laneAttributes->laneType.choice.median.buf) {
										SAFE_FREE(lane->laneAttributes->laneType.choice.median.buf);
									}
									if (lane->laneAttributes->laneType.choice.striping.buf) {
										SAFE_FREE(lane->laneAttributes->laneType.choice.striping.buf);
									}
									if (lane->laneAttributes->laneType.choice.trackedVehicle.buf) {
										SAFE_FREE(lane->laneAttributes->laneType.choice.trackedVehicle.buf);
									}
									if (lane->laneAttributes->laneType.choice.parking.buf) {
										SAFE_FREE(lane->laneAttributes->laneType.choice.parking.buf);
									}

									SAFE_FREE(lane->laneAttributes);
								}
								//
								if (lane->maneuvers) {
									if (lane->maneuvers->buf) {
										SAFE_FREE(lane->maneuvers->buf);
									}
									SAFE_FREE(lane->maneuvers);
								}
								//
								if (lane->connectsTo) {
									if (lane->connectsTo->list.array) {
										for (l = 0; l < lane->connectsTo->list.count; l++) {
											Connection_t *connectsTo = lane->connectsTo->list.array[l];
											if (connectsTo->remoteIntersection.region) {
												SAFE_FREE(connectsTo->remoteIntersection.region);
											}
											if (connectsTo->connectingLane) {
												if (connectsTo->connectingLane->maneuver) {
													if (connectsTo->connectingLane->maneuver->buf) {
														SAFE_FREE(connectsTo->connectingLane->maneuver->buf);
													}
													SAFE_FREE(connectsTo->connectingLane->maneuver);
												}
												SAFE_FREE(connectsTo->connectingLane);
											}
											if (connectsTo->phaseId) {
												SAFE_FREE(connectsTo->phaseId);
											}
											
											//
											if (connectsTo) {
												SAFE_FREE(connectsTo);
											}
										}
										SAFE_FREE(lane->connectsTo->list.array);
									}
									SAFE_FREE(lane->connectsTo);
								}
								//
								if (lane->speedLimits) {
									if (lane->speedLimits->list.array) {
										for (l = 0; l < lane->speedLimits->list.count; l++) {
											RegulatorySpeedLimit_t *speedLimit = lane->speedLimits->list.array[l];
											if (speedLimit) {
												SAFE_FREE(speedLimit);
											}
										}
										SAFE_FREE(lane->speedLimits->list.array);
									}
									SAFE_FREE(lane->speedLimits);
								}
								//
								if (lane->points) {
									if (lane->points->list.array) {
										for (l = 0; l < lane->points->list.count; l++) {
											RoadPoint_t *point = lane->points->list.array[l];
											if (point->posOffset.offsetV) {
												SAFE_FREE(point->posOffset.offsetV);
											}
											if (point) {
												SAFE_FREE(point);
											}
										}
										SAFE_FREE(lane->points->list.array);
									}
									SAFE_FREE(lane->points);
								}

								//
								if (lane) {
									SAFE_FREE(lane);
								}
							}
							SAFE_FREE(link->lanes.list.array);
						}
						
						//
						if (link) {
							SAFE_FREE(link);
						}
					}
					SAFE_FREE(node->inLinks->list.array);
				}
				
				SAFE_FREE(node->inLinks);
			}

			if (node) {
				SAFE_FREE(node);
			}
		}
		SAFE_FREE(map->nodes.list.array);
	}
}
