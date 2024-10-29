/********************************
func：v2x通信，asn编解码接口
author:zyh
data:2023.8.14
*********************************/
#ifndef _V2X_API_H_
#define _V2X_API_H_
	
#ifdef __cplusplus
extern "C" {
#endif
#include "MessageFrame.h"


#define SAFE_FREE(x) do {\
    if (x) {             \
        free(x);         \
        x = NULL;        \
    }                    \
} while(0)


//消息帧编码接口
int v2x_encode_msgFrame(MessageFrame_t *messageFrame, void *buffer, size_t buffer_size);

//消息帧解码接口, 调用者需释放messageframe内存
MessageFrame_t *v2x_decode_msgFrame(const void *buffer, size_t size);

//消息帧解码释放messageFrame内存
int v2x_decode_free(MessageFrame_t *messageframe);


//填充数据之后，用于释放内存（防止泄露，结构体参数逐个检查释放）
void v2x_rsi_msg_free(RoadSideInformation_t *rsi);
void v2x_rsm_msg_free(RoadsideSafetyMessage_t *rsm);
void v2x_bsm_msg_free(BasicSafetyMessage_t *bsm);
void v2x_spat_msg_free(SPAT_t *spat);
void v2x_map_msg_free(MapData_t *map);


#ifdef __cplusplus
}
#endif

#endif
