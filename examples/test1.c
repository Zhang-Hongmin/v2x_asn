/********************************
fun：v2x通信，asn编解码各类消息集测试
author:zyh
data:2023.8.14
*********************************/
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include <unistd.h> 
#include "v2x_api.h"


//路侧消息RSI（交通事件和交通标志）
void rsi_msg_fill(MessageFrame_t *msgFrame)
{
	msgFrame->present = MessageFrame_PR_rsiFrame;
	RoadSideInformation_t *rsi = &msgFrame->choice.rsiFrame;

	int i = 0, j = 0, k =0;
	
	//消息编号，发送方为自己发送的同类消息依次进行编号,数值为 0~127,循环使用，该
    //数据字段用于接收方对来自同一发送方的同一类消息，进行连续收包的监控和丢包的统计
	//int msgCnt = 0;
	static int msgCnt = 0;
    rsi->msgCnt = msgCnt++;//消息编号
	if (msgCnt > 127) {
		msgCnt = 0;
	}
	
	rsi->moy = NULL;//时间信息，表示当前年份，已经过去的总分钟数（UTC 时间）。其分辨率为 1 分钟
	
	//RSU ID编号
	rsi->id.buf = (uint8_t *)calloc(8, sizeof(uint8_t));//只能是8个字节，否则asn编码不通过
	rsi->id.size = 8;
	snprintf((char *)rsi->id.buf, rsi->id.size, "%s", "001");
	
	//参考位置点坐标（消息中所有的位置偏移量，均基于该参考坐标计算。真实位置坐标等于偏移量加上参考坐标）
	rsi->refPos.Long = 432154234;//经度， (-1799999999..1800000001)，分辨率 1e-7°
	rsi->refPos.lat = 231421543;//纬度， (-900000000..900000001) ，分辨率 1e-7°
	rsi->refPos.elevation = NULL;//海拔高程。分辨率为 0.1 米。 (-4096..61439)，数值-4096 表示无效数值。

	/*道路交通事件集合, 至少包含 1 个道路交通事件信息，最多包含 8 个*/
	rsi->rtes = (RTEList_t *)calloc(1, sizeof(RTEList_t));
	int rtes_num = 2;
	rsi->rtes->list.array = (RTEData_t **)calloc(rtes_num, sizeof(RTEData_t *));
	rsi->rtes->list.count = rtes_num;
	rsi->rtes->list.size = rtes_num;
	rsi->rtes->list.free = NULL;
	/**错误用法，事件数组列表是array[n][1]样式的，而不是array[1][n]，在此留作记录
	RTEData_t *rte = (RTEData_t *)calloc(2, sizeof(RTEData_t)); 
	rsi->rtes->list.array[0] = rte;//变成了array[0][0] = rte[0],  array[0][1] = rte[1]
	rsi->rtes->list.array[1] = ++rte;//变成了array[1][0] =  rte[1]
	或
	rsi->rtes->list.array[0] = &rte[0];
	rsi->rtes->list.array[1] = &rte[1];
	**/
	for (i = 0; i < rtes_num; i++) {
		//单个事件信息
		RTEData_t *rte = (RTEData_t *)calloc(1, sizeof(RTEData_t));
		rsi->rtes->list.array[i] = rte;
		rte->rteId = 0;//事件在本地的id编号，取值范围 (0..255)
		rte->eventType = 707;// 道路交通事件的类型编码。707：道路拥堵
		rte->eventSource = 5;//事件信息来源，0未知，1交警，2政府，3气象部门，4互联网，5本地检测
		//事件位置(定义三维的相对位置（相对经纬度和相对高程）。约定偏差值等于真实值减去参考值)
		rte->eventPos = (PositionOffsetLLV_t *)calloc(1, sizeof(PositionOffsetLLV_t));
		rte->eventPos->offsetLL.present = PositionOffsetLL_PR_position_LatLon;//64 比特经纬度位置。实际上，该尺度采用了真实经纬度数据进行描述，非相对位置
		rte->eventPos->offsetLL.choice.position_LatLon.lat = 231421543;
		rte->eventPos->offsetLL.choice.position_LatLon.lon = 432154234;
		rte->eventPos->offsetV = NULL;
		// 事件影响半径(表示绝对值半径大小。分辨率为 10cm)
		rte->eventRadius = (Radius_t *)calloc(1, sizeof(Radius_t));
		rte->eventRadius[0] = 100;
		// 事件补充描述
		rte->description = NULL;//SIZE(1..512)
		
		//事件详细时间（起止时间）
		rte->timeDetails = NULL;
		/*事件优先级，表示 RSI 消息中不同类型交通事件或交通标志的优先级。数值长度占 8 位，
		其中低五位为 0，为无效位，高三位为有效数据位。
		数值有效范围是 B00000000 到 B11100000，分别表示 8 档由低到高的优先级
		*/
		rte->priority = (RSIPriority_t *)calloc(1, sizeof(RSIPriority_t));
		rte->priority->buf = (uint8_t *)calloc(1, sizeof(uint8_t));//只能是1个字节，否则asn编译不通过
		rte->priority->size = 1;
		//rte->priority->buf[0] = (1 << 5);
		//rte->priority->buf[0] = (2 << 5);
		rte->priority->buf[0] = (7 << 5);
		
		//影响的关联路径坐标点点集合referencePaths（从上游到下游列出点，沿着车辆行驶方向，一条路径至少包括1个点，只有1个点的路径表示圆形警报区域）
		rte->referencePaths = (ReferencePathList_t *)calloc(1, sizeof(ReferencePathList_t));
		int referencePaths_num = 2;
		rte->referencePaths->list.array = (ReferencePath_t **)calloc(referencePaths_num, sizeof(ReferencePath_t *));
		//rte->referencePaths->list.array[0] = refp;
		rte->referencePaths->list.count = referencePaths_num;
		rte->referencePaths->list.size = referencePaths_num;
		rte->referencePaths->list.free = NULL;
		for (j = 0; j < referencePaths_num; j++) {
			//单条路径信息
			ReferencePath_t *refp = (ReferencePath_t *)calloc(1, sizeof(ReferencePath_t));
			rte->referencePaths->list.array[j] = refp;
		
			//路劲的半径，单位分米。用半径表示影响区域边界离中心线的垂直距离，反映该区域的宽度以覆盖实际路段
			refp->pathRadius = 10;
			
			//某条路径的中心线点坐标集合,用有序位置点列的方式，定义一个有向的作用范围,配合半径信息，用来表示一个的车辆行进轨迹区段
			int activePath_num = 2;
			refp->activePath.list.array = (PositionOffsetLLV_t **)calloc(activePath_num, sizeof(PositionOffsetLLV_t *));
			refp->activePath.list.count = activePath_num;
			refp->activePath.list.size = activePath_num;
			refp->activePath.list.free = NULL;
			for (k = 0; k < activePath_num; k++) {
				PositionOffsetLLV_t *pos_llv = (PositionOffsetLLV_t *)calloc(1, sizeof(PositionOffsetLLV_t));//关联路径的坐标点
				refp->activePath.list.array[k] = pos_llv;
			
				pos_llv->offsetLL.present = PositionOffsetLL_PR_position_LatLon;//经纬度偏差
				pos_llv->offsetLL.choice.position_LatLon.lat = 231421543;
				pos_llv->offsetLL.choice.position_LatLon.lon = 231421543;
				pos_llv->offsetV = NULL;//垂直方向位置偏差。
			}
		}
		
		//影响的关联路段集合
		rte->referenceLinks = (ReferenceLinkList_t *)calloc(1, sizeof(ReferenceLinkList_t));
		int referenceLinks_num = 2;
		rte->referenceLinks->list.array = (ReferenceLink_t **)calloc(referenceLinks_num, sizeof(ReferenceLink_t *));
		rte->referenceLinks->list.count = referenceLinks_num;
		rte->referenceLinks->list.size = referenceLinks_num;
		rte->referenceLinks->list.free = NULL;
		for (j = 0; j < referenceLinks_num; j++) {
			ReferenceLink_t *refl = (ReferenceLink_t *)calloc(1, sizeof(ReferenceLink_t));
			rte->referenceLinks->list.array[j] = refl;
		
			/*路段上游节点id*/
			//全局唯一的地区ID，一般由国家或区域性的管理部门统一管理和分配地图中各个划分区域的 ID 号，
			//取值范围 (0..65535)，数值 0 仅用于测试
			refl->upstreamNodeId.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t));
			refl->upstreamNodeId.region[0] = 666;//全局唯一的地区ID
			//路网最基本的构成即节点和节点之间连接的路段。
			//节点可以是路口，也可以是一条路的端点。一个节点的 ID 在同一个区域内是唯一的。
			//取值范围 (0..65535)，数值 0~255 预留为测试使用
			refl->upstreamNodeId.id = 2;//地区内部唯一的局部节点ID

			/*路段下游节点id*/
			refl->downstreamNodeId.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t));
			refl->downstreamNodeId.region[0] = 666;//全局唯一的地区ID
			refl->downstreamNodeId.id = 3;//地区内部唯一的局部节点ID

			/*路段关联车道*/
			//BIT_STRING_t`类型通常用于定义和处理位串，
			//二进制比特位表示车道号，比特位的取值为1，表示该车道为有效的关联车道，最多支持 15 条车道
			//reserved(bit 0)预留, lane1(bit 1),lane2(bit 2), lane3(bit 3), ... lane15(bit 15)
			//refl->referenceLanes = NULL;
			refl->referenceLanes = (ReferenceLanes_t *)calloc(1, sizeof(ReferenceLanes_t));
			refl->referenceLanes->buf = (uint8_t *)calloc(2, sizeof(uint8_t));//只能是2个字节，表示16个bit位，否则asn编译不通过
			unsigned short referenceLanes = 0B0000000000000110;
			//refl->referenceLanes->buf[0] = referenceLanes;
			//refl->referenceLanes->buf[1] = (referenceLanes >> 8);
			refl->referenceLanes->buf[0] = (referenceLanes & 0xff);
			refl->referenceLanes->buf[1] = ((referenceLanes >> 8) & 0xff);
			refl->referenceLanes->size = 2;
			refl->referenceLanes->bits_unused = 0;
		}
		
		//事件置信度。分辨率为 0.005 (范围0..200)
		rte->eventConfidence = (Confidence_t *)calloc(1, sizeof(Confidence_t));
		rte->eventConfidence[0] = 0.005*100;
	}
	
	/*道路交通标志集合，至少包含 1 个道路交通标志信息，最多包含 16 个*/
	rsi->rtss = (RTSList_t *)calloc(1, sizeof(RTSList_t));
	int rtss_num = 2;
	rsi->rtss->list.array = (RTSData_t **)calloc(rtss_num, sizeof(RTSData_t *));
	rsi->rtss->list.count = rtss_num;
	rsi->rtss->list.size = rtss_num;
	rsi->rtss->list.free = NULL;
	for (i = 0; i < rtss_num; i++) {
		//单个标志信息
		RTSData_t *rts = (RTSData_t *)calloc(1, sizeof(RTSData_t));
		rsi->rtss->list.array[i] = rts;
		rts->rtsId = 0;//在本地的id编号，取值范围 (0..255)
		rts->signType = 1;//交通标志类型,1为交叉路口，参照国标《GB 5768.2-2009 道路交通标志和标线 第2部分：道路交通标志》（交通标志中文名称索引表序号）

		//位置(定义三维的相对位置（相对经纬度和相对高程）。约定偏差值等于真实值减去参考值)
		rts->signPos = (PositionOffsetLLV_t *)calloc(1, sizeof(PositionOffsetLLV_t));
		rts->signPos->offsetLL.present = PositionOffsetLL_PR_position_LatLon;//64 比特经纬度位置。实际上，该尺度采用了真实经纬度数据进行描述，非相对位置
		rts->signPos->offsetLL.choice.position_LatLon.lat = 231421543;
		rts->signPos->offsetLL.choice.position_LatLon.lon = 432154234;
		rts->signPos->offsetV = NULL;

		//交通标志文本描述信息
		rts->description =  NULL;
		
		//详细时间
		rts->timeDetails =  NULL;
		
		/*优先级,同上描述，高三位有效,左移5位就好，其中根据后续国标要求：
0 预留
1 告示标志
2 旅游区标志
3 指路标志
4 警告标志
5 指示标志
6 禁令标志
7 预留
*/
		rts->priority = (RSIPriority_t *)calloc(1, sizeof(RSIPriority_t));
		rts->priority->buf = (uint8_t *)calloc(1, sizeof(uint8_t));//只能是1个字节，否则asn编译不通过
		rts->priority->size = 1;
		rts->priority->buf[0] = (3 << 5);

		/*影响的相关路径坐标点集合,同上描述*/
		rts->referencePaths = NULL;
		//rts->referencePaths = (ReferencePathList_t *)calloc(1, sizeof(ReferencePathList_t));
		//int rts_referencePaths_num = 3;
		//rts->referencePaths->list.count = rts_referencePaths_num;
		//rts->referencePaths->list.size = rts_referencePaths_num;
		//rts->referencePaths->list.free = NULL;
		
		/*影响的相关路段集合,同上描述*/
		rts->referenceLinks = NULL;
		//rts->referenceLinks = (ReferenceLinkList_t *)calloc(1, sizeof(ReferenceLinkList_t));
		//int rts_referenceLinks_num = 2;
		//rts->referenceLinks->list.array = (ReferenceLink_t **)calloc(rts_referenceLinks_num, sizeof(ReferenceLink_t *));
		//rts->referenceLinks->list.count = rts_referenceLinks_num;
		//rts->referenceLinks->list.size = rts_referenceLinks_num;
		//rts->referenceLinks->list.free = NULL;
	}
}

#if 0
void rsi_msg_free(RoadSideInformation_t *rsi)
{
	int i = 0;
	//int	j = 0, k = 0;

	if (rsi->moy) {
		SAFE_FREE(rsi->moy);
	}
	if (rsi->id.buf) {
		SAFE_FREE(rsi->id.buf);
	}
	if (rsi->refPos.elevation) {
		SAFE_FREE(rsi->refPos.elevation);
	}
	
	if (rsi->rtes) {
		if (rsi->rtes->list.array) {
			for (i = 0; i < rsi->rtes->list.count; i++) {
				RTEData_t *rte = rsi->rtes->list.array[i];
				if (rte->eventPos) {
					if (rte->eventPos->offsetV) {
						SAFE_FREE(rte->eventPos->offsetV);
					}
					SAFE_FREE(rte->eventPos);
					//...
				}
			}
			SAFE_FREE(rsi->rtes->list.array);
		}
		SAFE_FREE(rsi->rtes);
	}
	//...
}
#endif

//路侧安全消息（RSM）(周边交通参与者的实时状态信息)
void rsm_msg_fill(MessageFrame_t *msgFrame)
{
	msgFrame->present = MessageFrame_PR_rsmFrame;
	RoadsideSafetyMessage_t *rsm = &msgFrame->choice.rsmFrame;
	int i = 0;
	
	//int msgCnt = 0;
	static int msgCnt = 0;
    rsm->msgCnt = msgCnt++;//消息集编号，发送方为自己发送的同类消息依次进行编号,数值为 0~127,循环使用
	if (msgCnt > 127) {
		msgCnt = 0;
	}

	//RSU ID编号
	rsm->id.buf = (uint8_t *)calloc(8, sizeof(uint8_t));//只能是8个字节，否则asn编码不通过
	rsm->id.size = 8;
	snprintf((char *)rsm->id.buf, rsm->id.size, "%s", "001");

	//本消息参考位置点坐标
	rsm->refPos.lat = 231421543;
	rsm->refPos.Long = 432154234;
	rsm->refPos.elevation = NULL;

	/*道路交通参与者列表*/
	int participants_num = 2;
	rsm->participants.list.array = (ParticipantData_t **)calloc(participants_num, sizeof(ParticipantData_t *));
	rsm->participants.list.count = participants_num;
	rsm->participants.list.size = participants_num;
	rsm->participants.list.free = NULL;
	for (i = 0; i < participants_num; i++) {
		ParticipantData_t *ptc = (ParticipantData_t *)calloc(1, sizeof(ParticipantData_t));
		rsm->participants.list.array[i] = ptc;
	
		ptc->ptcType = 1;//道路交通参与者类型：0未知、1机动车、2非机动车、3行人、4rsu
		ptc->ptcId = 1;//道路交通参与者编号(0..65535)
		/*信息来源
			0：未知、
			1：RSU 自身信息、
			2：来源于参与者自身的 v2x 广播消息
			3：来源于视频传感器
			4：来源于微波雷达传感器；
			5：来源于地磁线圈传感器；
			6：来源于激光雷达传感器；
			7：2 类或以上感知数据的融合结果。
		*/
		ptc->source = 3;//信息来源
		
		//车辆临时id编号
		ptc->id = (OCTET_STRING_t *)calloc(1, sizeof(OCTET_STRING_t));
		ptc->id->buf = (uint8_t *)calloc(8, sizeof(uint8_t));//只能是8个字节，否则asn编码不通过
		ptc->id->size = 8;
		snprintf((char *)ptc->id->buf, ptc->id->size, "%s", "1");
		
		ptc->secMark = 1;//时间信息，定义 1 分钟内的毫秒级时刻。分辨率为 1 毫秒，有效范围是 0~59999。60000 及以上表示未知或无效数值。
		
		//坐标位置信息，其中64 比特经纬度位置，采用了真实经纬度数据进行描述，非相对位置
		ptc->pos.offsetLL.present = PositionOffsetLL_PR_position_LatLon;//经纬度偏差
		ptc->pos.offsetLL.choice.position_LatLon.lat = 33215840;
		ptc->pos.offsetLL.choice.position_LatLon.lon = 11123640;
		ptc->pos.offsetV = (VerticalOffset_t*)calloc(1, sizeof(VerticalOffset_t));
		ptc->pos.offsetV->present = VerticalOffset_PR_offset2;//垂直方向偏差，比特的数据，定义垂直方向（Z 轴）关于参考位置点的偏差。顺着 Z 轴方向为正,数据分辨率为 10 厘米。
		ptc->pos.offsetV->choice.offset2 = 100;
		/*位置精度
		unavailable (0),
		a500m (1),
		a200m (2)
		a100m (3)
		a50m (4)
		a20m (5)
		a10m (6)
		a5m (7)
		a2m (8)
		a1m (9)
		a50cm (10)
		a20cm (11)
		a10cm (12)
		a5cm (13)
		a2cm (14)
		a1cm (15)
		...
		*/
		ptc->posConfidence.pos = 9;//当前坐标位置精度
		ptc->posConfidence.elevation = NULL;
		
		ptc->transmission = NULL;//挡位信息：0空档、1停止档、2前进档、3倒档
		ptc->speed = 208;//速度，车辆或其他交通参与者的速度大小。分辨率为 0.02m/s，取值范围（(0..8191），数值 8191 表示无效数值。
		ptc->heading = 130;//航向角，为运动方向与正北方向的顺时针夹角。分辨率为 0.0125°，取值范围0..28800
		ptc->angle = NULL;//方向盘转角，向右为正，向左为负。分辨率为 1.5°，取值范围(-126..127)，数值 127 为无效数值
		ptc->motionCfd = NULL;//车辆运行状态精度
		ptc->accelSet = NULL;//车辆4轴加速度
	
		//车辆尺寸
		ptc->size.width = 180;
		ptc->size.length = 490;
		ptc->size.height = NULL;
		
		//车辆类型
		ptc->vehicleClass = NULL;
	}
}


//基本安全消息（BSM），车辆向外发送的消息
void bsm_msg_fill(MessageFrame_t *msgFrame)
{
	msgFrame->present = MessageFrame_PR_bsmFrame;
	BasicSafetyMessage_t *bsm = &msgFrame->choice.bsmFrame;
	
	//int msgCnt = 0;
	static int msgCnt = 0;
    bsm->msgCnt = msgCnt++;//消息集编号，发送方为自己发送的同类消息依次进行编号,数值为 0~127,循环使用
	if (msgCnt > 127) {
		msgCnt = 0;
	}

	//车辆临时id编号
	bsm->id.buf = (uint8_t *)calloc(8, sizeof(uint8_t));//只能是8个字节，否则asn编码不通过
	bsm->id.size = 8;
	snprintf((char *)bsm->id.buf, bsm->id.size, "%s", "001");

	bsm->secMark = 10;//时间信息，定义 1 分钟内的毫秒级时刻。分辨率为 1 毫秒，有效范围是 0~59999。60000 及以上表示未知或无效数值。
	bsm->timeConfidence = NULL;//时间精度

	//车辆位置信息
	bsm->pos.lat = 12;
	bsm->pos.Long = 23;
	bsm->pos.elevation = NULL;

	bsm->posAccuracy = NULL;//车辆定位系统自身精度
	bsm->posConfidence = NULL;//车辆当前位置精度

	bsm->transmission = 2;//车辆挡位状态，0空档、1停止档、2前进档、3倒档
	bsm->speed = 23;//车速，分辨率为 0.02m/s，取值范围（(0..8191），数值 8191 表示无效数值
	bsm->heading = 57;//航向角，为运动方向与正北方向的顺时针夹角。分辨率为 0.0125°，取值范围0..28800
	bsm->angle = NULL;//方向盘转角，向右为正，向左为负。分辨率为 1.5°，取值范围(-126..127)，数值 127 为无效数值

	bsm->motionCfd = NULL;//车辆运行状态精度

	bsm->accelSet.Long = 12;//车辆4轴加速度
	bsm->accelSet.lat = 34;
	bsm->accelSet.vert = 56;
	bsm->accelSet.yaw = 78;

	//车辆刹车系统状态
	bsm->brakes.brakePadel = NULL;	/* OPTIONAL */
	bsm->brakes.wheelBrakes = NULL;	/* OPTIONAL */
	bsm->brakes.traction = NULL;	/* OPTIONAL */
	bsm->brakes.abs = NULL;	/* OPTIONAL */
	bsm->brakes.scs = NULL;	/* OPTIONAL */
	bsm->brakes.brakeBoost = NULL;	/* OPTIONAL */
	bsm->brakes.auxBrakes = NULL;	/* OPTIONAL */

	//车辆尺寸
	bsm->size.width = 90;
	bsm->size.length = 100;
	bsm->size.height = NULL;

	//车辆类型
	bsm->vehicleClass.classification = 20;
	bsm->vehicleClass.fuelType = NULL;

	//车辆安全辅助信息
	bsm->safetyExt = NULL;

	//紧急车辆附加信息
	bsm->emergencyExt = NULL;
}


//信号灯相位与配时消息（SPAT）
void spat_msg_fill(MessageFrame_t *msgFrame)
{
	msgFrame->present = MessageFrame_PR_spatFrame;
	SPAT_t *spat = &msgFrame->choice.spatFrame;
	
	//int msgCnt = 0;
	static int msgCnt = 0;
    spat->msgCnt = msgCnt++;//消息集编号，发送方为自己发送的同类消息依次进行编号,数值为 0~127,循环使用
	if (msgCnt > 127) {
		msgCnt = 0;
	}

	spat->moy = NULL;//时间信息，表示当前年份，已经过去的总分钟数（UTC 时间）。其分辨率为 1 分钟

	//时间信息,一分钟内毫秒数，分辨率为 1 毫秒，有效范围是 0~59999。60000 及以上表示未知或无效数值
    spat->timeStamp = (DSecond_t *)calloc(1, sizeof(DSecond_t));
    *spat->timeStamp = 20;

	//信号灯注释
    spat->name = (DescriptiveName_t *)calloc(1, sizeof(DescriptiveName_t));
    spat->name->buf = (uint8_t *)calloc(16, sizeof(uint8_t));
	spat->name->size = 16;

	int i = 0, j = 0, k = 0;
#if 1
	//路口列表
	int intersections_num = 2;
    spat->intersections.list.array = (IntersectionState_t **)calloc(intersections_num, sizeof(IntersectionState_t *));//这个参数必须要有，否则编码不通过,也就是数组至少有一个
    spat->intersections.list.count = intersections_num;
    spat->intersections.list.size = intersections_num;
    spat->intersections.list.free = NULL;
	for (i = 0; i < intersections_num; i++) {
		//单个路口信息
		IntersectionState_t *interSecState = (IntersectionState_t *)calloc(1, sizeof(IntersectionState_t));
		spat->intersections.list.array[i] = interSecState;
	
		//路口区域ID（结合map消息中的地图里面的节点编号）
		//region全局唯一的地区ID,一般由国家或区域性的管理部门统一管理和分配地图中各个划分区域的 ID 号，取值范围 (0..65535)，数值 0 仅用于测试。
		//id地区内部唯一的局部节点ID，路网最基本的构成即节点和节点之间连接的路段。节点可以是路口，也可以是一条路的端点。一个节点的 ID 在同一个区域内是唯一的。取值范围 (0..65535)，数值 0~255 预留为测试使用。
		interSecState->intersectionId.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t));
		interSecState->intersectionId.region[0] = 666;//全局唯一的地区ID
		interSecState->intersectionId.id = i + 1;//局部节点ID
		
		//路口信号机工作状态指示
/*
采用比特位数表示，对应比特位置1表示有效，比特位定义如下：
manualControlIsEnabled (bit 0), 人工控制启用
stopTimeIsActivated (bit 1), 相位和倒计时均暂停变化
failureFlash (bit 2), 故障灯闪烁
preemptIsActive (bit 3), 信号优先功能开启
signalPriorityIsActive (bit 4), 相关相位正处于优先控制的临时状态
fixedTimeOperation (bit 5), 处在定时控制状态
trafficDependentOperation (bit 6), 处在基于交通流的感应控制状态
standbyOperation (bit 7), 处于待机状态
failureMode (bit 8), 信号控制系统发生内部执行错误
off (bit 9), 信号控制系统关闭

下面数字除一些特别场景外，一般情况下不使用
recentMAPmessageUpdate (bit 10),
recentChangeInMAPassignedLanesIDsUsed (bit 11),
noValidMAPisAvailableAtThisTime (bit 12),
noValidSPATisAvailableAtThisTime (bit 13)
-- SPAT system is not working at this time
-- 位14,15保留，应为零

*/
		interSecState->status.buf = (uint8_t *)calloc(2, sizeof(uint8_t));//只能是2字节，表示16个bit位，否则编码不通过
		interSecState->status.buf[0] = 0B00100000;
		interSecState->status.buf[1] = 0B00000000;
		interSecState->status.size = 2;
		interSecState->status.bits_unused = 0;
		interSecState->moy = NULL;//时间信息，当年过去的总分钟数
		interSecState->timeStamp = NULL;//时间信息，一分钟内毫秒数
		interSecState->timeConfidence = NULL;//时间精度
		
		//信号灯相位列表
		int phases_num = 2;
		interSecState->phases.list.array = (Phase_t **)calloc(phases_num, sizeof(Phase_t *));
		interSecState->phases.list.count = phases_num;
		interSecState->phases.list.size = phases_num;
		interSecState->phases.list.free = NULL;
		for (j = 0; j < phases_num; j++) {
			Phase_t *phase = (Phase_t *)calloc(1, sizeof(Phase_t));
			interSecState->phases.list.array[j] = phase;
		
			phase->id = j + 1;//定义信号灯相位 ID。取值范围 (0..255)，数值 0 表示无效 ID。

			//信号灯状态列表
			int phaseStates_num = 3;
			phase->phaseStates.list.array = (PhaseState_t **)calloc(phaseStates_num, sizeof(PhaseState_t *));
			phase->phaseStates.list.count = phaseStates_num;
			phase->phaseStates.list.size = phaseStates_num;
			phase->phaseStates.list.free = NULL;
			for (k = 0; k < phaseStates_num; k++) {
				PhaseState_t* phaseState = (PhaseState_t *)calloc(1, sizeof(PhaseState_t));
				phase->phaseStates.list.array[k] = phaseState;
/*
信号灯相位的灯色状态
unavailable (0), 无效
dark (1), 熄灭
flashing-red (2), 红闪
red (3), 红灯
flashing-green (4), 绿闪
permissive-green (5), 绿灯，该相位是非专用相位，直行和左转车流存在冲突，相位冲突情况未知或无法确定情况下使用。
protected-green (6), 绿灯，该相位是专用相位，车流受信号控制系统保护
yellow (7), 黄灯
flashing-yellow (8) , 黄闪

*/
				phaseState->light = 3;//信号灯相位的灯色状态
				/*信号灯相位的计时状态, 提供两种可选的计时状态格式(选1就好)，
				一种是倒计时形式，另一种是 UTC 世界标准时间的形式。具体参照国标里面的信号灯相位图*/
				phaseState->timing = (TimeChangeDetails_t *)calloc(1, sizeof(TimeChangeDetails_t));
				phaseState->timing->present = TimeChangeDetails_PR_counting;
				/*开始时间,以 0.1 秒为单位，定义一小时中的时间，可以表示当前小时中的时刻，
				也可以表示长度不超过 1 小时的时间段。有效范围是 0~35999，
				数值 36000 表示大于 1 小时的时间长度，数值 36001 表示无效数值*/
				phaseState->timing->choice.counting.startTime = 135;//开始时间
				phaseState->timing->choice.counting.likelyEndTime = 285;//预计结束时间
			}
		}
	}
#endif
}




//地图消息（MAP）：路侧单元发出的局部地图消息，包括局部区域的路口信息、路段信息、车道信息，道路之间的连接关系等；
void map_msg_fill(MessageFrame_t *msgFrame)
{
	msgFrame->present = MessageFrame_PR_mapFrame;
	MapData_t *map = &msgFrame->choice.mapFrame;
	
	//int msgCnt = 0;
	static int msgCnt = 0;
    map->msgCnt = msgCnt++;//消息集编号，发送方为自己发送的同类消息依次进行编号,数值为 0~127,循环使用
	if (msgCnt > 127) {
		msgCnt = 0;
	}
	map->timeStamp = NULL;//时间信息，表示当前年份，已经过去的总分钟数（UTC 时间）。其分辨率为 1 分钟

	int i = 0, j = 0, k = 0, l = 0;
	/*地图节点集合，数量（1~32）*/
	int nodes_num = 1;
	map->nodes.list.array = (Node_t **)calloc(nodes_num, sizeof(Node_t *));
	map->nodes.list.count = nodes_num;
	map->nodes.list.size = nodes_num;
	map->nodes.list.free = NULL;
	for (i = 0; i < nodes_num; i++) {
		//单个地图节点信息
		Node_t *node = (Node_t *)calloc(1, sizeof(Node_t));
		map->nodes.list.array[i] = node;
	
		//地图节点名称，1~63字节
		node->name = (DescriptiveName_t *)calloc(1, sizeof(DescriptiveName_t));
		node->name->buf = (uint8_t *)calloc(63, sizeof(uint8_t));//只能是63字节，否则编码不通过
		node->name->size = 63;
		snprintf((char *)node->name->buf, node->name->size, "%s", "node0");
		
		//region全局唯一的地区ID,一般由国家或区域性的管理部门统一管理和分配地图中各个划分区域的 ID 号，取值范围 (0..65535)，数值 0 仅用于测试。
		//id地区内部唯一的局部节点ID，路网最基本的构成即节点和节点之间连接的路段。节点可以是路口，也可以是一条路的端点。一个节点的 ID 在同一个区域内是唯一的。取值范围 (0..65535)，数值 0~255 预留为测试使用。
		node->id.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t));
		node->id.region[0] = 666;//全局唯一的地区ID
		node->id.id = i + 1;//局部地区内节点ID
		
		//参考位置点坐标（消息中所有的位置偏移量，均基于该参考坐标计算。真实位置坐标等于偏移量加上参考坐标）
		node->refPos.lat = 231421543;//纬度， (-900000000..900000001) ，分辨率 1e-7°
		node->refPos.Long = 432154234;//经度， (-1799999999..1800000001)，分辨率 1e-7°
		node->refPos.elevation = NULL;//海拔高程。分辨率为 0.1 米。 (-4096..61439)，数值-4096 表示无效数值。

#if 1
		/*节点上下游路段集合列表，数量（1~32）*/
		node->inLinks = (LinkList_t *)calloc(1, sizeof(LinkList_t));
		int inLinks_num = 1;
		node->inLinks->list.array = (Link_t **)calloc(inLinks_num, sizeof(Link_t *));
		node->inLinks->list.count = inLinks_num;
		node->inLinks->list.size = inLinks_num;
		node->inLinks->list.free = NULL;
		for (j = 0; j < inLinks_num; j++) {
			//单个路段信息
			Link_t *inLink = (Link_t *)calloc(1, sizeof(Link_t));
			node->inLinks->list.array[j] = inLink;

			//路段名称，1~63字节
			inLink->name = (DescriptiveName_t *)calloc(1, sizeof(DescriptiveName_t));
			inLink->name->buf = (uint8_t *)calloc(63, sizeof(uint8_t));
			inLink->name->size = 63;
			snprintf((char *)inLink->name->buf, inLink->name->size, "%s", "road0");

			//路段上游节点ID
			inLink->upstreamNodeId.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t));
			inLink->upstreamNodeId.region[0] = 666;//全局唯一的地区ID
			inLink->upstreamNodeId.id = 5;//局部地区内节点ID

			/*路段限速集合列表，数量（1~9）*/
			inLink->speedLimits = (SpeedLimitList_t *)calloc(1, sizeof(SpeedLimitList_t));
			int speedLimits_num = 1;
			inLink->speedLimits->list.array = (RegulatorySpeedLimit_t **)calloc(speedLimits_num, sizeof(RegulatorySpeedLimit_t *));
			inLink->speedLimits->list.count = speedLimits_num;
			inLink->speedLimits->list.size = speedLimits_num;
			inLink->speedLimits->list.free = NULL;
			for (k = 0; k < speedLimits_num; k++) {
				RegulatorySpeedLimit_t *speedLimit = (RegulatorySpeedLimit_t *)calloc(1, sizeof(RegulatorySpeedLimit_t));
				inLink->speedLimits->list.array[k] = speedLimit;
/*
定义限速类型，指示给出的限速大小对应的参考类型
0 未知，unknown
1 学区内的最大速度，maxSpeedInSchoolZone
2 儿童学区最大速度，maxSpeedInSchoolZoneWhenChildrenArePresent
3 施工区最大速度，maxSpeedInConstructionZone
4 车辆最小速度，vehicleMinSpeed
5 车辆最大速度，vehicleMaxSpeed
6 车辆夜间最大速度，vehicleNightMaxSpeed
7 卡车最低速度，truckMinSpeed
8 卡车最大速度，truckMaxSpeed
9 卡车夜间最大速度，truckNightMaxSpeed
10 拖车最小速度的车辆，vehiclesWithTrailersMinSpeed
11 拖车最大速度的车辆，vehiclesWithTrailersMaxSpeed
12 拖车车辆夜间最大速度，vehiclesWithTrailersNightMaxSpeed	
*/
				speedLimit->type = 4;
				speedLimit->speed = (120 / 3.6) / 0.02;//速度大小，分辨率（单位）为0.02m/s，范围 (0..8191)，数值8191表示无效数值
			}

			inLink->linkWidth = (5 / 0.01);//路段宽度，分辨率为1cm，范围 (0..32767)

			/*路段中间点坐标列表，数量（2~31）*/
			inLink->points = (PointList_t *)calloc(1, sizeof(PointList_t));
			int points_num = 3;
			inLink->points->list.array = (RoadPoint_t **)calloc(points_num, sizeof(RoadPoint_t *));
			inLink->points->list.count = points_num;
			inLink->points->list.size = points_num;
			inLink->points->list.free = NULL;
			for (k = 0; k < points_num; k++) {
				RoadPoint_t *point = (RoadPoint_t *)calloc(1, sizeof(RoadPoint_t));
				inLink->points->list.array[k] = point;
			
				//坐标位置信息，经纬度偏差，其中64 比特经纬度位置，采用了真实经纬度数据进行描述，非相对位置偏差值
				point->posOffset.offsetLL.present = PositionOffsetLL_PR_position_LatLon;
				point->posOffset.offsetLL.choice.position_LatLon.lat = 231421543 + k;
				point->posOffset.offsetLL.choice.position_LatLon.lon = 231421543 + k;
				point->posOffset.offsetV = NULL;//垂直方向偏差
			}
			
			/*路段连接列表，数量（1~32）
			描述一条路段与下游路段的连接关系，以及该连接对应的本地路口处信号灯相位 ID。
			此处的相位 ID 事实上也是 MAP 消息与 SPAT 消息的唯一关联。
			车辆在确定了转向行为后，能够通过该相位 ID 数据，查看到 SPAT 中对应的相位实时状态，
			从而获得行车过程中的信号灯数据辅助。
			*/
			inLink->movements = (MovementList_t *)calloc(1, sizeof(MovementList_t));
			int movements_num = 3;
			inLink->movements->list.array = (Movement_t **)calloc(movements_num, sizeof(Movement_t *));
			inLink->movements->list.count = movements_num;
			inLink->movements->list.size = movements_num;
			inLink->movements->list.free = NULL;
			for (k = 0; k < movements_num; k++) {
				Movement_t *movement = (Movement_t *)calloc(1, sizeof(Movement_t));
				inLink->movements->list.array[k] = movement;
			
				//下游路段出口节点ID
				movement->remoteIntersection.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t));
				movement->remoteIntersection.region[0] = 666;//全局唯一的地区ID
				movement->remoteIntersection.id = 6;//局部地区内节点ID

				//下游路段信号灯相位ID，范围 (0..255)，数值0表示无效ID
				movement->phaseId = (PhaseID_t *)calloc(1, sizeof(PhaseID_t));
				movement->phaseId[0] = k + 1;
			}

			/*一个路段中包含的车道列表，数量（1~32）*/
			int lanes_num = 3;
			inLink->lanes.list.array = (Lane_t **)calloc(lanes_num, sizeof(Lane_t *));
			inLink->lanes.list.count = lanes_num;
			inLink->lanes.list.size = lanes_num;
			inLink->lanes.list.free = NULL;
			for (k = 0; k < lanes_num; k++) {
				//单个车道信息
				Lane_t *lane = (Lane_t *)calloc(1, sizeof(Lane_t));
				inLink->lanes.list.array[k] = lane;
				/*
				车道ID。车道定义在每一条有向路段上，同一条有向路段上的每个车道，都拥有一个单独的ID。
				车道号，以该车道行驶方向为参考，自左向右从1开始编号，
				范围(0..255)，0表示无效ID或者未知ID，255为保留数值
				*/
				lane->laneID = k + 1;
				
				//车道宽度，分辨率（单位）为1cm，范围 (0..32767)
				lane->laneWidth = (LaneWidth_t *)calloc(1, sizeof(LaneWidth_t));
				lane->laneWidth[0] = (2 / 0.01);

#if 1
				/*定义车道属性，包括车道共享情况（shareWith）以及车道本身所属的类别特性（laneType）,如人行横道等*/
				lane->laneAttributes = (LaneAttributes_t *)calloc(1, sizeof(LaneAttributes_t));

				lane->laneAttributes->shareWith = (LaneSharing_t *)calloc(1, sizeof(LaneSharing_t));
				lane->laneAttributes->shareWith->buf = (uint8_t *)calloc(2, sizeof(uint8_t));//只能是2字节，表示16个bit位，否则编码不通过
				unsigned short shareWith = 0B0000000000000111;
				lane->laneAttributes->shareWith->buf[0] = (shareWith & 0xff);
				lane->laneAttributes->shareWith->buf[1] = ((shareWith >> 8) & 0xff);
				lane->laneAttributes->shareWith->size = 2;
				lane->laneAttributes->shareWith->bits_unused = 6;//未使用的bit位数量，必须为6, 否则编码不通过
				
				/*车道类型，大类
				未知
				vehicle 机动车道
				crosswalk 人行横道车道
				bikeLane 自行车道
				sidewalk 人行道
				median 车道中间的隔离带
				striping 标线车道
				trackedVehicle 履带车道
				parking 停车车道
				*/
				lane->laneAttributes->laneType.present = LaneTypeAttributes_PR_vehicle;//车道类型，大类
				//车道细分类，比特位表示
				#if 0
				lane->laneAttributes->laneType.choice.vehicle.buf = (uint8_t *)calloc(2, sizeof(uint8_t));//只能是2字节，表示16个bit位，否则编码不通过
				lane->laneAttributes->laneType.choice.vehicle.buf[0] = 0;
				lane->laneAttributes->laneType.choice.vehicle.buf[1] = 0;
				lane->laneAttributes->laneType.choice.vehicle.size = 2;
				lane->laneAttributes->laneType.choice.vehicle.bits_unused = 0;
				#endif
#endif


#if 1
/*车道出口允许的所有转向行为
采用比特位数表示，对应比特位置1表示有效，比特位定义如下：
bit 0允许直行
bit 1允许左转
bit 2允许右转
bit 3允许掉头
bit 4行驶方向为左转且在红灯期间允许左转
bit 5行驶方向为右转且在红灯期间允许右转
bit 6允许变道
bit 7不允许停车
bit 8 yieldAllwaysRequired (8)
bit 9在停止线前停车瞭望，确保安全后通行
bit 10注意通过停车线
bit 11预留
*/
				lane->maneuvers = (AllowedManeuvers_t *)calloc(1, sizeof(AllowedManeuvers_t));//车道出口允许的所有转向行为
				lane->maneuvers->buf = (uint8_t *)calloc(2, sizeof(uint8_t));//只能是2字节，表示16个bit位，否则编码不通过
				unsigned short maneuvers = 0B0000000000000111;
				lane->maneuvers->buf[0] = (maneuvers & 0xff);
				lane->maneuvers->buf[1] = ((maneuvers >> 8) & 0xff);
				lane->maneuvers->size = 2;
				lane->maneuvers->bits_unused = 4;//未使用的bit位数量，必须为4, 否则编码不通过
#endif
				//车道与下游路段车道的连接关系列表，数量（1~8）
				lane->connectsTo = (ConnectsToList_t *)calloc(1, sizeof(ConnectsToList_t));
				int connectsTo_num = 2;
				lane->connectsTo->list.array = (Connection_t **)calloc(connectsTo_num, sizeof(Connection_t *));
				lane->connectsTo->list.count = connectsTo_num;
				lane->connectsTo->list.size = connectsTo_num;
				lane->connectsTo->list.free = NULL;
				for (l = 0; l < connectsTo_num; l++) {
					Connection_t *connect = (Connection_t *)calloc(1, sizeof(Connection_t));
					lane->connectsTo->list.array[l] = connect;
				
					//下游路段出口节点ID
					connect->remoteIntersection.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t));
					connect->remoteIntersection.region[0] = 666;//全局唯一的地区ID
					connect->remoteIntersection.id = 6;//局部地区内节点ID
					
					/*下游路段车道基本信息*/
					connect->connectingLane = (ConnectingLane_t *)calloc(1, sizeof(ConnectingLane_t));
					connect->connectingLane->lane = 5 + l;//车道ID，(0..255)，0表示无效ID
					//车道出口允许的所有转向行为，采用比特位数表示：同上所述
					connect->connectingLane->maneuver = (AllowedManeuvers_t *)calloc(1, sizeof(AllowedManeuvers_t));
					connect->connectingLane->maneuver->buf = (uint8_t *)calloc(2, sizeof(uint8_t));//只能是2字节，表示16个bit位，否则编码不通过
					unsigned short connect_maneuver = 0B0000000000000111;
					connect->connectingLane->maneuver->buf[0] = (connect_maneuver & 0xff);
					connect->connectingLane->maneuver->buf[1] = ((connect_maneuver >> 8) & 0xff);
					connect->connectingLane->maneuver->size = 2;
					connect->connectingLane->maneuver->bits_unused = 4;//未使用的bit位数量，必须为4, 否则编码不通过
					
					//下游路段车道信号灯相位号ID，范围 (0..255)，数值0表示无效ID
					connect->phaseId = (PhaseID_t *)calloc(1, sizeof(PhaseID_t));
					connect->phaseId[0] = 0;
				}

				
				//车道速度限制列表，数量（1~9）
				lane->speedLimits = (SpeedLimitList_t *)calloc(1, sizeof(SpeedLimitList_t));
				int lane_speedLimits_num = 1;
				lane->speedLimits->list.array = (RegulatorySpeedLimit_t **)calloc(lane_speedLimits_num, sizeof(RegulatorySpeedLimit_t *));
				lane->speedLimits->list.count = lane_speedLimits_num;
				lane->speedLimits->list.size = lane_speedLimits_num;
				lane->speedLimits->list.free = NULL;
				for (l = 0; l < lane_speedLimits_num; l++) {
					RegulatorySpeedLimit_t *lane_speedLimit = (RegulatorySpeedLimit_t *)calloc(1, sizeof(RegulatorySpeedLimit_t));
					lane->speedLimits->list.array[l] = lane_speedLimit;
					lane_speedLimit->type = 4;
					lane_speedLimit->speed = (120 / 3.6) / 0.02;//速度大小，分辨率（单位）为0.02m/s，范围 (0..8191)，数值8191表示无效数值
				}

				//车道中间点列表，数量（2~31）
				lane->points = (PointList_t *)calloc(1, sizeof(PointList_t));
				int lane_points_num = 3;
				lane->points->list.array = (RoadPoint_t **)calloc(lane_points_num, sizeof(RoadPoint_t *));
				lane->points->list.count = lane_points_num;
				lane->points->list.size = lane_points_num;
				lane->points->list.free = NULL;
				for (l = 0; l < lane_points_num; l++) {
					RoadPoint_t *lane_point = (RoadPoint_t *)calloc(1, sizeof(RoadPoint_t));
					lane->points->list.array[l] = lane_point;
				
					//坐标位置信息，经纬度偏差，其中64 比特经纬度位置，采用了真实经纬度数据进行描述，非相对位置偏差值
					lane_point->posOffset.offsetLL.present = PositionOffsetLL_PR_position_LatLon;
					lane_point->posOffset.offsetLL.choice.position_LatLon.lat = 231421544 + l;
					lane_point->posOffset.offsetLL.choice.position_LatLon.lon = 231421544 + l;
					lane_point->posOffset.offsetV = NULL;//垂直方向偏差
				}
			}
		}
#endif

	}
}



/**
地图消息（MAP）：路侧单元发出的局部地图消息，包括局部区域的路口信息、路段信息、车道信息，道路之间的连接关系等；

信号灯消息（SPAT）：信号灯消息，包含了一个或多个路口信号灯的当前状态信息。结合 MAP 消息，为车辆提供实时的前方信号灯相位信息；

路侧交通事件及交通标志消息（RSI）:路侧单元向周围车载单元发布的交通事件信息以及交通标志信息。
该消息帧能够打包一个或多个交通事件信息或者交通标志信息，同时包含发送该消息的路侧单元编号以及参考位置坐标。

路侧安全消息（RSM）:路侧单元传感器通过本身拥有的检测手段，得到其周边交通参与者的实时状态信息
（这里交通参与者包括路侧单元本身、周围车辆、非机动车、行人等）并将这些信息广播给周边车辆；


车辆基本安全消息（BSM）：车辆向外发送的消息，车辆通过该消息的广播，将自身的实时状态告知周围车辆，
**/
int main(int argc, char *argv[])
{
	MessageFrame_t msgFrame = {0};
	MessageFrame_t *msgFrameOut = NULL;
	
	unsigned char buf[2048] = {0};
	int ret = 0;
	memset(&msgFrame, 0, sizeof(MessageFrame_t));

#if 1
	printf("\nRSI------------------------------:\n");		
	rsi_msg_fill(&msgFrame);
	printf("id=%s eventType=%ld\n", msgFrame.choice.rsiFrame.id.buf, msgFrame.choice.rsiFrame.rtes->list.array[0]->eventType);

	printf("编码:\n");
	ret = v2x_encode_msgFrame(&msgFrame, (void *)buf, sizeof(buf));
	if (ret <= 0) {
		printf("v2x_encode_MsgFrame failed\n");		
	}
	v2x_rsi_msg_free(&msgFrame.choice.rsiFrame);

	if (ret > 0) {
		printf("解码:\n");		
		msgFrameOut = v2x_decode_msgFrame(buf, ret);
		printf("id=%s eventType=%ld\n", msgFrameOut->choice.rsiFrame.id.buf,  msgFrameOut->choice.rsiFrame.rtes->list.array[0]->eventType);
		v2x_decode_free(msgFrameOut);
	}
#endif


#if 1
	printf("\nRSM------------------------------:\n");		
	rsm_msg_fill(&msgFrame);
	printf("id=%s speed=%ld\n", msgFrame.choice.rsmFrame.id.buf, msgFrame.choice.rsmFrame.participants.list.array[0]->speed);
	ret = v2x_encode_msgFrame(&msgFrame, (void *)buf, sizeof(buf));
	if (ret <= 0) {
		printf("v2x_encode_MsgFrame failed\n");		
	}
	v2x_rsm_msg_free(&msgFrame.choice.rsmFrame);

	if (ret > 0) {
		msgFrameOut = v2x_decode_msgFrame(buf, ret);
		printf("id=%s speed=%ld\n", msgFrameOut->choice.rsmFrame.id.buf, msgFrameOut->choice.rsmFrame.participants.list.array[0]->speed);
		v2x_decode_free(msgFrameOut);
	}
#endif

#if 1
	printf("\nBSM------------------------------:\n");		
	bsm_msg_fill(&msgFrame);
	printf("id=%s classification=%ld\n", msgFrame.choice.bsmFrame.id.buf, msgFrame.choice.bsmFrame.vehicleClass.classification);
	ret = v2x_encode_msgFrame(&msgFrame, (void *)buf, sizeof(buf));
	if (ret <= 0) {
		printf("v2x_encode_MsgFrame failed\n"); 	
	}
	v2x_bsm_msg_free(&msgFrame.choice.bsmFrame);

	if (ret > 0) {
		msgFrameOut = v2x_decode_msgFrame(buf, ret);
		printf("id=%s classification=%ld\n", msgFrameOut->choice.rsmFrame.id.buf, msgFrameOut->choice.bsmFrame.vehicleClass.classification);
		v2x_decode_free(msgFrameOut);
	}
#endif

#if 1
	printf("\nSPAT------------------------------:\n");		
	spat_msg_fill(&msgFrame);
	printf("timeStamp=%ld\n", msgFrame.choice.spatFrame.timeStamp[0]);
	ret = v2x_encode_msgFrame(&msgFrame, (void *)buf, sizeof(buf));
	if (ret <= 0) {
		printf("v2x_encode_MsgFrame failed\n"); 	
	}
	v2x_spat_msg_free(&msgFrame.choice.spatFrame);

	if (ret > 0) {
		msgFrameOut = v2x_decode_msgFrame(buf, ret);
		printf("timeStamp=%ld\n", msgFrameOut->choice.spatFrame.timeStamp[0]);
		v2x_decode_free(msgFrameOut);
	}
#endif

#if 1
	printf("\nMAP------------------------------:\n");

	map_msg_fill(&msgFrame);
	printf("node0 name=%s\n", msgFrame.choice.mapFrame.nodes.list.array[0]->name->buf);
	ret = v2x_encode_msgFrame(&msgFrame, (void *)buf, sizeof(buf));
	if (ret <= 0) {
		printf("v2x_encode_MsgFrame failed\n"); 	
	}
	v2x_map_msg_free(&msgFrame.choice.mapFrame);

	if (ret > 0) {
		msgFrameOut = v2x_decode_msgFrame(buf, ret);
		printf("node0 name=%s\n", msgFrameOut->choice.mapFrame.nodes.list.array[0]->name->buf);
		v2x_decode_free(msgFrameOut);
	}
#endif
	return 0;
}
