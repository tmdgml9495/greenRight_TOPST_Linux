#ifndef _MQTT_TOPICS_H_
#define _MQTT_TOPICS_H_

/*
 * 토픽 설계 (서버 없음 - 브로커의 pub/sub 만으로 동작)
 *
 * [차량 상태 (각 차량이 주기적으로 자신의 정보를 publish)]
 *  - 차량 -> 전체 : v2x/vehicle/<vehicle_id>/status   (payload: VehicleInfo JSON)
 *    -> 모든 차량이 v2x/vehicle/+/status 를 구독하면 브로커가 알아서 서로에게 뿌려준다.
 *    -> LWT(Last Will)로 동일 토픽에 빈 payload, retain=true 를 등록해서
 *       연결이 끊기면 다른 차량들이 즉시 감지(Fail-Safe, 통신 두절 처리)
 *
 * [신호등 정보 (추후 ESP32/V2I 쪽에서 publish 예정)]
 *  - v2x/trafficlight/<linked_tl_id>/status    (payload: {"color":"red|yellow|green","time_left":N}, retain=true)
 *    -> 신호등 ID는 topic의 <linked_tl_id>로 구분한다.
 */

#define TOPIC_VEHICLE_STATUS_PREFIX   "v2x/vehicle/"
#define TOPIC_VEHICLE_STATUS_FMT      "v2x/vehicle/%u/status"
#define TOPIC_VEHICLE_STATUS_WILDCARD "v2x/vehicle/+/status"

#define TOPIC_TRAFFICLIGHT_FMT        "v2x/trafficlight/%u/status"
#define TOPIC_TRAFFICLIGHT_WILDCARD   "v2x/trafficlight/+/status"

/* vehicle_id constants are defined in types.h. */

/* 차량 상태 publish 주기 (ms) - 발표자료 상 100ms 이내 송수신 기준 */
#define VEHICLE_PUBLISH_PERIOD_MS   100

#endif /* _MQTT_TOPICS_H_ */
