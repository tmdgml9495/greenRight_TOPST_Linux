#include "mqtt_handler.h"

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mqtt_topics.h"
#include "vehicle_codec.h"

static uint8_t parse_traffic_light_topic_id(const char* topic)
{
    const char* prefix = "v2x/trafficlight/";
    size_t prefix_len = strlen(prefix);
    if (!topic || strncmp(topic, prefix, prefix_len) != 0) return 0xFF;

    const char* id = topic + prefix_len;
    if (id[0] == 'T' && id[1] == 'L') {
        id += 2;
    }

    char* end = NULL;
    long value = strtol(id, &end, 10);
    if (end == id || value < 0 || value > 255) return 0xFF;
    return (uint8_t)value;
}

static void on_message(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* msg)
{
    (void)mosq;
    MqttHandler* handler = (MqttHandler*)userdata;
    if (!handler || !msg || !msg->topic || msg->payloadlen <= 0) return;

    char* payload = (char*)malloc((size_t)msg->payloadlen + 1);
    if (!payload) return;
    memcpy(payload, msg->payload, (size_t)msg->payloadlen);
    payload[msg->payloadlen] = '\0';

    if (strncmp(msg->topic, TOPIC_VEHICLE_STATUS_PREFIX, strlen(TOPIC_VEHICLE_STATUS_PREFIX)) == 0) {
        VehicleInfo vehicle;
        if (vehicle_info_from_json_string(payload, &vehicle) && vehicle.vehicle_id != handler->vehicle_id) {
            if (handler->callbacks.on_vehicle) {
                handler->callbacks.on_vehicle(&vehicle, handler->callbacks.user_data);
            }
        }
    } else if (strncmp(msg->topic, "v2x/trafficlight/", strlen("v2x/trafficlight/")) == 0) {
        uint8_t tl_id = parse_traffic_light_topic_id(msg->topic);
        TrafficLight traffic_light;
        if (tl_id != 0xFF && traffic_light_from_json_string(payload, &traffic_light)) {
            if (handler->callbacks.on_traffic_light) {
                handler->callbacks.on_traffic_light(tl_id, &traffic_light, handler->callbacks.user_data);
            }
        }
    }

    free(payload);
}

static void on_connect(struct mosquitto* mosq, void* userdata, int rc)
{
    MqttHandler* handler = (MqttHandler*)userdata;
    if (!handler) return;

    if (rc != 0) {
        fprintf(stderr, "[MQTT] connect failed: %s\n", mosquitto_connack_string(rc));
        atomic_store(&handler->connected, false);
        return;
    }

    atomic_store(&handler->connected, true);
    printf("[MQTT] connected vehicle_id=%u\n", handler->vehicle_id);
    mosquitto_subscribe(mosq, NULL, TOPIC_VEHICLE_STATUS_WILDCARD, 0);
    mosquitto_subscribe(mosq, NULL, TOPIC_TRAFFICLIGHT_WILDCARD, 0);
    printf("[MQTT] subscribed: %s, %s\n", TOPIC_VEHICLE_STATUS_WILDCARD, TOPIC_TRAFFICLIGHT_WILDCARD);
}

static void on_disconnect(struct mosquitto* mosq, void* userdata, int rc)
{
    (void)mosq;
    MqttHandler* handler = (MqttHandler*)userdata;
    if (!handler) return;

    atomic_store(&handler->connected, false);
    if (rc != 0) {
        fprintf(stderr, "[MQTT] disconnected unexpectedly rc=%d\n", rc);
    } else {
        printf("[MQTT] disconnected\n");
    }
}

bool mqtt_handler_init(
    MqttHandler* handler,
    const char* host,
    int port,
    uint8_t vehicle_id,
    const MqttHandlerCallbacks* callbacks
)
{
    if (!handler || !host) return false;
    memset(handler, 0, sizeof(*handler));
    atomic_init(&handler->connected, false);
    handler->vehicle_id = vehicle_id;
    if (callbacks) {
        handler->callbacks = *callbacks;
    }

    mosquitto_lib_init();

    char client_id[64];
    snprintf(client_id, sizeof(client_id), "v2x_vehicle_%02u", vehicle_id);

    handler->mosq = mosquitto_new(client_id, true, handler);
    if (!handler->mosq) {
        fprintf(stderr, "[MQTT] mosquitto_new failed\n");
        mosquitto_lib_cleanup();
        return false;
    }

    snprintf(handler->status_topic, sizeof(handler->status_topic), TOPIC_VEHICLE_STATUS_FMT, (unsigned)vehicle_id);

    mosquitto_connect_callback_set(handler->mosq, on_connect);
    mosquitto_disconnect_callback_set(handler->mosq, on_disconnect);
    mosquitto_message_callback_set(handler->mosq, on_message);
    /*
     * [기술력] LWT (Last Will and Testament) 설정
     * mosquitto_connect() 이전에 반드시 설정해야 브로커에 등록된다.
     * 이 클라이언트가 정상 종료(disconnect) 없이 죽거나 네트워크가 끊겨
     * keepalive(60s) 동안 응답이 없으면, 브로커가 우리 대신
     * status_topic 으로 "빈 payload + retain=true" 메시지를 발행해준다.
     * -> 다른 차량들은 이 retained 빈 메시지를 받고 "이 vehicle_id는 이탈했다"고 판단할 수 있고,
     *    타임아웃(mqtt_cleanup_stale_vehicles)보다 더 빠르게, 확실하게 이탈을 감지할 수 있다.
     */
    mosquitto_will_set(handler->mosq, handler->status_topic, 0, NULL, 0, true);

    int rc = mosquitto_connect(handler->mosq, host, port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[MQTT] broker connect failed %s:%d: %s\n", host, port, mosquitto_strerror(rc));
        mosquitto_destroy(handler->mosq);
        handler->mosq = NULL;
        mosquitto_lib_cleanup();
        return false;
    }

    rc = mosquitto_loop_start(handler->mosq);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[MQTT] loop_start failed: %s\n", mosquitto_strerror(rc));
        mosquitto_disconnect(handler->mosq);
        mosquitto_destroy(handler->mosq);
        handler->mosq = NULL;
        mosquitto_lib_cleanup();
        return false;
    }

    handler->initialized = true;
    return true;
}

void mqtt_handler_cleanup(MqttHandler* handler)
{
    if (!handler || !handler->initialized) return;
    atomic_store(&handler->connected, false);
    mosquitto_loop_stop(handler->mosq, true);
    mosquitto_disconnect(handler->mosq);
    mosquitto_destroy(handler->mosq);
    mosquitto_lib_cleanup();
    handler->mosq = NULL;
    handler->initialized = false;
}

bool mqtt_handler_publish_vehicle_info(MqttHandler* handler, const VehicleInfo* vehicle)
{
    if (!handler || !handler->initialized || !handler->mosq || !vehicle) return false;

    char* json = vehicle_info_to_json_string(vehicle);
    if (!json) return false;

    int rc = mosquitto_publish(
        handler->mosq,
        NULL,
        handler->status_topic,
        (int)strlen(json),
        json,
        0,
        true
    );
    free(json);
    return rc == MOSQ_ERR_SUCCESS;
}

bool mqtt_handler_is_connected(const MqttHandler* handler)
{
    if (!handler || !handler->initialized) return false;
    return atomic_load(&handler->connected);
}
