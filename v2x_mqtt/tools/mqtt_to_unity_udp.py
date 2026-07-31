#!/usr/bin/env python3
"""
Bridge GreenRight vehicle MQTT messages to Unity over UDP.

Example:
  python3 tools/mqtt_to_unity_udp.py --broker 192.168.0.11 --port 1884
  python3 tools/mqtt_to_unity_udp.py --broker 192.168.0.11 --port 1884 --vehicle-id 1

Unity side should listen on UDP 127.0.0.1:5005 by default.
"""

import argparse
import json
import socket
import sys
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("Missing dependency: paho-mqtt", file=sys.stderr)
    print("Install with: python3 -m pip install paho-mqtt", file=sys.stderr)
    raise


def parse_args():
    parser = argparse.ArgumentParser(description="MQTT vehicle topic to Unity UDP bridge")
    parser.add_argument("--broker", default="192.168.0.11", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1884, help="MQTT broker port")
    parser.add_argument("--vehicle-id", type=int, default=None, help="Forward only this vehicle ID. If omitted, forward all vehicles.")
    parser.add_argument("--udp-host", default="127.0.0.1", help="Unity UDP receiver host")
    parser.add_argument("--udp-port", type=int, default=5005, help="Unity UDP receiver port")
    parser.add_argument("--topic", default=None, help="Override MQTT topic")
    parser.add_argument("--print-every", type=int, default=10, help="Print one forwarded packet every N messages")
    return parser.parse_args()


def main():
    args = parse_args()
    topic = args.topic or (f"v2x/vehicle/{args.vehicle_id}/status" if args.vehicle_id is not None else "v2x/vehicle/+/status")

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_target = (args.udp_host, args.udp_port)
    forwarded_count = 0

    def on_connect(client, userdata, flags, rc, properties=None):
        if rc != 0:
            print(f"[bridge] MQTT connect failed rc={rc}", file=sys.stderr)
            return
        print(f"[bridge] MQTT connected {args.broker}:{args.port}")
        print(f"[bridge] subscribing {topic}")
        client.subscribe(topic, qos=0)

    def on_disconnect(client, userdata, rc, properties=None):
        if rc != 0:
            print(f"[bridge] MQTT disconnected rc={rc}; paho will try to reconnect")
        else:
            print("[bridge] MQTT disconnected")

    def on_message(client, userdata, msg):
        nonlocal forwarded_count
        try:
            payload = msg.payload.decode("utf-8")
            vehicle = json.loads(payload)
        except Exception as exc:
            print(f"[bridge] bad payload topic={msg.topic}: {exc}", file=sys.stderr)
            return

        vehicle_id = int(vehicle.get("vehicle_id", -1))
        if args.vehicle_id is not None and vehicle_id != args.vehicle_id:
            return

        # Keep the original fields and add a bridge receive timestamp for debugging.
        vehicle["bridge_received_ms"] = int(time.time() * 1000)
        out = json.dumps(vehicle, separators=(",", ":")).encode("utf-8")
        udp.sendto(out, udp_target)

        forwarded_count += 1
        if args.print_every > 0 and forwarded_count % args.print_every == 1:
            print(
                "[bridge] forwarded "
                f"id={vehicle_id} "
                f"x={vehicle.get('x')} y={vehicle.get('y')} "
                f"heading={vehicle.get('heading')} "
                f"lane={vehicle.get('lanelet_id')} "
                f"cz={vehicle.get('conflict_zone_ids')} "
                f"-> udp://{args.udp_host}:{args.udp_port}"
            )

    client_id_suffix = args.vehicle_id if args.vehicle_id is not None else "all"
    client = mqtt.Client(client_id=f"greenright_unity_bridge_{client_id_suffix}", protocol=mqtt.MQTTv311)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=5)

    print(f"[bridge] connecting to MQTT {args.broker}:{args.port}")
    if args.vehicle_id is None:
        print(f"[bridge] forwarding all vehicles to udp://{args.udp_host}:{args.udp_port}")
    else:
        print(f"[bridge] forwarding vehicle_id={args.vehicle_id} to udp://{args.udp_host}:{args.udp_port}")
    client.connect(args.broker, args.port, keepalive=30)
    client.loop_forever()


if __name__ == "__main__":
    main()
