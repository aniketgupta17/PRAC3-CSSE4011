#!/usr/bin/env python3
from flask import Flask, request, jsonify
import time
import requests

app = Flask(__name__)

# InfluxDB config (same as mock)
INFLUX_URL = "http://localhost:8086/api/v2/write?org=uq&bucket=prac3&precision=ns"
TOKEN = "supertoken"
HEADERS = {
    "Authorization": f"Token {TOKEN}",
    "Content-Type": "text/plain; charset=utf-8"
}

# Create line protocol entry
def line_protocol(meas, tags: dict, fields: dict, ts: int):
    tag_str = ",".join(f"{k}={v}" for k, v in tags.items())
    field_str = ",".join(f"{k}={v}" for k, v in fields.items())
    return f"{meas},{tag_str} {field_str} {ts}"

@app.route("/position", methods=["POST"])
def handle_position():
    try:
        data = request.get_json(force=True)

        # Validate input
        for k in ["x", "y", "vx", "vy"]:
            if k not in data:
                return f"Missing field: {k}", 400

        # Convert values
        x = float(data["x"])
        y = float(data["y"])
        vx = float(data["vx"])
        vy = float(data["vy"])
        ts = int(time.time() * 1e9)  # nanosecond precision

        # Match mock format exactly (kalman-filtered fused result)
        kalman_packet = line_protocol(
            "position",
            {"method": "kalman"},
            {"x": x, "y": y, "vx": vx, "vy": vy},
            ts
        )

        # Optional: add synthetic ultrasonic or rssi if needed

        # Send to InfluxDB
        response = requests.post(INFLUX_URL, headers=HEADERS, data=kalman_packet.encode())

        if response.status_code != 204:
            print(f"InfluxDB error: {response.status_code} - {response.text}")
            return "Failed to write to InfluxDB", 500

        print(f"✔ Real position received and forwarded: x={x:.2f}, y={y:.2f}, vx={vx:.2f}, vy={vy:.2f}")
        return jsonify({"status": "success"}), 200

    except Exception as e:
        print(f"❌ Error handling request: {e}")
        return "Internal server error", 500

if __name__ == "__main__":
    print("🛰️ Real Position Receiver started on http://0.0.0.0:5001")
    app.run(host="0.0.0.0", port=5001)