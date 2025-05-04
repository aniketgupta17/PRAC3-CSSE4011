#!/usr/bin/env python3
import sys, json, time, serial, requests
INFLUX = "http://localhost:8086/api/v2/write?org=uq&bucket=prac3&precision=ns"
TOKEN  = "supertoken"
HEADERS= {"Authorization": f"Token {TOKEN}"}

def json_to_lp(j):
    tags   = [f"method={j['method']}"]
    if 'ibeacon' in j: tags.append(f"ibeacon={j['ibeacon']}")
    fields = [f"{k}={j[k]}" for k in ["x","y","vx","vy","rssi","us_range"] if k in j]
    return f"position,{','.join(tags)} {','.join(fields)} {j['ts_ns']}"

def main(port):
    ser = serial.Serial(port, 115200, timeout=1)
    while True:
        line = ser.readline()
        if not line:
            continue
        try:
            pkt = json.loads(line)
            lp  = json_to_lp(pkt)
            requests.post(INFLUX, headers=HEADERS, data=lp.encode())
        except Exception as e:
            print("parse/write error:", e, file=sys.stderr)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: serial_to_influx.py /dev/ttyUSB0")
        sys.exit(1)
    main(sys.argv[1])