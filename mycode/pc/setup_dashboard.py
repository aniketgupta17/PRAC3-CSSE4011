#!/usr/bin/env python3
import requests
import json
import time
import os
import sys

# Configuration
INFLUX_URL = "http://influxdb:8086"  # Changed from localhost to Docker service name
GRAFANA_URL = "http://localhost:3000"
INFLUX_TOKEN = "supertoken"
GRAFANA_USER = "admin"
GRAFANA_PASSWORD = "admin"
KALMAN_DASHBOARD_FILE = "kalman_dashboard.json"

def wait_for_service(url, name, max_attempts=60, delay=1):
    """Wait for a service to become available"""
    print(f"Waiting for {name} to start...", end="", flush=True)
    for attempt in range(max_attempts):
        try:
            response = requests.get(url, timeout=2)
            if response.status_code < 500:
                print(f"\n{name} is ready!")
                return True
        except requests.RequestException:
            pass
        
        sys.stdout.write(".")
        sys.stdout.flush()
        time.sleep(delay)
    
    print(f"\nFailed to connect to {name} after {max_attempts} attempts")
    return False

def setup_influxdb_datasource():
    """Create the InfluxDB data source in Grafana"""
    print("Setting up InfluxDB data source in Grafana...")
    
    # Login to Grafana and get session cookie
    session = requests.Session()
    login_data = {"user": GRAFANA_USER, "password": GRAFANA_PASSWORD}
    response = session.post(f"{GRAFANA_URL}/login", json=login_data)
    
    if response.status_code != 200:
        print(f"Failed to login to Grafana: {response.status_code} {response.text}")
        return False
    
    # Check if InfluxDB datasource already exists
    response = session.get(f"{GRAFANA_URL}/api/datasources/name/InfluxDB")
    
    # If datasource exists, delete it to recreate with correct settings
    if response.status_code == 200:
        datasource_id = response.json()["id"]
        print(f"InfluxDB datasource already exists (ID: {datasource_id}), updating configuration...")
        session.delete(f"{GRAFANA_URL}/api/datasources/{datasource_id}")
    
    # Create InfluxDB datasource with Docker network configuration
    datasource = {
        "name": "InfluxDB",
        "type": "influxdb",
        "access": "proxy",
        "url": "http://influxdb:8086",  # Use Docker service name
        "jsonData": {
            "version": "Flux",
            "organization": "uq",
            "defaultBucket": "prac3",
            "tlsSkipVerify": True
        },
        "secureJsonData": {
            "token": INFLUX_TOKEN
        }
    }
    
    response = session.post(f"{GRAFANA_URL}/api/datasources", json=datasource)
    
    if response.status_code in [200, 201]:
        print("InfluxDB datasource created successfully!")
        return True
    else:
        print(f"Failed to create InfluxDB datasource: {response.status_code} {response.text}")
        return False

def import_dashboard():
    """Import the Kalman dashboard into Grafana"""
    print("Importing Kalman dashboard into Grafana...")
    
    # Check if dashboard file exists
    if not os.path.exists(KALMAN_DASHBOARD_FILE):
        print(f"Dashboard file '{KALMAN_DASHBOARD_FILE}' not found!")
        return False
    
    # Login to Grafana and get session cookie
    session = requests.Session()
    login_data = {"user": GRAFANA_USER, "password": GRAFANA_PASSWORD}
    response = session.post(f"{GRAFANA_URL}/login", json=login_data)
    
    if response.status_code != 200:
        print(f"Failed to login to Grafana: {response.status_code} {response.text}")
        return False
    
    # Load dashboard JSON
    with open(KALMAN_DASHBOARD_FILE, 'r') as f:
        dashboard_json = json.load(f)
    
    # Prepare dashboard import data
    dashboard_import = {
        "dashboard": dashboard_json,
        "overwrite": True,
        "inputs": [
            {
                "name": "DS_INFLUXDB",
                "type": "datasource",
                "pluginId": "influxdb",
                "value": "InfluxDB"
            }
        ]
    }
    
    # Import dashboard
    response = session.post(f"{GRAFANA_URL}/api/dashboards/import", json=dashboard_import)
    
    if response.status_code in [200, 201]:
        result = response.json()
        print(f"Dashboard imported successfully! URL: {GRAFANA_URL}{result['importedUrl']}")
        return True
    else:
        print(f"Failed to import dashboard: {response.status_code} {response.text}")
        return False

def main():
    print("Setting up Prac 3 dashboard environment...")
    
    # Wait for Grafana to start
    if not wait_for_service(GRAFANA_URL, "Grafana"):
        return
    
    # Setup InfluxDB datasource in Grafana
    if not setup_influxdb_datasource():
        return
    
    # Import Kalman dashboard
    if not import_dashboard():
        return
    
    print("\nSetup complete! You can now access your dashboard at:")
    print(f"{GRAFANA_URL}/d/prac3-loc/prac-3-localisation")
    print("\nIMPORTANT: Make sure to update your mock_bluetooth_data.py script")
    print("to use the correct InfluxDB connection URL:")
    print("INFLUX = \"http://localhost:8086/api/v2/write?org=uq&bucket=prac3&precision=ns\"")

if __name__ == "__main__":
    main()