#!/usr/bin/env python3
import threading
import time
import subprocess
from flask import Flask, render_template, jsonify

# --- CONFIGURATION ---
IPS_TO_PING = {
    'JETSON': '192.168.1.16',
    'RASPI':  '192.168.1.17' 
}

# Global dictionary to store live status (Thread-safe enough for this use case)
network_status = {key: False for key in IPS_TO_PING}

def network_watchdog():
    """Background thread that pings devices continuously."""
    print(f"--- Starting Watchdog ---")
    print(f"Monitoring: {IPS_TO_PING}")
    
    while True:
        for name, ip in IPS_TO_PING.items():
            try:
                # ping -c 1 (count 1), -W 1 (wait 1 sec max)
                # stdout/stderr sent to DEVNULL to keep terminal clean
                response = subprocess.call(
                    ['ping', '-c', '1', '-W', '1', ip],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                
                is_online = (response == 0)
                
                # Optional: Print change in status to terminal
                if network_status[name] != is_online:
                    status_str = "ONLINE" if is_online else "OFFLINE"
                    print(f"[Watchdog] {name} ({ip}) is now {status_str}")

                network_status[name] = is_online
                
            except Exception as e:
                print(f"Error pinging {name}: {e}")
                network_status[name] = False
        
        time.sleep(1.0) # Wait 1 second before next batch

# --- Flask Setup ---
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/status')
def get_status():
    return jsonify(network_status)

if __name__ == '__main__':
    # Start the pinger in a background thread
    pinger_thread = threading.Thread(target=network_watchdog, daemon=True)
    pinger_thread.start()
    
    # Start Web Server
    print("Starting Web Server on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000)