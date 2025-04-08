from flask import Flask, jsonify, request
from flask_cors import CORS
from functions import find_wifi_address, is_python_script_running, killProcess
import os, subprocess, csv, time

app = Flask(__name__)
CORS(app)

host = find_wifi_address()
current_directory = "C:/Users/vshar/OneDrive/Documents/strike/strike/strike_1"


@app.route("/ping/<ip>")
def PingDevices(ip):
    global host
    if ip == host:
        return jsonify({"message": True})
    else:
        return jsonify({"message": False})


@app.route("/start_strike", methods=["POST"])
def start_strike():
    with open(current_directory + "/flag.csv", "w", newline="") as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(["0"])
    try:
        subprocess.Popen(
            'start cmd.exe /k "C:/python311/python.exe C:/Users/vshar/OneDrive/Documents/strike/strike/strike_1/strike.py"',
            shell=True,
        )
        time.sleep(1)
        if is_python_script_running("{}/strike.py".format(current_directory)):
            return jsonify({"message": "started capturing"})
        else:
            return jsonify({"message": "Check again"})
    except Exception as e:
        return jsonify({"message": str(e)})


@app.route("/stop_strike", methods=["POST"])
def stop_strike():
    with open(current_directory + "/flag.csv", "w", newline="") as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(["1"])
    time.sleep(1)
    if is_python_script_running("{}/strike.py".format(current_directory)):
        is_killed1 = killProcess("{}/strike.py".format(current_directory))
        if is_killed1:
            return jsonify({"message": "Stoped"})
        else:
            return jsonify({"message": "Couldn't Stoped"})
    else:
        return jsonify({"message": "Stoped"})


@app.route("/check_running")
def check_running():
    try:
        is_running_cam = is_python_script_running(
            "{}/strike.py".format(current_directory)
        )
        data = {}
        if is_running_cam:
            data["strike"] = "Running"
        else:
            data["strike"] = "Not Running"
        return jsonify(data)
    except Exception as e:
        return jsonify({"message": str(e)})


if __name__ == "__main__":
    app.run(host="192.168.112.20", port=8000, debug=True)
