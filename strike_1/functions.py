import psutil, netifaces


def find_wifi_address():
    interfaces = netifaces.interfaces()

    for interface in interfaces:
        addresses = netifaces.ifaddresses(interface)
        if netifaces.AF_INET in addresses:  # Check if the interface has an IPv4 address
            ip_info = addresses[netifaces.AF_INET][0]
            ip_address = ip_info["addr"]
            if "etho" in interface.lower():
                return ip_address  # Return the WiFi IP address


def is_python_script_running(script_name):
    for proc in psutil.process_iter(["pid", "name", "cmdline"]):
        if proc.info["name"] == "python.exe" and len(proc.info["cmdline"]) > 1:
            if script_name in proc.info["cmdline"][1]:
                return True
    return False


def killProcess(script_name):
    for proc in psutil.process_iter(["pid", "name", "cmdline"]):
        if proc.info["name"] == "python.exe" and len(proc.info["cmdline"]) > 1:
            if script_name in proc.info["cmdline"][1]:
                proc.terminate()
                return True
    return False


killProcess("test.py")
