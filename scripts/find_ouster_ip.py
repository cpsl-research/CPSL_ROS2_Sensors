#!/usr/bin/env python3
import argparse
import concurrent.futures
import json
import re
import socket
import struct
import subprocess
import requests
import errno

# CLI Colors for output
GREEN = "\033[92m"
BLUE = "\033[94m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
END = "\033[0m"

def get_local_interfaces():
    """Detect local network interfaces, IPs, subnets, and broadcasts using 'ip addr'."""
    interfaces = []
    try:
        # Run ip command
        output = subprocess.check_output(["ip", "-o", "addr", "show", "scope", "global"], text=True)
        for line in output.splitlines():
            # Match line: <num>: <interface> inet <ip>/<cidr> brd <broadcast> ...
            match = re.search(r'\d+:\s+(\S+)\s+inet\s+(\d+\.\d+\.\d+\.\d+)/(\d+)(?:\s+brd\s+(\d+\.\d+\.\d+\.\d+))?', line)
            if match:
                iface, ip, cidr, brd = match.groups()
                interfaces.append({
                    "interface": iface,
                    "ip": ip,
                    "cidr": int(cidr),
                    "broadcast": brd
                })
    except Exception as e:
        print(f"{YELLOW}Warning: Could not auto-detect network interfaces via 'ip': {e}{END}")
    return interfaces

def get_arp_ips():
    """Read the host's ARP table from /proc/net/arp."""
    ips = set()
    try:
        with open("/proc/net/arp", "r") as f:
            lines = f.readlines()
            for line in lines[1:]:  # skip header
                parts = line.split()
                if len(parts) >= 1:
                    ip = parts[0]
                    # Validate IP format
                    if re.match(r'^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$', ip):
                        ips.add(ip)
    except Exception as e:
        print(f"{YELLOW}Warning: Could not read ARP table: {e}{END}")
    return ips

def check_ip_alive(ip, timeout=0.3):
    """
    Check if a host is alive by attempting a TCP connection to port 80 (HTTP)
    or port 22 (SSH) which are exposed by Ouster lidars.
    This is extremely fast and avoids launching shell ping processes.
    """
    for port in (80, 22):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(timeout)
                res = s.connect_ex((ip, port))
                if res == 0 or res == errno.ECONNREFUSED:
                    return True
        except Exception:
            pass
    return False

def ping_ip(ip, timeout=0.5):
    """Fallback ping method using ICMP."""
    try:
        w_param = max(1, int(timeout))
        cmd = ["ping", "-c", "1", "-W", str(w_param), "-w", str(w_param), ip]
        result = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return result.returncode == 0
    except Exception:
        return False

def broadcast_ping(broadcast_ip):
    """Send a broadcast ping to populate the ARP table."""
    if not broadcast_ip:
        return
    try:
        # Allow pinging broadcast (-b flag needed in Linux)
        subprocess.run(["ping", "-b", "-c", "1", "-w", "1", broadcast_ip],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass

def check_ouster_api(ip, timeout=0.5):
    """
    Check if the host at the given IP address is an Ouster lidar
    by querying its HTTP API endpoints.
    """
    endpoints = [
        f"http://{ip}/api/v1/system/status",
        f"http://{ip}/api/v1/sensor/metadata"
    ]
    for url in endpoints:
        try:
            response = requests.get(url, timeout=timeout)
            if response.status_code == 200:
                return response.json()
        except Exception:
            continue
    return None

def cidr_to_ips(ip, cidr):
    """Generate all host IP addresses in a CIDR subnet."""
    try:
        # Parse IP to 32-bit integer
        ip_parts = [int(x) for x in ip.split('.')]
        ip_num = (ip_parts[0] << 24) + (ip_parts[1] << 16) + (ip_parts[2] << 8) + ip_parts[3]
        
        # Calculate mask
        mask = (0xFFFFFFFF << (32 - cidr)) & 0xFFFFFFFF
        network = ip_num & mask
        
        # Determine number of hosts
        num_hosts = 1 << (32 - cidr)
        
        # If subnet is a /16 (like 169.254.0.0/16), generate the full link-local range:
        # 169.254.1.1 to 169.254.254.254 (per RFC 3927)
        if num_hosts > 1024:
            ips = []
            if ip.startswith("169.254"):
                # All possible link-local addresses for Ouster Lidar fall within 169.254.1.1 - 169.254.254.254
                for i in range(1, 255):
                    for j in range(1, 255):
                        ips.append(f"169.254.{i}.{j}")
            return ips
            
        ips = []
        for i in range(1, num_hosts - 1):
            host_num = network + i
            host_ip = f"{(host_num >> 24) & 0xFF}.{(host_num >> 16) & 0xFF}.{(host_num >> 8) & 0xFF}.{host_num & 0xFF}"
            ips.append(host_ip)
        return ips
    except Exception:
        return []

def scan_worker(ip, timeout):
    """Scan worker for a single IP address."""
    if check_ip_alive(ip, timeout) or ping_ip(ip, timeout):
        metadata = check_ouster_api(ip, timeout)
        if metadata:
            return {"ip": ip, "metadata": metadata}
    return None

def main():
    parser = argparse.ArgumentParser(description="Brute-force scan and discover Ouster Lidars on local subnets.")
    parser.add_argument("--subnet", type=str, help="Custom subnet to scan in CIDR notation (e.g. 169.254.1.0/24)")
    parser.add_argument("--timeout", type=float, default=0.5, help="Ping and HTTP timeout in seconds (default: 0.5)")
    parser.add_argument("--threads", type=int, default=150, help="Number of parallel scanning threads (default: 150)")
    parser.add_argument("--all-subnets", action="store_true", help="Scan all global network interfaces instead of prioritizing 169.254.x.x")
    args = parser.parse_args()

    print(f"{BOLD}{BLUE}=== Ouster Lidar Network Discovery ==={END}")
    
    # 1. Detect interfaces
    interfaces = get_local_interfaces()
    if not interfaces:
        print(f"{RED}No global network interfaces detected.{END}")
        return

    # Filter/Select target subnets
    target_subnets = []
    if args.subnet:
        # User specified a custom subnet
        if '/' in args.subnet:
            parts = args.subnet.split('/')
            target_subnets.append({"ip": parts[0], "cidr": int(parts[1]), "broadcast": None})
        else:
            target_subnets.append({"ip": args.subnet, "cidr": 24, "broadcast": None})
    else:
        # Detect subnets
        for iface in interfaces:
            # Prioritize link-local 169.254.x.x and typical local subnets
            if iface["ip"].startswith("169.254.") or args.all_subnets or iface["ip"].startswith("192.168."):
                target_subnets.append(iface)

    if not target_subnets:
        print(f"{YELLOW}No link-local (169.254.x.x) or local (192.168.x.x) subnets found. Use --all-subnets or --subnet to override.{END}")
        target_subnets = interfaces

    print(f"{BOLD}Target subnets to search:{END}")
    for subnet in target_subnets:
        brd_str = f" (broadcast: {subnet['broadcast']})" if subnet.get('broadcast') else ""
        print(f" - Interface: {subnet.get('interface', 'custom')}, Subnet: {subnet['ip']}/{subnet['cidr']}{brd_str}")

    # 2. Trigger broadcast pings to populate ARP tables
    print(f"\n{BLUE}Sending broadcast pings to populate host ARP tables...{END}")
    for subnet in target_subnets:
        if subnet.get("broadcast"):
            broadcast_ping(subnet["broadcast"])

    # 3. Read ARP cache IPs
    arp_ips = get_arp_ips()
    print(f"Detected {len(arp_ips)} active hosts in ARP cache.")

    # 4. Compile candidate IP list
    candidate_ips = set(arp_ips)
    
    # Also add standard default/common link-local addresses
    candidate_ips.add("169.254.1.1")
    candidate_ips.add("169.254.1.200")
    
    for subnet in target_subnets:
        subnet_ips = cidr_to_ips(subnet["ip"], subnet["cidr"])
        print(f"Subnet {subnet['ip']}/{subnet['cidr']} generates {len(subnet_ips)} candidate host IPs.")
        candidate_ips.update(subnet_ips)

    # Exclude loopback/own IPs
    own_ips = {iface["ip"] for iface in interfaces}
    candidate_ips = sorted(list(candidate_ips - own_ips))

    print(f"\n{BLUE}Scanning {len(candidate_ips)} potential IP addresses using {args.threads} parallel threads...{END}")

    found_sensors = []
    
    # Use ThreadPoolExecutor for concurrent ping & HTTP check
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.threads) as executor:
        futures = {executor.submit(scan_worker, ip, args.timeout): ip for ip in candidate_ips}
        for future in concurrent.futures.as_completed(futures):
            res = future.result()
            if res:
                found_sensors.append(res)
                ip = res["ip"]
                meta = res["metadata"]
                print(f"{GREEN}{BOLD}[FOUND] Ouster Lidar active at: {ip}{END}")
                if "system" in meta:
                    print(f"   Status: {meta['system'].get('status', 'N/A')}")
                elif "sensor_info" in meta:
                    info = meta["sensor_info"]
                    print(f"   Model : {info.get('prod_line', 'N/A')}")
                    print(f"   Serial: {info.get('serial_no', 'N/A')}")

    print(f"\n{BOLD}{BLUE}=== Discovery Finished ==={END}")
    if found_sensors:
        print(f"{GREEN}Successfully discovered {len(found_sensors)} Ouster Lidar(s):{END}")
        for s in found_sensors:
            ip = s["ip"]
            meta = s["metadata"]
            if "sensor_info" in meta:
                print(f" - {ip} (Model: {meta['sensor_info'].get('prod_line')}, Serial: {meta['sensor_info'].get('serial_no')})")
            elif "system" in meta:
                print(f" - {ip} (Status: {meta['system'].get('status')})")
            else:
                print(f" - {ip} (Ouster HTTP API active)")
    else:
        print(f"{RED}No Ouster Lidars detected on the network.{END}")
        print("Please check physical connections, link-local configurations, and power status.")

if __name__ == "__main__":
    main()
