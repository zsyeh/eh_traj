#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# by wxc (Refactored for clarity and flexibility)
"""
TCP Slave Bridge v2.0
- Connects to master's TCP server and translates commands for a local executor.
- Forwards local executor status back to the master.
- master_host and executor_ns are now passed as ROS params.
"""

import rospy
import threading
import socket
import struct
import json
import time

from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# ------------ TCP framing helpers (unchanged) ------------
def pack_msg(d: dict) -> bytes:
    raw = json.dumps(d).encode('utf-8')
    return struct.pack(">I", len(raw)) + raw

def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf += chunk
    return buf

def recv_frame(sock: socket.socket) -> dict:
    hdr = recv_exact(sock, 4)
    ln = struct.unpack(">I", hdr)[0]
    data = recv_exact(sock, ln)
    return json.loads(data.decode('utf-8'))

# ------------ Slave Bridge Node ------------
class TCPSlaveBridge:
    def __init__(self):
        rospy.init_node("tcp_slave_bridge", anonymous=True)

        # --- Parameters ---
        self.master_host = rospy.get_param("~master_host", "localhost")
        self.master_port = int(rospy.get_param("~master_port", 9999))
        self.executor_ns = rospy.get_param("~executor_ns", "iris_1")
        self.hb_interval = float(rospy.get_param("~hb_interval", 0.5))
        self.hb_timeout = float(rospy.get_param("~hb_timeout", 5.0))
        self.frame_id = rospy.get_param("~target_frame_id", "map")

        # --- Local executor interfaces (the ONLY place executor_ns is used) ---
        self.prefix = f"/trajectory_executor_{self.executor_ns}"
        rospy.loginfo(f"[SLAVE] Waiting for executor services at '{self.prefix}'...")
        try:
            rospy.wait_for_service(f"{self.prefix}/takeoff", timeout=15.0)
            rospy.wait_for_service(f"{self.prefix}/land", timeout=15.0)
        except rospy.ROSException:
            rospy.logerr(f"[SLAVE] Executor services not found. Is 'trajectory_executor_{self.executor_ns}' running?")
            rospy.signal_shutdown("Executor services not found")
            return

        self.takeoff_cli = rospy.ServiceProxy(f"{self.prefix}/takeoff", Trigger)
        self.land_cli    = rospy.ServiceProxy(f"{self.prefix}/land", Trigger)
        self.goto_pub    = rospy.Publisher(f"{self.prefix}/goto", PoseStamped, queue_size=1)
        self.status_sub  = rospy.Subscriber(f"{self.prefix}/status", String, self._status_cb)

        # --- TCP state ---
        self.sock = None
        self.sock_lock = threading.Lock()
        self.connected = False
        self.last_hb_rx = 0

        # --- Threads ---
        self.conn_thread = threading.Thread(target=self._conn_loop, daemon=True)
        self.conn_thread.start()
        self.watchdog_timer = rospy.Timer(rospy.Duration(1.0), self._watchdog)

        rospy.loginfo(f"[SLAVE] TCP Slave Bridge started. Will connect to {self.master_host}:{self.master_port}.")
        rospy.loginfo(f"[SLAVE] Bridging to executor with namespace '{self.executor_ns}'.")

    def _status_cb(self, msg: String):
        # Forward status to master
        self._send({"type": "STATUS", "payload": {"state": msg.data}})
        
        # When executor finishes a task (takeoff or goto), it enters HOVERING or IDLE
        if msg.data in ["HOVERING", "IDLE"]:
            rospy.loginfo(f"[SLAVE] Local executor reported task segment complete (status: {msg.data}). Notifying master.")
            self._send({"type": "WP_REACHED"})

    def _conn_loop(self):
        while not rospy.is_shutdown():
            try:
                rospy.loginfo(f"[SLAVE] Attempting to connect to master at {self.master_host}:{self.master_port}...")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(5.0)
                s.connect((self.master_host, self.master_port))
                s.settimeout(None)
                
                with self.sock_lock:
                    self.sock = s
                    self.connected = True
                    self.last_hb_rx = time.time() # Reset failsafe timer

                rospy.loginfo("[SLAVE] Connected to master.")

                rx_thread = threading.Thread(target=self._rx_loop, args=(s,), daemon=True)
                hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
                rx_thread.start()
                hb_thread.start()
                rx_thread.join()

            except Exception as e:
                rospy.logwarn(f"[SLAVE] Connection failed: {e}")
            finally:
                with self.sock_lock:
                    self.connected = False
                    if self.sock:
                        try: self.sock.close()
                        except: pass
                    self.sock = None
                rospy.logwarn("[SLAVE] Disconnected. Will retry in 3 seconds...")
                time.sleep(3.0)

    def _rx_loop(self, s: socket.socket):
        try:
            while not rospy.is_shutdown():
                msg = recv_frame(s)
                self.last_hb_rx = time.time() # Any message from master counts as a heartbeat
                
                t = msg.get("type")
                if t == "TAKEOFF":
                    rospy.loginfo("[SLAVE] Received TAKEOFF command.")
                    try: self.takeoff_cli(TriggerRequest())
                    except: rospy.logerr("[SLAVE] Takeoff service call failed.")
                elif t == "LAND":
                    rospy.loginfo("[SLAVE] Received LAND command.")
                    try: self.land_cli(TriggerRequest())
                    except: rospy.logerr("[SLAVE] Land service call failed.")
                elif t == "GOTO":
                    p = msg.get("payload", {})
                    rospy.loginfo(f"[SLAVE] Received GOTO: {p}")
                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = self.frame_id
                    pose.pose.position.x = p.get("x", 0.0)
                    pose.pose.position.y = p.get("y", 0.0)
                    pose.pose.position.z = p.get("z", 1.0)
                    pose.pose.orientation.w = 1.0
                    self.goto_pub.publish(pose)
                elif t == "HEARTBEAT":
                    pass # Master sends, we just receive to keep connection alive
                else:
                    rospy.logwarn(f"[SLAVE] Unknown command type received: {t}")

        except ConnectionError:
            rospy.logwarn("[SLAVE] RX loop: Connection closed by master.")
        except Exception as e:
            rospy.logwarn(f"[SLAVE] RX loop error: {e}")

    def _hb_loop(self):
        rate = rospy.Rate(1.0 / self.hb_interval)
        while not rospy.is_shutdown() and self.connected:
            self._send({"type": "HEARTBEAT"})
            rate.sleep()

    def _send(self, d: dict):
        with self.sock_lock:
            if not self.connected or not self.sock:
                return
            try:
                self.sock.sendall(pack_msg(d))
            except Exception as e:
                rospy.logwarn(f"[SLAVE] TCP send error: {e}")

    def _watchdog(self, _event):
        if not self.connected:
            return
        if time.time() - self.last_hb_rx > self.hb_timeout:
            rospy.logerr("[SLAVE] Master heartbeat timeout! Connection lost. Commanding LAND as failsafe.")
            try: self.land_cli(TriggerRequest())
            except: rospy.logerr("[SLAVE] Failsafe LAND call failed.")
            # The conn_loop will handle reconnection. We just land.
            # Close socket to force reconnection loop
            with self.sock_lock:
                if self.sock:
                    try: self.sock.close()
                    except: pass
                self.connected = False


if __name__ == "__main__":
    try:
        node = TCPSlaveBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass