#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# by wxc
"""
TCP Slave Bridge
- Connects to master's TCP server; maps TAKEOFF/LAND/GOTO to local executor.
- Publishes local status & waypoint_reached back to master.
- Heartbeat + failsafe (HB timeout -> LAND; command idle -> suggest HOVER).

Params:
  ~master_host: str (master IP)
  ~master_port: int
  ~executor_ns: str
  ~hb_interval: float
  ~hb_timeout: float
  ~cmd_timeout: float
"""

import rospy
import threading
import socket
import struct
import json
import time

from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

# ------------ TCP framing helpers ------------
def pack_msg(d: dict) -> bytes:
    """ Packs a dictionary into a length-prefixed JSON byte string. """
    raw = json.dumps(d).encode('utf-8')
    return struct.pack(">I", len(raw)) + raw

def recv_exact(sock: socket.socket, n: int) -> bytes:
    """ Receives exactly n bytes from a socket. """
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf += chunk
    return buf

def recv_frame(sock: socket.socket) -> dict:
    """ Receives a complete frame and decodes it from JSON. """
    hdr = recv_exact(sock, 4)
    ln = struct.unpack(">I", hdr)[0]
    data = recv_exact(sock, ln)
    return json.loads(data.decode('utf-8'))

# ------------ Slave Bridge Node ------------
class TcpSlaveBridge:
    def __init__(self):
        rospy.init_node("tcp_slave_bridge", anonymous=True)

        # --- Parameters ---
        try:
            self.master_host = rospy.get_param("~master_host")
        except KeyError:
            rospy.logerr("[SLAVE] Critical: ~master_host parameter is not set. Shutting down.")
            rospy.signal_shutdown("Missing required parameter")
            return
            
        self.master_port = int(rospy.get_param("~master_port", 9999))
        self.executor_ns = rospy.get_param("~executor_ns", "iris_1")
        self.hb_interval = float(rospy.get_param("~hb_interval", 0.5))
        self.hb_timeout = float(rospy.get_param("~hb_timeout", 5.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 3.0))

        self.prefix = f"/trajectory_executor_{self.executor_ns}"

        # --- Local executor interfaces ---
        rospy.loginfo(f"[SLAVE] Waiting for executor services at '{self.prefix}'...")
        try:
            rospy.wait_for_service(f"{self.prefix}/takeoff", timeout=10.0)
            rospy.wait_for_service(f"{self.prefix}/land", timeout=10.0)
        except rospy.ROSException:
            rospy.logerr(f"[SLAVE] Executor services not available. Make sure trajectory_executor_{self.executor_ns} is running.")
            rospy.signal_shutdown("Executor services not found")
            return

        self.takeoff_cli = rospy.ServiceProxy(f"{self.prefix}/takeoff", Trigger)
        self.land_cli    = rospy.ServiceProxy(f"{self.prefix}/land", Trigger)
        self.goto_pub    = rospy.Publisher(f"{self.prefix}/goto", PoseStamped, queue_size=1)
        
        # --- Local executor subscribers ---
        self.status_sub = rospy.Subscriber(f"{self.prefix}/status", String, self._status_cb)
        
        # MODIFIED: Logic now relies on /status topic.
        # The executor should publish "IDLE" when a waypoint is reached.
        # This simplifies the executor's logic to only one status topic.

        # --- TCP state ---
        self.sock = None
        self.sock_lock = threading.Lock()
        self.connected = False

        # --- Timers ---
        self.last_hb_rx = time.time()
        self.last_cmd_rx = time.time()

        # --- Threads ---
        self.rx_thread = None
        self.hb_thread = None
        self.conn_thread = threading.Thread(target=self._conn_loop, daemon=True)
        self.conn_thread.start()

        # --- Failsafe watchdog ---
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.5), self._watchdog)

        rospy.loginfo("[SLAVE] tcp_slave_bridge started.")

    # ---- ROS callbacks -> upstream ----
    def _status_cb(self, msg: String):
        """ Forwards the local executor's status to the master and checks for waypoint completion. """
        self._send({"type": "STATUS", "payload": {"state": msg.data}})
        
        # If the executor becomes IDLE, it means it has reached its destination.
        if msg.data == "IDLE":
            self._send({"type": "WP_REACHED"})

    # ---- TCP connection management ----
    def _conn_loop(self):
        """ Main loop for managing TCP connection to the master. """
        while not rospy.is_shutdown():
            try:
                rospy.loginfo(f"[SLAVE] Connecting to master at {self.master_host}:{self.master_port} ...")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(5.0) # Connection timeout
                s.connect((self.master_host, self.master_port))
                s.settimeout(None) # Reset to blocking for operations
                
                with self.sock_lock:
                    self.sock = s
                    self.connected = True
                    # ADDED: Reset timers on successful connection
                    self.last_hb_rx = time.time()
                    self.last_cmd_rx = time.time()

                rospy.loginfo("[SLAVE] Connected to master.")

                # Start RX and HB threads for the new connection
                self.rx_thread = threading.Thread(target=self._rx_loop, args=(s,), daemon=True)
                self.rx_thread.start()
                self.hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
                self.hb_thread.start()

                # This blocks until the rx_thread exits (due to error or disconnect)
                self.rx_thread.join()

            except Exception as e:
                rospy.logwarn(f"[SLAVE] Connection error: {e}")
                # Wait before retrying
                time.sleep(2.0)
            finally:
                with self.sock_lock:
                    if self.sock:
                        try:
                            self.sock.close()
                        except Exception:
                            pass
                    self.sock = None
                    self.connected = False
                rospy.logwarn("[SLAVE] Disconnected. Reconnecting...")

    # ---- TCP communication ----
    def _rx_loop(self, s: socket.socket):
        """ Loop for receiving messages from the master. """
        try:
            while not rospy.is_shutdown():
                msg = recv_frame(s)
                t = msg.get("type")
                
                # Update command timer for any real command
                if t not in ["HEARTBEAT", "ACK"]:
                    self.last_cmd_rx = time.time()

                if t == "HEARTBEAT":
                    self.last_hb_rx = time.time()
                elif t == "TAKEOFF":
                    rospy.loginfo("[SLAVE] Received TAKEOFF command.")
                    try:
                        self.takeoff_cli(TriggerRequest())
                    except Exception as e:
                        rospy.logerr(f"[SLAVE] Takeoff service call failed: {e}")
                elif t == "LAND":
                    rospy.loginfo("[SLAVE] Received LAND command.")
                    try:
                        self.land_cli(TriggerRequest())
                    except Exception as e:
                        rospy.logerr(f"[SLAVE] Land service call failed: {e}")
                elif t == "GOTO":
                    p = msg.get("payload") or {}
                    rospy.loginfo(f"[SLAVE] Received GOTO command: {p}")
                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = float(p.get("x", 0.0))
                    pose.pose.position.y = float(p.get("y", 0.0))
                    pose.pose.position.z = float(p.get("z", 1.0))
                    pose.pose.orientation.w = 1.0
                    self.goto_pub.publish(pose)
                elif t == "ACK":
                    # Acknowledgment from master, no action needed
                    pass
                else:
                    rospy.logwarn(f"[SLAVE] Received unknown message type: {t}")
        except Exception as e:
            rospy.logwarn(f"[SLAVE] RX loop error: {e}. Closing connection.")

    def _hb_loop(self):
        """ Loop for sending heartbeats to the master. """
        while not rospy.is_shutdown() and self.connected:
            self._send({"type": "HEARTBEAT"})
            time.sleep(self.hb_interval)

    def _send(self, d: dict):
        """ Safely sends a dictionary as a JSON message over TCP. """
        with self.sock_lock:
            if not self.sock:
                return
            try:
                self.sock.sendall(pack_msg(d))
            except Exception as e:
                # This will likely be caught by the rx_loop, causing a disconnect
                rospy.logwarn(f"[SLAVE] Send error: {e}")
                # No need to close socket here, rx_loop will handle it
                pass

    # ---- Failsafe ----
    def _watchdog(self, _):
        """ Timer-based watchdog for failsafes. """
        # MODIFIED: Only run watchdog checks if connected
        if not self.connected:
            return

        now = time.time()
        
        # 1. Heartbeat Timeout Failsafe
        if now - self.last_hb_rx > self.hb_timeout:
            rospy.logerr("[SLAVE] HEARTBEAT TIMEOUT! Master connection lost. Triggering failsafe LAND.")
            try:
                self.land_cli(TriggerRequest())
            except Exception as e:
                rospy.logerr(f"[SLAVE] Failsafe LAND service call error: {e}")
            # Reset timer to prevent spamming the land command while trying to reconnect
            self.last_hb_rx = now
        
        # 2. Command Idle Timeout
        elif now - self.last_cmd_rx > self.cmd_timeout:
            # This is a soft warning, suggesting the drone is idle.
            # No action is taken by default, but you could implement an auto-hover here.
            rospy.logwarn_throttle(10, "[SLAVE] No command received recently. Drone is idle.")
            # Reset timer to avoid spamming the log
            self.last_cmd_rx = now

if __name__ == "__main__":
    try:
        node = TcpSlaveBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass