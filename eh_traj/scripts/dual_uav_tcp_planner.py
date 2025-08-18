#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# by wxc (Refactored for clarity and flexibility)
"""
Dual UAV TCP Planner (Master) v2.0
- Opens a TCP server for the slave bridge.
- Commands local executor (via ROS topics) and remote slave (via TCP).
- Namespaces and waypoint files are now passed as ROS params for better decoupling.
- This node is agnostic of the slave's actual namespace (e.g., 'iris_1'). It only
  knows its own namespace ('self_ns') and a logical slave name ('slave_ns')
  used for fetching waypoints.
"""

import rospy
import socket
import threading
import time
import json
import struct

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
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

# ------------ Master node ------------
class DualUAVTCPPlanner:
    def __init__(self):
        rospy.init_node("dual_uav_tcp_planner")

        # --- Parameters are now cleaner, passed from launch file ---
        # Note: self_ns is used to build topic names for the local executor.
        # slave_ns is ONLY used as a key to look up waypoints in the YAML file.
        self.ns_self = rospy.get_param("~self_ns", "iris_0")
        self.ns_slave = rospy.get_param("~slave_ns", "iris_1")
        
        self.bind_host = rospy.get_param("~bind_host", "0.0.0.0")
        self.bind_port = int(rospy.get_param("~bind_port", 9999))
        self.hb_interval = float(rospy.get_param("~hb_interval", 0.5))
        self.wait_slave_timeout = float(rospy.get_param("~wait_slave_timeout", 10.0))
        self.frame_id = rospy.get_param("~target_frame_id", "map")

        # --- Waypoints (loaded via rosparam in launch file) ---
        self.traj_a, self.traj_b = [], []
        waypoints_ok = False
        try:
            # The entire YAML file is loaded under the private namespace of this node.
            wp_dict = rospy.get_param("~waypoints", {})
            self.traj_a = wp_dict.get(self.ns_self, [])
            self.traj_b = wp_dict.get(self.ns_slave, [])
            if self.traj_a and self.traj_b:
                waypoints_ok = True
                rospy.loginfo("[MASTER] Loaded %d and %d waypoints for %s / %s.",
                              len(self.traj_a), len(self.traj_b), self.ns_self, self.ns_slave)
        except Exception as e:
            rospy.logerr(f"[MASTER] Failed to load waypoints: {e}")

        if not waypoints_ok:
            rospy.logwarn("[MASTER] No valid waypoints found. Ensure YAML is loaded correctly into private param '~waypoints'.")
        if waypoints_ok and len(self.traj_a) != len(self.traj_b):
            rospy.logwarn("[MASTER] Waypoint length mismatch; will use the shorter length.")

        self.num_wp = min(len(self.traj_a), len(self.traj_b))

        # --- Local executor interfaces (topic construction is the ONLY place ns_self is used) ---
        self.exec_prefix_self = f"/trajectory_executor_{self.ns_self}"
        self.self_takeoff_cli = rospy.ServiceProxy(f"{self.exec_prefix_self}/takeoff", Trigger)
        self.self_land_cli    = rospy.ServiceProxy(f"{self.exec_prefix_self}/land",    Trigger)
        self.self_goto_pub    = rospy.Publisher(f"{self.exec_prefix_self}/goto", PoseStamped, queue_size=1)
        self.self_status_sub  = rospy.Subscriber(f"{self.exec_prefix_self}/status", String, self._self_status_cb)

        # --- ROS service to start the mission ---
        rospy.Service("~start_full_mission", Trigger, self.start_mission_cb)

        # --- TCP server state ---
        self.server_sock = None
        self.client_sock = None
        self.client_lock = threading.Lock()
        self.client_connected = False

        # --- Slave state tracking ---
        self.slave_state = "UNKNOWN"
        self.slave_state_lock = threading.Lock()
        
        # --- Synchronization events ---
        self.self_wp_reached_event = threading.Event()
        self.slave_wp_reached_event = threading.Event()

        # --- Threads ---
        self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.server_thread.start()
        self.hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self.hb_thread.start()

        rospy.loginfo(f"[MASTER] Dual UAV TCP Planner ready at {self.bind_host}:{self.bind_port}")
        rospy.loginfo(f"[MASTER] Controlling local UAV '{self.ns_self}' and waiting for slave '{self.ns_slave}'.")


    def _self_status_cb(self, msg: String):
        if msg.data in ["HOVERING", "IDLE"]:
            rospy.loginfo(f"[MASTER] SELF task segment complete (status: {msg.data}).")
            self.self_wp_reached_event.set()

    def _server_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.bind_host, self.bind_port))
        s.listen(1)
        self.server_sock = s
        rospy.loginfo(f"[MASTER] TCP server listening on {self.bind_host}:{self.bind_port}")

        while not rospy.is_shutdown():
            try:
                conn, addr = s.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                rospy.loginfo(f"[MASTER] Slave connected from {addr}")
                with self.client_lock:
                    if self.client_sock:
                        try: self.client_sock.close()
                        except Exception: pass
                    self.client_sock = conn
                    self.client_connected = True
                rx = threading.Thread(target=self._rx_loop, args=(conn,), daemon=True)
                rx.start()
                rx.join()
            except Exception as e:
                if not rospy.is_shutdown():
                    rospy.logwarn(f"[MASTER] Server accept/loop error: {e}")
            finally:
                with self.client_lock:
                    self.client_connected = False
                    if self.client_sock:
                        try: self.client_sock.close()
                        except Exception: pass
                    self.client_sock = None
                rospy.logwarn("[MASTER] Slave disconnected. Waiting for new connection...")

    def _rx_loop(self, conn: socket.socket):
        try:
            while not rospy.is_shutdown():
                msg = recv_frame(conn)
                t = msg.get("type")
                if t == "HEARTBEAT":
                    pass # Heartbeats are mainly for RX timeout on the slave side
                elif t == "STATUS":
                    with self.slave_state_lock:
                        self.slave_state = msg.get("payload", {}).get("state", "UNKNOWN")
                    rospy.loginfo(f"[MASTER] SLAVE STATUS updated: {self.slave_state}")
                elif t == "WP_REACHED":
                    rospy.loginfo("[MASTER] SLAVE waypoint reached signal received.")
                    self.slave_wp_reached_event.set()
                else:
                    rospy.logwarn(f"[MASTER] Unknown message from slave: {msg}")
        except ConnectionError:
            rospy.logwarn("[MASTER] RX loop: Connection closed by slave.")
        except Exception as e:
            rospy.logwarn(f"[MASTER] RX loop error: {e}")

    def _hb_loop(self):
        rate = rospy.Rate(1.0 / self.hb_interval)
        while not rospy.is_shutdown():
            self._send({"type": "HEARTBEAT"})
            rate.sleep()

    def _send(self, d: dict):
        with self.client_lock:
            if not self.client_connected or not self.client_sock:
                return
            try:
                self.client_sock.sendall(pack_msg(d))
            except Exception as e:
                rospy.logwarn(f"[MASTER] TCP send error: {e}")

    def start_mission_cb(self, _req):
        if self.num_wp == 0:
            return TriggerResponse(success=False, message="No waypoints loaded.")
        
        if not self.client_connected:
            return TriggerResponse(success=False, message="Slave is not connected.")

        rospy.loginfo(f"[MASTER] Mission started with {self.num_wp} waypoints.")
        
        # Abort helper
        def abort_mission(reason):
            rospy.logerr(f"[MASTER] MISSION ABORTED: {reason}")
            try: self.self_land_cli(TriggerRequest())
            except: pass
            self._send({"type": "LAND"})

        # --- Takeoff Phase ---
        rospy.loginfo("[MASTER] Commanding both UAVs to take off.")
        try:
            resp = self.self_takeoff_cli(TriggerRequest())
            if not resp.success:
                return TriggerResponse(success=False, message=f"Self takeoff failed: {resp.message}")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Self takeoff exception: {e}")
        
        self._send({"type": "TAKEOFF"})

        # --- Wait for Stabilization ---
        self.self_wp_reached_event.clear()
        rospy.loginfo("[MASTER] Waiting for SELF to stabilize...")
        if not self.self_wp_reached_event.wait(timeout=20.0):
            abort_mission("SELF takeoff timeout.")
            return TriggerResponse(success=False, message="Self takeoff timeout.")
        
        rospy.loginfo("[MASTER] Waiting for SLAVE to stabilize...")
        t_start = time.time()
        slave_stabilized = False
        while time.time() - t_start < 20.0:
            with self.slave_state_lock:
                if self.slave_state == "HOVERING":
                    slave_stabilized = True
                    break
            time.sleep(0.2)

        if not slave_stabilized:
            abort_mission("SLAVE takeoff timeout.")
            return TriggerResponse(success=False, message="Slave takeoff timeout.")

        rospy.loginfo("[MASTER] Both UAVs are stable. Starting mission waypoints.")
        time.sleep(1.0) 

        # --- Waypoint Iteration ---
        for i in range(self.num_wp):
            if rospy.is_shutdown(): break
            a, b = self.traj_a[i], self.traj_b[i]
            rospy.loginfo(f"[MASTER] Waypoint {i+1}/{self.num_wp}: self={a}, slave={b}")

            self.self_wp_reached_event.clear()
            self.slave_wp_reached_event.clear()

            # Command self
            pose_a = PoseStamped()
            pose_a.header.stamp = rospy.Time.now()
            pose_a.header.frame_id = self.frame_id
            pose_a.pose.position.x, pose_a.pose.position.y, pose_a.pose.position.z = a
            pose_a.pose.orientation.w = 1.0
            self.self_goto_pub.publish(pose_a)

            # Command slave
            self._send({"type": "GOTO", "payload": {"x": b[0], "y": b[1], "z": b[2]}})

            # Wait for both
            rospy.loginfo(f"[MASTER] Waiting for both to reach waypoint {i+1}...")
            self_reached = self.self_wp_reached_event.wait(timeout=60.0)
            slave_reached = self.slave_wp_reached_event.wait(timeout=60.0)

            if not (self_reached and slave_reached):
                reason = f"Timeout at waypoint {i+1}. "
                if not self_reached: reason += "SELF did not reach. "
                if not slave_reached: reason += "SLAVE did not reach."
                abort_mission(reason)
                return TriggerResponse(success=False, message=reason)
            
            rospy.loginfo(f"[MASTER] Both UAVs reached waypoint {i+1}.")
        
        # --- Landing Phase ---
        rospy.loginfo("[MASTER] Mission sequence finished. Commanding LAND.")
        try: self.self_land_cli(TriggerRequest())
        except: pass
        self._send({"type": "LAND"})

        return TriggerResponse(success=True, message="Mission completed successfully")

if __name__ == "__main__":
    try:
        node = DualUAVTCPPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass