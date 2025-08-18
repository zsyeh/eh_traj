#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# by wxc
"""
Dual UAV TCP Planner (Master) v1.2
- Opens a TCP server for the slave bridge.
- Commands local executor (trajectory_executor_<ns_self>) and remote slave via TCP.
- Reads paired waypoints from private param ~waypoints.
- FIX v1.2: Added explicit wait for SLAVE to be HOVERING before starting mission.
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

# ------------ TCP framing helpers ------------
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

        # --- params ---
        self.ns_self = rospy.get_param("~ns_self", "iris_0")
        self.ns_slave = rospy.get_param("~ns_slave", "iris_1")
        self.bind_host = rospy.get_param("~bind_host", "0.0.0.0")
        self.bind_port = int(rospy.get_param("~bind_port", 9999))
        self.hb_interval = float(rospy.get_param("~hb_interval", 0.5))
        self.wait_slave_timeout = float(rospy.get_param("~wait_slave_timeout", 10.0))

        # --- Waypoints ---
        self.traj_a, self.traj_b = [], []
        waypoints_ok = False
        try:
            wp_dict = rospy.get_param("~waypoints")
            if isinstance(wp_dict, dict) and "waypoints" in wp_dict:
                wp_dict = wp_dict["waypoints"]
            self.traj_a = wp_dict.get(self.ns_self, [])
            self.traj_b = wp_dict.get(self.ns_slave, [])
            waypoints_ok = True
            rospy.loginfo("[MASTER] Loaded %d and %d waypoints for %s / %s.",
                          len(self.traj_a), len(self.traj_b), self.ns_self, self.ns_slave)
        except Exception as e:
            rospy.logwarn(f"[MASTER] Failed to load ~waypoints: {e}")

        if not self.traj_a or not self.traj_b:
            rospy.logwarn("[MASTER] No waypoints found. Load YAML to this node's private param ~waypoints.")
        if waypoints_ok and len(self.traj_a) != len(self.traj_b):
            rospy.logwarn("[MASTER] Waypoint length mismatch; will use the shorter length.")

        self.num_wp = min(len(self.traj_a), len(self.traj_b))

        # --- local executor interfaces ---
        self.exec_prefix_self = f"/trajectory_executor_{self.ns_self}"
        self.self_takeoff_cli = rospy.ServiceProxy(f"{self.exec_prefix_self}/takeoff", Trigger)
        self.self_land_cli    = rospy.ServiceProxy(f"{self.exec_prefix_self}/land",    Trigger)
        self.self_goto_pub    = rospy.Publisher(f"{self.exec_prefix_self}/goto", PoseStamped, queue_size=1)
        self.self_status_sub  = rospy.Subscriber(f"{self.exec_prefix_self}/status", String, self._self_status_cb)

        # --- ROS service ---
        rospy.Service("~start_full_mission", Trigger, self.start_mission_cb)

        # --- TCP server state ---
        self.server_sock = None
        self.client_sock = None
        self.client_lock = threading.Lock()
        self.client_connected = False

        # --- ADDED: Slave state tracking ---
        self.slave_state = None
        self.slave_state_lock = threading.Lock()
        
        # --- Synchronization events ---
        self.self_wp_reached_event = threading.Event()
        self.slave_wp_reached_event = threading.Event()

        # --- Threads ---
        self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.server_thread.start()
        self.hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self.hb_thread.start()

        rospy.loginfo(f"[MASTER] dual_uav_tcp_planner ready at {self.bind_host}:{self.bind_port}")

    def _self_status_cb(self, msg: String):
        # 起飞完成后，状态变为HOVERING
        # GOTO航点到达后，状态变为空中IDLE
        # 这两个状态都代表一个任务段的完成，可以触发事件
        if msg.data == "HOVERING" or msg.data == "IDLE":
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
                rospy.logwarn(f"[MASTER] server accept/loop error: {e}")
            finally:
                with self.client_lock:
                    if self.client_sock:
                        try: self.client_sock.close()
                        except Exception: pass
                    self.client_sock = None
                    self.client_connected = False
                rospy.logwarn("[MASTER] Slave disconnected, waiting for new connection...")

    def _rx_loop(self, conn: socket.socket):
        try:
            while not rospy.is_shutdown():
                msg = recv_frame(conn)
                t = msg.get("type")
                if t == "HEARTBEAT":
                    self._send({"type": "ACK"})
                elif t == "STATUS":
                    payload = msg.get("payload", {})
                    state = payload.get("state")
                    rospy.loginfo(f"[MASTER] SLAVE STATUS: {state}")
                    # --- MODIFIED: Store slave state ---
                    with self.slave_state_lock:
                        self.slave_state = state
                elif t == "WP_REACHED":
                    rospy.loginfo("[MASTER] SLAVE waypoint reached.")
                    self.slave_wp_reached_event.set()
                elif t == "ACK":
                    pass
                else:
                    rospy.logwarn(f"[MASTER] Unknown message from slave: {msg}")
        except Exception as e:
            rospy.logwarn(f"[MASTER] rx error: {e}")

    def _hb_loop(self):
        rate_hz = 1.0 / self.hb_interval if self.hb_interval > 1e-6 else 2.0
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            self._send({"type": "HEARTBEAT"})
            rate.sleep()

    def _send(self, d: dict):
        with self.client_lock:
            if not self.client_sock:
                return
            try:
                self.client_sock.sendall(pack_msg(d))
            except Exception as e:
                rospy.logwarn(f"[MASTER] send error: {e}")

    def start_mission_cb(self, _req):
        if self.num_wp == 0:
            rospy.logwarn(f"[MASTER] ~waypoints missing or empty under node: {rospy.get_name()}")
            return TriggerResponse(success=False, message="No waypoints loaded.")

        t0 = time.time()
        while not rospy.is_shutdown() and not self.client_connected and (time.time() - t0) < self.wait_slave_timeout:
            rospy.loginfo("[MASTER] Waiting for slave TCP connection...")
            time.sleep(0.5)
        if not self.client_connected:
            return TriggerResponse(success=False, message="Slave not connected to TCP server.")

        rospy.loginfo(f"[MASTER] Mission started with {self.num_wp} waypoints.")
        
        # --- MODIFIED: Centralized landing logic in case of failure ---
        def abort_mission(self_msg, slave_msg=""):
            rospy.logerr(f"[MASTER] ABORTING MISSION. Reason: {self_msg}")
            if slave_msg:
                 self._send({"type": "STATUS", "payload": {"state": slave_msg}})
            try:
                self.self_land_cli(TriggerRequest())
            except Exception as e:
                rospy.logerr(f"[MASTER] Self land exception during abort: {e}")
            self._send({"type": "LAND"})
        
        # --- Takeoff Phase with Synchronization ---
        try:
            resp = self.self_takeoff_cli(TriggerRequest())
            if not resp.success:
                return TriggerResponse(success=False, message=f"Self takeoff failed: {resp.message}")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Self takeoff exception: {e}")

        self._send({"type": "TAKEOFF"})
        rospy.loginfo("[MASTER] TAKEOFF sent to both UAVs. Holding for stabilization...")
        
        # 1. Wait for SELF to stabilize
        self.self_wp_reached_event.clear()
        rospy.loginfo("[MASTER] Waiting for SELF to stabilize after takeoff...")
        takeoff_stabilized_self = self.self_wp_reached_event.wait(timeout=15.0) 
        if not takeoff_stabilized_self:
            abort_mission("SELF takeoff did not stabilize in time.")
            return TriggerResponse(success=False, message="Self takeoff timeout.")
        
        # --- ADDED: Wait for SLAVE to stabilize ---
        rospy.loginfo("[MASTER] Waiting for SLAVE to stabilize after takeoff...")
        t0_slave = time.time()
        takeoff_stabilized_slave = False
        while not rospy.is_shutdown() and (time.time() - t0_slave) < 15.0:
            with self.slave_state_lock:
                current_slave_state = self.slave_state
            if current_slave_state == "HOVERING":
                rospy.loginfo("[MASTER] SLAVE stabilized in HOVERING state.")
                takeoff_stabilized_slave = True
                break
            time.sleep(0.5)

        if not takeoff_stabilized_slave:
            abort_mission("SLAVE takeoff did not stabilize in time.")
            return TriggerResponse(success=False, message="Slave takeoff timeout.")

        rospy.loginfo("[MASTER] Both UAVs are stable. Starting mission waypoints.")
        time.sleep(1.0) 

        # --- Waypoint Iteration Phase ---
        for i in range(self.num_wp):
            if rospy.is_shutdown(): break

            a = self.traj_a[i]
            b = self.traj_b[i]
            
            rospy.loginfo(f"[MASTER] Sending waypoint {i+1}/{self.num_wp}: self={a}, slave={b}")

            self.self_wp_reached_event.clear()
            self.slave_wp_reached_event.clear()

            pose_a = PoseStamped()
            pose_a.header.stamp = rospy.Time.now()
            pose_a.header.frame_id = "map"
            pose_a.pose.position.x = float(a[0])
            pose_a.pose.position.y = float(a[1])
            pose_a.pose.position.z = float(a[2])
            pose_a.pose.orientation.w = 1.0
            self.self_goto_pub.publish(pose_a)

            self._send({
                "type": "GOTO",
                "payload": {"x": float(b[0]), "y": float(b[1]), "z": float(b[2])}
            })

            rospy.loginfo(f"[MASTER] Waiting for both UAVs to reach waypoint {i+1}...")
            
            wait_timeout = 60.0 
            
            self_reached = self.self_wp_reached_event.wait(timeout=wait_timeout)
            slave_reached = self.slave_wp_reached_event.wait(timeout=wait_timeout)

            if not (self_reached and slave_reached):
                err_msg = f"Timeout at waypoint {i+1}. "
                if not self_reached: err_msg += "SELF did not reach. "
                if not slave_reached: err_msg += "SLAVE did not reach. "
                abort_mission(err_msg)
                return TriggerResponse(success=False, message=err_msg)
            
            rospy.loginfo(f"[MASTER] Both UAVs reached waypoint {i+1}. Proceeding to next.")
        
        # --- Landing Phase ---
        rospy.loginfo("[MASTER] Mission sequence finished. Sending LAND command.")
        try:
            self.self_land_cli(TriggerRequest())
        except Exception as e:
            rospy.logerr(f"[MASTER] self land exception: {e}")
        self._send({"type": "LAND"})
        rospy.loginfo("[MASTER] LAND sent to slave.")

        return TriggerResponse(success=True, message="Mission completed")

if __name__ == "__main__":
    node = DualUAVTCPPlanner()
    rospy.spin()