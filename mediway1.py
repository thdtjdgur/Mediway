#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mediway1 노드
라즈베리파이 픽셀(u,v) → 미터(X,Y)
RViz Marker / /move_base_simple/goal /clicked_position
"""

import socket
import rospy
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Header

import math
import tf2_ros
import tf2_geometry_msgs

# ───────── 파라미터 & 상수 ─────────
PIX_W, PIX_H = 762.0, 574.0
lu_x, lu_y = 1.0, -1.8
ld_x, ld_y = 3.7, -0.3
ru_x, ru_y = -1.0, 1.7
rd_x, rd_y = 1.5, 3.2

RPI_IP = "10.24.184.1"     # ← 실제 IP
RPI_PORT = 5005            # ← RPi 수신 포트(좌표/신호)
RPI_PORT_SIGNAL = 5006     # ← 999,999 등 신호 분리용(원치 않으면 RPI_PORT와 동일하게 써도 됨)

GOAL_THRESH = 0.35

# 약품 위치(실좌표로 교체 가능)
MED1_GOAL = (1.1, -1.1)
MED2_GOAL = (3.0, -0.1)

# 상태 정의
STATE_IDLE          = "IDLE"
STATE_TO_USER       = "TO_USER"        # 사용자에게 이동 중
STATE_AT_USER       = "AT_USER"        # 사용자 도착(진료실 좌표 대기)
STATE_TO_CLINIC     = "TO_CLINIC"      # 진료실 이동 중 (RPi가 좌표 보냄)
STATE_TO_MED1       = "TO_MED1"        # 약1 이동 중 (RPi 좌표 없음)
STATE_TO_MED2       = "TO_MED2"        # 약2 이동 중 (RPi 좌표 없음)
STATE_WAIT_INPUT    = "WAITING_FOR_INPUT"
STATE_DELIVERING    = "DELIVERING"

# ───────── 전역 변수 ─────────
robot_x, robot_y = 0.0, 0.0
goal_x, goal_y   = 0.0, 0.0
state = STATE_IDLE

# 사용자 도착 신호 1회 송신용
user_signal_sent = False

# 약품 배달 단계(현재 타깃): True면 약1, False면 약2
medicine_target_is_med1 = True

# 약품 투입 중 들어온 사용자 호출 보류 용
pending_user_call = False
pending_u = None
pending_v = None

# 소켓
pos_sock = None  # 로봇 좌표 TX
tx_sock  = None  # 신호 TX

# TF
_tf_buffer = None
_tf_listener = None


def pixel_to_meter(u: float, v: float):
    s = u / PIX_W
    t = v / PIX_H
    X = ((1-s)*(1-t)*lu_x + s*(1-t)*ru_x + (1-s)*t*ld_x + s*t*rd_x)
    Y = ((1-s)*(1-t)*lu_y + s*(1-t)*ru_y + (1-s)*t*ld_y + s*t*rd_y)
    return X, Y


def send_pose_timer_cb(event):
    """0.5초마다 로봇 좌표를 RPi로 전송 (항상 송신: RPi가 필요할 때만 사용)"""
    global robot_x, robot_y, pos_sock
    if pos_sock is None:
        return
    try:
        msg = f"{robot_x:.3f},{robot_y:.3f}".encode()
        pos_sock.sendto(msg, (RPI_IP, RPI_PORT))
    except Exception as e:
        rospy.logwarn(f"[POSE_TX] UDP send failed: {e}")


def odom_callback(msg: Odometry):
    """odom→map 최신 시각 변환 (extrapolation 방지), 콜백 내 sleep 금지"""
    global robot_x, robot_y, _tf_buffer
    odom_pose = msg.pose.pose
    try:
        transform = _tf_buffer.lookup_transform(
            "map",
            msg.header.frame_id,   # 보통 "odom"
            rospy.Time(0),         # 최신
            rospy.Duration(0.2)
        )
    except Exception as ex:
        rospy.logwarn(f"Transform failed: {ex}")
        return

    map_pose_st = tf2_geometry_msgs.do_transform_pose(
        PoseStamped(header=msg.header, pose=odom_pose),
        transform
    )
    robot_x = map_pose_st.pose.position.x
    robot_y = map_pose_st.pose.position.y


def distance_to_goal() -> float:
    return math.hypot(robot_x - goal_x, robot_y - goal_y)


def set_goal(x, y):
    global goal_x, goal_y
    goal_x, goal_y = x, y


def udp_listener():
    global _tf_buffer, _tf_listener
    global pos_sock, tx_sock
    global state, goal_x, goal_y
    global user_signal_sent
    global medicine_target_is_med1
    global pending_user_call, pending_u, pending_v

    rospy.init_node("mediway1")

    point_pub  = rospy.Publisher("/clicked_position", Point, queue_size=10)
    goal_pub   = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    marker_pub = rospy.Publisher("/goal_marker", Marker, queue_size=10)
    servo_pub  = rospy.Publisher("/servo_control", Int32, queue_size=10)

    # TF
    _tf_buffer = tf2_ros.Buffer()
    _tf_listener = tf2_ros.TransformListener(_tf_buffer)
    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)

    # UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5005))
    sock.settimeout(2)  # 2초 안에 오면 got_udp=True (전제 유지)

    tx_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rospy.Timer(rospy.Duration(0.5), send_pose_timer_cb)  # 0.5초 주기 좌표 TX

    rospy.loginfo("UDP 5005 listening (mediway1)")
    rate = rospy.Rate(50)

    # 초기 상태: 약1부터 시작
    set_goal(*MED1_GOAL)
    state = STATE_TO_MED1
    rospy.loginfo("약품1 배달 지점으로 이동중(초기)")

    while not rospy.is_shutdown():
        # 1) RPi 좌표 수신
        got_udp = False
        u = v = None
        try:
            data, _ = sock.recvfrom(1024)
            u_str, v_str = data.decode().strip().split(",")
            u, v = float(u_str), float(v_str)
            got_udp = True
        except socket.timeout:
            pass
        except (ValueError, IndexError):
            rospy.logwarn("잘못된 UDP 데이터 형식 수신")

        # 1.5) 약품 이동 중 사용자 호출 즉시 전환
        if got_udp and state in [STATE_TO_MED1, STATE_TO_MED2]:
            set_goal(*pixel_to_meter(u, v))
            state = STATE_TO_USER
            user_signal_sent = False
            pending_user_call = False
            rospy.loginfo("사용자 호출 수신 → 사용자에게로 즉시 전환")

        # 1.6) 약품 투입 대기/배출 중 호출은 보류
        if got_udp and state in [STATE_WAIT_INPUT, STATE_DELIVERING]:
            pending_user_call = True
            pending_u, pending_v = u, v
            rospy.loginfo("사용자 호출 수신(약품 투입 중) → 종료 후 사용자로 전환 예약")

        # 2) 상태 머신 본 처리
        if state == STATE_TO_MED1:
            if distance_to_goal() < GOAL_THRESH:
                rospy.loginfo("약품1 배달 지점 도착!")
                state = STATE_WAIT_INPUT
                # 정지
                goal_pub.publish(PoseStamped(
                    header=Header(stamp=rospy.Time.now(), frame_id="map"),
                    pose=Pose(position=Point(robot_x, robot_y, 0), orientation=Quaternion(0,0,0,1))
                ))
            else:
                rospy.loginfo("약품1 배달 지점으로 이동중")

        elif state == STATE_TO_MED2:
            if distance_to_goal() < GOAL_THRESH:
                rospy.loginfo("약품2 배달 지점 도착!")
                state = STATE_WAIT_INPUT
                goal_pub.publish(PoseStamped(
                    header=Header(stamp=rospy.Time.now(), frame_id="map"),
                    pose=Pose(position=Point(robot_x, robot_y, 0), orientation=Quaternion(0,0,0,1))
                ))
            else:
                rospy.loginfo("약품2 배달 지점으로 이동중")

        elif state == STATE_WAIT_INPUT:
            # 블로킹 입력(현장에선 서비스/액션 권장)
            try:
                user_input = input("약품 배달 지점 도착. 약품 번호(1-4)를 입력하고 Enter: ")
                servo_num = int(user_input)
                if 1 <= servo_num <= 4:
                    rospy.loginfo(f"{servo_num}번 약품 배출 시작.")
                    servo_pub.publish(servo_num)
                    state = STATE_DELIVERING
                else:
                    print("잘못된 번호입니다. 1~4 사이 숫자를 입력하세요.")
            except ValueError:
                print("숫자를 입력해주세요.")

        elif state == STATE_DELIVERING:
            rospy.loginfo("약품 배출 중... (약 6초간 대기)")
            rospy.sleep(6.0)
            rospy.loginfo("약품 배출 완료.")

            # 배출 중 호출이 들어왔다면 최우선 전환
            if pending_user_call:
                set_goal(*pixel_to_meter(pending_u, pending_v))
                state = STATE_TO_USER
                user_signal_sent = False
                pending_user_call = False
                rospy.loginfo("배출 완료 → 예약된 사용자 호출로 전환")
            else:
                # 다음 약품 지점으로 즉시 이동 시작
                if medicine_target_is_med1:
                    # 방금 약1 처리 → 약2로
                    medicine_target_is_med1 = False
                    set_goal(*MED2_GOAL)
                    state = STATE_TO_MED2
                    rospy.loginfo("약품2 배달 지점으로 이동 시작")
                else:
                    # 방금 약2 처리 → 약1로
                    medicine_target_is_med1 = True
                    set_goal(*MED1_GOAL)
                    state = STATE_TO_MED1
                    rospy.loginfo("약품1 배달 지점으로 이동 시작")

        elif state == STATE_IDLE:
            # 대기 상태: 호출이 오면 0.5초마다 좌표가 온다
            if got_udp:
                set_goal(*pixel_to_meter(u, v))
                state = STATE_TO_USER
                user_signal_sent = False
                rospy.loginfo("사용자에게로 이동 시작")

        elif state == STATE_TO_USER:
            if got_udp:
                set_goal(*pixel_to_meter(u, v))
                rospy.loginfo("사용자에게로 이동중")
            if distance_to_goal() < GOAL_THRESH:
                state = STATE_AT_USER
                rospy.loginfo("사용자에게 도착")
                # 필요 시 사용자 도착 신호 1회 송신
                if not user_signal_sent:
                    try:
                        tx_sock.sendto(b"999,999", (RPI_IP, RPI_PORT_SIGNAL))
                        rospy.loginfo("Sent UDP '999,999' to Raspberry Pi")
                        user_signal_sent = True
                    except Exception as e:
                        rospy.logwarn(f"UDP send failed: {e}")

        elif state == STATE_AT_USER:
            # 진료실 좌표가 들어올 때까지 대기
            if got_udp:
                set_goal(*pixel_to_meter(u, v))
                state = STATE_TO_CLINIC
                rospy.loginfo("진료실로 이동 시작")
            else:
                rospy.loginfo("진료실 좌표 대기중")

        elif state == STATE_TO_CLINIC:
            # 요구사항: 항상 "진료실로 이동중"만 출력
            if got_udp:
                set_goal(*pixel_to_meter(u, v))   # 계속 추종
            else:
                # RPi가 좌표 전송을 멈춤(= 진료실 도착 판단) → 약품1로
                set_goal(*MED1_GOAL)
                state = STATE_TO_MED1
            rospy.loginfo("진료실로 이동중")

        # 3) 네비 목표 발행
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.orientation.w = 1.0
        goal_pub.publish(goal_msg)

        # 4) RViz 시각화
        point_pub.publish(Point(goal_x, goal_y, 0.0))
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "clicked_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.0, 1.0, 1.0
        marker_pub.publish(marker)

        rospy.loginfo(f"STATE({state}) GOAL({goal_x:.2f},{goal_y:.2f})")
        rate.sleep()


if __name__ == "__main__":
    try:
        udp_listener()
    except rospy.ROSInterruptException:
        pass

