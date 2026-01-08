#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# ==========================================
# [설정] 시작점, 끝점, 점 사이의 간격(m) 설정
# ==========================================
START_POINT = [302565.1459025361, 4123624.3132294514]   # (x, y)
END_POINT   = [302544.48577669484, 4123613.4277715874] # (x, y)
INTERVAL    = 0.1          # 점과 점 사이의 간격 (미터 단위)

def create_linear_path():
    rospy.init_node('linear_path_publisher', anonymous=True)
    # latch=True: 새로운 구독자가 오면 마지막 메시지를 즉시 보내줌
    path_pub = rospy.Publisher('/global_path', Path, queue_size=10, latch=True)

    path_msg = Path()
    path_msg.header.frame_id = "map"
    
    path_x = []
    path_y = []

    rospy.loginfo(f"경로 생성 시작: {START_POINT} -> {END_POINT} (간격: {INTERVAL}m)")

    try:
        # 1. 두 점 사이의 거리 계산
        dx = END_POINT[0] - START_POINT[0]
        dy = END_POINT[1] - START_POINT[1]
        distance = np.hypot(dx, dy)

        # 2. 생성할 점의 개수 계산
        if distance == 0:
            num_points = 1
        else:
            num_points = int(distance / INTERVAL) + 1

        # 3. 등간격 좌표 생성 (Numpy linspace 사용)
        # linspace: 시작점부터 끝점까지 num_points만큼 등분하여 배열 생성
        path_x = np.linspace(START_POINT[0], END_POINT[0], num_points)
        path_y = np.linspace(START_POINT[1], END_POINT[1], num_points)

        # 4. Path 메시지 생성 (기존 구조 유지)
        for idx in range(len(path_x)):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            
            pose.pose.position.x = path_x[idx]
            pose.pose.position.y = path_y[idx]
            pose.pose.position.z = 0.0
            
            # 방향(Orientation)은 기본값 (0,0,0,1) - 필요시 atan2로 계산 가능
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)

        rospy.loginfo(f"총 {len(path_msg.poses)}개의 포인트를 생성했습니다.")

        # 5. 주기적 발행 (while 루프)
        while not rospy.is_shutdown():
            path_msg.header.stamp = rospy.Time.now() # 시간 갱신
            path_pub.publish(path_msg)
            time.sleep(0.5)

    except Exception as e:
        rospy.logerr(f"오류 발생: {e}")

if __name__ == '__main__':
    try:
        create_linear_path()
    except rospy.ROSInterruptException:
        pass