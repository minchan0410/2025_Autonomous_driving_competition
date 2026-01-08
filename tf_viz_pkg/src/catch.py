#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import os
from nav_msgs.msg import Path

class PathToCSV:
    def __init__(self):
        rospy.init_node('path_to_csv_node', anonymous=True)
        
        # 설정 변수
        self.topic_name = "/global_path"
        self.output_file = "global_path.csv"
        self.is_saved = False  # 한 번만 저장하기 위한 플래그
        
        # Subscriber 설정
        self.sub = rospy.Subscriber(self.topic_name, Path, self.callback)
        rospy.loginfo(f"{self.topic_name} 토픽을 대기 중입니다...")

    def callback(self, data):
        if not self.is_saved:
            try:
                # 파일 저장 경로 (현재 실행 디렉토리)
                with open(self.output_file, mode='w', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    # 헤더 작성
                    writer.writerow(['seq', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
                    
                    # Path 내의 poses 반복문
                    for i, pose_stamped in enumerate(data.poses):
                        pos = pose_stamped.pose.position
                        ori = pose_stamped.pose.orientation
                        writer.writerow([
                            i, 
                            pos.x, pos.y, pos.z,
                            ori.x, ori.y, ori.z, ori.w
                        ])
                
                rospy.loginfo(f"성공적으로 데이터를 저장했습니다: {os.path.abspath(self.output_file)}")
                self.is_saved = True
                
                # 저장 후 노드 종료를 원한다면 주석 해제
                # rospy.signal_shutdown("Data saved successfully.")
                
            except Exception as e:
                rospy.logerr(f"파일 저장 중 오류 발생: {e}")

if __name__ == '__main__':
    try:
        PathToCSV()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass