#include <ros/ros.h>  // ROS 기본 기능을 포함하는 헤더 파일
#include <pcl/io/pcd_io.h>  // PCL의 PCD 파일 입출력 기능 포함
#include <pcl/point_types.h>  // PCL의 포인트 타입 정의 포함
#include <pcl_conversions/pcl_conversions.h>  // PCL과 ROS 메시지 변환을 위한 헤더 파일
#include <sensor_msgs/PointCloud2.h>  // ROS의 PointCloud2 메시지 타입 포함

#include "for_cloud.hpp"  // 사용자 정의 포인트 클라우드 관련 헤더 파일 포함
#include "for_desc.hpp"  // 사용자 정의 설명 관련 헤더 파일 포함
#include "for_time.hpp"  // 시간 측정을 위한 사용자 정의 헤더 파일 포함

// 로드한 PCD 맵을 저장할 포인트 클라우드 객체 선언
PointCloudPtr _raw_map(new PointCloud);

int main(int argc, char** argv) {
    ros::init(argc, argv, "load_map");  // ROS 노드 초기화
    ros::NodeHandle nh("~");  // 노드 핸들 생성 (Private 네임스페이스)

    nh.getParam("PARA_DIR_MAP", PARA_DIR_MAP);  // 파라미터 서버에서 맵 디렉토리 경로 가져오기

    // "/raw_map" 토픽에 PointCloud2 메시지를 발행하는 ROS 퍼블리셔 생성
    ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/raw_map", 1);
    
    // 맵 로딩 시간을 측정하기 위한 타이머 시작
    TicToc timer_map;

    // PCD 파일 로드
    if (pcl::io::loadPCDFile(PARA_DIR_MAP + "raw_map.pcd", *_raw_map)) {
        ROS_ERROR("Failed to load [[ Raw Map ]] from %s", (PARA_DIR_MAP + "raw_map.pcd").c_str());  // 로드 실패 시 오류 메시지 출력
        return -1;  // 프로그램 종료
    } 

    // 이상치 제거 (현재 주석 처리됨)
    // removeOutliersSOR(_raw_map, _raw_map, 50, 2.0);

    // 맵 로딩 완료 메시지 출력
    std::cout << "==================================================\n";
    std::cout << "||                                              ||\n";
    std::cout << "||       Point Cloud Map Loaded Successfully.   ||\n";
    std::cout << "||            Ready for LiDAR operation         ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "==================================================\n";

    // 로드된 포인트 클라우드 개수 및 로딩 시간 출력
    std::cout << "Loaded raw map contains: " << _raw_map->size() << " pts..\n";
    std::cout << "Loading map takes: " << timer_map.toc() << "ms\n\n";

    // ROS 루프 주기를 설정 (0.01Hz = 100초마다 한 번씩 발행)
    ros::Rate loop_rate(0.01);
    
    while (ros::ok()) {  // ROS가 실행 중일 동안 반복
        sensor_msgs::PointCloud2 map_msg;  // ROS 메시지 타입의 포인트 클라우드 객체 생성
        pcl::toROSMsg(*_raw_map, map_msg);  // PCL 데이터를 ROS 메시지로 변환
        map_msg.header.stamp = ros::Time::now();  // 현재 시간 설정
        map_msg.header.frame_id = "map";  // 좌표 프레임 ID를 "map"으로 설정

        pub_map.publish(map_msg);  // 변환된 포인트 클라우드를 퍼블리싱
        loop_rate.sleep();  // 설정된 주기에 맞춰 대기
    }

    return 0;  // 프로그램 종료
}
