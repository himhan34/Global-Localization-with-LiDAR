#include <iostream>  // 표준 입출력을 위한 헤더 파일
#include <fstream>   // 파일 입출력을 위한 헤더 파일
#include <iterator>  // 반복자 사용을 위한 헤더 파일
#include <string>    // 문자열 처리를 위한 헤더 파일
#include <vector>    // 벡터 컨테이너 사용을 위한 헤더 파일
#include <image_transport/image_transport.h>  // ROS 이미지 전송을 위한 헤더 파일
#include <nav_msgs/Odometry.h>  // ROS의 오도메트리 메시지를 사용하기 위한 헤더 파일
#include <nav_msgs/Path.h>  // ROS의 경로 메시지를 사용하기 위한 헤더 파일
#include <ros/ros.h>  // ROS 기본 기능을 사용하기 위한 헤더 파일
#include <rosbag/bag.h>  // ROSBAG 파일을 다루기 위한 헤더 파일
#include <geometry_msgs/PoseStamped.h>  // ROS의 PoseStamped 메시지를 사용하기 위한 헤더 파일
#include <sensor_msgs/image_encodings.h>  // 이미지 인코딩 관련 헤더 파일
#include <eigen3/Eigen/Dense>  // 선형대수 연산을 위한 Eigen 라이브러리
#include <pcl/point_cloud.h>  // PCL(Point Cloud Library)의 포인트 클라우드 기본 헤더 파일
#include <pcl/point_types.h>  // PCL의 포인트 타입 정의 헤더 파일
#include <pcl_conversions/pcl_conversions.h>  // PCL과 ROS 메시지 간 변환을 위한 헤더 파일
#include <sensor_msgs/PointCloud2.h>  // ROS의 포인트 클라우드 메시지를 사용하기 위한 헤더 파일
#include <pcl/io/pcd_io.h>  // PCL에서 PCD 파일을 입출력하기 위한 헤더 파일
#include <signal.h>  // 시그널 처리를 위한 헤더 파일
#include <algorithm>  // 알고리즘 라이브러리 (std::shuffle 등 사용)
#include <random>  // 난수 생성을 위한 헤더 파일
#include <queue>  // 큐 자료구조를 사용하기 위한 헤더 파일

bool g_shutdown_requested = false;  // 프로그램 종료 요청 여부를 저장하는 변수

// Ctrl+C 시그널 핸들러
void sigintHandler(int sig) {
    ROS_INFO("Ctrl-C detected, shutting down...");  // 종료 로그 출력
    g_shutdown_requested = true;  // 종료 플래그 설정
}

// .bin 파일을 읽어서 포인트 클라우드로 변환하는 함수
void loadBinToCloudXYZ(const std::string& bin_path, int index, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::stringstream ss;  // 문자열 스트림 선언
    ss << std::setw(6) << std::setfill('0') << index;  // 인덱스를 6자리로 맞추고 0으로 채움
    std::string filename = bin_path + ss.str() + ".bin";  // 파일 경로 생성

    std::fstream input(filename.c_str(), std::ios::in | std::ios::binary);  // 바이너리 파일 읽기 모드로 열기
    if (!input.good()) {  // 파일이 정상적으로 열리지 않으면
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));  // 오류 메시지 출력 후 종료
    }
    input.seekg(0, std::ios::beg);  // 파일의 처음으로 이동

    while (input.good() && !input.eof()) {  // 파일의 끝에 도달할 때까지 반복
        pcl::PointXYZ point;  // 포인트 객체 생성
        input.read((char *) &point.x, 3 * sizeof(float));  // x, y, z 좌표를 읽음
        float intensity;  // 강도(intensity) 값 저장 변수
        input.read((char *) &intensity, sizeof(float));  // 강도 값을 읽음
        cloud->push_back(point);  // 포인트 클라우드에 추가
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "play_kitti_for_relocate");  // ROS 노드 초기화
    ros::NodeHandle nh("~");  // ROS 노드 핸들 생성 (private namespace)

    std::string dataset_folder;  // 데이터셋 폴더 경로 변수
    std::string seq_number;  // KITTI 시퀀스 번호 변수
    int publish_delay;  // 퍼블리싱 딜레이 변수

    nh.getParam("dataset_folder", dataset_folder);  // ROS 파라미터에서 데이터셋 폴더 경로 가져오기
    nh.getParam("seq_number", seq_number);  // ROS 파라미터에서 시퀀스 번호 가져오기
    nh.getParam("publish_delay", publish_delay);  // ROS 파라미터에서 퍼블리싱 딜레이 가져오기
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;  // 딜레이가 0 이하이면 기본값 1로 설정

    std::cout << "======== Reading KITTI " << seq_number << " from: " << dataset_folder << " ========\n";  // 데이터셋 로드 메시지 출력

    ros::Publisher pub_laser_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 3000);  // LiDAR 포인트 클라우드 퍼블리셔 생성
    std::ifstream timestamp_file(dataset_folder + seq_number + "/times.txt", std::ifstream::in);  // 타임스탬프 파일 열기
    std::cout << dataset_folder + seq_number + "/times.txt" << "\n";  // 타임스탬프 파일 경로 출력

    ros::Duration(1.0).sleep();  // 1초 대기
    signal(SIGINT, sigintHandler);  // Ctrl+C 시그널 핸들러 등록

    std::string line;  // 한 줄씩 읽기 위한 문자열 변수
    std::vector<float> times;  // 타임스탬프를 저장할 벡터
    while (1) {  // 무한 루프
        if (std::getline(timestamp_file, line)) {  // 파일에서 한 줄 읽기
            float timestamp = stof(line);  // 문자열을 실수로 변환
            times.push_back(timestamp);  // 타임스탬프 벡터에 추가
        } else {
            break;  // 파일의 끝이면 루프 종료
        }
    }
    std::cout << "Total times: " << times.size() << "\n";  // 읽어들인 타임스탬프 개수 출력

    std::vector<int> indexes(times.size());  // 인덱스 벡터 생성
    std::iota(indexes.begin(), indexes.end(), 0);  // 0부터 N-1까지 채우기

    std::random_device rd;  // 난수 생성기
    std::mt19937 g(rd());  // 난수 엔진 초기화
    std::shuffle(indexes.begin(), indexes.end(), g);  // 인덱스 섞기

    std::queue<int> index_queue;  // 인덱스를 저장할 큐 생성
    for (int i = 0; i < indexes.size(); ++i) {  // 인덱스 벡터의 모든 요소를
        index_queue.push(indexes[i]);  // 큐에 삽입
    }

    ros::Rate r(10.0 / publish_delay);  // 퍼블리싱 속도 조정
    while (ros::ok() && !g_shutdown_requested) {  // ROS가 실행 중이고 종료 요청이 없을 경우 루프 실행
        if (index_queue.empty()) {  // 큐가 비었으면 종료
            break;
        }
        int data_id = index_queue.front();  // 큐의 맨 앞 인덱스 가져오기
        index_queue.pop();  // 큐에서 제거
        std::cout << data_id << "th pcd\n";  // 현재 처리 중인 PCD 번호 출력
        std::string bin_path = dataset_folder + seq_number + "/velodyne/";  // 바이너리 파일 경로 생성
        pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);  // 포인트 클라우드 객체 생성
        loadBinToCloudXYZ(bin_path, data_id, laser_cloud);  // 파일 로드하여 포인트 클라우드로 변환

        sensor_msgs::PointCloud2 laser_cloud_msg;  // ROS 포인트 클라우드 메시지 생성
        pcl::toROSMsg(*laser_cloud, laser_cloud_msg);  // PCL 데이터를 ROS 메시지로 변환
        laser_cloud_msg.header.stamp = ros::Time().fromSec(times[data_id]);  // 타임스탬프 설정
        laser_cloud_msg.header.frame_id = "/camera_init";  // 프레임 설정
        pub_laser_cloud.publish(laser_cloud_msg);  // 메시지 퍼블리시
        r.sleep();  // 다음 루프까지 대기
    }

    return 0;  // 프로그램 종료
}
