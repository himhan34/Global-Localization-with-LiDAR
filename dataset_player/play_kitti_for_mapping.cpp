#include <iostream>  // 표준 입출력 라이브러리
#include <fstream>  // 파일 입출력 라이브러리
#include <iterator>  // 반복자 관련 라이브러리
#include <string>  // 문자열 처리 라이브러리
#include <vector>  // 벡터 자료구조 사용을 위한 라이브러리
#include <image_transport/image_transport.h>  // ROS에서 이미지 전송을 위한 라이브러리
#include <nav_msgs/Odometry.h>  // ROS에서 오도메트리 메시지를 처리하기 위한 라이브러리
#include <nav_msgs/Path.h>  // ROS에서 경로 데이터를 다루기 위한 라이브러리
#include <ros/ros.h>  // ROS 기본 기능을 제공하는 라이브러리
#include <rosbag/bag.h>  // ROS bag 파일을 다루기 위한 라이브러리
#include <geometry_msgs/PoseStamped.h>  // ROS에서 위치 데이터를 표현하기 위한 메시지 타입
#include <sensor_msgs/image_encodings.h>  // 센서 이미지 인코딩 처리를 위한 라이브러리
#include <eigen3/Eigen/Dense>  // 선형대수 연산을 위한 Eigen 라이브러리
#include <pcl/point_cloud.h>  // PCL(Point Cloud Library)에서 포인트 클라우드 데이터를 다루기 위한 라이브러리
#include <pcl/point_types.h>  // PCL에서 제공하는 포인트 타입 사용을 위한 라이브러리
#include <pcl_conversions/pcl_conversions.h>  // PCL과 ROS 메시지 간 변환을 위한 라이브러리
#include <sensor_msgs/PointCloud2.h>  // ROS에서 포인트 클라우드 메시지를 다루기 위한 라이브러리
#include <pcl/io/pcd_io.h>  // PCL에서 PCD(Point Cloud Data) 파일 입출력을 위한 라이브러리
#include <signal.h>  // 신호 처리(예: Ctrl+C 감지)를 위한 라이브러리
#include <geometry_msgs/TransformStamped.h>  // ROS에서 좌표 변환 메시지를 다루기 위한 라이브러리

bool g_shutdown_requested = false;  // 프로그램 종료 여부를 저장하는 전역 변수

// Ctrl+C 입력을 감지하고 종료 요청을 처리하는 함수
void sigintHandler(int sig) {
    ROS_INFO("Ctrl-C detected, shutting down...");  // 종료 메시지 출력
    g_shutdown_requested = true;  // 종료 요청 플래그 설정
}

// 텍스트 파일에서 포즈 데이터를 읽어와 4x4 변환 행렬로 변환하는 함수
void loadPoseToMatrix(const std::string& filename, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poses) {
    std::ifstream ifs(filename);  // 파일 읽기 스트림 생성
    if (!ifs.is_open()) {  // 파일을 열지 못했을 경우
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));  // 오류 메시지 출력 후 종료
    }

    std::vector<std::vector<double>> values;  // 포즈 값을 저장할 2차원 벡터
    std::string line;  // 한 줄씩 읽어오기 위한 문자열 변수
    while (std::getline(ifs, line)) {  // 파일에서 한 줄씩 읽기
        std::istringstream ss(line);  // 문자열 스트림 생성
        values.emplace_back(std::istream_iterator<double>{ss}, std::istream_iterator<double>{});  // 문자열을 숫자 벡터로 변환
    }

    if (values.empty() || values[0].empty()) {  // 값이 비어있거나 잘못된 경우
        std::exit((std::cout << "Values data is empty or malformed.", EXIT_FAILURE));  // 오류 메시지 출력 후 종료
    }

    for (int i = 0; i < (int)values.size(); ++i) {  // 읽은 데이터를 행렬로 변환
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();  // 4x4 단위 행렬 초기화
        for (int j = 0; j < (int)values[0].size(); ++j) {  // 데이터 삽입
            int row = j / 4;  // 행 인덱스 계산
            int col = j % 4;  // 열 인덱스 계산
            pose(row, col) = values[i][j];  // 행렬에 값 삽입
        }
        poses.push_back(pose);  // 벡터에 추가
    } 

    ifs.close();  // 파일 닫기
}

// KITTI 포맷의 바이너리 포인트 클라우드를 로드하여 PCL 포인트 클라우드 객체로 변환하는 함수
void loadBinToCloudXYZ(const std::string& bin_path, int index, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << index;  // 인덱스를 6자리 숫자로 변환
    std::string filename = bin_path + ss.str() + ".bin";  // 파일 경로 생성
   
    std::fstream input(filename.c_str(), std::ios::in | std::ios::binary);  // 바이너리 파일 열기
    if (!input.good()) {  // 파일을 열 수 없는 경우
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));  // 오류 메시지 출력 후 종료
    }
    input.seekg(0, std::ios::beg);  // 파일의 시작으로 이동
    
    while (input.good() && !input.eof()) {  // 파일이 정상적으로 읽히는 동안
        pcl::PointXYZ point;    
        input.read((char *) &point.x, 3*sizeof(float));  // X, Y, Z 좌표 읽기
        float intensity;
        input.read((char *) &intensity, sizeof(float));  // 강도 값 읽기
        cloud->push_back(point);  // 포인트 클라우드에 추가
    } 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "play_kitti_for_mapping");  // ROS 노드 초기화
    ros::NodeHandle nh("~");  // ROS 노드 핸들 생성

    std::string dataset_folder;  // 데이터셋 폴더 경로 변수
    std::string seq_number;  // 시퀀스 번호 변수
    int publish_delay;  // 데이터 전송 지연 시간 변수

    nh.getParam("dataset_folder", dataset_folder);  // 파라미터에서 데이터셋 폴더 경로 가져오기
    nh.getParam("seq_number", seq_number);  // 파라미터에서 시퀀스 번호 가져오기
    nh.getParam("publish_delay", publish_delay);  // 파라미터에서 전송 지연 시간 가져오기
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;  // 최소 지연시간 1초 설정

    std::cout << "======== Reading KITTI " << seq_number << " from: " << dataset_folder << " ========\n";

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses;  // 포즈 데이터 저장 벡터
    std::string path_pose = dataset_folder + seq_number + "/lidar_pose.txt";  // 포즈 파일 경로
    loadPoseToMatrix(path_pose, poses);  // 포즈 데이터 로드
    std::cout << "Pose total: " << poses.size() << "\n";

    ros::Publisher pub_laser_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1000);  // 포인트 클라우드 퍼블리셔 생성
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/lidar_pose", 1000);  // 포즈 퍼블리셔 생성
    geometry_msgs::PoseStamped pose_msg;  // 포즈 메시지 객체

    ros::Duration(1.0).sleep();  // 구독자 연결을 기다리기 위한 대기

    signal(SIGINT, sigintHandler);  // 종료 시그널 핸들러 등록

    std::ifstream time_file(dataset_folder + seq_number + "/times.txt", std::ifstream::in);  // 타임스탬프 파일 열기
    int line_count = 0;
    ros::Rate r(10.0 / publish_delay);  // 지정된 속도로 메시지를 발행하기 위한 ROS 루프 속도 설정

    while (ros::ok() && !g_shutdown_requested) {  // ROS 실행 상태를 유지하는 동안 반복
        if (std::getline(time_file, line)) {  
            float timestamp = stof(line);  // 타임스탬프 읽기
            pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            loadBinToCloudXYZ(dataset_folder + seq_number + "/velodyne/", line_count, laser_cloud);  // 포인트 클라우드 로드
            ++line_count;
            r.sleep();
        } else {
            break;  // 파일 끝에 도달하면 루프 종료
        }
    }
    return 0;
}
