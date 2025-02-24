#include <pcl_ros/point_cloud.h>  // PCL 포인트 클라우드를 ROS 메시지로 변환하는 헤더 파일
#include <ros/ros.h>  // ROS 기본 기능을 포함하는 헤더 파일
#include <iostream>  // 표준 입출력 스트림을 포함하는 헤더 파일
#include <Eigen/Dense>  // Eigen 라이브러리를 사용하여 선형대수 연산을 위한 헤더 파일

#include "for_time.hpp"  // 시간 측정을 위한 사용자 정의 헤더 파일 포함
#include "for_cloud.hpp"  // 포인트 클라우드 관련 사용자 정의 헤더 파일 포함
#include "for_desc.hpp"  // 설명(description) 관련 사용자 정의 헤더 파일 포함
#include "for_io.hpp"  // 입출력 관련 사용자 정의 헤더 파일 포함

#include "pcl/io/pcd_io.h"  // PCL의 PCD 파일 입출력 기능을 포함하는 헤더 파일
#include <pcl/common/transforms.h>  // PCL에서 좌표 변환을 수행하기 위한 헤더 파일
#include <algorithm>  // std::max_element 등 알고리즘 함수를 사용하기 위한 헤더 파일
#include <omp.h>  // OpenMP 병렬 처리를 위한 헤더 파일

// 후보 포인트 클라우드를 저장하는 포인터 생성
PointCloudPtr _candidate_pts(new PointCloud);

// 중심 좌표 초기화 (0.0, 0.0, 0.0)
pcl::PointXYZ _cnt_pt(0.0f, 0.0f, 0.0f);

// 각도 해상도 설정 (360도를 PARA_THETA_NUM 개로 나눈 값)
float _theta_reso = M_PI * 2.0 / (float) PARA_THETA_NUM;

// 설명자(Descriptor)의 총 개수 초기화
int _desc_total_num = 0;

// 위치 재배치를 위한 회전 행렬 초기화 (항등 행렬)
Eigen::Matrix3f _rot_relocate = Eigen::Matrix3f::Identity();
// 위치 재배치를 위한 4x4 변환 행렬 초기화 (항등 행렬)
Eigen::Matrix4f _pose_relocate = Eigen::Matrix4f::Identity();
// 최종 x, y 좌표 결과 저장 변수 (초기값: 0.0, 0.0, 0.0)
pcl::PointXYZ _xy_result = pcl::PointXYZ(0.0f, 0.0f, 0.0f);

// 최종 각도 결과 저장 변수 (초기값: 0)
float _theta_result = 0;
// 최적의 각도 인덱스 저장 변수 (초기값: 0)
int _theta_index = 0;
// 최적의 포인트 인덱스 저장 변수 (초기값: 0)
int _pt_index = 0;

// 최대 투표 수 저장 변수 (초기값: 0)
int _max_vote = 0;

// 데이터 버퍼를 보호하기 위한 뮤텍스
std::mutex mBuf;

// 수신된 포인트 클라우드 데이터를 저장하는 큐
std::queue<sensor_msgs::PointCloud2ConstPtr> _cloud_buf;

// 스캔된 포인트 클라우드를 저장하는 포인터 생성
pcl::PointCloud<pcl::PointXYZ>::Ptr _scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);

// 키 위치와 관련된 정보를 저장하는 해시맵
std::unordered_map<size_t, std::vector<size_t>> _key_locations;

// 수신된 포인트 클라우드 데이터를 버퍼에 저장하는 콜백 함수
void recieveCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
	mBuf.lock();  // 뮤텍스를 잠가서 다중 스레드 환경에서 동기화 처리
	_cloud_buf.push(cloud_msg);  // 수신된 포인트 클라우드 데이터를 큐에 추가
	mBuf.unlock();  // 뮤텍스 해제
}

// 최대 투표 수를 찾는 함수
void findMaxVotes(const Eigen::MatrixXi& query) {
    _max_vote = 0;  // 최대 투표 수 초기화
    std::vector<int> votes(_desc_total_num, 0);  // 설명자 개수만큼 투표 카운트 배열 초기화

    // OpenMP를 사용하여 병렬 처리 수행 (최대 8개 스레드 사용)
    omp_set_num_threads(8);
    #pragma omp parallel for
    for (size_t key = 0; key < PARA_CELL_NUM; ++key) {  // 모든 셀에 대해 반복
        if (query(key) && _key_locations.count(key)) {  // 키가 존재하는 경우
            for (size_t index : _key_locations.at(key)) {  // 해당 키에 속하는 인덱스 순회
                #pragma omp atomic
                ++votes[index];  // 원자적 연산을 사용하여 투표 수 증가
            }
        }
    } 

    // votes 벡터에서 최댓값을 찾아 최대 투표 인덱스 결정
    auto max_iter = std::max_element(votes.begin(), votes.end());
    const int max_index = std::distance(votes.begin(), max_iter);

    // 최대 투표를 기반으로 최적의 포인트 및 각도 인덱스 설정
    _pt_index = max_index / PARA_THETA_NUM;  // 포인트 인덱스 계산
    _theta_index = max_index % PARA_THETA_NUM;  // 각도 인덱스 계산
    _max_vote = *max_iter;  // 최대 투표 수 저장
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "relocate");  // ROS 노드 초기화
    ros::NodeHandle nh("~");  // 노드 핸들 생성 (Private 네임스페이스)

    // 파라미터 서버에서 맵 디렉토리 및 높이 범위 가져오기
    nh.getParam("PARA_DIR_MAP", PARA_DIR_MAP);
    nh.getParam("PARA_MIN_Z_LOCATE", PARA_MIN_Z_LOCATE);
    nh.getParam("PARA_MAX_Z_LOCATE", PARA_MAX_Z_LOCATE);

    // LiDAR 정렬된 스캔 데이터를 퍼블리시하는 ROS 퍼블리셔 생성
    ros::Publisher pub_aligned_scan = nh.advertise<sensor_msgs::PointCloud2>("/aligned_scan", 1000);
    
    // Velodyne 포인트 클라우드 데이터를 구독하는 ROS 서브스크라이버 생성
    ros::Subscriber sub_scan = nh.subscribe("/velodyne_points", 1000, recieveCloud);

    // 맵 데이터 로딩 메시지 출력
    std::cout << "==================================================\n";
    std::cout << "||                                              ||\n";
    std::cout << "||           Loading Map Database..             ||\n";
    std::cout << "||     Please do not turn off the terminal.     ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "==================================================\n";

    // 후보 포인트 데이터 로딩
    if (pcl::io::loadPCDFile(PARA_DIR_MAP + "candidate_pts.pcd", *_candidate_pts) == 0) {
        std::cout << "Loaded candidate pts contains:" << _candidate_pts->size() << " points\n";
    } else {
        ROS_ERROR("Failed to load [[ Candidate Pts ]] from %s", (PARA_DIR_MAP + "candidate_pts.pcd").c_str());
    }

    // 설명자(Descriptor) 총 개수 설정
    _desc_total_num = _candidate_pts->size() * PARA_THETA_NUM;

    // 데이터베이스 로딩 시간 측정을 위한 타이머 시작
    TicToc timer_database;

    // 키 위치 데이터 로드
    for (size_t key = 0; key < PARA_CELL_NUM; ++key) {
        if (key % 400 == 0) {  // 진행률 출력
            int progress = std::ceil(((float)key / PARA_CELL_NUM) * 100);
            std::cout << "### Progress: " << progress << "% (" << key << "/" << PARA_CELL_NUM << ") ###\n";
        }

        std::vector<size_t> numbers;
        readIntegersFromBinary(PARA_DIR_MAP + "database/" + std::to_string(key) + ".bin", numbers);
        _key_locations[key].assign(numbers.begin(), numbers.end());  // 로드된 데이터를 맵에 저장
    }

    // 데이터베이스 로딩 완료 메시지 출력
    std::cout << "Loading database takes: " << timer_database.toc() << "ms\n\n";

    // 초기화 완료 및 재배치 시작 메시지 출력
    std::cout << "==================================================\n";
    std::cout << "||                                              ||\n";
    std::cout << "||          Begin Relocate Within Map           ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "||    LiDAR scans can be played at any time,    ||\n";
    std::cout << "||         without requiring sequential.        ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "==================================================\n\n";

    // ROS 실행 루프 설정 (1Hz)
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();  // 콜백 실행

        if (_cloud_buf.empty()) {
            continue;  // 데이터가 없으면 다음 루프로 넘어감
        }

        _scan_cloud->clear();  // 기존 포인트 클라우드 초기화
        pcl::fromROSMsg(*_cloud_buf.front(), *_scan_cloud);  // ROS 메시지를 PCL 포인트 클라우드로 변환
        _cloud_buf.pop();  // 버퍼에서 제거

        TicToc timer_locate;  // 위치 추정 시간 측정을 위한 타이머 시작

        // 포인트 클라우드 다운샘플링 (Voxel Grid 적용)
        voxelSampleCloud(_scan_cloud, _scan_cloud, 0.2, 0.2, 0.2);

        // 설명자 생성 (그리드 디스크립터)
        Eigen::MatrixXi scan_desc = makeGridDesc(_scan_cloud, _cnt_pt, PARA_ROWS, PARA_COLS, PARA_LENGTH, PARA_MIN_Z_LOCATE, PARA_MAX_Z_LOCATE);

        // 최적의 후보 점과 각도 찾기
        findMaxVotes(scan_desc);
        std::cout << "Relocated takes: " << timer_locate.toc() << "ms\n";

        // 위치 정보 설정
        _xy_result = _candidate_pts->at(_pt_index);  // 후보 포인트 중 최적 위치 선택
        _theta_result = _theta_index * _theta_reso;  // 최적 각도 설정

        // 회전 행렬 생성
        _rot_relocate = Eigen::Matrix3f::Identity();
        _rot_relocate(0, 0) =  std::cos(_theta_result); 
        _rot_relocate(0, 1) = -std::sin(_theta_result); 
        _rot_relocate(1, 0) =  std::sin(_theta_result); 
        _rot_relocate(1, 1) =  std::cos(_theta_result);

        // 변환 행렬 생성
        _pose_relocate = Eigen::Matrix4f::Identity();
        _pose_relocate.block<3, 3>(0, 0) = _rot_relocate.inverse();
        _pose_relocate(0, 3) = _xy_result.x; 
        _pose_relocate(1, 3) = _xy_result.y;

        // 포인트 클라우드 좌표 변환 적용
        pcl::transformPointCloud(*_scan_cloud, *_scan_cloud, _pose_relocate);

        // 변환된 포인트 클라우드를 ROS 메시지로 변환 후 퍼블리싱
        sensor_msgs::PointCloud2 aligned_scan_msg;
        pcl::toROSMsg(*_scan_cloud, aligned_scan_msg);
        aligned_scan_msg.header.stamp = ros::Time::now();
        aligned_scan_msg.header.frame_id = "map";
        pub_aligned_scan.publish(aligned_scan_msg);

        loop_rate.sleep();  // 주기 맞춰 실행
    }

    return 0;  // 프로그램 종료
}


