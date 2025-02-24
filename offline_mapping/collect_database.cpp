#include <pcl_ros/point_cloud.h>  // ROS에서 PCL 포인트 클라우드를 사용하기 위한 헤더 파일
#include <ros/ros.h>  // ROS 기본 기능을 사용하기 위한 헤더 파일
#include <iostream>  // 표준 입출력을 위한 헤더 파일
#include <fstream>  // 파일 입출력을 위한 헤더 파일
#include <Eigen/Dense>  // Eigen 라이브러리를 사용하여 선형 대수 연산을 수행하기 위한 헤더 파일

#include "for_time.hpp"  // 시간 관련 기능을 제공하는 사용자 정의 헤더 파일
#include "for_cloud.hpp"  // 포인트 클라우드 관련 기능을 제공하는 사용자 정의 헤더 파일
#include "for_desc.hpp"  // 특징점 기술자 관련 기능을 제공하는 사용자 정의 헤더 파일
#include "for_io.hpp"  // 입출력 관련 기능을 제공하는 사용자 정의 헤더 파일

#include "pcl/io/pcd_io.h"  // PCL에서 PCD 파일을 읽고 쓰기 위한 헤더 파일
#include <pcl/filters/extract_indices.h>  // 포인트 클라우드에서 특정 인덱스를 추출하기 위한 필터 헤더 파일
#include <pcl/search/kdtree.h>  // K-D 트리를 사용하여 최근접 이웃 검색을 수행하기 위한 헤더 파일
#include <pcl/common/transforms.h>  // 포인트 클라우드 변환을 위한 헤더 파일

void simulateOrientationWithinMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_map, const pcl::PointXYZ& cnt_pt,
    int rows, int cols, int length, int theta_num, std::vector<Eigen::MatrixXi>& descs) {
    
    // 포인트 클라우드가 비어있는 경우, 모든 descriptor를 0으로 초기화하여 반환
    if (cloud_map->empty()) {
        Eigen::MatrixXi desc = Eigen::MatrixXi::Zero(1, rows * cols); // 빈 descriptor 생성
        descs.resize(theta_num, desc); // 모든 방향에 대해 0으로 채워진 descriptor 할당
        return;
    }
    
    float reso_theta = M_PI * 2.0 / theta_num;  // 회전 각도 분해능 (전체 360도를 theta_num 개수로 분할)
    float cnt_pt_x = cnt_pt.x;  // 기준점 x 좌표
    float cnt_pt_y = cnt_pt.y;  // 기준점 y 좌표

    // descriptor 행렬 사전 할당
    Eigen::MatrixXi desc(1, rows * cols);  
    for (int i = 0; i < theta_num; ++i) {  // theta_num 개수만큼 반복하여 회전 적용
        Eigen::Matrix4d trans = Eigen::Matrix4d::Identity(); // 4x4 단위 행렬 생성 (변환 행렬)
        double theta = i * reso_theta;  // 현재 회전 각도 계산
        double theta_cos = std::cos(theta);  // 회전 각도의 코사인 값
        double theta_sin = std::sin(theta);  // 회전 각도의 사인 값      

        // 회전 변환 행렬 설정 (2D 회전)
        trans(0, 0) = theta_cos; 
        trans(0, 1) = -theta_sin; 
        trans(1, 0) = theta_sin; 
        trans(1, 1) = theta_cos;
        
        // 중심점을 기준으로 회전 변환 적용 (좌표 변환)
        double cos_offset = 1 - theta_cos; 
        trans(0, 3) = cos_offset * cnt_pt_x + theta_sin * cnt_pt_y; // x축 이동 계산
        trans(1, 3) = cos_offset * cnt_pt_y - theta_sin * cnt_pt_x; // y축 이동 계산

        // 포인트 클라우드 회전 적용
        PointCloudPtr rotated_map(new PointCloud);  // 변환된 포인트 클라우드 생성
        pcl::transformPointCloud(*cloud_map, *rotated_map, trans);  // 포인트 클라우드 회전 변환 수행

        // 회전된 포인트 클라우드를 기반으로 descriptor 생성 및 저장
        desc = makeGridDesc(rotated_map, cnt_pt, rows, cols, length, -100, 100);
        descs.push_back(desc);  // descriptor 벡터에 추가
    }    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "collect_database");  // ROS 노드를 초기화하고, 노드 이름을 "collect_database"로 설정
    ros::NodeHandle nh("~");;  // ROS 노드 핸들 생성 (개인 네임스페이스 사용)

    nh.getParam("PARA_DIR_MAP", PARA_DIR_MAP);  // 파라미터 서버에서 "PARA_DIR_MAP" 값을 가져와 변수에 저장

    std::cout << "**************************************************\n";
    std::cout << "***  Loading Offline Map and Candidate Points  ***\n";
    std::cout << "***    Please do not turn off the terminal.    ***\n";
    std::cout << "**************************************************\n";

    PointCloudPtr candidate_pts(new PointCloud), pass_map(new PointCloud);  // 후보 포인트 클라우드와 통과 가능 지도 포인트 클라우드 생성

    // 후보 포인트 클라우드 로드
    if (pcl::io::loadPCDFile(PARA_DIR_MAP + "candidate_pts.pcd", *candidate_pts) == 0) {  
        std::cout << "Loaded candidate pts contains: " << candidate_pts->size() << " pts..\n";  // 성공적으로 로드된 경우 출력
    } else {
        ROS_ERROR("Failed to load candidate points from %s", (PARA_DIR_MAP + "candidate_pts.pcd").c_str());  // 파일 로드 실패 시 오류 메시지 출력
    }
    for (auto &pt : candidate_pts->points) {  
        pt.z = 0.0f;  // 후보 포인트의 z 좌표를 0으로 설정
    }

    // 통과 가능 지도 로드
    if (pcl::io::loadPCDFile(PARA_DIR_MAP + "pass_map.pcd", *pass_map) == 0) {  
        std::cout << "Loaded pass map contains: " << pass_map->size() << " pts..\n";  // 성공적으로 로드된 경우 출력
    } else {
        ROS_ERROR("Failed to load pass map from %s", (PARA_DIR_MAP + "pass_map.pcd").c_str());  // 파일 로드 실패 시 오류 메시지 출력
    }
    // removeOutliersSOR(pass_map, pass_map, 50, 1.5);  // 이상치 제거 (주석 처리됨)
    // pcl::io::savePCDFileASCII(PARA_DIR_MAP + "pass_map_sor.pcd", *pass_map);  // 수정된 지도 저장 (주석 처리됨)

    PointCloudPtr xoy_map(new PointCloud);  // XY 평면용 포인트 클라우드 생성
    pcl::copyPointCloud(*pass_map, *xoy_map);  // 통과 가능 지도 데이터를 복사하여 XY 평면용 클라우드 생성
    for (auto &pt : xoy_map->points) {  
        pt.z = 0.0;  // 모든 포인트의 z 값을 0으로 설정하여 2D 지도 생성
    }
    // pcl::io::savePCDFileASCII(PARA_DIR_MAP + "xoy_map.pcd", *xoy_map);  // XY 평면 지도 저장 (주석 처리됨)

    randomSampleCloud(xoy_map, xoy_map, 500000);  // 랜덤 샘플링을 통해 XY 지도에서 50만 개의 점을 선택
    // pcl::io::savePCDFileASCII(PARA_DIR_MAP + "xoy_sam_map.pcd", *xoy_map);  // 샘플링된 지도 저장 (주석 처리됨)

    // 지도 데이터베이스 수집을 위한 반지름 계산
    float radius = 1.41421 * PARA_ROWS * 0.5 * PARA_LENGTH;  // 지도의 반지름을 계산하는 수식

    std::cout << "**************************************************\n";
    std::cout << "***   Collecting map database, please don't    ***\n";
    std::cout << "***        shut down... Just a moment!         ***\n";
    std::cout << "**************************************************\n";

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  // Kd-Tree 검색 구조 생성
    tree->setInputCloud(xoy_map);  // XY 평면 포인트 클라우드를 Kd-Tree의 입력으로 설정
    pcl::ExtractIndices<pcl::PointXYZ>::Ptr index_extractor(new pcl::ExtractIndices<pcl::PointXYZ>);  // 포인트 추출기 생성
    index_extractor->setInputCloud(xoy_map);  // 입력 클라우드를 XY 평면 포인트 클라우드로 설정
    
    std::vector<Eigen::MatrixXi> descs_total;  // 후보 포인트의 기술자(Descriptors) 저장을 위한 벡터 생성
    for (int i = 0; i < candidate_pts->size(); ++i) {  
        if (!ros::ok()) {  
            ROS_WARN("Program interrupted! Stopping processing.");  // ROS가 종료되었을 경우 경고 메시지를 출력하고 루프를 종료
            break;
        }

        // 진행 상태 출력
        int progress = std::ceil((static_cast<float>(i) / candidate_pts->size()) * 100);  // 진행률(%) 계산
        if (i % 1000 == 0 || i == candidate_pts->size() - 1) {  
            std::cout << "### Progress: " << progress << "% (" << i << "/" << candidate_pts->size() << ") ###\n";  // 1000개마다 또는 마지막 포인트에서 진행률 출력
        }

        const pcl::PointXYZ& pt = candidate_pts->points[i];  // 현재 후보 포인트를 가져옴
        std::vector<int> indices;  // 반경 내 포인트들의 인덱스를 저장할 벡터
        std::vector<float> distances_sq;  // 반경 내 포인트들의 거리 제곱 값을 저장할 벡터
        tree->radiusSearch(pt, radius, indices, distances_sq);  // Kd-Tree를 사용하여 반경 내 포인트 검색

        pcl::IndicesPtr indices_ptr = boost::make_shared<std::vector<int>>(indices);  // 검색된 인덱스를 공유 포인터로 변환
        index_extractor->setIndices(indices_ptr);  // 검색된 인덱스를 추출기에 설정
        index_extractor->setNegative(false);  // 추출된 포인트를 유지하도록 설정
        PointCloudPtr near_map(new PointCloud);  // 주변 포인트 클라우드를 저장할 변수 생성
        index_extractor->filter(*near_map);  // 인덱스를 기반으로 주변 포인트 클라우드 추출
        
        std::vector<Eigen::MatrixXi> desc_onept;  // 하나의 후보 포인트에 대한 기술자 벡터
        simulateOrientationWithinMap(near_map, pt, PARA_ROWS, PARA_COLS, PARA_LENGTH, PARA_THETA_NUM, desc_onept);  // 후보 포인트에 대해 지도 내 방향성을 시뮬레이션하여 기술자 생성
        for (int j = 0; j < desc_onept.size(); ++j) {  
            descs_total.push_back(desc_onept[j]);  // 생성된 기술자를 전체 벡터에 추가
        }
    }
    

    std::cout << "**************************************************\n";
    std::cout << "***      Finding hash keys, please don't       ***\n";
    std::cout << "***        shut down... Just a moment!         ***\n";
    std::cout << "**************************************************\n";
    std::unordered_map<size_t, std::vector<size_t>> key_locations;  // 해시 키와 해당되는 인덱스를 저장할 맵 생성
    for (size_t i = 0; i < descs_total.size(); ++i) {  
        int progress = std::ceil((static_cast<float>(i) / descs_total.size()) * 100);  // 진행률(%) 계산
        if (i % 50000 == 0 || i == descs_total.size() - 1) {  
            std::cout << "### Progress: " << progress << "% (" << i << "/" << descs_total.size() << ") ###\n";  // 50000개마다 또는 마지막 포인트에서 진행률 출력
        }

        for (size_t key = 0; key < (PARA_ROWS * PARA_COLS); ++key) {  // 행렬 내 각 키에 대해 반복
            if (descs_total[i](0, key) > 0.5) {  // 기술자 값이 0.5보다 큰 경우 해당 키 저장
                key_locations[key].push_back(i);  // 해당 키에 대한 인덱스 추가
            }
        }
    }

    std::cout << "**************************************************\n";
    std::cout << "***      Saving offline files, please don't    ***\n";
    std::cout << "***        shut down... Just a moment!         ***\n";
    std::cout << "**************************************************\n";
    for (const auto& pair : key_locations) {  
        size_t key = pair.first;  // 해시 키 가져오기
        saveIntegersAsBinary(pair.second, PARA_DIR_MAP + "database/" + std::to_string(key) + ".bin");  // 해당 키에 대한 데이터를 바이너리 파일로 저장
    }

    return 0;  // 프로그램 종료
}


