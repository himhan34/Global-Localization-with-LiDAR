#ifndef _LIDAR_100_UTILS_OSC_H_
#define _LIDAR_100_UTILS_OSC_H_

#include <Eigen/Core>  // Eigen 기본 연산을 위한 헤더 파일
#include <Eigen/Dense>  // Eigen Dense 연산을 위한 헤더 파일
#include <vector>  // 벡터 컨테이너를 사용하기 위한 헤더 파일
#include <cmath>  // 수학 연산을 위한 헤더 파일
#include "for_cloud.hpp"  // 포인트 클라우드 관련 유틸리티 헤더 파일

// 매핑 관련 파라미터
float PARA_GRID_SIZE_MAP = 0.5;  // 맵 격자 크기
int PARA_MAX_PTS_PER_MAP_GRID = 20;  // 하나의 맵 격자에 포함될 최대 포인트 수
float PARA_MIN_NOVE = 1.0;  // 최소 이동 거리
float PARA_GRID_SIZE_GROUND = 1.0;  // 지면 맵 격자 크기
// float PARA_MIN_HEIGHT = -0.5;  // 최소 높이 (사용되지 않음)
// float PARA_MAX_HEIGHT = 50.0;  // 최대 높이 (사용되지 않음)
float PARA_MIN_Z_MAP = -0.5;  // 맵의 최소 Z 좌표
float PARA_MAX_Z_MAP = 50.0;  // 맵의 최대 Z 좌표
int PARA_MIN_PTS_PER_GROUND_GRID = 40;  // 지면 격자당 최소 포인트 개수
float PARA_VOXEL_LEAF = 0.2;  // Voxel Grid 필터 크기
float PARA_SENSOR_HEIGHT = 1.75;  // 센서 높이
std::string PARA_DIR_MAP = "./";  // 맵 저장 경로

// 데이터베이스 및 재배치 관련 파라미터
int PARA_ROWS = 40;  // 행 개수
int PARA_COLS = 40;  // 열 개수
int PARA_THETA_NUM = 60;  // 각도 샘플링 개수
float PARA_LENGTH = 1.0;  // 격자의 길이
float PARA_MIN_Z_LOCATE = -0.5;  // 재배치 시 최소 Z 좌표
float PARA_MAX_Z_LOCATE = 50.0;  // 재배치 시 최대 Z 좌표
int PARA_CELL_NUM = PARA_ROWS * PARA_COLS;  // 총 격자 개수

// 포인트 클라우드를 그리드 기반의 이진 기술자로 변환하는 함수
Eigen::MatrixXi makeGridDesc(const PointCloudPtr& cloud_in, const pcl::PointXYZ& pt_cnt, 
    const int& rows, const int& cols, const float& length, const float& min_z, const float& max_z) {

    // X, Y 범위 설정
    float max_x = pt_cnt.x + rows / 2.0 * length;  // 최대 X 좌표
    float min_x = pt_cnt.x - rows / 2.0 * length;  // 최소 X 좌표
    float max_y = pt_cnt.y + cols / 2.0 * length;  // 최대 Y 좌표
    float min_y = pt_cnt.y - cols / 2.0 * length;  // 최소 Y 좌표

    Eigen::MatrixXi desc = Eigen::MatrixXi::Zero(1, rows * cols);  // 기술자 행렬 초기화

    for (int i = 0; i < cloud_in->size(); ++i) {  // 포인트 클라우드 순회
        const pcl::PointXYZ &pt = cloud_in->points[i];  // 현재 포인트

        // Z, X, Y 범위를 벗어나면 무시
        if (pt.z < min_z || pt.z > max_z || pt.x < min_x || pt.x > max_x || pt.y < min_y || pt.y > max_y) {
            continue;
        }

        // 격자 위치 계산
        int r = (max_x - pt.x) / length;  // 행 인덱스 계산
        int c = (max_y - pt.y) / length;  // 열 인덱스 계산

        // 유효한 범위 내에 있는지 확인
        if (r < 0 || r >= rows || c < 0 || c >= cols) {
            continue;
        }

        desc(0, r * cols + c) = 1;  // 해당 격자 위치를 1로 설정
    }
    return desc;  // 이진 기술자 반환
}

#endif  // _LIDAR_100_UTILS_OSC_H_
