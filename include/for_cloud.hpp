#ifndef _LIDAR_100_UTILS_CLOUD_H_
#define _LIDAR_100_UTILS_CLOUD_H_

#include <pcl/point_cloud.h>  // PCL(Point Cloud Library)의 포인트 클라우드 헤더 파일
#include <pcl/point_types.h>  // PCL 포인트 타입 정의 헤더 파일
#include <pcl/filters/voxel_grid.h>  // PCL의 Voxel Grid 필터링을 위한 헤더 파일
#include <pcl/filters/random_sample.h>  // PCL의 랜덤 샘플링 필터를 위한 헤더 파일
#include <Eigen/Core>  // Eigen의 기본 연산을 위한 헤더 파일
#include <Eigen/Dense>  // Eigen의 Dense 연산을 위한 헤더 파일
#include <pcl/filters/extract_indices.h>  // PCL의 인덱스 기반 필터링을 위한 헤더 파일
#include <pcl/filters/statistical_outlier_removal.h>  // PCL의 이상치 제거 필터를 위한 헤더 파일
#include <fstream>  // 파일 입출력을 위한 헤더 파일
#include <iomanip>  // 입출력 형식 조정을 위한 헤더 파일

// 포인트 클라우드 타입 정의
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  // XYZ 타입 포인트 클라우드
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;  // XYZ 타입 포인트 클라우드 스마트 포인터

// 랜덤 샘플링을 통해 포인트 클라우드에서 N개의 샘플을 추출하는 함수
void randomSampleCloud(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out, int N) {
    pcl::RandomSample<pcl::PointXYZ> random_sampler;  // 랜덤 샘플링 객체 생성
    random_sampler.setInputCloud(cloud_in);  // 입력 포인트 클라우드 설정
    random_sampler.setSample(N);  // 샘플링할 포인트 개수 설정
    random_sampler.filter(*cloud_out);  // 샘플링된 포인트 클라우드를 저장
}

// Voxel Grid 필터를 적용하여 포인트 클라우드의 해상도를 줄이는 함수
void voxelSampleCloud(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out,
    const float leaf_x, const float leaf_y, const float leaf_z) {
    pcl::VoxelGrid<pcl::PointXYZ> grid;  // Voxel Grid 필터 객체 생성
    grid.setInputCloud(cloud_in);  // 입력 포인트 클라우드 설정
    grid.setLeafSize(leaf_x, leaf_y, leaf_z);  // Voxel 크기 설정
    grid.filter(*cloud_out);  // 필터링된 포인트 클라우드 저장
}

// Statistical Outlier Removal (SOR) 필터를 적용하여 이상치를 제거하는 함수
void removeOutliersSOR(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out, 
    int mean_k, double stddev_mul_thresh) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;  // SOR 필터 객체 생성
    sor.setInputCloud(cloud_in);  // 입력 포인트 클라우드 설정
    sor.setMeanK(mean_k);  // 평균을 계산할 주변 포인트 개수 설정
    sor.setStddevMulThresh(stddev_mul_thresh);  // 표준편차 곱 임계값 설정
    sor.setNegative(false);  // 이상치 제거 모드 설정 (false: 이상치 제거, true: 이상치만 유지)
    sor.filter(*cloud_out);  // 필터링된 포인트 클라우드 저장
}

#endif  // _LIDAR_100_UTILS_CLOUD_H_
