#include <pcl_ros/point_cloud.h>  // ROS에서 PCL 포인트 클라우드를 다루기 위한 헤더 파일
#include <ros/ros.h>  // ROS 기본 기능을 사용하기 위한 헤더 파일
#include <pcl_conversions/pcl_conversions.h>  // PCL과 ROS 메시지 간 변환을 위한 헤더 파일
#include <sensor_msgs/PointCloud2.h>  // ROS의 PointCloud2 메시지를 사용하기 위한 헤더 파일
#include <pcl/common/transforms.h>  // PCL의 좌표 변환 기능을 사용하기 위한 헤더 파일
#include <geometry_msgs/PoseStamped.h>  // 로봇의 위치 정보를 포함하는 PoseStamped 메시지를 사용하기 위한 헤더 파일
#include <queue>  // FIFO 방식의 큐 자료구조를 사용하기 위한 헤더 파일
#include <mutex>  // 다중 스레드 환경에서 동기화를 위해 뮤텍스를 사용하기 위한 헤더 파일
#include <unordered_map>  // 해시 기반의 빠른 키-값 저장을 위한 unordered_map 헤더 파일
#include "for_time.hpp"  // 시간 관련 기능을 제공하는 사용자 정의 헤더 파일
#include "for_cloud.hpp"  // 포인트 클라우드 관련 기능을 제공하는 사용자 정의 헤더 파일
#include "for_desc.hpp"  // 특징점 추출 및 설명자 관련 기능을 제공하는 사용자 정의 헤더 파일

#include <pcl/filters/extract_indices.h>  // 특정 인덱스에 해당하는 포인트를 필터링하는 기능 제공
#include "offline_mapping/linefit/ground_segmentation.h"  // 지면 분할을 위한 LineFit 알고리즘 관련 헤더 파일
#include "pcl/io/pcd_io.h"  // PCD 파일 입출력을 위한 헤더 파일

std::mutex mBuf;  // 버퍼 접근을 동기화하기 위한 뮤텍스 객체
std::queue<sensor_msgs::PointCloud2ConstPtr> _cloud_buf;  // 포인트 클라우드 데이터를 저장하는 큐
std::queue<geometry_msgs::PoseStamped::ConstPtr> _pose_buf;  // 로봇의 위치 정보를 저장하는 큐
PointCloudPtr _scan_cloud(new PointCloud);  // 스캔된 포인트 클라우드를 저장할 변수 (PCL 포인트 클라우드 객체)
Eigen::Matrix4f _scan_pose = Eigen::Matrix4f::Identity();  // 스캔 포즈를 나타내는 4x4 변환 행렬 (초기값: 단위 행렬)

// GridIndex 구조체 정의
struct GridIndex {
    int x, y, z;  // 격자의 x, y, z 좌표값

    // 두 GridIndex 객체가 같은지 비교하는 연산자 오버로딩
    bool operator==(const GridIndex &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// GridIndex에 대한 해시 함수 정의 (비트 연산을 사용하여 해시 생성)
struct GridIndexHash {
    size_t operator()(const GridIndex& grid) const {
        return (std::hash<int>()(grid.x) ^ (std::hash<int>()(grid.y) << 1) ^ (std::hash<int>()(grid.z) << 2));
    }
};

// 주어진 포인트의 좌표를 바탕으로 GridIndex를 계산하는 함수
GridIndex calculateGridIndex(const pcl::PointXYZ& point, float voxel_size) {
    return {
        static_cast<int>(std::floor(point.x / voxel_size)),  // x 좌표를 격자 단위로 변환
        static_cast<int>(std::floor(point.y / voxel_size)),  // y 좌표를 격자 단위로 변환
        static_cast<int>(std::floor(point.z / voxel_size))   // z 좌표를 격자 단위로 변환
    };
}

// 격자별로 원시 포인트 데이터를 저장하는 맵
std::unordered_map<GridIndex, std::vector<pcl::PointXYZ>, GridIndexHash> _grid_raw_map;

// 격자별로 필터링된 포인트 데이터를 저장하는 맵
std::unordered_map<GridIndex, std::vector<pcl::PointXYZ>, GridIndexHash> _grid_pass_map;

// 격자별로 지면 포인트 데이터를 저장하는 맵
std::unordered_map<GridIndex, std::vector<pcl::PointXYZ>, GridIndexHash> _grid_ground;

// 원시 포인트 클라우드 데이터를 저장하는 포인트 클라우드 객체
PointCloudPtr _raw_map(new PointCloud);

// 필터링된 포인트 클라우드 데이터를 저장하는 포인트 클라우드 객체
PointCloudPtr _pass_map(new PointCloud);

// 후보 포인트 데이터를 저장하는 포인트 클라우드 객체
PointCloudPtr _candidate_pts(new PointCloud);

// 지면 포인트 클라우드 데이터를 저장하는 객체
PointCloudPtr _ground_scan(new PointCloud);

// 지면 분할 알고리즘의 파라미터 설정
GroundSegmentationParams _ground_params;

// 지면 분할 알고리즘 객체 생성 (파라미터를 입력하여 초기화)
GroundSegmentation _ground_segmenter(_ground_params);

/////////////////////////////////////////////////////////////////////
// PointCloud2 메시지를 수신하는 콜백 함수
void recieveCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    mBuf.lock();  // 버퍼 접근을 위한 뮤텍스 잠금
    _cloud_buf.push(msg);  // 수신한 메시지를 큐에 추가
    mBuf.unlock();  // 뮤텍스 잠금 해제
}


// PoseStamped 메시지를 수신하는 콜백 함수
void recievePose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    mBuf.lock();  // 뮤텍스 잠금
    _pose_buf.push(msg);  // 수신한 PoseStamped 메시지를 큐에 추가
    mBuf.unlock();  // 뮤텍스 잠금 해제
}

// 최소 거리 및 최대 거리 내의 포인트만 남기고 필터링하는 함수
void removeCloseAndFarPts(const PointCloud &cloud_in, PointCloud &cloud_out, float min_dist, float max_dist) {
    float min_dist2 = min_dist * min_dist;  // 최소 거리의 제곱값
    float max_dist2 = max_dist * max_dist;  // 최대 거리의 제곱값
    PointCloud temp_cloud;  // 필터링된 포인트 클라우드를 저장할 임시 객체
    temp_cloud.header = cloud_in.header;  // 입력 클라우드의 헤더를 유지

    // 입력 포인트 클라우드를 순회하며 거리 조건을 만족하는 포인트만 저장
    for (const auto &pt : cloud_in.points) {
        float dist2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;  // 현재 포인트의 거리 제곱값 계산
        if (dist2 >= min_dist2 && dist2 <= max_dist2) {  // 최소/최대 거리 조건 만족 시 추가
            temp_cloud.points.push_back(pt);
        }
    }

    // 필터링된 포인트 클라우드의 속성 업데이트
    temp_cloud.width = static_cast<uint32_t>(temp_cloud.points.size());  // 포인트 개수 설정
    temp_cloud.height = 1;  // 1D 포인트 클라우드 설정
    temp_cloud.is_dense = true;  // 밀집된 데이터로 설정
    cloud_out = std::move(temp_cloud);  // 결과를 출력 클라우드로 이동
}

// geometry_msgs::PoseStamped 메시지를 Eigen 4x4 변환 행렬로 변환하는 함수
Eigen::Matrix4f transformMsgToMatrix(const geometry_msgs::PoseStamped::ConstPtr& msg){
    float x = msg->pose.position.x;  // 위치 정보 (x)
    float y = msg->pose.position.y;  // 위치 정보 (y)
    float z = msg->pose.position.z;  // 위치 정보 (z)

    float qx = msg->pose.orientation.x;  // 쿼터니언 회전 (x)
    float qy = msg->pose.orientation.y;  // 쿼터니언 회전 (y)
    float qz = msg->pose.orientation.z;  // 쿼터니언 회전 (z)
    float qw = msg->pose.orientation.w;  // 쿼터니언 회전 (w)

    Eigen::Quaternionf quat(qw, qx, qy, qz);  // Eigen 쿼터니언 객체 생성
    Eigen::Matrix3f rotation_matrix = quat.toRotationMatrix();  // 회전 행렬 변환

    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();  // 4x4 단위 행렬 초기화
    transform_matrix.block<3, 3>(0, 0) = rotation_matrix;  // 회전 행렬 적용
    transform_matrix(0, 3) = x;  // x 위치 설정
    transform_matrix(1, 3) = y;  // y 위치 설정
    transform_matrix(2, 3) = z;  // z 위치 설정

    return transform_matrix;  // 변환 행렬 반환
}


// 외곽(outlier) 포인트를 격자(grid)에 추가하는 함수
void addPassOutliersToGrid(const pcl::PointXYZ& point) {
    GridIndex grid_index = calculateGridIndex(point, PARA_GRID_SIZE_MAP);  // 포인트의 그리드 인덱스 계산
    if (_grid_raw_map[grid_index].size() < PARA_MAX_PTS_PER_MAP_GRID) {  // 맵 내 포인트 수 제한 확인
        _grid_raw_map[grid_index].push_back(point);  // 해당 격자에 포인트 추가
        _raw_map->points.push_back(point);  // 원시 맵(raw map)에 포인트 추가
    }
}

// 내부(inlier) 포인트를 격자(grid)에 추가하는 함수
void addPassInliersToGrid(const pcl::PointXYZ& point) {
    GridIndex grid_index = calculateGridIndex(point, PARA_GRID_SIZE_MAP);  // 포인트의 그리드 인덱스 계산

    // 원시(raw) 맵에 포인트 추가 (포인트 수 제한 확인)
    if (_grid_raw_map[grid_index].size() < PARA_MAX_PTS_PER_MAP_GRID) {
        _grid_raw_map[grid_index].push_back(point);
        _raw_map->points.push_back(point);
    }

    // 필터링된(pass) 맵에 포인트 추가 (포인트 수 제한 확인)
    if (_grid_pass_map[grid_index].size() < PARA_MAX_PTS_PER_MAP_GRID) {
        _grid_pass_map[grid_index].push_back(point);
        _pass_map->points.push_back(point);
    }
}

// 지면(ground) 포인트를 격자(grid)에 추가하는 함수
void addGroundToGrid(const pcl::PointXYZ& point) {
    GridIndex grid_index = calculateGridIndex(point, PARA_GRID_SIZE_GROUND);  // 포인트의 그리드 인덱스 계산

    // 지면 데이터 저장 (최소 포인트 개수 제한 확인)
    if (_grid_ground[grid_index].size() < PARA_MIN_PTS_PER_GROUND_GRID) {
        _grid_ground[grid_index].push_back(point);

        // 특정 개수만큼 포인트가 채워졌을 때 후보 포인트에 추가
        if (_grid_ground[grid_index].size() == PARA_MIN_PTS_PER_GROUND_GRID) {
            pcl::PointXYZ pt;
            pt.x = (grid_index.x + 0.5) * PARA_GRID_SIZE_GROUND;  // 격자 중심 x 좌표 설정
            pt.y = (grid_index.y + 0.5) * PARA_GRID_SIZE_GROUND;  // 격자 중심 y 좌표 설정
            pt.z = point.z;  // z 좌표 유지
            _candidate_pts->points.push_back(std::move(pt));  // 후보 포인트 리스트에 추가
        }
    }
}

// 포인트 클라우드에서 특정 범위 내의 포인트를 추출하는 필터 함수
void passThrough(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_inlier, PointCloudPtr& cloud_outlier,
    const float min_x, const float max_x, const float min_y, const float max_y, const float min_z, const float max_z) {
    
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());  // 유효한 포인트들의 인덱스를 저장할 포인터

    // 입력 포인트 클라우드를 순회하며 특정 범위 내의 포인트를 선택
    for (int i = 0; i < cloud_in->size(); ++i) {
        const pcl::PointXYZ& pt = cloud_in->points[i];

        // x, y, z 좌표 범위를 벗어나는 경우 제외
        if (pt.z < min_z || pt.z > max_z || pt.y < min_y || pt.y > max_y 
            || pt.x < min_x || pt.x > max_x) {
            continue;
        }

        inliers_ptr->indices.push_back(i);  // 유효한 포인트의 인덱스 추가
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;  // 포인트 인덱스를 기반으로 필터링할 객체 생성
    extract.setInputCloud(cloud_in);  // 입력 클라우드 설정
    extract.setIndices(inliers_ptr);  // 선택된 인덱스 설정

    extract.setNegative(false);  // 선택된 포인트 유지
    extract.filter(*cloud_inlier);  // inlier 클라우드 생성

    extract.setNegative(true);  // 선택된 포인트 제외
    extract.filter(*cloud_outlier);  // outlier 클라우드 생성

    return;
}

// LiDAR 스캔 데이터에서 지면을 분할하는 함수
void segmentGroundFromScan(const PointCloudPtr& cloud_in) {
    _ground_scan.reset(new PointCloud);  // 기존의 지면 포인트 클라우드 데이터를 초기화
    
    std::vector<int> labels;  // 각 포인트의 지면 여부를 저장할 라벨 벡터

    _ground_segmenter.segment(*cloud_in, &labels);  // 입력된 포인트 클라우드를 기반으로 지면 분할 수행

    // 라벨 벡터를 확인하여 지면(ground)으로 분류된 포인트만 저장
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] == 1) {  // 라벨이 1이면 지면으로 판단
            _ground_scan->push_back(cloud_in->points[i]);  // 지면 포인트 클라우드에 추가
        }
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "mapping");  // ROS 노드 초기화
    ros::NodeHandle nh("~");  // ROS 노드 핸들 생성 (네임스페이스 포함)

    // launch 파일에서 설정된 매개변수들을 ROS 파라미터 서버에서 가져오기
    nh.getParam("PARA_GRID_SIZE_MAP", PARA_GRID_SIZE_MAP);  // 맵 격자 크기
    nh.getParam("PARA_MIN_NOVE", PARA_MIN_NOVE);  // 키프레임 생성 최소 이동 거리
    nh.getParam("PARA_MAX_PTS_PER_MAP_GRID", PARA_MAX_PTS_PER_MAP_GRID);  // 맵 격자당 최대 포인트 개수
    nh.getParam("PARA_MIN_PTS_PER_GROUND_GRID", PARA_MIN_PTS_PER_GROUND_GRID);  // 지면 격자당 최소 포인트 개수
    nh.getParam("PARA_MIN_Z_MAP", PARA_MIN_Z_MAP);  // 맵에서 최소 Z값
    nh.getParam("PARA_MAX_Z_MAP", PARA_MAX_Z_MAP);  // 맵에서 최대 Z값
    nh.getParam("PARA_GRID_SIZE_GROUND", PARA_GRID_SIZE_GROUND);  // 지면 격자 크기
    nh.getParam("PARA_DIR_MAP", PARA_DIR_MAP);  // 맵 저장 디렉토리
    nh.getParam("PARA_VOXEL_LEAF", PARA_VOXEL_LEAF);  // Voxel 필터 크기
    nh.getParam("PARA_SENSOR_HEIGHT", PARA_SENSOR_HEIGHT);  // 센서 높이 설정

    _ground_params.setSensorHeight(PARA_SENSOR_HEIGHT);  // 지면 분할에 사용할 센서 높이 설정

    // LiDAR 스캔 및 Pose 데이터를 구독하는 subscriber 설정
    ros::Subscriber sub_scan = nh.subscribe("/velodyne_points", 1000, recieveCloud);
    ros::Subscriber sub_pose = nh.subscribe("/lidar_pose", 1000, recievePose);

    // 포인트 클라우드 데이터를 발행하는 publisher 설정
    ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map_raw_cloud", 1);
    ros::Publisher pub_pass_map = nh.advertise<sensor_msgs::PointCloud2>("/map_pass_cloud", 1);
    ros::Publisher pub_candidate = nh.advertise<sensor_msgs::PointCloud2>("/candidate_pts", 1);

    // 시작 메시지 출력
    std::cout << "==================================================\n";
    std::cout << "||                                              ||\n";
    std::cout << "||             Begin Offline Mapping            ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "||       Waiting for LiDAR scans to arrive      ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "==================================================\n";

    int keyframe_count = 0;  // 키프레임 개수 초기화
    Eigen::Vector3f curr_xyz = Eigen::Vector3f::Zero();  // 현재 위치 초기화
    Eigen::Vector3f last_xyz = Eigen::Vector3f::Zero();  // 마지막 위치 초기화

    ros::Rate loop_rate(20);  // 루프 실행 속도 설정 (20Hz)
    ros::Time last_publish_time = ros::Time::now();  // 마지막 발행 시간 기록

    while (ros::ok()) {
        ros::spinOnce();  // ROS 이벤트 처리 (콜백 실행)

        // 6초 동안 데이터가 없으면 맵을 저장
        if ((ros::Time::now() - last_publish_time).toSec() > 6 && _cloud_buf.empty() && _pose_buf.empty()) {
            if (!_pass_map->empty() && !_candidate_pts->empty() && !_raw_map->empty()) {
                ROS_WARN("No message publish for 6 seconds. Saving map...");
                std::cout << "Candidates contains " << static_cast<int>(_candidate_pts->size()) << " pts...\n";
                std::cout << "Pass map contains " << static_cast<int>(_pass_map->size()) << " pts..\n";
                std::cout << "Raw map contains " << static_cast<int>(_raw_map->size()) << " pts..\n";

                // 포인트 클라우드 크기 설정
                _candidate_pts->width = static_cast<uint32_t>(_candidate_pts->size());
                _candidate_pts->height = 1;
                _candidate_pts->is_dense = true;
                _pass_map->width = static_cast<uint32_t>(_pass_map->size());
                _pass_map->height = 1;
                _pass_map->is_dense = true;
                _raw_map->width = static_cast<uint32_t>(_raw_map->size());
                _raw_map->height = 1;
                _raw_map->is_dense = true;
                
                // 후보 포인트 클라우드 저장 (이상치 제거 후 저장)
                removeOutliersSOR(_candidate_pts, _candidate_pts, 50, 1.0);
                if (pcl::io::savePCDFileASCII(PARA_DIR_MAP + "candidate_pts.pcd", *_candidate_pts) == 0) {
                    std::cout << "Successfully saved candidates, " << _candidate_pts->size()<< " pts...\n";
                } else {
                    ROS_ERROR("Failed to save candidates in %s", (PARA_DIR_MAP + "candidate_pts.pcd").c_str());
                }
                
                // 필터링된(pass) 맵 저장 (Voxel 필터 적용 후 저장)
                voxelSampleCloud(_pass_map, _pass_map, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF);
                if (pcl::io::savePCDFileASCII(PARA_DIR_MAP + "pass_map.pcd", *_pass_map) == 0) {
                    std::cout << "Successfully saved pass map, " << _pass_map->size()<< " pts...\n";
                } else {
                    ROS_ERROR("Failed to save pass map in %s", (PARA_DIR_MAP + "pass_map.pcd").c_str());
                }

                // 원시(raw) 맵 저장 (Voxel 필터 적용 후 저장)
                voxelSampleCloud(_raw_map, _raw_map, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF);
                if (pcl::io::savePCDFileASCII(PARA_DIR_MAP + "raw_map.pcd", *_raw_map) == 0) {
                    std::cout << "Successfully saved raw map, " << _raw_map->size()<< " pts...\n";
                } else {
                    ROS_ERROR("Failed to save raw map in %s", (PARA_DIR_MAP + "raw_map.pcd").c_str());
                }

                // 맵 데이터 초기화
                _raw_map->clear();
                _pass_map->clear();
                _candidate_pts->clear();
            }
        }

        // LiDAR 스캔 또는 위치 정보가 없으면 계속 대기
        if (_cloud_buf.empty() || _pose_buf.empty()) {
            continue;
        }
        
        // LiDAR 데이터와 Pose 데이터의 타임스탬프 동기화 확인
        double time_cloud = _cloud_buf.front()->header.stamp.toSec();  // LiDAR 데이터의 타임스탬프
        double time_pose = _pose_buf.front()->header.stamp.toSec();  // Pose 데이터의 타임스탬프
        if (time_cloud != time_pose) {  // 시간 불일치 발생 시 오류 출력 후 종료
            std::printf("unsync messeage!");
            ROS_BREAK();
        } 
        
        // 버퍼 접근을 위한 뮤텍스 잠금
        mBuf.lock();
        _scan_cloud->clear();  // 기존 포인트 클라우드 초기화
        pcl::fromROSMsg(*_cloud_buf.front(), *_scan_cloud);  // ROS 메시지를 PCL 포인트 클라우드로 변환
        _cloud_buf.pop();  // 처리된 LiDAR 데이터 제거

        geometry_msgs::PoseStamped::ConstPtr pose_msg = _pose_buf.front();  // Pose 메시지 가져오기
        _scan_pose = transformMsgToMatrix(pose_msg);  // Pose 데이터를 변환 행렬로 변환
        _pose_buf.pop();  // 처리된 Pose 데이터 제거
        mBuf.unlock();  // 뮤텍스 해제
        
        // 키프레임 생성: 최소 이동 거리(PARA_MIN_NOVE) 이상 이동해야 생성
        curr_xyz = _scan_pose.block<3, 1>(0, 3);  // 현재 위치 추출
        if ((curr_xyz - last_xyz).norm() < PARA_MIN_NOVE) {  
            last_publish_time = ros::Time::now();  // 마지막 발행 시간 초기화
            continue;
        } else {
            ++keyframe_count;  // 새로운 키프레임 개수 증가
            last_xyz = curr_xyz;  // 현재 위치를 마지막 키프레임 위치로 업데이트
        }
        
        TicToc tic_total;  // 성능 측정을 위한 타이머 시작

        // 근거리 및 원거리 포인트 제거 (5m ~ 50m 범위 유지)
        removeCloseAndFarPts(*_scan_cloud, *_scan_cloud, 5.0, 50.0);

        // LineFit 알고리즘을 이용한 지면 분할 수행
        segmentGroundFromScan(_scan_cloud);
        pcl::transformPointCloud(*_ground_scan, *_ground_scan, _scan_pose);  // 지면 포인트 좌표 변환
        for (const auto& pt : _ground_scan->points) {  // 지면 포인트를 그리드에 추가
            addGroundToGrid(pt);
        }
        
        // Voxel 필터 적용 및 특정 범위 내 포인트 필터링 수행
        voxelSampleCloud(_scan_cloud, _scan_cloud, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF, PARA_VOXEL_LEAF);
        PointCloudPtr _scan_pass_in(new PointCloud), _scan_pass_out(new PointCloud);
        passThrough(_scan_cloud, _scan_pass_in, _scan_pass_out, -80, 80, -80, 80, PARA_MIN_Z_MAP, PARA_MAX_Z_MAP);

        // 필터링된 포인트 클라우드를 좌표 변환 후 맵에 추가
        pcl::transformPointCloud(*_scan_pass_in, *_scan_pass_in, _scan_pose);
        for (const auto& pt : _scan_pass_in->points) {
            addPassInliersToGrid(pt);
        }
        pcl::transformPointCloud(*_scan_pass_out, *_scan_pass_out, _scan_pose);
        for (const auto& pt : _scan_pass_out->points) {
            addPassOutliersToGrid(pt);
        }

        // 키프레임 정보 및 맵 크기 출력
        std::cout << keyframe_count << "th keyframe, takes: " << tic_total.toc() << "ms\n";
        std::cout << "Raw map:  " << _raw_map->size() << "\n";
        std::cout << "Pass map: " << _pass_map->size() << "\n";

        // 후보 포인트 클라우드 메시지 발행
        sensor_msgs::PointCloud2 msg_candidate;
        pcl::toROSMsg(*_candidate_pts, msg_candidate);
        msg_candidate.header.stamp = ros::Time::now();
        msg_candidate.header.frame_id = "map";
        pub_candidate.publish(msg_candidate);  

        // 원시(raw) 맵 포인트 클라우드 메시지 발행
        sensor_msgs::PointCloud2 msg_raw_map;
        pcl::toROSMsg(*_raw_map, msg_raw_map);
        msg_raw_map.header.stamp = ros::Time::now();
        msg_raw_map.header.frame_id = "map";  
        pub_map.publish(msg_raw_map);        

        // 필터링된(pass) 맵 포인트 클라우드 메시지 발행
        sensor_msgs::PointCloud2 msg_pass_map;
        pcl::toROSMsg(*_pass_map, msg_pass_map);
        msg_pass_map.header.stamp = ros::Time::now();
        msg_pass_map.header.frame_id = "map"; 
        pub_pass_map.publish(msg_pass_map);       
           
        last_publish_time = ros::Time::now();  // 마지막 발행 시간 초기화
        loop_rate.sleep();  // 루프 속도 조절
    }
    return 0;
}

