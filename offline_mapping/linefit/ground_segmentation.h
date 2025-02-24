#ifndef _LIDAR100_LINEFIT_GROUND_SEGMENTATION_H_
#define _LIDAR100_LINEFIT_GROUND_SEGMENTATION_H_

#include <mutex>  // 다중 스레드 환경에서 데이터 동기화를 위한 뮤텍스를 포함합니다.
#include <thread>  // 다중 스레드 처리를 위한 표준 라이브러리를 포함합니다.
#include <glog/logging.h>  // Google Logging 라이브러리를 포함하여 로깅 기능을 지원합니다.
#include <pcl/point_cloud.h>  // PCL(Point Cloud Library)의 포인트 클라우드 기능을 포함합니다.
#include <pcl/point_types.h>  // PCL에서 제공하는 다양한 포인트 타입을 사용하기 위해 포함합니다.
#include <pcl/visualization/pcl_visualizer.h>  // PCL 시각화를 위한 라이브러리를 포함합니다.
#include "offline_mapping/linefit/segment.h"  // 세그먼트 처리를 위한 사용자 정의 헤더 파일을 포함합니다.



struct GroundSegmentationParams {
  GroundSegmentationParams() :
      // visualize(true),  // 시각화를 활성화할 경우 true로 설정 (현재는 비활성화)
      visualize(false),  // 시각화를 비활성화 (기본값)
      r_min_square(0.3 * 0.3),  // 최소 반경의 제곱 (0.3m의 제곱)
      r_max_square(20 * 20),  // 최대 반경의 제곱 (20m의 제곱)
      n_bins(30),  // 각 세그먼트당 Bin의 개수
      n_segments(180),  // 총 세그먼트 개수 (0~360도 범위를 180개로 나눔)
      max_dist_to_line(0.15),  // 포인트를 선형 모델과 비교할 때 허용되는 최대 거리
      max_slope(0.2),  // 선형 모델에서 허용되는 최대 기울기
      n_threads(4),  // 다중 스레드 사용 시 생성할 스레드 개수
      max_error_square(0.01),  // 선형 모델과 포인트 간 최대 허용 오차의 제곱 값
      long_threshold(2.0),  // 장거리 포인트의 임계값 (이보다 크면 분리됨)
      max_long_height(0.1),  // 장거리 포인트에서 허용되는 최대 높이 차이
      max_start_height(0.2),  // 시작 높이에서 허용되는 최대 값
      sensor_height(1.75),  // 센서의 높이 (기본값 1.75m)
      line_search_angle(0.2) {}  // 선 검색을 수행할 최대 각도 범위

    void setSensorHeight(double height) {
    sensor_height = height;  // 센서의 높이를 설정하는 함수입니다.
  }

  // 지면 추정 결과를 시각화할지 여부를 설정하는 변수입니다.
  bool visualize;

  // 분할 시 최소 반경의 제곱 값입니다.
  double r_min_square;

  // 분할 시 최대 반경의 제곱 값입니다.
  double r_max_square;

  // 각 세그먼트에서 사용될 Bin의 개수입니다.
  int n_bins;

  // 원형으로 나눌 각도 세그먼트의 개수입니다.
  int n_segments;

  // 포인트가 지면으로 분류되기 위해 허용되는 최대 거리 값입니다.
  double max_dist_to_line;

  // 지면으로 간주할 수 있는 최대 기울기 값입니다.
  double max_slope;

  // 선형 모델을 적합시킬 때 허용되는 최대 오차 값입니다.
  double max_error_square;

  // 일정 거리 이상 떨어진 포인트들이 서로 분리되어야 하는 기준 거리입니다.
  double long_threshold;

  // 장거리 포인트의 최대 기울기 값입니다.
  double max_long_height;

  // 초기 선을 지면으로 라벨링할 때 허용되는 최대 높이 값입니다.
  double max_start_height;

  // 센서가 지면 위에 위치한 높이 값입니다.
  double sensor_height;

  // 선을 검색할 때 허용되는 최대 각도 범위(rad)입니다.
  double line_search_angle;

  // 다중 스레드로 사용할 스레드 개수입니다.
  int n_threads;
};


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// PCL의 포인트 클라우드 타입을 정의합니다.

typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> PointLine;
// 두 개의 3D 포인트를 연결하여 선(Line)으로 표현하는 타입을 정의합니다.

class GroundSegmentation {
  // 지면 분할을 수행하는 클래스입니다.

  const GroundSegmentationParams params_;
  // 지면 분할에 사용될 파라미터를 저장하는 상수 멤버 변수입니다.

  // segments_[segment][bin] 형태로 접근할 수 있는 벡터입니다.
  std::vector<Segment> segments_;

  // 각 포인트의 Bin 인덱스를 저장하는 벡터입니다.
  std::vector<std::pair<int, int>> bin_index_;

  // 각 포인트의 2D 좌표 (d, z)를 저장하는 벡터입니다.
  std::vector<Bin::MinZPoint> segment_coordinates_;

  // PCL 시각화를 위한 viewer 객체입니다.
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

  // 포인트 클라우드에서 각 점을 클러스터에 할당하는 함수입니다.
  void assignCluster(std::vector<int>* segmentation);

  // assignCluster()에서 사용할 다중 스레드 처리 함수입니다.
  void assignClusterThread(const unsigned int& start_index,
                           const unsigned int& end_index,
                           std::vector<int>* segmentation);

  // 입력 포인트 클라우드를 분할하여 저장하는 함수입니다.
  void insertPoints(const PointCloud& cloud);

  // insertPoints()에서 사용할 다중 스레드 처리 함수입니다.
  void insertionThread(const PointCloud& cloud,
                       const size_t start_index,
                       const size_t end_index);

  // 최소 Z 값을 가지는 포인트들을 추출하는 함수입니다.
  void getMinZPoints(PointCloud* out_cloud);

  // 세그먼트에서 선을 추출하는 함수입니다.
  void getLines(std::list<PointLine>* lines);

  // getLines()에서 사용할 다중 스레드 처리 함수입니다.
  void lineFitThread(const unsigned int start_index, const unsigned int end_index,
                     std::list<PointLine> *lines, std::mutex* lines_mutex);

  // 2D MinZPoint를 3D 좌표로 변환하는 함수입니다.
  pcl::PointXYZ minZPointTo3d(const Bin::MinZPoint& min_z_point, const double& angle);

  // 최소 Z 값을 가지는 포인트 클라우드를 생성하는 함수입니다.
  void getMinZPointCloud(PointCloud* cloud);

  // 포인트 클라우드를 시각화하는 함수입니다.
  void visualizePointCloud(const PointCloud::ConstPtr& cloud,
                           const std::string& id = "point_cloud");

  // 추출된 선을 시각화하는 함수입니다.
  void visualizeLines(const std::list<PointLine>& lines);

  // 전체 시각화(포인트 클라우드 + 선)를 수행하는 함수입니다.
  void visualize(const std::list<PointLine>& lines, const PointCloud::ConstPtr& cloud, 
                 const PointCloud::ConstPtr& ground_cloud, const PointCloud::ConstPtr& obstacle_cloud);

public:

  // GroundSegmentation 클래스의 생성자입니다.
  GroundSegmentation(const GroundSegmentationParams& params = GroundSegmentationParams());

  // 입력 포인트 클라우드를 처리하여 지면과 장애물을 분할하는 함수입니다.
  void segment(const PointCloud& cloud, std::vector<int>* segmentation);
};

#endif //_LIDAR100_LINEFIT_GROUND_SEGMENTATION_H_

