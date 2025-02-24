#include "offline_mapping/linefit/ground_segmentation.h"  // GroundSegmentation 클래스의 선언을 포함합니다.
#include <chrono>  // 시간 측정을 위한 라이브러리입니다.
#include <cmath>  // 수학 연산을 수행하기 위한 라이브러리입니다.
#include <list>  // 리스트 컨테이너를 사용하기 위한 라이브러리입니다.
#include <memory>  // 스마트 포인터(std::shared_ptr 등)를 사용하기 위한 라이브러리입니다.
#include <thread>  // 멀티스레딩을 지원하는 라이브러리입니다.
#include <boost/date_time/posix_time/posix_time.hpp>  // Boost 라이브러리에서 날짜 및 시간 처리를 위한 헤더입니다.
#include <boost/thread.hpp>  // Boost의 스레드 라이브러리를 포함합니다.

void GroundSegmentation::visualizePointCloud(const PointCloud::ConstPtr& cloud,
                                             const std::string& id) {
  // 주어진 포인트 클라우드를 시각화 도구(viewer)에 추가합니다.
  viewer_->addPointCloud(cloud, id, 0);
}

void GroundSegmentation::visualizeLines(const std::list<PointLine>& lines) {
  // 선을 시각화하는 함수입니다.
  size_t counter = 0;  // 선의 개수를 추적하는 변수입니다.
  for (auto it = lines.begin(); it != lines.end(); ++it) {
    // 각 선을 viewer에 추가하며, 고유한 ID를 문자열(counter)로 변환하여 설정합니다.
    viewer_->addLine<pcl::PointXYZ>(it->first, it->second, std::to_string(counter++));
  }
}

void GroundSegmentation::visualize(const std::list<PointLine>& lines,
                                   const PointCloud::ConstPtr& min_cloud,
                                   const PointCloud::ConstPtr& ground_cloud,
                                   const PointCloud::ConstPtr& obstacle_cloud) {
  // 포인트 클라우드 및 라인을 시각화하는 함수입니다.

  viewer_->setBackgroundColor(0, 0, 0);  // 배경색을 검은색으로 설정합니다.
  viewer_->addCoordinateSystem(1.0);  // 좌표축을 추가합니다.
  viewer_->initCameraParameters();  // 카메라 매개변수를 초기화합니다.
  viewer_->setCameraPosition(-2.0, 0, 2.0, 1.0, 0, 0);  // 카메라 위치를 설정합니다.

  visualizePointCloud(min_cloud, "min_cloud");  // 최소 높이 포인트 클라우드를 시각화합니다.
  viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                            0.0f, 1.0f, 0.0f,  // 초록색
                                            "min_cloud");
  viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                            2.0f,
                                            "min_cloud");

  visualizePointCloud(ground_cloud, "ground_cloud");  // 지면 포인트 클라우드를 시각화합니다.
  viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                            1.0f, 0.0f, 0.0f,  // 빨간색
                                            "ground_cloud");

  visualizePointCloud(obstacle_cloud, "obstacle_cloud");  // 장애물 포인트 클라우드를 시각화합니다.
  visualizeLines(lines);  // 감지된 선을 시각화합니다.

  while (!viewer_->wasStopped()) {
    // 사용자가 시각화 창을 닫기 전까지 계속 실행합니다.
    viewer_->spinOnce(100);  // 100ms마다 화면을 갱신합니다.
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));  // 100ms 동안 대기합니다.
  }
}

GroundSegmentation::GroundSegmentation(const GroundSegmentationParams& params) :
    params_(params),
    segments_(params.n_segments, Segment(params.n_bins,
                                         params.max_slope,
                                         params.max_error_square,
                                         params.long_threshold,
                                         params.max_long_height,
                                         params.max_start_height,
                                         params.sensor_height)) {
  // GroundSegmentation 클래스의 생성자입니다.
  // 'params_'에 입력 매개변수를 저장하고, segments_를 설정하여 각 세그먼트의 초기화를 수행합니다.

  if (params.visualize) {
    // 시각화가 활성화된 경우, PCLVisualizer를 생성합니다.
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  }
void GroundSegmentation::segment(const PointCloud& cloud, std::vector<int>* segmentation) {
  // 지면 분할을 수행하는 함수입니다.

  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  // 현재 시간을 기록하여 성능 측정을 시작합니다.

  segmentation->clear();  // segmentation 벡터를 초기화합니다.
  segmentation->resize(cloud.size(), 0);  // 포인트 클라우드 크기만큼 0으로 초기화합니다.
  bin_index_.resize(cloud.size());  // 포인트 클라우드 크기만큼 bin 인덱스 배열을 초기화합니다.
  segment_coordinates_.resize(cloud.size());  // 세그먼트 좌표 배열을 초기화합니다.

  insertPoints(cloud);  // 입력된 포인트 클라우드를 삽입하여 초기 데이터를 설정합니다.

  std::list<PointLine> lines;
  if (params_.visualize) {
    getLines(&lines);  // 시각화가 필요하면 라인 정보를 가져옵니다.
  }
  else {
    getLines(NULL);  // 시각화가 필요하지 않으면 NULL을 전달합니다.
  }

  assignCluster(segmentation);  // 포인트 클라우드의 각 포인트에 대해 클러스터를 할당합니다.

  PointCloud::Ptr ground_cloud(new PointCloud());  // 지면 포인트 클라우드를 저장할 변수입니다.
  if (params_.visualize) {
    // 시각화가 활성화된 경우 실행됩니다.

    PointCloud::Ptr obstacle_cloud(new PointCloud());  // 장애물 포인트 클라우드를 저장할 변수입니다.

    // 지면과 장애물 포인트를 분류합니다.
    for (size_t i = 0; i < cloud.size(); ++i) {
      if (segmentation->at(i) == 1) ground_cloud->push_back(cloud[i]);  // 지면 포인트 추가
      else obstacle_cloud->push_back(cloud[i]);  // 장애물 포인트 추가
    }

    PointCloud::Ptr min_cloud(new PointCloud());  // 최소 z 값을 가지는 포인트 클라우드를 저장할 변수입니다.
    getMinZPointCloud(min_cloud.get());  // 최소 높이 포인트 클라우드를 추출합니다.
    visualize(lines, min_cloud, ground_cloud, obstacle_cloud);  // 시각화를 수행합니다.
  }

  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> fp_ms = end - start;
  // 성능 측정을 종료하고 경과 시간을 계산합니다.

  // std::cout << "Done! Took " << fp_ms.count() << "ms\n";  // 수행 시간 출력 (현재는 주석 처리됨)
  // std::cout << "Ground Points " << ground_cloud->size() << "\n";  // 지면 포인트 개수 출력 (현재는 주석 처리됨)
}

void GroundSegmentation::getLines(std::list<PointLine> *lines) {
  // 다중 스레드를 활용하여 각 세그먼트에서 라인을 추출하는 함수입니다.

  std::mutex line_mutex;  // 여러 스레드에서 동기화하기 위한 뮤텍스 객체입니다.
  std::vector<std::thread> thread_vec(params_.n_threads);  // 사용할 스레드 벡터를 선언합니다.
  
  unsigned int i;
  for (i = 0; i < params_.n_threads; ++i) {
    const unsigned int start_index = params_.n_segments / params_.n_threads * i;  // 스레드 시작 인덱스
    const unsigned int end_index = params_.n_segments / params_.n_threads * (i+1);  // 스레드 종료 인덱스

    // 새로운 스레드를 생성하여 lineFitThread 함수 실행
    thread_vec[i] = std::thread(&GroundSegmentation::lineFitThread, this,
                                start_index, end_index, lines, &line_mutex);
  }

  for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
    it->join();  // 모든 스레드가 완료될 때까지 기다립니다.
  }
}

void GroundSegmentation::lineFitThread(const unsigned int start_index,
                                       const unsigned int end_index,
                                       std::list<PointLine> *lines, std::mutex* lines_mutex) {
  // 특정 세그먼트 범위에서 선형 적합(Line Fitting)을 수행하는 스레드 함수입니다.

  const bool visualize = lines;  // lines가 존재하면 시각화를 수행합니다.
  const double seg_step = 2*M_PI / params_.n_segments;  // 세그먼트별 각도 증가량을 계산합니다.
  double angle = -M_PI + seg_step/2 + seg_step * start_index;  // 시작 각도를 설정합니다.

  for (unsigned int i = start_index; i < end_index; ++i) {
    segments_[i].fitSegmentLines();  // 해당 세그먼트에서 선형 적합을 수행합니다.

    // 시각화를 수행해야 하는 경우
    if (visualize) {
      std::list<Segment::Line> segment_lines;  // 현재 세그먼트의 선들을 저장할 리스트입니다.
      segments_[i].getLines(&segment_lines);  // 세그먼트에서 라인을 가져옵니다.

      for (auto line_iter = segment_lines.begin(); line_iter != segment_lines.end(); ++line_iter) {
        const pcl::PointXYZ start = minZPointTo3d(line_iter->first, angle);  // 시작점 변환
        const pcl::PointXYZ end = minZPointTo3d(line_iter->second, angle);  // 종료점 변환
        
        lines_mutex->lock();  // 뮤텍스를 사용하여 데이터 경합을 방지합니다.
        lines->emplace_back(start, end);  // 변환된 3D 선을 리스트에 추가합니다.
        lines_mutex->unlock();  // 뮤텍스를 해제합니다.
      }

      angle += seg_step;  // 각도를 다음 세그먼트로 증가시킵니다.
    }
  }
}

void GroundSegmentation::getMinZPointCloud(PointCloud* cloud) {
  // 최소 z 값을 가지는 포인트 클라우드를 생성하는 함수입니다.

  const double seg_step = 2*M_PI / params_.n_segments;  // 각 세그먼트의 각도 증가량을 계산합니다.
  double angle = -M_PI + seg_step/2;  // 초기 각도를 설정합니다.

  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    // 각 세그먼트에 대해 반복합니다.
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter) {
      // 각 세그먼트 내의 Bin에 대해 반복합니다.
      const pcl::PointXYZ min = minZPointTo3d(bin_iter->getMinZPoint(), angle);
      // 최소 z 값을 가지는 3D 포인트를 변환합니다.
      cloud->push_back(min);  // 변환된 포인트를 클라우드에 추가합니다.
    }

    angle += seg_step;  // 다음 세그먼트로 이동하면서 각도를 증가시킵니다.
  }
}

pcl::PointXYZ GroundSegmentation::minZPointTo3d(const Bin::MinZPoint &min_z_point,
                                                const double &angle) {
  // MinZPoint를 3D 좌표로 변환하는 함수입니다.

  pcl::PointXYZ point;
  point.x = cos(angle) * min_z_point.d;  // x 좌표를 계산합니다.
  point.y = sin(angle) * min_z_point.d;  // y 좌표를 계산합니다.
  point.z = min_z_point.z;  // z 좌표는 그대로 유지합니다.

  return point;  // 변환된 3D 포인트를 반환합니다.
}

void GroundSegmentation::assignCluster(std::vector<int>* segmentation) {
  // 포인트 클라우드에서 각 점을 클러스터에 할당하는 함수입니다.

  std::vector<std::thread> thread_vec(params_.n_threads);  // 다중 스레드 벡터를 선언합니다.
  const size_t cloud_size = segmentation->size();  // 포인트 클라우드의 크기를 저장합니다.

  for (unsigned int i = 0; i < params_.n_threads; ++i) {
    // 각 스레드가 처리할 데이터 범위를 계산합니다.
    const unsigned int start_index = cloud_size / params_.n_threads * i;
    const unsigned int end_index = cloud_size / params_.n_threads * (i+1);

    // assignClusterThread 함수를 스레드로 실행합니다.
    thread_vec[i] = std::thread(&GroundSegmentation::assignClusterThread, this,
                                start_index, end_index, segmentation);
  }

  for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
    it->join();  // 모든 스레드가 종료될 때까지 기다립니다.
  }
} 

void GroundSegmentation::assignClusterThread(const unsigned int &start_index,
                                             const unsigned int &end_index,
                                             std::vector<int> *segmentation) {
  // 특정 범위의 포인트를 클러스터에 할당하는 스레드 함수입니다.

  const double segment_step = 2*M_PI/params_.n_segments;  // 세그먼트 간 각도 차이를 계산합니다.

  for (unsigned int i = start_index; i < end_index; ++i) {
    // 각 포인트에 대해 반복합니다.

    Bin::MinZPoint point_2d = segment_coordinates_[i];  // 포인트의 2D 좌표를 가져옵니다.
    const int segment_index = bin_index_[i].first;  // 해당 포인트가 속한 세그먼트 인덱스를 가져옵니다.

    if (segment_index >= 0) {
      // 유효한 세그먼트인 경우 실행합니다.

      double dist = segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);
      // 현재 세그먼트의 수직 거리(distance)를 계산합니다.

      int steps = 1;  // 인접 세그먼트 검색을 위한 단계 수를 설정합니다.
      while (dist == -1 && steps * segment_step < params_.line_search_angle) {
        // 거리가 유효하지 않은 경우, 인접한 세그먼트를 검색합니다.

        int index_1 = segment_index + steps;  // 정방향으로 검색할 인덱스입니다.
        while (index_1 >= params_.n_segments) index_1 -= params_.n_segments;  // 인덱스가 초과되면 보정합니다.

        int index_2 = segment_index - steps;  // 역방향으로 검색할 인덱스입니다.
        while (index_2 < 0) index_2 += params_.n_segments;  // 인덱스가 음수가 되면 보정합니다.

        // 인접 세그먼트에서 수직 거리를 계산합니다.
        const double dist_1 = segments_[index_1].verticalDistanceToLine(point_2d.d, point_2d.z);
        const double dist_2 = segments_[index_2].verticalDistanceToLine(point_2d.d, point_2d.z);

        // 유효한 거리 중 더 큰 값을 선택합니다.
        if (dist_1 > dist) {
          dist = dist_1;
        }
        if (dist_2 > dist) {
          dist = dist_2;
        }
        ++steps;  // 다음 단계로 이동합니다.
      }

      if (dist < params_.max_dist_to_line && dist != -1) {
        // 거리가 임계값 이하이면 해당 포인트를 지면(ground)으로 분류합니다.
        segmentation->at(i) = 1;
      }
    }
  }
}

void GroundSegmentation::getMinZPoints(PointCloud* out_cloud) {
  // 각 세그먼트와 Bin에서 최소 z 값을 가지는 포인트를 추출하여 클라우드에 저장하는 함수입니다.

  const double seg_step = 2*M_PI / params_.n_segments;  // 세그먼트별 각도 증가량을 계산합니다.
  const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square))
      / params_.n_bins;  // Bin의 반경 증가량을 계산합니다.

  const double r_min = sqrt(params_.r_min_square);  // 최소 반경을 계산합니다.
  double angle = -M_PI + seg_step/2;  // 시작 각도를 설정합니다.

  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    // 세그먼트별로 반복합니다.

    double dist = r_min + bin_step/2;  // 초기 거리 값을 설정합니다.
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter) {
      // 각 Bin을 순회하면서 최소 z 값을 가지는 포인트를 추출합니다.

      pcl::PointXYZ point;
      if (bin_iter->hasPoint()) {  
        // Bin에 유효한 포인트가 있는 경우 실행합니다.

        Bin::MinZPoint min_z_point(bin_iter->getMinZPoint());  
        // 최소 z 값을 가지는 포인트를 가져옵니다.

        point.x = cos(angle) * min_z_point.d;  // x 좌표를 계산합니다.
        point.y = sin(angle) * min_z_point.d;  // y 좌표를 계산합니다.
        point.z = min_z_point.z;  // z 좌표는 그대로 유지합니다.

        out_cloud->push_back(point);  // 변환된 3D 포인트를 클라우드에 추가합니다.
      }
      dist += bin_step;  // 다음 Bin의 거리 값을 증가시킵니다.
    }
    angle += seg_step;  // 다음 세그먼트로 이동하면서 각도를 증가시킵니다.
  }
}

/(포인트 클라우드 삽입) 
void GroundSegmentation::insertPoints(const PointCloud& cloud) {
  // 포인트 클라우드를 다중 스레드를 활용하여 삽입하는 함수입니다.

  std::vector<std::thread> threads(params_.n_threads);  // 다중 스레드 벡터를 선언합니다.
  const size_t points_per_thread = cloud.size() / params_.n_threads;  
  // 각 스레드가 처리할 포인트 개수를 계산합니다.

  // std::cout << "PC: points size per thread: " << points_per_thread << "\n";  // 디버깅 출력

  // 스레드를 실행하여 포인트 삽입을 병렬 처리합니다.
  for (unsigned int i = 0; i < params_.n_threads - 1; ++i) {
    const size_t start_index = i * points_per_thread;  // 시작 인덱스를 설정합니다.
    const size_t end_index = (i+1) * points_per_thread - 1;  // 종료 인덱스를 설정합니다.

    threads[i] = std::thread(&GroundSegmentation::insertionThread, this,
                             cloud, start_index, end_index);
  }

  // 마지막 스레드는 나머지 포인트를 처리해야 하므로 범위를 조정합니다.
  const size_t start_index = (params_.n_threads - 1) * points_per_thread;
  const size_t end_index = cloud.size() - 1;

  threads[params_.n_threads - 1] =
      std::thread(&GroundSegmentation::insertionThread, this, cloud, start_index, end_index);

  // 모든 스레드가 종료될 때까지 기다립니다.
  for (auto it = threads.begin(); it != threads.end(); ++it) {
    it->join();
  }
}

void GroundSegmentation::insertionThread(const PointCloud& cloud,
                                         const size_t start_index,
                                         const size_t end_index) {
  // 포인트 클라우드를 적절한 세그먼트와 Bin에 삽입하는 스레드 함수입니다.

  const double segment_step = 2*M_PI / params_.n_segments;  
  // 세그먼트별 각도 증가량을 계산합니다.

  const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square))
      / params_.n_bins;  
  // Bin의 반경 증가량을 계산합니다.

  const double r_min = sqrt(params_.r_min_square);  
  // 최소 반경을 계산합니다.

  for (unsigned int i = start_index; i < end_index; ++i) {
    // 지정된 인덱스 범위의 포인트를 처리합니다.

    // std::cout << "idx: " << i << std::endl;  // 디버깅 출력

    pcl::PointXYZ point(cloud[i]);  // 현재 포인트를 가져옵니다.
    const double range_square = point.x * point.x + point.y * point.y;  
    // 포인트의 반경의 제곱을 계산합니다.

    const double range = sqrt(range_square);  
    // 포인트의 실제 반경 값을 계산합니다.

    // std::cout << "PC: 0 \n";  // 디버깅 출력

    if (range_square < params_.r_max_square && range_square > params_.r_min_square) {
      // 포인트가 유효한 거리 범위 내에 있는 경우 처리합니다.

      const double angle = std::atan2(point.y, point.x);  
      // x, y 좌표를 이용하여 각도를 계산합니다.

      const unsigned int bin_index = (range - r_min) / bin_step;  
      // 포인트가 속한 Bin 인덱스를 계산합니다.

      const unsigned int segment_index = (angle + M_PI) / segment_step;  
      // 포인트가 속한 세그먼트 인덱스를 계산합니다.

      // ShiPC 방법으로 인덱스를 확인합니다.
      if (segment_index >= params_.n_segments) {
        // 세그먼트 인덱스가 초과하는 경우, 첫 번째 세그먼트에 추가합니다.
        segments_[0][bin_index].addPoint(range, point.z);
        continue;
      }

      segments_[segment_index][bin_index].addPoint(range, point.z);  
      // 해당 세그먼트와 Bin에 포인트를 추가합니다.

      bin_index_[i] = std::make_pair(segment_index, bin_index);  
      // 해당 포인트의 Bin 인덱스를 저장합니다.
    }
    else {
      bin_index_[i] = std::make_pair<int, int>(-1, -1);  
      // 유효한 범위 내에 없는 경우 (-1, -1)로 설정합니다.
    }

    segment_coordinates_[i] = Bin::MinZPoint(range, point.z);  
    // 포인트의 2D 거리와 z 값을 저장합니다.
  }
}

