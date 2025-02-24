#ifndef _LIDAR100_LINEFIT_BIN_H_SEGMENT_H_
#define _LIDAR100_LINEFIT_BIN_H_SEGMENT_H_

#include <list>  // 리스트 컨테이너를 포함하는 헤더 파일
#include <map>  // 맵 컨테이너를 포함하는 헤더 파일
#include "offline_mapping/linefit/bin.h"  // Bin 클래스가 정의된 헤더 파일 포함

// Segment 클래스 정의
class Segment {

public:
  // Line 타입 정의: 두 개의 Bin::MinZPoint를 한 쌍으로 가지는 pair
  typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line;
  // LocalLine 타입 정의: 두 개의 double 값을 한 쌍으로 가지는 pair
  typedef std::pair<double, double> LocalLine;

private:
  // 클래스 내부에서 사용할 파라미터들. GroundSegmentation 클래스에서 설명됨.
  const double max_slope_;  // 최대 기울기 허용값
  const double max_error_;  // 최대 오차 허용값
  const double long_threshold_;  // 긴 세그먼트로 판단하는 임계값
  const double max_long_height_;  // 긴 세그먼트의 최대 높이
  const double max_start_height_;  // 초기 시작점의 최대 높이
  const double sensor_height_;  // 센서의 높이

  std::vector<Bin> bins_;  // Bin 객체들을 저장하는 벡터
  std::list<Line> lines_;  // 감지된 선분(Line)들을 저장하는 리스트

  // 주어진 포인트 리스트를 기반으로 로컬 선분을 피팅하는 함수
  LocalLine fitLocalLine(const std::list<Bin::MinZPoint>& points);
  
  // 주어진 로컬 선분과 포인트 리스트의 평균 오차를 계산하는 함수
  double getMeanError(const std::list<Bin::MinZPoint>& points, const LocalLine& line);
  
  // 주어진 로컬 선분과 포인트 리스트의 최대 오차를 계산하는 함수
  double getMaxError(const std::list<Bin::MinZPoint>& points, const LocalLine& line);
  
  // 로컬 좌표계에서 구한 선을 전역 좌표계의 Line 타입으로 변환하는 함수
  Line localLineToLine(const LocalLine& local_line, const std::list<Bin::MinZPoint>& line_points);

public:
  // Segment 클래스의 생성자
  Segment(const unsigned int& n_bins,  // 사용할 Bin 개수
          const double& max_slope,  // 최대 기울기 값
          const double& max_error,  // 최대 허용 오차 값
          const double& long_threshold,  // 긴 세그먼트 판단 임계값
          const double& max_long_height,  // 긴 세그먼트의 최대 높이
          const double& max_start_height,  // 시작 높이의 최대 값
          const double& sensor_height);  // 센서 높이 값

  // 특정 거리(d)에서 선까지의 수직 거리(z)를 계산하는 함수
  double verticalDistanceToLine(const double& d, const double &z);
  
  // 세그먼트 내에서 선분을 피팅하는 함수
  void fitSegmentLines();

  // 인덱스를 사용하여 Bin 객체에 접근하는 연산자 오버로딩 함수
  inline Bin& operator[](const size_t& index) {
    return bins_[index];  // 해당 인덱스의 Bin 객체 반환
  }

  // bins_ 벡터의 시작 반복자를 반환하는 함수
  inline std::vector<Bin>::iterator begin() {
    return bins_.begin();
  }

  // bins_ 벡터의 끝 반복자를 반환하는 함수
  inline std::vector<Bin>::iterator end() {
    return bins_.end();
  }

  // lines_ 리스트에 저장된 선분들을 외부로 반환하는 함수
  bool getLines(std::list<Line>* lines);
};

#endif //_LIDAR100_LINEFIT_BIN_H_SEGMENT_H_
