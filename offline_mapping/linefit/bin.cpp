#include "offline_mapping/linefit/bin.h"  // 'bin.h' 헤더 파일을 포함하여 Bin 클래스 선언을 가져옵니다.
#include <limits>  // std::numeric_limits를 사용하기 위해 포함합니다.

Bin::Bin() : min_z(std::numeric_limits<double>::max()), has_point_(false) {}  
// Bin 기본 생성자: 
//  - min_z를 double 타입의 최댓값으로 초기화하여 최소값 비교 시 올바르게 동작하도록 설정합니다.
//  - has_point_를 false로 설정하여 아직 점이 추가되지 않았음을 나타냅니다.

Bin::Bin(const Bin& bin) : min_z(std::numeric_limits<double>::max()),
                                           has_point_(false) {}  
// Bin 복사 생성자:  
//  - min_z를 기본 생성자와 동일하게 초기화합니다.
//  - has_point_를 false로 설정하여 기존 Bin의 값을 복사하지 않습니다.

void Bin::addPoint(const pcl::PointXYZ& point) {  
  // pcl::PointXYZ 타입의 점을 받아 추가하는 함수입니다.
  const double d = sqrt(point.x * point.x + point.y * point.y);  
  // 점의 x, y 좌표를 이용해 원점으로부터의 거리(d)를 계산합니다.
  addPoint(d, point.z);  
  // 거리와 z 좌표를 addPoint 함수에 전달합니다.
}

void Bin::addPoint(const double& d, const double& z) {  
  // 거리(d)와 높이(z)를 받아 점을 추가하는 함수입니다.
  has_point_ = true;  // 점이 추가되었음을 나타내는 플래그를 true로 설정합니다.
  if (z < min_z) {  
    // 새로운 점의 z 값이 기존의 min_z보다 작다면 업데이트합니다.
    min_z = z;  
    min_z_range = d;  // 최소 z 값을 가진 점의 거리 값을 저장합니다.
  }
}

Bin::MinZPoint Bin::getMinZPoint() {  
  // 현재 Bin에서 가장 작은 z 값을 가지는 점을 반환하는 함수입니다.
  MinZPoint point;  // MinZPoint 구조체 인스턴스를 생성합니다.

  if (has_point_) {  
    // 최소 z 값을 가지는 점이 존재할 경우 해당 값을 설정합니다.
    point.z = min_z;  
    point.d = min_z_range;
  }

  return point;  // 최종적으로 MinZPoint 구조체를 반환합니다.
}
