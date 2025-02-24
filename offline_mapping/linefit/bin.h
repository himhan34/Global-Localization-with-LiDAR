#ifndef _LIDAR100_LINEFIT_BIN_H_
#define _LIDAR100_LINEFIT_BIN_H_

#include <atomic>  // std::atomic을 사용하기 위해 포함합니다.
#include <pcl/point_cloud.h>  // PCL의 포인트 클라우드 관련 기능을 사용하기 위해 포함합니다.
#include <pcl/point_types.h>  // PCL의 기본 포인트 타입을 사용하기 위해 포함합니다.

class Bin {
public:
  struct MinZPoint {  
    // 최소 z 값을 가진 점을 나타내는 구조체입니다.
    MinZPoint() : z(0), d(0) {}  
    // 기본 생성자로 z와 d를 0으로 초기화합니다.

    MinZPoint(const double& d, const double& z) : z(z), d(d) {}  
    // 거리(d)와 높이(z)를 입력받아 초기화하는 생성자입니다.

    bool operator==(const MinZPoint& comp) {return z == comp.z && d == comp.d;}  
    // 두 개의 MinZPoint가 동일한지 비교하는 연산자 오버로딩 함수입니다.

    double z;  // 점의 높이(z 좌표)
    double d;  // 점의 거리(반경)
  };

private:

  std::atomic<bool> has_point_;  // 현재 Bin에 점이 추가되었는지를 나타내는 플래그입니다.
  std::atomic<double> min_z;  // 현재 Bin에서 최소 z 값을 저장하는 변수입니다.
  std::atomic<double> min_z_range;  // 최소 z 값을 가진 점의 거리 값을 저장하는 변수입니다.

public:

  Bin();  
  // 기본 생성자로, min_z를 최대값으로 설정하고 has_point_를 false로 초기화합니다.

  /// \brief 벡터<vector<Bin>> 초기화를 허용하기 위한 복사 생성자입니다.
  Bin(const Bin& segment);  
  // 복사 생성자. 내부 상태를 복사하지 않고 새로운 객체를 초기화하는 역할을 합니다.

  void addPoint(const pcl::PointXYZ& point);  
  // PCL의 3D 점을 받아 Bin에 추가하는 함수입니다.

  void addPoint(const double& d, const double& z);  
  // 거리(d)와 높이(z)를 받아 Bin에 추가하는 함수입니다.

  MinZPoint getMinZPoint();  
  // 현재 Bin에서 최소 z 값을 가진 점을 반환하는 함수입니다.

  inline bool hasPoint() {return has_point_;}  
  // Bin이 적어도 하나의 점을 포함하고 있는지 여부를 반환하는 함수입니다.
};

#endif //_LIDAR100_LINEFIT_BIN_H_
