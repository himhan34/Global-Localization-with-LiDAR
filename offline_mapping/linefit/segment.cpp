#include "offline_mapping/linefit/segment.h"  // Segment 클래스의 헤더 파일 포함

// Segment 클래스의 생성자 정의
Segment::Segment(const unsigned int& n_bins,  // Bin 개수
                 const double& max_slope,  // 최대 기울기 값
                 const double& max_error,  // 최대 허용 오차 값
                 const double& long_threshold,  // 긴 세그먼트 판단 임계값
                 const double& max_long_height,  // 긴 세그먼트의 최대 높이
                 const double& max_start_height,  // 시작 높이의 최대 값
                 const double& sensor_height) :  // 센서 높이 값
                 bins_(n_bins),  // n_bins 크기의 bins_ 벡터 초기화
                 max_slope_(max_slope),  // 최대 기울기 값 초기화
                 max_error_(max_error),  // 최대 허용 오차 값 초기화
                 long_threshold_(long_threshold),  // 긴 세그먼트 판단 임계값 초기화
                 max_long_height_(max_long_height),  // 긴 세그먼트의 최대 높이 초기화
                 max_start_height_(max_start_height),  // 시작 높이의 최대 값 초기화
                 sensor_height_(sensor_height) {}  // 센서 높이 값 초기화

void Segment::fitSegmentLines() {
  // 첫 번째 포인트를 찾음.
  auto line_start = bins_.begin();
  while (!line_start->hasPoint()) {  // 포인트가 있는 Bin을 찾을 때까지 반복
    ++line_start;
    // 마지막 Bin까지 도달하면 종료
    if (line_start == bins_.end()) return;
  }

  // 선(line)들을 저장하는 과정
  bool is_long_line = false;  // 긴 선 여부를 나타내는 변수
  double cur_ground_height = -sensor_height_;  // 현재 지면 높이 (센서 기준)
  std::list<Bin::MinZPoint> current_line_points(1, line_start->getMinZPoint());  // 현재 선을 구성하는 포인트 리스트
  LocalLine cur_line = std::make_pair(0,0);  // 현재 선의 기울기와 절편 초기화

  // 첫 번째 포인트 이후의 포인트들을 순차적으로 처리
  for (auto line_iter = line_start + 1; line_iter != bins_.end(); ++line_iter) {
    if (line_iter->hasPoint()) {  // 현재 Bin이 포인트를 포함하는 경우
      Bin::MinZPoint cur_point = line_iter->getMinZPoint();  // 현재 Bin의 최소 Z 포인트 가져오기

      // 현재 포인트가 이전 포인트와 일정 거리 이상 떨어져 있으면 긴 선으로 간주
      if (cur_point.d - current_line_points.back().d > long_threshold_) is_long_line = true;

      // 현재 라인에 2개 이상의 포인트가 있는 경우
      if (current_line_points.size() >= 2) {
        // 예상되는 Z 값 초기화 (초기값은 최대값)
        double expected_z = std::numeric_limits<double>::max();

        // 긴 선이면서 2개 이상의 포인트가 있는 경우 예측 Z 값 계산
        if (is_long_line && current_line_points.size() > 2) {
          expected_z = cur_line.first * cur_point.d + cur_line.second;  // y = mx + b 형태로 예측 값 계산
        }

        current_line_points.push_back(cur_point);  // 현재 포인트를 리스트에 추가
        cur_line = fitLocalLine(current_line_points);  // 현재 포인트 리스트를 바탕으로 선을 피팅
        const double error = getMaxError(current_line_points, cur_line);  // 피팅된 선과 데이터의 최대 오차 계산

        // 만약 선이 유효하지 않은 경우 (오차가 허용 범위를 벗어남)
        if (error > max_error_ ||
            std::fabs(cur_line.first) > max_slope_ ||  // 기울기 초과
            (is_long_line && std::fabs(expected_z - cur_point.z) > max_long_height_)) {  // 긴 선에서의 높이 초과
          // 마지막 포인트 이전까지의 선을 지면으로 저장
          current_line_points.pop_back();

          // 2개 이하의 포인트로 구성된 선은 허용하지 않음
          if (current_line_points.size() >= 3) {
            const LocalLine new_line = fitLocalLine(current_line_points);  // 새로운 선 피팅
            lines_.push_back(localLineToLine(new_line, current_line_points));  // 변환 후 리스트에 추가
            cur_ground_height = new_line.first * current_line_points.back().d + new_line.second;  // 새로운 지면 높이 갱신
          }

          // 새로운 선 시작
          is_long_line = false;
          current_line_points.erase(current_line_points.begin(), --current_line_points.end());  // 기존 선 초기화
          --line_iter;  // 현재 Bin 다시 검사
        }
        // 선이 유효한 경우 계속 진행
        else { }
      }
      else {  // 현재 선의 포인트가 부족한 경우
        if (cur_point.d - current_line_points.back().d < long_threshold_ &&
            std::fabs(current_line_points.back().z - cur_ground_height) < max_start_height_) {
          // 유효한 포인트이면 추가
          current_line_points.push_back(cur_point);
        }
        else {
          // 새로운 선을 시작
          current_line_points.clear();
          current_line_points.push_back(cur_point);
        }
      }
    }
  }

  // 마지막 선 추가
  if (current_line_points.size() > 2) {
    const LocalLine new_line = fitLocalLine(current_line_points);  // 새로운 선 피팅
    lines_.push_back(localLineToLine(new_line, current_line_points));  // 변환 후 리스트에 추가
  }
}

// 로컬 좌표계의 LocalLine을 전역 좌표계의 Line으로 변환하는 함수
Segment::Line Segment::localLineToLine(const LocalLine& local_line,
                                       const std::list<Bin::MinZPoint>& line_points) {
  Line line;  // 변환된 선(Line) 객체 생성
  const double first_d = line_points.front().d;  // 첫 번째 점의 거리 값(d)
  const double second_d = line_points.back().d;  // 마지막 점의 거리 값(d)

  // 직선 방정식을 이용해 첫 번째 점과 마지막 점의 z 값 계산
  const double first_z = local_line.first * first_d + local_line.second;
  const double second_z = local_line.first * second_d + local_line.second;

  // 계산된 z 값과 d 값을 line의 첫 번째 점에 저장
  line.first.z = first_z;
  line.first.d = first_d;

  // 계산된 z 값과 d 값을 line의 두 번째 점에 저장
  line.second.z = second_z;
  line.second.d = second_d;

  return line;  // 변환된 선(Line) 반환
}

// 주어진 점(d, z)이 기존 선과의 수직 거리를 계산하는 함수
double Segment::verticalDistanceToLine(const double &d, const double &z) {
  static const double kMargin = 0.1;  // 거리를 비교할 마진 값 설정
  double distance = -1;  // 초기 거리값을 -1로 설정 (유효한 선을 찾지 못한 경우)

  // 모든 선을 순회하면서 해당하는 선을 찾음
  for (auto it = lines_.begin(); it != lines_.end(); ++it) {
    // 현재 점(d)이 선분의 범위 내에 있는지 확인
    if (it->first.d - kMargin < d && it->second.d + kMargin > d) {
      const double delta_z = it->second.z - it->first.z;  // 선의 z 값 차이
      const double delta_d = it->second.d - it->first.d;  // 선의 d 값 차이

      // 주어진 d 값에서 예상되는 z 값 계산 (선형 보간)
      const double expected_z = (d - it->first.d) / delta_d * delta_z + it->first.z;

      // 실제 z 값과 예상 z 값의 차이를 거리로 설정
      distance = std::fabs(z - expected_z);
    }
  }
  return distance;  // 계산된 거리 반환
}

// 주어진 점 리스트(points)와 선(line) 사이의 평균 오차를 계산하는 함수
double Segment::getMeanError(const std::list<Bin::MinZPoint> &points, const LocalLine &line) {
  double error_sum = 0;  // 오차 합 초기화

  // 모든 포인트에 대해 오차를 계산하여 합산
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;  // 예상 z 값과 실제 z 값의 차이
    error_sum += residual * residual;  // 오차를 제곱하여 합산
  }

  return error_sum / points.size();  // 평균 제곱 오차 반환
}


// 주어진 점 리스트(points)와 선(line) 사이의 최대 오차를 계산하는 함수
double Segment::getMaxError(const std::list<Bin::MinZPoint> &points, const LocalLine &line) {
  double max_error = 0;  // 최대 오차 초기화

  // 모든 포인트에 대해 오차를 계산
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;  // 예상 z 값과 실제 z 값의 차이
    const double error = residual * residual;  // 오차를 제곱하여 계산

    // 현재까지의 최대 오차보다 크면 업데이트
    if (error > max_error) max_error = error;
  }
  
  return max_error;  // 최대 오차 반환
}

// 주어진 포인트 리스트를 이용하여 최적의 로컬 선을 피팅하는 함수
Segment::LocalLine Segment::fitLocalLine(const std::list<Bin::MinZPoint> &points) {
  const unsigned int n_points = points.size();  // 포인트 개수 저장

  // 포인트 개수만큼 행렬 X와 벡터 Y 초기화
  Eigen::MatrixXd X(n_points, 2);
  Eigen::VectorXd Y(n_points);
  
  unsigned int counter = 0;  // 인덱스 카운터 초기화

  // 행렬 X와 벡터 Y에 값 저장
  for (auto iter = points.begin(); iter != points.end(); ++iter) {
    X(counter, 0) = iter->d;  // 거리 값(d) 저장 (기울기 항)
    X(counter, 1) = 1;  // 상수 항 (절편을 위한 값)
    Y(counter) = iter->z;  // z 값 저장
    ++counter;
  }

  // QR 분해를 사용하여 선형 방정식 X * beta = Y를 푸는 과정
  Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);

  // 계산된 기울기와 절편을 저장
  LocalLine line_result;
  line_result.first = result(0);  // 기울기 저장
  line_result.second = result(1);  // 절편 저장

  return line_result;  // 피팅된 선 반환
}

// 현재 저장된 선(lines_)을 외부로 반환하는 함수
bool Segment::getLines(std::list<Line> *lines) {
  if (lines_.empty()) {  // 저장된 선이 없는 경우
    return false;  // false 반환
  }
  else {
    *lines = lines_;  // 리스트를 복사하여 반환
    return true;  // 성공적으로 반환했음을 알림
  }
}
