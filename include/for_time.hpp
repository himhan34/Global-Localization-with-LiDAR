#ifndef _LIDAR_100_UTILS_TIME_H_
#define _LIDAR_100_UTILS_TIME_H_

#include <chrono>  // 시간 측정을 위한 헤더 파일

// 실행 시간 측정을 위한 클래스
class TicToc {
public:
    TicToc() {
        tic();  // 객체 생성 시 자동으로 시작 시간 기록
    }

    // 시작 시간 기록
    void tic() {
        start = std::chrono::system_clock::now();
    }

    // 경과 시간(ms) 반환
    double toc() {
        end = std::chrono::system_clock::now();  // 현재 시간 기록
        std::chrono::duration<double> elapsed_seconds = end - start;  // 경과 시간 계산
        return elapsed_seconds.count() * 1000;  // 밀리초 단위 변환 후 반환
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;  // 시작 및 종료 시간 저장 변수
};

#endif  // _LIDAR_100_UTILS_TIME_H_
