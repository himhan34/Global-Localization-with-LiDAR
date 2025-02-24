#ifndef _LIDAR_100_UTILS_IO_H_
#define _LIDAR_100_UTILS_IO_H_

#include <iomanip>  // setprecision 사용을 위한 헤더 파일
#include <cstdlib>  // EXIT_FAILURE 사용을 위한 헤더 파일
#include <fstream>  // 파일 입출력을 위한 헤더 파일
#include <iostream>  // 표준 입출력을 위한 헤더 파일
#include <string>  // 문자열 처리를 위한 헤더 파일
#include <vector>  // 벡터 컨테이너를 사용하기 위한 헤더 파일
#include <Eigen/Core>  // Eigen 기본 연산을 위한 헤더 파일
#include <Eigen/Dense>  // Eigen Dense 연산을 위한 헤더 파일

// KITTI 포인트 클라우드 데이터를 로드하는 함수
void loadKITTIBinToCloudXYZ(const std::string& bin_path, int index, PointCloudPtr& cloud) {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << index;  // 인덱스를 6자리로 맞추고 0으로 채움
    std::string filename = bin_path + ss.str() + ".bin";  // 파일 경로 생성

    std::fstream input(filename.c_str(), std::ios::in | std::ios::binary);  // 바이너리 파일 읽기 모드로 열기
    if (!input.good()) {  // 파일이 정상적으로 열리지 않으면 오류 메시지 출력 후 종료
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));
    }
    input.seekg(0, std::ios::beg);  // 파일의 처음으로 이동

    while (input.good() && !input.eof()) {  // 파일의 끝에 도달할 때까지 반복
        pcl::PointXYZ point;  // 포인트 객체 생성
        input.read((char*)&point.x, 3 * sizeof(float));  // x, y, z 좌표를 읽음
        float intensity;  // 강도(intensity) 값 저장 변수
        input.read((char*)&intensity, sizeof(float));  // 강도 값을 읽음
        cloud->push_back(point);  // 포인트 클라우드에 추가
    }
}

// KITTI 포즈 데이터를 로드하는 함수
void loadKITTIPoses(const std::string& filename, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& poses) {
    std::ifstream ifs(filename);  // 파일 열기
    if (!ifs.is_open()) {  // 파일이 정상적으로 열리지 않으면 오류 메시지 출력 후 종료
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));
    }

    std::string line;
    while (getline(ifs, line)) {  // 파일의 각 줄을 읽음
        std::vector<double> values(12);  // 12개의 변환 행렬 요소 저장 벡터
        std::stringstream ss(line);  // 문자열 스트림 생성
        for (int i = 0; i < 12; ++i) {
            ss >> values[i];  // 변환 행렬 요소를 읽음
        }
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();  // 4x4 단위 행렬 초기화
        pose(0, 0) = values[0]; pose(0, 1) = values[1]; pose(0, 2) = values[2]; pose(0, 3) = values[3];
        pose(1, 0) = values[4]; pose(1, 1) = values[5]; pose(1, 2) = values[6]; pose(1, 3) = values[7];
        pose(2, 0) = values[8]; pose(2, 1) = values[9]; pose(2, 2) = values[10]; pose(2, 3) = values[11];
        poses.push_back(pose);  // 포즈 벡터에 추가
    }
    ifs.close();  // 파일 닫기
}

// 정수를 바이너리 파일로 저장하는 함수
void saveIntegersAsBinary(const std::vector<size_t>& numbers, const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);  // 바이너리 파일 쓰기 모드로 열기
    if (!file) {  // 파일이 정상적으로 열리지 않으면 오류 메시지 출력 후 종료
        std::exit((std::cout << "Failed to save:" << filename << "\n", EXIT_FAILURE));
    }
    size_t size = numbers.size();  // 벡터 크기 저장
    file.write(reinterpret_cast<const char*>(&size), sizeof(size));  // 크기 정보 저장
    file.write(reinterpret_cast<const char*>(numbers.data()), numbers.size() * sizeof(size_t));  // 데이터 저장
    file.close();  // 파일 닫기
}

// 바이너리 파일에서 정수를 읽는 함수
void readIntegersFromBinary(const std::string& filename, std::vector<size_t>& numbers) {
    std::ifstream file(filename, std::ios::binary);  // 바이너리 파일 읽기 모드로 열기
    if (!file) {  // 파일이 정상적으로 열리지 않으면 오류 메시지 출력 후 종료
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));
    }
    size_t size;  // 크기 정보를 저장할 변수
    file.read(reinterpret_cast<char*>(&size), sizeof(size_t));  // 크기 정보 읽기
    numbers.resize(size);  // 벡터 크기 조정
    file.read(reinterpret_cast<char*>(numbers.data()), size * sizeof(size_t));  // 데이터 읽기
    file.close();  // 파일 닫기
}

#endif  // _LIDAR_100_UTILS_IO_H_
