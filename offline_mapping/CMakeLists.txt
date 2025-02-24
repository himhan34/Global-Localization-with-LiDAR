add_subdirectory(linefit)  // 'linefit' 디렉토리를 서브디렉토리로 추가합니다.

add_executable(mapping mapping.cpp)  // 'mapping.cpp'를 기반으로 'mapping' 실행 파일을 생성합니다.
target_link_libraries(mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${LIDAR100_LINEFIT_LIB})  // 'mapping' 실행 파일을 'catkin', 'PCL', 'LIDAR100_LINEFIT_LIB' 라이브러리와 링크합니다.

add_executable(collect_database collect_database.cpp)  // 'collect_database.cpp'를 기반으로 'collect_database' 실행 파일을 생성합니다.
target_link_libraries(collect_database ${catkin_LIBRARIES} ${PCL_LIBRARIES})  // 'collect_database' 실행 파일을 'catkin' 및 'PCL' 라이브러리와 링크합니다.
