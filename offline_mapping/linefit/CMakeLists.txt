SET(LIB_NAME lidar100_linefit)  // 라이브러리 이름을 'lidar100_linefit'으로 설정합니다.
SET(LIB_NAME_UPPER LIDAR100_LINEFIT)  // 라이브러리 이름의 대문자 버전을 'LIDAR100_LINEFIT'으로 설정합니다.

include_directories(${PCL_INCLUDE_DIRS})  // PCL(Poin Cloud Library) 헤더 파일을 포함할 디렉토리를 설정합니다.
aux_source_directory(./ ${LIB_NAME_UPPER}_SRC)  // 현재 디렉토리의 모든 소스 파일을 검색하여 'LIDAR100_LINEFIT_SRC' 변수에 저장합니다.

SET(${LIB_NAME_UPPER}_LIB ${LIB_NAME} CACHE INTERNAL "lib")  // 내부 캐시 변수로 라이브러리 이름을 설정합니다.

ADD_LIBRARY(${${LIB_NAME_UPPER}_LIB} SHARED 
            ${${LIB_NAME_UPPER}_SRC})  // 'LIDAR100_LINEFIT_LIB'를 공유 라이브러리(SHARED)로 생성하고, 'LIDAR100_LINEFIT_SRC'의 소스 파일을 추가합니다.

TARGET_LINK_LIBRARIES(${${LIB_NAME_UPPER}_LIB} 
                       ${PCL_LIBRARIES}  // PCL 라이브러리와 링크합니다.
                       ${Boost_LIBRARIES})  // Boost 라이브러리와 링크합니다.
