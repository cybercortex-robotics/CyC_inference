find_package(OpenSSL REQUIRED)

add_executable(App_SignalingServer
	main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/../../cyc_core/communication/CSignalingServer.h
	${CMAKE_CURRENT_SOURCE_DIR}/../../cyc_core/communication/CSignalingServer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/../../cyc_core/communication/CCommUtils.h
	${CMAKE_CURRENT_SOURCE_DIR}/../../cyc_core/communication/CCommUtils.cpp)

target_link_libraries(App_SignalingServer
	PRIVATE
		datachannel
		msgpackc
		#spdlog
		${OPENSSL_CRYPTO_LIBRARIES}
		${OPENSSL_SSL_LIBRARIES})

target_include_directories(App_SignalingServer
	PRIVATE
		${OPENSSL_INCLUDE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/../../CyC_core/communication/)
