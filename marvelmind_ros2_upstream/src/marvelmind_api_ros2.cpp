/*
Copyright (c) 2022, Marvelmind Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUfTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

/* Credits
 * 
 * Special thanks to Carson Loyal (mailto: ctl0021@auburn.edu),
 * an original author of this ROS2 package implementation. 
 * Original source code of the package is available by the link: 
 * https://drive.google.com/file/d/1R5RfeAG1CbsC296TrZgwc1sQUx_sHGKC/view?usp=sharing
 */ 


#include <marvelmind_ros2/marvelmind_api_ros2.hpp>
#include <string.h>
#ifdef WIN32
#include <direct.h>
#endif

void marvelmind_api_ros2::mmAPIrequestProcess(marvelmind_ros2_msgs::srv::MarvelmindAPI::Request::SharedPtr req,
	marvelmind_ros2_msgs::srv::MarvelmindAPI::Response::SharedPtr res) {

	std::vector<uint8_t>* pdv = &req->request;
	uint8_t* rq_data = pdv->data();
	uint32_t rq_size = (uint32_t) pdv->size();
	int64_t rq_command = req->command_id;

	uint8_t response_buf[1024];
	uint32_t response_size = 1024;
	int32_t error_code = -1;

    bool result = marvelmindAPICall(rq_command, rq_data, rq_size, &response_buf[0], &response_size, &error_code);
	res->success = result;
	res->error_code = error_code;
	std::vector<uint8_t>* resv = &res->response;
	if (res->success) {
		resv->resize(response_size);
		uint8_t* resp_data = resv->data();
		for (uint32_t i = 0; i < response_size; i++) {
			resp_data[i] = response_buf[i];
		}
	}
	else {
		resv->resize(0);
	}
	
	switch(rq_command) {
		case MM_API_ID_OPEN_PORT:
		case MM_API_ID_OPEN_PORT_BY_NAME:
		case MM_API_ID_OPEN_PORT_UDP:
		case MM_API_ID_CLOSE_PORT: {
			if (openPortTryNeeded) {
				RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Port access function call: automatic port open cancelled");
				openPortTryNeeded= false;
			}
		}
	}


	//RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "MM API request %d",(int) rq_command);
	//for (int i = 0; i < rq_size; i++)
	//	RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "  %d", (int)rq_data[i]);
}

void marvelmind_api_ros2::timer1Callback()
{
	if (openPortTryNeeded) {
		tryOpenPort();
	}
	
	if (openPortTryNeeded)
	  return;
	
	MarvelmindDevicesList md;
	mmGetDevicesList(&md);
}

void marvelmind_api_ros2::tryOpenPort(void) {
	char portName[256];
	strcpy(portName, this->tty_filename.c_str());
	if (mmOpenPortByName(portName)) {
		RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Port opened: %s", portName);
		openPortTryNeeded= false;
	}
	else {
		RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Failed to open port: %s. Retrying...", portName);
	}
}

marvelmind_api_ros2::marvelmind_api_ros2() : rclcpp::Node("marvelmind_api_ros2") {
	static const rclcpp::Logger mm_api_logger = rclcpp::get_logger(MM_LOGGER_NAME);
	RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Constructor: marvelmind_api_ros2 pre-init");

	// Declare params found in config
	// Topics
	this->declare_parameter("hedgehog_pos_topic", "hedgehog_pos");

	// Other Params
	this->declare_parameter("data_input_semaphore_name", "/data_input_semaphore");
	this->declare_parameter("marvelmind_publish_rate_in_hz", 1);
	this->declare_parameter("marvelmind_tty_baudrate", 9600);
#ifdef WIN32
	this->declare_parameter("marvelmind_tty_filename", "//./com1");
#else
	this->declare_parameter("marvelmind_tty_filename", "/dev/ttyACM0");
#endif

	//Startup

	this->publish_rate_in_hz = this->get_parameter("marvelmind_publish_rate_in_hz").as_int();
	this->tty_filename = this->get_parameter("marvelmind_tty_filename").as_string();

	this->publish_rate_ms = std::chrono::milliseconds((int)round(1000.0 / this->publish_rate_in_hz));
	this->marvelmind_ros2_pub_timer = this->create_wall_timer(this->publish_rate_ms, std::bind(&marvelmind_api_ros2::publishTimerCallback, this));

	RCLCPP_DEBUG(rclcpp::get_logger(MM_LOGGER_NAME), "Pub rate ms: %li", this->publish_rate_ms.count());

	// Create Marvelmind API service

	server_= this->create_service<marvelmind_ros2_msgs::srv::MarvelmindAPI>(
		"marvelmind_api",
		std::bind(&marvelmind_api_ros2::mmAPIrequestProcess, this, std::placeholders::_1, std::placeholders::_2));

	rclcpp::Service<marvelmind_ros2_msgs::srv::MarvelmindAPI>::SharedPtr server_;

	//

#ifdef WIN32
	char buff[FILENAME_MAX];
	if (_getcwd(buff, FILENAME_MAX) == NULL) {
		RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Open Marvelmind API by default path");
		marvelmindAPILoad(NULL);
	}
	else {
		char sbuf[1024];
		sprintf(sbuf, "%s\\dashapi.dll", buff);
 
		RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Open Marvelmind API by path: %s", sbuf);
		marvelmindAPILoad(sbuf);
    }
  #else 
	RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Open Marvelmind API by default path");
    marvelmindAPILoad(NULL);// Load Marvelmind API library
  #endif

	mm_api_version = 0;
	openPortTryNeeded= true;
	if (mmAPIVersion(&mm_api_version)) {
		RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Marvelmind API version: %d", (int)mm_api_version);

        tryOpenPort();

		//for(int i=0;i<argc;i++)
		//	RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "ARG %d: %s", i, argv[i]);
	}
	else {
		RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Failed to get Marvelmind API version");
	}

	m_timer1 = this->create_wall_timer(
	    std::chrono::milliseconds(1000),
		std::bind(&marvelmind_api_ros2::timer1Callback, this));
}

marvelmind_api_ros2::~marvelmind_api_ros2() {
  mmClosePort();// Close port (if was opened)

  marvelmindAPIFree();// Free Marvelmind API library memory

  RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Destructor starting marvelmind_api_ros2...");

  RCLCPP_INFO(rclcpp::get_logger(MM_LOGGER_NAME), "Destructor: marvelmind_api_ros2");
}



void marvelmind_api_ros2::publishTimerCallback() {

  
} // end timer callback

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // create and spin marvelmind ros2 node
  auto marvelmind_api_ros2_node = std::make_shared<marvelmind_api_ros2>();
  rclcpp::spin(marvelmind_api_ros2_node);
  rclcpp::shutdown();
  return 0;
}
