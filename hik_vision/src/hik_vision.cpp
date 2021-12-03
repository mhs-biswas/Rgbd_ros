// #include "hik_vision/camera_interface.hpp"
#include "hik_vision/hik_vision.hpp"

#include </opt/MVS/include/MvCameraControl.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>

namespace camera {
  

  hik_vision::hik_vision() : handle_(nullptr), initialized_(false) {
      
  }

  hik_vision::~hik_vision() {
      shutdown();
  }

  void hik_vision::set_dev_num(int num)
  {
    device_num=num;
  }

  bool hik_vision::initialize() {
    if(!initialized_.load()) {
      int rv = MV_OK;
      MV_CC_DEVICE_INFO_LIST device_list;
      memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
      memset(&param_, 0, sizeof(MVCC_INTVALUE));
      std::cout<<"HELLOW 1\n";
      // enumerate devices
      if ((rv = MV_CC_EnumDevices( MV_USB_DEVICE, &device_list)) == MV_OK) {

        if (device_list.nDeviceNum > 0) {

          // print devices
          for (int i = 0; i < device_list.nDeviceNum; i++) {
              std::cout << "[device " << i << "]: " << std::endl;
              MV_CC_DEVICE_INFO* device_info = device_list.pDeviceInfo[i];
              if (device_info != nullptr) {
                  std::cout << "model name: " << device_info->SpecialInfo.stUsb3VInfo.chModelName << std::endl;
              } 
          }  

          // create handle
          if((rv = MV_CC_CreateHandleWithoutLog(&handle_, device_list.pDeviceInfo[device_num]) == MV_OK)) {
            std::cout<<"HELLOW I AM A HERE 1\n";
            // open device
            if((rv = MV_CC_OpenDevice(handle_)) == MV_OK) {
              // set optimal network package size (if GigE)
              std::cout<<"HELLOW I AM A HERE 2\n";
              if (device_list.pDeviceInfo[device_num]->nTLayerType == MV_GIGE_DEVICE) {
                int packet_size = MV_CC_GetOptimalPacketSize(handle_);
                std::cout<<"HELLOW I AM A HERE 3\n";
                if (packet_size > 0) {
                  if((rv = MV_CC_SetIntValue(handle_,"GevSCPSPacketSize",packet_size)) != MV_OK) {
                    std::cerr << "warning... could not set optimal packet size for GigE: " << rv << std::endl;
                  }
                }
                else {
                  std::cerr << "warning... could not retrieve optimal packet size for GigE: " << packet_size << std::endl;
                }
              }

              // set to software trigger mode
              std::cout<<"HELLOW I AM A HERE" << std::endl;
              if((rv = MV_CC_SetEnumValue(handle_, "TriggerMode", 0)) == MV_OK) {
                std::cout<<"HELLOW I AM A HERE A" << std::endl;;
                // retrieve payload scn
                if((rv = MV_CC_GetIntValue(handle_, "PayloadSize", &param_)) == MV_OK) {
                  std::cout<<"HELLOW I AM A HERE B" << std::endl;;
                  // enable "image grabbing"
                  if((rv = MV_CC_StartGrabbing(handle_)) == MV_OK) {
                    std::cout<<"HELLOW I AM A HERE C" << std::endl;;
                    initialized_.store(true);
                    return EXIT_SUCCESS;
                  } else { std::cerr << "could not enable image grabbing: " << rv << std::endl; }
                } else { std::cerr << "could not retrieve payload size: " << rv << std::endl; }
              } else { std::cerr << "could not set camera to software trigger mode: " << rv << std::endl; }
            } else { std::cerr << "could not open device: " << rv << std::endl; }
          } else { std::cerr << "could not create handle: " << rv << std::endl; }
        } else { std::cerr << "could not find any camera devices... " << std::endl; }
      } else { std::cerr << "failed to enumerate camera devices: " << rv << std::endl; }
    }
    return EXIT_FAILURE;
  }

  cv::Mat hik_vision::capture() {
    std::lock_guard<std::mutex> lock(io_sync_);
    using binary_buffer = std::vector<unsigned char>;
    int rv = MV_OK;

    MV_FRAME_OUT_INFO_EX image_info = {0};
    memset(&image_info, 0, sizeof(MV_FRAME_OUT_INFO_EX));
   
    binary_buffer frame(param_.nCurValue);
    std::fill(frame.begin(), frame.end(), 0);

    // capture frame
    if((rv = MV_CC_GetOneFrameTimeout(handle_, frame.data(), frame.size(), &image_info, 1000)) == MV_OK) {
      std::cout << "got frame with: " 
        << "width = " << image_info.nWidth << "; " 
        << "height = " << image_info.nHeight << "; "
        << "frame_number = " << image_info.nFrameNum << std::endl;

      // prepare image buffer and adjust "saving" parameter
      binary_buffer image(image_info.nWidth * image_info.nHeight * 4 + 2048);
      std::fill(image.begin(), image.end(), 0);

      MV_SAVE_IMAGE_PARAM_EX save_param;
      memset(&save_param, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
      
      save_param.enImageType = MV_Image_Jpeg; 
      save_param.enPixelType = image_info.enPixelType; 
      save_param.nBufferSize = image_info.nWidth * image_info.nHeight * 4 + 2048;
      save_param.nWidth      = image_info.nWidth; 
      save_param.nHeight     = image_info.nHeight; 
      save_param.pData       = frame.data();
      save_param.nDataLen    = image_info.nFrameLen;
      save_param.pImageBuffer = image.data();
      save_param.nJpgQuality = 80;
      std::cout<<"HELLOW I AM A HERE -- Capture \n";
      // pull image from frame
      if((rv = MV_CC_SaveImageEx2(handle_, &save_param)) == MV_OK) {        
        auto cv_image = cv::imdecode(image, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
        // cv::cvtColor(cv_image, cv_image, cv::COLOR_GRAY2BGR);
        
        // free(frame.data());
        return cv_image;
      } else { std::cerr << "could not convert frame to bmp: " << rv << std::endl; }

      
    } else { std::cerr << "could not capture frame: " << rv << std::endl; }
    return {};
  }



  // cv::Mat hik_vision::capture1()
  // { 

    

  //   int nRet = MV_OK;
  //   cv::Mat dumy_img;
  //   // ch:获取数据包大小 | en:Get payload size
  //   MVCC_INTVALUE stParam;
  //   memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  //   nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
  //   if (MV_OK != nRet)
  //   {
  //       printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
  //       return dumy_img;
  //   }

  //   MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  //   memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  //   unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  //   if (NULL == pData)
  //   {
  //       return dumy_img;
  //   }
  //   unsigned int nDataSize = stParam.nCurValue;

  //   // while(1)
  //   // {
	// 	// if(g_bExit)
	// 	// {
	// 	// 	break;
	// 	// }

  //     nRet = MV_CC_GetOneFrameTimeout(handle_, pData, nDataSize, &stImageInfo, 1000);
  //     if (nRet == MV_OK)
  //     {
  //         printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", 
  //             stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
  //     }
  //     else{
  //         printf("No data[%x]\n", nRet);
  //     }
  //   // }

  //   free(pData);
  //   return dumy_img;

  // }

  bool hik_vision::shutdown() {
    std::cout<<"Shutting down: \n";
    int rv = MV_OK;
    if(initialized_.load()) {
      if((rv = MV_CC_StopGrabbing(handle_)) == MV_OK) {
        if((rv = MV_CC_DestroyHandle(handle_)) == MV_OK) {
          initialized_.store(false);
          return EXIT_SUCCESS;
        } else { std::cerr << "could not shutdown... could not destroy handle" << std::endl; }  
      } else { std::cerr << "could not shutdown... could not stop \"image grabbing\"" << std::endl; }
    } else { std::cerr << "could not shutdown... camera was not initialized" << std::endl; }
    return EXIT_FAILURE;
  }
}