/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "camera_driver.h"
#include <libuvc/libuvc.h>

namespace lsd_slam
{


using namespace cv;

CameraDriver::CameraDriver():
    state_(kInitial),
    ctx_(NULL), dev_(NULL), devh_(NULL), rgb_frame_(NULL)
{
    m_nDevForOpen   = 0;
    m_frameFormat   = UVC_FRAME_FORMAT_ANY;

//    m_width     = 640;
//    m_height    = 480;

    width_     = 0;
    height_    = 0;
    m_frameRate = 30;

    // imagebuffer
    imageBuffer = new NotifyBuffer<TimestampedMat>(8);
    undistorter = 0;
    lastSEQ     = 0;

    haveCalib = false;
}

CameraDriver::~CameraDriver()
{
//    CloseCamera();
    Stop();

    if (rgb_frame_)
        uvc_free_frame(rgb_frame_);

//    if (ctx_)
//        // Destroys dev_, devh_, etc.
//        uvc_exit(ctx_);

    delete imageBuffer;

    puts("Camera closed...");
}

bool CameraDriver::Start()
{
    assert(state_ == kInitial);

    uvc_error_t err;

    /* Initialize a UVC service context. Libuvc will set up its own libusb
     * context. Replace NULL with a libusb_context pointer to run libuvc
     * from an existing libusb context. */
    err = uvc_init(&ctx_, NULL);

    if (err != UVC_SUCCESS)
    {
        uvc_perror(err, "ERROR: uvc_init");
        return false;
    }

    puts("UVC initialized");

    state_ = kStopped;

    OpenCamera();

    return state_ == kRunning;
}

void CameraDriver::Stop()
{
  boost::recursive_mutex::scoped_lock(mutex_);

  assert(state_ != kInitial);

  if (state_ == kRunning)
  {
//    CloseCamera();
      /* Release our handle on the device */

      /* End the stream. Blocks until last callback is serviced */
  //    uvc_stop_streaming(devh_);
      uvc_stream_stop(strmh_);

  //    uvc_error_t stream_err = uvc_stream_stop(strmh_);

  //    if (stream_err != UVC_SUCCESS)
  //    {
  //        /* unable to start stream */
  //        uvc_perror(stream_err, "uvc_start_streaming");

  //        uvc_close(devh_);
  //        uvc_unref_device(dev_);
  //        return;
  //    }
      puts("Done streaming.");

      uvc_close(devh_);
      devh_ = NULL;

      uvc_unref_device(dev_);
      dev_ = NULL;

      // change state
      state_ = kStopped;
      puts("Device closed");
  }

  assert(state_ == kStopped);

  uvc_exit(ctx_);
  ctx_ = NULL;

  state_ = kInitial;
}

//void CameraDriver::showFrame()
//{
//    cvImg = cvCreateImageHeader( cvSize(bgr->width, bgr->height), IPL_DEPTH_8U, 3);
//    cvSetData(cvImg, bgr->data, bgr->width * 3);
//    cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
//    cvShowImage("Test", cvImg);
//    cvWaitKey(10);
//    cvReleaseImageHeader(&cvImg);
//}

void CameraDriver::ImageCallback(uvc_frame_t *frame)
{
/////  ros::Time timestamp = ros::Time(frame->capture_time.tv_sec, frame->capture_time.tv_usec);

//    uvc_frame_t *rgb;
    uvc_error_t ret;

    boost::recursive_mutex::scoped_lock(mutex_);

    if(state_ != kRunning)
        return;

//    assert(state_ == kRunning);
    assert(rgb_frame_);

    /* Do the BGR conversion */
    ret = uvc_any2rgb(frame, rgb_frame_);

    if (ret)
    {
        uvc_perror(ret, "uvc_any2bgr");
        uvc_free_frame(rgb_frame_);
        return;
    }

//    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);

    // frame callback
    newFrameReceived( (u_char*)rgb_frame_->data );

/*
//  sensor_msgs::Image::Ptr image(new sensor_msgs::Image());

//  image->width = config_.width;
//  image->height = config_.height;
//  image->step = image->width * 3;
//  image->data.resize(image->step * image->height);

//  if (frame->frame_format == UVC_FRAME_FORMAT_BGR)
//  {
//    image->encoding = "bgr8";
//    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
//  }
//  else if (frame->frame_format == UVC_FRAME_FORMAT_RGB)
//  {
//    image->encoding = "rgb8";
//    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
//  }
//  else if (frame->frame_format == UVC_FRAME_FORMAT_UYVY)
//  {
//    image->encoding = "yuv422";
//    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
//  }
//  else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV)
//  {
//    // FIXME: uvc_any2bgr does not work on "yuyv" format, so use uvc_yuyv2bgr directly.
//    uvc_error_t conv_ret = uvc_yuyv2bgr(frame, rgb_frame_);
//    if (conv_ret != UVC_SUCCESS)
//    {
//      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
//      return;
//    }
//    image->encoding = "bgr8";
//    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
//  }
//  else if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG)
//  {
//    // FIXME: uvc_any2bgr does not work on "mjpeg" format, so use uvc_mjpeg2rgb directly.
//    uvc_error_t conv_ret = uvc_mjpeg2rgb(frame, rgb_frame_);
//    if (conv_ret != UVC_SUCCESS)
//    {
//      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
//      return;
//    }
//    image->encoding = "rgb8";
//    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
//  }
//  else
//  {
//    uvc_error_t conv_ret = uvc_any2bgr(frame, rgb_frame_);
//    if (conv_ret != UVC_SUCCESS)
//    {
//      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
//      return;
//    }
//    image->encoding = "bgr8";
//    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
//  }

/////  sensor_msgs::CameraInfo::Ptr cinfo(
/////    new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

//  image->header.frame_id = config_.frame_id;
//  image->header.stamp = timestamp;

//  cinfo->header.frame_id = config_.frame_id;
//  cinfo->header.stamp = timestamp;

//  cam_pub_.publish(image, cinfo);

//  if (config_changed_)
//  {
//    config_server_.updateConfig(config_);
//    config_changed_ = false;
//  }
*/
}

/* static */ void CameraDriver::ImageCallbackAdapter(   uvc_frame_t *frame,
                                                        void        *ptr    )
{
    // cust to driver
    CameraDriver *driver = static_cast<CameraDriver*>(ptr);
    // call callback function
    driver->ImageCallback(frame);
}

void CameraDriver::AutoControlsCallback(    enum uvc_status_class       status_class,
                                            int                         event,
                                            int                         selector,
                                            enum uvc_status_attribute   status_attribute,
                                            void *data, size_t data_len)
{
//  boost::recursive_mutex::scoped_lock(mutex_);

  printf("Controls callback. class: %d, event: %d, selector: %d, attr: %d, data_len: %u\n",
         status_class, event, selector, status_attribute, data_len);

  if (status_attribute == UVC_STATUS_ATTRIBUTE_VALUE_CHANGE)
  {
    switch (status_class)
    {
    case UVC_STATUS_CLASS_CONTROL_CAMERA:
    {
      switch (selector)
      {
      case UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
        uint8_t *data_char      =   (uint8_t*) data;
        uint32_t exposure_int   =   ( (data_char[0])        | (data_char[1] << 8) |
                                    (  data_char[2] << 16)  | (data_char[3] << 24)  );

//        config_.exposure_absolute   = exposure_int * 0.0001;
//        config_changed_             = true;
        break;
      }
      break;
    }
    case UVC_STATUS_CLASS_CONTROL_PROCESSING:
    {
      switch (selector)
      {
      case UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
        uint8_t *data_char = (uint8_t*) data;

//        config_.white_balance_temperature =  data_char[0] | (data_char[1] << 8);
//        config_changed_ = true;

        break;
      }
      break;
    }
    }

    // config_server_.updateConfig(config_);
  }
}

/* static */ void CameraDriver::AutoControlsCallbackAdapter(  enum uvc_status_class status_class,
                                                                                int event,
                                                                                int selector,
                                                          enum uvc_status_attribute status_attribute,
                                                                               void *data,
                                                                             size_t data_len,
                                                                               void *ptr        )
{
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);

  driver->AutoControlsCallback(status_class, event, selector, status_attribute, data, data_len);
}

void CameraDriver::OpenCamera()
{
    assert(state_ == kStopped);

    uvc_device_t **devs;
    /* Locates the first attached UVC device, stores in dev */
    uvc_error_t find_err = uvc_find_devices( ctx_, &devs,
                                            /* filter devices: vendor_id, product_id, "serial_num" */
                                            0, 0, NULL );

    if (find_err != UVC_SUCCESS)
    {
        /* no devices found */
        uvc_perror(find_err, "uvc_find_device");
        return;
    }

    puts("Device found");

    // select device by index
    dev_        = NULL;
    int dev_idx = 0;
    // for all device in list
    while ( devs[dev_idx] != NULL )
    {
        if(dev_idx == m_nDevForOpen)
            dev_ = devs[dev_idx];
        else
            uvc_unref_device(devs[dev_idx]);

        dev_idx++;
    }

    if(dev_ == NULL)
    {
        printf("Unable to find device at index %d", m_nDevForOpen);
        return;
    }

    /* Try to open the device: requires exclusive access */
    uvc_error_t open_err = uvc_open(dev_, &devh_);

    if (open_err != UVC_SUCCESS)
    {
        switch (open_err)
        {
        case UVC_ERROR_ACCESS:
        #ifdef __linux__
            printf("Permission denied opening /dev/bus/usb/%03d/%03d \n",
                        uvc_get_bus_number(dev_), uvc_get_device_address(dev_));
        #else
          ROS_ERROR("Permission denied opening device %d on bus %d",
                    uvc_get_device_address(dev_), uvc_get_bus_number(dev_));
        #endif
          break;
        default:
        #ifdef __linux__
            printf("Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                            uvc_get_bus_number(dev_), uvc_get_device_address(dev_),
                            uvc_strerror(open_err), open_err);
        #else
          ROS_ERROR("Can't open device %d on bus %d: %s (%d)",
                    uvc_get_device_address(dev_), uvc_get_bus_number(dev_),
                    uvc_strerror(open_err), open_err);
        #endif
          break;
        }

        uvc_unref_device(dev_);
        return;
    }

    /* Print out a message containing all the information that libuvc
     * knows about the device */
    uvc_print_diag(devh_, stderr);

    //Set callback for status change
    uvc_set_status_callback(devh_, &CameraDriver::AutoControlsCallbackAdapter, this);

        /* Try to negotiate a 640x480 30 fps YUYV stream profile */
    uvc_error_t mode_err = uvc_get_stream_ctrl_format_size( devh_, &ctrl_,                   /* result stored in ctrl */
                                                            m_frameFormat,                  /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
                                                            width_, height_, m_frameRate);/* width, height, fps */
    /* Print out the result */
    uvc_print_stream_ctrl(&ctrl_, stderr);

    if (mode_err != UVC_SUCCESS)
    {
      /* device doesn't provide a matching stream */
        uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");

        uvc_close(devh_);
        uvc_unref_device(dev_);

        puts("check video_mode/width/height/frame_rate are available");
        uvc_print_diag(devh_, NULL);
        return;
    }

    // TODO: !!!!!!!
    uvc_error_t open_stream_err = uvc_stream_open_ctrl( devh_, &strmh_, &ctrl_ );
    if (open_stream_err != UVC_SUCCESS)
    {
        /* unable to start stream */
        uvc_perror(open_stream_err, "uvc_stream_open_ctrl");

        uvc_close(devh_);
        uvc_unref_device(dev_);
        return;
    }


    /* Start the video stream. The library will call user function cb:
     *   cb(frame, (void*) 12345)
     */
//    uvc_error_t stream_err = uvc_start_streaming(devh_, &ctrl_, &CameraDriver::ImageCallbackAdapter, this, 0);
     uvc_error_t stream_err = uvc_stream_start(  strmh_, &CameraDriver::ImageCallbackAdapter, this, 0);

    if (stream_err != UVC_SUCCESS)
    {
        /* unable to start stream */
        uvc_perror(stream_err, "uvc_start_streaming");

        uvc_close(devh_);
        uvc_unref_device(dev_);
        return;
    }

    puts("Streaming...");

//     uvc_set_ae_mode(devh_, 1); /* e.g., turn on auto exposure */

    // if already reserved
    if (rgb_frame_)
        // free buffer
        uvc_free_frame(rgb_frame_);

    // allocate buffer
    rgb_frame_ = uvc_allocate_frame( width_ * height_ * 3);

    assert(rgb_frame_);
    // change state
    state_ = kRunning;
}

void CameraDriver::run()
{
    // Start Camera
    this->Start();

    // Start SLAM thread
    boost::thread thread(boost::ref(*this));
}

void CameraDriver::setCalibration(std::string file)
{
//    if(file == "")
//    {
//        ros::Subscriber info_sub         = nh_.subscribe(nh_.resolveName("camera_info"),1, &ROSImageStreamThread::infoCb, this);

//        printf("WAITING for ROS camera calibration!\n");
//        while(width_ == 0)
//        {
//            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.03));
//        }
//        printf("RECEIVED ROS camera calibration!\n");

//        info_sub.shutdown();
//    }
//    else
//    {
    undistorter = Undistorter::getUndistorterForFile(file.c_str());

    if(undistorter==0)
    {
        printf("Failed to read camera calibration from file... wrong syntax?\n");
        exit(0);
    }

    fx_ = undistorter->getK().at<double>(0, 0);
    fy_ = undistorter->getK().at<double>(1, 1);
    cx_ = undistorter->getK().at<double>(2, 0);
    cy_ = undistorter->getK().at<double>(2, 1);

    width_  = undistorter->getOutputWidth();
    height_ = undistorter->getOutputHeight();
//    }

    haveCalib = true;
}

void CameraDriver::operator()()
{
    // TODO: Wait interrapt !
    while(1)
    {}
}

//void CameraDriver::CloseCamera()
//{
//    // if running
//    assert(state_ == kRunning);

//    /* Release our handle on the device */

//    /* End the stream. Blocks until last callback is serviced */
////    uvc_stop_streaming(devh_);
//    uvc_stream_stop(strmh_);
////    uvc_error_t stream_err = uvc_stream_stop(strmh_);

////    if (stream_err != UVC_SUCCESS)
////    {
////        /* unable to start stream */
////        uvc_perror(stream_err, "uvc_start_streaming");

////        uvc_close(devh_);
////        uvc_unref_device(dev_);
////        return;
////    }
//    puts("Done streaming.");

//    uvc_close(devh_);
//    devh_ = NULL;

//    uvc_unref_device(dev_);
//    dev_ = NULL;

//    // change state
//    state_ = kStopped;
//    puts("Device closed");
//}

void CameraDriver::newFrameReceived(u_char *data)
{

    if(!haveCalib)
        return;

//    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

//    if(img->header.seq < (unsigned int)lastSEQ)
//    {
//        printf("Backward-Jump in SEQ detected, but ignoring for now.\n");
//        lastSEQ = 0;
//        return;
//    }

//    lastSEQ = img->header.seq;

    lastSEQ++;

//    TimestampedMat bufferItem;
//     bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());

//    // !!!!!!!!!!!!
////    if(img->header.stamp.toSec() != 0)
////        bufferItem.timestamp =  Timestamp(img->header.stamp.toSec());
////    else
////        bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());

//    if(undistorter != 0)
//    {
//        assert(undistorter->isValid());
//        undistorter->undistort( cv_ptr->image, bufferItem.data );
//    }
//    else
//    {
//        bufferItem.data = cv_ptr->image;
//    }

//    imageBuffer->pushBack( bufferItem );
}

}

