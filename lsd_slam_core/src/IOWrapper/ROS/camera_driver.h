#pragma once

#include <libuvc/libuvc.h>

//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <image_transport/camera_publisher.h>
//#include <dynamic_reconfigure/server.h>
//#include <camera_info_manager/camera_info_manager.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>

//#include <libuvc_camera/UVCCameraConfig.h>

#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"
#include "IOWrapper/InputImageStream.h"

#include "util/Undistorter.h"

namespace lsd_slam
{


class CameraDriver : public InputImageStream
{
public:
  CameraDriver(/*ros::NodeHandle nh, ros::NodeHandle priv_nh*/);
  virtual ~CameraDriver();

  bool Start();
  void Stop();  

  void OpenCamera();
//  void CloseCamera();

  // For LSD_SLAM
  /**
   * Starts the thread.
   */
  void run();

  void setCalibration(std::string file);

  /**
   * Thread main function.
   */
  void operator()();

protected:
    // New frame virtual routine
    virtual void newFrameReceived( u_char* data );

private:
  // show frame
//  void showFrame();

private:
  enum State
  {
    kInitial = 0,
    kStopped = 1,
    kRunning = 2,
  };

  // Flags controlling whether the sensor needs to be stopped (or reopened) when changing settings
  static const int kReconfigureClose    = 3;    // Need to close and reopen sensor to change this setting
  static const int kReconfigureStop     = 1;    // Need to stop the stream before changing this setting
  static const int kReconfigureRunning  = 0;    // We can change this setting without stopping the stream

  // Accept changes in values of automatically updated controls
  void AutoControlsCallback(enum uvc_status_class status_class,
                            int event,
                            int selector,
                            enum uvc_status_attribute status_attribute,
                            void *data, size_t data_len);

  static void AutoControlsCallbackAdapter(enum uvc_status_class status_class,
                                          int event,
                                          int selector,
                                          enum uvc_status_attribute status_attribute,
                                          void *data, size_t data_len,
                                          void *ptr);

    // Accept a new image frame from the camera
    void          ImageCallback(uvc_frame_t *frame);
    static void   ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);

    State state_;
    boost::recursive_mutex mutex_;

    uvc_context_t         *ctx_;

    uvc_device_t          *dev_;
    uvc_device_handle_t   *devh_;

    uvc_frame_t           *rgb_frame_;

    uvc_stream_handle_t   	*strmh_;
    uvc_stream_ctrl_t       ctrl_;

//  uvc_stream_ctrl_t ctrl;
//  uvc_error_t res;

protected:
    int                 m_nDevForOpen;
    uvc_frame_format    m_frameFormat;

//    int                 m_width;
//    int                 m_height;

    int                 m_frameRate;

    // For LSD_SLAM
    bool            haveCalib;
    Undistorter*    undistorter;

    int             lastSEQ;

//  image_transport::ImageTransport it_;
//  image_transport::CameraPublisher cam_pub_;

//  dynamic_reconfigure::Server<UVCCameraConfig> config_server_;
//  UVCCameraConfig config_;
//  bool config_changed_;

//  camera_info_manager::CameraInfoManager cinfo_manager_;
};

}

