#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include <ros/ros.h>
#include <ros/console.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <math.h>
#include <cmath>

#include <common.hpp>
#include <ocv_common.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking/tracker.hpp>

#include <inference_engine.hpp>

#include <object_detection/BoxMax.h>
#include <object_detection/Centroid.h>
#include <object_detection/Deviation.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <std_msgs/String.h>

using namespace InferenceEngine;
using namespace std;
using namespace cv;



class ObjectDetection{

public:

  ObjectDetection();
  ~ObjectDetection();

  // Start inference and detection
  void objectInference();
  // Init of model, weights
  void initNetworks();
  // Init camera
  void readInput();

  void frameToBlob(const cv::Mat& frame,
                   InferRequest::Ptr& inferRequest,
                   const std::string& inputName);
  cv::Point2f findCentroid(cv::Point2f left_point,cv::Point2f right_point);

  void commandCallback(const std_msgs::String::ConstPtr& msg);
  void Update();


  bool isStarted;

  void drawCentroid();


private:

  ros::NodeHandle nh;

  image_transport::Publisher image_raw_pub;
  image_transport::Publisher image_detected_pub;

  ros::Publisher boxes_max_pub;
  ros::Publisher centroid_pub;
  ros::Publisher deviation_pub;

  ros::Subscriber command_topic;

  std::string DEVICE = "MYRIAD";

  //std::string PATH_TO_DIR = "/home/pi/catkin_ws/src/object_detection/nets/";
  std::string PATH_TO_DIR = "/home/ksrve/catkin_ws/src/object_detection/nets/";

  std::string XML_MODEL_NAME = "person-detection-retail-0013.xml";
  std::string BIN_MODEL_NAME = "person-detection-retail-0013.bin";

  //std::string XML_MODEL_NAME = "face-detection-retail-0004.xml";
  //std::string BIN_MODEL_NAME = "face-detection-retail-0004.bin";

  std::string XML_MODEL_PATH = PATH_TO_DIR + XML_MODEL_NAME;
  std::string BIN_MODEL_PATH = PATH_TO_DIR + BIN_MODEL_NAME;

  const int cameraID = 0;

  bool ok;
  bool key = false;

  size_t input_width;
  size_t input_height;

  std::string imageInputName;
  std::string imageInfoInputName;

  size_t netInputHeight;
  size_t netInputWidth;

  bool FLAGS_auto_resize;

  Blob::Ptr inpuBlob;

  InferRequest::Ptr async_infer_request_curr;
  InferRequest::Ptr async_infer_request_next;

  int maxProposalCount;
  int objectSize;

  std::string outputName;

  InferenceEngine::Core ie;
  InferenceEngine::CNNNetReader netReader;
  InferenceEngine::ExecutableNetwork network;

  StatusCode ncsCode;

  cv::VideoCapture capture;
  cv::Mat curr_frame;
  cv::Mat next_frame;
  cv::Mat raw_frame;

  object_detection::BoxMax box_max;
  object_detection::Centroid centroid;
  object_detection::Deviation deviation;

  double area;

  bool is_telegram_online = false;
  std::string telegram_msg;
  bool isTracking;

  float MIN_CONFIDENCE = 0.8;

  std::vector<cv::Point2f> left_points;
  std::vector<cv::Point2f> right_points;
  cv::Point2f centroid_coords;

  float center_image_x;
  float center_image_y;

  cv::Size img_size;

  cv::Scalar whiteColor = cv::Scalar(255,255,255);
  cv::Scalar redColor = cv::Scalar(0, 0, 255);
  cv::Scalar greenColor = cv::Scalar(0, 255, 0);

  float xmin;
  float ymin;
  float xmax;
  float ymax;

  float nec_area;

  cv::Rect2d trackingBox;

  string trackerType = "MIL";

  Ptr<TrackerMIL> tracker = TrackerMIL::create();


};


#endif /* OBJECT_DETECTION_H */
