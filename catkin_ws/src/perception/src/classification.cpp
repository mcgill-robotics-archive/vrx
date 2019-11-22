#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;

class BoundingBox
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  image_transport::Publisher pub;

public:
  BoundingBox () : it(nh) {
    string camera_topic;
    ros::param::get("~camera_topic", camera_topic);
    sub = it.subscribe(camera_topic, 1,&BoundingBox::classificationCallBack, this);
    pub = it.advertise("bounding_debug", 1);
  }

  void classificationCallBack(const sensor_msgs::ImageConstPtr& msg) {


  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_INFO("Error with conversion to opencv.");
  }

  // convert to gray scale
  cv_bridge::CvImagePtr src_gray;
  cvtColor(cv_ptr->image, src_gray->image, COLOR_BGR2GRAY);
  blur(src_gray->image, src_gray->image, Size(3, 3));

  // Perform canny edge detection
  cv_bridge::CvImagePtr canny_output;
  Canny(src_gray->image, canny_output->image, 100, 200);

  // Find contours using output of canny
  vector<vector<Point>> contours;
  findContours(canny_output->image, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

  // Compute bounding rectangle for each object
  vector<vector<Point>> contours_poly (contours.size());
  vector<Rect> boundRect (contours.size());
  vector<Point2f> centers (contours.size());

  for (int i = 0; i < contours.size(); i++) {
    approxPolyDP(contours[i], contours_poly[i], 3, true);
    boundRect[i] = boundingRect (contours_poly[i]);
  }

  cv_bridge::CvImagePtr ptr_final;
  ptr_final->image = Mat::zeros (canny_output->image.size(), CV_8UC3);
  RNG rng(12345);

  for (int i = 0 ; i < contours.size(); i++) {
    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    drawContours(ptr_final->image, contours_poly, (int) i, color);
    rectangle(ptr_final->image, boundRect[i].tl(), boundRect[i].br(), color, 2);
  }

  pub.publish(ptr_final->toImageMsg());
  }
};


int main(int argc, char** argv) {

  ros::init(argc, argv, "classification_server");
  BoundingBox bb;
  ros::spin();
  return 0;
}

// Convert to gray scale
// Perform canny edge detection
// Find bounding boxes for object contours
// Average pixel values in bounding box to classify object
