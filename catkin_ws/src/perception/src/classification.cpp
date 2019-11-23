#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <perception/Classification.h>
#include <geometry_msgs/PoseStamped.h>
#include <unordered_map>
#include <random>
using namespace cv;
using namespace std;

class Classifier
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  ros::Publisher pub;
  // object_id -> (H %,V 0 - 100);
  // Hard coded: BAD but desperate
  unordered_map<string, vector<float>> object_to_hs;
  string black_totem;
  vector<string> polyform;
  float hsv_threshold;
  float black_threshold;


public:
  Classifier () : it(nh) {
    string camera_topic;
    ros::param::get("~camera_topic", camera_topic);
    sub = it.subscribe(camera_topic, 1,&Classifier::classificationCallBack, this);
    pub = nh.advertise<perception::Classification>("object_classification", 10);
    ros::param::get("~hsv_threshold", hsv_threshold);
    object_to_hs["yellow_totem"] = {62.6, 70.2};
    object_to_hs["red_totem"] = {355.5, 76.1};
    object_to_hs["blue_totem"] = {239.3, 69.0};
    object_to_hs["green_totem"] = {120.4, 71.0};
    object_to_hs["surmark46104"] = {0.0, 100.0};
    object_to_hs["surmark950400"] = {153.0, 51.0};
    object_to_hs["surmark950410"] = {3.7, 54.1};
    black_totem = "black_totem";
    polyform = {"polyform_a3", "polyform_a5", "polyform_a7"};
    black_threshold = 5.f;
  }

  vector<Rect> getObjectBoundingBoxes(cv_bridge::CvImagePtr& img_ptr) {
     // convert to gray scale
    Mat src_gray;
    cvtColor(img_ptr->image, src_gray, COLOR_BGR2GRAY);
    blur(src_gray, src_gray, Size(3, 3));

    // Perform canny edge detection
    Mat canny_output;
    Canny(src_gray, canny_output, 150, 200);

    // Find contours using output of canny
    vector<vector<Point>> contours;
    findContours(canny_output, contours, RETR_TREE, CHAIN_APPROX_TC89_L1);

    // Compute bounding rectangle for each object
    vector<vector<Point>> contours_poly (contours.size());
    vector<Rect> boundRect (contours.size());
    vector<Point2f> centers (contours.size());

    for (int i = 0; i < contours.size(); i++) {
      approxPolyDP(contours[i], contours_poly[i], 3, true);
      boundRect[i] = boundingRect (contours_poly[i]);
    }

    return boundRect;
  }

string computeObjectID(Rect& boundingBox, cv_bridge::CvImagePtr& img_ptr) {
  // Convert image to HSV
  Mat img_hsv;
  cvtColor(img_ptr->image, img_hsv, CV_BGR2HSV);

  // Compute average hsv value in bounding box
  Mat roi(img_hsv, boundingBox);
  Scalar hsv = mean(roi);

  // Try match on hue first and store all matches in vector
  vector<string> match_vector;
  for (unordered_map<string, vector<float>>::iterator it = object_to_hs.begin();
       it != object_to_hs.end();
       it++) {

      // First check for black
      if (hsv[2] < black_threshold) {

        // If aspect ratio is less than one return black_totem
        if ((boundingBox.width / boundingBox.height) < 1.f) return black_totem;

        // Else guess between the three possible polyforms
        default_random_engine generator;
        uniform_int_distribution<int> distribution(0,2);
        return polyform[distribution(generator)];
      }

      // Now match on hue
      float color_val = (it->second[0]) / 2.f;
      float hue_distance = min(abs(color_val - hsv[0]), 180 - abs(color_val - hsv[0]));

      if (hue_distance < hsv_threshold) match_vector.push_back(it->first);
  }

  // If size of match vector is 1 return this string
  if (match_vector.size() == 1) return match_vector[0];

  // Otherwise compare on V in HSV
  string best_match = "";
  float error = 0.f;
  for (auto object_id : match_vector) {
    float curr_error = abs(object_to_hs[object_id][1] - hsv[2]);
    if (curr_error < error) {
      best_match = object_id;
      error = curr_error;
    }
  }
  return best_match;
  }


  void classificationCallBack(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_INFO("Error with conversion to opencv.");
    return;
  }

  // Obtain bounding boxes
  vector<Rect> boundRect = getObjectBoundingBoxes(cv_ptr);

  // Create c++ map that finds object for bounding box
  vector<Rect> boundRec_filtered;
  vector<string> class_map;
  for (auto boundingBox: boundRect) {

    // If aspect ratio (width / height) > 1, don't add to map as it is
    // probably not an object
    if ((boundingBox.width / boundingBox.height) > 1.1f) continue;
    if ((boundingBox.width) * (boundingBox.height) < 5.f) continue;

    // Compute Mean HSV value for pixels in bounding box
    std::string object_id = computeObjectID(boundingBox, cv_ptr);
    if (object_id.compare("") == 0) continue;
    class_map.push_back(object_id);
    boundRec_filtered.push_back(boundingBox);
  }

  ROS_INFO("Number of objects: %d", boundRec_filtered.size());
  perception::Classification classification_msg;
  // Iterate over all mapped objects and publish msg
  for (int i = 0 ; i < class_map.size(); i++) {
    geometry_msgs::PoseStamped object;
    object.header.frame_id = class_map[i];
    object.pose.position.x = (boundRec_filtered[i]).x + (boundRec_filtered[i]).width / 2.f;
    object.pose.position.y = (boundRec_filtered[i]).y - (boundRec_filtered[i]).height / 2.f;
    classification_msg.objects.push_back(object);
  }
  pub.publish(classification_msg);
  }
};


int main(int argc, char** argv) {

  ros::init(argc, argv, "classification_server");
  Classifier c;
  ros::spin();
  return 0;
}
