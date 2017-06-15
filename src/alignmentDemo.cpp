#include <iostream>

#include "Mocha/Receiver.h"
#include <ctime>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <unistd.h>

float findTransform(std::vector<std::pair<cv::Mat, cv::Mat>> CP, cv::Mat &R,
                    cv::Mat &t);
float findDistance(cv::Mat one, cv::Mat two);
cv::Mat vecToMat(tf::Vector3 in);
tf::Vector3 MatToVec(cv::Mat in);
tf::Quaternion averageQuat(tf::Quaternion acc, tf::Quaternion add);
cv::Mat MarkerArr[4];


std::map<std::string, tf::Transform> lastTransforms;

bool operator==( const tf::Transform& t1, const tf::Transform& t2 )
{
    return t1.getOrigin() == t2.getOrigin() && t1.getRotation() == t2.getRotation();
}

bool operator!=( const tf::Transform& t1, const tf::Transform& t2 )
{
    return !(t1.getOrigin() == t2.getOrigin() && t1.getRotation() == t2.getRotation());
}


void sendTransforms(Mocha::Frame::KPtr frame) { // This function is triggered
                                                // every time a new frame is
                                                // received
  static tf::TransformBroadcaster br;
  bool isGlasses = 0;
  static bool notFirst = 0;
  static cv::Mat R;
  static cv::Mat t;


  // std::cout<<"id "<<frame->getFrameNumber()<<" Latency:
  // "<<frame->getLatency()<<std::endl; // Information about the frame
  for (auto &name : frame->listRigidBodiesNames()) // For loop over all the
                                                   // bodies contained in the
                                                   // frame
  {
    const Mocha::RigidBody::KPtr &myBody(frame->getRigidBody(name));
    const Mocha::Structure::Point3D &pos(myBody->getPosition());
    const Mocha::Structure::Quaternion &orient(myBody->getOrientation());
    if (pos.x == 0.0 && pos.y == 0.0 && pos.z == 0.0) {
      continue;
    }

    // std::cout<<"Name: "<<name<< "   X: "<<pos.x<< "   Y: "<<pos.y<< "   Z:
    // "<<pos.z;  //global position of the rigid body
    // std::cout<<"   I: "<<orient.i<<"   J: "<<orient.j<<"   K: "<<orient.k<<"
    // W: "<<orient.w<<std::endl;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pos.x, -pos.z, pos.y));

    tf::Quaternion q(orient.i, -orient.k, orient.j, -1.0 * orient.w);
    q = q.inverse();
    // tf::Quaternion q(1.0*orient.i, orient.j, orient.k, 1.0*orient.w);
    // std::cout << "rot1:" << q.getX() << " " << q.getY() << " " << q.getZ() <<
    // " " << q.getW() << std::endl;
    transform.setRotation(q);
    if(lastTransforms.find(name) != lastTransforms.end()) {
        if(lastTransforms.at(name) != transform) {
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now()-ros::Duration(0,2000000*frame->getLatency()), "mocha_world", name));
        }
    }

    lastTransforms[name] = transform;

    std::vector<std::pair<cv::Mat, cv::Mat>> correspondingPoints;
    for (auto &markerName :
         myBody->listAssociatedMarkersNames()) // For loop over all the markers
                                               // contained in the rigid body.
    {
      const Mocha::Marker::KPtr &marker(
          myBody->getAssociatedMarker(markerName));
      const Mocha::Structure::Point3D &posMarker(marker->getPosition());
      std::string subString = markerName.substr(0, name.length());


      if (subString ==
          "glasses") { // rigid body has our name of interest as a root.
             isGlasses = 1;
          cv::Mat thisMarker = (cv::Mat_<float>(3, 1) << 1000 * posMarker.x,
                                -1000 * posMarker.z, 1000 * posMarker.y);

          std::string CharNumber = markerName.substr(name.length() + 1);
          int number = atoi(CharNumber.c_str());
          std::pair<cv::Mat, cv::Mat> thisPair;
          thisPair.second = thisMarker;
          thisPair.first = MarkerArr[number];
          correspondingPoints.push_back(thisPair);
//          std::cout <<"ball number:"<< number<< "x:"<< posMarker.x << "y:"<<
//          posMarker.y<< "z:"<< posMarker.z<< std::endl;

      }


    }
    if (isGlasses) {
      if (correspondingPoints.size() > 2)
        findTransform(correspondingPoints, R, t);
      notFirst = 1;
    }

    correspondingPoints.clear(); // empty the points.
  }

  if (notFirst) {

    tf::Transform glassesT;
    glassesT.setOrigin(tf::Vector3(1.0 * t.at<float>(0) / 1000.0,
                              t.at<float>(1) / 1000.0,
                              t.at<float>(2) / 1000.0));
    tf::Matrix3x3 rotMat(
        R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
        R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
        R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2));
    tf::Quaternion quat;
    rotMat.getRotation(quat);
    glassesT.setRotation(quat);
    br.sendTransform(
        tf::StampedTransform(glassesT, ros::Time::now(), "mocha_world", "glassesOut"));
  }
}
bool isMotive = 0;
int main(int argc, char **argv) {



  cv::Mat bot =
      (cv::Mat_<float>(3, 1) << -305.55, 363.83,1022.14);
  cv::Mat front =
      (cv::Mat_<float>(3, 1) <<-366.00, 379.30, 1040.10);
  cv::Mat side = (cv::Mat_<float>(3, 1) << -222.64, 419.35, 1061.97);
  cv::Mat top =
      (cv::Mat_<float>(3, 1) <<-257.31, 330.11, 1105.16);


  MarkerArr[0] = bot;
  MarkerArr[3] = front;
  MarkerArr[1] = side;
  MarkerArr[2] = top;

  /// end of calibrated values.
  ros::init(argc, argv, "ros_mocha_node");
  if (argc < 3) {
    std::cout << "Usage: rosrun ros_mocha ros_mocha_node IP_CLIENT IP_SERVER"
              << std::endl;
    return 0;
  }

  ros::NodeHandle node;
  (void)node;

  Mocha::Receiver receiver(argv[1], argv[2]); // creates the connection

  while (
      !receiver
           .hasServerInfo()) { // waits that the connection becomes established
    ros::Duration(0.1).sleep();
    if (!ros::ok()) {
      return 0;
    }
  }

  Mocha::ServerInfo info(receiver.getServerInfo()); // retrieve some information
  std::cout << "Server Info" << std::endl;
  std::cout << "Name: " << receiver.getServerInfo().name << std::endl;
  if (receiver.getServerInfo().name == "Motive")
    isMotive = 1;
  std::cout << "Version: " << info.version.major << " " << info.version.minor
            << " " << info.version.build << " " << info.version.revision
            << std::endl;

  std::function<void(Mocha::Frame::KPtr)> callback =
      sendTransforms; // Definition of the callback function
  receiver.addCallBack(
      callback); // adds the callback function to the list of callbacks

  cv::Mat help;
  std::string path = ros::package::getPath("ros_mocha");
  std::stringstream ss;
  ss << path << "/src/image.png";
  help = cv::imread(ss.str().c_str());
  cv::imshow("mochaNodeHelp", help);
  while (ros::ok()) {
    char k = cv::waitKey(1);

    static tf::TransformListener listener;
    static tf::TransformBroadcaster br;


//    br.sendTransform(tf::StampedTransform(opticalAxis, ros::Time::now(),
//                                          "inkjet", "CamCentreDummy"));
    // std::cout << "transform" << std::endl;
    // std::cout << "transform" << std::endl;
    ros::spinOnce();
  }

  return 0;
}

tf::Quaternion averageQuat(tf::Quaternion acc, tf::Quaternion add) {
  float x = 0.9 * acc.x() + 0.1 * add.x();
  float y = 0.9 * acc.y() + 0.1 * add.y();
  float z = 0.9 * acc.z() + 0.1 * add.z();
  float w = 0.9 * acc.w() + 0.1 * add.w();
  tf::Quaternion ret = tf::Quaternion(x, y, z, w);
  return ret;
}

cv::Mat vecToMat(tf::Vector3 in) {
  cv::Mat ret = (cv::Mat_<float>(3, 1) << in.x(), in.y(), in.z());
  return ret;
}
tf::Vector3 MatToVec(cv::Mat in) {
  tf::Vector3 ret(in.at<float>(0), in.at<float>(1), in.at<float>(2));
  return ret;
}

float findDistance(cv::Mat one, cv::Mat two) {
  float ans = 0;
  if (one.rows != 0 && two.rows != 0) {
    ans = sqrt(pow(one.at<float>(0, 0) - two.at<float>(0, 0), 2) +
               pow(one.at<float>(1, 0) - two.at<float>(1, 0), 2) +
               pow(one.at<float>(2, 0) - two.at<float>(2, 0), 2));
  }
  return ans;
}

float findTransform(std::vector<std::pair<cv::Mat, cv::Mat>> CP, cv::Mat &R,
                    cv::Mat &t) {
  cv::Mat ac = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
  cv::Mat bc = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
  int samples = CP.size();
  for (int i = 0; i < samples; i++) {
    ac += CP.at(i).first;
    // std::cout << "first" << i << " " << CP.at(i).first << std::endl;
    // std::cout << "second" << i << " " << CP.at(i).second << std::endl;
    bc += CP.at(i).second;
  }
  ac /= samples;
  bc /= samples;

  cv::Mat H = cv::Mat(3, 3, CV_32F, cv::Scalar(0));

  for (int i = 0; i < samples; i++) {
    cv::Mat bT;
    cv::transpose(CP.at(i).second - bc, bT);
    H += (CP.at(i).first - ac) * (bT);
  }

  cv::SVD svd(H);
  cv::Mat v;
  cv::transpose(svd.vt, v);
  cv::Mat ut;
  cv::transpose(svd.u, ut);
  R = v * ut;
  // std::cout << cv::determinant(v)<< std::endl;
  t = -R * ac + bc;
  float val = 0;
  for (int i = 0; i < samples; i++) {
    cv::Mat thePoint = (R * CP.at(i).first) + t;
    cv::Mat er = thePoint - CP.at(i).second;
    float thisVal = sqrt(pow(er.at<float>(0), 2) + pow(er.at<float>(1), 2) +
                         pow(er.at<float>(2), 2));
    // std::cout<< i << "  :" << thePoint << std::endl;
    val += thisVal;
  }
  val /= samples;
  // std::cout <<"averageVal :"<< val << std::endl;
  return val;
}
