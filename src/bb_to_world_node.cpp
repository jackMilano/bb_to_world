#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <projected_game_msgs/Pose2DStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tld_msgs/BoundingBox.h>
//#include <cv_bridge/cv_bridge.h>
//#include <pcl/filters/filter.h>

#include <bb_to_world/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>

#include <cmath> // abs, round
//#include <stdint.h>

#include <dynamic_reconfigure/server.h>
#include <bb_to_world/BBToWorldConfig.h>

#define END_FRAME "/world"
#define FIRST_FRAMES 30
#define MIN_CONFIDENCE 0.1f
//#define __DEPRECATED


typedef pcl::PointXYZRGB PointT;
typedef tf::Stamped<tf::Vector3> StampedPoint;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, tld_msgs::BoundingBox>
MySyncPolicy;

// Global variables
ros::Publisher showMePointCloud;
ros::Subscriber depthCameraInfoSub;
image_geometry::PinholeCameraModel cam_model_;
// Altrimenti 'StereoCameraModel' per una coppia di pinhole camera.
double unit_scaling;
float center_x;
float center_y;
float constant_x;
float constant_y;
bool camera_info_received;
double z_thresh = -1.0;
double min_confidence = MIN_CONFIDENCE;

void dynamicReconfCb(bb_to_world::BBToWorldConfig &conf, uint32_t level)
{
  z_thresh = conf.z_threshold;
  min_confidence = conf.min_confidence;
}

void depthCameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info)
{
  // load the camera model structure with the pinhole model
  //  (Intrinsic + distortion coefficients) of the IR camera (depth)
  cam_model_.fromCameraInfo(depth_camera_info);

  ROS_INFO("Depth Camera Model Loaded");

  //do it once since the camera model is constant
  depthCameraInfoSub.shutdown();

  ROS_INFO("Camera Info subscriber shut down");

  // Use correct principal point from calibration
  center_x = cam_model_.cx();
  center_y = cam_model_.cy();
  ROS_DEBUG("center_x = %f, center_y = %f.", center_x, center_y);

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  unit_scaling = depth_image_proc::DepthTraits<uint16_t>::toMeters( uint16_t(1) );
  ROS_DEBUG("unit_scaling = %f", unit_scaling);
  constant_x = unit_scaling / cam_model_.fx();
  constant_y = unit_scaling / cam_model_.fy();
  ROS_DEBUG("constant_x = %f, constant_y = %f.", constant_x, constant_y);

  camera_info_received = true;

  ROS_DEBUG("depthCameraInfoCb end.");

  return;
}

void boundingBoxCallback(
  const sensor_msgs::Image::ConstPtr& sensorDepthImage,
  const tld_msgs::BoundingBox::ConstPtr& bBox,
  const std::string endFrame,
  const tf::TransformListener* transformer,
  const ros::Publisher* robotPosePub)
{
  //ROS_DEBUG("boundingBoxCallback started.");

  static int seq; // Sequence number dei pacchetti da me inviati.
  //bool sameFrame = (bBox->header.frame_id.compare(sensorDepthImage->header.frame_id) == 0);

  // non inviare quando la confidenza è bassa
  if( bBox->confidence < min_confidence )
  {
    ROS_INFO("Confidence is too low!");
    return;
  }

  /*if( seq < FIRST_FRAMES )
  {
    ROS_INFO("Sono entrato: n.%d.", seq);
  }*/

  try
  {
    // Trasforma le coordinate del'angolo sinistro della Bounding Box,
    //  calcolate nel frame della Bounding Box, nel frame della Depth Map.
    StampedPoint stampedPointTopLeft;
    {
      StampedPoint stampedPointIn;
      stampedPointIn.frame_id_ = bBox->header.frame_id;
      stampedPointIn.stamp_ = bBox->header.stamp;
      stampedPointIn.setData( tf::Point(bBox->x, bBox->y, 0) );
      transformer->transformPoint(sensorDepthImage->header.frame_id, stampedPointIn, stampedPointTopLeft);
    }

    //ROS_DEBUG("Before ROI.");

    // Viene creata una Point Cloud corrispondente alle coordinate della Bounding Box.
    pcl::PointCloud<pcl::PointXYZ> bbPcl;
    pcl_conversions::toPCL(sensorDepthImage->header, bbPcl.header);
    bbPcl.height = 1; // unorganized PC
    bbPcl.is_dense = true; // does not contain NaN/Inf
    {
      const cv::Rect roi_rect(stampedPointTopLeft.getX(), stampedPointTopLeft.getY(), bBox->width,
                              bBox->height);
      ROS_DEBUG("Rect x %d, Rect y: %d.", roi_rect.x, roi_rect.y);
      const int img_cols = sensorDepthImage->width;
      ROS_DEBUG("img_cols = %d.", img_cols);
      //const int image_size = img_cols * sensorDepthImage->height;
      const int image_size = sensorDepthImage->data.size();
      int rows=-1, cols=-1;
      int printDebug = 1;
      int pix=-1;
      for(int i=0; i<image_size; i+=2)
      {
        ++pix;
        ++cols;
        //if(i%depth_img.cols == 0)
        if(pix%img_cols == 0)
        //if(i%img_cols == 0)
        {
          ++rows;
          cols = 0;
        }

        //ROS_DEBUG("cx %f cy %f fx %f fy %f", center_x, center_y, constant_x, constant_y);

        // Dando per scontato che l'immagine sia codificata su 16 bit e
        //  che sia little endian.
        const uint16_t pDepth = sensorDepthImage->data[i] | (sensorDepthImage->data[i+1] << 8); //'data' è 1 Byte
        //const uint8_t pDepth = sensorDepthImage->data[i];
        if( pDepth > 0 && roi_rect.contains(cv::Point(cols,rows)) )
        {
          if(printDebug)
          {
            ROS_DEBUG("point %d %d",cols, rows );
            printDebug = 0;
          }
          pcl::PointXYZ pclPoint;
          pclPoint.x = (cols - center_x) * pDepth * constant_x;
          pclPoint.y = (rows - center_y) * pDepth * constant_y;
          pclPoint.z = depth_image_proc::DepthTraits<uint16_t>::toMeters(pDepth);

          bbPcl.points.push_back(pclPoint);
          //ROS_DEBUG("Aggiunto il punto.");
        }
      }
    }
    bbPcl.width = bbPcl.points.size();
    ROS_DEBUG("points size: %lu", bbPcl.points.size());
    ROS_DEBUG("bbPcl width: %d", bbPcl.width);
    ROS_DEBUG("bbPcl height: %d", bbPcl.height);

    //trasforma nel sistema di riferimento finale
    if(!transformer->waitForTransform(endFrame, bbPcl.header.frame_id, sensorDepthImage->header.stamp, ros::Duration(5.0)))
    {
      ROS_ERROR("Wait for transform timed out.");
      return;
    }

    pcl_ros::transformPointCloud(endFrame, bbPcl, bbPcl, *transformer);

    pcl::PointCloud<pcl::PointXYZ> refinedPcl;
    refinedPcl.header.frame_id = endFrame;
    refinedPcl.height = 1; // unorganized PC
    refinedPcl.is_dense = true; // does not contain NaN/Inf

    for(int i = 0; i < bbPcl.points.size(); ++i)
    {
      if(bbPcl.points[i].z > z_thresh)
        refinedPcl.points.push_back(bbPcl.points[i]);
    }
    ROS_DEBUG("%lu points, threshold %f", refinedPcl.points.size(), z_thresh);
    refinedPcl.width = refinedPcl.points.size();

    // NaN Check
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(bbPcl, bbPcl, indices);

    // Calcolo del centroide (nel sistema di riferimento della Depth Map)
    Eigen::Vector4d centroid;
      if ( pcl::compute3DCentroid(refinedPcl, centroid) == 0 )
      {
        ROS_ERROR("centroid not computed!");
        return;
      }


    //ROS_DEBUG("centroid (depth map): %.2f %.2f %.2f", stampedCentroid.getX(), stampedCentroid.getY(), stampedCentroid.getZ());

    // Mostra la PointCloud corrispondente alla Bounding Box.
    sensor_msgs::PointCloud2 bbPclMsg;
    pcl::toROSMsg(refinedPcl, bbPclMsg);
    showMePointCloud.publish(bbPclMsg);

    // Conversione dal frame della Depth Map al frame finale (endFrame)
//    StampedPoint stampedPointEnd;
//    transformer->transformPoint(endFrame, stampedCentroid, stampedPointEnd);
//    ROS_DEBUG("centroid (end frame): %.2f %.2f %.2f", stampedPointEnd.getX(), stampedPointEnd.getY(), stampedPointEnd.getZ());

    // Preparazione del msg Pose2DStamped per essere inviato
    projected_game_msgs::Pose2DStamped pose2DStampedMsg;
    pose2DStampedMsg.header.frame_id = endFrame;
    pose2DStampedMsg.header.seq = seq++;
    pose2DStampedMsg.header.stamp = sensorDepthImage->header.stamp;
    pose2DStampedMsg.pose.theta = 0.0;
    pose2DStampedMsg.pose.x = centroid(0);
    pose2DStampedMsg.pose.y = centroid(1);

    ROS_DEBUG("Before publishing.");

    // Pubblicazione delle coordinate del punto in un topic
    robotPosePub->publish(pose2DStampedMsg);
  }
  catch(const std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bb_to_world_node");

  std::string endFrame;
  camera_info_received = false;

  // Check arguments
  if( argc != 2 )
  {
    ROS_ERROR("Usage: %s <end_frame>.", argv[0]);
    endFrame = END_FRAME;
  }
  else
  {
    endFrame = argv[1];
  }
  ROS_DEBUG("endFrame = %s.", endFrame.c_str());

  ros::NodeHandle node, nh_priv("~");

  double rate_hz;
  nh_priv.param("z_threshold", z_thresh, -1.0);
  nh_priv.param("min_confidence", min_confidence, 0.5);
  nh_priv.param("rate", rate_hz, 60.0);

  dynamic_reconfigure::Server<bb_to_world::BBToWorldConfig> dynamic_reconf_server;
  dynamic_reconf_server.setCallback(boost::bind(dynamicReconfCb, _1, _2));

  message_filters::Subscriber<sensor_msgs::Image> imageSub(node, "camera/image", 1);
  message_filters::Subscriber<tld_msgs::BoundingBox> bBoxSub(node, "tracker/bounding_box", 1);

  tf::TransformListener transformer;

  ros::Publisher robotPosePub = node.advertise<projected_game_msgs::Pose2DStamped> ("robot_pose", 1);

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageSub, bBoxSub);
  sync.registerCallback(boost::bind(&boundingBoxCallback, _1, _2, endFrame, &transformer,
                                    &robotPosePub));

  depthCameraInfoSub = node.subscribe("camera/camera_info", 1, depthCameraInfoCb);

  showMePointCloud = node.advertise<sensor_msgs::PointCloud2>("bbPointCloud", 1);

  ros::Rate rate(rate_hz);

  while(!camera_info_received)
  {
    ros::spinOnce();
    rate.sleep();
  }
  
  ros::spin();

  return 0;
}
