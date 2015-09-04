// Il nodo si occupa di convertire le coordinate dell'oggetto tracckato
// dal suo sistema di riferimento a quello fisso del mondo ('world')

// ROS library
#include <ros/ros.h>

// ROS messages
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <projected_game_msgs/Pose2DStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tld_msgs/BoundingBox.h>

// PCL libraries
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// Dynamic Reconfigure
#include <bb_to_world/BBToWorldConfig.h>
#include <dynamic_reconfigure/server.h>

// TF libraries
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <bb_to_world/depth_traits.h> // 'depth_image_proc'
#include <image_geometry/pinhole_camera_model.h>

// C libraries
#include <cmath> // abs, round

// Defines
#define MIN_CONFIDENCE 0.1f
#define TARGET_FRAME_DEF "/world"
#define POINT_TOO_HIGH 0.3 // meters (roomba is 0.08 m ca)
#define WAIT_TRANS_TIME 5.0

// Typedefs and Enums
// XXX: attenzione sta venendo utilizzata la politica 'ExactTime'.
// 'ApproximateTime' policy uses an adaptive algorithm to match messages based on their timestamp.
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, tld_msgs::BoundingBox> MySyncPolicy;
typedef pcl::PointXYZRGB PointT;
typedef tf::Stamped<tf::Vector3> StampedPoint;

// Global variables
image_geometry::PinholeCameraModel cam_model_;
//projected_game_msgs::Pose2DStamped old_pose_2D_stamped_msg;
geometry_msgs::PoseWithCovarianceStamped old_pose_2D_stamped_msg;
ros::Publisher show_me_point_cloud;
ros::Subscriber depth_camera_info_sub;
bool camera_info_received;
double min_confidence;
double point_max_height;
double unit_scaling;
double z_thresh;
float center_x;
float center_y;
float constant_x;
float constant_y;
//uint16_t p_depth_thresh;


void dynamicReconfCb(bb_to_world::BBToWorldConfig &conf, uint32_t level)
{
  min_confidence = conf.min_confidence;
  ROS_DEBUG("min_confidence has changed: %f.", min_confidence);

  point_max_height = conf.point_max_height;
  ROS_DEBUG("point_max_height has changed: %f.", point_max_height);

  z_thresh = conf.z_threshold;
  ROS_DEBUG("z_thresh has changed: %f.", z_thresh);
}

void depthCameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info)
{
  // load the camera model structure with the pinhole model
  //  (Intrinsic + distortion coefficients) of the IR camera (depth)
  cam_model_.fromCameraInfo(depth_camera_info);

  ROS_INFO("Depth Camera Model Loaded");

  //do it once since the camera model is constant
  depth_camera_info_sub.shutdown();

  ROS_INFO("Camera Info subscriber shut down");

  // Use correct principal point from calibration
  center_x = cam_model_.cx();
  center_y = cam_model_.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  unit_scaling = depth_image_proc::DepthTraits<uint16_t>::toMeters( uint16_t(1) );
  constant_x = unit_scaling / cam_model_.fx();
  constant_y = unit_scaling / cam_model_.fy();

  camera_info_received = true;

  ROS_INFO("depthCameraInfoCb end.");

  return;
}

void boundingBoxCallback(
    const sensor_msgs::Image::ConstPtr& sensor_depth_image,
    const tld_msgs::BoundingBox::ConstPtr& b_box,
    const std::string target_frame,
    const tf::TransformListener* transformer,
    const ros::Publisher* robot_pose_pub,
    const ros::Publisher* robot_pose_to_localization_pub)
{
  static int seq; // Sequence number of the packages sent from this node.

  try
  {
    // Quando la confidenza è bassa viene segnalato,
    if( b_box->confidence < min_confidence )
    {
      ROS_WARN("Confidence is too low! Confidence = %.2f.", b_box->confidence);
      ROS_WARN("I'm publishing the last valid transform.\n");

      // Le vecchie coordinate del punto vengono stampate in un topic.
      // - serve a 'robot_localization'
      {
        // XXX: se la confidenza e' troppo bassa, allora la covarianza del pacchetto deve venire
        //      aumentata
        boost::array<double, 36ul> new_pose_covariance =
        { 1e3, 0, 0, 0, 0, 0,                                         // larger covariance on visual tracking x
          0, 1e3, 0, 0, 0, 0,                                         // larger covariance on visual tracking y
          0, 0, 1e-6, 0, 0, 0,                                         // small covariance on visual tracking z
          0, 0, 0, 1e6, 0, 0,                                          // huge covariance on rot x
          0, 0, 0, 0, 1e6, 0,                                          // huge covariance on rot y
          0, 0, 0, 0, 0, 1e6};                                         // huge covariance on rot z
        old_pose_2D_stamped_msg.pose.covariance = new_pose_covariance;
      }

      old_pose_2D_stamped_msg.header.seq = seq;

      // XXX: nel momento in cui si pubblica bisogna inserirlo nel 'try'
      robot_pose_to_localization_pub->publish(old_pose_2D_stamped_msg);

      seq++;

      return;
    }

    // Trasforma le coordinate dell'angolo sinistro della Bounding Box,
    //  dal frame della Bounding Box, al frame della Depth Map.
    StampedPoint stamped_point_top_left;
    {
      StampedPoint stamped_point_in;
      stamped_point_in.frame_id_ = b_box->header.frame_id;
      stamped_point_in.stamp_ = b_box->header.stamp;
      stamped_point_in.setData( tf::Point(b_box->x, b_box->y, 0) );
      transformer->transformPoint(sensor_depth_image->header.frame_id, stamped_point_in, stamped_point_top_left);
    }

    // bb_pcl e' la Point Cloud costruita a partire dai punti contenuti all'interno della bounding box,
    //  in modo da poterne estrarre il centroide, per poterlo usare come rappresentativo della posizione
    //  dell oggetto tracciato.
    pcl::PointCloud<pcl::PointXYZ> bb_pcl;
    bb_pcl.header.frame_id = target_frame;
    //bb_pcl.header.frame_id = sensor_depth_image->header.frame_id;
    bb_pcl.height = 1;      // unorganized PC
    bb_pcl.is_dense = true; // does not contain NaN/Inf

    // Non appena e' disponibile la transform tra il sistema di riferimento finale
    // (world) e quello dell'immagine depth, si procede all'operazione di estrazione
    // dei punti.
    if(!transformer->waitForTransform(target_frame, bb_pcl.header.frame_id,
          sensor_depth_image->header.stamp, ros::Duration(WAIT_TRANS_TIME)))
    {
      ROS_ERROR("Wait for transform timed out.");
      return;
    }

    {
      // cv::Rect corrispondente alla Bounding Box nel sistema di riferimento
      //  dell'immagine di profondita' (optical frame)
      const cv::Rect roi_rect(stamped_point_top_left.getX(), stamped_point_top_left.getY(), b_box->width, b_box->height);
      const int image_size = sensor_depth_image->data.size();
      const int img_cols = sensor_depth_image->width;
      int pix=-1;
      int print_debug = 1;
      int rows=-1, cols=-1;
      for(int i=0; i<image_size; i+=2)
      {
        // per ogni pixel dell'immagine di profondita' viene calcolata la corrispondente
        //  posizione in termine di righe e colonne
        ++pix;
        ++cols;
        if(pix%img_cols == 0)
        {
          ++rows;
          cols = 0;
        }

        // L'immagine ottenuta da '/kinect2/xxd/image_depth_rect' ha una codifica little endian su 16 bit.
        //  - little endian: least significant bit all'indirizzo minore
        //  - 'data' è di tipo uint8, per cui bisogna prendere insieme 2 valori consecutivi per
        //    ottenere un valore corretto
        //  - p_depth corrisponde alla profondita' del pixel, in mm, rispetto all'optical_frame
        const uint16_t p_depth = sensor_depth_image->data[i] | (sensor_depth_image->data[i+1] << 8);

        // Il punto viene aggiunto alla point cloud se:
        // 1. è all'interno della bounding box
        // 2. non è attaccato alla camera (p_depth = 0)
        if( roi_rect.contains(cv::Point(cols,rows)) && p_depth > 0 )
        {
          if(print_debug)
          {
            ROS_DEBUG("point %d %d", cols, rows);
            print_debug = 0;
          }

          //pcl::PointXYZ pcl_point;
          //pcl_point.x = (cols - center_x) * p_depth * constant_x;
          //pcl_point.y = (rows - center_y) * p_depth * constant_y;
          //pcl_point.z = depth_image_proc::DepthTraits<uint16_t>::toMeters(p_depth);

          StampedPoint stamped_point_bounding_box_world;
          {
            // Vengono calcolate le coordinate tridimensionali del punto.
            const tfScalar tf_x = (cols - center_x) * p_depth * constant_x;
            const tfScalar tf_y = (rows - center_y) * p_depth * constant_y;
            const tfScalar tf_z = depth_image_proc::DepthTraits<uint16_t>::toMeters(p_depth); // From mm to m.
            const tf::Point tf_point_bounding_box_optical(tf_x, tf_y, tf_z);

            // Dal sistema di riferimento dell'immagine di profondita'
            // si passa al sistema di riferimento del 'target_frame'.
            StampedPoint stamped_point_bounding_box_optical;
            stamped_point_bounding_box_optical.frame_id_ = sensor_depth_image->header.frame_id;
            stamped_point_bounding_box_optical.stamp_ = sensor_depth_image->header.stamp;
            stamped_point_bounding_box_optical.setData( tf_point_bounding_box_optical );
            transformer->transformPoint(target_frame, stamped_point_bounding_box_optical, stamped_point_bounding_box_world);
          }

          pcl::PointXYZ pcl_point;
          pcl_point.x = stamped_point_bounding_box_world.getX();
          pcl_point.y = stamped_point_bounding_box_world.getY();
          pcl_point.z = stamped_point_bounding_box_world.getZ();

          // come condizioni aggiuntive il punto viene aggiunto alla bounding box se
          // 1. non è in una posizione troppo elevata rispetto al suolo
          //  - l'occlusione parziale della bounding box a causa del passaggio
          //    di una persona causa il calcolo errato della posizione del centroide
          // 2. l'altezza supera una certa soglia 'z_thresh'
          //  - in questo modo non andiamo a prendere punti nel pavimento ed il calcolo del centroide
          //    diventa piu' preciso
          //if( pcl_point.z < POINT_TOO_HIGH  && pcl_point.z > z_thresh )
          if( pcl_point.z < point_max_height  && pcl_point.z > z_thresh )
          {
            bb_pcl.points.push_back(pcl_point);
          }
        }
      }   // for loop
    }

    bb_pcl.width = bb_pcl.points.size();
    ROS_DEBUG("bb_pcl points size: %lu", bb_pcl.points.size());
    ROS_DEBUG("bb_pcl width: %d", bb_pcl.width);
    ROS_DEBUG("bb_pcl height: %d", bb_pcl.height);

    // Non appena e' disponibile la transform tra il sistema di riferimento finale
    //  (world) e quello dell'immagine depth, trasforma la 'bb_pcl' dal secondo
    //  al primo.
    //if(!transformer->waitForTransform(target_frame, bb_pcl.header.frame_id,
    //      sensor_depth_image->header.stamp, ros::Duration(WAIT_TRANS_TIME)))
    //{
    //  ROS_ERROR("Wait for transform timed out.");
    //  return;
    //}
    //pcl_ros::transformPointCloud(target_frame, bb_pcl, bb_pcl, *transformer);

    // 'refined_pcl' è una Point Cloud che contiene solamente
    //  i punti dati dalla bounding box del tracker
    //  che abbiano un'elevazione superiore a 'z_thresh'
    //  in questo modo il calcolo del centroide diventa più preciso.
    //pcl::PointCloud<pcl::PointXYZ> refined_pcl;
    //refined_pcl.header.frame_id = target_frame;
    //refined_pcl.height = 1;       // unorganized PC
    //refined_pcl.is_dense = true;  // does not contain NaN/Inf
    //for(int i = 0; i < bb_pcl.points.size(); ++i)
    //{
    //  if(bb_pcl.points[i].z > z_thresh)
    //  {
    //    refined_pcl.points.push_back(bb_pcl.points[i]);
    //  }
    //}
    //ROS_DEBUG("'refined_pcl' %lu points, z_thresh %f", refined_pcl.points.size(), z_thresh);
    //refined_pcl.width = refined_pcl.points.size();

    // Se il calcolo del centroide fallisce, non viene pubblicato niente.
    //if ( pcl::compute3DCentroid(refined_pcl, centroid) == 0 )
    Eigen::Vector4d centroid;
    if ( pcl::compute3DCentroid(bb_pcl, centroid) == 0 )
    {
      ROS_ERROR("centroid not computed! z_thresh = %.2f. bb_pcl.points.size() = %d.", z_thresh, (int) bb_pcl.points.size());

      return;
    }

    // Viene pubblicata la Point Cloud
    sensor_msgs::PointCloud2 bb_pcl_msg;
    //pcl::toROSMsg(refined_pcl, bb_pcl_msg);
    pcl::toROSMsg(bb_pcl, bb_pcl_msg);
    show_me_point_cloud.publish(bb_pcl_msg);

    // Preparazione del msg Pose2DStamped per essere inviato
    projected_game_msgs::Pose2DStamped pose_2D_stamped_msg;
    pose_2D_stamped_msg.header.frame_id = target_frame;
    pose_2D_stamped_msg.header.seq = seq;
    pose_2D_stamped_msg.header.stamp = sensor_depth_image->header.stamp;
    pose_2D_stamped_msg.pose.theta = 0.0;
    pose_2D_stamped_msg.pose.x = centroid(0);
    pose_2D_stamped_msg.pose.y = centroid(1);

    // Pubblicazione delle coordinate del punto in un topic
    robot_pose_pub->publish(pose_2D_stamped_msg);

    // Preparazione del msg PoseWithCovarianceStamped per essere inviato
    geometry_msgs::PoseWithCovarianceStamped geom_pose_2D_stamped_msg;
    geom_pose_2D_stamped_msg.header.frame_id = target_frame;
    geom_pose_2D_stamped_msg.header.seq = seq;
    geom_pose_2D_stamped_msg.header.stamp = sensor_depth_image->header.stamp;
    geom_pose_2D_stamped_msg.pose.pose.orientation.w = 1;
    geom_pose_2D_stamped_msg.pose.pose.orientation.x = 0;
    geom_pose_2D_stamped_msg.pose.pose.orientation.y = 0;
    geom_pose_2D_stamped_msg.pose.pose.orientation.z = 0;
    geom_pose_2D_stamped_msg.pose.pose.position.x = centroid(0);
    geom_pose_2D_stamped_msg.pose.pose.position.y = centroid(1);
    geom_pose_2D_stamped_msg.pose.pose.position.z = 0;
    {
      // XXX: il pacchetto arriva a 'robot_localization'
      boost::array<double, 36ul> pose_covariance =
      { 1e-2, 0, 0, 0, 0, 0,                                         // small covariance on visual tracking x
        0, 1e-2, 0, 0, 0, 0,                                         // small covariance on visual tracking y
        0, 0, 1e-6, 0, 0, 0,                                         // small covariance on visual tracking z
        0, 0, 0, 1e6, 0, 0,                                          // huge covariance on rot x
        0, 0, 0, 0, 1e6, 0,                                          // huge covariance on rot y
        0, 0, 0, 0, 0, 1e6};                                         // huge covariance on rot z
      geom_pose_2D_stamped_msg.pose.covariance = pose_covariance;
    }

    // Pubblicazione delle coordinate del punto in un topic
    robot_pose_to_localization_pub->publish(geom_pose_2D_stamped_msg);

    // Salviamo l'ultimo pacchetto inviato
    old_pose_2D_stamped_msg = geom_pose_2D_stamped_msg;

    // after the dispatch the sequence number is incremented
    seq++;
  }
  catch(const tf::TransformException& tf_ex)
  {
    ROS_ERROR("%s", tf_ex.what());
    ros::Duration(1.0).sleep();
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

  std::string target_frame;
  camera_info_received = false;
  min_confidence = MIN_CONFIDENCE;
  point_max_height = POINT_TOO_HIGH;
  z_thresh = -1.0;

  // Check arguments
  if( argc != 2 )
  {
    ROS_WARN("Usage: %s <target_frame>.\n'target_frame' set to %s.", argv[0], TARGET_FRAME_DEF);
    target_frame = TARGET_FRAME_DEF;
  }
  else
  {
    target_frame = argv[1];
  }
  ROS_DEBUG("target_frame = %s.", target_frame.c_str());

  ros::NodeHandle node, nh_priv("~");

  double rate_hz;
  nh_priv.param("min_confidence", min_confidence, 0.5);
  nh_priv.param("rate", rate_hz, 60.0);
  nh_priv.param("z_threshold", z_thresh, -1.0);
  nh_priv.param("point_max_height", point_max_height, 0.5);

  dynamic_reconfigure::Server<bb_to_world::BBToWorldConfig> dynamic_reconf_server;
  dynamic_reconf_server.setCallback(boost::bind(dynamicReconfCb, _1, _2));

  // Sincronizzazione tra 2 canali di ingresso
  //  (l'immagine di profondita' e la posizione restituita dal tracker visuale)
  message_filters::Subscriber<sensor_msgs::Image> image_sub(node, "camera/image", 1);
  message_filters::Subscriber<tld_msgs::BoundingBox> b_box_sub(node, "tracker/bounding_box", 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, b_box_sub);

  tf::TransformListener transformer;

  // Viene pubblicata la posa 2D del robot, ottenuta dal tracking visuale,
  //  convertita nelle cordinate del mondo.
  ros::Publisher robot_pose_pub = node.advertise<projected_game_msgs::Pose2DStamped> ("robot_2d_pose", 1);

  // Viene pubblicata la posa 2D del robot, come se fosse un messaggio PoseWithCovarianceStamped,
  //  in modo da poterla inviare al nodo 'robot_localization' per la sensor fusion.
  ros::Publisher robot_pose_to_localization_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped> ("robot_2d_geometry_pose", 1);

  sync.registerCallback(
      boost::bind(
        &boundingBoxCallback,
        _1, _2,
        target_frame,
        &transformer,
        &robot_pose_pub,
        &robot_pose_to_localization_pub));

  // This callback will be performed once (camera model is costant).
  depth_camera_info_sub = node.subscribe("camera/camera_info", 1, depthCameraInfoCb);

  show_me_point_cloud = node.advertise<sensor_msgs::PointCloud2>("bb_point_cloud", 1);

  sleep(2); //sleep 2 seconds. Wait 'openni_launch' to come up.

  // 'ros::Rate' makes a best effort at mantaining a particular rate for a loop.
  ros::Rate rate(rate_hz);

  // Non appena sono arrivate le informazioni sulla camera,
  //  attiviamo la boundingBoxCallback.
  while(!camera_info_received)
  {
    ros::spinOnce();
    rate.sleep();
  }

  ros::spin();

  return 0;
}
