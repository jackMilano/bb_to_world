// Il nodo prende in ingresso la Bounding Box corrispondente al target restituita dal Visual Tracker, nel sistema di
// riferimento della camera, e restituisce un messaggio di `Pose` contenente la posizione del target nel sistema di
// riferimento "output frame" (tipicamente il frame del mondo).

// Il nodo si occupa anche di "bloccare" la pubblicazione dei messaggi in determinati casi:
// - confidenza nulla (lost tracking)
// - messaggio ricevuto dal nodo `reset_visual_tracker` (falso positivo)


// ROS library
#include <ros/ros.h>

// ROS messages
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <tld_msgs/BoundingBox.h>
//#include <projected_game_msgs/Pose2DStamped.h>

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

// Parametri camera
#include <bb_to_world/depth_traits.h> // 'depth_image_proc'
#include <image_geometry/pinhole_camera_model.h>

// C libraries
#include <cmath> // abs, round

// Defines
#define MIN_CONFIDENCE 0.1f
#define POINT_TOO_HIGH 0.3f // meters (roomba is 0.08 m ca)
#define TARGET_FRAME_DEF "/world"
#define WAIT_TRANS_TIME 5.0f

// Typedefs and Enums
// Notare che sta venendo utilizzata la politica 'ExactTime'.
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, tld_msgs::BoundingBox> MySyncPolicy;

typedef pcl::PointXYZRGB PointT;
typedef tf::Stamped<tf::Vector3> StampedPoint;

// Global variables
//ros::Publisher show_me_point_cloud;
image_geometry::PinholeCameraModel cam_model_;
ros::Publisher robot_pose_rviz_pub;
ros::Subscriber depth_camera_info_sub;
ros::Subscriber is_false_positive_sub;
bool camera_info_received;
bool is_false_positive;
double min_confidence;
double point_max_height;
double unit_scaling;
double z_thresh;
float center_x;
float center_y;
float constant_x;
float constant_y;


void dynamicReconfCb(bb_to_world::BBToWorldConfig& conf, uint32_t level)
{
  min_confidence = conf.min_confidence;
  ROS_DEBUG("min_confidence has changed: %f.", min_confidence);

  point_max_height = conf.point_max_height;
  ROS_DEBUG("point_max_height has changed: %f.", point_max_height);

  z_thresh = conf.z_threshold;
  ROS_DEBUG("z_thresh has changed: %f.", z_thresh);

  return;
}

void depthCameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info)
{
  // Load the camera model structure with the pinhole model (Intrinsic + distortion coefficients)
  // of the IR camera (depth).
  cam_model_.fromCameraInfo(depth_camera_info);

  ROS_INFO("Depth Camera Model Loaded");

  // Do it once since the camera model is constant
  depth_camera_info_sub.shutdown();

  ROS_INFO("Camera Info subscriber shut down");

  // Use correct principal point from calibration
  center_x = cam_model_.cx();
  center_y = cam_model_.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  unit_scaling = depth_image_proc::DepthTraits<uint16_t>::toMeters(uint16_t(1));
  constant_x = unit_scaling / cam_model_.fx();
  constant_y = unit_scaling / cam_model_.fy();

  camera_info_received = true;

  ROS_INFO("depthCameraInfoCb end.");

  return;
}


void isFalsePositiveCb(const std_msgs::Bool::ConstPtr& is_false_positive_msg)
{

  if(is_false_positive_msg->data)
  {
    ROS_ASSERT_MSG(!is_false_positive, "In 'isFalsePositiveCb'. is_false_positive is already set to true!");
    ROS_WARN("bb_to_world: we are in a FALSE POSITIVE state!");
    is_false_positive = true;
  }
  else
  {
    ROS_ASSERT_MSG(is_false_positive, "In 'isFalsePositiveCb'. is_false_positive is already set to false!");
    ROS_WARN("bb_to_world: the FALSE POSITIVE state has ended!");
    is_false_positive = false;
  }

  return;
}

// 1. la bounding box (bidimensionale) viene portata nel sistema di riferimento della camera
// 2. utilizzando la bounding box si estraggono dalla mappa di profondità tutti i punti in essa
//    contenuti
// 3. ognuno di questi punti bidimensionali viene convertito nel corrispondente punto 3D, usando i
//    valori di profondità ed i parametri della camera
// 4. i punti vengono convertiti nel sistema di riferimento del target frame
// 5. questi punti vengono salvati in una point cloud se non corrispondono al pavimento o alle
//    persone che passano davanti
// 6. viene calcolato il centroide di questa point cloud
void boundingBoxCallback(const sensor_msgs::Image::ConstPtr& sensor_depth_image,
                         const tld_msgs::BoundingBox::ConstPtr& b_box, const std::string target_frame, const tf::TransformListener* transformer,
                         const ros::Publisher* robot_pose_rviz_pub)//, const ros::Publisher* robot_pose_to_localization_pub)
{
  static int seq; // Sequence number of the packages sent from this node.

  // Quando la confidenza è bassa (o nulla) viene segnalato e non viene pubblicato niente.
  if(b_box->confidence < min_confidence)
  {
    if(b_box->confidence)
    {
      ROS_WARN("Confidence is too low! Confidence = %.2f.", b_box->confidence);
    }
    else
    {
      ROS_WARN("LOST TRACKING.");
    }

    ROS_WARN("I'm not publishing anything.\n");

    return;
  }

  // Quando la posizione ricevuta e' marcata come falso positivo, viene segnalato e non viene pubblicato niente.
  if(is_false_positive)
  {
    ROS_WARN("FALSE POSITIVE!");
    ROS_WARN("I'm not publishing anything.\n");
    return;
  }

  try
  {
    // Trasforma le coordinate dell'angolo sinistro della Bounding Box, dal sistema di riferimento
    // della Bounding Box, al sistema di riferimento della Depth Map.
    StampedPoint stamped_point_top_left;
    {
      StampedPoint stamped_point_in;
      stamped_point_in.frame_id_ = b_box->header.frame_id;
      stamped_point_in.stamp_ = b_box->header.stamp;
      stamped_point_in.setData(tf::Point(b_box->x, b_box->y, 0));
      transformer->transformPoint(sensor_depth_image->header.frame_id, stamped_point_in, stamped_point_top_left);
    }

    // `bb_pcl` è la Point Cloud costruita a partire dai punti contenuti all'interno della bounding box.
    //  Da questa Point Cloud verrà poi estratto un centroide che rappresenti la posizione dell'oggetto tracciato.
    pcl::PointCloud<pcl::PointXYZ> bb_pcl;
    bb_pcl.header.frame_id = target_frame;
    bb_pcl.height = 1;      // unorganized PC
    bb_pcl.is_dense = true; // does not contain NaN/Inf

    // Non appena è disponibile la transform tra il sistema di riferimento finale (world) e quello dell'immagine depth,
    // si procede all'operazione di estrazione dei punti.
    //if(!transformer->waitForTransform(target_frame, bb_pcl.header.frame_id, // ehm: 'bb_pcl.header.frame_id = target_frame;'
    //sensor_depth_image->header.stamp, ros::Duration(WAIT_TRANS_TIME)))
    if(!transformer->waitForTransform(target_frame,
                                      sensor_depth_image->header.frame_id, // ehm: 'bb_pcl.header.frame_id = target_frame;'
                                      sensor_depth_image->header.stamp, ros::Duration(WAIT_TRANS_TIME)))
    {
      ROS_ERROR("bb_to_world: wait for transform timed out!!");
      return;
    }

    {
      // cv::Rect corrispondente alla Bounding Box nel sistema di riferimento dell'immagine di profondità (optical frame)
      const cv::Rect roi_rect(stamped_point_top_left.getX(), stamped_point_top_left.getY(), b_box->width, b_box->height);
      const int image_size = sensor_depth_image->data.size();
      const int img_cols = sensor_depth_image->width;
      int pix = -1;
      int print_debug = 1;
      int rows = -1, cols = -1;

      for(int i = 0; i < image_size; i += 2)
      {
        // Per ogni pixel dell'immagine di profondità viene calcolata la corrispondente posizione
        // in termini di righe e colonne.
        ++pix;
        ++cols;

        if(pix % img_cols == 0)
        {
          ++rows;
          cols = 0;
        }

        // L'immagine ottenuta da '/kinect2/xxd/image_depth_rect' ha una codifica little endian su 16 bit.
        //  - little endian: least significant bit all'indirizzo minore
        //  - 'data' è di tipo uint8, per cui bisogna prendere insieme 2 valori consecutivi per
        //    ottenere un valore corretto
        //  - p_depth corrisponde alla profondità del pixel, in mm, rispetto all'optical_frame
        const uint16_t p_depth = sensor_depth_image->data[i] | (sensor_depth_image->data[i + 1] << 8);

        // Il punto di coordinate (cols, rows) viene aggiunto alla Point Cloud se:
        // 1. è all'interno della bounding box
        // 2. il valore di profondità è un valore valido (p_depth != 0)
        if(roi_rect.contains(cv::Point(cols, rows)) && p_depth > 0)
        {
          StampedPoint stamped_point_bounding_box_world;
          {
            // Vengono calcolate le coordinate tridimensionali del punto, nel sistema di riferimento ottico della profondità.
            const tfScalar tf_x = (cols - center_x) * p_depth * constant_x;
            const tfScalar tf_y = (rows - center_y) * p_depth * constant_y;
            // 'DepthTraits<uint16_t>::toMeters' converte da mm a m, semplicemente dividendo per
            // 1000.
            // 'DepthTraits<uint16_t>::fromMeters' invece converte da m a mm, moltiplicando per
            // 1000 e aggiungendo mezzo millimetro.
            const tfScalar tf_z = depth_image_proc::DepthTraits<uint16_t>::toMeters(p_depth);
            const tf::Point tf_point_bounding_box_optical(tf_x, tf_y, tf_z);

            // Dal sistema di riferimento 'camera depth' si passa al sistema di riferimento del 'target_frame'.
            StampedPoint stamped_point_bounding_box_optical;
            stamped_point_bounding_box_optical.frame_id_ = sensor_depth_image->header.frame_id;
            stamped_point_bounding_box_optical.stamp_ = sensor_depth_image->header.stamp;
            stamped_point_bounding_box_optical.setData(tf_point_bounding_box_optical);
            transformer->transformPoint(target_frame, stamped_point_bounding_box_optical, stamped_point_bounding_box_world);
          }

          pcl::PointXYZ pcl_point;
          pcl_point.x = stamped_point_bounding_box_world.getX();
          pcl_point.y = stamped_point_bounding_box_world.getY();
          pcl_point.z = stamped_point_bounding_box_world.getZ();

          // Come condizioni aggiuntive il punto viene aggiunto alla bounding box se
          // 1. non è in una posizione troppo elevata rispetto al suolo
          //  - l'occlusione parziale della bounding box a causa del passaggio
          //    di una persona causa il calcolo errato della posizione del centroide
          // 2. l'altezza supera una certa soglia 'z_thresh'
          //  - in questo modo non andiamo a prendere punti nel pavimento ed il calcolo del centroide
          //    diventa più preciso
          if(pcl_point.z < point_max_height && pcl_point.z > z_thresh)
          {
            bb_pcl.points.push_back(pcl_point);
          }

        }  // end if
      }  // end for loop
    }

    bb_pcl.width = bb_pcl.points.size();
    ROS_DEBUG("bb_pcl points size: %lu", bb_pcl.points.size());
    ROS_DEBUG("bb_pcl width: %d", bb_pcl.width);
    ROS_DEBUG("bb_pcl height: %d", bb_pcl.height);

    // Se il calcolo del centroide fallisce, non viene pubblicato niente.
    Eigen::Vector4d centroid;

    if(pcl::compute3DCentroid(bb_pcl, centroid) == 0)
    {
      ROS_ERROR("centroid not computed! z_thresh = %.2f. bb_pcl.points.size() = %d.", z_thresh, (int) bb_pcl.points.size());

      return;
    }

    // Viene pubblicata la Point Cloud.
    //sensor_msgs::PointCloud2 bb_pcl_msg;
    //pcl::toROSMsg(bb_pcl, bb_pcl_msg);
    //show_me_point_cloud.publish(bb_pcl_msg);

    // Viene visualizzata su rviz la posizione dell'oggetto restituita dal visual tracker.
    {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = target_frame;
      pose_stamped.header.seq = seq;
      pose_stamped.header.stamp = sensor_depth_image->header.stamp;
      pose_stamped.pose.orientation.w = 1;
      pose_stamped.pose.orientation.x = 0;
      pose_stamped.pose.orientation.y = 0;
      pose_stamped.pose.orientation.z = 0;
      pose_stamped.pose.position.x = centroid(0);
      pose_stamped.pose.position.y = centroid(1);
      pose_stamped.pose.position.z = 0;

      robot_pose_rviz_pub->publish(pose_stamped);
    }

    // Preparazione del msg Pose2DStamped per essere inviato ad Unity Bridge
//    {
//      projected_game_msgs::Pose2DStamped pose_2D_stamped_msg;
//      pose_2D_stamped_msg.header.frame_id = target_frame;
//      pose_2D_stamped_msg.header.seq = seq;
//      pose_2D_stamped_msg.header.stamp = sensor_depth_image->header.stamp;
//      pose_2D_stamped_msg.pose.theta = 0.0;
//      pose_2D_stamped_msg.pose.x = centroid(0);
//      pose_2D_stamped_msg.pose.y = centroid(1);

//      robot_pose_pub->publish(pose_2D_stamped_msg);
//    }

    // Preparazione del msg PoseWithCovarianceStamped per essere inviato al filtro di Kalman.
    //geometry_msgs::PoseWithCovarianceStamped geom_pose_2D_stamped_msg;
    //geom_pose_2D_stamped_msg.header.frame_id = target_frame;
    //geom_pose_2D_stamped_msg.header.seq = seq;
    //geom_pose_2D_stamped_msg.header.stamp = sensor_depth_image->header.stamp;
    //geom_pose_2D_stamped_msg.pose.pose.orientation.w = 1;
    //geom_pose_2D_stamped_msg.pose.pose.orientation.x = 0;
    //geom_pose_2D_stamped_msg.pose.pose.orientation.y = 0;
    //geom_pose_2D_stamped_msg.pose.pose.orientation.z = 0;
    //geom_pose_2D_stamped_msg.pose.pose.position.x = centroid(0);
    //geom_pose_2D_stamped_msg.pose.pose.position.y = centroid(1);
    //geom_pose_2D_stamped_msg.pose.pose.position.z = 0;
    //{
    //// XXX: il pacchetto arriva a 'robot_localization'
    //boost::array<double, 36ul> pose_covariance =
    //{
    //1e-2, 0, 0, 0, 0, 0,                                    // small covariance on visual tracking x
    //0, 1e-2, 0, 0, 0, 0,                                    // small covariance on visual tracking y
    //0, 0, 1e-6, 0, 0, 0,                                    // small covariance on visual tracking z
    //0, 0, 0, 1e6, 0, 0,                                     // huge covariance on rot x
    //0, 0, 0, 0, 1e6, 0,                                     // huge covariance on rot y
    //0, 0, 0, 0, 0, 1e6                                      // huge covariance on rot z
    //};
    //geom_pose_2D_stamped_msg.pose.covariance = pose_covariance;
    //}

    //robot_pose_to_localization_pub->publish(geom_pose_2D_stamped_msg);

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
    ros::Duration(1.0).sleep();
  }

  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bb_to_world_node");

  // inizializzazione variabili globali
  camera_info_received = false;
  is_false_positive = false;
  min_confidence = MIN_CONFIDENCE;
  point_max_height = POINT_TOO_HIGH;
  z_thresh = -1.0;

  std::string target_frame;

  // Check arguments
  if(argc != 2)
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
  //  (l'immagine di profondità e la posizione restituita dal tracker visuale)
  message_filters::Subscriber<sensor_msgs::Image> image_sub(node, "camera/image", 1);
  message_filters::Subscriber<tld_msgs::BoundingBox> b_box_sub(node, "tracker/bounding_box", 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, b_box_sub);

  tf::TransformListener transformer;

  //show_me_point_cloud = node.advertise<sensor_msgs::PointCloud2>("bb_point_cloud", 1);
  ros::Publisher robot_pose_rviz_pub = node.advertise<geometry_msgs::PoseStamped>("robot_pose_rviz", 1);

  // Viene pubblicata la posa 2D del robot, ottenuta dal tracking visuale,
  //  convertita nelle cordinate del mondo.
  //  ros::Publisher robot_pose_pub = node.advertise<projected_game_msgs::Pose2DStamped> ("robot_2d_pose", 1);

  // Viene pubblicata la posa 2D del robot, come se fosse un messaggio PoseWithCovarianceStamped,
  //  in modo da poterla inviare al nodo 'robot_localization' per la sensor fusion.
  //ros::Publisher robot_pose_to_localization_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped> ("robot_2d_geometry_pose", 1);

  sync.registerCallback(boost::bind(&boundingBoxCallback, _1, _2, target_frame, &transformer, &robot_pose_rviz_pub));//,
  //&robot_pose_to_localization_pub));

//  sync.registerCallback(
//    boost::bind(
//      &boundingBoxCallback,
//      _1, _2,
//      target_frame,
//      &transformer,
//      &robot_pose_rviz_pub,
//      &robot_pose_pub,
//      &robot_pose_to_localization_pub));

  // Subscriber for receiving the false positive signal.
  is_false_positive_sub = node.subscribe("is_false_positive", 1, isFalsePositiveCb);

  // This callback will be performed once (camera model is costant).
  depth_camera_info_sub = node.subscribe("camera/camera_info", 1, depthCameraInfoCb);

  sleep(2); //sleep 2 seconds. Wait 'openni_launch' to come up.

  // 'ros::Rate' makes a best effort at mantaining a particular rate for a loop.
  ros::Rate rate(rate_hz);

  // Non appena sono arrivate le informazioni sulla camera, attiviamo la boundingBoxCallback.
  while(!camera_info_received)
  {
    ros::spinOnce();
    rate.sleep();
  }

  ros::spin();

  return 0;
}
