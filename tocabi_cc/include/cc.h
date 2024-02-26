#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <random>
#include <fstream>
#include <ctime>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <sys/stat.h>
#include <chrono>
// #include <boost/shared_ptr.hpp>

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    //void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);
    void PublishHapticData();

    void HapticPoseCallback(const geometry_msgs::PoseConstPtr &msg);
    void CupPosCallback(const geometry_msgs::PointPtr &msg);
    Eigen::Matrix3d Quat2rotmatrix(double q0, double q1, double q2, double q3);
    float PositionMapping( float haptic_pos, int i);
    bool saveImage(const sensor_msgs::ImageConstPtr &image_msg);
    void camera_img_callback(const sensor_msgs::ImageConstPtr &msg);
    // sensor_msgs::ImageConstPtr

    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;
    ros::Subscriber haptic_pose_sub_;
    ros::Publisher haptic_force_pub_;
    ros::Subscriber cup_pos_sub;


    
    Eigen::Vector3d haptic_pos_;
    Eigen::Vector4d haptic_ori_;
    Eigen::Matrix3d haptic_orientation_;
    Eigen::Vector3d cup_pos_;

    void resetRobotPose(double duration);
    bool target_reached_ = false;
    Eigen::Matrix<double, MODEL_DOF, 1> q_init_;
    double time_init_ = 0.0;
    
    std::string folderPath, fileName, filePath;
    std::string folderPath2, fileName2, filePath2;

    // float pos_x_;

    //WholebodyController &wbc_;
    //TaskCommand tc;

    double haptic_force_[3];

    ros::Publisher camera_flag_pub;
    std_msgs::Bool camera_flag_msg;

    image_transport::Subscriber camera_image_sub;

    int camera_tick_ = 0;
    bool data_collect_start_ = true;
    bool make_dir = true;

    float distance_hand2obj;
    
private:
    Eigen::VectorQd ControlVal_;
    

};


