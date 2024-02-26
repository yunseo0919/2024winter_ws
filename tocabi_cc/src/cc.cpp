#include "cc.h"
ofstream fout;

std::default_random_engine generator;
ros::Publisher new_cup_pos_pub;
geometry_msgs::Point new_cup_pos_msg_;


using namespace TOCABI;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    haptic_pose_sub_ = nh_cc_.subscribe("/haptic/pose", 1, &CustomController::HapticPoseCallback, this);
    cup_pos_sub = nh_cc_.subscribe("/cup_pos", 1, &CustomController::CupPosCallback, this);
    haptic_force_pub_ = nh_cc_.advertise<geometry_msgs::Vector3>("/haptic/force", 10);
    ControlVal_.setZero();
    image_transport::ImageTransport it(nh_cc_);
    camera_flag_pub = nh_cc_.advertise<std_msgs::Bool>("/mujoco_ros_interface/camera/flag", 1);
    camera_image_sub = it.subscribe("/mujoco_ros_interface/camera/image", 1, &CustomController::camera_img_callback, this);
    new_cup_pos_pub = nh_cc_.advertise<geometry_msgs::Point>("/new_cup_pos", 1);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

// void CustomController::taskCommandToCC(TaskCommand tc_)
// {
//     tc = tc_;
// }

void CustomController::PublishHapticData()
{
    geometry_msgs::Vector3 force;
    force.x = haptic_force_[0];
    force.y = haptic_force_[1];
    force.z = haptic_force_[2];

    haptic_force_pub_.publish(force);
}

double getRandomPosition(double minValue, double maxValue) 
        {
            std::uniform_real_distribution<double> distribution(minValue, maxValue);
            return distribution(generator);
        }


std::string folderPath, fileName, filePath;

bool CustomController::saveImage(const sensor_msgs::ImageConstPtr &image_msg) {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(image_msg, image_msg->encoding.c_str())->image;
    }
    catch(cv_bridge::Exception)
    {
      ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), image_msg->encoding.c_str());
      return false;
    }

    if (!image.empty()) {
        std::stringstream fileNameSS;
        fileNameSS << "image-" << ros::Time::now().sec << "-" << ros::Time::now().nsec << + ".jpg";
        fileName = fileNameSS.str();

        std::stringstream filePathSS;
        filePathSS << folderPath << "/" << fileName;
        filePath = filePathSS.str();

        cv::imwrite(filePath, image);
        ROS_INFO("Saved image %s", fileName.c_str());
    }
    else
    {
        ROS_ERROR("Couldn't save image, no data!");
        return false;
    }

    return true;
}

void CustomController::camera_img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        // ROS_INFO("Camera Callback.");
        // save the image
        if (!saveImage(msg)) return;
        // ROS_INFO("Image Saved.");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void CustomController::computeSlow()
{
    //MODE 6,7,8,9 is reserved for cc
    queue_cc_.callAvailable(ros::WallDuration());
    
    

    
    
    if (rd_.tc_.mode == 6)
    {   
        double ang2rad = 0.0174533;

        static bool init_qp;
        
        static VectorQd init_q_mode6;

        static Matrix3d rot_hand_init;
        static Matrix3d rot_haptic_init;

        static Vector3d pos_hand_init;
        static Vector3d pos_haptic_init;


        if (rd_.tc_init) //한번만 실행됩니다(gui)
        {

            init_qp = true;

            std::cout << "mode 6 init!" << std::endl;
            rd_.tc_init = false;
            rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;

            init_q_mode6 = rd_.q_;

            rot_hand_init = rd_.link_[Right_Hand].rotm;
            rot_haptic_init = haptic_orientation_;

            pos_hand_init = rd_.link_[Right_Hand].xpos;
            pos_haptic_init = haptic_pos_;
            camera_tick_ = 0;
            data_collect_start_ = true;
            make_dir = true;

        }
        

        if(data_collect_start_){
            if(make_dir)
            {
                std::stringstream folderPathSS, folderPathSS2;
                // [Check] ros::Time::now().sec is int32 type.
                auto now = std::chrono::system_clock::now();
                std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
                // folderPathSS << "/home/hokyun20/2024winter_ws/src/camera_pubsub/" << ros::Time::now().sec;
                folderPathSS << "/home/yunseo/catkin_ws/src/tocabi_cc/" << std::put_time(std::localtime(&currentTime), "%Y%m%d-%H%M%S");
                folderPathSS2 << folderPathSS.str();

                folderPathSS << "-image";
                folderPathSS2 << "-text";

                folderPath = folderPathSS.str();
                folderPath2 = folderPathSS2.str();

                int result = mkdir(folderPath.c_str(), 0777);
                int result2 = mkdir(folderPath2.c_str(), 0777);
                if(result != 0){
                    ROS_ERROR("Couldn't make folder(dir), Check folderPath");
                }
                if(result2 != 0){
                    ROS_ERROR("Couldn't make folder2(dir), Check folderPath2");
                }
                make_dir = false;
                std::stringstream fileNameSS2;
                fileNameSS2 << "text-" << ros::Time::now().sec << "-" << ros::Time::now().nsec << + ".txt";
                fileName2 = fileNameSS2.str();
                std::stringstream filePathSS2;
                filePathSS2 << folderPath2 << "/" << fileName2;
                filePath2 = filePathSS2.str();
                fout.open(filePath2);
                fout << "camera_tick" << "\t" << "ros_time" << "\t" << "hand_pose_x"<< "\t" << "hand_pose_y"<< "\t" << "hand_pose_z"<< "\t" << "cup_pos_x" << "\t" << "cup_pos_y"<< "\t" << "cup_pos_z"<< "\t" << "distance"<< endl; 
                if(!fout.is_open())
                {
                    ROS_ERROR("Couldn't open text file");
                }
            }
            
            Eigen::Vector3d rhand_pos_;
            rhand_pos_ << rd_.link_[Right_Hand].xpos;
            
            float dx = cup_pos_(0) - rhand_pos_(0);
            float dy = cup_pos_(1) - rhand_pos_(1);
            float dz = cup_pos_(2) - rhand_pos_(2);
            distance_hand2obj = std::sqrt(dx*dx + dy*dy + dz*dz);

            if(camera_tick_%200 == 0)
            {
                ROS_INFO("I heard: [%d]", camera_tick_);
                camera_flag_msg.data = true;
                camera_flag_pub.publish(camera_flag_msg);
                auto now = std::chrono::system_clock::now();
                std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
                Eigen::Vector3d rhand_pos_;
                rhand_pos_ << rd_.link_[Right_Hand].xpos;
            
                {
                    fout << camera_tick_ << "\t" << currentTime << "\t" << rd_.link_[Right_Hand].xpos(0) <<"\t" << rd_.link_[Right_Hand].xpos(1) <<"\t" << rd_.link_[Right_Hand].xpos(2)<<"\t" << cup_pos_(0)<< "\t" << cup_pos_(1)<<"\t" << cup_pos_(2)<<"\t" << distance_hand2obj << endl;
                }
            }
            camera_tick_++;


            WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
            if (rd_.tc_.customTaskGain)
            {
                rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                rd_.link_[Right_Hand].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            }

            rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
            rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;
            rd_.link_[Pelvis].rot_desired = DyrosMath::rotateWithY(rd_.tc_.pelv_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.link_[Pelvis].yaw_init);

            rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init;
            rd_.link_[Right_Hand].x_desired(0) += rd_.tc_.r_x;
            rd_.link_[Right_Hand].x_desired(1) += rd_.tc_.r_y;
            rd_.link_[Right_Hand].x_desired(2) += rd_.tc_.r_z;
            rd_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(rd_.tc_.r_roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.r_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.r_yaw * ang2rad) * DyrosMath::Euler2rot(0, 1.5708, -1.5708).transpose();
            // rd_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(rd_.tc_.r_roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.r_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.r_yaw * ang2rad);

            rd_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithX(rd_.tc_.roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.yaw * ang2rad);

            rd_.link_[Pelvis].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].xi_init, rd_.link_[Pelvis].x_desired);
            rd_.link_[Pelvis].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

            Vector3d hand_pos_desired;
            // hand_pos_desired[0] = haptic_pos_[0] + pos_hand_init[0] - pos_haptic_init[0];
            // hand_pos_desired[1] = haptic_pos_[1] + pos_hand_init[1] - pos_haptic_init[1];
            // hand_pos_desired[2] = haptic_pos_[2] + pos_hand_init[2] - pos_haptic_init[2];
            hand_pos_desired[0] = rd_.link_[Right_Hand].x_desired(0) + haptic_pos_[0] - pos_haptic_init[0];
            hand_pos_desired[1] = rd_.link_[Right_Hand].x_desired(1) + haptic_pos_[1] - pos_haptic_init[1];
            hand_pos_desired[2] = rd_.link_[Right_Hand].x_desired(2) + haptic_pos_[2] - pos_haptic_init[2];

            Matrix3d hand_rot_desired = rot_hand_init * rot_haptic_init.transpose() * haptic_orientation_;

            rd_.link_[Right_Hand].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, hand_pos_desired);
            // rd_.link_[Right_Hand].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
            // rd_.link_[Right_Hand].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, hand_rot_desired, false);

            // std::cout<<"pos"<<std::endl;
            // std::cout<<haptic_pos_<<std::endl;
            // std::cout<<"ori"<<std::endl;
            // std::cout<<haptic_orientation_<< std::endl;


            // std::cout<<"Hand pos "<< rd_.link_[Right_Hand].xpos[0] << std::endl;
            // std::cout<<"Cup pos "<< cup_pos_.transpose() << std::endl;
            
            rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

            rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

            // Eigen::MatrixXd Jtask1 = rd_.link_[Right_Hand].Jac();
            // Eigen::VectorXd fstar1 = WBC::GetFstar6d(rd_.link_[Right_Hand], true);
            // Eigen::MatrixXd Lambda1 = (Jtask1 * rd_.A_inv_ * rd_.N_C * Jtask1.transpose()).inverse();
            // Eigen::VectorQd torque_com_task1;
            // torque_com_task1.setZero(33);
            // torque_com_task1.segment(12,21) = (Jtask1.transpose() * Lambda1 * fstar1).segment(12,21);

            // Eigen::VectorQd torque_pos_hold;
            // torque_pos_hold.setZero(33);

            // for (int i=0;i<MODEL_DOF;i++)
            // {
            //     if(i >= 20 && i < 23){
            //         torque_pos_hold[i] = rd_.pos_kp_v[i] * (init_q_mode6[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * ( - rd_.q_dot_[i]);
            //     }
            // }

            // Eigen::VectorQd torque_upper_mode6;
            // torque_upper_mode6.setZero(33);
            // torque_upper_mode6.segment(12,21) = (rd_.torque_grav).segment(12,21);

            // for (int i=0;i<MODEL_DOF;i++)
            // {
            //     rd_.torque_desired[i] = torque_pos_hold[i] + torque_com_task1[i] + torque_upper_mode6[i];
            // }

            TaskSpace ts_(6);
            Eigen::MatrixXd Jtask = rd_.link_[Pelvis].JacCOM();
            Eigen::VectorXd fstar = WBC::GetFstar6d(rd_.link_[Pelvis], true, true);

            ts_.Update(Jtask, fstar);
            WBC::CalcJKT(rd_, ts_);
            WBC::CalcTaskNull(rd_, ts_);
            static CQuadraticProgram task_qp_;
            WBC::TaskControlHQP(rd_, ts_, task_qp_, rd_.torque_grav, MatrixXd::Identity(MODEL_DOF, MODEL_DOF), init_qp);

            VectorQd torque_Task2 = ts_.torque_h_ + rd_.torque_grav;

            TaskSpace ts1_(6);
            Eigen::MatrixXd Jtask1 = rd_.link_[Right_Hand].Jac();
            Eigen::VectorXd fstar1 = WBC::GetFstar6d(rd_.link_[Right_Hand], true);

            ts1_.Update(Jtask1, fstar1);
            WBC::CalcJKT(rd_, ts1_);
            WBC::CalcTaskNull(rd_, ts1_);
            static CQuadraticProgram task_qp1_;
            WBC::TaskControlHQP(rd_, ts1_, task_qp1_, torque_Task2, ts_.Null_task, init_qp);

            torque_Task2 = ts_.torque_h_ + ts_.Null_task * ts1_.torque_h_ + rd_.torque_grav;

            TaskSpace ts2_(3);
            Eigen::MatrixXd Jtask2 = rd_.link_[Upper_Body].Jac().bottomRows(3);
            Eigen::VectorXd fstar2 = WBC::GetFstarRot(rd_.link_[Upper_Body]);
            ts2_.Update(Jtask2, fstar2);
            WBC::CalcJKT(rd_, ts2_);

            static CQuadraticProgram task_qp2_;
            WBC::TaskControlHQP(rd_, ts2_, task_qp2_, torque_Task2, ts_.Null_task * ts1_.Null_task, init_qp);

            torque_Task2 = ts_.torque_h_ + ts_.Null_task * ts1_.torque_h_ + ts_.Null_task * ts1_.Null_task * ts2_.torque_h_ + rd_.torque_grav;

                        // rd_.torque_desired[i] = rd_.pos_kp_v[i] * (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (zero_m[i] - rd_.q_dot_[i]);
            VectorQd torque_pos_hold;

            for (int i=0;i<MODEL_DOF;i++)
            {
                torque_pos_hold[i] = rd_.pos_kp_v[i] * (init_q_mode6[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * ( - rd_.q_dot_[i]);
            }


            torque_pos_hold.segment(25,8).setZero();

            VectorQd torque_right_arm;
            
            torque_right_arm.setZero();

            torque_right_arm.segment(25,8) = WBC::ContactForceRedistributionTorque(rd_, torque_Task2).segment(25,8);

            for (int i=12;i<MODEL_DOF;i++)
            {
                rd_.torque_desired[i] = torque_pos_hold[i] + torque_right_arm[i];
            }
            // rd_.torque_desired = torque_pos_hold + torque_right_arm;
            
            // std::cout <<"torque" << rd_.RH_CF_FT<< std::endl;

            // haptic_force_[0] = rd_.RH_CF_FT[0];
            // haptic_force_[1] = rd_.RH_CF_FT[1];
            // haptic_force_[2] = rd_.RH_CF_FT[2];

            haptic_force_[0] = rd_.RH_CF_FT[0];
            haptic_force_[1] = rd_.RH_CF_FT[1];


            if(camera_tick_%200 == 0)
            {

                // std::cout << "hand_pos_desired : \n" << hand_pos_desired << std::endl;

                // std::cout <<"distance_hand2obj : " << distance_hand2obj << std::endl;
                
                // std::cout <<"cup_pos_(0) " << cup_pos_(0)<< "rhand_pos_(0) " << rhand_pos_(0) <<std::endl;
                // std::cout <<"cup_pos_(1) " << cup_pos_(1)<< "rhand_pos_(1) " << rhand_pos_(1) <<std::endl;
                // std::cout <<"cup_pos_(2) " << cup_pos_(2)<< "rhand_pos_(2) " << rhand_pos_(2) <<std::endl;

            }

            if (distance_hand2obj < 0.2)
            {
                std::cout<< "Reached" <<std::endl;
                data_collect_start_ == false;
                rd_.tc_.mode = 7;
                rd_.tc_init = true;

            }
        }
    }
    else if (rd_.tc_.mode == 7)
    {
        // std::cout << "tc 7" << std::endl;
        if(fout.is_open()==true)
        {
            fout.close();
        }
        double duration = 3.0;
        if (!target_reached_)
        {
            target_reached_ = true;
            q_init_ = rd_.q_;
            time_init_ = rd_.control_time_;

            const double minX = 0.3;
            const double maxX = 0.6;
            const double minY = -0.7;
            const double maxY = -0.3;

            new_cup_pos_msg_.x = getRandomPosition(minX, maxX);
            new_cup_pos_msg_.y = getRandomPosition(minY, maxY);
            new_cup_pos_msg_.z = 1.0;
            new_cup_pos_pub.publish(new_cup_pos_msg_);
        }
        resetRobotPose(duration);

        if (rd_.control_time_ > time_init_ + duration)
        {
            rd_.tc_.mode = 6;
            target_reached_ = false;
        }
    }
}

void CustomController::resetRobotPose(double duration)
{
    Eigen::Matrix<double, MODEL_DOF, 1> q_target;
    Eigen::Matrix<double, MODEL_DOF, 1> q_cubic;
    q_target << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, 0.0,
            0.3, 0.3, 1.5, -1.27, -1.0, 0.0, -1.0, 0.0,
            0.0, 0.0,
            -0.3, -0.9, -1.5, 1.57, 1.9, 0.0, 0.5, 0.0;
    for (int i = 0; i <MODEL_DOF; i++)
    {
        q_cubic(i) = DyrosMath::cubic(rd_.control_time_, time_init_, time_init_ +duration, q_init_(i), q_target(i), 0.0, 0.0);
    }
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kp;
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kv;

    kp.setZero();
    kv.setZero();
    kp.diagonal() <<   2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        6000.0, 10000.0, 10000.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0,
                        100.0, 100.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0;
    kv.diagonal() << 15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        200.0, 100.0, 100.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0,
                        2.0, 2.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0;


    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
    rd_.torque_desired =  kp*(q_cubic-rd_.q_)-kv* rd_.q_dot_ +  WBC::GravityCompensationTorque(rd_);
}


void CustomController::computeFast()
{
    if (rd_.tc_.mode == 6)
    {
    }
    else if (rd_.tc_.mode == 7)
    {
    }
}

void CustomController::HapticPoseCallback(const geometry_msgs::PoseConstPtr &msg)
{

    float pos_x = CustomController::PositionMapping(msg -> position.x, 0);
    float pos_y = CustomController::PositionMapping(msg -> position.y, 1);
    float pos_z = CustomController::PositionMapping(msg -> position.z, 2);
    float ori_x = CustomController::PositionMapping(msg -> orientation.x, 3);
    float ori_y = CustomController::PositionMapping(msg -> orientation.y, 4);
    float ori_z = CustomController::PositionMapping(msg -> orientation.z, 5);
    float ori_w = CustomController::PositionMapping(msg -> orientation.w, 6);

    // double posx = static_cast<double>(pos_x);
    // double posy = static_cast<double>(pos_y); 
    // double posz = static_cast<double>(pos_z);
    double orix = static_cast<double>(ori_x);
    double oriy = static_cast<double>(ori_y); 
    double oriz = static_cast<double>(ori_z);
    double oriw = static_cast<double>(ori_w);

    haptic_pos_[0] = pos_x;
    haptic_pos_[1] = pos_y; 
    haptic_pos_[2] = pos_z;

    haptic_orientation_ = CustomController::Quat2rotmatrix(orix, oriy, oriz, oriw);

}

void CustomController::CupPosCallback(const geometry_msgs::PointPtr &msg)
{
    float cup_x = msg -> x;
    float cup_y = msg -> y;
    float cup_z = msg -> z;
 
    cup_pos_ << cup_x, cup_y, cup_z;
    // std::cout << "CupPos subscribed!" << std::endl;
}


float CustomController::PositionMapping(float haptic_val, int i)
{
    if (i == 0){
        return -1 * (haptic_val + 0.051448) * 5.0 ;
    }

    else if(i == 1){
        return -1 * (haptic_val + 0.000152) * 5.0;
    }

    else if (i == 2){
        return (haptic_val - 0.007794) * 5.0;
    }
    else {
     return haptic_val;
    }
    
}

Eigen::Matrix3d CustomController::Quat2rotmatrix(double q0, double q1, double q2, double q3)
{
    double r00 = 2 * (q0 * q0 + q1 * q1) - 1 ;
    double r01 = 2 * (q1 * q2 - q0 * q3) ;
    double r02 = 2 * (q1 * q3 + q0 * q2) ;

    double r10 = 2 * (q1 * q2 + q0 * q3) ;
    double r11 = 2 * (q0 * q0 + q2 * q2) - 1 ;
    double r12 = 2 * (q2 * q3 - q0 * q1) ;

    double r20 = 2 * (q1 * q3 - q0 * q2) ;
    double r21 = 2 * (q2 * q3 + q0 * q1) ;
    double r22 = 2 * (q0 * q0 + q3 * q3) - 1 ;

    Eigen::Matrix3d rot_matrix;
    rot_matrix << r00, r01, r02,
                    r10, r11, r12,
                    r20, r21, r22;
    return rot_matrix;
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}
