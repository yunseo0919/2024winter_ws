/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

#include "mjros.h"
#include "mujoco_rgbd_camera.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>

// MuJoCo basic data structures
mjModel* model_ = NULL;
mjData* data_ = NULL;

mjvCamera camera;
mjvScene scene;
mjvOption option;

image_transport::Publisher camera_image_pub;
bool camera_pub_flag_=false;
cv::Mat pub_img;
sensor_msgs::ImagePtr img_msg;

ros::Publisher cup_pos_pub;
geometry_msgs::Point cup_pos_msg_;
int body_id;
int body_id1;

int geomIndex;

// drop file callback
void drop(GLFWwindow *window, int count, const char **paths)
{
    // make sure list is non-empty
    if (count > 0)
    {
        mju_strncpy(filename, paths[0], 1000);
        settings.loadrequest = 1;
        ROS_INFO("DROP REQUEST");
    }
}

// load mjb or xml model
void loadmodel(void)
{
    // clear request
    settings.loadrequest = 0;

    // make sure filename is not empty
    if (!filename[0])
        return;

    // load and compile
    char error[500] = "";
    mjModel *mnew = 0;
    if (strlen(filename) > 4 && !strcmp(filename + strlen(filename) - 4, ".mjb"))
    {
        mnew = mj_loadModel(filename, NULL);
        if (!mnew)
            strcpy(error, "could not load binary model");
    }
    else
    {
        mnew = mj_loadXML(filename, NULL, error, 500);
    }
    if (!mnew)
    {
        printf("%s\n", error);
        return;
    }

    // compiler warning: print and pause
    if (error[0])
    {
        // mj_forward() below will print the warning message
        printf("Model compiled, but simulation warning (paused):\n  %s\n\n",
               error);
        settings.run = 0;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);

    int i = settings.key;
    d->time = m->key_time[i];
    mju_copy(d->qpos, m->key_qpos + i * m->nq, m->nq);
    mju_copy(d->qvel, m->key_qvel + i * m->nv, m->nv);
    mju_copy(d->act, m->key_act + i * m->na, m->na);

    if (m->actuator_biastype[0])
    {
        mju_copy(d->ctrl, m->key_qpos + 7 + i * m->nq, m->nu);
    }
    mj_forward(m, d);

    ros_sim_started = true;
    ctrl_command = mj_stackAlloc(d, (int)m->nu);
    ctrl_command2 = mj_stackAlloc(d, (int)(m->nbody * 6));

    // re-create scene and context
    mjv_makeScene(m, &scn, maxgeom);
    mjr_makeContext(m, &con, 50 * (settings.font + 1));

    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    pert.skinselect = -1;

    // align and scale view, update scene
    alignscale();
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to model name
    if (window && m->names)
    {
        char title[200] = "Simulate : ";
        strcat(title, m->names);
        strcat(title, ros::this_node::getNamespace().c_str());
        glfwSetWindowTitle(window, title);
    }

    // rebuild UI sections
    makesections();

    // full ui update
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);

    updatesettings();
    mujoco_ros_connector_init();
    std::cout << " MODEL LOADED " << std::endl;
}

// void RGBD_sensor(mjModel* model, mjData* data, string* camera_name, string* pub_topic_name, string* sub_topic_name)
void RGBD_sensor(mjModel* model, mjData* data)
{
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(1200, 800, "Camera", NULL, NULL);
  // glfwSetWindowAttrib(window, GLFW_RESIZABLE, GLFW_FALSE);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // setup camera
  mjvCamera rgbd_camera;
  rgbd_camera.type = mjCAMERA_FIXED;
  rgbd_camera.fixedcamid = mj_name2id(model, mjOBJ_CAMERA, "camera");

//   std::cout << "debugging111" << std::endl;
  
  mjvOption sensor_option;
  mjvPerturb sensor_perturb;
  mjvScene sensor_scene;
  mjrContext sensor_context;

  mjv_defaultOption(&sensor_option);
  mjv_defaultScene(&sensor_scene);
  mjr_defaultContext(&sensor_context);

  // create scene and context
  mjv_makeScene(model, &sensor_scene, 1000);
  mjr_makeContext(model, &sensor_context, mjFONTSCALE_150);

  RGBD_mujoco mj_RGBD;

  while (!glfwWindowShouldClose(window))
  {
    // get framebuffer viewport
    mjrRect viewport = {0,0,0,0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    
    mj_RGBD.set_camera_intrinsics(model, rgbd_camera, viewport);

    // update scene and render
    mjv_updateScene(model, data, &sensor_option, NULL, &rgbd_camera, mjCAT_ALL, &sensor_scene);
    mjr_render(viewport, &sensor_scene, &sensor_context);

    mj_RGBD.get_RGBD_buffer(model, viewport, &sensor_context);

    mtx.lock();


    if(camera_pub_flag_){
        pub_img = mj_RGBD.get_color_image();
        if(pub_img.empty())
        {
            ROS_ERROR("Could not read the image.");
            // return -1;
        }
        else
        {

            ////CAMERA img publish
            cv::Mat resized_img;
            // cv::Size desired_size(64, 64); // 원하는 크기 지정하세요 FOV는 .xml에서..
            cv::Size desired_size(256, 256); // 원하는 크기 지정하세요
            cv::resize(pub_img, resized_img, desired_size);

            img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_img).toImageMsg();
            camera_image_pub.publish(img_msg);
            camera_pub_flag_=false;


            ////CUP POSE
            body_id = -1;
            body_id = mj_name2id(model, mjOBJ_BODY, "cup");


            if (body_id >= 0)
            {
                geomIndex = model->body_geomadr[body_id];

                cup_pos_msg_.x = data->geom_xpos[3*geomIndex];
                cup_pos_msg_.y = data->geom_xpos[3*geomIndex + 1];
                cup_pos_msg_.z = data->geom_xpos[3*geomIndex + 2];

            }
            else
            {
                ROS_WARN("NO CUP POS");
            }
            cup_pos_pub.publish(cup_pos_msg_);

        }
    }

    mtx.unlock();


    
    // if(camera_pub_flag_2){
    //     pub_img = mj_RGBD.get_color_image();
    //     if(pub_img.empty())
    //     {
    //         ROS_ERROR("Could not read the image.");
    //         // return -1;
    //     }
    //     else
    //     {
    //         cv::Mat resized_img;
    //         // cv::Size desired_size(64, 64); // 원하는 크기 지정하세요 FOV는 .xml에서..
    //         cv::Size desired_size(128, 128); // 원하는 크기 지정하세요
    //         cv::resize(pub_img, resized_img, desired_size);

    //         img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_img).toImageMsg();
    //         camera_image_pub2.publish(img_msg2);
    //         camera_pub_flag_2=false;
    //     }
    // }


    // mtx.lock();
    // *color_cloud = mj_RGBD.generate_color_pointcloud();
    // mtx.unlock();

    // Swap OpenGL buffers
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Do not forget to release buffer to avoid memory leak
    mj_RGBD.release_buffer();
  }

  mjv_freeScene(&sensor_scene);
  mjr_freeContext(&sensor_context);
}

void camera_flag_callback(const std_msgs::BoolConstPtr &msg){
    if(msg->data){
        camera_pub_flag_ = true;
    }
}

// run event loop
int main(int argc, char **argv)
{
    // :: ROS CUSTUM :: initialize ros
    ros::init(argc, argv, "mujoco_ros");
    ros::NodeHandle nh("~");
    std::string key_file;
    nh.param<std::string>("license", key_file, "mjkey.txt");

    nh.param("use_shm", use_shm, false);
    sim_command_sub = nh.subscribe<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100, sim_command_callback);
    sim_command_pub = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_sim2con", 1);

    ros::Subscriber camera_flag_sub = nh.subscribe<std_msgs::Bool>("/mujoco_ros_interface/camera/flag", 1, camera_flag_callback);
    image_transport::ImageTransport it(nh);
    camera_image_pub = it.advertise("/mujoco_ros_interface/camera/image", 1);


    cup_pos_pub = nh.advertise<geometry_msgs::Point>("/cup_pos", 1);
    new_cup_pos_sub = nh.subscribe<geometry_msgs::Point>("/new_cup_pos", 1, NewCupPosCallback);
    
    
    
    if (!use_shm)
    {        
        nh.param("pub_mode", pub_total_mode, false);
        std::cout<<"Name Space: " << ros::this_node::getNamespace() << std::endl;

        //register publisher & subscriber
        char prefix[200] = "/mujoco_ros_interface";
        char joint_set_name[200];
        char sim_status_name[200];
        char joint_state_name[200];
        char sim_time_name[200];
        char sensor_state_name[200];

        strcpy(joint_set_name, prefix);
        strcpy(sim_status_name, prefix);
        strcpy(joint_state_name, prefix);
        strcpy(sim_time_name, prefix);
        strcpy(sensor_state_name, prefix);
        if (ros::this_node::getNamespace() != "/")
        {
            strcat(joint_set_name, ros::this_node::getNamespace().c_str());
            strcat(sim_status_name, ros::this_node::getNamespace().c_str());
            strcat(joint_state_name, ros::this_node::getNamespace().c_str());
            strcat(sim_time_name, ros::this_node::getNamespace().c_str());
            strcat(sensor_state_name, ros::this_node::getNamespace().c_str());
        }
        strcat(joint_set_name, "/joint_set");
        strcat(sim_status_name, "/sim_status");
        strcat(joint_state_name, "/joint_states");
        strcat(sim_time_name, "/sim_time");
        strcat(sensor_state_name, "/sensor_states");

        joint_set = nh.subscribe<mujoco_ros_msgs::JointSet>(joint_set_name, 1, jointset_callback, ros::TransportHints().tcpNoDelay(true));

        if (pub_total_mode)
        {
            sim_status_pub = nh.advertise<mujoco_ros_msgs::SimStatus>(sim_status_name, 1);
        }
        else
        {
            joint_state_pub = nh.advertise<sensor_msgs::JointState>(joint_state_name, 1);
            sim_time_pub = nh.advertise<std_msgs::Float32>(sim_time_name, 1);
            sensor_state_pub = nh.advertise<mujoco_ros_msgs::SensorState>(sensor_state_name, 1);
        }
    }
    else
    {
#ifdef COMPILE_SHAREDMEMORY
        init_shm(shm_msg_key, shm_msg_id, &mj_shm_);
#endif
    }

    //ROS_INFO("ROS initialize complete");
    sim_time_ros = ros::Duration(0);
    sim_time_run = ros::Time::now();
    sim_time_now_ros = ros::Duration(0);

    // initialize everything
    init();

    std::string model_file;
    // request loadmodel if file given (otherwise drag-and-drop)
    if (nh.getParam("model_file", model_file))
    {
        mju_strncpy(filename, model_file.c_str(), 1000);
        settings.loadrequest = 2;
        ROS_INFO("model is at %s", model_file.c_str());
    }

    // start simulation thread
    ROS_INFO("debugging--Hi");
    // // start simulation thread
    std::thread simthread(simulate);
    std::thread visual_thread;
    // std::thread visual_thread2;

    // event loop
    while ((!glfwWindowShouldClose(window) && !settings.exitrequest) && ros::ok())
    {
        // start exclusive access (block simulation thread)
        mtx.lock();
        // load model (not on first pass, to show "loading" label)
        if (settings.loadrequest == 1)
        {
            ROS_INFO("Load Request");
            loadmodel();
            visual_thread = std::thread(RGBD_sensor, m, d);
            //TODO
            // visual_thread2 = std::thread(RGBD_sensor, m, d, "camera2");
        }
        else if (settings.loadrequest > 1)
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // ros events
        rosPollEvents();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);
    }

    // stop simulation thread
    settings.exitrequest = 1;
    simthread.join();
    visual_thread.join();
    // visual_thread2.join();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // deactive MuJoCo
    // mj_deactivate();

    std_msgs::String pmsg;
    pmsg.data = std::string("terminate");
    sim_command_pub.publish(pmsg);

#ifdef COMPILE_SHAREDMEMORY
        deleteSharedMemory(shm_msg_id, mj_shm_);
#endif
// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 0;
}
