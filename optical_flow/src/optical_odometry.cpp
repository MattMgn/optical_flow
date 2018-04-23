/*
Name:        optical_odometry
Purpose:     Provide odometry from Lucas Kanade optical flow and Publish nav_msgs/Odometry Message
             Optimal observation with Extended Kalman Filter for Pose estimation

Author:      Matthieu MAGNON
Created:     April 2018

Env : openCV2, ROS kinetic
-------------------------------------------------------------------------------
List of Classes:
 OpticalFlow 

List of methodes:
 imageCallback
 */

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include <ctime>
#define SAVE 1
 
// CALIBRATION CAMERA
#define CALIB_ANG 10*3.14
#define CALIB_LIN 0.05

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

int init = 1;

// PARAMETERS
int DEBUG = 1;
int DEBUG_K = 1;



class OpticalFlow
{

public:

    // CONSTRUCTEUR

    OpticalFlow(){

        subCam = nh.subscribe("camera/raw_image", 500, &OpticalFlow::imageCallback, this);
        subRg = nh.subscribe("fake_ultrasound", 500, &OpticalFlow::rangeCallback, this);
        pub = nh.advertise<nav_msgs::Odometry>("optical_flow/odom",10);
        ros::NodeHandle nh_private("~");

        // record pose estimate for visualization with python script
        fp = fopen("/home/matt/rosbag/camera_pose_estimate.txt","a+"); // rw + creation de fichier
    }

    // METHODES

    void imageCallback(const sensor_msgs::ImageConstPtr & msg){

        static int k = 1;
        
        if (DEBUG) {cout<<"** Image nb : " << k << " **"<<endl;}

        // Recuperation --> img_raw_ptr->image
        cv_bridge::CvImagePtr img_raw_ptr;
        img_raw_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Conversion to GRAY --> img_cur
        cv::Mat img_cur;
        cv::cvtColor(img_raw_ptr->image, img_cur, cv::COLOR_BGR2GRAY);

        // Init : exe only once
        if ( init==1 ) {
            init = 0;
            img_cur.copyTo(img_prev);
        }

        //Affichage
        cv::imshow("Image Precedente", img_prev);
        //cv::imshow("Image Courante", img_cur);
        cv::waitKey(1);
        

        computeLucasKanade(img_prev, img_cur);

        k++;

    }


    void computeLucasKanade(cv::Mat img_prev , cv::Mat img_cur){

        // Corner Detector : detecte des coins
        if (DEBUG) { cout<<"Detect corner"<<endl;}
        vector <cv::Point2f> prev_corner, cur_corner; // Define les vecteurs de coins detectes
        goodFeaturesToTrack(img_prev, prev_corner, 200, 0.01, 30);
        /* goodFeaturesToTrack(InputArray image, OutputArray corners, int maxCorners, 
        double qualityLevel, double minDistance, InputArray mask=noArray(), int blockSize=3, 
        bool useHarrisDetector=false, double k=0.04 ) */
        if (DEBUG) {cout<<"Corner detected : "<<prev_corner.size()<<endl;}

        // Lucas Kanade optical flow compute
        vector <uchar> status; // status 1:flow found | 0:not found for this features
        vector <float> err;  // l'erreur sur chaque features
        if (prev_corner.size() > 10){
            if (DEBUG) {cout<<"Compute LK OF"<<endl;}
            calcOpticalFlowPyrLK(img_prev, img_cur, prev_corner, cur_corner, status, err);
        }
        else { if (DEBUG) {cout<<"No LK OF to compute "<<endl;} }

        // Filter only good matches
        vector <cv::Point2f> prev_corner_good, cur_corner_good;
        for (int s=0 ; s<status.size() ; s++){
            if ( status[s]==1 ){
                prev_corner_good.push_back(prev_corner[s]);
                cur_corner_good.push_back(cur_corner[s]);
            }
        }
        if (DEBUG) {cout<<"Corner kept after filtering : "<<prev_corner.size()<<endl;}

        // Calcul T = Translation + Rotation par la methode des moindres carres
        // si fullAffine=false alors T = [cTheta sTheta tx ; -sTheta cTheta ty]
        try {
            T = estimateRigidTransform(prev_corner_good, cur_corner_good, false);
        }
        catch (const std::exception& e) {
            ROS_INFO("Can't estimate Rigid Transform "); //. Error: " << e);

        }

        if(T.data == NULL) { // si pas de Transfo dispo, reprendre la precedente 
            ROS_WARN("Rigid Transform is nul"); 
            last_T.copyTo(T);
        }

        if (DEBUG) { cout<<"Matrice de Transformation "<<endl; cout<<T<<endl;}

        // Calcul de l'angle pixel psi
        double d_psi = acos(T.at<double>(1,0)/T.at<double>(0,0)) - 3.1416/2.0000;
        if (DEBUG) { cout<<"d_psi  =  "<<d_psi<<endl; }

        // Calcul de la vitesse pixel suivant z
        double v_z = T.at<double>(0,0)/cos(CALIB_ANG*d_psi); 
        if (DEBUG) { cout<<"v_z  =  "<<v_z<<endl; }

        // ---- Publie le topic Odometry ----
        // header
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "base_link";
        odom.child_frame_id = "camera";
        // twist
        odom.twist.twist.linear.x = CALIB_LIN*T.at<double>(0,2);
        odom.twist.twist.linear.y = CALIB_LIN*T.at<double>(1,2);
        odom.twist.twist.linear.z = CALIB_LIN*v_z;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = CALIB_ANG*d_psi*dt
        // pose
        double dt, begin;
        double x_0, y_0, z_0, qx_0, qy_0, qz_0, qw_0;

        if ( init==1 ) { 
        	dt=0.0; 
        	x_0 = 0.0;
        	y_0 = 0.0;
        	z_0 = 0.0;
        	qx_0 = 0.0; // quaternion
        	qy_0 = 0.0;
        	qz_0 = 1.0; // modele : rotation only on z axis
        	qw_0 = 0.0;
        }
        else { dt = double(clock() - begin) / CLOCKS_PER_SEC;} // get time laps between 2 frames
    	begin = clock(); // RaZ timer
        odom.pose.pose.position.x = x_0 + odom.twist.twist.linear.x*dt;
        odom.pose.pose.position.y = y_0 + odom.twist.twist.linear.y*dt;
        odom.pose.pose.position.z = raw_range.range;
        odom.pose.pose.orientation.x = qx_0;
        odom.pose.pose.orientation.y = qy_0;
        odom.pose.pose.orientation.z = qz_0; 
        odom.pose.pose.orientation.w = qw_0 + odom.twist.twist.angular.z;

        pub.publish(odom);

        if (DEBUG) { cout<<"Vx =  "<<odom.twist.twist.linear.x <<endl; }
        if (DEBUG) { cout<<"Vy =  "<<odom.twist.twist.linear.y <<endl; }
        if (DEBUG) { cout<<"Vz =  "<<odom.twist.twist.linear.z <<endl; }



        if (SAVE) { fprintf(fp , " \n %f , %f , %f , %f", odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z, d_psi );}
        
        // Placer la cur dans la prev
        T.copyTo(last_T);
        img_cur.copyTo(img_prev);

    }

    void rangeCallback(const sensor_msgs::Range & msg){
        raw_range = msg;
    }

    // ATTRIBUTS

protected:    

    ros::NodeHandle nh;
    ros::Subscriber subCam;
    ros::Subscriber subRg;
    ros::Publisher pub;
    cv::Mat img_prev;
    cv::Mat T;
    cv::Mat last_T;
    sensor_msgs::Range raw_range;

    FILE * fp;

};



int main(int argc, char *argv[]){  
    
    ROS_INFO("Starting Optical Flow Node");

/* Record Pose */


    /* node initialisation */
    ros::init(argc, argv, "optical_flow_range");

    OpticalFlow flow;

    ros::spin();

    return 0;
    
}