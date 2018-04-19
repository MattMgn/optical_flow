/*
Name:        optical_flow
Purpose:     get visual odometrey from camera pointing down and rangefinder values
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
#include "geometry_msgs/Twist.h"

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

int init = 1;

// PARAMETERS
int DEBUG = 0;

class OpticalFlow
{

public:

    // CONSTRUCTEUR

    OpticalFlow(){

        sub = nh.subscribe("camera/raw_image", 10, &OpticalFlow::imageCallback, this);
        pub = nh.advertise<geometry_msgs::Twist>("optical_flow/velocity",10);
        ros::NodeHandle nh_private("~");

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

        /*Affichage
        cv::imshow("Image Precedente", img_prev);
        cv::imshow("Image Courante", img_cur);
        cv::waitKey(1);
        */

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
        

        // Publie la vitesse de defilement
        geometry_msgs::Twist twist;
        if (DEBUG) { cout<<"Vx =  "<<T.at<double>(0,2)<<endl; }
        if (DEBUG) { cout<<"Vy =  "<<T.at<double>(1,2)<<endl; }


        twist.linear.x = T.at<double>(0,2);
        twist.linear.y = T.at<double>(1,2);
        twist.linear.z = 0.0;
        pub.publish(twist);


        // Placer la cur dans la prev
        T.copyTo(last_T);
        img_cur.copyTo(img_prev);
        k++;

    }

    // ATTRIBUTS

protected:    

    ros::NodeHandle nh;
    
    ros::Subscriber sub;
    ros::Publisher pub;
    cv::Mat img_prev;
    cv::Mat T;
    cv::Mat last_T;


};



int main(int argc, char *argv[]){  
    
    ROS_INFO("Starting Optical Flow Node");

    /* node initialisation */
    ros::init(argc, argv, "optical_flow_range");

    OpticalFlow flow;

    ros::spin();

    return 0;
    
}