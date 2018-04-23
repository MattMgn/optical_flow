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
#include "sensor_msgs/Range.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h> 
#define DELTA_T 0.1
#define N 4 // dim du vecteur X
#define M 2 // dim du vecteur Y
#define SAVE 1

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

int init = 1;

// PARAMETERS
int DEBUG = 1;




class OpticalFlow
{

public:

    // CONSTRUCTEUR
    /*
    Eigen::MatrixXd F;
    Eigen::MatrixXd C;
    Eigen::MatrixXd P_k_k;
    Eigen::MatrixXd P_kp_k;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd K;
    Eigen::MatrixXd x_kp_k;        
    Eigen::MatrixXd x_k_k;
    Eigen::MatrixXd Y;
    */


    OpticalFlow() : F(N,N), C(M,N), P_k_k(N,N), P_kp_k(N,N), R(M,M), Q(N,N), K(N,M), x_kp_k(N,1), x_k_k(N,1), Y(M,1){

        subCam = nh.subscribe("camera/raw_image", 10, &OpticalFlow::imageCallback, this);
        subRg = nh.subscribe("fake_ultrasound", 10, &OpticalFlow::rangeCallback, this);
        pub = nh.advertise<geometry_msgs::Twist>("optical_flow/visual_velocity",10);
        ros::NodeHandle nh_private("~");

        fp = fopen("/home/matt/rosbag/camera_pose_estimate.txt","a+"); // rw + creation de fichier


        /* ********* KALMAN FILTER INITIALISATION ********* */
        // Vecteur d'état : X = [x y z theta vx vy vz vtheta]
        // matrices d'état
        //Eigen::MatrixXd F(4,4);
        F << 1,0,DELTA_T,0,
             0,1,0,DELTA_T,
             0,0,1,0,
             0,0,0,1;
        cout<<"\n F : "<<endl;
        cout<<F<<endl;

        // Vecteur de mesure : Y = [z theta vx vy]
        //Eigen::MatrixXd C(2,4);
        C << 1,0,0,0,
             0,1,0,0;
        cout<<"\n C : "<<endl;
        cout<<C<<endl;

        // matrice de covariance etape de corresction
        //Eigen::MatrixXd P_k_k(4,4);
        P_k_k << 100,0,  0,  0,
                   0,  100,0,  0,
                   0,  0,  100,0,
                   0,  0,  0,  100;
        cout<<"\n P_0_0 : "<<endl;
        cout<<P_k_k<<endl;

        // matrice de covariance etape de prediction
        //Eigen::MatrixXd P_kp_k(4,4);

        // matrice de variance du bruit de mesure // Grand si Pas confiance
        //Eigen::MatrixXd R(2,2);
        R << 5, 0,
              0, 5;
        cout<<"\n R : "<<endl;
        cout<<R<<endl;

        //matrice de bruit de processus
        //Eigen::MatrixXd Q(4,4);
        Q << 5,  0,  0,  0,
             0,  5,  0,  0,
             0,   0, 5,  0,
             0,   0,  0, 5;
        cout<<"\n Q : "<<endl;
        cout<<Q<<endl;

        // Coefficient de correction K
        //Eigen::MatrixXd K(4,2);

        //vecteur d'état etape de prediction
        //igen::MatrixXd x_kp_k(4,1);

        //vecteur d'état etape de correction [x,y,vx,vy]'
        //Eigen::MatrixXd x_k_k(4,1);
        x_k_k << 0, 250, 30, 30;
        cout<<"\n x_0_0 : "<<endl;
        cout<<x_k_k<<endl;

        //Mesure
        Y << 0.0, 0.0;

        cout<<"\n Y : "<<endl;
        //Y.resize(3,1);
        cout<<Y<<endl;       
        

        //matrice identité
        Eigen::MatrixXd I(4,4);
        I << 1,  0, 0,  0,
             0,  1, 0,  0,
             0,  0, 1,  0,
             0,  0, 0,  1;
        cout<<"\n I : "<<endl;
        cout<<I<<endl;

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
        
        // Calcul de l'angle theta
        double theta = acos(T.at<double>(1,0)/T.at<double>(0,0)) - 3.1416/2.0000;
        if (DEBUG) { cout<<"Theta  =  "<<theta<<endl; }


        // Publie la vitesse de defilement
        geometry_msgs::Twist twist;
        if (DEBUG) { cout<<"Vx =  "<<T.at<double>(0,2)<<endl; }
        if (DEBUG) { cout<<"Vy =  "<<T.at<double>(1,2)<<endl; }

        twist.linear.x = T.at<double>(0,2);
        twist.linear.y = T.at<double>(1,2);
        twist.linear.z = 0.0;
        pub.publish(twist);


        // Utilisation de Range
        cout<<"raw_range : "<<raw_range.range <<endl;
        //cout<<"F : "<<F<<endl;


        // Mets a jour l'estimateur
        Y(0,0) = raw_range.range;
        Y(1,0) = theta;
        //Y(2,0) = T.at<double>(0,2);
        //Y(3,0) = T.at<double>(1,2);        

        update_kalman(Y);

        if (SAVE) { fprintf(fp , " \n %f , %f , %f , %f", twist.linear.x, twist.linear.y, raw_range.range, theta );}
        


        // Placer la cur dans la prev
        T.copyTo(last_T);
        img_cur.copyTo(img_prev);


    }

    void update_kalman( Eigen::MatrixXd Y ) {
        if (DEBUG) { cout<<"# Kalman | Y \n "<<Y<<endl; }


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

    Eigen::MatrixXd F;
    Eigen::MatrixXd C;
    Eigen::MatrixXd P_k_k;
    Eigen::MatrixXd P_kp_k;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd K;
    Eigen::MatrixXd x_kp_k;        
    Eigen::MatrixXd x_k_k;
    Eigen::MatrixXd Y;

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