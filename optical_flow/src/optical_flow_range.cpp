/*
Name:        optical_flow
Purpose:     get visual odometrey from camera pointing down and rangefinder values. 
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
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h> 
#define DELTA_T 0.1
#define N 8 // dim du vecteur X
#define M 5 // dim du vecteur Y
#define SAVE 1
 /*
// MATRICE DE COVARIANCE MESURE : R
#define R_Z 0.02 // en metre
#define R_PSI 0.15 // en radian 0.15rad = 10deg
#define R_POSE 0.2 // en metre
#define R_POSE 0.2 // en metre
// MATRICE DE COVARIANCE MODELE : E
#define Q_POSE 0.2 // 0.05 en metre // linearise donc soumis a erreur
#define Q_Z 0.02 // en metre
#define Q_PSI 0.15 // en radian 0.15rad = 10deg
#define Q_VEL_E 0.5 
#define Q_VEL_Z 0.5 
#define Q_VEL_PSI 0.15
*/
// MATRICE DE COVARIANCE MESURE : R
#define R_Z 0.1 // en metre
#define R_VEL_Z 10 // en metre
#define R_VEL_PSI 1 // en radian 0.15rad = 10deg
#define R_POSE 1 // en metre
// MATRICE DE COVARIANCE MODELE : E
#define Q_POSE 1 // 0.05 en metre // linearise donc soumis a erreur
#define Q_Z 1 // en metre
#define Q_PSI 1 // en radian 0.15rad = 10deg
#define Q_VEL_B 1 
#define Q_VEL_Z 1 
#define Q_VEL_PSI 1
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

    OpticalFlow() : F(N,N), C(M,N), P_k_k(N,N), P_kp_k(N,N), R(M,M), Q(N,N), K(N,M), x_kp_k(N,1), x_k_k(N,1), Y(M,1), I(N,N) {

        subCam = nh.subscribe("camera/raw_image", 500, &OpticalFlow::imageCallback, this);
        subRg = nh.subscribe("fake_ultrasound", 500, &OpticalFlow::rangeCallback, this);
        pub = nh.advertise<geometry_msgs::Twist>("optical_flow/visual_velocity",10);
        ros::NodeHandle nh_private("~");

        // record pose estimate for visualization with python script
        fp = fopen("/home/matt/rosbag/camera_pose_estimate.txt","a+"); // rw + creation de fichier


        /* ********* KALMAN FILTER INITIALISATION ********* */
        // Vecteur d'état : X = [x_E y_E z psi vz w_psi vx_B vy_B]
        // Etape correction
        x_k_k << 0, 0, 1, 0, 0, 0, 0, 0;
        cout<<"\n x_0_0 : "<<endl;
        cout<<x_k_k<<endl;

        // Etape prediction
        x_k_k << 0, 0, 1, 0, 0, 0, 0, 0;
        double dt = 0.033; //ms -> A MODIFIER en calculant ce dt pour chaque image
        F = linearize_state(x_k_k , dt);
        cout<<"\n F : "<<endl;
        cout<<F<<endl;

        // Vecteur de mesure : Y = [z vz w_psi vx_B vy_B]
        //Eigen::MatrixXd C(2,4);
        C << 0, 0, 1, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 0, 0, 1;
        cout<<"\n C : "<<endl;
        cout<<C<<endl;

        // matrice de covariance etape de correction
        Eigen::Matrix<double, N, N>::Identity();
        P_k_k.setIdentity(N,N);
        P_k_k *= 10.0;
        cout<<"\n P_0_0 : "<<endl;
        cout<<P_k_k<<endl;

        // matrice de covariance etape de prediction
        Eigen::Matrix<double, N, N>::Identity();
        P_kp_k.setIdentity(N,N);
        P_kp_k *= 10.0;

        // matrice de variance du bruit de mesure // Grand si Pas confiance en mesure
        Eigen::Matrix<double, M, M>::Identity();
        R.setIdentity(M,M);
        R(0,0) = R_Z;
        R(1,1) = R_VEL_Z;
        R(2,2) = R_VEL_PSI;
        R(3,3) = R_POSE;
        R(4,4) = R_POSE;
        cout<<"\n R : "<<endl;
        cout<<R<<endl;

        //matrice de bruit de processus // Grand si Pas confiance en modele
        Eigen::Matrix<double, N, N>::Identity();
        Q.setIdentity(N,N);
        Q(0,0) = Q_POSE; 
        Q(1,1) = Q_POSE;
        Q(2,2) = Q_Z;
        Q(3,3) = Q_PSI;
        Q(4,4) = Q_VEL_Z;
        Q(5,5) = Q_VEL_PSI; 
        Q(6,6) = Q_VEL_B;
        Q(7,7) = Q_VEL_B;
        //Q(8,8) = Q_VEL_E;
        //Q(9,9) = Q_VEL_E;
        cout<<"\n Q : "<<endl;
        cout<<Q<<endl;
        
        //matrice identité
        Eigen::Matrix<double, N, N>::Identity();
        I.setIdentity(N,N);
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
        double d_psi = acos(T.at<double>(1,0)/T.at<double>(0,0)) - 3.1416/2.0000;
        if (DEBUG) { cout<<"d_psi  =  "<<d_psi<<endl; }

        // Calcul de la vitesse suivant z
        double v_z = CALIB_LIN*T.at<double>(0,0)/cos(CALIB_ANG*d_psi); 
        if (DEBUG) { cout<<"v_z  =  "<<v_z<<endl; }

        // Publie la vitesse de defilement
        geometry_msgs::Twist twist;
        if (DEBUG) { cout<<"Vx =  "<<T.at<double>(0,2)<<endl; }
        if (DEBUG) { cout<<"Vy =  "<<T.at<double>(1,2)<<endl; }

        twist.linear.x = T.at<double>(0,2);
        twist.linear.y = T.at<double>(1,2);
        twist.linear.z = v_z;
        pub.publish(twist);


        // Utilisation de Range
        cout<<"raw_range : "<<raw_range.range <<endl;
        //cout<<"F : "<<F<<endl;


        // Mets a jour l'estimateur EKF avec la mesure : Y = [z vz w_psi vx_B vy_B]
        Y(0,0) = raw_range.range;
        Y(1,0) = v_z;
        Y(2,0) = CALIB_ANG*d_psi;
        Y(3,0) = CALIB_LIN*T.at<double>(0,2);
        Y(4,0) = CALIB_LIN*T.at<double>(1,2);        

        double dt = 0.033; //ms -> A MODIFIER en calculant ce dt pour chaque image
        update_kalman(dt);

        if (DEBUG_K) { cout<<"\n X_k_k =  \n"<<x_k_k.transpose()<<endl; }
        if (DEBUG_K) { cout<<"\n C*X_k_k =  \n"<<(C*x_k_k).transpose()<<endl; }
        if (DEBUG_K) { cout<<"\n Y =  \n"<<Y.transpose()<<endl; }
        if (DEBUG_K) { cout<<"\n P_k_k =  \n"<<P_k_k<<endl; }
        if (DEBUG_K) { cout<<"\n F =  \n"<<F<<endl; }
        cv::waitKey(0);

        if (SAVE) { fprintf(fp , " \n %f , %f , %f , %f", twist.linear.x, twist.linear.y, v_z, d_psi );}
        
        // Placer la cur dans la prev
        T.copyTo(last_T);
        img_cur.copyTo(img_prev);


    }

    void update_kalman(double dt)  {

        /* ----- PREDICTION avec le modele non lineaire ----- */

        // Calcul de x_kp_k
        x_kp_k = F*x_k_k; // pr plus de facilite on utilise la partie lineaire naturelle
        // onupdate juste les elements non lineaires : 
        x_kp_k(0) = x_k_k(0) + ( cos(x_k_k(3))*x_k_k(6) - sin(x_k_k(3))*x_k_k(7) )*dt; //x_kp = x_k + ( cos(psi)*vx_B) + sin(psi)*vy_B )*dt 
        x_kp_k(1) = x_k_k(1) + ( sin(x_k_k(3))*x_k_k(6) + cos(x_k_k(3))*x_k_k(7) )*dt; // y_kp = y_k + ( -sin(psi)*vx_B) + cos(psi)*vy_B )*dt

        // Calcul de P_kp_k 
        //F = linearize_state(x_k_k, dt);
        P_kp_k = F*P_k_k*F.transpose() + Q; 


        /* ----- CORRECTION avec les mesures ----- */

        // Calcul du gain d'innovation K
        K = P_kp_k*C.transpose() * ( C*P_kp_k*C.transpose() + R ).inverse();

        // Correction sachant la mesure
        x_k_k = x_kp_k + K * (Y - C*x_kp_k);

        // MaJ de la variance
        P_k_k = (I - K*C)*P_kp_k;

    }



    Eigen::MatrixXd linearize_state(Eigen::MatrixXd x_k_k , double dt) {
        // return F matrix linearized around x_k_k
        // x_k_k = [x_E y_E z psi vz w_psi vx_B vy_B]
        Eigen::Matrix<double, N, N>::Identity();
        F.setIdentity(N,N);
        // x_kp = x_k + ( cos(psi)*vx_B) + sin(psi)*vy_B )*dt --> Jacobienne
        F(0,3) = -sin(x_k_k(3))*x_k_k(4) + cos(x_k_k(3))*x_k_k(5)*dt; 
        F(0,6) =  cos(x_k_k(3))*dt;
        F(0,7) =  sin(x_k_k(3))*dt;
        // y_kp = y_k + ( -sin(psi)*vx_B) + cos(psi)*vy_B )*dt --> Jacobienne
        F(1,3) = -sin(x_k_k(3))*x_k_k(4) + cos(x_k_k(3))*x_k_k(5)*dt;
        F(1,6) = -sin(x_k_k(3))*dt;
        F(1,7) =  cos(x_k_k(3))*dt;
        // z_kp = z_k + vz*dt --> Jacobienne
        F(2,4) = dt;
        // psi_kp = psi_k + w_psi*dt --> Jacobienne
        F(3,5) = dt;
        return F;
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
    Eigen::MatrixXd I;

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