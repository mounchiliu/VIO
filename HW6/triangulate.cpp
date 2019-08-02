//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标
};
int main()
{

    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for(int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / ( poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x/z,y/z);
    }
    
    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est;           // 结果保存到这个变量
    P_est.setZero();
    /* your code begin */
    //Initialize
    Eigen::MatrixXd D((end_frame_id-start_frame_id)*2,4);

     for (int i = start_frame_id; i < end_frame_id; ++i) {
         Eigen::Vector4d P_3,P_1,P_2;

         Eigen::Matrix4d Twc = Eigen::Matrix4d::Zero();

         Twc.block(0,0,3,3) = camera_pose[i].Rwc;
         Twc.block(0,3,3,1) = camera_pose[i].twc;
         Twc(3,3) = 1;

         Eigen::Matrix4d Tcw = Twc.inverse();


         //Third row
         P_3<<Tcw.row(2).transpose();

         //First Row for u
         P_1<<Tcw.row(0).transpose();


         //Second Row for v
         P_2<<Tcw.row(1).transpose();

         D.row(2*(i-3)) = camera_pose[i].uv.x()*P_3.transpose() - P_1.transpose();//u
         D.row(2*(i-3)+1) = camera_pose[i].uv.y()*P_3.transpose() - P_2.transpose();//v

     }

     //Do SVD decomposition
     //The recommended one is the BDCSVD class, which scale well for large problems
     //and automatically fall-back to the JacobiSVD class for smaller problems.
     //For both classes, their solve() method is doing least-squares solving.

     using namespace Eigen;
     JacobiSVD<MatrixXd> svd(D.transpose()*D, ComputeThinU | ComputeThinV);

     //check sigma_4 << sigma_3
     double s_3 = svd.singularValues().z();
     double s_4 = svd.singularValues().w();

     //valid
     if(s_3>= s_4*1000)

        //get u4
        P_est = svd.matrixU().col(3).topRows(3)/svd.matrixU().col(3)[3];//Dont forget to normalize!!!


    /* your code end */


    
    std::cout <<"ground truth: \n"<< Pw.transpose() <<std::endl;
    std::cout <<"your result: \n"<< P_est.transpose() <<std::endl;
    return 0;
}
