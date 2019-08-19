#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
using namespace Eigen;

// path to trajectory file
string file = "/home/meng7/HW7/dataset/pose_output.txt";//_noise
string file2 = "/home/meng7/HW7/dataset/pose_output_nonoise1.txt";
string file3 = "/home/meng7/HW7/dataset/imu_pose.txt";


// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
//Calculate R
Matrix3d CalculateR(Vector3d p_g, Vector3d p_e,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void ShowRMSE(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main() {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g, poses_e;
    Vector3d point_e(0,0,0), point_g(0,0,0);//质心


    /// implement pose reading code
    // start your code here (5~10 lines)

    vector<double> ts; //Timesteps

    //Open 1st file and read data
    ifstream File;
    File.open(file); //file with_noise //file2 no_noise

    while(!File.eof())
    {
      //read data
      double te;
      //vector t
      Vector3d te_v;
      double e_d[4];

      File>>te>>te_v(0)>>te_v(1)>>te_v(2)>>e_d[1]>>e_d[2]>>e_d[3]>>e_d[0];
      ts.push_back(te);

      //Quaternion //For Eigen, q = s + ai + bj + ck = [s,a,b,c]
      Quaterniond  qe(e_d[0],e_d[1],e_d[2],e_d[3]);

      //t看作(X,Y,Z)
      //计算两组点的质心
      point_e += te_v;

      //SE3
      Sophus::SE3 pose_e(qe,te_v);
      poses_e.push_back(pose_e);
    }
    File.close();


    //Open 2nd file and read data
    ifstream File2;
    File2.open(file3);//imu ground truth
    int count = 0;
    while(!File2.eof())
    {
      //read data
      double tg;
      double temp;
      //vector t
      Vector3d tg_v;
      double g_d[4];

      //Get data
      File2>>tg>>g_d[0]>>g_d[1]>>g_d[2]>>g_d[3]>>tg_v[0]>>tg_v[1]>>tg_v[2]
              >>temp>>temp>>temp>>temp>>temp>>temp;

      //alignment
      if(abs(tg-ts[count])<0.005){

          count++;

          //Quaternion //For Eigen, q = s + ai + bj + ck = [s,a,b,c]
          Quaterniond  qg(g_d[0],g_d[1],g_d[2],g_d[3]);

          //t看作(X,Y,Z)
          //计算两组点的质心
          point_g += tg_v;

          //SE3
          Sophus::SE3 pose_g(qg,tg_v);
          poses_g.push_back(pose_g);
      }
    }
    File2.close();




    point_e *= (1.0/poses_e.size());
    point_g *= (1.0/poses_e.size());


    //e->g
    Matrix3d R = CalculateR(point_g,point_e,poses_g,poses_e);
    Vector3d t = point_g - R*point_e;

    Sophus::SE3 SE3_T(R,t);
    cout<<"Tge = "<<endl<<SE3_T.matrix().inverse()<<endl;
    Matrix4d T_Update;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_update;

    //g->e
    for(int i = 0; i< poses_e.size();i++)
    {
       T_Update = SE3_T.matrix().inverse()*poses_g[i].matrix();
       Sophus::SE3 pose_g_update(T_Update.block<3,3>(0,0),T_Update.block<3,1>(0,3));
       poses_update.push_back(pose_g_update);

    }

    DrawTrajectory(poses_e,poses_update);

    ShowRMSE(poses_update,poses_e);


    // draw original trajectory in pangolin
    //DrawTrajectory(poses_g,poses_e);



    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) {
    if (poses1.empty()||poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        //Draw ground truth
        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            glColor3f(250.0f, 0.0f, 0);
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            //两点连线
            glEnd();
        }


        //Draw estimated
        glLineWidth(2);
        for (size_t i = 0; i < poses2.size() - 1; i++) {
            glColor3f(0.0f, 250.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        //usleep(5000);   // sleep 5 ms
    }

}



Matrix3d CalculateR(Vector3d p_g, Vector3d p_e, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3> > poses_g, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3> > poses_e)
{
   Matrix3d W = Matrix3d::Zero();
   Matrix3d R;
  //qi = ei - pe; qi' = gi - p_e;
  for(int i = 0; i< poses_e.size();i++)
  {
    Vector3d point_e = poses_e[i].matrix().block<3,1>(0,3);
    Vector3d qe = point_e - p_e;

    Vector3d point_g = poses_g[i].matrix().block<3,1>(0,3);
    Vector3d qg = point_g - p_g;

    //Calculate W
    W += qg*qe.transpose();

  }

  JacobiSVD<MatrixXd> svd(W, ComputeFullU | ComputeFullV);

  R = svd.matrixU()*svd.matrixV().transpose();

  return R;

}


void ShowRMSE(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e){

  double Sum_ei_d = 0;
  size_t size = min(poses_g.size(), poses_e.size());

  for (size_t i = 0; i < size ; i++) {

    Sophus::SE3 Tg_inv = poses_g[i].inverse();


    Sophus::SE3 Ei_SE3;
    Ei_SE3 = Tg_inv*poses_e[i];
    //Result 6x1 vector
    typedef Eigen::Matrix<double,6,1> Vector6d;
    Vector6d ei_v= Ei_SE3.log();


    //||ei||2
    Eigen::MatrixXd ei_2 = ei_v.transpose()*ei_v;

    //Sum
    Sum_ei_d = Sum_ei_d + ei_2(0,0);

  }

  //n = poses.size
  double dRMSE = sqrt((1/double(size)) * Sum_ei_d );

  cout << "RMSE = " << dRMSE << endl;



}

