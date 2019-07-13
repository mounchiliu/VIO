#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.h"

#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()

using namespace std;

int main()
{
    //Random Rotation matrix
    srand(time(0));  // Initialize random number generator.
    double r = ((double) rand() / (RAND_MAX));
    Eigen::AngleAxisd rotation_vector(r*M_PI,Eigen::Vector3d::Random());
    cout<<"Rotation matrix= \n" <<rotation_vector.matrix()<<endl<<endl;
    Sophus::SO3 SO3_R(rotation_vector.toRotationMatrix());


    //Update //method 1
    Eigen::Vector3d update(0.01,0.02,0.03);
    Sophus::SO3 SO3_R_updated = SO3_R*Sophus::SO3::exp(update);
    cout<<"Updated result of Rexp(w^)= \n" <<SO3_R_updated.matrix()<<endl<<endl;

    //method 2
    Eigen::Quaterniond q_R(rotation_vector.toRotationMatrix());
    //q_R.normalize();
    Eigen::Quaterniond q_update(1,0.5*update[0],0.5*update[1],0.5*update[2]);
    //q_update.normalize();
    Eigen::Quaterniond q_R_updated = q_R * q_update;

    //Dont forget to normalize the quaternion.
    q_R_updated.normalize();
    cout<<"Updated result of Quaternion= \n" <<q_R_updated.matrix()<<endl<<endl;


    cout<<"Difference= \n" <<SO3_R_updated.matrix()-q_R_updated.matrix()<<endl<<endl;





}
