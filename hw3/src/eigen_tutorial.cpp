#include "ros/ros.h"
#include <Eigen/Dense>

Eigen::Vector3d TranslationFirst(Eigen::Vector3d vector,
                                 Eigen::Matrix3d rotation, Eigen::Vector3d translation){
    vector = rotation * (vector + translation);
    return vector;
}

Eigen::Vector3d RotationFirst(Eigen::Vector3d vector,
                              Eigen::Matrix3d rotation, Eigen::Vector3d translation){
    vector = rotation * vector + translation;
    return vector;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eigen_tutorial");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    // Matrix addition and subtraction
    Eigen::Matrix4d transform1;
    transform1 << 0, -1, 0, 1,
                  1, 0, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1;
    Eigen::Matrix4d transform2;
    transform2 << 0, 1, 0, 0,
                  -1, 0, 0, -1,
                  0, 0, 1, 0,
                  0, 0, 0, 1;
    std::cout << "transform1: \n" << transform1 << std::endl;
    std::cout << "transform2: \n" << transform2 << std::endl;
    std::cout << "transform1 + transform2: \n" << transform1 + transform2 << std::endl;
    std::cout << "transform1 - transform2: \n" << transform1 - transform2 << std::endl;
    std::cout << "-----------------------" << std::endl;

    // Vector dot and cross operation
    Eigen::Vector3d vector1(1, 0, 0);
    Eigen::Vector3d vector2(0, 1, 0);
    std::cout << "vector1: " << vector1.transpose() << std::endl;
    std::cout << "vector2: " << vector2.transpose() << std::endl;
    std::cout << "vector1 dot vector2: " << vector1.dot(vector2) << std::endl;
    std::cout << "vector1 cross vector2: " << vector1.cross(vector2).transpose() << std::endl;
    std::cout << "-----------------------" << std::endl;
    vector2 = vector1;
    while (ros::ok())
    {
        // Block opertaion for rotation and translation
        Eigen::Matrix3d rotation1 = transform1.block<3,3>(0,0);
        Eigen::Matrix3d rotation2 = transform2.block<3,3>(0,0);
        Eigen::Vector3d translation1 = transform1.block<3,1>(0,3);
        Eigen::Vector3d translation2 = transform2.block<3,1>(0,3);
        vector1 = TranslationFirst(vector1, rotation1, translation1);
        vector1 = TranslationFirst(vector1, rotation2, translation2);

        vector2 = RotationFirst(vector2, rotation1, translation1);
        vector2 = RotationFirst(vector2, rotation2, translation2);
        std::cout << "Apply TranslationFirst transform in vector1: " << vector1.transpose() << std::endl;
        std::cout << "Apply RotationFirst transform in vector2: " << vector2.transpose() << std::endl;

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
