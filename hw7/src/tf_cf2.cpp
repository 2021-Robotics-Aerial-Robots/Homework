#include <ros/ros.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>


using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{

    //下面三個變量作為下面演示的中間變量

    AngleAxisd t_V(M_PI / 4, Vector3d(0, 0, 1));
    Matrix3d t_R = t_V.matrix();
    Quaterniond t_Q(t_V);


    //旋轉向量（軸角）賦值的三大種方法

    //1.使用旋轉的角度和旋轉軸向量（此向量為單位向量）來初始化角軸
    AngleAxisd V1(M_PI / 4, Vector3d(0, 0, 1));//以（0,0,1）為旋轉軸，旋轉45度
    cout << "Rotation_vector1" << endl << V1.matrix() << endl;

    //2.使用旋轉矩陣轉旋轉向量的方式

    //2.1 使用旋轉向量的fromRotationMatrix()函數來對旋轉向量賦值（注意此方法為旋轉向量獨有,四元數沒有）
    AngleAxisd V2;
    V2.fromRotationMatrix(t_R);
    cout << "Rotation_vector2" << endl << V2.matrix() << endl;

    //2.2 直接使用旋轉矩陣來對旋轉向量賦值
    AngleAxisd V3;
    V3 = t_R;
    cout << "Rotation_vector3" << endl << V3.matrix() << endl;

    //2.3 使用旋轉矩陣來對旋轉向量進行初始化
    AngleAxisd V4(t_R);
    cout << "Rotation_vector4" << endl << V4.matrix() << endl;

    //3. 使用四元數來對旋轉向量進行賦值

    //3.1 直接使用四元數來對旋轉向量賦值
    AngleAxisd V5;
    V5 = t_Q;
    cout << "Rotation_vector5" << endl << V5.matrix() << endl;

    //3.2 使用四元數來對旋轉向量進行初始化
    AngleAxisd V6(t_Q);
    cout << "Rotation_vector6" << endl << V6.matrix() << endl;


//------------------------------------------------------

    //對四元數賦值的三大種方法（注意Eigen庫中的四元數前三維是虛部,最後一維是實部）

    //1.使用旋轉的角度和旋轉軸向量（此向量為單位向量）來初始化四元數,即使用q=[cos(A/2),n_x*sin(A/2),n_y*sin(A/2),n_z*sin(A/2)]
    Quaterniond Q1(cos((M_PI / 4) / 2), 0 * sin((M_PI / 4) / 2), 0 * sin((M_PI / 4) / 2), 1 * sin((M_PI / 4) / 2));//以（0,0,1）為旋轉軸，旋轉45度
    //第一種輸出四元數的方式
    cout << "Quaternion1" << endl << Q1.coeffs() << endl;

    //第二種輸出四元數的方式
    cout << Q1.x() << endl << endl;
    cout << Q1.y() << endl << endl;
    cout << Q1.z() << endl << endl;
    cout << Q1.w() << endl << endl;

    //2. 使用旋轉矩陣轉四元數的方式

    //2.1 直接使用旋轉矩陣來對旋轉向量賦值
    Quaterniond Q2;
    Q2 = t_R;
    cout << "Quaternion2" << endl << Q2.coeffs() << endl;


    //2.2 使用旋轉矩陣來對四元數進行初始化
    Quaterniond Q3(t_R);
    cout << "Quaternion3" << endl << Q3.coeffs() << endl;

    //3. 使用旋轉向量對四元數來進行賦值

    //3.1 直接使用旋轉向量對四元數來賦值
    Quaterniond Q4;
    Q4 = t_V;
    cout << "Quaternion4" << endl << Q4.coeffs() << endl;

    //3.2 使用旋轉向量來對四元數進行初始化
    Quaterniond Q5(t_V);
    cout << "Quaternion5" << endl << Q5.coeffs() << endl;



//----------------------------------------------------

    //對旋轉矩陣賦值的三大種方法

    //1.使用旋轉矩陣的函數來初始化旋轉矩陣
    Matrix3d R1=Matrix3d::Identity();
    cout << "Rotation_matrix1" << endl << R1 << endl;

    //2. 使用旋轉向量轉旋轉矩陣來對旋轉矩陣賦值
    
    //2.1 使用旋轉向量的成員函數matrix()來對旋轉矩陣賦值
    Matrix3d R2;
    R2 = t_V.matrix();
    cout << "Rotation_matrix2" << endl << R2 << endl;

    //2.2 使用旋轉向量的成員函數toRotationMatrix()來對旋轉矩陣賦值
    Matrix3d R3;
    R3 = t_V.toRotationMatrix();
    cout << "Rotation_matrix3" << endl << R3 << endl;

    //3. 使用四元數轉旋轉矩陣來對旋轉矩陣賦值

    //3.1 使用四元數的成員函數matrix()來對旋轉矩陣賦值
    Matrix3d R4;
    R4 = t_Q.matrix();
    cout << "Rotation_matrix4" << endl << R4 << endl;

    //3.2 使用四元數的成員函數toRotationMatrix()來對旋轉矩陣賦值
    Matrix3d R5;
    R5 = t_Q.toRotationMatrix();
    cout << "Rotation_matrix5" << endl << R5 << endl;

    return 0;

}


