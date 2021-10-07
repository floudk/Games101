#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
using std::sin;
using std::cos;
using std::cout;
using std::endl;
int main(){

    Eigen::Vector3f point(2.0f,1.0f,1.0f);
    cout<<"initial point:\n"<<point<<endl;
    //逆时针旋转45◦
    float a = 45;
    float a2angle = a/180.0*std::acos(-1);
    Eigen::Matrix3f HelpM1,HelpM2;
    HelpM1<<cos(a2angle),sin(a2angle),0.0f,-sin(a2angle),cos(a2angle),0.0f,0.0f,0.0f,1.0f;
    point=point.transpose()*HelpM1;
    cout<<"after rotate\n"<<point<<endl;
    //再平移(1,2),
    float x = 1.0f, y=2.0f;
    HelpM2<<1.0f,0.0f,0.0f,0.0f,1.0f,0.0f,x,y,1.0f;
    point=point.transpose()*HelpM2;
    //输出坐标
    cout<<point<<endl;



    return 0;
}