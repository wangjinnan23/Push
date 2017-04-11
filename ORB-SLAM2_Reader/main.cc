#include <iostream>
#include <Eigen/Dense>


//using Eigen::MatrixXd;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;


using namespace std;


int main()
{

        cout<<"*******************1D-object****************"<<endl;


        Vector4d v1;
        v1<< 1,2,3,4;
        cout<<"v1=\n"<<v1<<endl;


        VectorXd v2(3);
        v2<<1,2,3;
        cout<<"v2=\n"<<v2<<endl;


        Array4i v3;
        v3<<1,2,3,4;
        cout<<"v3=\n"<<v3<<endl;


        ArrayXf v4(3);
        v4<<1,2,3;
        cout<<"v4=\n"<<v4<<endl;
}
