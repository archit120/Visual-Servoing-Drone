#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>    

#include <math.h>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
using namespace Eigen;


//using Eigen::completeOrthogonalDecomposition
//sift parameters for detection(f2d)
double cx = 0;
double cy = 0;
float f = 1;

float error_threshold = 0.1;

int lambda = 1; //for setting the extra speed of camera

vector<Point2d> reference;

vector<Point2d> get_points()
{
   auto a =  vector<Point2d>();   
   a.push_back(Point2d(0.5+1,1));
   a.push_back(Point2d(1,1));
   a.push_back(Point2d(0.5,1.5));
   a.push_back(Point2d(1,1.5));

   return a;
}

double get_depth()
{
    return 1;
}

int main(int argc, char **argv)
{
    
   reference.push_back(Point(2,0));
   reference.push_back(Point(1,0));
   reference.push_back(Point(0,1));
   reference.push_back(Point(1,1));

    int number = reference.size();

    MatrixXf velocity(4, 1);
    MatrixXf L1(1, 4);
    L1.resize(2 * reference.size(), 4);
    MatrixXf LInv(4, 1);
    LInv.resize(4, 2 * reference.size());

    MatrixXf s_ref(1, 1);
    s_ref.resize(2 * reference.size(), 1);
    MatrixXf s(1, 1);
    s.resize(2 * reference.size(), 1);
    int i = 0;
    for (int i = 0; i < reference.size(); i += 2)
    {
        s_ref(i, 0) = (reference[i].x - cx)/f;
        s_ref(i + 1, 0) = (reference[i].y - cy)/f;
    }

    while (1)
    {
        auto pt = get_points();
        auto d = get_depth();

        for (int i = 0; i < reference.size(); i += 2)
        {
            s(i, 0) = (pt[i].x - cx)/f;
            s(i + 1, 0) = (pt[i].y - cy)/f;
        }

        if((s-s_ref).norm() < error_threshold)
            break;

        for (int i = 0; i < reference.size(); i += 2)
        {
            float x = (pt[i].x - cx)/f;
            float y = (pt[i].y - cy)/f;
            L1(i, 0) = -1/d;
            L1(i, 1) = 0;
            L1(i, 2) = x/d;
            L1(i, 3) = y;

            L1(i+1, 0) = 0;
            L1(i+1, 1) = -1/d;
            L1(i+1, 2) = y/d;
            L1(i+1, 3) = -x;
        }

        LInv = (L1.transpose()*L1).inverse()*L1.transpose();
        
        velocity = -lambda*LInv*(s-s_ref);

        cout << velocity(0,0) << " " << velocity(1, 0) << " " << velocity(2, 0) << " " << velocity(3, 0) << "\n";

        cin >> number;
    }

    

    return (0);
}

void readme()
{
    cout << " Usage : ./new <img1> <img2> " << std::endl;
}