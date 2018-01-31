using namespace cv;
#include "pointsToWorld.h"

void pointsToWorld(double A, double u, double v, double *point_w)
{	
	// 		a b c 		d
	// R=	e f g 	t=	h
	// 		i j k		l
	double a,b,c,d,e,f,g,h,i,j,k,l;
	Mat rvec = (Mat_<double>(3,1) << -1.5734859013414031e-02, 3.8540140385001559e-02, -3.3601829722008696e-02);
	Mat rvec33;// (Mat_<double>(3, 3));
	Rodrigues(rvec, rvec33);
	a = rvec33.at<double>(0,0); b = rvec33.at<double>(0,1); c = rvec33.at<double>(0,2);
	e = rvec33.at<double>(1.0); f = rvec33.at<double>(1,1); g = rvec33.at<double>(1,2);
	i = rvec33.at<double>(2,0); j = rvec33.at<double>(2,1); k = rvec33.at<double>(2,2);

	d = -1.8384949000129683e-01;
	h = -5.2925587512327975e-02;
	l = 6.3507158879309944e-01;
	double cx = 4.0181508814556707e+02;
	double cy = 3.0103483917618354e+02;
	double fx = 1.0440790575580390e+03;
	double fy = 1.0440790575580390e+03;
	double x = (u - cx)/fx;
	double y = (v - cy)/fy;
	double tmpXa = (A * b * g - A * b * k * y + A * c * j * y + A * f * k * x - A * g * j * x 
				  + b * h - b * l * y - d * f + d * j * y + f * l * x - h * j * x);
	double tmpXb = (-a * f + a * j * y + b * e - b * i * y - e * j * x + f * i * x); //0だとだめ
	double X = -tmpXa / tmpXb;
	double tmpYa = (a * A * g - a * A * k * y + a * h - a * l * y - A * c * e + A * c * i * y 
				  + A * e * k * x - A * g * i * x - d * e + d * i * y + e * l * x - h * i * x);
	double tmpYb = (a * f - a * j * y - b * e + b * i * y + e * j * x - f * i * x); //0だとだめ
	double Y = -tmpYa / tmpYb;
	point_w[0] = X;
	point_w[1] = Y;
}