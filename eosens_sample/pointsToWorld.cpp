using namespace cv;
#include "pointsToWorld.h"
//誤差が結構乗る　桁落ちが原因かもしれない
void pointsToWorld(double A, double u, double v, double *point_w)
{
	// 		a b c 		d
	// R=	e f g 	t=	h
	// 		i j k		l
	long double a, b, c, d, e, f, g, h, i, j, k, l;
	Mat1d rvec(3, 1);//1dをつけるのを忘れていて誤差が乗るバグがあった　2018/01/31
	rvec(1, 0) = -1.5734859013414031e-02;
	rvec(2, 0) = 3.8540140385001559e-02;
	rvec(3, 0) = -3.3601829722008696e-02;
	
	//Mat rvec33(3, 3, cv::DataType<long double>::type);
	Mat1d rvec33(3,3);
	long double rvec3[3][3];
	double homo[12];
	double cameraMat[9];
	Rodrigues(Vec3d(-1.7704202610533269e-02, 1.0087691995358833e-03, -3.1504014037017566e-02), rvec33);

	//a = rvec33.at<double>(0, 0); b = rvec33.at<double>(0, 1); c = rvec33.at<double>(0, 2);
	//e = rvec33.at<double>(1, 0); f = rvec33.at<double>(1, 1); g = rvec33.at<double>(1, 2);
	//i = rvec33.at<double>(2, 0); j = rvec33.at<double>(2, 1); k = rvec33.at<double>(2, 2);

	a = rvec33(0, 0); b = rvec33(0, 1); c = rvec33(0, 2);
	e = rvec33(1, 0); f = rvec33(1, 1); g = rvec33(1, 2);
	i = rvec33(2, 0); j = rvec33(2, 1); k = rvec33(2, 2);

	d = -2.1071242174317445e-01;
	h = -5.2834036637918233e-02;
	l = 6.2727378334584816e-01;

	//homo[0] = rvec33.at<double>(0, 0); homo[1] = rvec33.at<double>(0, 1); homo[2] = rvec33.at<double>(0, 2); homo[3] = d;
	//homo[4] = rvec33.at<double>(1, 0); homo[5] = rvec33.at<double>(1, 1); homo[6] = rvec33.at<double>(1, 2); homo[7] = h;
	//homo[8] = rvec33.at<double>(2, 0); homo[9] = rvec33.at<double>(2, 1); homo[10] = rvec33.at<double>(2, 2); homo[11] = l;
	homo[0] = rvec33(0, 0); homo[1] = rvec33(0, 1); homo[2] = rvec33(0, 2); homo[3] = d;
	homo[4] = rvec33(1, 0); homo[5] = rvec33(1, 1); homo[6] = rvec33(1, 2); homo[7] = h;
	homo[8] = rvec33(2, 0); homo[9] = rvec33(2, 1); homo[10] = rvec33(2, 2); homo[11] = l;
	Mat homoMat = Mat(3, 4, CV_64F, homo);

	long double fx = 1.0440197075029514e+03;
	long double fy = 1.0440197075029514e+03;
	long double cx = 4.0247113475503613e+02;
	long double cy = 3.0094079274175499e+02;

	cameraMat[0] = fx;  cameraMat[1] = 0.0; cameraMat[2] = cx;
	cameraMat[3] = 0.0; cameraMat[4] = fy;  cameraMat[5] = cy;
	cameraMat[6] = 0.0; cameraMat[7] = 0.0; cameraMat[8] = 1.0;
	Mat cameraMatrix = Mat(3, 3, CV_64F, cameraMat);
	Mat ch = cameraMatrix*homoMat;

	double P[9];
	P[0] = ch.at<double>(0, 0); P[1] = ch.at<double>(0, 1); P[2] = A * ch.at<double>(0, 2) + ch.at<double>(0, 3);
	P[3] = ch.at<double>(1, 0); P[4] = ch.at<double>(1, 1); P[5] = A * ch.at<double>(1, 2) + ch.at<double>(1, 3);
	P[6] = ch.at<double>(2, 0); P[7] = ch.at<double>(2, 1); P[8] = A * ch.at<double>(2, 2) + ch.at<double>(2, 3);
	Mat p = Mat(3, 3, CV_64F, P);
	double imgCrd[3] = { u,v,1 };
	Mat imageCord = Mat(3, 1, CV_64F, imgCrd);
	Mat result = p.inv()*imageCord;
	long double x = (u - cx)/fx;
	long double y = (v - cy)/fy;
	long double tmpXa = (A * b * g 
				  - A * b * k * y 
				  - A * c * f
				  + A * c * j * y 
				  + A * f * k * x 
				  - A * g * j * x 
				  + b * h 
				  - b * l * y 
				  - d * f 
				  + d * j * y 
				  + f * l * x 
				  - h * j * x);
	long double tmpXb = (-a * f 
				   + a * j * y 
				   + b * e 
				   - b * i * y 
				   - e * j * x 
				   + f * i * x); //0だとだめ
	long double X = -tmpXa / tmpXb;

	long double tmpYa = (a * A * g 
				  - a * A * k * y 
				  + a * h
				  - a * l * y 
				  - A * c * e 
				  + A * c * i * y 
				  + A * e * k * x 
				  - A * g * i * x 
				  - d * e 
				  + d * i * y 
				  + e * l * x 
				  - h * i * x);
	long double tmpYb = (a * f 
				  - a * j * y 
				  - b * e 
				  + b * i * y
				  + e * j * x 
				  - f * i * x); //0だとだめ
	long double Y = -tmpYa / tmpYb;




	double tmpXaa = (
		+ b * h
		- b * l * y
		- d * f
		+ d * j * y
		+ f * l * x
		- h * j * x);
	double tmpXbb = (-a * f
		+ a * j * y
		+ b * e
		- b * i * y
		- e * j * x
		+ f * i * x); //0だとだめ
	double XX = -tmpXa / tmpXb;

	double tmpYaa = (
		- d * e
		+ d * i * y
		+ e * l * x
		- h * i * x);
	double tmpYbb = (a * f
		- a * j * y
		- b * e
		+ b * i * y
		+ e * j * x
		- f * i * x); //0だとだめ
	double YY = -tmpYa / tmpYb;
	//point_w[0] = X;
	//point_w[1] = Y;
	point_w[0] = result.at<double>(0)/ result.at<double>(2);
	point_w[1] = result.at<double>(1)/ result.at<double>(2);
	//point_w[0] = rvec(1,0);//homoMat.at<double>(0,0);
	//point_w[1] = b;//homoMat.at<double>(0,1);
}