#include "globalInclude.h"
#include "corrector.h"
int circle_r = 410;
vector<Point> center(4);
vector<string> img_names;
vector<cv::Mat> src_image;
vector<cv::Mat> out_image;
bool isDispCorrectRet = false;

void Ontrackbar(int pos, void* mat)
{
	Mat temp;
	temp = src_image[0].clone();
	cv::circle(temp, center[0], circle_r, Scalar(0, 0, 255), 3);
	cv::imshow("get_circle", temp);
}

int main()
{
	vector<correctParameters> params(4);
	
	vector<Mat> iMat(4);
	
	img_names.push_back("fisheye_image/front.jpg");
	img_names.push_back("fisheye_image/right.jpg");
	img_names.push_back("fisheye_image/back.jpg");
	img_names.push_back("fisheye_image/left.jpg");
	//读入矩阵
	cv::FileStorage Rmat("fisheye_mat/Mat.yml", cv::FileStorage::READ);
	if (!Rmat.isOpened()) // failed
	{
		std::cout << " File Failed!" << std::endl;
		return -1;
	}
	else // 打开成功
	{

		Rmat["intrinsic_0"] >> iMat[0];
		Rmat["intrinsic_1"] >> iMat[1];
		Rmat["intrinsic_2"] >> iMat[2];
		Rmat["intrinsic_3"] >> iMat[3];
	}
	
	for (int i = 0; i < iMat.size(); i++)
	{
		center[i] = Point(iMat[i].at<double>(0, 2), iMat[i].at<double>(1, 2));
		//center[i] = Point(1280.0/2, 720.0/2);
	}
	for (int i = 0; i < img_names.size(); i++)
	{
		Mat temp = cv::imread(img_names[i], 1);
		src_image.push_back(temp);
	}
	//cv::namedWindow("get_circle");
	//cv::imshow("get_circle", src_image[0]);
	////寻找圆参数
	//cv::createTrackbar("半径", "get_circle", &circle_r, src_image[0].cols/2, Ontrackbar);

	//cv::waitKey(0);
	//cv::destroyWindow("get_circle");
	for (int i = 0; i < params.size(); i++)
	{
		corrector adjuster;

		params[i].imgOrg = src_image[i].clone();
		params[i].center = center[i];
		params[i].radius = circle_r;
		//findCircleParameter::getCircleParatemer(params[i].center, params[i].radius);
		params[i].w_longtitude = PI / 2;
		params[i].w_latitude = PI / 2;
		params[i].distMap = PERSPECTIVE;
		params[i].theta_left = 0;
		params[i].phi_up = 0;
		params[i].camerFieldAngle = PI;
		params[i].camProjMode = EQUIDISTANCE;
		params[i].typeOfCorrect = Forward;
		corrector::correctMethod method = corrector::correctMethod::PERSPECTIVE_LONG_LAT_MAP_CAMERA_LEN_MODEL;
		out_image.push_back(
			adjuster.correctImage(params[i], method,
				isDispCorrectRet)
		);
		cv::namedWindow("out", CV_WINDOW_NORMAL);
		//cv::namedWindow("out2", CV_WINDOW_NORMAL);
		cv::imshow("out", out_image[i]);
		/*float cs = cos(60/180.0*PI);
		Mat ROI = out_image[i](Rect(0, 0, out_image[i].cols, out_image[i].rows*cs));
		Mat out2;
		cv::resize(ROI, out2, out_image[i].size());
		cv::imshow("out2", out2);*/
		cv::waitKey(0);
	}
	cv::destroyWindow("out");
	return 0;
}