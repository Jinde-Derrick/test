#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


int main(int argc, char** argv) {

	cv::Mat image = imread("lena.jpg", CV_LOAD_IMAGE_COLOR);
	
	//創造
	Mat im_test1(image.rows, image.cols, CV_8UC3, Scalar(0, 0, 0));

	//複製
	//cv::Mat_ <cv::Vec3b> im_test1 = image.clone();

	namedWindow("Origin image 1", CV_WINDOW_NORMAL);
	imshow("Origin image 1", image);

	/*------------------------------------------------------------------------*/
	//轉灰階

	for (int i = 0; i<im_test1.rows; i++) {
		for (int j = 0; j<im_test1.cols; j++) {

			float sum = 0;
			
			sum = (image.at<cv::Vec3b>(i, j)[0] + image.at<cv::Vec3b>(i, j)[1] + image.at<cv::Vec3b>(i, j)[2]) / 3.0;
					
			im_test1.at<cv::Vec3b>(i, j)[0] = sum;
			im_test1.at<cv::Vec3b>(i, j)[1] = sum;
			im_test1.at<cv::Vec3b>(i, j)[2] = sum;
				
		}
	}

	namedWindow("灰階", CV_WINDOW_NORMAL);
	imshow("灰階", im_test1);

	/*------------------------------------------------------------------------*/
	//加雜訊


	for (int k = 0; k < 10000; k++){

		int i = rand() % im_test1.rows;
		int j = rand() % im_test1.cols;
		im_test1.at<cv::Vec3b>(i, j)[0] = 255;
		im_test1.at<cv::Vec3b>(i, j)[1] = 255;
		im_test1.at<cv::Vec3b>(i, j)[2] = 255;
	}

	namedWindow("雜訊", CV_WINDOW_NORMAL);
	imshow("雜訊", im_test1);


	/*------------------------------------------------------------------------*/
	//濾波
	//convolution filter image processing 
	//使用opencv內建濾波

	//Kernel 
	float MC[9] = { 1 / 9.f, 1 / 9.f, 1 / 9.f,
					1 / 9.f, 1 / 9.f, 1 / 9.f,
					1 / 9.f, 1 / 9.f, 1 / 9.f };
	
	//Mat(int rows, int cols, int type, void* data, size_t step = AUTO_STEP);
	cv::Mat cvM(3,3,CV_32FC1,MC);

	Mat imtest_opencv(image.rows, image.cols, CV_8UC3, Scalar(0, 0, 0));

	cv::filter2D(im_test1, imtest_opencv, -1, cvM);

	namedWindow("cvfilter", CV_WINDOW_NORMAL);
	imshow("cvfilter", imtest_opencv);

	/*------------------------------------------------------------------------*/
	//濾波-DIY
	//convolution filter image processing 
	
	//原理
	//http://www.csie.ntnu.edu.tw/~u91029/Image.html#4
	//https://drive.google.com/drive/folders/0B7ApYvCSc8x0WVpSdjNLZlhUOXM

	//PS 中值濾波 對胡椒鹽雜訊效果好 (下面沒有-再自己寫QQ)
	//中值就是點乘9宮格後排序 取中間那個值 (非平均)
	//https://sites.google.com/a/ms.ttu.edu.tw/cse2012dance-robot/yan-jiu-cheng-guo/opencv-ruan-ti-she-ji/zhong-zhi-fa

	Mat imtest2(image.rows, image.cols, CV_8UC3, Scalar(0, 0, 0));

	//Kernel 
	//https://en.wikipedia.org/wiki/Kernel_(image_processing)
	//理論上Kernel大小(幾乘幾)都要可以調 下面程式就需要重新設計

	//平滑(平均)
	float M[3][3] = { 1 / 9.f, 1 / 9.f, 1 / 9.f,
					  1 / 9.f, 1 / 9.f, 1 / 9.f,
					  1 / 9.f, 1 / 9.f, 1 / 9.f };

	//Sharpen-銳化
	float M2[3][3] ={ {0,-1,0},
					  {-1,5,-1}, 
	                  {0,-1,0} };

	//Gaussian-高斯
	float M3[3][3] = { { 1 / 16.f, 2 / 16.f, 1 / 16.f },
					   { 2 / 16.f, 4 / 16.f, 2 / 16.f },
					   { 1 / 16.f, 2 / 16.f, 1 / 16.f } };

	//
	int i, j;
	
	for (i = 0+3; i<imtest2.rows-3;i++) {

		for (j = 0+3; j<imtest2.cols-3;j++) {
			
				float sum = 0;

				
				for ( int pi = i - 1, mi=0; pi <= i+1; pi++, mi++) {
					for ( int pj = j - 1, mj=0; pj <= j+1; pj++, mj++) {						

						//試試不同Kernel
						
						//對灰階原圖
						//sum = sum + M[mi][mj] * image.at<cv::Vec3b>(pi, pj)[0]; //平滑濾波
						//sum = sum + M2[mi][mj] * image.at<cv::Vec3b>(pi, pj)[0]; //銳化濾波
						//sum = sum + M3[mi][mj] * image.at<cv::Vec3b>(pi, pj)[0]; //高斯濾波

						//對雜訊
						sum = sum + M[mi][mj] * im_test1.at<cv::Vec3b>(pi, pj)[0]; //平滑濾波
						//sum = sum + M2[mi][mj] * im_test1.at<cv::Vec3b>(pi, pj)[0]; //銳化濾波
						//sum = sum + M3[mi][mj] * im_test1.at<cv::Vec3b>(pi, pj)[0]; //高斯濾波

					}
				}

				imtest2.at<cv::Vec3b>(i, j)[0] = sum;
				imtest2.at<cv::Vec3b>(i, j)[1] = sum;
				imtest2.at<cv::Vec3b>(i, j)[2] = sum;
			
		}
	}
	



	namedWindow("DIY_filter", CV_WINDOW_NORMAL);
	imshow("DIY_filter", imtest2);


	/*------------------------------------------------------------------------*/
	//產生測試影像
	Mat imtes5(image.rows, image.cols, CV_8UC3, Scalar(0, 0, 0));

	for (int i = 0; i<imtes5.rows;i++) {
		for (int j = 0; j<imtes5.cols; j++) {

			for (int k = 0; k <= 2; k++) {

				if (j % 4 == 0)
				imtes5.at<cv::Vec3b>(i, j)[k] = 150;

				if (i % 8 == 0)
				imtes5.at<cv::Vec3b>(i, j)[k] = 220;

			}

		}
	}
	namedWindow("TT", CV_WINDOW_NORMAL);
	imshow("TT", imtes5);
	

	/*------------------------------------------------------------------------*/
	waitKey(0);
	return 0;
}
