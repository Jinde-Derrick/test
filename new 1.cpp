
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <highgui.h>

#include <cv.h>
#include <cxcore.h>
#include <cstdlib>
#include <ctype.h>

#include <unistd.h>

#include <fcntl.h>
#include <fstream>

#include <termios.h>
#include <iostream>

#include <math.h>


using namespace cv;
using namespace std;


void TXS(string myWord, int uart_filestream){

	char tab2[1024];
	strcpy(tab2, myWord.c_str());

	if(uart_filestream !=-1)
	{
		//int count=write(uart_filestream,&tx_buf[0],(p_buffer-&tx_buf[0]));
		int count = write( uart_filestream, &tab2[0], myWord.length() );

		if(count<0)
			printf("TX error \r\n");
	}

}

void contourFilter(Mat image, Mat skin_img)
{

	RNG rng(12345);
	Mat threshold_img;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	threshold(skin_img, threshold_img, 125, 255, THRESH_BINARY);
	//將skin_img二值化

	findContours(threshold_img, contours, hierarchy,
		CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	//在threshold_img影像中尋找所有的輪廓

	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		//計算可以包含輪廓的最小長方形區域

		boundRect[i] = boundingRect(Mat(contours_poly[i]));

		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		//隨機給一個顏色


		if (boundRect[i].area() > 900)
		{//長方形區域面積超過900，則畫在影像上


			drawContours(image, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
			rectangle(image, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);


		}
	}
	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	imshow("Contours", image);

}



void calcCircles(const Mat &input, vector<Vec3f> &circles){

	Mat contours;

	Canny(input,contours,50,200);

	HoughCircles(contours, circles, CV_HOUGH_GRADIENT, 

		1, 
		200,//160, 

		150, 
		5

		,1,
		120

		);

	/*

	//! finds circles in the grayscale image using 2+1 gradient Hough transform
	CV_EXPORTS_W void HoughCircles( InputArray image, OutputArray circles,
	int method, 

	double dp, 
	double minDist,

	double param1=100, 
	double param2=100,

	int minRadius=0, 
	int maxRadius=0 );


	void HoughCircles(InputArray image, OutputArray circles, int method, double dp, double minDist, double param1=100, doubleparam2=100, int minRadius=0, int maxRadius=0)

	image：輸入圖，8位元單通道圖。
	circles：以vector< Vec3f >記錄所有圓的資訊，每個Vec3f紀錄一個圓的資訊，包含3個浮點數資料，分別表示x、y、radius。
	method：偵測圓的方法，目前只能使用CV_HOUGH_GRADIENT。

	dp：偵測解析度倒數比例，假設dp=1，偵測圖和輸入圖尺寸相同，假設dp=2，偵測圖長和寬皆為輸入圖的一半。
	minDist：圓彼此間的最短距離，太小的話可能會把鄰近的幾個圓視為一個，太大的話可能會錯過某些圓。

	param1：圓偵測內部會呼叫Canny()尋找邊界，param1就是Canny()的高閾值，低閾值自動設為此值的一半。
	param2：計數閾值，超過此值的圓才會存入circles。

	minRadius：最小的圓半徑。
	maxRadius：最大的圓半徑。


	*/
}



void drawCircle(Mat &input, const vector<Vec3f> &circles){


	for(int i=0; i<circles.size(); i++){


		Point center( cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		circle(input, center, radius, Scalar(255,0,0), 3, 8, 0 );


		/*
		int x,y;

		//cvRound 320 X 240

		printf("circles.size=%d\n",circles.size());

		x=circles[0][0]; 
		y=circles[0][1];
		radius = circles[0][2];

		printf("x=%d\ny=%d\nr=%d\n",x,y,radius);
		*/



	}



}



int main(int argc, char** argv)
{
	VideoCapture cap;
	cap.open(0);

	Mat image;
	Mat hsvImg, skinImg, resizeImg;



	//setup UART
	int uart_filestream=-1;
	uart_filestream=open("/dev/ttyAMA0",O_RDWR | O_NOCTTY | O_NDELAY);


	if(uart_filestream==-1)
	{
		printf("Unable to Open UART \r\n ");
		return 0;
	}

	//Configure UART
	struct termios options;
	tcgetattr(uart_filestream,&options);
	options.c_cflag=B9600 | CS8 | CLOCAL | CREAD;
	options.c_iflag=IGNPAR;
	options.c_oflag=0;
	options.c_lflag=0;
	tcflush(uart_filestream,TCIFLUSH);
	tcsetattr(uart_filestream,TCSANOW,&options);

	string myWord = "";

	while(1)
	{
		Mat frame;
		cap >> frame;

		if (frame.empty())
			break;

		cv::resize(frame, resizeImg, cv::Size(320, 240));
		resizeImg.copyTo(image);


		/*------------------------------------------------------------------------*/
		//濾波
		//convolution filter image processing 
		//使用opencv內建濾波

		//Kernel

		///*
		//Sharpen-銳化
		//float MC2[3][3] ={ 0,-1,0,-1,5,-1,0,-1,0 };
		//*/

		/*
		//Gaussian-高斯
		float MC[3][3] = {  1 / 16.f, 2 / 16.f, 1 / 16.f ,
		2 / 16.f, 4 / 16.f, 2 / 16.f ,
		1 / 16.f, 2 / 16.f, 1 / 16.f  };
		*/


		/*
		float MC2[9] = { 1 / 9.f, 1 / 9.f, 1 / 9.f,
		1 / 9.f, 1 / 9.f, 1 / 9.f,
		1 / 9.f, 1 / 9.f, 1 / 9.f };
		*/


		//Mat(int rows, int cols, int type, void* data, size_t step = AUTO_STEP);
		//cv::Mat cvM(3,3,CV_32FC1,MC);
		//cv::filter2D(image, image, -1, cvM);
		GaussianBlur( image, image, Size(5, 5), 2, 2 );


		cvtColor(image, hsvImg, CV_BGR2HSV);


		//inRange(hsvImg, Scalar(0, 58, 20), Scalar(50, 173, 230), skinImg);
		inRange(hsvImg, Scalar(13, 150, 150), Scalar(37, 255, 255), skinImg);
		//篩選hsvImg在HSV顏色空間屬於黃色的區域


		cv::dilate(skinImg, skinImg, Mat(), Point(-1, -1), 4);
		//對skinImg進行膨脹


		GaussianBlur( skinImg, skinImg, Size(5,5), 2, 2 );
		//

		cv::erode(skinImg, skinImg, Mat(), Point(-1, -1), 5);
		//對skinImg進行侵蝕




		vector<Vec3f> circles;
		calcCircles(skinImg, circles);

		//
		//drawCircle(image, circles);

		imshow("img", image);
		//imshow("Demo", skinImg);


		//contourFilter(image, skinImg);
		//imshow("OpenCV Demo", skinImg);


		waitKey(1);



		///*
		int x,y,radius;

		//cvRound 320 X 240
		if(circles.size()>0){

			///*
			printf("circles.size=%d\n",circles.size());

			x = circles[0][0]; 
			y = circles[0][1];
			radius = circles[0][2];

			printf("x=%d\ny=%d\nr=%d\n",x,y,radius);


			//Transmitting   
			if(x<=100)
				myWord = "150,55\n";
			else if (x>=220)
				myWord = "150,115\n";
			else if(100<x && x<220)
				myWord = "150,85\n";

			TXS(myWord,uart_filestream);

		}

		//*/


	}

	close(uart_filestream);
	printf("finish\r\n");
	return 0;
}

