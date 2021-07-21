// marker_detect_1.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <string>
#include <thread>
#include <ctime>

//#include "serial.h"
//#include "Safe_Network_tcp.h"

using namespace std;
using namespace cv;

const int MAX_MARKER_NUM = 1; // 一度に読めるマーカーの数
const double MARKER_SIZE = 100;  //マーカーの縦の長さをmmで指定

// 画面内に収まるマーカの範囲を指定
const double MIN_X = -21.5;
const double MAX_X = 100.0;
const double MIN_Y = 15.0;
const double MAX_Y = 89.0;

const int ADJUST_FR = -2;
const int ADJUST_FL = -4;
const int ADJUST_BR = 2;
const int ADJUST_BL = 4;

const double MAX_DIFF = 15; // 許容範囲

const int MAX_MOTOR_POWER = 19; // 最大出力 正方向　縦移動時
const int MIN_MOTOR_POWER = 17; // 最小出力　正方向　縦移動時

const int MAX_SIDE_MOTOR_POWER = 35; // 最大出力 正方向　横移動時
const int MIN_SIDE_MOTOR_POWER = 30; // 最小出力　正方向　横移動時

const int MAX_TURN_MOTOR_POWER = 45; // 最大出力　正方向　旋回時
const int MIN_TURN_MOTOR_POWER = 35; // 最小出力　正方向　旋回時

const double CENTER_X = (MIN_X + MAX_X) / 2;
const double CENTER_Y = (MIN_Y + MAX_Y) / 2;

int readMatrix(const char* filename, cv::Mat& cameraMat, cv::Mat& distCoeffs);
int CalibrationCamera(VideoCapture& cap, cv::Mat& cameraMat, cv::Mat& distCoeffs);

typedef enum RL{
	RIGHT,
	LEFT
} RL;

typedef enum XY {
	X,
	Y,
} XY;

typedef enum Mode {
	NORMAL, // 通常
	SETTING_TURN_RIGHT_90, // 右折設定
	SETTING_TURN_LEFT_90, // 左折設定
	SETTING_STRAIGHT, // 直進設定
	SETTING_BACK, // 後進設定
	MANUAL, // 手動
	GO_STRAIGHT, // 直進
	GO_RIGHT, // 右折
	GO_LEFT, // 左折
	GO_BACK, // 後進
	BACK, // 後進
	WAIT, // 待ち
} Mode;

typedef struct Motor_Power {
	int Front_Right_Motor;
	int Front_Left_Motor;
	int Back_Right_Motor;
	int Back_Left_Motor;
} Moter_Power;

int motor_FL_Data[5] = {0, 0, 0, 0, 0};
int motor_FR_Data[5] = {0, 0, 0, 0, 0};
int motor_BL_Data[5] = {0, 0, 0, 0, 0};
int motor_BR_Data[5] = {0, 0, 0, 0, 0};
int motor_count = 0;
int motor_FL_Sum = 0;
int motor_FR_Sum = 0;
int motor_BL_Sum = 0;
int motor_BR_Sum = 0;

void Not_moter_Outlier();

void moter_average_5(int FR, int FL, int BR, int BL);
void Reset_average_5();

int Map(int value, float min_a, float max_a, float min_b, float max_b);
void motor_map(float min, float max);

bool Positioning_X(double now_x); // 位置調整
bool Positioning_Y(double now_y); // 位置調整
bool Positioning(double now_x, double now_y);

void Send_Data(int fd); // 送信関数
void Recive_Data(int fd); // 受信関数

double Target_Angle(RL rl, double now_angle, int angle); // 目標角度取得関数

bool Turn(RL rl, double now_angle, double target_angle); // 旋回関数
bool GoStraight(int marker_num); // 直進関数
bool GoRight(int marker_num); // 右直進関数
bool GoLeft(int marker_num); // 左直進関数
bool GoBack(int marker_num); // 後直進関数

double kz = 3.1829046 / 3;  //実測値cmと、このパソコンの長さの調整係数
double kx = 2.06589452;
double ky = kx;

float Servo_angle = 90;
char Servo_angle_char[4] = "90";
char value_Servo[200]; 

int marker_set[4] = { 42, 18,
                      27, 43 };

struct Send_Data{
	int drone_distanceX;
	int drone_distanceY;
	int drone_distanceZ;
    int drone_angleX;
	int drone_angleY;
	int drone_angleZ;
};

int mode = NORMAL;
Moter_Power motor = {0, 0};
int xy_flag = Y; // 補正対象軸

time_t p_time = 0;
time_t n_time = 0;

int main(int argc, const char* argv[])
{
	cv::Mat drone_image;
	//背景画像
	cv::Mat dstImg;
	cv::Mat backimage;

	cv::Mat image;
	cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 4.50869314e+02, 0, 2.47413306e+02, 0, 4.55471466e+02, 1.85222260e+02, 0, 0, 1);
	cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 4.65341144e-01, -2.18649613e+00, 5.14274386e-03, 3.62427799e-03, 3.71240124e+00);
	// readMatrix("caliburation.xml",cameraMatrix, distCoeffs);  //xmlファイルからcameraMatrixとdistCoeffsを読み込む
	//dictionary生成
	const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

	//マーカーの検出
	std::vector<int> marker_ids;
	std::vector<std::vector<cv::Point2f>> marker_corners;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

	drone_image = cv::imread("drone.png", 1);
	if (drone_image.data == NULL) return -1;

	VideoCapture cap(0);  //カメラの映像の読み込み
    cap.set(3, 800); // 横
	cap.set(4, 800); // 縦

	Mat mirror_image;  //表示画面の左右反転の用意

	int key = 0;
	double t_angle; // 目標角度

	double angleX[MAX_MARKER_NUM];
	double angleY[MAX_MARKER_NUM];
	double angleZ[MAX_MARKER_NUM];
	for (int i = 0; i < MAX_MARKER_NUM; ++i) {
		angleX[i] = angleY[i] = angleZ[i] = 0.0;
	}


	double ave_angleX = 0;
	double ave_angleY = 0;
	double ave_angleZ = 0;

	double distanceX = 0;
	double distanceY = 0;
	double distanceZ = 0;
	double distanceR = 0;

	/* シリアルポートオープン */
	int fd = serialOpen("/dev/serial0", 57600);
	if(fd<0){
		printf("can not open serialport\n");
	}
	else {
		printf("open serialport\n");
	}
	wiringPiSetup();
	fflush(stdout);

	//中心等の座標の取得
	float marker_location[MAX_MARKER_NUM][8] = {};
	float marker_center[MAX_MARKER_NUM][2] = {};
	float all_marker_center[MAX_MARKER_NUM][2] = {};
	float average_center[2] = {};

	while (((key = cv::waitKey(1)) != 'q')) {  //qが押されるまで繰り返す
	
		dstImg = cv::imread("back.png", 1);
		if (dstImg.data == NULL) return -1;

		backimage = cv::imread("back2.png", 1);
		if (backimage.data == NULL) return -1;

		if (!cap.isOpened()){
			cap.open(0);
			this_thread::sleep_for(chrono::seconds(1));
			continue;
		}

		cap >> image;  //imageにカメラの映像を代入
		if (image.empty())  //映像がないときはとばす 
			continue;

		//マーカーの検知
		cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, parameters);
		//マーカーの描画
		cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

		if (MAX_MARKER_NUM < marker_ids.size()) {
			std::cout << "マーカーが多すぎる　：　" << marker_ids.size() << " 枚\n";
			continue;
		}

		// マーカなしならモータストップ
		if (marker_ids.size() == 0) {
			if (!(mode == GO_STRAIGHT || mode == GO_RIGHT || mode == GO_LEFT || mode == GO_BACK)) {
				motor.Front_Right_Motor = 0;
				motor.Front_Left_Motor = 0;
				motor.Back_Right_Motor = 0;
				motor.Back_Left_Motor = 0;	
			}
		}

        //それぞれのマーカーの中心のスクリーン座標系を取得
		for (int i = 0; i < marker_ids.size(); i++)
		{
			for (int j = 0; j < 4; j++)
			{
				marker_location[i][j * 2] = marker_corners[i][j].x;
				marker_location[i][(j * 2) + 1] = marker_corners[i][j].y;
				marker_center[i][0] = marker_center[i][0] + marker_corners[i][j].x;
				marker_center[i][1] = marker_center[i][1] + marker_corners[i][j].y;
			}
			marker_center[i][0] = marker_center[i][0] / 4;  //x座標
			marker_center[i][1] = marker_center[i][1] / 4;  //y座標
		}

		//4つのマーカーのスクリーン座標系の中心の取得
		for (int i = 0; i < marker_ids.size(); i++) {
			switch (marker_ids[i])
			{
			case 42:
				all_marker_center[i][0] = marker_location[i][4] + 0.6071 * (marker_location[i][4] - marker_center[i][0]);
				all_marker_center[i][1] = marker_location[i][5] + 0.5 * (marker_location[i][5] - marker_center[i][1]);

				if (mode == NORMAL) {
					mode = SETTING_TURN_RIGHT_90;
					cout << "Change mode : NORMAL  SETTING_TURN_RIGHT_90" << endl;
				}

				break;

			case 18:
				all_marker_center[i][0] = marker_location[i][6] - 0.6071 * (marker_center[i][0] - marker_location[i][6]);
				all_marker_center[i][1] = marker_location[i][7] + 0.5 * (marker_location[i][7] - marker_center[i][1]);

				if (mode == NORMAL) {
					mode = SETTING_TURN_LEFT_90;
					cout << "Change mode : NORMAL  SETTING_TURN_LEFT_90" << endl;
				}

				break;

			case 43:
				all_marker_center[i][0] = marker_location[i][0] - 0.6071 * (marker_center[i][0] - marker_location[i][0]);
				all_marker_center[i][1] = marker_location[i][1] - 0.5 * (marker_center[i][1] - marker_location[i][1]);
				
				if (mode == NORMAL) {
					mode = SETTING_STRAIGHT;
					cout << "Change mode : NORMAL  SETTING_STRAIGHT" << endl;
				}

				break;

			case 27:
				all_marker_center[i][0] = marker_location[i][2] + 0.6071 * (marker_location[i][2] - marker_center[i][0]);
				all_marker_center[i][1] = marker_location[i][3] - 0.5 * (marker_center[i][1] - marker_location[i][3]);
				
				if (mode == NORMAL) {
					mode = SETTING_BACK;
					cout << "Change mode : NORMAL  SETTING_BACK" << endl;
				}

				break;
			}
			average_center[0] = average_center[0] + all_marker_center[i][0];
			average_center[1] = average_center[1] + all_marker_center[i][1];
		}

		average_center[0] = average_center[0] / marker_ids.size();  //x座標
		average_center[1] = average_center[1] / marker_ids.size();  //y座標


        //カメラからARマーカーまでの距離を求める

        //回転行列を入れるための配列
		cv::Mat rmatrix = (cv::Mat_<double>(3, 3));

		//マーカーの3軸の検出、ｘｙｚの取得
		if (marker_ids.size() > 0)
		{
			ave_angleX = 0;
			ave_angleY = 0;
			ave_angleZ = 0;

			distanceX = 0;
			distanceY = 0;
			distanceZ = 0;
			distanceR = 0;

			cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
			std::vector<cv::Vec3d> rvecs, tvecs;
			cv::aruco::estimatePoseSingleMarkers(marker_corners, MARKER_SIZE * 0.001, cameraMatrix, distCoeffs, rvecs, tvecs);
			for (int i = 0; i < marker_ids.size(); i++)
			{
				//cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
				
				//3*1行列から3*3行列に戻す
				Rodrigues(rvecs[i], rmatrix);

				angleX[i] = atan2(rmatrix.at<double>(2, 1), rmatrix.at<double>(2, 2)) * 360 / (2 * 3.14);
				if (angleX[i] > 0) {
					angleX[i] = 180 - (atan2(rmatrix.at<double>(2, 1), rmatrix.at<double>(2, 2)) * 360 / (2 * 3.14));
				}
				else {
					angleX[i] = - 180 - (atan2(rmatrix.at<double>(2, 1), rmatrix.at<double>(2, 2)) * 360 / (2 * 3.14));
				}

				angleY[i] = asin(-rmatrix.at<double>(2, 0)) * 360 / (2 * 3.14);
				angleZ[i] = atan2(rmatrix.at<double>(1, 0), rmatrix.at<double>(0, 0)) * 360 / (2 * 3.14);

				ave_angleX += angleX[i];
				ave_angleY += angleY[i];
				ave_angleZ += angleZ[i];

				cv::Mat tmatrix = (cv::Mat_<double>(3, 1) << tvecs[i][0], tvecs[i][1], tvecs[i][2]);
				cv::Mat distance = (cv::Mat_<double>(3, 1));
				cv::Mat screen = (cv::Mat_<double>(3, 1) << marker_center[i][0], marker_center[i][1], 1);

				distanceY += tmatrix.at<double>(0, 0) * kx * MARKER_SIZE;
				distanceX += tmatrix.at<double>(1, 0) * ky * MARKER_SIZE;
				distanceZ += tmatrix.at<double>(2, 0) * kz * MARKER_SIZE;

			}


            //平均値を出す
            distanceX /= marker_ids.size();
			distanceY /= marker_ids.size();
			distanceZ /= marker_ids.size();

			ave_angleX /= marker_ids.size();
			ave_angleY /= marker_ids.size();
			ave_angleZ /= marker_ids.size();

			// printf("%c\n" , serialGetchar(fd) );
			// fflush(stdin);

			for(int i=0; i<marker_ids.size(); i++){
				marker_ids[i] = 0;
			}
		}

		flip(image, mirror_image, 1);

		//取得した値の表示

		cv::imshow("out(Mirror)", /*mirror_*/image);  

		/*
		cv::namedWindow("angle", cv::WINDOW_AUTOSIZE);
		
		
		char value_x[100];
		char value_y[100];
		char value_z[100];
		sprintf(value_x, "angle X =%f", ave_angleX);
		sprintf(value_y, "angle Y =%f", ave_angleY);
		sprintf(value_z, "angle Z =%f", ave_angleZ);
		putText(backimage, value_x, Point(50, 100), FONT_HERSHEY_SIMPLEX, 1.2, Scalar(100, 200, 100), 2);
		putText(backimage, value_y, Point(50, 150), FONT_HERSHEY_SIMPLEX, 1.2, Scalar(100, 200, 100), 2);
		putText(backimage, value_z, Point(50, 200), FONT_HERSHEY_SIMPLEX, 1.2, Scalar(100, 200, 100), 2);
		
		char value2_x[100];
		char value2_y[100];
		char value2_z[100];
		sprintf(value2_x, "distance X =%f cm", distanceX);
		sprintf(value2_y, "distance Y =%f cm", distanceY);
		sprintf(value2_z, "distance Z =%f cm", distanceZ);
		putText(backimage, value2_x, Point(50, 250), FONT_HERSHEY_SIMPLEX, 1.2, Scalar(100, 200, 100), 2);
		putText(backimage, value2_y, Point(50, 300), FONT_HERSHEY_SIMPLEX, 1.2, Scalar(100, 200, 100), 2);
		putText(backimage, value2_z, Point(50, 350), FONT_HERSHEY_SIMPLEX, 1.2, Scalar(100, 200, 100), 2);
		

		cv::imshow("angle", backimage);
		*/

		Recive_Data(fd);

		switch (mode) {
			case NORMAL:
				break;
			case SETTING_TURN_RIGHT_90:
				if (!Positioning(distanceX, distanceY)) {
					//if (!Turn(RIGHT, ave_angleZ, t_angle)) {
						p_time = time(nullptr);
						mode = GO_RIGHT;
						cout << "Change mode : SETTING_TURN_RIGHT_90  GO_RIGHT" << endl;
					//}
				}
				Send_Data(fd); // 送信
				break;
			case SETTING_TURN_LEFT_90:
				if (!Positioning(distanceX, distanceY)) {
					//if (Turn(LEFT, ave_angleZ, t_angle)) {
						p_time = time(nullptr);
						mode = GO_LEFT;
						cout << "Change mode :  SETTING_TURN_LEFT_90  GO_LEFT" << endl;
					//}
				}
				Send_Data(fd); // 送信
				break;
			case SETTING_STRAIGHT:
				if (!Positioning(distanceX, distanceY)) {
					//if (Turn(LEFT, ave_angleZ, t_angle)) {
						p_time = time(nullptr);
						mode = GO_STRAIGHT;
						cout << "Change mode :  SETTING_STRAIGHT  GO_STRAIGHT" << endl;
					//}
				}
				Send_Data(fd); // 送信
				break;
			case SETTING_BACK:
				if (!Positioning(distanceX, distanceY)) {
					//if (Turn(LEFT, ave_angleZ, t_angle)) {
						p_time = time(nullptr);
						mode = GO_BACK;
						cout << "Change mode :  SETTING_BACK  GO_BACK" << endl;
					//}
				}
				Send_Data(fd); // 送信
				break;
			case MANUAL:
				break;
			case GO_STRAIGHT:
				if (!GoStraight(marker_ids.size())) {
					mode = NORMAL;
					cout << "Change mode : GO_STRAIGHT  NORMAL" << endl;
				}
				Send_Data(fd); // 送信
				break;
			case GO_RIGHT:
				if (!GoRight(marker_ids.size())) {
					mode = NORMAL;
					cout << "Change mode : GO_RIGHT  NORMAL" << endl;
				}
				Send_Data(fd); // 送信
				break;
			case GO_LEFT:
				if (!GoLeft(marker_ids.size())) {
					mode = NORMAL;
					cout << "Change mode : GO_LEFT  NORMAL" << endl;
				}
				Send_Data(fd); // 送信
				break;
			case GO_BACK:
				if (!GoBack(marker_ids.size())) {
					mode = NORMAL;
					cout << "Change mode : GO_BACK  NORMAL" << endl;
				}
				Send_Data(fd); // 送信
				break;
			case WAIT:
				break;
		}

		distanceX = 0;
		distanceY = 0;
		distanceZ = 0;

		ave_angleX = 0;
		ave_angleY = 0;
		ave_angleZ = 0;
	}

	cv::waitKey(0);

	//Serial.close();

	return 0;
}

void Not_moter_Outlier() {
	// 異常値を弾く
	if (motor.Front_Right_Motor < -MAX_SIDE_MOTOR_POWER  || motor.Front_Left_Motor < -MAX_SIDE_MOTOR_POWER) {
		motor.Front_Right_Motor = 0;
		motor.Front_Left_Motor = 0;
	}
	else if (MAX_SIDE_MOTOR_POWER < motor.Front_Right_Motor || MAX_SIDE_MOTOR_POWER < motor.Front_Left_Motor) {
		motor.Front_Right_Motor = 0;
		motor.Front_Left_Motor = 0;
	}

	if (motor.Back_Right_Motor < -MAX_SIDE_MOTOR_POWER  || motor.Back_Left_Motor < -MAX_SIDE_MOTOR_POWER) {
		motor.Back_Right_Motor = 0;
		motor.Back_Left_Motor = 0;
	}
	else if (MAX_SIDE_MOTOR_POWER < motor.Back_Right_Motor || MAX_SIDE_MOTOR_POWER < motor.Back_Left_Motor) {
		motor.Back_Right_Motor = 0;
		motor.Back_Left_Motor = 0;
	}
}


void motor_average_5(int FR, int FL, int BR, int BL) {
	if (5 <= motor_count) {
		motor_count = 0;
	}

	if (motor.Front_Right_Motor == 0) {
		FR += ADJUST_FR;
	}
	if (motor.Front_Left_Motor == 0) {
		FL += ADJUST_FL;
	}
	if (motor.Back_Right_Motor == 0) {
		BR += ADJUST_BR;
	}
	if (motor.Back_Left_Motor == 0) {
		BL += ADJUST_BL;
	}

	motor_FL_Sum -= motor_FL_Data[motor_count];
	motor_FR_Sum -= motor_FR_Data[motor_count];
	motor_BL_Sum -= motor_BL_Data[motor_count];
	motor_BR_Sum -= motor_BR_Data[motor_count];
	motor_FL_Data[motor_count] = FL;
	motor_FR_Data[motor_count] = FR;
	motor_BL_Data[motor_count] = BL;
	motor_BR_Data[motor_count] = BR;
	motor_FL_Sum += motor_FL_Data[motor_count];
	motor_FR_Sum += motor_FR_Data[motor_count];
	motor_BL_Sum += motor_BL_Data[motor_count];
	motor_BR_Sum += motor_BR_Data[motor_count];
	++motor_count;	

	motor.Front_Right_Motor = motor_FR_Sum / 5;
	motor.Front_Left_Motor = motor_FL_Sum / 5;
	motor.Back_Right_Motor = motor_BR_Sum / 5;
	motor.Back_Left_Motor = motor_BL_Sum / 5;
}

void Reset_average_5() {
	for (int i = 0; i < 5; ++i) {
		motor_FL_Data[i] = 0;
		motor_FR_Data[i] = 0;
		motor_BL_Data[i] = 0;
		motor_BR_Data[i] = 0;
		motor_FL_Sum = 0;
		motor_FR_Sum = 0;
		motor_BL_Sum = 0;
		motor_BR_Sum = 0;
		motor_count = 0;
	}
}


int Map(int value, float min_a, float max_a, float min_b, float max_b) {
	float ret = (value - min_a) / (max_a - min_a);
	ret *= (max_b - min_b);
	ret += min_b;

	return ret;
}

void motor_map(float min, float max) {
	if (0 < motor.Front_Left_Motor)  {
		motor.Front_Left_Motor = Map(motor.Front_Left_Motor, 0, max, min, max);
	}
	else if (motor.Front_Left_Motor < 0) {
		motor.Front_Left_Motor = Map(motor.Front_Left_Motor, -max, 0, -max, -min);
	}

	if (0 < motor.Front_Right_Motor) {
		motor.Front_Right_Motor = Map(motor.Front_Right_Motor, 0, max, min, max);
	}
	else if (motor.Front_Right_Motor < 0) {
		motor.Front_Right_Motor = Map(motor.Front_Right_Motor, -max, 0, -max, -min);
	}
	
	if (0 < motor.Back_Left_Motor)  {
		motor.Back_Left_Motor = Map(motor.Back_Left_Motor, 0, max, min, max);
	}
	else if (motor.Back_Left_Motor < 0) {
		motor.Back_Left_Motor = Map(motor.Back_Left_Motor, -max, 0, -max, -min);
	}

	if (0 < motor.Back_Right_Motor) {
		motor.Back_Right_Motor = Map(motor.Back_Right_Motor, 0, max, min, max);
	}
	else if (motor.Back_Right_Motor < 0) {
		motor.Back_Right_Motor = Map(motor.Back_Right_Motor, -max, 0, -max, -min);
	}
}


int readMatrix(const char* filename, cv::Mat& cameraMat, cv::Mat& distCoeffs) {
	try{
		// (2)open file storage
		cv::FileStorage cvfs(filename, cv::FileStorage::READ);

		/***************************************************************
		// (3)read data from file storage
		cv::FileNode node(cvfs.fs, NULL); // Get Top Node
		read(node["cameraMatrix"], cameraMat);
		read(node["distCoeffs"], distCoeffs);
		*******************************************************************/
	}
	catch (std::exception e) {
		return -1;
	}

	return 0;
}

// 位置調整
bool Positioning_X(double now_x) {
	double diff_x = now_x - CENTER_X;

	motor.Front_Right_Motor = 0;
	motor.Front_Left_Motor = 0;
	motor.Back_Right_Motor = 0;
	motor.Back_Left_Motor = 0;

	
	if (abs(diff_x) < MAX_DIFF) {
		return false;
	}
	// x方向のズレ補正
	else if (MAX_DIFF < abs(diff_x)) {
		if (xy_flag == Y) {
			xy_flag = X;
			Reset_average_5();
		}
		// 中心より右に位置している場合
		if (diff_x < 0) {
			// 左に動かす
			motor.Front_Right_Motor -= int(MAX_SIDE_MOTOR_POWER * abs(diff_x) / (CENTER_X - MIN_X));
			motor.Front_Left_Motor  += int(MAX_SIDE_MOTOR_POWER * abs(diff_x) / (CENTER_X - MIN_X));
			motor.Back_Right_Motor += int(MAX_SIDE_MOTOR_POWER * abs(diff_x) / (CENTER_X - MIN_X));
			motor.Back_Left_Motor  -= int(MAX_SIDE_MOTOR_POWER * abs(diff_x) / (CENTER_X - MIN_X));
		
		}
		// 中心より左に位置している場合
		else if (0 < diff_x) {
			// 右に動かす
			motor.Front_Right_Motor  += int(MAX_SIDE_MOTOR_POWER * abs(diff_x) / (MAX_X - CENTER_X));
			motor.Front_Left_Motor -= int(MAX_SIDE_MOTOR_POWER * abs(diff_x) / (MAX_X - CENTER_X));
			motor.Back_Right_Motor  -= int(MAX_SIDE_MOTOR_POWER * abs(diff_x) / (MAX_X - CENTER_X));
			motor.Back_Left_Motor += int(MAX_SIDE_MOTOR_POWER * abs(diff_x) / (MAX_X - CENTER_X));
		}
	}

	Not_moter_Outlier();

	motor_map(MIN_SIDE_MOTOR_POWER, MAX_SIDE_MOTOR_POWER);

	motor_average_5(motor.Front_Right_Motor, motor.Front_Left_Motor, motor.Back_Right_Motor, motor.Back_Left_Motor);

	return true;

}

// 位置調整
bool Positioning_Y(double now_y) {
	double diff_y = CENTER_Y - now_y;

	motor.Front_Right_Motor = 0;
	motor.Front_Left_Motor = 0;
	motor.Back_Right_Motor = 0;
	motor.Back_Left_Motor = 0;

	// 補正なし
	if (abs(diff_y) < MAX_DIFF) {
		return false;
	}
	// y方向のズレ補正
	else if (MAX_DIFF < abs(diff_y)) {
		if (xy_flag == X) {
			xy_flag = Y;
			Reset_average_5();
		}

		// 中心より後ろに位置している場合
		if (diff_y < 0) {
			// 前に動かす
			motor.Front_Left_Motor += int(MAX_MOTOR_POWER * abs(diff_y) / (CENTER_Y - MIN_Y));
			motor.Back_Left_Motor += int(MAX_MOTOR_POWER * abs(diff_y) / (CENTER_Y - MIN_Y));
			motor.Front_Right_Motor += int(MAX_MOTOR_POWER * abs(diff_y) / (CENTER_Y - MIN_Y));
			motor.Back_Right_Motor += int(MAX_MOTOR_POWER * abs(diff_y) / (CENTER_Y - MIN_Y));
		}
		// 中心より前に位置している場合
		else if (0 < diff_y) {
			// 後ろに動かす
			motor.Front_Left_Motor -= int(MAX_MOTOR_POWER * abs(diff_y) / (MAX_Y - CENTER_Y));
			motor.Back_Left_Motor -= int(MAX_MOTOR_POWER * abs(diff_y) / (MAX_Y - CENTER_Y));
			motor.Front_Right_Motor -= int(MAX_MOTOR_POWER * abs(diff_y) / (MAX_Y - CENTER_Y));
			motor.Back_Right_Motor -= int(MAX_MOTOR_POWER * abs(diff_y) / (MAX_Y - CENTER_Y));
		}
	}

	Not_moter_Outlier();

	motor_map(MIN_MOTOR_POWER, MAX_MOTOR_POWER);

	motor_average_5(motor.Front_Right_Motor, motor.Front_Left_Motor, motor.Back_Right_Motor, motor.Back_Left_Motor);

	return true;
}

bool Positioning(double now_x, double now_y) {
	if (Positioning_Y(now_y)) {
		return true;
	}
	else if (Positioning_X(now_x)) {
		return true;
	}

	return false;
}

// 送信関数
void Send_Data(int fd) {
	/* 送信処理 */
	serialPutchar(fd,'H');

	unsigned char power = motor.Front_Right_Motor + 100; // 0 ~ 200に補正
	serialPutchar(fd, power); 
	//printf("FR %d  ", power - 100);


	power = motor.Front_Left_Motor + 100;
	serialPutchar(fd, power);
	//printf("FL %d  ", power - 100);

	power = motor.Back_Right_Motor + 100;
	serialPutchar(fd, power);
	//printf("BR %d  ", power - 100);

	power = motor.Back_Left_Motor + 100;
	serialPutchar(fd, power);
	//printf("BL %d  ", power - 100);

	//printf(" send \n");

} 

// 受信関数
void Recive_Data(int fd) {
	if (0 < serialDataAvail(fd)) {
		uint8_t h = serialGetchar(fd);

		if (h == 'M') {
			mode = NORMAL;
			serialFlush(fd);
			cout << "Change mode : MANUAL Button Pushed" << endl;
		}
	}
}


double Target_Angle(RL rl, double now_angle, int angle) {
	double target_angle; // 目標角度

	if (rl == RIGHT) {
		angle *= -1;
	}

	target_angle = now_angle + angle;

	if (target_angle < -180) {
		target_angle += 360;
	}
	else if (180 < target_angle) {
		target_angle -= 360;
	}

	return target_angle;
}


// 旋回関数
bool Turn(RL rl, double now_angle, double target_angle) {
	int diff = target_angle - now_angle; // 目標までの差

	motor.Front_Right_Motor = 0;
	motor.Front_Left_Motor = 0;
	motor.Back_Right_Motor = 0;
	motor.Back_Left_Motor = 0;

	// 誤差10度まで許容
	if (diff < -3 || 3 < diff) {

		if (rl == RIGHT) {
			motor.Front_Right_Motor += MIN_TURN_MOTOR_POWER;
			motor.Front_Left_Motor -= MIN_TURN_MOTOR_POWER;
			motor.Back_Right_Motor += MIN_TURN_MOTOR_POWER;
			motor.Back_Left_Motor -= MIN_TURN_MOTOR_POWER;
		}
		else if (rl == LEFT) {
			motor.Front_Right_Motor -= MIN_TURN_MOTOR_POWER;
			motor.Front_Left_Motor += MIN_TURN_MOTOR_POWER;
			motor.Back_Right_Motor -= MIN_TURN_MOTOR_POWER;
			motor.Back_Left_Motor += MIN_TURN_MOTOR_POWER;
		}
		return true;
	}
	else {
		return false;
	}
}

// 直進関数
bool GoStraight(int marker_num) {
	motor.Front_Right_Motor = MAX_SIDE_MOTOR_POWER;
	motor.Front_Left_Motor = MAX_SIDE_MOTOR_POWER;
	motor.Back_Right_Motor = MAX_SIDE_MOTOR_POWER;
	motor.Back_Left_Motor = MAX_SIDE_MOTOR_POWER;


	n_time = time(nullptr);

	if (0 < marker_num && 5 < n_time - p_time) {
		return false;
	}

	return true;
}

// 右直進関数
bool GoRight(int marker_num) {
	motor.Front_Right_Motor = MAX_SIDE_MOTOR_POWER;
	motor.Front_Left_Motor = -MAX_SIDE_MOTOR_POWER;
	motor.Back_Right_Motor = -MAX_SIDE_MOTOR_POWER;
	motor.Back_Left_Motor = MAX_SIDE_MOTOR_POWER;

	n_time = time(nullptr);

	if (0 < marker_num && 5 < n_time - p_time) {
		return false;
	}

	return true;
}

// 左直進関数
bool GoLeft(int marker_num) {
	motor.Front_Right_Motor = -MAX_SIDE_MOTOR_POWER;
	motor.Front_Left_Motor = MAX_SIDE_MOTOR_POWER;
	motor.Back_Right_Motor = MAX_SIDE_MOTOR_POWER;
	motor.Back_Left_Motor = -MAX_SIDE_MOTOR_POWER;

	n_time = time(nullptr);

	if (0 < marker_num && 5 < n_time - p_time) {
		return false;
	}

	return true;
} 

// 後直進関数
bool GoBack(int marker_num) {
	motor.Front_Right_Motor = -MAX_SIDE_MOTOR_POWER;
	motor.Front_Left_Motor = -MAX_SIDE_MOTOR_POWER;
	motor.Back_Right_Motor = -MAX_SIDE_MOTOR_POWER;
	motor.Back_Left_Motor = -MAX_SIDE_MOTOR_POWER;

	n_time = time(nullptr);

	if (0 < marker_num && 5 < n_time - p_time) {
		return false;
	}

	return true;
}
