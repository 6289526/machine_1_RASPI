// marker_detect_1.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <string>
#include <thread>

//#include "serial.h"
//#include "Safe_Network_tcp.h"

using namespace std;
using namespace cv;

const int MAX_MARKER_NUM = 1; // 一度に読めるマーカーの数
const double MARKER_SIZE = 100;  //マーカーの縦の長さをmmで指定

// 画面内に収まるマーカの範囲を指定
const double MIN_X = -9.00;
const double MAX_X = 50.8;
const double MIN_Y = -16.0;
const double MAX_Y = 54.4;

const double CENTER_X = (MIN_X + MAX_X) / 2;
const double CENTER_Y = (MIN_Y + MAX_Y) / 2;
const double MAX_DIFF = 10; // 許容範囲

const int MAX_MOTOR_POWER = 16; // 最大出力 正方向
const int MIN_MOTOR_POWER = 10; // 最小出力　正方向

int readMatrix(const char* filename, cv::Mat& cameraMat, cv::Mat& distCoeffs);
int CalibrationCamera(VideoCapture& cap, cv::Mat& cameraMat, cv::Mat& distCoeffs);

typedef enum RL{
	RIGHT,
	LEFT
} RL;

typedef enum Mode {
	NORMAL, // 通常
	SETTING_TURN_RIGHT_90, // 旋回設定
	SETTING_TURN_LEFT_90,
	MANUAL, // 手動
	STRAIGHT, // 直進
	TURN_RIGHT, // 旋回
	TURN_LEFT, // 旋回
	WAIT, // 待ち
} Mode;

typedef struct Motor_Power {
	int Right_Motor;
	int Left_Motor;
} Moter_Power;

int motor_L_Data[5] = {0, 0, 0, 0, 0};
int motor_R_Data[5] = {0, 0, 0, 0, 0};
int motor_count = 0;
int motor_L_Sum = 0;
int motor_R_Sum = 0;

void moter_average_5(int R, int L);
int Map(int value, float min_a, float max_a, float min_b, float max_b);
void motor_map();
void Positioning_X(double now_x); // 位置調整
void Positioning_Y(double now_y); // 位置調整
void Send_Data(int fd); // 送信関数
double Target_Angle(RL rl, double now_angle, int angle); // 目標角度取得関数
void Turn(RL rl, double now_angle, double target_angle); // 旋回関数
void Straight(); // 直進関数

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
bool first_flag = true;

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

		// マーカなしならモードリセット
		if (marker_ids.size() == 0) {
			mode = NORMAL;
			motor.Right_Motor = 0;
			motor.Left_Motor = 0;

			if (first_flag) {
				motor.Right_Motor = 0;
				motor.Left_Motor = 0;
				Send_Data(fd); // 送信
				first_flag = false;
			}
		}
		else {
			first_flag = true;
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
				}

				break;

			case 18:
				all_marker_center[i][0] = marker_location[i][6] - 0.6071 * (marker_center[i][0] - marker_location[i][6]);
				all_marker_center[i][1] = marker_location[i][7] + 0.5 * (marker_location[i][7] - marker_center[i][1]);

				if (mode == NORMAL) {
					mode = SETTING_TURN_LEFT_90;
				}

				break;

			case 43:
				all_marker_center[i][0] = marker_location[i][0] - 0.6071 * (marker_center[i][0] - marker_location[i][0]);
				all_marker_center[i][1] = marker_location[i][1] - 0.5 * (marker_center[i][1] - marker_location[i][1]);
				break;

			case 27:
				all_marker_center[i][0] = marker_location[i][2] + 0.6071 * (marker_location[i][2] - marker_center[i][0]);
				all_marker_center[i][1] = marker_location[i][3] - 0.5 * (marker_center[i][1] - marker_location[i][3]);
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

		switch (mode) {
			case NORMAL:
				break;
			case SETTING_TURN_RIGHT_90:
				t_angle = Target_Angle(RIGHT, ave_angleZ, 90); // 右回転90度旋回設定
				mode = TURN_RIGHT;
				break;
			case SETTING_TURN_LEFT_90:
				t_angle = Target_Angle(LEFT, ave_angleZ, 90); // 左回転90度旋回設定
				mode = TURN_LEFT;
				break;
			case MANUAL:
				break;
			case STRAIGHT:
				break;
			case TURN_RIGHT:
				//Turn(RIGHT, ave_angleZ, t_angle); // 設定された右回転
				Positioning_Y(distanceY);
				Send_Data(fd); // 送信
				break;
			case TURN_LEFT:
				//Turn(LEFT, ave_angleZ, t_angle); // 設定された左回転
				Positioning_Y(distanceY);
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

void motor_average_5(int R, int L) {
	if (5 <= motor_count) {
		motor_count = 0;
	}

	// 正回転補正
	if (0 < motor.Right_Motor) {
		R += 4;
	}
	if (0 < motor.Left_Motor) {
		L += 4;
	}

	motor_L_Sum -= motor_L_Data[motor_count];
	motor_R_Sum -= motor_R_Data[motor_count];
	motor_L_Data[motor_count] = L;
	motor_R_Data[motor_count] = R;
	motor_L_Sum += motor_L_Data[motor_count];
	motor_R_Sum += motor_R_Data[motor_count];
	++motor_count;	

	motor.Right_Motor = motor_R_Sum / 5;
	motor.Left_Motor = motor_L_Sum / 5;
}


int Map(int value, float min_a, float max_a, float min_b, float max_b) {
	float ret = (value - min_a) / (max_a - min_a);
	ret *= (max_b - min_b);
	ret += min_b;

	return ret;
}

void motor_map() {
	if (0 < motor.Left_Motor)  {
		motor.Left_Motor = Map(motor.Left_Motor, 0, MAX_MOTOR_POWER, MIN_MOTOR_POWER, MAX_MOTOR_POWER);
	}
	else if (motor.Left_Motor < 0) {
		motor.Left_Motor = Map(motor.Left_Motor, -MAX_MOTOR_POWER, 0, -MAX_MOTOR_POWER, -MIN_MOTOR_POWER);
	}

	if (0 < motor.Right_Motor) {
		motor.Right_Motor = Map(motor.Right_Motor, 0, MAX_MOTOR_POWER, MIN_MOTOR_POWER, MAX_MOTOR_POWER);
	}
	else if (motor.Right_Motor < 0) {
		motor.Right_Motor = Map(motor.Right_Motor, -MAX_MOTOR_POWER, 0, -MAX_MOTOR_POWER, -MIN_MOTOR_POWER);
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
void Positioning_X(double now_x) {
	double diff_x = now_x - CENTER_X;

	motor.Right_Motor = 0;
	motor.Left_Motor = 0;

	// x方向のズレ補正
	if (MAX_DIFF < abs(diff_x)) {
		// 中心より左に位置している場合
		if (diff_x < 0) {
			// 右に動かす
			motor.Left_Motor = int(MAX_MOTOR_POWER * abs(diff_x) / (CENTER_X - MIN_X));
		}
		// 中心より右に位置している場合
		else if (0 < diff_x) {
			// 左に動かす
			motor.Right_Motor = int(MAX_MOTOR_POWER * abs(diff_x) / (MAX_X - CENTER_X));
		}
	}
}

// 位置調整
void Positioning_Y(double now_y) {
	double diff_y = CENTER_Y - now_y;

	motor.Right_Motor = 0;
	motor.Left_Motor = 0;

	// y方向のズレ補正
	if (MAX_DIFF < abs(diff_y)) {
		// 中心より後ろに位置している場合
		if (diff_y < 0) {
			// 前に動かす
			motor.Left_Motor += int(MAX_MOTOR_POWER * abs(diff_y) / (CENTER_Y - MIN_Y));
			motor.Right_Motor += int(MAX_MOTOR_POWER * abs(diff_y) / (CENTER_Y - MIN_Y));
		}
		// 中心より前に位置している場合
		else if (0 < diff_y) {
			// 後ろに動かす
			motor.Left_Motor -= int(MAX_MOTOR_POWER * abs(diff_y) / (MAX_Y - CENTER_Y));
			motor.Right_Motor -= int(MAX_MOTOR_POWER * abs(diff_y) / (MAX_Y - CENTER_Y));
		}
	}

	// 異常値を弾く
	if (motor.Right_Motor < -MAX_MOTOR_POWER  || motor.Left_Motor < -MAX_MOTOR_POWER) {
		motor.Right_Motor = 0;
		motor.Left_Motor = 0;
	}
	else if (MAX_MOTOR_POWER < motor.Right_Motor || MAX_MOTOR_POWER < motor.Left_Motor) {
		motor.Right_Motor = 0;
		motor.Left_Motor = 0;
	}

	motor_map(); // 最小値を設定

	motor_average_5(motor.Right_Motor, motor.Left_Motor);
}


// 送信関数
void Send_Data(int fd) {
	/* 送信処理 */
	serialPutchar(fd,'H');

	unsigned char power = motor.Right_Motor + 100; // 0 ~ 200に補正
	serialPutchar(fd, power); 
	printf("%d ", power - 100);


	power = motor.Left_Motor + 100;
	serialPutchar(fd, power);
	printf("%d", power - 100);

	printf(" send \n");
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
void Turn(RL rl, double now_angle, double target_angle) {
	int diff = target_angle - now_angle; // 目標までの差

	// 誤差10度まで許容
	if (diff < -10 || 10 < diff) {
		diff = target_angle - now_angle;
		cout << "あと  "  << diff << endl; 

		if (rl == RIGHT) {
			motor.Right_Motor = -15;
			motor.Left_Motor = 15;
		}
		else if (rl == LEFT) {
			motor.Right_Motor = 15;
			motor.Left_Motor = -15;
		}
	}
	else {
		cout << "終了" << endl;
		mode = WAIT;
	}
}

// 直進関数
void Straight() {

}
