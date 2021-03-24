// marker_detect_1.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。


/// 通信関係のコメントアウト
//// 文字出力をコメントアウト

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>

#include <thread>

#include "serial.h"
#include "Safe_Network_tcp.h"

using namespace std;
using namespace cv;

int readMatrix(const char* filename, cv::Mat& cameraMat, cv::Mat& distCoeffs);
int CalibrationCamera(VideoCapture& cap, cv::Mat& cameraMat, cv::Mat& distCoeffs);
double marker_size = 61;  //マーカーの縦の長さをmmで指定
double kz = 3.1829046 / 3;  //実測値cmと、このパソコンの長さの調整係数
double kx = 2.06589452;
double ky = kx;

int count_frame = 0;
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
Send_Data send_data = {};

/// Safe_Server server_tcp;

int main(int argc, const char* argv[])
{
	
	cv::Mat drone_image;
	//背景画像
	cv::Mat dstImg;
	cv::Mat backimage;

	cv::Mat image;
	cv::Mat cameraMatrix;// = (cv::Mat_<double>(3, 3) << 4.50869314e+02, 0, 2.47413306e+02, 0, 4.55471466e+02, 1.85222260e+02, 0, 0, 1);
	cv::Mat distCoeffs;// = (cv::Mat_<double>(1, 5) << 4.65341144e-01, -2.18649613e+00, 5.14274386e-03, 3.62427799e-03, 3.71240124e+00);
	readMatrix("caliburation.xml",cameraMatrix, distCoeffs);  //xmlファイルからcameraMatrixとdistCoeffsを読み込む
	//dictionary生成
	const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

	//マーカーの検出
	std::vector<int> marker_ids;
	std::vector<std::vector<cv::Point2f>> marker_corners;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

	drone_image = cv::imread("drone.png", 1);
	if (drone_image.data == NULL) return -1;

	/// server_tcp.init(50200);
	/// server_tcp.start_accept(false);

	/// thread th_server([&] {server_tcp.run(); });

	VideoCapture cap(0);  //カメラの映像の読み込み
    cap.set(3, 1280);
	cap.set(4, 720);

	Mat mirror_image;  //表示画面の左右反転の用意

	int key = 0;

	double angleX[6] = { 0,0,0,0,0,0 };
	double angleY[6] = { 0,0,0,0,0,0 };
	double angleZ[6] = { 0,0,0,0,0,0 };
	double ave_angleX = 0;
	double ave_angleY = 0;
	double ave_angleZ = 0;

	double distanceX = 0;
	double distanceY = 0;
	double distanceZ = 0;
	double distanceR = 0;

	while ((key = cv::waitKey(1)) != 'q') {  //qが押されるまで繰り返す

		/// cout << server_tcp.is_open(0) << endl;
		//背景画像の読み込み
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

		if (key == 'c') {  //cが押されたときにキャリブレーションモードにする
			CalibrationCamera(cap, cameraMatrix, distCoeffs);
		}
		if (key == 'h')  //hで保存
		imwrite("imgwww.png", image);

		//マーカーの検知
		cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, parameters);
		//マーカーの描画
		cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);


		//中心等の座標の取得
		float marker_location[6][8] = {};
		float marker_center[6][2] = {};
		float all_marker_center[6][2] = {};
		float average_center[2] = {};

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
		int z = 0;
		for (int i = 0; i < marker_ids.size(); i++) {
			switch (marker_ids[i])
			{
			case 42:
				all_marker_center[i][0] = marker_location[i][4] + 0.6071 * (marker_location[i][4] - marker_center[i][0]);
				all_marker_center[i][1] = marker_location[i][5] + 0.5 * (marker_location[i][5] - marker_center[i][1]);
				z++;
				break;

			case 18:
				all_marker_center[i][0] = marker_location[i][6] - 0.6071 * (marker_center[i][0] - marker_location[i][6]);
				all_marker_center[i][1] = marker_location[i][7] + 0.5 * (marker_location[i][7] - marker_center[i][1]);
				z++;
				break;

			case 43:
				all_marker_center[i][0] = marker_location[i][0] - 0.6071 * (marker_center[i][0] - marker_location[i][0]);
				all_marker_center[i][1] = marker_location[i][1] - 0.5 * (marker_center[i][1] - marker_location[i][1]);
				z++;
				break;

			case 27:
				all_marker_center[i][0] = marker_location[i][2] + 0.6071 * (marker_location[i][2] - marker_center[i][0]);
				all_marker_center[i][1] = marker_location[i][3] - 0.5 * (marker_center[i][1] - marker_location[i][3]);
				z++;
				break;
			}
			average_center[0] = average_center[0] + all_marker_center[i][0];
			average_center[1] = average_center[1] + all_marker_center[i][1];
		}

		average_center[0] = average_center[0] / z;  //x座標
		average_center[1] = average_center[1] / z;  //y座標


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
			cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
			for (int i = 0; i < marker_ids.size(); i++)
			{
				cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
				
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

					distanceX += tmatrix.at<double>(0, 0) * kx * marker_size;
					distanceY += tmatrix.at<double>(1, 0) * ky * marker_size;
					distanceZ += tmatrix.at<double>(2, 0) * kz * marker_size;
			}


            //平均値を出す
            distanceX /= marker_ids.size();
			distanceY /= marker_ids.size();
			distanceZ /= marker_ids.size();

			ave_angleX /= marker_ids.size();
			ave_angleY /= marker_ids.size();
			ave_angleZ /= marker_ids.size();

		}
		 
	    //サーボモーターを動かす角度を調べる
		
		char finish = '\n';
        if(marker_ids.size() > 0) {
			if(count_frame % 30 == 0){
			    
                if(atan((360 - average_center[1]) / (average_center[0] - 640)) / (3.14 * 2) * 360 > 90){
					Servo_angle += 180 - (atan((360 - average_center[1]) / (average_center[0] - 640)) / (3.14 * 2) * 360) ;
				}else {
					Servo_angle += atan((360 - average_center[1]) / (average_center[0] - 640)) / (3.14 * 2) * 360;
				}
				
				if(Servo_angle >= 180){
					Servo_angle -= 180;
				}else if(Servo_angle <= 0){
					Servo_angle += 180;
				}
				
				sprintf(Servo_angle_char, "%.0f", Servo_angle); 
			}
		}
		count_frame++;


        
		sprintf(value_Servo, "value=%.0f", Servo_angle); //変数の値も含めた表示したい文字列をchar型変数に格納

		//真ん中に円を描画
		circle(image, Point(average_center[0], average_center[1]), 10, Scalar(255, 0, 0), 3, 4);

		flip(image, mirror_image, 1);

    	//// putText(/*mirror_*/image, /*valuex_i*/Servo_angle_char, Point(50, 50), FONT_HERSHEY_SIMPLEX, 1.2, Scalar(100, 200, 100), 2);

		//取得した値の表示

		cv::imshow("out(Mirror)", /*mirror_*/image);  

		cv::namedWindow("angle", cv::WINDOW_AUTOSIZE);

		////
		/*
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
		*/
		////
		cv::imshow("angle", backimage);

        int drone_angle = 0;
		int drone_angleX = 0;
		int drone_angleY = 0;
		int drone_angleZ = 0;
		int drone_distanceX = 0;
		int drone_distanceY = 0;
		int drone_distanceZ = 0;

        //操縦パソコンに送る値のケイサン
		//地上機体との相対的な位置を計算する
	    //サーボモーターを動かさない場合
		drone_distanceX = distanceX;
		drone_distanceY = distanceY;
		drone_distanceZ = distanceZ;

        drone_angleX = ave_angleX;
		drone_angleY = ave_angleY;
		drone_angleZ = ave_angleZ;

		//操縦パソコンに送る値
		send_data.drone_distanceX = drone_distanceX;
		send_data.drone_distanceY = drone_distanceY;
		send_data.drone_distanceZ = drone_distanceZ;
        send_data.drone_angleX = drone_angleX;
		send_data.drone_angleY = drone_angleY;
		send_data.drone_angleZ = drone_angleZ;

		/// if(server_tcp.is_open(0)){
		/// 	server_tcp.send(0, send_data);

		/// 	char dummy;
		/// 	if(server_tcp.available(0) >= 1)
		/// 		server_tcp.read(0, &dummy);
		/// }else{
		/// 	ave_angleX = 0;
		/// 	ave_angleY = 0;
		/// 	ave_angleZ = 0;

		/// 	distanceX = 0;
		/// 	distanceY = 0;
		/// 	distanceZ = 0;
		/// 	distanceR = 0;
		/// }
	}
	cv::waitKey(0);

	//Serial.close();
	/// server_tcp.stop();
	///if(th_server.joinable())
	///	th_server.join();

	return 0;
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

int CalibrationCamera(VideoCapture& cap, cv::Mat& cameraMat, cv::Mat& distCoeff) {

	//std::cout << "Hello World!\n"; 
	const int BOARD_W = 10;  //チェスボードの横方向コーナーの数
	const int BOARD_H = 7;  //チェスボードの縦方向コーナーの数
	const Size BOARD_SIZE = Size(BOARD_W, BOARD_H);
	const int N_CORNERS = BOARD_W * BOARD_H;
	int N_BOARDS = 0;  //チェスボードの撮影枚数
	const float SCALE = 24;   //チェスボードの正方形の辺の長さ(mm)
	Size IM_SIZE;// = Size(512, 384); //撮影写真のピクセル数
	double rms;
	Mat src_image;

	if (!cap.isOpened())
		return -1;

	IM_SIZE.width = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
	IM_SIZE.height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);

	Mat cameraMatrix;
	Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	Mat dst_image;
	Mat mirror_image;
	while (cv::waitKey(1) != 'q') {  //qを押すまでキャリブレーションモード
		cap >> src_image;  //画像の読み込み

		if (src_image.empty())
			continue;

		//左右反転画像の表示
		flip(src_image, mirror_image, 1);
		imshow("Source(Mirror)", mirror_image);  

		//ーーーimagePointsの取得ーーー
		vector<vector<Point2f>>imagePoints;
		vector<Point2f> imageCorners; //このvectorがimagePoints
		Mat gray_image;
		bool found;
		Mat dst_image;
		//ーーーコーナーの検出ーーー
		found = findChessboardCorners(src_image, BOARD_SIZE, imageCorners);

		if (!found)  //コーナーがないときとばす
			continue;

		//ーーー制度を高めるーーー
		cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);
		cornerSubPix(gray_image, imageCorners, Size(9, 9), Size(-1, -1), TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));
		////ーーーコーナーを描画するーーー
		dst_image = src_image.clone();
		drawChessboardCorners(dst_image, BOARD_SIZE, imageCorners, found);
		////ーーーコーナーを表示するーーー
		//namedWindow("コーナー検出画像　" + to_string(i));
		//imshow("コーナー検出画像　" + to_string(i), dst_image);
		////ーーーコーナーの座標を確認するーーー
		//for (int i = 0; i < N_CORNERS; i++)
		//	cout << i << "  " << (int)(imageCorners[i].x + 0.5) << "  " << (int)(imageCorners[i].y + 0.5) << endl;
		////ーーーimagePointsに書き込むーーー
		imagePoints.push_back(imageCorners);

		N_BOARDS++;

		//ーーーobjectPointsの設定ーーー
		vector<vector<Point3f>> objectPoints;
		vector<Point3f>objectCorners; //このvectorがobjectPoints
		for (int j = 0; j < BOARD_H; j++)
			for (int i = 0; i < BOARD_W; i++)
			{
				objectCorners.push_back(Point3f(i*SCALE, j*SCALE, 0.0f));
			}
		objectPoints.push_back(objectCorners);
		//ーーーカメラキャリブレーションを行うーーー
		rms = calibrateCamera(objectPoints, imagePoints, Size(IM_SIZE.width, IM_SIZE.height), cameraMatrix, distCoeffs, rvecs, tvecs);
	}
	if (cameraMatrix.empty())  //cameraMatrixがないときエラー
		return -1;

	//ーーーキャリブレーションの結果ーーー
	cout << fixed << right;
	cout << "Re_projection Error(unit: pixel)" << endl;
	cout << "  " << rms << endl;
	cout << endl;
	cout << "CameraMatrix(unit: pixel)" << endl;
	cout << "  fx=" << cameraMatrix.at<float>(0, 0);
	cout << "  fy=" << cameraMatrix.at<float>(1, 1);
	cout << "  cx=" << cameraMatrix.at<float>(0, 2);
	cout << "  cy=" << cameraMatrix.at<float>(1, 2);
	cout << endl << endl;
	cout << "distCoeffs" << endl;
	cout << "  k1=" << distCoeffs.at<float>(0, 0);
	cout << "  k2=" << distCoeffs.at<float>(0, 1);
	cout << "  p1=" << distCoeffs.at<float>(0, 2);
	cout << "  p2=" << distCoeffs.at<float>(0, 3);
	cout << "  p3=" << distCoeffs.at<float>(0, 4);
	cout << endl << endl;
	//ーーーキャリブレーション結果を格納するーーー
	FileStorage fs("caliburation.xml", FileStorage::WRITE);
	fs << "cameraMatrix" << cameraMatrix;
	fs << "distCoeffs" << distCoeffs;
	fs.release();

	//ーーー画像を補正するーーー
	undistort(src_image, dst_image, cameraMatrix, distCoeffs);
	namedWindow("補正画像");
	imshow("補正画像", dst_image);

	waitKey(0);
	destroyAllWindows();

	//カメラキャリブレーション値の更新
	cameraMat = cameraMatrix;
	distCoeff = distCoeffs;

	return 0;
}
