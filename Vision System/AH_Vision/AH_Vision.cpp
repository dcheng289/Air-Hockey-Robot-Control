/*
AH_Vision.cpp
Derek Cheng derekwch@usc.edu
USC AME-441a Senior Projects Laboratory - Fall '16
Optimizing a Trajectory Tracking System - Air Hockey Robot
*/

#include "stdafx.h"	
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc.hpp" 
#include "opencv2/photo/photo.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <windows.h>
#include <conio.h>
#include "tserial.h"
#include "bot_control.h"
#include <iomanip>

using namespace cv;
using namespace std;

// Defense strategy
static int leftBound = 50;
static int halfLine = 350;	// When the puck crosses this, the robot moves

// Attack strategy
static int leftBoundAttack = leftBound + 50;	// Attack meets puck farther forward
static int halfLineAttack = halfLine+50;	// When the puck crosses this, the robot moves

// Wall bounds	
static int upperBound = 80;	
static int lowerBound = 420;		
static int rightBound = 620;
Point2i topLeft(leftBound, upperBound);
Point2i topRight(rightBound, upperBound);
Point2i bottomLeft(leftBound, lowerBound);
Point2i bottomRight(rightBound, lowerBound);

// For communication
serial comm_x;
serial comm_y;

// Select Strategy
int strategy = 1;	// 0 for defense, 1 for  attack

// Recording variables
String fileName = "C:/Users/Derek/Pictures/AirHockey/NewAttack_HighCurrent_1.avi";
int fps_read = 100; 
int fps_write = 40;			// speed that video is written to file
bool testingWalls = false;
bool niceRecording = true;

// Tracking, prediction, and filtering variables
int dx;
int dy;
bool motorMoved, robotCentered;
int framesTracked;

// Functions 
Point2i puckLocation(Mat frame); 
Point2i puckPrediction(Point2i cur);
void checkBounce(Point2i* cur, Point2i* pred, Point2i* firstBounce,int dx, int dy, Mat* frame, int strategy_);
void drawPrediction(Mat frame, Point2i a, Point2i b);
void sendMessage(int Timestamp, int pred_x, int pred_y, int pos_x, int pos_y);
void centerRobot();
void drawBouncePrediction(Mat frame, Point2i a, Point2i b);

int main()
{
	// Initialize filtering variables
	DWORD firstTimestamp = 0;
	DWORD frameTimestamp = 0;
	DWORD oldFrameTimestamp = 0;
	motorMoved = false;
	robotCentered = true;
	framesTracked = 0;

	// Initialize video recording from PS3 Eye Camera
	VideoCapture video(1);
	namedWindow("Video", CV_WINDOW_AUTOSIZE);
	Mat frame;
	video.set(CV_CAP_PROP_FPS, fps_read);
	bool suc = video.read(frame);
	double fps = video.get(CV_CAP_PROP_FPS);
	int dWidth = video.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	int dHeight = video.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	cout << "Frame size : " << dWidth << " x " << dHeight << " Frame rate: " << fps_read << endl;
	
	
	// Reading from recorded video
	/*
	VideoCapture video(fileName);
	namedWindow("Video", CV_WINDOW_AUTOSIZE);
	Mat frame;
	bool suc = video.read(frame);
	double fps = video.get(CV_CAP_PROP_FPS);
	cout << "Frames per second using video.get(CV_CAP_PROP_FPS) : " << fps << endl;
	*/
	
	// Initialize VideoWriter object
	VideoWriter vw;
	int numFrame = 0;		// number of frames being saved
	if (niceRecording) {
		Size frameSize(640, 480);
		vw.open(fileName, -1, fps_write, frameSize, true); 
		if (!vw.isOpened())
			return -1;
		// When prompted, use Intel Codec
	}

	// Initialize communication
	comm_x.startDevice("COM4", 115200);	//Baud Rate 115200	COM4
	comm_y.startDevice("COM5", 115200);

	// Initialize tracking variables
	Point2i prev(-1, -1);
	Point2i cur(-1, -1);
	Point2i firstBounce(-1, -1);
	Point2i lastpredictionPoint;
	bool first_frame = true;
	float num = 0;
	float den = 0;
	float bounceNum = 0;
	float bounceDenom = 0;
	float last_pred_y;
	bool averaging = true;
	int x;

	if (strategy == 0)	// Defense
		x = leftBound;
	if (strategy == 1) // Attack
		x = leftBoundAttack;

	// Loop until out of frames
	while (suc && !frame.empty())
	{
		cur = puckLocation(frame);

		// If no puck detected, cur = prev
		if (cur.x == -1 && cur.y == -1) 
		{
			cur = prev;
			circle(frame, lastpredictionPoint, 5, Scalar(0, 0, 255), 4, 8, 0); // draw prediction circle
		}

		// First frame issue
		if (first_frame) {
			last_pred_y = 240;
			first_frame = false;
		}

		// dy dx math
		dy = cur.y - prev.y;
		dx = cur.x - prev.x;

		// Calculate prediction point
		Point2i predictionPoint;
		if (cur.x > 0 && cur.y > 0 && dx < -5) // don't calculate unless puck move towards robot
		{
			int x_dist = x - cur.x;
			float pred_y;

			if (dy != 0 && dx != 0)	// Calculate pred_y with slope
				pred_y = (((float)dy / dx) * x_dist) + cur.y;
			else // puck didn't move
				pred_y = last_pred_y;

			Point2i buf(x, pred_y);
			predictionPoint = buf;	// prediction point established

			// Check bounce
			checkBounce(&cur, &predictionPoint, &firstBounce, dx, dy, &frame, strategy);

			// Ensure averaging is within bounds
			if (predictionPoint.y > lowerBound)
				predictionPoint.y = lowerBound;
			if (predictionPoint.y < upperBound)
				predictionPoint.x = upperBound;

			// Averaging and updating average
			den++;
			num += predictionPoint.y;
			predictionPoint.y = num / den;

			// Draw new prediction
			circle(frame, predictionPoint, 5, Scalar(0, 0, 255), 4, 8, 0); // prediction point circle
			if (firstBounce.x != -1) {
				bounceDenom++;
				bounceNum += firstBounce.x;
			}

			lastpredictionPoint = predictionPoint;
		}


		// No bounce
		if (firstBounce.x == -1 && firstBounce.y == -1) {
			if (dx < -5)	// Only draw when puck is moving towards robot
				 drawPrediction(frame, cur, predictionPoint);
		}
		// Puck bounce
		else { 
			if (dx < -5) {
				firstBounce.x = (int)(bounceNum / bounceDenom);
				 drawPrediction(frame, cur, firstBounce);	// firstBounce
				 drawPrediction(frame, firstBounce, predictionPoint);
			}
		}

		// Reset averaging and reset robot because puck has been returned
		if (dx > 5 && averaging) {
			den = 0;
			num = 0;
			lastpredictionPoint.x = -1;
			lastpredictionPoint.y = -1;
			bounceDenom = 0;
			bounceNum = 0;
			motorMoved = false;
			centerRobot();
		}

		// Debugging - Draw blue lines for walls
		if (testingWalls) {
			line(frame, topLeft, topRight, Scalar(255, 0, 0), 2, 8, 0);
			line(frame, topLeft, bottomLeft, Scalar(255, 0, 0), 2, 8, 0);
			line(frame, bottomRight, topRight, Scalar(255, 0, 0), 2, 8, 0);
			line(frame, bottomRight, bottomLeft, Scalar(255, 0, 0), 2, 8, 0);

			// Light blue line for half line
			int halfLineAdjusted;
			if (strategy == 0)
				halfLineAdjusted = halfLine;
			if (strategy == 1)
				halfLineAdjusted = halfLineAttack;

			Point2i top(halfLineAdjusted, upperBound);
			Point2i bottom(halfLineAdjusted, lowerBound);
			line(frame, top, bottom, Scalar(255, 255, 0), 2, 8, 0);
		}

		// Update
		prev = cur;
		firstBounce.x = -1;
		firstBounce.y = -1;

		// Show Results
		imshow("Video", frame);	// frame_resize
		waitKey(1);

		// Save Video
		if (niceRecording) {
			vw.write(frame);
			numFrame++;
			if (numFrame == fps_write*30) {	// 30 fps * 30 sec = 600 frames 
				vw.release();
				cout << "close video" << endl;
			}
		}

		// Send serial packet
		sendMessage(1, predictionPoint.x, predictionPoint.y, cur.x, cur.y);

		// New Read
		oldFrameTimestamp = frameTimestamp;
		bool suc = video.read(frame);
		frameTimestamp = GetTickCount();
	}
	comm_x.stopDevice();
	comm_y.stopDevice();
	video.release();
	destroyWindow("Video");
	return 0;
}

Point2i puckLocation(Mat frame)
{
	Point2i cur(-1, -1);
	Mat frame_red, frame_gray;
	imshow("Video", frame);

	// Convert to grayscale and store to src_gray	
	cvtColor(frame, frame_gray, CV_BGR2GRAY);
	GaussianBlur(frame_gray, frame_gray, Size(9, 9), 2, 2);	// filter the noise

	vector<Vec3f> circles;

	/// Apply the Hough Transform to find the circles
	int param1 = 30;	// higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
	int param2 = 18;	// the smaller it is, the more circles (potentially false) are returned
	int min_radius = 10;
	int max_radius = 15;
	HoughCircles(frame_gray, circles, CV_HOUGH_GRADIENT, 1, frame_gray.rows / 8, param1, param2, min_radius, max_radius);
	// method: CV_HOUGH_GRADIENT

	/// Draw the circles detected
	for (size_t i = 0; i < circles.size(); i++)
	{
		if (i > 0)
			break;
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);

		// checking table bounds
		bool xBound = center.x > topLeft.x && center.x > bottomLeft.x && center.x < topRight.x && center.x < bottomRight.x;
		bool yBound = center.y - radius > topLeft.y && center.y < bottomLeft.y && center.y > topRight.y - radius && center.y < bottomRight.y;

		if (xBound && yBound)
		{
			// draw circle center
			 circle(frame, center, 3, Scalar(0, 255, 0), -1, 8, 0);
			// draw circle outline
			 circle(frame, center, radius, Scalar(255, 0, 0), 3, 8, 0);
			//cout << "Point: " << center << endl  << ", Time: " << frameTimestamp - oldFrameTimestamp;
			cur = center;
		}

	}
	return cur;
}

void checkBounce(Point2i* cur, Point2i* predPoint, Point2i* bouncePoint, int dx, int dy, Mat* frame, int strategy_)
{
	int y_upper_dist = cur->y - upperBound;
	int y_lower_dist = lowerBound - cur->y;
	int leftBoundAdjusted;
	float pred_y, dist_y, dist_x;

	if (strategy_ == 0) // Defense
		leftBoundAdjusted = leftBound;
	if (strategy_ == 1) // Attack
		leftBoundAdjusted = leftBoundAttack;

	if (dx == 0 || dy == 0)	// not moving
		return;

	if (predPoint->y > upperBound && predPoint->y < lowerBound) // doesnt hit wall
		return;

	else if (predPoint->y < upperBound) // upper wall
	{	
		
		// calculate bouncePoint
		float bounce_x = -1 * abs((((float)dx / dy) * y_upper_dist)) + cur->x;
		bouncePoint->x = bounce_x;
		bouncePoint->y = upperBound;

		// calculate predPoint
		dist_x = bounce_x - leftBound;	
		dist_y = ((float)dy / dx) * dist_x;
		pred_y = upperBound + dist_y;

		predPoint->y = (int)pred_y;
		predPoint->x = leftBoundAdjusted;

	}
	else	// lower wall
	{	
		// calculate bouncePoint
		float bounce_x = -1 * abs((((float)dx / dy) * y_lower_dist)) + cur->x;	// seems wrong
		bouncePoint->x = bounce_x;
		bouncePoint->y = lowerBound;

		// calculate predPoint
		dist_x = bounce_x - leftBound;
		dist_y = (-1 * dist_x)*((float)dy / dx);
		pred_y = lowerBound - dist_y;

		predPoint->y = (int)pred_y;
		predPoint->x = leftBoundAdjusted;	
	}

}

void drawPrediction(Mat frame, Point2i a, Point2i b)
{
	if (niceRecording)
		line(frame, a, b, Scalar(0, 255, 0), 3, 8, 0);
}


void sendMessage(int Timestamp, int pred_x, int pred_y, int pos_x, int pos_y)
{
	// Select Strategy
	int moveThreshold, xBound;
	if (strategy == 0) {
		moveThreshold = halfLine;
		xBound = leftBound;
	}
	if (strategy == 1) {
		moveThreshold = halfLineAttack;
		xBound = leftBoundAttack;
	}

	// Message protocol, axis conversion is intentional
	string buf = "s20t" + to_string(pred_y) + "t" + to_string(pred_x) + "t" + to_string(pos_y) + "t" + to_string(pos_x) + "tr";

	// 4 conditions
		// Motor has not moved yet for this attack
		// Puck has crossed half line (pos_x < halfLine)
		// Puck is moving towards goal (dx < 20)
		// There is a prediction (pred_x == leftBound)
	if (!motorMoved && pos_x < moveThreshold && dx < 20 && pred_x == xBound)
		framesTracked++;

		// If all conditions met, then send the robot to intercept puck
	if (!motorMoved && pos_x < moveThreshold && dx < 20 && pred_x == xBound && framesTracked > 2) {
		for (int i = 0; i < buf.size(); i++)
		{
			comm_x.send_data(buf[i]);
			comm_y.send_data(buf[i]);
		}
		cout << buf << endl;
		motorMoved = true;
		robotCentered = false;
		framesTracked = 0;
	}

}

void centerRobot()	// center the robot when puck is returned
{
	if (!robotCentered)
	{
		String resetMessage = "m";

		for (int i = 0; i < resetMessage.size(); i++) {
			comm_x.send_data(resetMessage[i]);
			comm_y.send_data(resetMessage[i]);
		}
		robotCentered = true;
	}

}
