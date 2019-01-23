#include "opencv2/opencv.hpp" 
#include "iostream"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	char* video=NULL;
	VideoCapture cap;

	if(argc<2)
	{
		video="record.avi";	
		cap.open(video);
		if(!cap.isOpened())
		{
		cout<<"Usage: ./play_video file_of_video"<<endl;	
		return 0;
		}
		cout<<"You are try to open the defalut video: "<< video <<endl;	
	}
	else
	{
		video=argv[1];
		cap.open(video);
		if(!cap.isOpened())
		{
		cout<<"pls make sure that you input the correct file_of_video"<<endl;
		return 0;
		}
		cout<<"you are playing: "<<video<<endl;
	}

	Mat frame;
	while(1)
	{
	cap>>frame;
	if(frame.empty())
		break;
	imshow("video", frame);
	waitKey(30);
	}
	cap.release();
	return 0;
}
