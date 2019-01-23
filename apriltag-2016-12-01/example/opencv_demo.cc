/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <iostream>

#include "opencv2/opencv.hpp"  //https://www.jianshu.com/p/7eee92d8ad7b

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/getopt.h"
#include "common/matd.h"
#include "common/homography.h"

#include "deep_math.h"

using namespace std;
using namespace cv;

struct cam_info_ //cam params
{
double fx;
double fy;
double cx;
double cy;
//double k1, k2, k3;
double p1,p2;
};

int cam_id=1;
bool show_info=true;

struct tag_info_
{
double size;
};

//cam_info_ cam_info={1225.836011,1226.075047,560.226194,356.268581};
cam_info_ cam_info={1238.906128,1256.916504,542.145490,362.780263};
tag_info_ tag_info={0.03};  //we need to half the size to get param s; 

int nrows=3;
int ncols=3; //reference :http://emb.hqyj.com/Column/10184.html
double data[]={1225.836011,0.000000,560.226194,0.000000,1226.075047,356.268581,0.000000,0.000000,1.000000};
matd_t* cam_intrinsic=matd_create_data(nrows, ncols, data);
matd_t* P_inv=matd_inverse(cam_intrinsic);

int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize camera
    VideoCapture cap(cam_id);

//to save out image--by deep 20181025
cap.set(CV_CAP_PROP_FRAME_WIDTH,960);
cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
VideoWriter out;
cv::Size sWH=cv::Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH),(int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
char* out_path="record.avi";
out.open(out_path, CV_FOURCC('M','P','4','2'),25.0,sWH);

    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11"))
        tf = tag36h11_create();
    else if (!strcmp(famname, "tag36h10"))
        tf = tag36h10_create();
    else if (!strcmp(famname, "tag36artoolkit"))
        tf = tag36artoolkit_create();
    else if (!strcmp(famname, "tag25h9"))
        tf = tag25h9_create();
    else if (!strcmp(famname, "tag25h7"))
        tf = tag25h7_create();
    else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }
    tf->black_border = getopt_get_int(getopt, "border");

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    Mat frame, gray;
    while (true) {
        cap >> frame;  //get frame from the camera
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);
        //cout << zarray_size(detections) << " tags detected" << endl; //the number of tags being detected
	static int k=0;
	if(0==zarray_size(detections))
	{
	k++;
	}
	else
	{
	k=0;
	}
	
	if(100==k)
	{
	cout<<"no tags being detected"<<endl;
	k=0;  //clear the variable k
	}
	
	//to pass date --deep
	matd_t* result;

	//show all the id being detected
	for (int j =0; j< zarray_size(detections); j++)
	{
	apriltag_detection_t *det;
        zarray_get(detections, j, &det);
	cout<<"tag:#"<<det->id<<" detected"<<endl;
	//print the H matrix==>matd_t *H; H is a pointer
	//to get the pose that is proportional to the H
	matd_t* pose=homography_to_pose(det->H,cam_info.fx,cam_info.fy,cam_info.cx,cam_info.cy);//4x4 matrix

	//matd_t* result=matd_scale(pose, 2/tag_info.size);
	result=matd_scale(pose, tag_info.size/2);
	//matd_t* h=matd_scale(det->H,tag_info.size/2);//将homography变成tag的实际尺寸
	//result=homography_to_pose(h,cam_info.fx,cam_info.fy,cam_info.cx,cam_info.cy);

	double q[4];
	matrix_to_quat(result,q);
	cout<<"position=>x:"<<result->data[3]<<" y:"<<result->data[7]<<" z:"<<result->data[11]<<endl;//need to make clear the sign of the z
	cout<<"orientation=>q0:"<<q[0]<<" q1:"<<q[1]<<" q2:"<<q[2]<<" q3:"<<q[3]<<endl;
	}
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

	//put the sensing information in the frame--deep
	if(show_info)
	{	stringstream ss_x,ss_y,ss_z,ss_qw,ss_qx,ss_qy,ss_qz;
		int fontface = FONT_HERSHEY_SIMPLEX;
		ss_x<<result->data[3];
		ss_y<<result->data[7];
		ss_z<<result->data[11];

		double q[4];
		matrix_to_quat(result,q);

		ss_qw<<q[0];
		ss_qx<<q[1];
		ss_qy<<q[2];
		ss_qz<<q[3];

		String px=ss_x.str();
		String py=ss_y.str();
		String pz=ss_z.str();
		String qw=ss_qw.str();
		String qx=ss_qx.str();
		String qy=ss_qy.str();
		String qz=ss_qz.str();

		double fontscale_info = 1.0;
		int h=30;
		int sh=50;
		int sw=10;
		putText(frame,"p_x: "+px, Point(sw,sh),fontface,fontscale_info,Scalar(0, 0, 0xff), 2);
		putText(frame,"p_y: "+py, Point(sw,sh+h),fontface,fontscale_info,Scalar(0, 0, 0xff), 2);
		putText(frame,"p_z: "+pz, Point(sw,sh+2*h),fontface,fontscale_info,Scalar(0, 0, 0xff), 2);
		putText(frame,"q_w: "+qw, Point(sw,sh+3*h),fontface,fontscale_info,Scalar(0, 0, 0xff), 2);
		putText(frame,"q_x: "+qx, Point(sw,sh+4*h),fontface,fontscale_info,Scalar(0, 0, 0xff), 2);
		putText(frame,"q_y: "+qy, Point(sw,sh+5*h),fontface,fontscale_info,Scalar(0, 0, 0xff), 2);
		putText(frame,"q_z: "+qz, Point(sw,sh+6*h),fontface,fontscale_info,Scalar(0, 0, 0xff), 2);
		
	}
        }
        zarray_destroy(detections);

        imshow("Tag Detections", frame);
	//save the video --deep
	out<<frame;

        if (waitKey(30) >= 0)
            break;
    }

    apriltag_detector_destroy(td);
    if (!strcmp(famname, "tag36h11"))
        tag36h11_destroy(tf);
    else if (!strcmp(famname, "tag36h10"))
        tag36h10_destroy(tf);
    else if (!strcmp(famname, "tag36artoolkit"))
        tag36artoolkit_destroy(tf);
    else if (!strcmp(famname, "tag25h9"))
        tag25h9_destroy(tf);
    else if (!strcmp(famname, "tag25h7"))
        tag25h7_destroy(tf);
    getopt_destroy(getopt);

    return 0;
}
