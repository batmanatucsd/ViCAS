/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/SetPen.h"
// %EndTag(MSG_HEADER)%

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libv4l2.h>

#include <sstream>


#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>

#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <errno.h>
#include <linux/videodev2.h>


#define BYTE uint8_t
#define TRUE 1
#define FALSE 0

using namespace std;
using namespace cv;

static int xioctl(int fd, int request, void *arg);
static void bayer16_convert_bayer8(int16_t *inbuf, uint8_t *outbuf, int width, int height, int shift);
void bayer_to_rgb24(BYTE *pBay, BYTE *pRGB24, int width, int height, int pix_order);
static void bayer_to_rgbbgr24(BYTE *bayer, BYTE *bgr, int width, int height, bool start_with_green, bool blue_line);
static void convert_border_bayer_line_to_bgr24( BYTE* bayer, BYTE* adjacent_bayer, BYTE *bgr, int width, bool start_with_green, bool blue_line);

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

    //name of node
    ros::init(argc, argv, "leopard_camera");
    ros::NodeHandle n;

    //topic to publish to
    ros::Publisher frameNumbers_pub = n.advertise<std_msgs::String>("frameNumbers", 1000);
    //ros::Publisher image_pub_node = n.advertise<sensor_msgs::Image>("image_rgb", 1);
    image_transport::ImageTransport it(n);

    image_transport::Publisher left_camera_pub = it.advertise("/left/image_rgb", 1);
    image_transport::Publisher right_camera_pub = it.advertise("/right/image_rgb", 1);
    //image_pub = it.advertise("/image_rgb", 1);


    //set's the loop rate to manage sleeping
    ros::Rate loop_rate(30);

    //loop_rate.sleep();
    //pubrate.sleep();

    //give time for the server and nodes to start up
    ros::Duration(1).sleep();

    //cv::VideoCapture* cap = new cv::VideoCapture(1);
    //cv::Mat imgRaw(480, 640, CV_16U);

    //open the device for reading

    int fd1 = v4l2_open("/dev/video1", O_RDWR);//TODO consider O_NONBLOCK flag
    int fd2 = v4l2_open("/dev/video2", O_RDWR);
    int bytesused1, bytesused2;
    int count = 0;
    int width = 640;//1280
    int height = 480;//720
/*
    //set video parameters
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;


    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
    {
        ROS_INFO("Error: Setting Pixel Format");
        return 1;
    }
*/
    while (ros::ok())
    {
        int16_t rawBuf1[width*height*3];
        int16_t rawBuf2[width*height*3];

        //read from device
        bytesused1 = v4l2_read(fd1, rawBuf1, width*height*3);
        bytesused2 = v4l2_read(fd2, rawBuf2, width*height*3);

        //handle erros from reading
        if (-1 == bytesused1 )
        {
            switch (errno)
            {
                case EAGAIN:
                    ROS_INFO("Cam1: No data available for read");

                    break;
                case EINVAL:
                    ROS_INFO("Cam1: Read method error, try mmap instead");

                    break;
                case EIO:
                    ROS_INFO("Cam1: read I/O Error");

                    break;
                default:
                    ROS_INFO("Cam1: read");

                    break;
            }

            break;
        }
        if (-1 == bytesused2 )
        {
            switch (errno)
            {
                case EAGAIN:
                    ROS_INFO("Cam2: No data available for read");

                    break;
                case EINVAL:
                    ROS_INFO("Cam2: Read method error, try mmap instead");

                    break;
                case EIO:
                    ROS_INFO("Cam2: read I/O Error");

                    break;
                default:
                    ROS_INFO("Cam2: read");

                    break;
            }

            break;
        }

        //ROS_INFO("Number of bytes read: %d", bytesused);

        /*
        //display raw info as hex
        for (int i = 0; i < 5; i++){

            ROS_INFO("rawBuf data as hex: %04x", rawBuf[i]);
        }
        ROS_INFO("Next Frame");
        */

        //decode the data from bayer to rgb
        uint8_t bayer8img1[width*height];
        uint8_t bayer8img2[width*height];
        bayer16_convert_bayer8(rawBuf1, bayer8img1, width, height, 4);
        bayer16_convert_bayer8(rawBuf2, bayer8img2, width, height, 4);

        uint8_t img_frame_rgb24_1[width*height*3];
        uint8_t img_frame_rgb24_2[width*height*3];
        bayer_to_rgb24(bayer8img1,img_frame_rgb24_1, width, height, 1);
        bayer_to_rgb24(bayer8img2,img_frame_rgb24_2, width, height, 1);

        //store the data into a cv mat
        cv::Mat cvImgRGB1(height, width, CV_8UC3, img_frame_rgb24_1);
        cv::Mat cvImgRGB2(height, width, CV_8UC3, img_frame_rgb24_2);
        //cv::Mat cvImgRGB(height, width, CV_8UC3);
        //how is img_frame_rgb24 stored in memory? how is it being coppied into cvImgRGB?
        //TODO how to get color? posssibly copy data using memcopy (from v4l2uvc.c) //update: it is the same as above
        //memcpy(&cvImgRGB.data[0], img_frame_rgb24, width*height * 3);


        //imshow("Camera1: rgbImage", cvImgRGB1);
        //imshow("Camera2: rgbImage", cvImgRGB2);
        cv::waitKey(1);

        //publish the image over ROS using image transport
        sensor_msgs::ImagePtr msg_rgb1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImgRGB1).toImageMsg();
        sensor_msgs::ImagePtr msg_rgb2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImgRGB2).toImageMsg();
        left_camera_pub.publish(msg_rgb1);
        right_camera_pub.publish(msg_rgb2);




        /*
        if (!cap->isOpened())
        {
            ROS_INFO("can not open webcam!");
        }

        bool bSuccess = cap->read(imgRaw); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            ROS_INFO("Cannot read a frame from video stream");

        }*/
/*
        //convert from raw bayer16 into bayer8
        for(int i = 0; i < imgRaw.rows; i++) {
            for(int j = 0; j < imgRaw.cols; j++) {
                //const cv::Point2f& v = ipImg.at<cv::Point2f>(i, j);
                unsigned short pixel = imgRaw.at<ushort>(i, j);

                ROS_INFO("imgRaw 16bit pixels: %04x", pixel);


                if (pixel == 0 || pixel < 5){
                    //opImg.at<float>(1, 1) = 255;
                    depth_cleaned.at<uchar>(i,j) = 255;
                    //depth.data[depth.step[0]*i + depth.step[1]*j + 0] = 255;
                    //depth_cleaned.data[depth_cleaned.step[0]*i + depth_cleaned.step[1]*j + 0] = 255;
                }
                //ROS_INFO("rows: %d", opImg.rows);
                //ROS_INFO("cols: %d", opImg.cols);
                //ROS_INFO("Pixel at (%d,%d): %d", j,i,pixel);
                //ROS_INFO("depth Dims: %d", depth.dims);
                //ROS_INFO("depth_cleaned Dims: %d", depth_cleaned.dims);

            }
        }
*/
        //Mat imRGB;
        //cv::cvtColor(imBayer, imRGB, CV_BayerGR2RGB);

        //imshow("Raw image", imgRaw); //show the frame in "MyVideo" window
        //imshow("Camera1: RGB image", imRGB1); //show the frame in "MyVideo" window

        //waitKey(1);


        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;

        std::stringstream ss;
        ss << "Frame number: " << count;
        msg.data = ss.str();

        //ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        frameNumbers_pub.publish(msg);

        //check for callbacks
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
      }

    //close the device
    v4l2_close(fd1);
    v4l2_close(fd2);


    return 0;
}

static int xioctl(int fd, int request, void *arg)
{
    int r;
        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);
        return r;
}

static void bayer16_convert_bayer8(int16_t *inbuf, uint8_t *outbuf, int width, int height, int shift)
{
    int i = 0, j = 0;

    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
            outbuf[i * width + j] = (inbuf[i * width + j] >> shift);
        }
    }
}
void bayer_to_rgb24(BYTE *pBay, BYTE *pRGB24, int width, int height, int pix_order)
{
    switch (pix_order)
    {
        //conversion functions are build for bgr, by switching b and r lines we get rgb
        case 0: /* gbgbgb... | rgrgrg... (V4L2_PIX_FMT_SGBRG8)*/
            bayer_to_rgbbgr24(pBay, pRGB24, width, height, TRUE, FALSE);
            break;

        case 1: /* grgrgr... | bgbgbg... (V4L2_PIX_FMT_SGRBG8)*/
            bayer_to_rgbbgr24(pBay, pRGB24, width, height, TRUE, TRUE);
            break;

        case 2: /* bgbgbg... | grgrgr... (V4L2_PIX_FMT_SBGGR8)*/
            bayer_to_rgbbgr24(pBay, pRGB24, width, height, FALSE, FALSE);
            break;

        case 3: /* rgrgrg... ! gbgbgb... (V4L2_PIX_FMT_SRGGB8)*/
            bayer_to_rgbbgr24(pBay, pRGB24, width, height, FALSE, TRUE);
            break;

        default: /* default is 0*/
            bayer_to_rgbbgr24(pBay, pRGB24, width, height, TRUE, FALSE);
            break;
    }
}

static void bayer_to_rgbbgr24(BYTE *bayer,
    BYTE *bgr, int width, int height,
    bool start_with_green, bool blue_line)
{
    /* render the first line */
    convert_border_bayer_line_to_bgr24(bayer, bayer + width, bgr, width,
        start_with_green, blue_line);
    bgr += width * 3;

    /* reduce height by 2 because of the special case top/bottom line */
    for (height -= 2; height; height--)
    {
        int t0, t1;
        /* (width - 2) because of the border */
        BYTE *bayerEnd = bayer + (width - 2);

        if (start_with_green)
        {
            /* OpenCV has a bug in the next line, which was
            t0 = (bayer[0] + bayer[width * 2] + 1) >> 1; */
            t0 = (bayer[1] + bayer[width * 2 + 1] + 1) >> 1;
            /* Write first pixel */
            t1 = (bayer[0] + bayer[width * 2] + bayer[width + 1] + 1) / 3;
            if (blue_line)
            {
                *bgr++ = t0;
                *bgr++ = t1;
                *bgr++ = bayer[width];
            }
            else
            {
                *bgr++ = bayer[width];
                *bgr++ = t1;
                *bgr++ = t0;
            }

            /* Write second pixel */
            t1 = (bayer[width] + bayer[width + 2] + 1) >> 1;
            if (blue_line)
            {
                *bgr++ = t0;
                *bgr++ = bayer[width + 1];
                *bgr++ = t1;
            }
            else
            {
                *bgr++ = t1;
                *bgr++ = bayer[width + 1];
                *bgr++ = t0;
            }
            bayer++;
        }
        else
        {
            /* Write first pixel */
            t0 = (bayer[0] + bayer[width * 2] + 1) >> 1;
            if (blue_line)
            {
                *bgr++ = t0;
                *bgr++ = bayer[width];
                *bgr++ = bayer[width + 1];
            }
            else
            {
                *bgr++ = bayer[width + 1];
                *bgr++ = bayer[width];
                *bgr++ = t0;
            }
        }

        if (blue_line)
        {
            for (; bayer <= bayerEnd - 2; bayer += 2)
            {
                t0 = (bayer[0] + bayer[2] + bayer[width * 2] +
                    bayer[width * 2 + 2] + 2) >> 2;
                t1 = (bayer[1] + bayer[width] +
                    bayer[width + 2] + bayer[width * 2 + 1] +
                    2) >> 2;
                *bgr++ = t0;
                *bgr++ = t1;
                *bgr++ = bayer[width + 1];

                t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
                t1 = (bayer[width + 1] + bayer[width + 3] +
                    1) >> 1;
                *bgr++ = t0;
                *bgr++ = bayer[width + 2];
                *bgr++ = t1;
            }
        }
        else
        {
            for (; bayer <= bayerEnd - 2; bayer += 2)
            {
                t0 = (bayer[0] + bayer[2] + bayer[width * 2] +
                    bayer[width * 2 + 2] + 2) >> 2;
                t1 = (bayer[1] + bayer[width] +
                    bayer[width + 2] + bayer[width * 2 + 1] +
                    2) >> 2;
                *bgr++ = bayer[width + 1];
                *bgr++ = t1;
                *bgr++ = t0;

                t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
                t1 = (bayer[width + 1] + bayer[width + 3] +
                    1) >> 1;
                *bgr++ = t1;
                *bgr++ = bayer[width + 2];
                *bgr++ = t0;
            }
        }

        if (bayer < bayerEnd)
        {
            /* write second to last pixel */
            t0 = (bayer[0] + bayer[2] + bayer[width * 2] +
                bayer[width * 2 + 2] + 2) >> 2;
            t1 = (bayer[1] + bayer[width] +
                bayer[width + 2] + bayer[width * 2 + 1] +
                2) >> 2;
            if (blue_line)
            {
                *bgr++ = t0;
                *bgr++ = t1;
                *bgr++ = bayer[width + 1];
            }
            else
            {
                *bgr++ = bayer[width + 1];
                *bgr++ = t1;
                *bgr++ = t0;
            }
            /* write last pixel */
            t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
            if (blue_line)
            {
                *bgr++ = t0;
                *bgr++ = bayer[width + 2];
                *bgr++ = bayer[width + 1];
            }
            else
            {
                *bgr++ = bayer[width + 1];
                *bgr++ = bayer[width + 2];
                *bgr++ = t0;
            }
            bayer++;
        }
        else
        {
            /* write last pixel */
            t0 = (bayer[0] + bayer[width * 2] + 1) >> 1;
            t1 = (bayer[1] + bayer[width * 2 + 1] + bayer[width] + 1) / 3;
            if (blue_line)
            {
                *bgr++ = t0;
                *bgr++ = t1;
                *bgr++ = bayer[width + 1];
            }
            else
            {
                *bgr++ = bayer[width + 1];
                *bgr++ = t1;
                *bgr++ = t0;
            }
        }

        /* skip 2 border pixels */
        bayer += 2;

        blue_line = !blue_line;
        start_with_green = !start_with_green;
    }

    /* render the last line */
    convert_border_bayer_line_to_bgr24(bayer + width, bayer, bgr, width,
        !start_with_green, !blue_line);
}

static void convert_border_bayer_line_to_bgr24( BYTE* bayer, BYTE* adjacent_bayer,
    BYTE *bgr, int width, bool start_with_green, bool blue_line)
{
    int t0, t1;

    if (start_with_green)
    {
    /* First pixel */
        if (blue_line)
        {
            *bgr++ = bayer[1];
            *bgr++ = bayer[0];
            *bgr++ = adjacent_bayer[0];
        }
        else
        {
            *bgr++ = adjacent_bayer[0];
            *bgr++ = bayer[0];
            *bgr++ = bayer[1];
        }
        /* Second pixel */
        t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
        t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
        if (blue_line)
        {
            *bgr++ = bayer[1];
            *bgr++ = t0;
            *bgr++ = t1;
        }
        else
        {
            *bgr++ = t1;
            *bgr++ = t0;
            *bgr++ = bayer[1];
        }
        bayer++;
        adjacent_bayer++;
        width -= 2;
    }
    else
    {
        /* First pixel */
        t0 = (bayer[1] + adjacent_bayer[0] + 1) >> 1;
        if (blue_line)
        {
            *bgr++ = bayer[0];
            *bgr++ = t0;
            *bgr++ = adjacent_bayer[1];
        }
        else
        {
            *bgr++ = adjacent_bayer[1];
            *bgr++ = t0;
            *bgr++ = bayer[0];
        }
        width--;
    }

    if (blue_line)
    {
        for ( ; width > 2; width -= 2)
        {
            t0 = (bayer[0] + bayer[2] + 1) >> 1;
            *bgr++ = t0;
            *bgr++ = bayer[1];
            *bgr++ = adjacent_bayer[1];
            bayer++;
            adjacent_bayer++;

            t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
            t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
            *bgr++ = bayer[1];
            *bgr++ = t0;
            *bgr++ = t1;
            bayer++;
            adjacent_bayer++;
        }
    }
    else
    {
        for ( ; width > 2; width -= 2)
        {
            t0 = (bayer[0] + bayer[2] + 1) >> 1;
            *bgr++ = adjacent_bayer[1];
            *bgr++ = bayer[1];
            *bgr++ = t0;
            bayer++;
            adjacent_bayer++;

            t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
            t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
            *bgr++ = t1;
            *bgr++ = t0;
            *bgr++ = bayer[1];
            bayer++;
            adjacent_bayer++;
        }
    }

    if (width == 2)
    {
        /* Second to last pixel */
        t0 = (bayer[0] + bayer[2] + 1) >> 1;
        if (blue_line)
        {
            *bgr++ = t0;
            *bgr++ = bayer[1];
            *bgr++ = adjacent_bayer[1];
        }
        else
        {
            *bgr++ = adjacent_bayer[1];
            *bgr++ = bayer[1];
            *bgr++ = t0;
        }
        /* Last pixel */
        t0 = (bayer[1] + adjacent_bayer[2] + 1) >> 1;
        if (blue_line)
        {
            *bgr++ = bayer[2];
            *bgr++ = t0;
            *bgr++ = adjacent_bayer[1];
        }
        else
        {
            *bgr++ = adjacent_bayer[1];
            *bgr++ = t0;
            *bgr++ = bayer[2];
        }
    }
    else
    {
        /* Last pixel */
        if (blue_line)
        {
            *bgr++ = bayer[0];
            *bgr++ = bayer[1];
            *bgr++ = adjacent_bayer[1];
        }
        else
        {
            *bgr++ = adjacent_bayer[1];
            *bgr++ = bayer[1];
            *bgr++ = bayer[0];
        }
    }
}
