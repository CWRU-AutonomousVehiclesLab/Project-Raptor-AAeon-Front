#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class laneMarker{
    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber rawSubscriber;
        image_transport::Publisher debug_gray;
        image_transport::Publisher debug_mask;
        image_transport::Publisher debug_Gaussian;
        image_transport::Publisher debug_CannyEdge;
        image_transport::Publisher debug_Hough;
        image_transport::Publisher resultOverlay;
        image_transport::Publisher result;
        ros::Publisher lane_publisher;
        int length,height;
        /*
        * This function is useful for publishing an image. It takes in a publisher and an cv::Mat image
        */
        void publishOpenCVImage(image_transport::Publisher publisher, cv::Mat inputImage, bool option){
            //cv_bridge::CvImage(header, encoding, image)
            cv::Mat displayImage;
            if (option){
                cv::cvtColor(inputImage,displayImage,CV_GRAY2BGR);
            }else{
                displayImage = inputImage;
            }
            cv_bridge::CvImage imgBridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, displayImage);
            sensor_msgs::Image imageMessage;
            imgBridge.toImageMsg(imageMessage);
            publisher.publish(imageMessage);
        }
        
    public:
        //* Constructor

        laneMarker() : it(nh) {
            //! Subscriber subscribe and then process the image using process
            rawSubscriber = it.subscribe("/front_realSense/color/image_raw", 1, &laneMarker::processIMG, this);
            debug_Gaussian = it.advertise("/debug/front_realSense/Gaussian",1);
            debug_mask = it.advertise("/debug/front_realSense/MaskPure",1);
            debug_CannyEdge = it.advertise("/debug/front_realSense/Cannyedge",1);
            debug_Hough = it.advertise("/debug/front_realSense/Houghtransform",1);

            resultOverlay = it.advertise("/front_realSense/cv_processed/overlay",1);
            result = it.advertise("/front_realSense/cv_processed/result",1);

            lane_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/front_realSense/detected_lane",1);
        }


        void processIMG(const sensor_msgs::ImageConstPtr& srcImg){
            //! 0. Process Magic Variable (Can be replaced with dynamic reconfigure later)
            //* mask
            double mask_top_ratio = 0.3;
            double mask_bottom_ratio = 0.25;
            int secondary_crop = 5; //pixels
            //* Gaussain Blur:
            cv::Size gaussian_Size = cv::Size(9,9);
            double gaussian_SigmaX = 0;
            double gaussian_Sigmay = 0;

            //* Canny Edge:
            double canny_lowThreashold = 50;
            double canny_highThreashold = 200;
            double canny_kernalSize = 3;

            //* Hough Transform:
            double hough_rho = 2;
            double hough_theta = CV_PI/180;
            int hough_threshold = 100;
            double hough_minLineLength = 40; 
            double hough_maxLineGap = 5;



            ROS_INFO("Image Update Recieved!");
            //! 1. Read in image
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(srcImg,sensor_msgs::image_encodings::BGR8);
            }
            catch(const cv_bridge::Exception& e)
            {   
                ROS_ERROR("cv_bridge BGR converstion exception: %s",e.what());
                return;
            }
            cv::Mat processed_IMG;
            ROS_INFO("Image Read In Complete!");

            //! 2. Convert into Gray Scale:
            cv::cvtColor(cv_ptr->image,processed_IMG,CV_BGR2GRAY);
            publishOpenCVImage(debug_gray,processed_IMG,true);
            ROS_INFO("Image Grayscale Conversion Complete!");

            //! 3. Binarize and isolate out useless information
            cv::Mat mask(processed_IMG.rows,processed_IMG.cols,CV_8UC1,cv::Scalar(0));
            //* Binary Mask Keypoints:
            std::vector<cv::Point> trapzoid;
            length = processed_IMG.cols;
            height = processed_IMG.rows;
            trapzoid.push_back(cv::Point(0+length*mask_top_ratio, 0));
            trapzoid.push_back(cv::Point(length*(1-mask_top_ratio),0));
            trapzoid.push_back(cv::Point(length,height*(1-mask_bottom_ratio)));
            trapzoid.push_back(cv::Point(length,height));
            trapzoid.push_back(cv::Point(0,height));
            trapzoid.push_back(cv::Point(0,height*(1-mask_bottom_ratio)));
            //* Draw Mask
            cv::fillConvexPoly( mask, trapzoid, 255 );
            publishOpenCVImage(debug_mask,mask,true);
            processed_IMG = processed_IMG & mask;
            ROS_INFO("Masking 1 Complete!");

            //! 4. Gaussian Blur:
            /**
             * @param src input image
             * @param dst output image of the same size and type as src.
             * @param ksize Gaussian kernel size.
             * @param sigmaX Gaussian kernel standard deviation in X direction
             * @param sigmaY Gaussian kernel standard deviation in Y direction
             * @param borderType pixel extrapolation method
             * 
             * CV_EXPORTS_W void GaussianBlur( InputArray src, OutputArray dst, Size ksize,
                                double sigmaX, double sigmaY = 0,
                                int borderType = BORDER_DEFAULT );
             */
            cv::GaussianBlur(processed_IMG,processed_IMG,gaussian_Size,gaussian_SigmaX,gaussian_Sigmay);
            publishOpenCVImage(debug_Gaussian,processed_IMG,true);
            ROS_INFO("Gaussian Blur Complete!");

            //! 5. Canny Edge Detector
            cv::Canny(processed_IMG,processed_IMG,canny_lowThreashold, canny_highThreashold, canny_kernalSize);
            ROS_INFO("Canny Edge Complete!");

            //! 6. Binarize and isolate out useless information
            cv::Mat mask2(processed_IMG.rows,processed_IMG.cols,CV_8UC1,cv::Scalar(0));
            
            //* Binary Mask Keypoints:
            std::vector<cv::Point> trapzoid2;
            length = processed_IMG.cols;
            height = processed_IMG.rows;
            trapzoid2.push_back(cv::Point(0+length*mask_top_ratio+secondary_crop, 0));
            trapzoid2.push_back(cv::Point(length*(1-mask_top_ratio)-secondary_crop,0));
            trapzoid2.push_back(cv::Point(length-secondary_crop,height*(1-mask_bottom_ratio)-secondary_crop));
            trapzoid2.push_back(cv::Point(length-secondary_crop,height));
            trapzoid2.push_back(cv::Point(0+secondary_crop,height));
            trapzoid2.push_back(cv::Point(0+secondary_crop,height*(1-mask_bottom_ratio)-secondary_crop));

            //* Draw Mask
            cv::fillConvexPoly( mask2, trapzoid2, 255 );
            processed_IMG = processed_IMG & mask2;
            publishOpenCVImage(debug_CannyEdge,processed_IMG,true);
            ROS_INFO("Masking & Canny Edge 2 Complete!");

            //! 7. Hough transform
            std::vector<cv::Vec4i> lines;
            /**
            * @param processed_IMG
            * @param outputArray
            * @param rho Distance resolution of the accumulator in pixels
            * @param theta Angle resolution of the accumulator in radians.
            * The larger the above two, the less precise they will be. Too small is also bad!
            * @param threshold Accumulator threshold parameter. Only those lines are returned that get enough votes ( \f$>\texttt{threshold}\f$ ).
            * @param minLineLength Minimum line length. Line segments shorter than that are rejected.
            * @param maxLineGap Maximum allowed gap between points on the same line to link them.
            */
            cv::HoughLinesP(processed_IMG,lines,hough_rho,hough_theta,hough_threshold,hough_minLineLength,hough_maxLineGap);


            //! Final Generation
            publishOpenCVImage(result,processed_IMG,true);
            cv::Mat display;
            cv::cvtColor(processed_IMG,display,CV_GRAY2BGR);
            publishOpenCVImage(resultOverlay,display+cv_ptr->image,false);
            return;
        }
    void displayHough();
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "laneMarker");
    laneMarker worker;
    ros::spin();    
    return 0;
}