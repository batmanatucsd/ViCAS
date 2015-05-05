#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <leopard_imaging_camera/copterTrackerV2Config.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <string>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>

//OpenCV specific includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/objdetect/objdetect.hpp>


#define MAX_INT 9999
#define MIN_INT -9999

#define CLUSTER_STATE_BAD_AREA 1
#define CLUSTER_STATE_GOOD_AREA_BAD_SHAPE 2
#define CLUSTER_STATE_GOOD_AREA_GOOD_SHAPE 3

#define VIS_METH_ALL_CLUSTERS 0
#define VIS_METH_CLUSTER_STATE 1
#define VIS_METH_OFF 2
#define USING_VISUALIZER 1

#define ROI_STATIC 0
#define ROI_DYNAMIC 1

//set this
#define SHAPE_THRESH 4.4 //how much longer an object's length is compared to its width. 1 is a square

#define TRAINING_PCD_FILE_PATH "//home/frank/cv_ros_ws/src/ViCAS/leopard_imaging_camera/training/copter_PCDs/"
#define TRAINING_VFH_MODEL_PATH "//home/frank/cv_ros_ws/src/ViCAS/leopard_imaging_camera/training/copter_VFH_model/"
#define TRAINING_SAVING_PCD 0 //change on compiling. choose what start up phase you want when training a new copter model
#define TRAINING_VFH_MODEL 0

//set this
#define TRAINING_CASCADE 0

#define RGB_HEIGHT 480
#define RGB_WIDTH 640
#define CASCADE_WIDTH 100
#define CASCADE_HEIGHT 40

#define NUM_FRAMES_DELAYED 4

//should match the size used for training. (NO NOT CHANGE)
#define BOUNDING_RECT_SCALE_FACTOR 1.9 //size of image passed to classifier (how tight the image is the the copter)

//set these
#define TRAIN_PATH_NEG "//home/frank/cv_ros_ws/src/ViCAS/leopard_imaging_camera/training/neg05/"
#define TRAIN_PATH_POS "//home/frank/cv_ros_ws/src/ViCAS/leopard_imaging_camera/training/pos05/"
#define COPTER_CLASSIFIER_PATH "//home/frank/cv_ros_ws/src/ViCAS/leopard_imaging_camera/classifiers/crazyflie/copter_1300_1300_24/cascade.xml"


typedef std::pair<std::string, std::vector<float> > vfh_model;

/**********GLOBALS: dynamic recofigure parameters*********/
bool use_voxel_filter; //true or false
float vox_leaf_size; //side length of each filtered cube in meters (larger, fewer clusters, but faster)
float cluster_tolerance;
int cluster_size_min; //clustrers of size less than cluster_size_min will be ignored (filtered in voxel filtering)
int cluster_size_max; //clusters of size greater than cluster_size_max will be ignored (filtered in voxel filtering)
pcl::VoxelGrid<pcl::PointXYZ>* pvoxel_grid;
pcl::EuclideanClusterExtraction<pcl::PointXYZ>* pcluster_extractor;
float g_copter_area_min, g_copter_area_max;
int vis_method;
float roi_size_multiplier; //multipied by g_copter_area_max to find the length of a side of the ROI cube
bool use_roi, use_classifier;
float roi_size_penalty_multiplier_increment;
int roi_type;

class CopterTrackerV2
{

    //image_transport::Publisher image_pub;
    ros::Publisher pub_cloud;
    ros::Publisher pub_copter_center_stamped3d;
    ros::Publisher pub_copter_center_3d;

    ros::Subscriber sub;
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
    void cloud_img_cb(const sensor_msgs::PointCloud2ConstPtr& input_cloud, const sensor_msgs::ImageConstPtr& input_img);


    image_transport::Subscriber image_sub_rgb;
    void callback_rgb(const sensor_msgs::ImageConstPtr& msg);
    image_transport::ImageTransport iTrans;
    image_transport::Publisher image_pub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_filt;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_rgb_filt;
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> *sync;


    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid; //used to downsample cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree; //used to search through voxels to do extraction

    pcl::visualization::PCLVisualizer* pviewer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud_roi_out;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud_roi_in;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_finalists;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_finalists_rgb;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> shared_clusters_finalists;
    bool shared_clusters_finalists_reading, shared_clusters_finalists_writing;

    std::vector<int> indecies_map; //map location of point in filtered (without Nan) point cloud to location in original (with NaN) point cloud

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
    //vector of vector of ints. cluster_indicies[0] is first cluster
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::PassThrough<pcl::PointXYZ> passThrough_filter;

    float roi_x_min, roi_x_max, roi_y_min, roi_y_max, roi_z_min, roi_z_max;
    pcl::PointXYZ copter_center;
    bool copter_found, cleared_for_roi, shared_copter_found, shared_copter_found_reading, shared_copter_found_writing;

    float roi_size_penalty_multiplier;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud_copter_cand; //copter cloud. used when saving cloud to disk or getting image locations
    int saveNum;

    std::queue<cv::Mat> image_queue;
    int numPos, numNeg;
    cv::CascadeClassifier copter_classifier;
    int finalists_count_sent;
    int finalists_count_received;
    int callback_rgb_num, callback_cloud_num;


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> getClusterColor(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud, int index);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> getClusterColorState(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud, int state);
    void applyDynamicROI3D(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud,
                    typename pcl::PointCloud< pcl::PointXYZ >::Ptr &cloud_inside_roi,
                    typename pcl::PointCloud< pcl::PointXYZ >::Ptr &cloud_outside_roi,
                    const pcl::PointXYZ & copter_center, const float copter_area_max, const float roi_local_multiplier, const float roi_size_penalty, pcl::visualization::PCLVisualizer* pviewer);
    float calcApproxSurfaceArea(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud);
    pcl::PointXYZ calcCenter(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud);

    int filterByShapeAndSize(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud, const float, const float, const float, const char*);
    void createAndWriteModelVFH(std::string load_pcd_dir, std::string write_model_dir);
    void loadPCDs (const boost::filesystem::path &base_dir, std::string &extension,
                       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &pcds);
    void computeNormals(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &pcds, std::vector<pcl::PointCloud<pcl::Normal>::Ptr> &normals);
    void getBoundingRect(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud, int &row_min, int &row_max, int &col_min, int &col_max);
    cv::Rect getAdjustedRect(const int row_min, const int row_max, const int col_min, const int col_max, const float scale);
    bool isCopter(cv::Mat &croppedFinalist, cv::CascadeClassifier &copter_classifier);


    public:

    CopterTrackerV2(ros::NodeHandle n) : iTrans(n),
        pcloud(new pcl::PointCloud<pcl::PointXYZ>),
        pcloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
        pcloud_roi_out(new pcl::PointCloud<pcl::PointXYZ>),
        pcloud_roi_in(new pcl::PointCloud<pcl::PointXYZ>)
    {

        //sub = n.subscribe("/stereo/points2", 1, &CopterTrackerV2::cloud_cb, this);
        pub_cloud = n.advertise<sensor_msgs::PointCloud2> ("/stereo/points2_copter", 1); //TODO consider subscribing at a lower frequency to take less CPU
        pub_copter_center_stamped3d = n.advertise<geometry_msgs::PointStamped>("copter_center_stamped_3d", 1);
        pub_copter_center_3d = n.advertise<geometry_msgs::Point>("copter_center_3d", 1);
        //image_sub_rgb = iTrans.subscribe("/stereo/left/image_rect", 1, &CopterTrackerV2::callback_rgb, this);
        image_pub = iTrans.advertise("/stereo/left/copter_boxed", 1);

        cloud_sub_filt.subscribe(n, "/stereo/points2", 1);
        image_sub_rgb_filt.subscribe(n, "/stereo/left/image_rect", 1);
        sync = new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image>(cloud_sub_filt, image_sub_rgb_filt, 10);
        sync->registerCallback(boost::bind(&CopterTrackerV2::cloud_img_cb, this, _1, _2));


        //1cm cubic box
        voxel_grid.setLeafSize (vox_leaf_size, vox_leaf_size, vox_leaf_size);
        tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>());

        if (USING_VISUALIZER){
            pviewer = new pcl::visualization::PCLVisualizer("myViewer");
            pviewer->setBackgroundColor(0,0,0);
            pviewer->addCoordinateSystem(1.0);
            pviewer->initCameraParameters();
        }

        cluster_extractor.setClusterTolerance (cluster_tolerance); // 2cm //lower means more clusters, higher means less clusters
        cluster_extractor.setMinClusterSize (cluster_size_min);
        cluster_extractor.setMaxClusterSize (cluster_size_max);
        cluster_extractor.setSearchMethod (tree);

        pvoxel_grid = &voxel_grid;
        pcluster_extractor = &cluster_extractor;

        copter_found = false;
        shared_copter_found = false;
        shared_copter_found_reading = false;
        shared_copter_found_writing = false;

        cleared_for_roi = false; //do not use roi until first copter has been found
        roi_size_penalty_multiplier = 1; //no penality initially
        saveNum = 0;

        if (TRAINING_VFH_MODEL){
            createAndWriteModelVFH(TRAINING_PCD_FILE_PATH, TRAINING_VFH_MODEL_PATH);

        }
        shared_clusters_finalists_reading = false;
        shared_clusters_finalists_writing = false;

        numPos = 0;
        numNeg = 0;
        if( !copter_classifier.load(COPTER_CLASSIFIER_PATH)){
            ROS_INFO("--(!)Error loading face cascade");
        }

        finalists_count_sent = 0;
        finalists_count_received = 0;
        callback_cloud_num = 0;
        callback_rgb_num = 0;
    }

};

void CopterTrackerV2::cloud_img_cb(const sensor_msgs::PointCloud2ConstPtr& input_cloud, const sensor_msgs::ImageConstPtr& input_img){

    //ROS_INFO("\ngot synched callback\n");


/*
    //check if copter was previously found: for roi adjustments
    if (!shared_copter_found_writing){
        shared_copter_found_reading = true;
        copter_found = shared_copter_found;
        shared_copter_found_reading = false;
    }
*/
    // Convert to PCL data type
    pcl::fromROSMsg(*input_cloud, *pcloud);

    //Convert to OpenCV data type
    cv_bridge::CvImagePtr mono_orig;

    try {
        mono_orig = cv_bridge::toCvCopy(input_img, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        ROS_INFO("CV_BRIDGE ERROR\n");
        return;
    }

    cv::Mat rgb_image(mono_orig->image.rows, mono_orig->image.cols, CV_8UC3);
    cvtColor(mono_orig->image, rgb_image, CV_GRAY2RGB);

    cv::Mat img_cropped;
    cv::Mat img_cropped_resized;

    cv::Mat gray_img;
    //cvtColor(rgb_orig->image, gray_img, CV_RGB2GRAY);
    gray_img = mono_orig->image; //TODO consider copy here



    if (copter_found){
        roi_size_penalty_multiplier = 1; //use normal ROI size
        cleared_for_roi = true;

        if (TRAINING_SAVING_PCD){
            std::cout << "Do you wish to save the cluster? (y/n)" << std::endl;

            if (getchar() == 'y'){
                char buffer[10];
                saveNum++;

                sprintf(buffer, "%d", saveNum);
                std::string filename = std::string(TRAINING_PCD_FILE_PATH) + std::string(buffer) + std::string(".pcd");

                ROS_INFO("Saving Cluster to %s", filename.c_str());
                pcl::io::savePCDFileASCII (filename.c_str(), *pcloud_copter_cand);
            }
        }
    }
    else {
        //increase ROI when copter is not found
        roi_size_penalty_multiplier += roi_size_penalty_multiplier_increment;
    }

    //TODO filter cloud based on copter's altitude
    //TODO reset roi (cleared_for_roi = false) when copter's velocity doesn't match object's velocity... maybe not. wind drift may cause movement or cancel movement
    //TODO filter based on 3D shape
    //TODO filter on appaerance (cascade classifier)
    //TODO visualize the ROI addCube (last one in doc)
    //TODO filter if object is on wall or floor. (label large planes with 3 points) (from kinect cloud)

    //TODO don't use ROI when stuck on a static false positive (update cleared_for_roi) (UPDATE: maybe not... when moving, nothing is static)
    //TODO sometimes copter is detected way far back because of the shadow errors and thus the roi is set far away. fix these false positives or do initial thresh on z axis (too close or too far)
    //TODO ROI is jumping even though nothing has been red boxed (confirmed via cascade classifier)
    if (roi_type == ROI_DYNAMIC){
        //if using roi and a copter has been previously found (applies ROI around copter's center)
        if (!TRAINING_CASCADE && use_roi && cleared_for_roi){
            applyDynamicROI3D(pcloud, pcloud, pcloud_roi_out, copter_center, g_copter_area_max, roi_size_multiplier, roi_size_penalty_multiplier, pviewer);
        }
    }
    else if (roi_type == ROI_STATIC){
        if (!TRAINING_CASCADE && use_roi && copter_found){
            applyDynamicROI3D(pcloud, pcloud, pcloud_roi_out, copter_center, g_copter_area_max, roi_size_multiplier, 1, pviewer);
        }
    }
    copter_found = false;
/*
    //copter was found
    if (!shared_copter_found_reading){
        shared_copter_found_writing = true;
        shared_copter_found = false;
        shared_copter_found_writing = false;
    }
*/

    //remove NaN from input cloud (all non features are NaN hence PCD is 640x480 points)
    pcl::removeNaNFromPointCloud(*pcloud, *pcloud, indecies_map);

    //sample (filter) point cloud to improve speed
    if (use_voxel_filter){

        voxel_grid.setInputCloud(pcloud);
        voxel_grid.filter(*pcloud_filtered);
    }
    else {
        *pcloud_filtered = *pcloud;
    }

    //check for empty pcd
    //if empty, show rgb image and return
    if (pcloud_filtered->size() == 0){

        cv::waitKey(1);
        cv::imshow("left_camera: published copter location", rgb_image);

        //cv_bridge::CvImage rgb_image_bridge(rgb_image, sensor_msgs::image_encodings::TYPE_8UC3);
        //image_pub.publish(rgb_image_bridge.toImageMsg());
        sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
        image_pub.publish(msg_rgb);
        return;
    }

    tree->setInputCloud(pcloud_filtered);


    cluster_extractor.setInputCloud (pcloud_filtered);
    cluster_extractor.extract (cluster_indices);


    int cluster_count = 0;
    int cluster_state;
    //iterate through all extracted clusters
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);

        //iterate throguh the indicies of the cluster
        //populate cluster with the data from the point cloud at the indicies of the cluster
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cluster->points.push_back (pcloud_filtered->points[*pit]); //*

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;


        char cluster_id[10];
        sprintf(cluster_id, "cloud%d", cluster_count);

        //TODO min and max not globals... accessed in loop. #define or make reconfig callback a class function
        cluster_state = filterByShapeAndSize(cluster, g_copter_area_min, g_copter_area_max, SHAPE_THRESH, cluster_id);

        if (USING_VISUALIZER){
            if (vis_method == VIS_METH_ALL_CLUSTERS){
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color = getClusterColor(pcloud_filtered, cluster_count);
                pviewer->addPointCloud<pcl::PointXYZ> (cluster, single_color, cluster_id);
            }
            else if (vis_method == VIS_METH_CLUSTER_STATE){
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color = getClusterColorState(pcloud_filtered, cluster_state);
                pviewer->addPointCloud<pcl::PointXYZ> (cluster, single_color, cluster_id);
            }
            else if (VIS_METH_OFF){
                //do not display anything
            }
        }

/*
        //view the clusters //TODO make bad clusters red and good green (also yellow and white)
        if (cluster_meets_shape_and_size){
            ROS_INFO("%s meets the shape and size requirement", cluster_id);
            //add final candidates to the viewer
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color = getClusterColor(pcloud_filtered, cluster_count);
            pviewer->addPointCloud<pcl::PointXYZ> (cluster, single_color, cluster_id);
        }
*/
        if (cluster_state == CLUSTER_STATE_GOOD_AREA_GOOD_SHAPE){
            //copter_center = calcCenter(cluster);
            //copter_found = true;
            //TODO add to vector of cluster copter candidates
            clusters_finalists.push_back(cluster);
            pcloud_copter_cand = cluster;
            //int row_min, row_max, col_min, col_max;
            //getBoundingRect(pcloud_copter_cand, row_min, row_max, col_min, col_max);
        }
        else {
            //copter_found = false;
        }
        cluster_count++;


    }
    //copter is not found if no clusters were processed
    //if (cluster_count == 0){
    //    copter_found = false;
    //}

    ROS_INFO("number of clusters processed: %d", cluster_count);
/*
    if (!shared_clusters_finalists_reading) {
        shared_clusters_finalists_writing = true;
        shared_clusters_finalists = clusters_finalists;
        //shared_centers_3d_finalists = centers_3d_finalists;
        shared_clusters_finalists_writing = false;
        finalists_count_sent++;
        ROS_INFO("sent new finalists: %d", finalists_count_sent);
    }
*/
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pcloud_filtered, output);

    // Publish the data
    pub_cloud.publish(output); //TODO when should this be published, if at all?

    //only publish the center when a copter has been found
    //if (copter_found){
        /*
        geometry_msgs::Point msg_3d;
        msg_3d.x = copter_center.x;
        msg_3d.y = copter_center.y;
        msg_3d.z = copter_center.z;

        pub_copter_center_3d.publish(msg_3d);

        geometry_msgs::PointStamped msg_3d_s;
        msg_3d_s.header.frame_id = "static_stereo_cam_left";
        msg_3d_s.point = msg_3d;
        pub_copter_center_stamped3d.publish(msg_3d_s);
        */
    //}







    //TODO test if bluring yeilds better results
    //cv::GaussianBlur(gray_img, gray_img, cv::Size(3,3), 3, cv::BORDER_DEFAULT);

/*
    if (!shared_clusters_finalists_writing){
        shared_clusters_finalists_reading = true;
        clusters_finalists_rgb = shared_clusters_finalists;
        //centers_3d_finalists_rgb = shared_centers_3d_finalists;
        shared_clusters_finalists_reading = false;
    }
    else {
        ROS_INFO("finalists not ready");
    }
*/

    clusters_finalists_rgb = clusters_finalists;
    if (clusters_finalists_rgb.size() == 0){
        //TODO publish gray image and return
        ROS_INFO("No finalists. Copter not found!");
    }

    int numCopters = 0;
    //go through each finalist and determine if it "looks" like our copter
    for (int i = 0; i < clusters_finalists_rgb.size(); i++){
        int row_min, row_max, col_min, col_max;
        getBoundingRect(clusters_finalists_rgb[i], row_min, row_max, col_min, col_max);

        //make box bigger by BOUNDING_RECT_SCALE_FACTOR ammount
        cv::Rect rect_adjusted = getAdjustedRect(row_min, row_max, col_min, col_max, BOUNDING_RECT_SCALE_FACTOR);
        if (rect_adjusted.area() <= 0) continue; //test for bad points

        //then resize to 40x100
        //cv::Rect box(col_min, row_min, col_max-col_min, row_max-row_min);

        img_cropped = gray_img(rect_adjusted);
        cv::resize(img_cropped, img_cropped_resized, cv::Size(CASCADE_WIDTH, CASCADE_HEIGHT));

        //if training, step through images and label as positive or negative
        if (TRAINING_CASCADE){
            cv::waitKey(1);

            cv::rectangle(rgb_image, rect_adjusted, cv::Scalar(0,0,255), 4);

            cv::imshow("cropped iamge", img_cropped_resized);
            cv::imshow("left_camera: training_window", rgb_image);

            std::cout << "Label image positive (p), negative (n), else skip: " << std::endl;

            char buffer[10];
            int key;
            //FOR TRAINING CASCADE CLASSIFIER
            std::string filename;
            key = cv::waitKey(0);
            //if positive, save as positive
            if (key == 112){
                numPos++;
                sprintf(buffer, "%d", numPos);
                filename = std::string(TRAIN_PATH_POS) + std::string(buffer) + std::string(".pgm");
                ROS_INFO("numPos: %d", numPos);
                try {
                        cv::imwrite(filename, img_cropped_resized);
                    }
                    catch (std::runtime_error& ex) {
                        ROS_INFO("Exception converting image to format: %s", ex.what());

                    }
                /*
                if (cv::imwrite(filename, *it))
                {
                    ROS_INFO("saved positive image number: %d", numPos);
                }
                else {
                    ROS_INFO("error saving image");
                }*/

            }
            //if negative, save as negative
            else if (key == 110){
                numNeg++;
                sprintf(buffer, "%d", numNeg);
                filename = std::string(TRAIN_PATH_NEG) + std::string(buffer) + std::string(".pgm");
                ROS_INFO("numNeg: %d", numNeg);
                try {
                        cv::imwrite(filename, img_cropped_resized);
                    }
                    catch (std::runtime_error& ex) {
                        ROS_INFO("Exception converting image to format: %s", ex.what());

                    }
                /*
                if (cv::imwrite(filename, *it))
                {
                    ROS_INFO("saved negative image number: %d", numNeg);
                }
                else {
                    ROS_INFO("error saving image");
                }*/
            }
        }
        //if not training, use existing classifier to check
        if (!TRAINING_CASCADE && use_classifier){

            //TODO assign copter found. but shared to adjust roi
            if (isCopter(img_cropped_resized, copter_classifier)){
                //publish center
                copter_center = calcCenter(clusters_finalists_rgb[i]);
                //pcl::PointXYZ copter_center_rgb = copter_center;

                geometry_msgs::Point msg_3d;
                msg_3d.x = copter_center.x;
                msg_3d.y = copter_center.y;
                msg_3d.z = copter_center.z;

                pub_copter_center_3d.publish(msg_3d);

                geometry_msgs::PointStamped msg_3d_s;
                msg_3d_s.header.frame_id = "static_stereo_cam_left";
                msg_3d_s.point = msg_3d;
                pub_copter_center_stamped3d.publish(msg_3d_s);

                //box copter in orig image
                cv::rectangle(rgb_image, rect_adjusted, cv::Scalar(0,0,255), 4);


                numCopters++;
                if (numCopters > 1){
                    ROS_INFO("WARNING: More than one copter was found in the scene!");
                }

                //copter was found
                copter_found = true;
                /*
                if (!shared_copter_found_reading){
                    shared_copter_found_writing = true;
                    shared_copter_found = true;
                    shared_copter_found_writing = false;
                }*/
            }


        }

        //if not using classifier, then publish all the finalist's centers and box original image
        if (!TRAINING_CASCADE && !use_classifier){
            //publish center
            pcl::PointXYZ copter_center_rgb = calcCenter(clusters_finalists_rgb[i]);

            geometry_msgs::Point msg_3d;
            msg_3d.x = copter_center_rgb.x;
            msg_3d.y = copter_center_rgb.y;
            msg_3d.z = copter_center_rgb.z;

            pub_copter_center_3d.publish(msg_3d);

            geometry_msgs::PointStamped msg_3d_s;
            msg_3d_s.header.frame_id = "static_stereo_cam_left";
            msg_3d_s.point = msg_3d;
            pub_copter_center_stamped3d.publish(msg_3d_s);

            //box copter in orig image
            cv::rectangle(rgb_image, rect_adjusted, cv::Scalar(0,0,255), 4);


            numCopters++;
            if (numCopters > 1){
                ROS_INFO("WARNING: More than one copter was found in the scene. Consider using a classifier!");
            }

            //copter was found
            copter_found = true;
            /*
            if (!shared_copter_found_reading){
                shared_copter_found_writing = true;
                shared_copter_found = true;
                shared_copter_found_writing = false;
            }*/

        }

    }

    cv::waitKey(1);
    cv::imshow("left_camera: published copter location", rgb_image);

    //cv_bridge::CvImage rgb_image_bridge(rgb_image, sensor_msgs::image_encodings::TYPE_8UC3);
    //image_pub.publish(rgb_image_bridge.toImageMsg());
    sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
    image_pub.publish(msg_rgb);









    cluster_indices.clear();
    clusters_finalists.clear();

    if (USING_VISUALIZER){
        pviewer->spinOnce();
        pviewer->removeAllPointClouds();
    }

}

void CopterTrackerV2::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    callback_cloud_num++;
    ROS_INFO("got cloud callback: %d", callback_cloud_num);

    //check if copter was previously found: for roi adjustments
    if (!shared_copter_found_writing){
        shared_copter_found_reading = true;
        copter_found = shared_copter_found;
        shared_copter_found_reading = false;
    }

    // Convert to PCL data type
    pcl::fromROSMsg(*input, *pcloud);


    if (copter_found){
        roi_size_penalty_multiplier = 1; //use normal ROI size
        cleared_for_roi = true;

        if (TRAINING_SAVING_PCD){
            std::cout << "Do you wish to save the cluster? (y/n)" << std::endl;

            if (getchar() == 'y'){
                char buffer[10];
                saveNum++;

                sprintf(buffer, "%d", saveNum);
                std::string filename = std::string(TRAINING_PCD_FILE_PATH) + std::string(buffer) + std::string(".pcd");

                ROS_INFO("Saving Cluster to %s", filename.c_str());
                pcl::io::savePCDFileASCII (filename.c_str(), *pcloud_copter_cand);
            }
        }
    }
    else {
        //increase ROI when copter is not found
        roi_size_penalty_multiplier += roi_size_penalty_multiplier_increment;
    }

    //TODO filter cloud based on copter's altitude
    //TODO reset roi (cleared_for_roi = false) when copter's velocity doesn't match object's velocity... maybe not. wind drift may cause movement or cancel movement
    //TODO filter based on 3D shape
    //TODO filter on appaerance (cascade classifier)
    //TODO visualize the ROI addCube (last one in doc)
    //TODO filter if object is on wall or floor. (label large planes with 3 points) (from kinect cloud)

    //TODO don't use ROI when stuck on a static false positive (update cleared_for_roi) (UPDATE: maybe not... when moving, nothing is static)
    //TODO sometimes copter is detected way far back because of the shadow errors and thus the roi is set far away. fix these false positives or do initial thresh on z axis (too close or too far)
    if (roi_type == ROI_DYNAMIC){
        //if using roi and a copter has been previously found (applies ROI around copter's center)
        if (!TRAINING_CASCADE && use_roi && cleared_for_roi){
            applyDynamicROI3D(pcloud, pcloud, pcloud_roi_out, copter_center, g_copter_area_max, roi_size_multiplier, roi_size_penalty_multiplier, pviewer);
        }
    }
    else if (roi_type == ROI_STATIC){
        if (!TRAINING_CASCADE && use_roi && copter_found){
            applyDynamicROI3D(pcloud, pcloud, pcloud_roi_out, copter_center, g_copter_area_max, roi_size_multiplier, 1, pviewer);
        }
    }
    //copter_found = false;
    //copter was found
    if (!shared_copter_found_reading){
        shared_copter_found_writing = true;
        shared_copter_found = false;
        shared_copter_found_writing = false;
    }


    //remove NaN from input cloud (all non features are NaN hence PCD is 640x480 points)
    pcl::removeNaNFromPointCloud(*pcloud, *pcloud, indecies_map);

    //sample (filter) point cloud to improve speed
    if (use_voxel_filter){

        voxel_grid.setInputCloud(pcloud);
        voxel_grid.filter(*pcloud_filtered);
    }
    else {
        *pcloud_filtered = *pcloud;
    }

    //check for empty pcd
    if (pcloud_filtered->size() == 0){
        return;
    }

    tree->setInputCloud(pcloud_filtered);


    cluster_extractor.setInputCloud (pcloud_filtered);
    cluster_extractor.extract (cluster_indices);


    int cluster_count = 0;
    int cluster_state;
    //iterate through all extracted clusters
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);

        //iterate throguh the indicies of the cluster
        //populate cluster with the data from the point cloud at the indicies of the cluster
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cluster->points.push_back (pcloud_filtered->points[*pit]); //*

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;


        char cluster_id[10];
        sprintf(cluster_id, "cloud%d", cluster_count);

        //TODO min and max not globals... accessed in loop. #define or make reconfig callback a class function
        cluster_state = filterByShapeAndSize(cluster, g_copter_area_min, g_copter_area_max, SHAPE_THRESH, cluster_id);

        if (USING_VISUALIZER){
            if (vis_method == VIS_METH_ALL_CLUSTERS){
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color = getClusterColor(pcloud_filtered, cluster_count);
                pviewer->addPointCloud<pcl::PointXYZ> (cluster, single_color, cluster_id);
            }
            else if (vis_method == VIS_METH_CLUSTER_STATE){
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color = getClusterColorState(pcloud_filtered, cluster_state);
                pviewer->addPointCloud<pcl::PointXYZ> (cluster, single_color, cluster_id);
            }
            else if (VIS_METH_OFF){
                //do not display anything
            }
        }

/*
        //view the clusters //TODO make bad clusters red and good green (also yellow and white)
        if (cluster_meets_shape_and_size){
            ROS_INFO("%s meets the shape and size requirement", cluster_id);
            //add final candidates to the viewer
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color = getClusterColor(pcloud_filtered, cluster_count);
            pviewer->addPointCloud<pcl::PointXYZ> (cluster, single_color, cluster_id);
        }
*/
        if (cluster_state == CLUSTER_STATE_GOOD_AREA_GOOD_SHAPE){
            copter_center = calcCenter(cluster);
            //copter_found = true;
            //TODO add to vector of cluster copter candidates
            clusters_finalists.push_back(cluster);
            pcloud_copter_cand = cluster;
            int row_min, row_max, col_min, col_max;
            getBoundingRect(pcloud_copter_cand, row_min, row_max, col_min, col_max);
        }
        else {
            //copter_found = false;
        }
        cluster_count++;


    }
    //copter is not found if no clusters were processed
    //if (cluster_count == 0){
    //    copter_found = false;
    //}

    ROS_INFO("number of clusters processed: %d", cluster_count);

    if (!shared_clusters_finalists_reading) {
        shared_clusters_finalists_writing = true;
        shared_clusters_finalists = clusters_finalists;
        //shared_centers_3d_finalists = centers_3d_finalists;
        shared_clusters_finalists_writing = false;
        finalists_count_sent++;
        ROS_INFO("sent new finalists: %d", finalists_count_sent);
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pcloud_filtered, output);

    // Publish the data
    pub_cloud.publish(output); //TODO when should this be published, if at all?

    //only publish the center when a copter has been found
    //if (copter_found){
        /*
        geometry_msgs::Point msg_3d;
        msg_3d.x = copter_center.x;
        msg_3d.y = copter_center.y;
        msg_3d.z = copter_center.z;

        pub_copter_center_3d.publish(msg_3d);

        geometry_msgs::PointStamped msg_3d_s;
        msg_3d_s.header.frame_id = "static_stereo_cam_left";
        msg_3d_s.point = msg_3d;
        pub_copter_center_stamped3d.publish(msg_3d_s);
        */
    //}

    cluster_indices.clear();
    clusters_finalists.clear();

    if (USING_VISUALIZER){
        pviewer->spinOnce();
        pviewer->removeAllPointClouds();
    }

    //ros::Duration(2).sleep();

}

void CopterTrackerV2::callback_rgb(const sensor_msgs::ImageConstPtr& msg)
{
    callback_rgb_num++;
    ROS_INFO("got rgb image: %d", callback_rgb_num);
    cv_bridge::CvImagePtr mono_orig;


    try {
        mono_orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mono_delayed;
    mono_delayed = mono_orig->image; //remove this
/*
    if (!TRAINING_CASCADE){
        //use an older image to match the time the point cloud was taken
        image_queue.push(mono_orig->image);
        if (image_queue.size() < NUM_FRAMES_DELAYED){
            return;
        }

        mono_delayed = image_queue.front();
        image_queue.pop();
    }
    else {
        mono_delayed = mono_orig->image;
    }
*/
    cv::Mat rgb_image(mono_delayed.rows, mono_delayed.cols, CV_8UC3);
    cvtColor(mono_delayed, rgb_image, CV_GRAY2RGB);

    cv::Mat img_cropped;
    cv::Mat img_cropped_resized;

    cv::Mat gray_img;
    //cvtColor(rgb_orig->image, gray_img, CV_RGB2GRAY);
    gray_img = mono_delayed; //TODO consider copy here

    //TODO test if bluring yeilds better results
    //cv::GaussianBlur(gray_img, gray_img, cv::Size(3,3), 3, cv::BORDER_DEFAULT);


    if (!shared_clusters_finalists_writing){
        shared_clusters_finalists_reading = true;
        clusters_finalists_rgb = shared_clusters_finalists;
        //centers_3d_finalists_rgb = shared_centers_3d_finalists;
        shared_clusters_finalists_reading = false;
    }
    else {
        ROS_INFO("finalists not ready");
    }

    if (clusters_finalists_rgb.size() > 0){
        finalists_count_received++;
        ROS_INFO("received new finalists: %d", finalists_count_received);
    }

    int numCopters = 0;
    //go through each finalist and determine if it "looks" like our copter
    for (int i = 0; i < clusters_finalists_rgb.size(); i++){
        int row_min, row_max, col_min, col_max;
        getBoundingRect(clusters_finalists_rgb[i], row_min, row_max, col_min, col_max);

        //make box bigger by BOUNDING_RECT_SCALE_FACTOR ammount
        cv::Rect rect_adjusted = getAdjustedRect(row_min, row_max, col_min, col_max, BOUNDING_RECT_SCALE_FACTOR);
        if (rect_adjusted.area() <= 0) continue; //test for bad points

        //then resize to 40x100
        //cv::Rect box(col_min, row_min, col_max-col_min, row_max-row_min);

        img_cropped = gray_img(rect_adjusted);
        cv::resize(img_cropped, img_cropped_resized, cv::Size(CASCADE_WIDTH, CASCADE_HEIGHT));



        //if training, step through images and label as positive or negative
        if (TRAINING_CASCADE){
            cv::waitKey(1);

            cv::rectangle(rgb_image, rect_adjusted, cv::Scalar(0,0,255), 4);

            cv::imshow("cropped iamge", img_cropped_resized);
            cv::imshow("left_camera: training_window", rgb_image);

            std::cout << "Label image positive (p), negative (n), else skip: " << std::endl;

            char buffer[10];
            int key;
            //FOR TRAINING CASCADE CLASSIFIER
            std::string filename;
            key = cv::waitKey(0);
            //if positive, save as positive
            if (key == 112){
                numPos++;
                sprintf(buffer, "%d", numPos);
                filename = std::string(TRAIN_PATH_POS) + std::string(buffer) + std::string(".pgm");
                ROS_INFO("numPos: %d", numPos);
                try {
                        cv::imwrite(filename, img_cropped_resized);
                    }
                    catch (std::runtime_error& ex) {
                        ROS_INFO("Exception converting image to format: %s", ex.what());

                    }
                /*
                if (cv::imwrite(filename, *it))
                {
                    ROS_INFO("saved positive image number: %d", numPos);
                }
                else {
                    ROS_INFO("error saving image");
                }*/

            }
            //if negative, save as negative
            else if (key == 110){
                numNeg++;
                sprintf(buffer, "%d", numNeg);
                filename = std::string(TRAIN_PATH_NEG) + std::string(buffer) + std::string(".pgm");
                ROS_INFO("numNeg: %d", numNeg);
                try {
                        cv::imwrite(filename, img_cropped_resized);
                    }
                    catch (std::runtime_error& ex) {
                        ROS_INFO("Exception converting image to format: %s", ex.what());

                    }
                /*
                if (cv::imwrite(filename, *it))
                {
                    ROS_INFO("saved negative image number: %d", numNeg);
                }
                else {
                    ROS_INFO("error saving image");
                }*/
            }
        }
        //if not training, use existing classifier to check
        if (!TRAINING_CASCADE && use_classifier){

            //TODO assign copter found. but shared to adjust roi
            if (isCopter(img_cropped_resized, copter_classifier)){
                //publish center
                pcl::PointXYZ copter_center_rgb = calcCenter(clusters_finalists_rgb[i]);

                geometry_msgs::Point msg_3d;
                msg_3d.x = copter_center_rgb.x;
                msg_3d.y = copter_center_rgb.y;
                msg_3d.z = copter_center_rgb.z;

                pub_copter_center_3d.publish(msg_3d);

                geometry_msgs::PointStamped msg_3d_s;
                msg_3d_s.header.frame_id = "static_stereo_cam_left";
                msg_3d_s.point = msg_3d;
                pub_copter_center_stamped3d.publish(msg_3d_s);

                //box copter in orig image
                cv::rectangle(rgb_image, rect_adjusted, cv::Scalar(0,0,255), 4);


                numCopters++;
                if (numCopters > 1){
                    ROS_INFO("WARNING: More than one copter was found in the scene!");
                }

                //copter was found
                if (!shared_copter_found_reading){
                    shared_copter_found_writing = true;
                    shared_copter_found = true;
                    shared_copter_found_writing = false;
                }
            }


        }

        //if not using classifier, then publish all the finalist's centers and box original image
        if (!TRAINING_CASCADE && !use_classifier){
            //publish center
            pcl::PointXYZ copter_center_rgb = calcCenter(clusters_finalists_rgb[i]);

            geometry_msgs::Point msg_3d;
            msg_3d.x = copter_center_rgb.x;
            msg_3d.y = copter_center_rgb.y;
            msg_3d.z = copter_center_rgb.z;

            pub_copter_center_3d.publish(msg_3d);

            geometry_msgs::PointStamped msg_3d_s;
            msg_3d_s.header.frame_id = "static_stereo_cam_left";
            msg_3d_s.point = msg_3d;
            pub_copter_center_stamped3d.publish(msg_3d_s);

            //box copter in orig image
            cv::rectangle(rgb_image, rect_adjusted, cv::Scalar(0,0,255), 4);


            numCopters++;
            if (numCopters > 1){
                ROS_INFO("WARNING: More than one copter was found in the scene. Consider using a classifier!");
            }

            //copter was found
            if (!shared_copter_found_reading){
                shared_copter_found_writing = true;
                shared_copter_found = true;
                shared_copter_found_writing = false;
            }

        }

    }

    cv::waitKey(1);
    cv::imshow("left_camera: published copter location", rgb_image);

    //cv_bridge::CvImage rgb_image_bridge(rgb_image, sensor_msgs::image_encodings::TYPE_8UC3);
    //image_pub.publish(rgb_image_bridge.toImageMsg());
    sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
    image_pub.publish(msg_rgb);


}

void CopterTrackerV2::applyDynamicROI3D(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud,
                typename pcl::PointCloud< pcl::PointXYZ >::Ptr &cloud_inside_roi,
                typename pcl::PointCloud< pcl::PointXYZ >::Ptr &cloud_outside_roi,
                const pcl::PointXYZ & copter_center, const float copter_area_max, const float roi_local_multiplier, const float roi_size_penality, pcl::visualization::PCLVisualizer* pviewer){

    float x_min, x_max, y_min, y_max, z_min, z_max;
    float side_length = roi_local_multiplier*sqrt(copter_area_max)*roi_size_penality; //penality is 1 when copter is found. it increases when not found
    float half_length = side_length/2;

    x_min = copter_center.x - half_length;
    x_max = copter_center.x + half_length;
    y_min = copter_center.y - half_length;
    y_max = copter_center.y + half_length;
    z_min = copter_center.z - half_length;
    z_max = copter_center.z + half_length;

    //pcl::PassThrough<PointType> ptfilter (true); // Initializing with true will allow us to extract the removed indices

    //passThrough_filter.setNegative(true); //alows us to extract removed indicies
    passThrough_filter.setInputCloud (cloud);
    passThrough_filter.setFilterFieldName ("z");
    passThrough_filter.setFilterLimits (z_min, z_max);
    passThrough_filter.filter(*cloud_inside_roi);

    passThrough_filter.setInputCloud (cloud_inside_roi);
    passThrough_filter.setFilterFieldName ("x");
    passThrough_filter.setFilterLimits (x_min, x_max);
    passThrough_filter.filter(*cloud_inside_roi);

    passThrough_filter.setInputCloud (cloud_inside_roi);
    passThrough_filter.setFilterFieldName ("y");
    passThrough_filter.setFilterLimits (y_min, y_max);
    passThrough_filter.filter(*cloud_inside_roi);

    if (USING_VISUALIZER){
        //draw a cube as the roi
        pviewer->removeShape("cube");
        pviewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max, 255, 255, 255);
    }


    // The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
    //*cloud_outside_roi = passThrough_filter.getRemovedIndices ();
/*
    // The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
    // and also indexes all non-finite points of cloud_in
    ptfilter.setIndices (indices_x);
    ptfilter.setFilterFieldName ("z");
    ptfilter.setFilterLimits (-10.0, 10.0);
    ptfilter.setNegative (true);
    ptfilter.filter (*indices_xz);
    // The indices_xz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 10.0 or smaller than -10.0
    ptfilter.setIndices (indices_xz);
    ptfilter.setFilterFieldName ("intensity");
    ptfilter.setFilterLimits (FLT_MIN, 0.5);
    ptfilter.setNegative (false);
    ptfilter.filter (*cloud_out);
    // The resulting cloud_out contains all points of cloud_in that are finite and have:
    // x between 0.0 and 1000.0, z larger than 10.0 or smaller than -10.0 and intensity smaller than 0.5.
*/
}

float CopterTrackerV2::calcApproxSurfaceArea(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud)
{
    float area;
    float min_x, max_x, min_y, max_y;
    float min_x_cand = MAX_INT;
    float max_x_cand = MIN_INT;
    float min_y_cand = MAX_INT;
    float max_y_cand = MIN_INT;

    //std::vector<pcl::PointXYZ> data = cloud->points;
    ROS_INFO("first point x: %lf", cloud->points[0].x);
/*
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); it++){

    }
*/
    float x, y, z;
    for (int i = 0; i < cloud->size(); i++){
        x = cloud->points[i].x;
        y = cloud->points[i].y;
        if (x < min_x_cand) min_x_cand = x;
        if (x > max_x_cand) max_x_cand = x;
        if (y < min_y_cand) min_y_cand = y;
        if (y > max_y_cand) max_y_cand = y;
    }

    area = (max_x_cand - min_x_cand)*(max_y_cand - min_y_cand);

    //for ()
    return area;
}

pcl::PointXYZ CopterTrackerV2::calcCenter(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud){

    float x_total = 0;
    float y_total = 0;
    float z_total = 0;
    for (int i = 0; i < cloud->size(); i++){
        x_total += cloud->points[i].x;
        y_total += cloud->points[i].y;
        z_total += cloud->points[i].z;
    }

    pcl::PointXYZ center(x_total/cloud->size(), y_total/cloud->size(), z_total/cloud->size());
    return center;

}

void CopterTrackerV2::createAndWriteModelVFH(std::string load_pcd_dir, std::string write_model_dir)
{
    /*
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_vfh (new pcl::search::KdTree<pcl::PointXYZ> ());
    vfh.setSearchMethod(tree_vfh);
    
    std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> vfhs_vector;
    
    ROS_INFO("construcor called.\ncreateAndWriteModelVFH!");
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcds;
    //load PCD files
    std::string extension = ".pcd";
    loadPCDs(load_pcd_dir, extension, pcds);
    ROS_INFO("number of PCDs read in: %lu\n\n\n\n\n", pcds.size());
    //find normals
    std::vector<pcl::PointCloud<pcl::Normal>::Ptr> normals;
    computeNormals(pcds, normals);
    ROS_INFO("normals clouds calculated: %lu\n\n\n\n\n", normals.size());
    
    //compute VFH signatures for the model
    //find vfh for each cloud and normal set
    for (int i = 0; i < pcds.size(); i++){
        vfh.setInputCloud(pcds[i]);
        vfh.setInputNormals(normals[i]);

        pcl::PointCloud<pcl::VFHSignature308>::Ptr pcloud_vfh (new pcl::PointCloud<pcl::VFHSignature308> ());
        vfh.compute(*pcloud_vfh);

        vfhs_vector.push_back(pcloud_vfh);
    }
    
    //populate each model with the signatures
    std::vector<vfh_model> models;
    std::vector <pcl::PCLPointField> fields;
    for (int i = 0; i < vfhs_vector.size(); i++){

        vfh_model model;
        model.second.resize(308);

        //get fields of each vfh point cloud
        pcl::getFieldIndex (vfhs_vector[i], "vfh", fields);

        for (int j = 0; j < fields[vfh_idx].count; ++j){
            model.second[j] = vfhs_vector[j].points[0].histogram[j];
        }
        model.first = std::string("TODO");

    }
    
    
    //write model to disk

*/
}

/** \brief returns the center of the "true" copter. filters out false positives via cascade classifier. designed to be called in a loop
  * \param cropppedFinalists: cropped images of copter candidates to be passed into classifier
  * \param copter_rects: bounding rects of the remaining candidates: used to draw red box in original image
  * \param copter_classifier: the cascade classifier used to filter
  */
bool CopterTrackerV2::isCopter(cv::Mat &croppedFinalist, cv::CascadeClassifier &copter_classifier){ //, Ptr<FeatureEvaluator>& feval

    //if (croppedFinalists.empty()) { return cv::Point3f(0,0,0); }

    std::vector<cv::Rect> results;
    copter_classifier.detectMultiScale(croppedFinalist, results, 1.1);

    if (!results.empty()){

        for (std::vector<cv::Rect>::iterator it_results = results.begin() ; it_results != results.end(); ++it_results){

            //add the image cropped at the rectangle
            //copter_rects.push_back(*it_results);
            //ROS_INFO("copter found!");

        }
        //if any copters are found within the croppedFinalist, then the image is of a copter
        results.clear();
        return true;
    }
    else {
        return false;
    }

}

void CopterTrackerV2::getBoundingRect(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud, int &row_min, int &row_max, int &col_min, int &col_max){

    Eigen::Vector3f left, right, top, bottom;

    float x, y, z, min_x, max_x, min_y, max_y;
    min_x = MAX_INT;
    max_x = -MAX_INT;
    min_y = MAX_INT;
    max_y = -MAX_INT;

    for (int i = 0; i < cloud->size(); i++){
        x = cloud->points[i].x;
        y = cloud->points[i].y;
        z = cloud->points[i].z;
        if (x < min_x) {
            min_x = x;
            left << x, y, z;
        }
        if (x > max_x) {
            max_x = x;
            right << x, y, z;
        }
        if (y < min_y) {
            min_y = y;
            top << x, y, z;
        }
        if (y > max_y) {
            max_y = y;
            bottom << x, y, z;
        }
    }

    //TODO use Eigen
    //use camera_projection_matrix of left camera to project 3D points to 2D image points
    //use rectification matrix to rectify points to allign with rectified image stream
    //col_min = indecies_map[]
    Eigen::MatrixXf camera_calibration_matrix(3,3);
    camera_calibration_matrix << 791.328269, 0.0, 296.619217, 0.0, 791.328269, 251.0756, 0.0, 0.0, 1.0;
    Eigen::MatrixXf camera_rect_matrix(3,3);
    camera_rect_matrix << 0.999292, -0.003632, 0.037436, 0.003154, 0.9999129999999999, 0.012837, -0.037479, -0.01271, 0.9992169999999999;

    //TODO use rectificaiton matrrix here
    Eigen::Vector3f left3 = camera_calibration_matrix*left;
    Eigen::Vector3f right3 = camera_calibration_matrix*right;
    Eigen::Vector3f bottom3 = camera_calibration_matrix*bottom;
    Eigen::Vector3f top3 = camera_calibration_matrix*top;
/*
    left3 = camera_rect_matrix*left3;
    right3 = camera_rect_matrix*right3;
    top3 = camera_rect_matrix*top3;
    bottom3 = camera_rect_matrix*bottom3;
*/
/*
    top3 << top3(0)/top3(2), top3(1)/top3(2), 1;
    bottom3 << bottom3(0)/bottom3(2), bottom3(1)/bottom3(2), 1;
    left3 << left3(0)/left3(2), left3(1)/left3(2), 1;
    right3 << right3(0)/right3(2), right3(1)/right3(2), 1;

    top3 = camera_rect_matrix*top3;
    bottom3 = camera_rect_matrix*bottom3;
    left3 = camera_rect_matrix*left3;
    right3 = camera_rect_matrix*right3;
*/
    row_min = top3(1)/top3(2);
    row_max = bottom3(1)/bottom3(2);
    col_min = left3(0)/left3(2);
    col_max = right3(0)/right3(2);

//    ROS_INFO("Copter left 2D_homo: (%lf, %lf, %lf)", left3(0), left3(1), left3(2));
//    ROS_INFO("Copter center 2D: (%d, %d)", (col_min+col_max)/2, (row_min+row_max)/2);

    //Eigen::Vector2f right2 = right3/right3(2);
    //Eigen::Vector2f top2 = top3/top3(2);
    //Eigen::Vector2f bottom2 = bottom3/bottom3(2);

}


void CopterTrackerV2::loadPCDs (const boost::filesystem::path &base_dir, std::string &extension,
                   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &pcds)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      ROS_INFO ("Loading %s (%lu point clouds loaded so far).\n", ss.str ().c_str (), (unsigned long)pcds.size ());

      loadPCDs (it->path (), extension, pcds);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      //vfh_model m;
      //if (loadHist (base_dir / it->path ().filename (), m))
      // models.push_back (m);

      pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_curr(new pcl::PointCloud<pcl::PointXYZ>);
      boost::filesystem::path thePath = it->path();

      std::string fullPath = std::string(thePath.string());// + extension;

      if (pcl::io::loadPCDFile<pcl::PointXYZ> (fullPath.c_str(), *pcd_curr) != -1){
          pcds.push_back(pcd_curr);
      }
    }
  }
}

void CopterTrackerV2::computeNormals(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &pcds, std::vector<pcl::PointCloud<pcl::Normal>::Ptr> &normals)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03); //TODO adjust this value

    //computer normals for every cloud
    //iterate through the clouds
    for (int i = 0; i < pcds.size(); i++){
        pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>());\

        ne.setInputCloud(pcds[i]);
        ne.compute(*normal);

        normals.push_back(normal);
    }

}

//filter the cloud. calculates the area and shape of the cluster in one pass
int CopterTrackerV2::filterByShapeAndSize(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cluster_cand, const float area_min, const float area_max, const float shapeThresh, const char* cluster_id){

    //TODO calc surface area including z coordinate
    /********************CALC APPROX SURFAC AREA*****************/
    float area;
    float min_x = MAX_INT;
    float max_x = MIN_INT;
    float min_y = MAX_INT;
    float max_y = MIN_INT;

    float x, y, z;
    for (int i = 0; i < cluster_cand->size(); i++){
        x = cluster_cand->points[i].x;
        y = cluster_cand->points[i].y;
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }

    area = (max_x - min_x)*(max_y - min_y);
    ROS_INFO("%s has approx surface area of: %lf", cluster_id, area);

    //if the area of the cluster meets the criteria, proceed to filter by shape
    if (area > area_min && area < area_max){

        /***********Find the shape*********/
        //if candidate is too long in a dimension, it is not the copter
        float shapeFactor = (max_x - min_x) > (max_y - min_y) ? (max_x - min_x)/(max_y - min_y) : (max_y - min_y)/(max_x - min_x);
        if (shapeFactor > shapeThresh) return CLUSTER_STATE_GOOD_AREA_BAD_SHAPE;

        return CLUSTER_STATE_GOOD_AREA_GOOD_SHAPE;

    }
    //the cluster is not the copter
    else {
        return CLUSTER_STATE_BAD_AREA;
    }

}
//TODO return white if outside of ROI,
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> CopterTrackerV2::getClusterColor(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud, int index)
{
    if (index == 0){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pcloud_filtered, 0, 255, 0);
        return color;
    }
    else if (index == 1){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pcloud_filtered, 0, 0, 255);
        return color;
    }
    else if (index == 2){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pcloud_filtered, 255, 0, 0);
        return color;
    }
    else if (index == 3){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pcloud_filtered, 255, 255, 0);
        return color;
    }
    else if (index == 4){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pcloud_filtered, 0, 255, 255);
        return color;
    }
    else if (index == 5){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pcloud_filtered, 255, 0, 255);
        return color;
    }
    else if (index == 6){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pcloud_filtered, 255, 128, 0);
        return color;
    }
    else {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pcloud_filtered, 255, 255, 255);
        return color;
    }


}

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> CopterTrackerV2::getClusterColorState(const typename pcl::PointCloud< pcl::PointXYZ >::ConstPtr &cloud, int state){

    if (state == CLUSTER_STATE_BAD_AREA){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 255, 0, 0);
        return color;
    }
    else if (state == CLUSTER_STATE_GOOD_AREA_BAD_SHAPE){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 255, 255, 0);
        return color;
    }
    else if (state == CLUSTER_STATE_GOOD_AREA_GOOD_SHAPE){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0, 255, 0);
        return color;
    }

}

cv::Rect CopterTrackerV2::getAdjustedRect(const int row_min, const int row_max, const int col_min, const int col_max, const float scale){

    int height = row_max-row_min;
    int width = col_max-col_min;

    //adjust to mantain cropped aspect ratio
    int height_cand = (float(CASCADE_HEIGHT)/CASCADE_WIDTH)*width;
    int width_cand = (float(CASCADE_WIDTH)/CASCADE_HEIGHT)*height;

    int height_adjusted, width_adjusted;

    //pick the dimension to be adjusted: only incease a side
    if ((height_cand-height) > 0){
        height_adjusted = height_cand*scale;
        width_adjusted = width*scale;
    }
    else {
        height_adjusted = height*scale;
        width_adjusted = width_cand*scale;
    }



    int top = row_min-(height_adjusted-height)/2;
    int left = col_min-(width_adjusted-width)/2;

    //check for box being out of bounds of image
    if (top < 0){
        //shift down
        top -= top;
    }
    if (left < 0){
        //shift right
        left -= left;
    }
    if (top + height_adjusted >= RGB_HEIGHT){
        //shift up
        top -= ((top+height_adjusted)-(RGB_HEIGHT-1));
    }
    if (left + width_adjusted >= RGB_WIDTH){
        //shift left
        left -= (left+width_adjusted-(RGB_WIDTH-1));
    }

    //check for width or height being bigger than the image
    //height_adjusted >= RGB_HEIGHT || width_adjusted >= RGB_WIDTH || height_adjusted == 0 || width_adjusted == 0)
    if (left < 0 || width_adjusted < 0 || (left+width_adjusted) >= RGB_WIDTH || top < 0 || height_adjusted < 0 || (top+height_adjusted) >= RGB_HEIGHT)
    {
        cv::Rect empty(0,0,0,0);
        return empty;
    }


    cv::Rect rect(left, top, width_adjusted, height_adjusted);
    return rect;
}


//TODO learn how to put this as a class member function to have access to member variables. will avoid globals
void callback_reconfig(leopard_imaging_camera::copterTrackerV2Config &config, uint32_t level) {
    /*
    ROS_INFO("Reconfigure Request: %d %f %s %s %d",
            config.int_param, config.voxel_leaf_size,
            config.str_param.c_str(),
            config.use_voxel_filter?"True":"False",
            config.size);
    */

    vox_leaf_size = config.voxel_leaf_size;
    use_voxel_filter = config.use_voxel_filter;
    pvoxel_grid->setLeafSize(vox_leaf_size, vox_leaf_size, vox_leaf_size);
    cluster_size_min = config.cluster_size_min;
    cluster_size_max = config.cluster_size_max;

    cluster_tolerance = config.cluster_tolerance;
    pcluster_extractor->setClusterTolerance(cluster_tolerance);
    pcluster_extractor->setMinClusterSize(cluster_size_min);
    pcluster_extractor->setMaxClusterSize(cluster_size_max);

    g_copter_area_min = config.copter_area_min;
    g_copter_area_max = config.copter_area_max;

    vis_method = config.VisualizationMode;
    roi_size_multiplier = config.ROI_size_multiplier;
    use_roi = config.use_ROI;
    roi_size_penalty_multiplier_increment = config.ROI_size_penality_multiplier_increment;

    roi_type = config.roiType;
    use_classifier = config.use_classifier;

}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "clustering_node");
    ros::NodeHandle nh;

    CopterTrackerV2 copterTrackerV2(nh);

    dynamic_reconfigure::Server<leopard_imaging_camera::copterTrackerV2Config> server;
    dynamic_reconfigure::Server<leopard_imaging_camera::copterTrackerV2Config>::CallbackType f;

    f = boost::bind(&callback_reconfig, _1, _2);
    server.setCallback(f);


    // Spin
    ros::spin ();
}
