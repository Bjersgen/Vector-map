#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <CSF.h>
#include <time.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>
#include "test_pcl_node.h"

using namespace std;
const char* config_file_path = "/home/bjersgen2004/pc_new/src/test_pcl/src/params.cfg";
// ros::Publisher pub_ply_map;

void CSF_addPointCloud(const vector<int>& index_vec, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
{
    auto& points = cloud_filtered->points;
    const auto& pointclouds = cloud->points;

    for_each(index_vec.begin(), index_vec.end(), [&](const auto& index) {
        pcl::PointXYZRGB pc;
        pc.x = pointclouds[index].x;
        pc.y = pointclouds[index].y;
        pc.z = pointclouds[index].z;
        // pc.intensity = pointclouds[index].intensity;
        pc.b = 127;

        points.push_back(pc);
    });

    cloud_filtered->height = 1;
    cloud_filtered->width = cloud_filtered->points.size();
}


void clothSimulationFilter(const vector< csf::Point >& pc,vector<int> &groundIndexes,vector<int> & offGroundIndexes)
{
    //step 1 read point cloud
    Cfg cfg;
    std::string slop_smooth;
	cfg.readConfigFile(config_file_path, "slop_smooth", slop_smooth);
	bool ss = false;
	if (slop_smooth == "true" || slop_smooth == "True")
	{
		ss = true;
	}
	else if (slop_smooth == "false" || slop_smooth == "False")
	{
		ss = false;
	}
	else{
		if (atoi(slop_smooth.c_str()) == 0){
			ss = false;
		}
		else
		{
			ss = true;
		}
	}
    std::string class_threshold;
	cfg.readConfigFile(config_file_path, "class_threshold", class_threshold);
	std::string cloth_resolution;
	cfg.readConfigFile(config_file_path, "cloth_resolution", cloth_resolution);
	std::string interations;
	cfg.readConfigFile(config_file_path, "interations", interations);
	std::string rigidness;
	cfg.readConfigFile(config_file_path, "rigidness", rigidness);
	std::string time_step;
	cfg.readConfigFile(config_file_path, "time_step", time_step);


    CSF csf;
    csf.setPointCloud(pc);// or csf.readPointsFromFile(pointClouds_filepath); 
    //pc can be vector< csf::Point > or PointCloud defined in point_cloud.h

    //step 2 parameter settings
    //Among these paramters:  
    //time_step  interations class_threshold can remain as defualt in most cases.
    csf.params.bSloopSmooth = ss;
    csf.params.cloth_resolution = atof(cloth_resolution.c_str());
    csf.params.rigidness = atoi(rigidness.c_str());

    csf.params.time_step = atof(time_step.c_str());
    csf.params.class_threshold = atof(class_threshold.c_str());
    csf.params.interations = atoi(interations.c_str());

    //step 3 do filtering
    //result stores the index of ground points or non-ground points in the original point cloud
    
    csf.do_filtering(groundIndexes, offGroundIndexes);
    //csf.do_filtering(groundIndexes, offGroundIndexes,true); 
    //if the third parameter is set as true, then a file named "cloth_nodes.txt" will be created, 
    //it respresents the cloth nodes.By default, it is set as false

}

string class_threshold;

int main (int argc, char **argv)
{

    clock_t start, end;
    start = clock();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_nonground (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ground (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_nearby (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr after_sege (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);

    // if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/bjersgen2004/Documents/map.pcd", *cloud) == -1)
	// {
	// 	PCL_ERROR("Couldn't read file1 \n");
	// 	return (-1);
	// }

    // std::cerr << "Cloud before filtering: " << std::endl;
    // std::cerr << *cloud << std::endl;

    // //using csf
    // vector<csf::Point> pc;
    // const auto& pointclouds = cloud->points;
    // pc.resresizeize(cloud->size());
    // transform(pointclouds.begin(), pointclouds.end(), pc.begin(), [&](const auto& p)->csf::Point {
    //     csf::Point pp;
    //     pp.x = p.x;
    //     pp.y = p.y;
    //     pp.z = p.z;
    //     return pp;
    //     });

    // std::vector<int> groundIndexes, offGroundIndexes;
    // clothSimulationFilter(pc, groundIndexes, offGroundIndexes);
    
    // CSF_addPointCloud(groundIndexes, cloud,cloud_filtered_ground);
    pcl::PCDWriter writer;
    // writer.write<pcl::PointXYZRGB>("groundPointCloud.pcd", *cloud_filtered_ground, false);

    // CSF_addPointCloud(offGroundIndexes, cloud,cloud_filtered_nonground);
    // writer.write<pcl::PointXYZRGB>("nonGroundPointCloud.pcd", *cloud_filtered_nonground, false);

    // cout<<"cloud_filtered_ground points:"<<cloud_filtered_ground->points.size() 
    // << "cloud_filtered_nonground points:"<<cloud_filtered_nonground->points.size()<<endl;

    // //bian li 
    // auto& points = cloud_filtered_ground->points;
    // float max_x = 0 , min_x = 0, max_y = 0, min_y=0, max_z=0, min_z=0;
    // for(int i=0 ; i<cloud_filtered_ground->points.size();i++){
    //     if(points[i].x<min_x)min_x = points[i].x;
    //     if(points[i].x>max_x)max_x = points[i].x;
    //     if(points[i].y<min_y)min_y = points[i].y;
    //     if(points[i].y>max_y)max_y = points[i].y;
    //     if(points[i].z<min_z)min_z = points[i].z;
    //     if(points[i].z>max_z)max_z = points[i].z;

    // }

    // cout<<"max_x:"<<max_x<<endl;
    // cout<<"min_x:"<<min_x<<endl;
    // cout<<"max_y:"<<max_y<<endl;
    // cout<<"min_y:"<<min_y<<endl;
    // cout<<"max_z:"<<max_z<<endl;
    // cout<<"min_z:"<<min_z<<endl;

    // auto& nonground_points = cloud_filtered_nonground->points;
    // auto& ground_nearby_points = ground_nearby->points;

    // for(int i=0; i<cloud_filtered_nonground->points.size();i++){

    //     if(nonground_points[i].x>=min_x && nonground_points[i].x<=max_x &&
    //     nonground_points[i].y>=min_y && nonground_points[i].y<=max_y &&
    //     nonground_points[i].z>=-5 && nonground_points[i].z<=5){
    //         pcl::PointXYZRGB pc;
    //         pc.x = nonground_points[i].x;
    //         pc.y = nonground_points[i].y;
    //         pc.z = nonground_points[i].z;
    //         ground_nearby_points.push_back(pc);
    //     }

    // }
    // ground_nearby->height = 1;
    // ground_nearby->width = ground_nearby->points.size();
    // cout<<"groundnearby points:" <<ground_nearby->points.size()<<endl;
    // writer.write<pcl::PointXYZRGB>("GroundNearbyPointCloud.pcd", *ground_nearby, false);


    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/bjersgen2004/pc_new/GroundNearbyPointCloud.pcd", *ground_nearby) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}
    auto& ground_nearby_points = ground_nearby->points;
    auto& after_sege_points = after_sege->points;
    //EuclideanClusterExtraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.1); // 25cm
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (2500000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (ground_nearby);
    ec.extract (cluster_indices);
    cout<< "cluster_indices:" << cluster_indices.size() << endl;
    int j = 0;
    for (const auto& cluster : cluster_indices){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*ground_nearby)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        for(int i = 0; i<cloud_cluster->points.size();i++){

            pcl::PointXYZRGB pc2;
            pc2.x = cloud_cluster->points[i].x;
            pc2.y = cloud_cluster->points[i].y;
            pc2.z = cloud_cluster->points[i].z;
            pc2.rgb  = j;
            // cloud_cluster->points[i].rgb = j;
            after_sege_points.push_back(pc2);

        }
    
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << j;
        writer.write<pcl::PointXYZRGB> ("./cloud_cluster/cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); //*
        j++;
    }
    after_sege->height=1;
    after_sege->width=after_sege->points.size();
    writer.write<pcl::PointXYZRGB>("AfterSege.pcd", *after_sege, false);


    end = clock();
	double dur = (double)(end - start);
	printf("Use Time:%f\n", (dur / CLOCKS_PER_SEC));


    //convert to ros and pub to rviz

    ros::init (argc, argv, "pcl_example");
    ros::NodeHandle nh;
    ros::Publisher pub_ply_map=nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    sensor_msgs::PointCloud2 output;
    //Convert the cloud to ROS message
    pcl::toROSMsg(*cloud_filtered_ground, output);
    output.header.frame_id = "odom";

    //循环每秒发送一次
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pub_ply_map.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

