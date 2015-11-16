#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <ostream>
#include <iomanip>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <vector>


namespace po=boost::program_options;
namespace fs = boost::filesystem;
using namespace std;

struct fileSortWithNumber
{
    bool operator ()(const fs::path& a, const fs::path& b)
    {
        stringstream aa(a.stem().string());
        stringstream bb(b.stem().string());
        int a_num, b_num;
        aa>>a_num;
        bb>>b_num;
        return a_num<b_num;
    }
};

int main(int argc, char** argv)
{
    int sequence;
    try {
        po::options_description desc("Allowed options");
        desc.add_options()
                ("help,h", "print usage message")
                ("sequence,s", po::value<int>()->required(), "sequence of dataset")
                ;
        po::variables_map vm;
        store(parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            cout << desc << endl;
            return 0;
        }
        vm.notify();
        sequence = vm["sequence"].as<int>();
    }
    catch (exception& e){
        cerr<<e.what()<<endl;
    }

    vector<fs::path> file_list;
    try {
        cout<<"Playing sequence "<<setw(2)<<setfill('0')<<sequence<<endl;
        stringstream database_dir;
        database_dir<<"/media/ub1404/data/Dataset/KITTI_Dataset/data_odometry_velodyne/dataset/sequences/";
        database_dir<<setw(2)<<setfill('0')<<sequence<<"/velodyne";
        fs::path dir(database_dir.str());

        typedef multimap<std::time_t, fs::path> result_set_t;

        result_set_t result_set;
        if(fs::exists(dir))
        {
            if(fs::is_directory(dir))
            {
                fs::directory_iterator end_iter;
                copy(fs::directory_iterator(dir), fs::directory_iterator(), back_inserter(file_list));
                sort(file_list.begin(), file_list.end(), fileSortWithNumber());

                //print sorted files list
//                copy(file_list.begin(), file_list.end(), ostream_iterator<fs::path>(cout,"\n"));

//            for(fs::directory_iterator iter(dir); iter != end_iter; ++iter)
//            {
//                if(fs::is_regular_file(iter->path())){
////                    cout<<iter->path().string()<<endl;
//
//                    cout<<iter->path().filename().stem()<<endl;
//                }
//            }
            }
            else
                cerr<<dir<<" is not a directory"<<endl;
        }
        else
            cerr<<dir<<" does not exist."<<endl;
    }
    catch(const fs::filesystem_error& e)
    {
        cout<<e.what()<<endl;
    }

    /**********************Init Ros Node**************************/

    ros::init(argc, argv, "kitti_velodyne_reader");
    ros::NodeHandle nh;

    ros::Publisher pub_velodyne = nh.advertise<sensor_msgs::PointCloud2>("velodyne64", 10);

    ros::Rate rate(10.0);

    vector<fs::path>::iterator pcd_file_name = file_list.begin();
    vector<fs::path>::iterator end_iter;
    float *data = new float[1000000*sizeof(float)];
    int cnt = 0;
    while(ros::ok() && pcd_file_name!=end_iter)
    {
//        sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->header.frame_id = "velodyne";
//        cloud->header.stamp = ros::Time::now().toNSec();
        cloud->header.seq =cnt++;
        FILE *stream;
        stream = fopen((*pcd_file_name).string().c_str(),"rb");
        int n_points = fread(data, sizeof(float), 1e6, stream) / 4;
        float *px,*py,*pz,*pi;
        px = data+0; py = data+1; pz = data+2; pi = data+3;
        cloud->width = n_points;
        cloud->height = 1;
        cloud->points.resize(cloud->width*cloud->height);
        for(int i=0;i<n_points;i++){
            cloud->points[i].x = *px;
            cloud->points[i].y = *py;
            cloud->points[i].z = *pz;
            cloud->points[i].intensity = *pi;
            px+=4; py+=4; pz+=4; pi+=4;
        }
        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud, *cloud_msg);
        pub_velodyne.publish(cloud_msg);
        ros::spinOnce();
        pcd_file_name++;
        rate.sleep();
        fclose(stream);
    }
    delete[] data;

    return 1;
}