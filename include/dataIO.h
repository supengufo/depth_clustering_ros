//
// Created by alex on 2020/4/19.
//

#ifndef PCL_TEST_DATAIO_H
#define PCL_TEST_DATAIO_H
//PCL
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
//std
#include <string>
using namespace std;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
struct Point {
    float x, y;
    int label_;
    Point(){}
    Point(float x_in, float y_in) {
        x = x_in;
        y = y_in;
        label_=0;
    }
};

class DataIO {
public:
    vector<string> file_names = {"dresser",
                                 "mantel",
                                 "sink",
                                 "piano",
                                 "tent",
                                 "range_hood",
                                 "bookshelf",
                                 "chair",
                                 "night_stand",
                                 "lamp",
                                 "bed",
                                 "toilet",
                                 "sofa",
                                 "guitar",
                                 "vase",
                                 "keyboard",
                                 "stool",
                                 "cup",
                                 "xbox",
                                 "stairs",
                                 "plant",
                                 "bench",
                                 "wardrobe",
                                 "door",
                                 "bowl",
                                 "monitor",
                                 "desk",
                                 "radio",
                                 "table",
                                 "bottle",
                                 "person",
                                 "airplane",
                                 "curtain",
                                 "car",
                                 "bathtub",
                                 "tv_stand",
                                 "cone",
                                 "flower_pot",
                                 "laptop"
    };

    void getPLYData(vector<pcl::PointCloud<PointT>> &all_pointsCloud) {
        PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
        for (auto &name:file_names) {
            string file_path = "/home/alex/code/code_tools/pcl_test/ply_data/" + name + "/train/" + name + "_0001.ply";
//            cout << file_path << endl;
            if (pcl::io::loadPLYFile(file_path, *cloud_in) < 0) {
                PCL_ERROR ("Error loading cloud %s.\n", &file_path);
                return;
            }
            all_pointsCloud.push_back(*cloud_in);
        }
        return;
    }
    //
    PointCloudT::Ptr readBinFile(string &input_dir){
        fstream input(input_dir, ios::in | ios::binary);
        if(!input.good()){
            cerr << "Could not read file: " << input_dir << endl;
            exit(EXIT_FAILURE);
        }
        input.seekg(0, ios::beg);
        PointCloudT::Ptr pointCloud (new PointCloudT);
        int i;
        for (i=0; input.good() && !input.eof(); i++) {
            PointT point;
            input.read((char *) &point.x, 3*sizeof(float));
            input.read((char *) &point.intensity, sizeof(float));
            point.intensity*=255;
            pointCloud->push_back(point);
        }
        input.close();

        cout << "Read bin file from [" << input_dir << "]: "<< i << " points"<< endl;
        return pointCloud;
//        if (pcl::io::savePCDFileBinary(outfile, *pointCloud) == -1)
//        {
//            PCL_ERROR("Couldn't write file\n");
//            return false;
//        }
    }

    //read data from txt
    vector<vector<Point>> readTXT(const string &root_path,const vector<string> &txt_file_names){
        //1.read all file names path
        vector<string> paths;
        for (const auto &path:txt_file_names) {
            paths.push_back(root_path + path);
        }
        // read data
        vector<vector<Point>> res;
        for(const auto &file:paths){
            ifstream inFile(file);
            string strShow;
            vector<Point> point_vec;
            if (inFile) {
                string strLine;
                while (getline(inFile, strLine)) // line中不包括每行的换行符
                {
                    auto front_length = strLine.find(',');
                    int back_length = strLine.length() - front_length;
                    auto x = strLine.substr(0, front_length);
                    auto y = strLine.substr(front_length + 1, back_length);
                    point_vec.emplace_back(atof(x.c_str()),atof(y.c_str()));
                }
            }
            res.push_back(point_vec);
        }
        return res;
    }
};

#endif //PCL_TEST_DATAIO_H
