#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include "pose3d.hpp"

using namespace boost::filesystem;

// read operations
void readData(const std::string fname, std::vector<std::string> &data);
void readData(const std::string fname, int rows, int cols, char split, Eigen::MatrixXd &res);
void readData(const std::string fname, char split, Eigen::MatrixXd &out);
void readData(const std::string fname, char split, std::vector<std::vector<double>> &data);
void readData(const std::string fname, char split, int lstart, std::map<uint32_t, Pose3D> &Poses);

// write operations
void writeData(const std::string fname, const cv::Mat& data);
void writeData(const std::string fname, const std::vector<std::string>& data);

// access filesystems
void getFilenamesByType(std::string path, std::string type, std::vector<std::string> &fnames);
void getFilenamesByTypeAndDirPrefix(const std::string path, const std::string type, std::vector<std::string> &fnames);
void getFilenamesByTypeAndDirPrefix(const std::string path, const std::string type, const std::string prefix, std::vector<std::string> &fnames);
void createCleanFolder(std::string path);
void createFolder(std::string path);

void readData(const std::string fname, std::vector<std::string> &data) {

    std::ifstream fid(fname.c_str());
    std::string line;
    while(std::getline(fid, line)) {
        data.push_back(line);
    }
}

void readData(const std::string fname, int rows, int cols, char split, Eigen::MatrixXd &res) {
    std::vector<std::vector<double>> data;
    readData(fname, split, data);

    if(data.empty() || rows > data.size() || cols > data[0].size()) {
        std::cerr << "Failed to read: " << fname << std::endl;
        EXIT_FAILURE;
    }

    res = Eigen::MatrixXd(rows, cols);
    for(int i=0; i<rows; i++) {
        for(int j=0; j<cols; j++) {
            res(i,j) = data[i][j];
        }
    }
}

void readData(const std::string fname, char split, Eigen::MatrixXd &out) {
    
    std::vector<std::vector<double>> data;

    readData(fname, split, data);

    Eigen::MatrixXd datamat(data.size(), data[0].size());

    for(size_t i=0; i<data.size(); i++) {
        for(size_t j=0; j<data[i].size(); j++) {
            datamat(i, j) = data[i][j];
        }
    }
    out = datamat;
}

void readData(const std::string fname, char split, std::vector<std::vector<double>> &data) {
    
    std::ifstream pose_fid(fname.c_str());
    std::string line;

    while(std::getline(pose_fid, line)) {
        std::vector<double> row;
        std::stringstream iss(line);
        std::string val;

        while (std::getline(iss, val, split)) {
            double v = std::atof(val.c_str());
            row.push_back(v);
        }
        data.push_back(row);
    }
}

void readData(const std::string fname, char split, int lstart, std::map<uint32_t, Pose3D> &Poses){

    Eigen::MatrixXd poseData;
    readData(fname, ',', poseData);
    for(size_t i=lstart; i<poseData.rows(); i++) {
        auto imageID = poseData(i, 0);
        Quaternion camQuaternion = Quaternion(poseData(i, 7), poseData(i, 4), 
                                              poseData(i, 5), poseData(i, 6));
        
        Vec3 camPosition(poseData(i, 1), poseData(i, 2), poseData(i, 3));
        Pose3D camPose(camQuaternion, camPosition);
        Poses.insert(std::make_pair(imageID, camPose));
    }

    std::cout << "Load poses " << Poses.size() << std::endl;
}

void writeData(const std::string fname, const cv::Mat& data) {
    std::ofstream outFile;
    outFile.open(fname);
    for(size_t i=0; i<data.rows; i++) {
        for(size_t j=0; j<data.cols; j++) {
            outFile << std::setprecision(10) << data.at<double>(i, j) << " ";
        }
        outFile << std::endl;
    }
    outFile.close();
}

void writeData(const std::string fname, const std::vector<std::string>& data) {
    std::ofstream outFile;
    outFile.open(fname);
    for(const auto& s : data) {
        outFile << s << std::endl;
    }
    outFile.close();
}

void getFilenamesByType(std::string path, std::string type, std::vector<std::string> &fnames) {
    fnames.clear();
    // path root(path);

    if (exists(path)) {
        directory_iterator end_iter;
        for (directory_iterator iter(path); iter != end_iter; ++iter) {
            if (is_regular_file(iter->status())) {
                if(iter->path().extension().string() == type) {
                    fnames.push_back(iter->path().string());
                }
            }
        }
        std::sort(fnames.begin(), fnames.end());
    } else {
        std::cerr << "Path " << path << " doesn't exist. " << std::endl;
    } 
}

void getFilenamesByTypeAndDirPrefix(const std::string path, const std::string type, std::vector<std::string> &fnames) {
    fnames.clear();

    if(exists(path)) {
        recursive_directory_iterator end_iter;
        for (recursive_directory_iterator iter(path); iter != end_iter; ++iter)
        {
            if(is_directory(iter->status()))
            {
                std::string dir;
                dir = iter->path().string();
                std::vector<std::string> subfnames;
                getFilenamesByType(dir, type, subfnames);
                fnames.insert(fnames.end(), subfnames.begin(), subfnames.end());
                
            }
        }

        // convert to relative paths
        for (auto& fn : fnames) {
            fn.erase(0, path.size());
        }
    }
}
void getFilenamesByTypeAndDirPrefix(const std::string path, const std::string type, const std::string prefix, std::vector<std::string> &fnames) {
    fnames.clear();

    if(exists(path)) {
        recursive_directory_iterator end_iter;
        for (recursive_directory_iterator iter(path); iter != end_iter; ++iter)
        {
            if(is_directory(iter->status()))
            {
                std::string dir;
                dir = iter->path().string();
                if(iter->path().leaf().string() == prefix) {
                    std::vector<std::string> subfnames;
                    getFilenamesByType(dir, type, subfnames);
                    fnames.insert(fnames.end(), subfnames.begin(), subfnames.end());
                }
            }
        }

        // convert to relative paths
        for (auto& fn : fnames) {
            fn.erase(0, path.size());
        }
    }
}

void createCleanFolder(std::string path) {
    if (exists(path)) {
        remove_all(path);
        create_directory(path);
    } else {
        create_directory(path);
    }
}

void createFolder(std::string path) {
    if (!exists(path)) {
        create_directory(path);
    }
}