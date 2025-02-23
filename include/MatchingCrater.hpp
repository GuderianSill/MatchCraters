#ifndef MATCHINGCRATER_H
#define MATCHINGCRATER_H
#include "crater.hpp"
#include "kdtree.hpp"
#include "getdata.hpp"
#include "GDALTransformer.hpp"

#include <unordered_map>
#include <cmath>
#include <queue>
#include <set>
#include <algorithm>
#include <regex>
#include <fstream>
#include <atomic>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <gdal_priv.h>
#include <cpl_conv.h>

class MatchingCrater
{
private:         
    //宏参数 
    double DOMAIN_RANGE;
    double DOMAIN_FACTOR_RATIO;
    double DISTANCE_MAX_GAP;
    double ANGLE_TOLERANCE;
    double DISTANCE_RATIO;
    double AREA_TOLERANCE_RATIO;
    double ASPECTRADIO_RADIO;
    double SEARCH_RANGE;

    //文件路径
    ////std::string imagesPath;
    std::string tifPath;
    std::string savePath;
    std::string tmp_savePath;
    std::string csvPath;

    //配置文件
    std::string configFile;
    bool matchedByRatio;

    // *图像信息
    GDALDataset* dataset1;
    GDALDataset* dataset2;

    int Src1Width;
    int Src1Height;
    int Src2Width;
    int Src2Height;

    GDALCoordinateTransformer* transformer1;
    GDALCoordinateTransformer* transformer2;

    //所有能被匹配的埙石坑数目 
    int totalMatchingPoints;

    struct keys{int id; double x; double y; double diameter;};

    //用于中心位置存储领域坑
    struct NeighborInformation
    {
        double distance;
        double angle;
        std::shared_ptr<Crater> crater;
        NeighborInformation(double distance, double angle, std::shared_ptr<Crater> crater): distance(distance), angle(angle), crater(crater){}

        bool operator <(const NeighborInformation& other) const;
    };    

    //用于存储每个图片中的埙石坑信息
    struct CraterImage    
    {
        std::string imageName;
        int imageId;
        int sumIds;
        std::vector<std::shared_ptr<Crater>> craters;      
        std::unique_ptr<KDTree> kdTree;
        std::unordered_map<std::shared_ptr<Crater>, std::vector<std::shared_ptr<NeighborInformation>>> neighborCraters;
        std::unordered_map<int, std::vector<std::shared_ptr<NeighborInformation>>> IdGetNeighborCraters;
    };
    //用于存储每个图片中的埙石坑信息
    std::vector<std::unique_ptr<CraterImage>> CraterImages;
    int ImagesNum;

    std::string get_imageName(const std::string& imagePath);

    void getTotalMatchingPoints();
    void build_dataStructure();
    void get_NeighborInformation();
    cv::Point2f convertCoordinates(const cv::Point2f& originalCoord, const cv::Size& originalSize, const cv::Size& resizedSize);
    cv::Mat GDALBlockToMat(GDALRasterBand *band, int xoff, int yoff, int width, int height);

    std::pair<bool, double> judge_matchingPoint(const std::vector<std::shared_ptr<NeighborInformation>>& a, const std::vector<std::shared_ptr<NeighborInformation>>& b) const;
    std::vector<std::pair<std::shared_ptr<Crater>, double>> matching_pointProgram(const std::shared_ptr<Crater>& crater, const int imageId);
    void matching_imageProgram();
    void show_keys(const std::unique_ptr<CraterImage>& image1, const std::unique_ptr<CraterImage>& image2);
    void writeKeys(const std::vector<keys>& key1, const std::vector<keys>& key2, int imageId1, int imageId2);
    void show_matched_image(const std::vector<keys>& key1, const std::vector<keys>& key2, int imageId1, int imageId2);

    //
    static bool CompareNeighborInformation(const std::shared_ptr<NeighborInformation>& a, const std::shared_ptr<NeighborInformation>& b);
    static bool CompareCratersByArea(const std::shared_ptr<Crater>& a, const std::shared_ptr<Crater>& b);   

    struct CompareSet
    {
        bool operator ()(const std::shared_ptr<Crater>& a, const std::shared_ptr<Crater>& b) const;
    };

public:
    MatchingCrater(const std::string name1, const std::string name2);
    ~MatchingCrater();
    void runMatching();
    void get_keys();

    //测试用    
    void test_get_NeighborInformation();
    void test_matching_pointProgram();
    void test_showAllMatchingPoints();    

    static int keyNums;
    friend void CSVGet_crater_object(const std::string name1, const std::string name2, MatchingCrater& matchingCrater);
};

#endif // MATCHINGCRATER_H
