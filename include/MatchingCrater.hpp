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

//#include <sys/types.h>
//#include <sys/stat.h>
#include <unistd.h>

#include <gdal_priv.h>
#include <cpl_conv.h>

class MatchingCrater
{
private:         
    //宏参数
    double NUM_RATIO;
    double DOMAIN_RANGE;
    double DOMAIN_FACTOR_RATIO;
    double DISTANCE_MAX_GAP;
    double ANGLE_TOLERANCE;
    double DISTANCE_RATIO;
    double AREA_TOLERANCE_RATIO;
    double ASPECTRADIO_RADIO;
    double SEARCH_RANGE;
    bool FILTER;
    bool SHOW_IMAGE;
    bool TEST;

    enum MatchingMethod 
    {
        GeographicCoordinateTransformation,  // 地理坐标转换法
        GuidedMatching                        // 引导匹配法
    }matchingMethod;

    //文件路径
    std::string pathFile;
    std::string tifPath;
    std::string savePath;
    std::string tmp_savePath;
    std::string csvPath;
    std::string folderName;

    //配置文件
    std::string configFile;
    bool matchedByRatio;

    // 图像信息
    GDALDataset* datasetSrc;
    GDALDataset* datasetDst;

    int SrcWidth;
    int SrcHeight;

    int DstWidth;
    int DstHeight;

    // 图像偏移信息
    std::vector<double> offset;
    std::vector<double> dispersion; // 偏移量的标准差

    GDALCoordinateTransformer* transformerSrc;
    GDALCoordinateTransformer* transformerDst;

    //所有能被匹配的埙石坑数目 
    int totalMatchingPoints;

    struct keys{int id; double x; double y; double diameter;};
    int FilterByHomographyMat(std::vector<keys>& vecSrcKeys, std::vector<keys>& vecDstKeys, std::vector<keys>& vecSrcNew, std::vector<keys>& vecDstNew, cv::Mat& h);
    std::vector<keys> vecPtSrc;
    std::vector<keys> vecPtDst;

    //用于中心位置存储领域坑
    struct NeighborInformation
    {
        double distance;
        double angle;
        std::shared_ptr<Crater> crater;
        NeighborInformation(double distance, double angle, std::shared_ptr<Crater> crater): distance(distance), angle(angle), crater(crater){}
    };

    //用于存储每个图片中的埙石坑信息
    struct CraterImage    
    {
        std::string imageName;
        int sumIds;
        std::vector<std::shared_ptr<Crater>> craters;
        std::vector<std::shared_ptr<Crater>> matchableCraters;
        std::unique_ptr<KDTree> kdTree;
        std::unordered_map<std::shared_ptr<Crater>, std::vector<std::shared_ptr<NeighborInformation>>> neighborCraters;
        std::unordered_map<int, std::vector<std::shared_ptr<NeighborInformation>>> IdGetNeighborCraters;
    };
    //用于存储每个图片中的埙石坑信息
    std::vector<std::unique_ptr<CraterImage>> CraterImages;

    std::unique_ptr<CraterImage> SrcCraterImage;
    std::unique_ptr<CraterImage> DstCraterImage;

    std::string get_imageName(const std::string& imagePath);

    // 初始化函数
    void Init(const std::string& name1, const std::string& name2);
  
    void getTotalMatchingPoints();

    void build_dataStructure();
    void get_NeighborInformation();
    int get_offset();

    cv::Point2f convertCoordinates(const cv::Point2f& originalCoord, const cv::Size& originalSize, const cv::Size& resizedSize);
    cv::Mat GDALBlockToMat(GDALRasterBand *band, int xoff, int yoff, int width, int height);

    std::pair<bool, double> judge_matchingPoint(const std::vector<std::shared_ptr<NeighborInformation>>& a, const std::vector<std::shared_ptr<NeighborInformation>>& b) const;
    std::vector<std::pair<std::shared_ptr<Crater>, double>> matching_pointProgram(const std::shared_ptr<Crater>& crater, const std::unique_ptr<CraterImage>& DstImage);
    void show_keys(const std::unique_ptr<CraterImage>& image1, const std::unique_ptr<CraterImage>& image2);
    void writeKeys(const std::vector<keys>& key1, const std::vector<keys>& key2);
    int writeLog(const std::vector<keys>& k1, const std::vector<keys>& k2, const double probability);
    void show_matched_image(const std::vector<keys>& key1, const std::vector<keys>& key2);

    // struct CompareSet
    // {
    //     bool operator ()(const std::shared_ptr<Crater>& a, const std::shared_ptr<Crater>& b) const;
    // };

public:
    MatchingCrater(const std::string& name1, const std::string& name2, bool filter, bool show_image, bool test);
    MatchingCrater(const std::string& name1, const std::string& name2, const std::string& offset, const std::string& variance,
        bool filter, bool show_image, bool test);
    ~MatchingCrater();    
    void runMatching();
    void get_keys();

    static int keyNums;
    friend void CSVGet_crater_object(const std::string name1, const std::string name2, MatchingCrater& matchingCrater);
};

#endif // MATCHINGCRATER_H
