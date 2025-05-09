#include "MatchingCrater.hpp"
#include "json_serializer.hpp"
//#include "RemoveOutliers.hpp"
#include <random>
using std::cin;
using std::cout;
using std::endl;

int MatchingCrater::keyNums = 1;
bool normalExit = false;

/**
 * @brief Constructor section for the MatchingCrater class.
 * 
 * This section defines two constructors for the MatchingCrater class. 
 * The first constructor initializes the object using the Geographic Coordinate Transformation matching method. 
 * The second constructor initializes the object using the Guided Matching method and also parses the offset and variance parameters.
 * 
 * @param name1 The name of the first image.
 * @param name2 The name of the second image.
 * @param offset A string representing the offset, formatted as "x,y".
 * @param variance A string representing the variance, formatted as "x,y".
 */
MatchingCrater::MatchingCrater(const std::string& name1, const std::string& name2, bool filter, bool show_image, bool test):
    FILTER(filter), SHOW_IMAGE(show_image), TEST(test)
{
    matchingMethod = GeographicCoordinateTransformation;
    Init(name1, name2);            
}
MatchingCrater::MatchingCrater(const std::string& name1, const std::string& name2, const std::string& offset, const std::string& variance, 
    bool filter, bool show_image, bool test): 
    FILTER(filter), SHOW_IMAGE(show_image), TEST(test)
{
    matchingMethod = GuidedMatching;
    std::istringstream iss1(offset);
    std::istringstream iss2(variance);
    std::string token;
    while (std::getline(iss1, token, ','))
        this->offset.push_back(std::stod(token));
    if (this->offset.size() != 2)
        throw std::runtime_error("Offset error");

    while (std::getline(iss2, token, ','))
        dispersion.push_back(std::sqrt(std::stod(token)));
    if (dispersion.size() != 2)
        throw std::runtime_error("Dispersion error");
    
    cout << dispersion[0] << " " << dispersion[1] << std::endl;
    Init(name1, name2);
}

/**
 * @brief Function to initialize the MatchingCrater object.
 * 
 * This function is used to initialize the `MatchingCrater` object and performs the following operations:
 * 1. Register all GDAL drivers.
 * 2. Read the configuration file and set the parameters required for matching.
 * 3. Open two TIFF image files and obtain the height and width of the images.
 * 4. Create coordinate transformers to handle coordinate conversion for the images.
 * 5. Read crater information from CSV files.
 * 6. Set the image ID for each crater image.
 * 7. Calculate the total number of possible matching points.
 * 
 * @param SrcName The name of the first image, without the file extension.
 * @param DstName The name of the second image, without the file extension.
 */
void MatchingCrater::Init(const std::string& SrcName, const std::string& DstName)
{
    GDALAllRegister();

    // Set the paths for the images, CSV files, and configuration file.
    this->pathFile = "cfg/path.cfg";
    std::ifstream path(pathFile);
    std::string line;
    if (!path.is_open())
        throw std::runtime_error("path file not found");

    std::map<std::string, std::string> pathMap;

    while (std::getline(path, line))
    {
        std::istringstream iss(line);
        std::string label;
        std::string value;
        if (iss >> label >> value)
            pathMap[label] = value;
    }

    this->tifPath = pathMap["tifPath"];
    this->savePath = pathMap["savePath"];
    this->csvPath = pathMap["csvPath"];
    this->matchedByRatio = true;
    this->configFile = pathMap["configFile"];

    path.close();

    std::ifstream config(configFile);
    if (!config.is_open())    
        throw std::runtime_error("config file not found");

    // Create a map to store key-value pairs from the configuration file
    std::map<std::string, double> configMap;
    
    while (std::getline(config, line))
    {
        std::istringstream iss(line);
        std::string label;
        double value;
        if (iss >> label >> value)
            configMap[label] = value;
    }

    this->NUM_RATIO = configMap["NUM_RATIO"];
    this->DOMAIN_RANGE = configMap["DOMAIN_RANGE"];
    this->DOMAIN_FACTOR_RATIO = configMap["DOMAIN_FACTOR_RATIO"];
    this->DISTANCE_MAX_GAP = configMap["DISTANCE_MAX_GAP"];
    this->ANGLE_TOLERANCE = configMap["ANGLE_TOLERANCE"];
    this->DISTANCE_RATIO = configMap["DISTANCE_RATIO"];
    this->AREA_TOLERANCE_RATIO = configMap["AREA_TOLERANCE_RATIO"];
    this->ASPECTRADIO_RADIO = configMap["ASPECTRADIO_RADIO"];
    this->SEARCH_RANGE = configMap["SEARCH_RANGE"];

    config.close();
    
    // Init image information
    std::string tifName1 = tifPath + SrcName + ".tif";
    std::string tifName2 = tifPath + DstName + ".tif";

    datasetSrc = (GDALDataset*)GDALOpen(tifName1.c_str(), GA_ReadOnly); 
    datasetDst = (GDALDataset*)GDALOpen(tifName2.c_str(), GA_ReadOnly);

    if (datasetSrc == nullptr || datasetDst == nullptr)    
        throw std::runtime_error("can not open tiff file");

    this->transformerSrc = new GDALCoordinateTransformer(datasetSrc);
    this->transformerDst = new GDALCoordinateTransformer(datasetDst);

    SrcHeight = datasetSrc->GetRasterYSize();
    SrcWidth = datasetSrc->GetRasterXSize();
    DstHeight = datasetDst->GetRasterYSize();
    DstWidth = datasetDst->GetRasterXSize();

    std::cout << "Image information read completed" << "\n";    

    // Read crater information from CSV files.
    CSVGet_crater_object(csvPath + SrcName + ".csv", csvPath + DstName + ".csv", *this);
    
    folderName = savePath + SrcName + "_" + DstName + "/";
    int status = mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

    if (status == 0) cout << "储存文件夹创建成功" << endl;
    else        cout << "储存文件夹已存在" << endl;

    std::cout << "Crater information stored successfully" << std::endl;    

    getTotalMatchingPoints();
}

MatchingCrater::~MatchingCrater()
{
    delete transformerSrc;
    delete transformerDst;
    normalExit = true;
}

void MatchingCrater::getTotalMatchingPoints()
{
    double maxRange = std::max(DOMAIN_RANGE, SEARCH_RANGE);
    for (auto& crater: SrcCraterImage->craters)
    {
        double SrcGeoX, SrcGeoY;
        transformerSrc->pixelToGeo(crater->get_coordinates()[0], crater->get_coordinates()[1], SrcGeoX, SrcGeoY);
        double DstPixelX, DstPixelY;
        // 计算像素坐标
        transformerDst->geoToPixel(SrcGeoX, SrcGeoY, DstPixelX, DstPixelY);

        if (DstPixelX >= 0 - maxRange && DstPixelX < DstWidth + maxRange &&
            DstPixelY >= 0 - maxRange && DstPixelY < DstHeight + maxRange)
        {
            SrcCraterImage->matchableCraters.push_back(std::move(crater));
        }
    }
    for (auto& crater: DstCraterImage->craters)
    {
        double DstGeoX, DstGeoY;
        transformerDst->pixelToGeo(crater->get_coordinates()[0], crater->get_coordinates()[1], DstGeoX, DstGeoY);
        double SrcPixelX, SrcPixelY;
        // 计算像素坐标
        transformerSrc->geoToPixel(DstGeoX, DstGeoY, SrcPixelX, SrcPixelY);
        if (SrcPixelX >= 0 - maxRange && SrcPixelX < SrcWidth + maxRange &&
            SrcPixelY >= 0 - maxRange && SrcPixelY < SrcHeight + maxRange)
        {
            DstCraterImage->matchableCraters.push_back(std::move(crater));
        }        
    }
    SrcCraterImage->craters.clear();
    DstCraterImage->craters.clear();
    totalMatchingPoints = std::min(SrcCraterImage->matchableCraters.size(), DstCraterImage->matchableCraters.size());
    cout << "Src:" << SrcCraterImage->matchableCraters.size() << ", ";
    cout << "Dst:" << DstCraterImage->matchableCraters.size() << endl;
    if (SrcCraterImage->matchableCraters.size() == 0 || DstCraterImage->matchableCraters.size() == 0)
    {
        writeLog(std::vector<keys>(), std::vector<keys>(), 0);
        std::cerr << "没有匹配点" << std::endl;
        exit(-1);
    }
}

void MatchingCrater::build_dataStructure()
{
    std::vector<std::thread> threads;

    threads.emplace_back([&](){SrcCraterImage->kdTree = std::unique_ptr<KDTree>(new KDTree(SrcCraterImage->matchableCraters));});
    threads.emplace_back([&](){DstCraterImage->kdTree = std::unique_ptr<KDTree>(new KDTree(DstCraterImage->matchableCraters));});    
    
    for (auto& thread : threads) 
        thread.join();

    cout << "DataStructure build success" << endl;
}

void MatchingCrater::get_NeighborInformation()
{
    std::mutex mtx;
    std::vector<std::thread> threads;

    // 定义一个线程函数，用于处理单个 craterImage
    auto processCraterImage = [&mtx, this](std::unique_ptr<MatchingCrater::CraterImage> &craterImage) 
    {
        craterImage->neighborCraters.reserve(craterImage->matchableCraters.size());

        const int num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> sub_threads;
        const size_t chunk_size = (craterImage->matchableCraters.size() + num_threads - 1) / num_threads;
        //craters
        auto process_chunk = [&mtx, this, &craterImage](size_t start, size_t end) 
        {
            for (size_t i = start; i < end; ++i)
            {
                auto& KDCrater = craterImage->matchableCraters[i];
                std::vector<std::shared_ptr<Crater>> neighbors = craterImage->kdTree->findNeighbors(*KDCrater, this->DOMAIN_RANGE);        

                std::vector<std::shared_ptr<NeighborInformation>> neighborInformations;
                neighborInformations.reserve(neighbors.size());

                //获取邻域角度，距离信息
                for (auto& neighbor: neighbors)
                {                
                    const double distance = Crater::euclideanDistance(*KDCrater, *neighbor);
                    const double angle = Crater::getAngle(*KDCrater, *neighbor);

                    neighborInformations.emplace_back(std::make_shared<NeighborInformation>(distance, angle, neighbor));
                }
                std::sort(neighborInformations.begin(), neighborInformations.end(),
                    [](const std::shared_ptr<MatchingCrater::NeighborInformation> &a, const std::shared_ptr<MatchingCrater::NeighborInformation> &b) 
                    {
                        return a->distance < b->distance; // 按照 distance 从小到大排序
                    });
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    craterImage->neighborCraters.try_emplace(KDCrater, std::move(neighborInformations));
                }
            }
        };
        for (int i = 0; i < num_threads; ++i) 
        {
            size_t start = i * chunk_size;
            size_t end = std::min(start + chunk_size, craterImage->matchableCraters.size());
            if (start < end) 
            {
                sub_threads.emplace_back(process_chunk, start, end);
            }
        }

        for (auto& thread: sub_threads)
            thread.join();
        
    };

    threads.emplace_back(processCraterImage, std::ref(SrcCraterImage));
    threads.emplace_back(processCraterImage, std::ref(DstCraterImage));

    // 等待所有线程完成
    for (auto& thread : threads)
        thread.join();

    cout << "NeighborInformation get success" << endl;
}

void MatchingCrater::runMatching()
{    
    build_dataStructure();
    get_NeighborInformation();

    int ifOffset = get_offset();

    if (ifOffset == 1)
    {
        matchingMethod = GuidedMatching;
    }
    else if (ifOffset == -1)
    {
        cout << "引导匹配失败" << endl;
    }
}

void MatchingCrater::get_keys()
{
    show_keys(SrcCraterImage, DstCraterImage);
}

/**
 * @brief 展示两个图片的匹配点
 *
 * @param SrcImage 第一个图片的陨石坑对象
 * @param DstImage 第二个图片的陨石坑对象
 *
 * @return std::vector<std::pair<std::shared_ptr<Crater>, double>> 匹配成功的陨石坑对象和概率
 */
void MatchingCrater::show_keys(const std::unique_ptr<CraterImage>& SrcImage, const std::unique_ptr<CraterImage>& DstImage)
{
    std::atomic<int> matchedPoints{0};
    std::atomic<int> currentId{0};
    std::vector<keys> k1, k2;
    std::mutex k_mutex; // 保护k1/k2和输出
    std::mutex file_mutex;

    // 创建进度条显示线程
    auto progress_monitor = [&, this]()
    {
        //std::cout << std::unitbuf;  // 禁用输出缓冲
        const int bar_width = 40;
    
        //while (currentId < SrcImage->sumIds)
        while (currentId < SrcImage->matchableCraters.size())
        {
            // 计算各阶段进度
            float match_progress = currentId / (float)SrcImage->matchableCraters.size();
            
            // 组合总进度（根据阶段权重）
            float total_progress = match_progress;
            
            // 构建进度条
            cout << "\r[";
            int pos = bar_width * total_progress;
            for (int i = 0; i < bar_width; ++i) {
                if (i < pos) cout << "=";
                else if (i == pos) cout << ">";
                else cout << " ";
            }
            cout << "] "
                << double(total_progress * 100.0) << "% "
                << "(Match: " << currentId << "/" << SrcImage->matchableCraters.size() << ") "
                << std::flush;
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        cout << "\r[";
        for (int i = 0; i < bar_width; ++i)
            cout << "=";
        cout << "] " << 100 << "% " << "(Match: " << SrcImage->matchableCraters.size() << "/" << SrcImage->matchableCraters.size() << ")    " << std::flush;
    };
    std::thread progress_thread(progress_monitor);

    const int NUM_THREADS = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    ICraterSerializer::json output = ICraterSerializer::json::array();

    std::sort(DstCraterImage->matchableCraters.begin(), DstCraterImage->matchableCraters.end(),
    [](const std::shared_ptr<Crater>& a, const std::shared_ptr<Crater>& b)
        {return a->get_area() < b->get_area();});

    // 匹配工作
    for (int i = 0; i < NUM_THREADS; i++)
    {
        threads.emplace_back([&]()
        {
            while (true)
            {
                //动态获取任务
                int CraterId = currentId.fetch_add(1);
                if (CraterId >= SrcImage->matchableCraters.size()) break;

                //auto& originCrater = SrcImage->neighborCraters[SrcImage->matchableCraters[CraterId]];
                auto& originCrater = SrcImage->matchableCraters[CraterId];


                double originx = originCrater->get_coordinates()[0];
                double originy = originCrater->get_coordinates()[1];
                double originDiameter = originCrater->get_diameter();

                //将DstImage中与originCrater中可能匹配的点存放到容器中
                std::vector<std::pair<std::shared_ptr<Crater>, double>> matchedCraters = matching_pointProgram(originCrater, DstImage);

                if (!matchedCraters.empty())
                {
                    
                    // TODO 输出所有可能匹配的信息
                    if (TEST)
                    {
                        auto serializer = CraterJsonSerializer::create(
                        {
                            .include_neighbors = false,
                            .float_precision = 4
                        });
                        
                        ICraterSerializer::json json = serializer->serialize(originCrater, matchedCraters);
                        {                            
                            std::lock_guard<std::mutex> lock(file_mutex);
                            output.push_back(json);
                        }
                    }

                    int currentMatch = matchedPoints.fetch_add(1) + 1;
                    double maxProbability = 0;

                    //找出最佳匹配
                    std::shared_ptr<Crater> resultCrater = nullptr;
                    for (auto& matchedCrater: matchedCraters)
                    {
                        if (matchedCrater.second > maxProbability)
                        {
                            maxProbability = matchedCrater.second;
                            resultCrater = matchedCrater.first;
                        }
                    }
                    if (resultCrater)
                    {
                        std::lock_guard<std::mutex> lock(k_mutex);
                        k1.push_back({currentMatch, originx, originy, originDiameter});
                        k2.push_back({currentMatch, resultCrater->get_coordinates()[0],
                            resultCrater->get_coordinates()[1], resultCrater->get_diameter()});
                    }
                }
            }
        });
    }
    
    progress_thread.join();
    for (auto& t: threads)
        t.join();

    if (TEST)
    {
        std::ofstream file;
        std::string filename = folderName + "craters.json";
        file.open(filename, std::ios::out);
        if (!file.is_open())
        {
            std::cerr << "无法打开文件: " << filename << std::endl;
        }
        else
        {
            file << output.dump(4);
            std::cout << "数据已写入文件: " << filename << std::endl;
        }
        file.close();
    }
    
    double probability = static_cast<double>(matchedPoints.load()) / totalMatchingPoints;
    cout << "\n匹配概率：" << probability << endl;

    // std::vector<keys>
    vecPtSrc = k1;
    vecPtDst = k2;
    if (this->FILTER)
    {
        auto tmpSrc = vecPtSrc;
        auto tmpDst = vecPtDst;
        cv::Mat h;
        cout << FilterByHomographyMat(k1, k2, tmpSrc, tmpDst, h) << " ";
        cout << FilterByHomographyMat(tmpDst, tmpSrc, vecPtDst, vecPtSrc, h) << endl;
    }

    int logCode = writeLog(k1, k2, probability);

    switch (logCode)
    {
    case -1:
        cout << "写入日志失败" << endl;
        return;
        break;
    case -2:
        cout << "key1 size(" << k1.size() << ") is too small" << std::endl;
        return;
        break;
    case -3:
        cout << "ransac size(" << vecPtSrc.size() << ") is too small" << std::endl;
        return;
    default:
        break;
    }

    writeKeys(vecPtSrc, vecPtDst);
    cout << "writeKeys success" << endl;
    if (this->SHOW_IMAGE)
    show_matched_image(vecPtSrc, vecPtDst);
}

/**
 * @brief 点匹配程序
 *
 * @param originCrater 传入点
 * @param DstImage 目标图像
 *
 * @return std::vector<std::pair<std::shared_ptr<Crater>, double>> 匹配成功的点的集合
 */
std::vector<std::pair<std::shared_ptr<Crater>, double>> MatchingCrater::matching_pointProgram(const std::shared_ptr<Crater>& originCrater, const std::unique_ptr<CraterImage>& DstImage)
{
    double geoX1, geoY1;
    transformerSrc->pixelToGeo(originCrater->get_coordinates()[0], originCrater->get_coordinates()[1], geoX1, geoY1);

    auto& targetCraters = DstImage->matchableCraters;


    const double originArea = originCrater->get_area();
    const auto& originImage = SrcCraterImage;

    double lowArea = originArea / AREA_TOLERANCE_RATIO;
    double highArea = originArea * AREA_TOLERANCE_RATIO;

    auto lower = std::lower_bound(targetCraters.begin(), targetCraters.end(), std::make_shared<Crater>(lowArea), [](const std::shared_ptr<Crater> &a, const std::shared_ptr<Crater> &b) {return a->get_area() < b->get_area();});
    auto higher = std::upper_bound(targetCraters.begin(), targetCraters.end(), std::make_shared<Crater>(highArea), [](const std::shared_ptr<Crater> &a, const std::shared_ptr<Crater> &b) {return a->get_area() < b->get_area();});

    std::vector<std::shared_ptr<Crater>> similarCraters;
    for (auto it = lower; it != higher; it++)
    {
        if (std::max(it->get()->get_aspectRatio(), originCrater->get_aspectRatio())/std::min(it->get()->get_aspectRatio(), originCrater->get_aspectRatio())
                > ASPECTRADIO_RADIO)
            continue;
        else
        {
            switch (matchingMethod)
            {
            case MatchingMethod::GeographicCoordinateTransformation:
                {
                    // 转换成地理坐标
                    double geoX2, geoY2;                    
                    transformerDst->pixelToGeo(it->get()->get_coordinates()[0], it->get()->get_coordinates()[1], geoX2, geoY2);
                    if (std::sqrt(std::pow(geoX1 - geoX2, 2) + std::pow(geoY1 - geoY2, 2)) <= SEARCH_RANGE)
                        similarCraters.push_back(*it);   
                }
                break;
            case MatchingMethod::GuidedMatching:
                {
                    // 偏移像素坐标
                    double pixelx = originCrater->get_coordinates()[0] + offset[0];
                    double pixely = originCrater->get_coordinates()[1] + offset[1];

                    // 目标坐标
                    double targetX = it->get()->get_coordinates()[0];
                    double targetY = it->get()->get_coordinates()[1];

                    // 计算目标与预测的差值
                    double dx = std::fabs(targetX - pixelx);
                    double dy = std::fabs(targetY - pixely);

                    if (dx <= 3 * dispersion[0] && dy <= 3 * dispersion[1])
                        similarCraters.push_back(*it);
                }
                break;
            default:
                break;
            }    
        }        
    }

    std::vector<std::pair<std::shared_ptr<Crater>, double>> results;
    for (auto it = similarCraters.begin(); it != similarCraters.end(); it++)
    {
        std::pair<bool, double> judgeProbability = judge_matchingPoint(originImage->neighborCraters[originCrater], DstImage->neighborCraters[*it]);
        if (judgeProbability.first)
        {
            //cout << "概率:" << judgeProbability.second << endl;
            double 面积长宽比概率 = std::min(it->get()->get_aspectRatio(), originCrater->get_aspectRatio())/std::max(it->get()->get_aspectRatio(), originCrater->get_aspectRatio())
                    *std::min(it->get()->get_area(), originCrater->get_area())/std::max(it->get()->get_area(), originCrater->get_area());
            results.push_back({*it, judgeProbability.second * 面积长宽比概率});
        }
    }
    return results;
}

/**
 * @brief 判断两个陨石坑是否匹配
 *
 * @param a 第一个陨石坑
 * @param b 第二个陨石坑
 *
 * @return std::pair<bool, double> 匹配成功返回true和概率，匹配失败返回false和0
 */
std::pair<bool, double> MatchingCrater::judge_matchingPoint(const std::vector<std::shared_ptr<NeighborInformation>>& a, const std::vector<std::shared_ptr<NeighborInformation>> &b) const
{
    //面积，长宽比因素在matching_pointProgram已经考虑
    std::vector<std::shared_ptr<NeighborInformation>> pa(a.begin() + 1, a.end());
    std::vector<std::shared_ptr<NeighborInformation>> pb(b.begin() + 1, b.end());

    double sumNeighborPoints = std::min(pa.size(), pb.size());
    double matchedNeighborPoints = 0;

    //坑数比例判断
    if (!pa.size() || !pb.size()) return {false, 0};
    double CraterRatio = static_cast<double>(std::min(pa.size(), pb.size())) / std::max(pa.size(), pb.size());
    if (CraterRatio < this->NUM_RATIO)
        return {false, 0};

    // 领域坑距离和角度判断
    for (const auto& nowa: pa)
    {
        for (auto itb = pb.begin(); itb != pb.end(); itb++)
        {
            //距离判断
            if (!matchedByRatio && std::fabs(nowa->distance - (*itb)->distance) > DISTANCE_MAX_GAP)
            {
                if (nowa->distance > (*itb)->distance)                
                    continue;                
                else break;
            }
            else if (matchedByRatio && (std::max(nowa->distance, (*itb)->distance) / std::min(nowa->distance, (*itb)->distance)) > DISTANCE_RATIO)
            {
                //todo, 2025-02-16
                continue;
            }            
            else
            {
                //角度判断
                if (std::fabs(nowa->angle - (*itb)->angle > ANGLE_TOLERANCE))
                    continue;
                else
                {
                    //判断邻域陨石坑的基本属性
                    double areaRatio = nowa->crater->get_area() / (*itb)->crater->get_area();
                    double aspectRadio_radio = nowa->crater->get_aspectRatio() / (*itb)->crater->get_aspectRatio();
                    if (areaRatio < 1) areaRatio = 1 / areaRatio;
                    if (aspectRadio_radio < 1) aspectRadio_radio = 1 / aspectRadio_radio;
                    //cout << "areaRatio:" << areaRatio << " aspectRadio:" << aspectRadio_radio << endl;
                    if (areaRatio > AREA_TOLERANCE_RATIO || aspectRadio_radio > ASPECTRADIO_RADIO)
                        continue;
                    else
                    {
                        matchedNeighborPoints++;
                        //@todo, delete it or not.
                        pb.erase(itb);
                        break;
                    }
                }
            }
        }
    }
    auto probability = matchedNeighborPoints / sumNeighborPoints;
    auto totalProbability = probability * CraterRatio;
    if (totalProbability < DOMAIN_FACTOR_RATIO)
        return {false, totalProbability};
    return {true, totalProbability};
}


std::string MatchingCrater::get_imageName(const std::string& imagePath)
{
    std::string imageName;
    auto itend = imagePath.rbegin();
    for (auto it = imagePath.rbegin(); it != imagePath.rend(); it++)
    {
        if (*it == '.')
            itend = it + 1;
        if (*it == '/')
        {
            imageName = std::string(itend, it);
            std::reverse(imageName.begin(), imageName.end());
            break;
        }
        else if (it + 1 == imagePath.rend())
        {
            imageName = std::string(itend, it + 1);
            std::reverse(imageName.begin(), imageName.end());
            break;
        }
    }

    return imageName;
}

int MatchingCrater::get_offset()
{
    std::sort(DstCraterImage->matchableCraters.begin(), DstCraterImage->matchableCraters.end(),
     [](const std::shared_ptr<Crater> &a, const std::shared_ptr<Crater> &b) {return a->get_area() < b->get_area();});

    if (matchingMethod == MatchingMethod::GuidedMatching)
        return 0;
    
    const int NUM_MATCHES = totalMatchingPoints / 50;
    if (NUM_MATCHES == 0)
        return -1;
    const int NUM_THREADS = std::thread::hardware_concurrency();
    std::atomic<int> matchedPoints{0};
    std::vector<std::thread> threads;
    std::mutex mtx;

    // 创建随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());

    auto& imageSrc = SrcCraterImage;
    auto& imageDst = DstCraterImage;

    std::vector<double> offsetsX;
    std::vector<double> offsetsY;
    std::vector<keys> k1, k2;
    // 记录偏移量的总和
    double totalOffsetX = 0.0;
    double totalOffsetY = 0.0;

    double dispersionX = 0.0;
    double dispersionY = 0.0;

    for (int i = 0; i < NUM_THREADS; i++) 
    {
        threads.emplace_back([&]()
        {
            // 生成均匀分布的整数随机数
            std::uniform_int_distribution<> dis(0, imageSrc->matchableCraters.size() - 1);
            while (true)
            {
                if (matchedPoints >= NUM_MATCHES)
                    break;

                // 随机选择一个陨石坑
                int randomIndex = dis(gen);
                auto& originCrater = imageSrc->matchableCraters[randomIndex];
                
                double originx = originCrater->get_coordinates()[0];
                double originy = originCrater->get_coordinates()[1];
                double originDiameter = originCrater->get_diameter();          

                auto matchedCraters = matching_pointProgram(originCrater, imageDst);
                if (!matchedCraters.empty()) 
                {
                    int currentMatch = matchedPoints.fetch_add(1) + 1;                    
                    double maxProbability = 0;

                    // 找出最佳匹配
                    std::shared_ptr<Crater> resultCrater = nullptr;
                    for (auto& matchedCrater : matchedCraters) 
                    {
                        if (matchedCrater.second > maxProbability) 
                        {
                            maxProbability = matchedCrater.second;
                            resultCrater = matchedCrater.first;
                        }
                    }
                    if (resultCrater) 
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        k1.push_back({currentMatch, originx, originy, originDiameter});
                        k2.push_back({currentMatch, resultCrater->get_coordinates()[0],
                            resultCrater->get_coordinates()[1], resultCrater->get_diameter()});
                    }
                }
            }
        });
    }

    for (auto& thread : threads)
        thread.join();

    if (matchedPoints == 0)
        return -1;

    auto tmpSrc = vecPtSrc;
    auto tmpDst = vecPtDst;
    cv::Mat h;
    cout << FilterByHomographyMat(k1, k2, tmpSrc, tmpDst, h) << " ";
    cout << FilterByHomographyMat(tmpDst, tmpSrc, k2, k1, h) << endl;

    for (int i = 0; i < k1.size(); i++)
    {
        double offsetX = k2[i].x - k1[i].x;
        double offsetY = k2[i].y - k1[i].y;
        offsetsX.push_back(offsetX);
        offsetsY.push_back(offsetY);
        // 累加偏移量
        totalOffsetX += offsetX;
        totalOffsetY += offsetY;
    }

    // 计算平均偏移量
    double averageOffsetX = totalOffsetX / k1.size();
    double averageOffsetY = totalOffsetY / k2.size();

    // 输出平均偏移量
    std::cout << "Average Offset X: " << averageOffsetX << ", ";
    std::cout << "Average Offset Y: " << averageOffsetY << std::endl;

    // 计算偏移量的标准差
    for (double offsetX : offsetsX)    
        dispersionX += (offsetX - averageOffsetX) * (offsetX - averageOffsetX);    

    for (double offsetY : offsetsY)
        dispersionY += (offsetY - averageOffsetY) * (offsetY - averageOffsetY);

    dispersionX = std::sqrt(dispersionX / matchedPoints);
    dispersionY = std::sqrt(dispersionY / matchedPoints);

    cout << "dispersionX: " << dispersionX << ", ";
    cout << "dispersionY: " << dispersionY << endl;

    dispersion.push_back(dispersionX);
    dispersion.push_back(dispersionY);

    offset.push_back(averageOffsetX);
    offset.push_back(averageOffsetY);    

    if (std::isnan(averageOffsetX) || std::isnan(averageOffsetY) || dispersionX >= 10 || dispersionY >= 10) 
    {
        // 处理 NaN 的情况
        std::cerr << "Warning: offset error" << std::endl;
        return -1;
    }
    
    return 1;
}

cv::Point2f MatchingCrater::convertCoordinates(const cv::Point2f &originalCoord, const cv::Size &originalSize, const cv::Size &resizedSize)
{
    double scaleX = static_cast<double>(resizedSize.width) / originalSize.width;
    double scaleY = static_cast<double>(resizedSize.height) / originalSize.height;
    double newX = static_cast<double>(originalCoord.x * scaleX);
    double newY = static_cast<double>(originalCoord.y * scaleY);
    return cv::Point2f(newX, newY);
}

cv::Mat MatchingCrater::GDALBlockToMat(GDALRasterBand *band, int xoff, int yoff, int width, int height)
{
    GDALDataType dataType = band->GetRasterDataType();
    cv::Mat mat;
    switch (dataType)
    {
    case GDT_Byte:
        mat = cv::Mat(height, width, CV_8UC1);
        break;
    case GDT_UInt16:
        mat = cv::Mat(height, width, CV_16UC1);
        break;
    default:
        std::cerr << "Unsupported data type" << std::endl;
        return cv::Mat();
    }

    if (band->RasterIO(GF_Read, xoff, yoff, width, height, mat.data, width, height, dataType, 0, 0) != CE_None)
    {
        std::cerr << "RasterIO failed" << std::endl;
        return cv::Mat();
    }
    return mat;
}

/**
 * @brief 展示匹配的点
 *
 * @param key1 第一个点
 * @param key2 第二个点
 * @param imageId1 第一个点所在图像id
 * @param imageId2 第二个点所在图像id
 *
 * @return void
 */
void MatchingCrater::writeKeys(const std::vector<MatchingCrater::keys>& key1, const std::vector<MatchingCrater::keys>& key2)
{
    std::string ID = std::to_string(keyNums++);

    std::string name1 = get_imageName(SrcCraterImage->imageName);
    std::string name2 = get_imageName(DstCraterImage->imageName);

    // int status = mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

    // if (status == 0) cout << "key文件夹创建成功" << endl;
    // else        cout << "key文件夹已存在" << endl;

    tmp_savePath = folderName;

    // 定义面积范围和对应的标识
    std::vector<std::pair<double, double>> diameterRanges = {{0, 20}, {20, 50}, {50, 100}};
    std::vector<std::string> diameterLabels = {"P20m_", "P50m_", "P100m_"};

    // 用于存储每个面积范围对应的过滤后的数据
    std::vector<std::vector<MatchingCrater::keys>> filteredKey1s(diameterRanges.size());
    std::vector<std::vector<MatchingCrater::keys>> filteredKey2s(diameterRanges.size());

    // 遍历 key1 和 key2，根据面积范围将数据存入对应的向量
    for (size_t j = 0; j < key1.size(); ++j) 
    {
        double diameter = key1[j].diameter;
        for (size_t i = 0; i < diameterRanges.size(); ++i) 
        {
            auto [minDiameter, maxDiameter] = diameterRanges[i];
            if (diameter >= minDiameter && diameter < maxDiameter) 
            {
                filteredKey1s[i].push_back(key1[j]);
                filteredKey2s[i].push_back(key2[j]);
                break;
            }
        }
    }
    
    std::string fileName1 = folderName + name1 + "__" + name2 + "_L" + ".key";
    std::string fileName2 = folderName + name1 + "__" + name2 + "_R" + ".key";
    std::string fileName3 = folderName + name2 + "__" + name1 + "_L" + ".key";
    std::string fileName4 = folderName + name2 + "__" + name1 + "_R" + ".key";

    // 定义写入文件的函数
    auto writeToFile = [](std::string fileName, const std::vector<std::vector<MatchingCrater::keys>>& filteredKeys, const std::vector<std::string>& diameterLabels, int Width, int Height, size_t keySize) 
    {
        std::ofstream file(fileName);
        if (!file.is_open())
        {
            std::cerr << "open key file error" << std::endl;
            exit(1);
        }

        size_t totalSize = 0;
        for (const auto& filteredKey: filteredKeys)
            totalSize += filteredKey.size();

        file << "IC\n" << "Crater_Matched_Points\n";
        file << Width << "\n" << Height << std::endl;
        file << totalSize << std::endl;

        int index = 1;
        for (size_t i = 0; i < filteredKeys.size(); ++i) 
        {
            std::string diameterLabel = diameterLabels[i];
            for (const auto& data: filteredKeys[i]) 
            {
                file << diameterLabel << index++  << " " << data.x << " " << data.y << std::endl;
            }
        }
    };
    std::thread write1(writeToFile, fileName1, std::ref(filteredKey1s), std::ref(diameterLabels), SrcWidth, SrcHeight, key1.size());
    std::thread write2(writeToFile, fileName2, std::ref(filteredKey2s), std::ref(diameterLabels), DstWidth, DstHeight, key2.size());
    std::thread write3(writeToFile, fileName3, std::ref(filteredKey2s), std::ref(diameterLabels), DstWidth, DstHeight, key2.size());
    std::thread write4(writeToFile, fileName4, std::ref(filteredKey1s), std::ref(diameterLabels), SrcWidth, SrcHeight, key1.size());

    write1.join();
    write2.join();
    write3.join();
    write4.join();    
}
int MatchingCrater::writeLog(const std::vector<keys> &k1, const std::vector<keys> &k2, const double probability)
{
    const int minKey = 5;
    const int minRansac = 10;

    std::string name1 = get_imageName(SrcCraterImage->imageName);
    std::string name2 = get_imageName(DstCraterImage->imageName);

    std::string logName = folderName + "match.log";
    std::ofstream logfile(logName);

    if (!logfile.is_open())
    {
        std::cerr << "open log file error" << std::endl;
        return -1;
    }

    if (k1.size() > minKey && vecPtSrc.size() )
    {
        logfile << "TRUE" << std::endl;
        logfile << "matchable points:" << totalMatchingPoints << std::endl;
        logfile << "probability:" << probability << std::endl;
        logfile << "Before ransac: " << k1.size() << std::endl;
        logfile << "After ransac: " << vecPtSrc.size() << std::endl;
        logfile << "Offset: " << offset[0] << " " << offset[1] << std::endl;
        logfile << "Dispersion: " << dispersion[0] << " " << dispersion[1] << std::endl;
        logfile.close();
        return 0;
    }

    if (k1.size() <= minKey)
    {
        logfile << "FALSE" << std::endl;
        logfile << "key1 size(" << k1.size() << ")is too small" << std::endl;
        logfile << "Offset: " << offset[0] << " " << offset[1] << std::endl;
        logfile << "Dispersion: " << dispersion[0] << " " << dispersion[1] << std::endl;
        logfile.close();
        return -2;
    }
    
    if (vecPtSrc.size() <= minRansac)
    {
        logfile << "WARNING" << std::endl;
        logfile << "ransac size(" << vecPtSrc.size() << ")is too small" << std::endl;
        logfile << "Offset: " << offset[0] << " " << offset[1] << std::endl;
        logfile << "Dispersion: " << dispersion[0] << " " << dispersion[1] << std::endl;
        logfile.close();
        return -3;
    }
    return -4;
}

void MatchingCrater::show_matched_image(const std::vector<MatchingCrater::keys> &key1, const std::vector<MatchingCrater::keys> &key2)
{

    auto convertGDALBlockToMat = [this](GDALRasterBand* poSrcBand, int width, int height, cv::Mat& img) {img = this->GDALBlockToMat(poSrcBand, 0, 0, width, height);};
    
    auto convertKeypointsToPoints = [this](const std::vector<MatchingCrater::keys>& key, std::vector<cv::Point2f>& points) 
    {
        for (auto& i: key) 
            points.push_back(cv::Point2f(i.x, i.y));
    };

    std::vector<cv::Point2f> points1, points2; 

    GDALRasterBand* poSrcBand1 = datasetSrc->GetRasterBand(1);
    GDALRasterBand* poSrcBand2 = datasetDst->GetRasterBand(1);

    cv::Mat img1;
    cv::Mat img2;

    // 多线程
    std::thread ToPoint2f1(convertKeypointsToPoints, std::ref(key1), std::ref(points1));
    std::thread ToPoint2f2(convertKeypointsToPoints, std::ref(key2), std::ref(points2));
    
    std::thread ToMat1(convertGDALBlockToMat, poSrcBand1, SrcWidth, SrcHeight, std::ref(img1));
    std::thread ToMat2(convertGDALBlockToMat, poSrcBand2, DstWidth, DstHeight, std::ref(img2));  

    ToPoint2f1.join();
    ToPoint2f2.join();
    ToMat1.join();        
    ToMat2.join();          
    
    // TODO 防止内存不足提前释放    
    GDALClose(datasetSrc);    
    GDALClose(datasetDst);
    
    cout << "Mat success" << endl;

    cv::Size originalSize1 = img1.size();
    cv::Size originalSize2 = img2.size();

    cv::Size resizedSize1 = originalSize1;
    cv::Size resizedSize2 = originalSize2;

    while (img1.cols * img1.rows > 20000000)
    {
        cv::Mat resized_img;
        cv::resize(img1, resized_img, cv::Size(img1.cols / 2, img1.rows / 2));
        img1 = resized_img;
        resizedSize1 = img1.size();
    }
    while (img2.cols * img2.rows > 20000000)
    {
        cv::Mat resized_img;
        cv::resize(img2, resized_img, cv::Size(img2.cols / 2, img2.rows / 2));
        img2 = resized_img;
        resizedSize2 = img2.size();
    }

    int newWidth = img1.cols + img2.cols;
    int newHeight = std::max(img1.rows, img2.rows);
    cv::Mat Channel3img1;
    cv::Mat Channel3img2;
    cv::cvtColor(img1, Channel3img1, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img2, Channel3img2, cv::COLOR_GRAY2BGR);
    cv::Mat mergedImg(newHeight, newWidth, Channel3img1.type(), cv::Scalar(0, 0, 0));

    // 将img1和img2复制到合并图像中
    Channel3img1.copyTo(mergedImg(cv::Rect(0, 0, Channel3img1.cols, Channel3img1.rows)));
    Channel3img2.copyTo(mergedImg(cv::Rect(Channel3img1.cols, 0, Channel3img2.cols, Channel3img2.rows)));

    // 创建一个与原始图像相同大小和类型的临时图像
    cv::Mat tempImage = mergedImg.clone();

    // 在合并图像上绘制对应点之间的连线
    for (size_t i = 0; i < points1.size(); ++i)
    {
        cv::Point2f p1 = convertCoordinates(points1[i], originalSize1, resizedSize1);
        cv::Point2f p2 = convertCoordinates(points2[i], originalSize2, resizedSize2) + cv::Point2f(Channel3img1.cols, 0);
        uchar b = rand() % 256;
        uchar g = rand() % 256;
        uchar r = rand() % 256;

        uchar alpha = 1;
        cv::Scalar color = cv::Scalar(b, g, r, alpha);
        //cv::line(mergedImg, p1, p2, color, 1);
        // 在临时图像上绘制直线
        cv::line(tempImage, p1, p2, color, 1, cv::LINE_AA);
    }

    // 定义透明度，范围从 0 到 1，值越小越透明
    double alpha = 0.5;

    // 将临时图像与原始图像进行混合
    cv::addWeighted(tempImage, alpha, mergedImg, 1 - alpha, 0, mergedImg);

    std::string saveName = get_imageName(SrcCraterImage->imageName) + "_and_" + get_imageName(DstCraterImage->imageName) + ".png";
    // 显示合并后的图像
    cv::imshow(saveName, mergedImg);
    cv::imwrite(tmp_savePath + saveName, mergedImg);    
    cv::waitKey(100);
    cv::destroyWindow(saveName);
}


// bool MatchingCrater::NeighborInformation::operator <(const MatchingCrater::NeighborInformation &other) const
// {
//     return distance < other.distance;
// }


// bool MatchingCrater::CompareSet::operator ()(const std::shared_ptr<Crater> &a, const std::shared_ptr<Crater> &b) const
// {
//     return a->get_id() < b->get_id();
// }


int MatchingCrater::FilterByHomographyMat(std::vector<keys>& vecSrcKeys, std::vector<keys>& vecDstKeys, std::vector<keys>& vecSrcNew, std::vector<keys>& vecDstNew, cv::Mat& h)
{
    if (vecSrcKeys.size() != vecDstKeys.size())
        return -1;

    const int minMatchesAllowedRansac = 6;
    if (vecSrcKeys.size() < minMatchesAllowedRansac)
        return -1;

    // 从keys结构体中提取坐标点
    std::vector<cv::Point2f> srcPoints, dstPoints;
    for (size_t i = 0; i < vecSrcKeys.size(); ++i) {
        srcPoints.emplace_back(vecSrcKeys[i].x, vecSrcKeys[i].y);
        dstPoints.emplace_back(vecDstKeys[i].x, vecDstKeys[i].y);
    }

    // 计算单应矩阵并获取内点掩码
    std::vector<uchar> inliersMask;
    cv::Mat Homography = cv::findHomography(srcPoints, dstPoints, inliersMask, cv::RANSAC);
    h = Homography.clone();

    vecSrcNew.clear();
    vecDstNew.clear();
    for (size_t i = 0; i < inliersMask.size(); ++i) 
    {
        if (inliersMask[i]) 
        {
            vecSrcNew.push_back(vecSrcKeys[i]);
            vecDstNew.push_back(vecDstKeys[i]);
        }
    }

    return vecSrcKeys.size() - vecSrcNew.size();
}