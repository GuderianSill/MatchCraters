#include "MatchingCrater.hpp"
using std::cin;
using std::cout;
using std::endl;

int MatchingCrater::keyNums = 1;
bool normalExit = false;

/**
 * @brief 构造函数
 *
 * @param name1 第一个CSV文件名
 * @param name2 第二个CSV文件名
 * @param isByExcel 是否使用Excel文件
 *
 * @return 无
 */
MatchingCrater::MatchingCrater(const std::string name1, const std::string name2)
{
    GDALAllRegister();

    //配置参数, 读取配置文件
    //// this->imagesPath = "images/";
    this->tifPath = "tif/";
    this->savePath = "saves/";
    this->csvPath = "csv/";
    this->matchedByRatio = true;
    this->configFile = "config.txt";

    std::ifstream config(configFile);
    if (!config.is_open())    
        throw std::runtime_error("配置文件打开错误");

    std::map<std::string, double> configMap;
    std::string line;

    while (std::getline(config, line))
    {
        std::istringstream iss(line);
        std::string label;
        double value;
        if (iss >> label >> value)
            configMap[label] = value;
    }

    this->DOMAIN_RANGE = configMap["DOMAIN_RANGE"];
    this->DOMAIN_FACTOR_RATIO = configMap["DOMAIN_FACTOR_RATIO"];
    this->DISTANCE_MAX_GAP = configMap["DISTANCE_MAX_GAP"];
    this->ANGLE_TOLERANCE = configMap["ANGLE_TOLERANCE"];
    this->DISTANCE_RATIO = configMap["DISTANCE_RATIO"];
    this->AREA_TOLERANCE_RATIO = configMap["AREA_TOLERANCE_RATIO"];
    this->ASPECTRADIO_RADIO = configMap["ASPECTRADIO_RADIO"];
    this->SEARCH_RANGE = configMap["SEARCH_RANGE"];

    config.close();

    // 初始化图像信息
    std::string tifName1 = tifPath + name1 + ".tif";
    std::string tifName2 = tifPath + name2 + ".tif";

    dataset1 = (GDALDataset*)GDALOpen(tifName1.c_str(), GA_ReadOnly); 
    dataset2 = (GDALDataset*)GDALOpen(tifName2.c_str(), GA_ReadOnly);

    if (dataset1 == nullptr || dataset2 == nullptr)    
        throw std::runtime_error("无法打开TIFF文件");

    this->transformer1 = new GDALCoordinateTransformer(dataset1);
    this->transformer2 = new GDALCoordinateTransformer(dataset2);

    Src1Height = dataset1->GetRasterYSize();
    Src1Width = dataset1->GetRasterXSize();
    Src2Height = dataset2->GetRasterYSize();
    Src2Width = dataset2->GetRasterXSize();

    std::cout << "图像信息读取完成" << std::endl;    

    // 从CSV读取图像信息
    CSVGet_crater_object(csvPath + name1 + ".csv", csvPath + name2 + ".csv", *this);    
    getTotalMatchingPoints();

    std::cout << "陨石坑信息存储完成" << std::endl;    

    for (auto& craterImage: CraterImages)    
        craterImage->imageId = craterImage->craters[0]->get_image_id();
}

MatchingCrater::~MatchingCrater()
{
    delete transformer1;
    delete transformer2;
    normalExit = true;
}

void MatchingCrater::runMatching()
{
    std::cout << "开始匹配" << std::endl;
    build_dataStructure();
    get_NeighborInformation();
}

void MatchingCrater::get_keys()
{
    show_keys(CraterImages[0], CraterImages[1]);
    cout << "get keys success" << endl;
}

void MatchingCrater::test_get_NeighborInformation()
{
    while (true)
    {
        cout << "检验查询坑附近的坑信息" << endl;
        int imageId, craterId;
        cout << "输入查询图片id，位置id, -1退出" << endl;
        cin >> imageId;
        if (imageId == -1)
        {
            cout << "exit" << endl;
            break;
        }
        cin >> craterId;
        if (imageId < 0 || imageId >= ImagesNum)
        {
            cout <<"图片id错误" << endl;
            continue;
        }
        if (craterId <= 0 || craterId > CraterImages[imageId]->sumIds)
        {
            cout <<"陨石坑id错误" << endl;
            continue;
        }
        auto& nowCrater = CraterImages[imageId]->IdGetNeighborCraters[craterId][0]->crater;
        printf("当前查询点坐标:(%lf, %lf), 面积:%lf, 长宽比:%lf\n", nowCrater->get_coordinates()[0], nowCrater->get_coordinates()[1], nowCrater->get_area(), nowCrater->get_aspectRatio());
        if (CraterImages[imageId]->IdGetNeighborCraters[craterId].size() - 1)
        {
            for (auto& neighborCrater: CraterImages[imageId]->IdGetNeighborCraters[craterId])
            {
                cout << "编号:" << neighborCrater->crater->get_id() << " 距离:" << neighborCrater->distance << " 角度:" << neighborCrater->angle;
                printf(" x:%lf y:%lf\n", neighborCrater->crater->get_coordinates()[0], neighborCrater->crater->get_coordinates()[1]);
            }
        }
        else
            printf("周围%lf范围没有可匹配的坑\n", DOMAIN_RANGE);
    }
}

void MatchingCrater::test_matching_pointProgram()
{
    //查询相匹配的坑
    while (true)
    {
        cout << "查询相匹配的坑" << endl;
        int imageId, craterId;
        cout << "输入查询图片id，位置id, -1退出" << endl;
        cin >> imageId;
        if (imageId == -1)
        {
            cout << "exit" << endl;
            break;
        }
        cin >> craterId;
        if (imageId < 0 || imageId >= ImagesNum)
        {
            cout <<"图片id错误" << endl;
            continue;
        }
        if (craterId <= 0 || craterId > CraterImages[imageId]->sumIds)
        {
            cout <<"陨石坑id错误" << endl;
            continue;
        }
        auto& originCrater = CraterImages[imageId]->IdGetNeighborCraters[craterId];
        auto& nowCrater = originCrater[0]->crater;
        printf("当前查询点坐标:(%lf, %lf), 面积:%lf, 长宽比:%lf\n", nowCrater->get_coordinates()[0], nowCrater->get_coordinates()[1], nowCrater->get_area(), nowCrater->get_aspectRatio());
        for (int i = 0; i <= ImagesNum - 1; i++)
        {
            if (i != imageId)
            {
                auto matchedCraters = matching_pointProgram(nowCrater, i);
                if (matchedCraters.empty())
                    printf("没有在图片%d找到匹配点\n", i);
                else
                {
                    printf("在图片%d找到匹配点\n", i);
                    for (const auto& matchedCrater: matchedCraters)
                    {
                        printf("id:%d 坐标:(%lf, %lf) ", matchedCrater.first->get_id(), matchedCrater.first->get_coordinates()[0], matchedCrater.first->get_coordinates()[1]);
                        cout << "概率：" << matchedCrater.second << endl;
                    }
                }
            }
        }
    }
}

void MatchingCrater::test_showAllMatchingPoints()
{
    for (int imageId = 0; imageId <= ImagesNum - 1; imageId++)
    {
        printf("匹配图像%d所有点\n", imageId);
        int matchedPoints = 0;
        for (int i = 0; i <= ImagesNum - 1; i++)
        {
            if (i == imageId) continue;
            for (int craterId = 1; craterId <= CraterImages[imageId]->sumIds; craterId++)
            {
                auto& originCrater = CraterImages[imageId]->IdGetNeighborCraters[craterId];
                auto& nowCrater = originCrater[0]->crater;
                printf("当前查询点坐标:(%lf, %lf), id:%d, 面积:%lf, 长宽比:%lf\n", nowCrater->get_coordinates()[0], nowCrater->get_coordinates()[1], nowCrater->get_id(),
                        nowCrater->get_area(), nowCrater->get_aspectRatio());
                std::vector<std::pair<std::shared_ptr<Crater>, double>> matchedCraters = matching_pointProgram(originCrater[0]->crater, i);
                if (matchedCraters.empty())
                {
                    printf("没有找到匹配点\n");
                }
                else
                {
                    matchedPoints++;
                    printf("在图片%d找到匹配点\n", i);
                    for (const auto& matchedCrater: matchedCraters)
                    {
                        printf("坐标：(%lf, %lf) ", matchedCrater.first->get_coordinates()[0], matchedCrater.first->get_coordinates()[1]);
                        cout << "概率：" << matchedCrater.second << endl;
                    }
                }
                cout << endl;
            }
            printf("图片有%d个坑匹配成功\n", matchedPoints);
            cout << "***************************************************************************************************************************************" << endl;
        }
    }
}

/**
 * @brief 展示两个图片的匹配点
 *
 * @param image1 第一个图片的陨石坑对象
 * @param image2 第二个图片的陨石坑对象
 *
 * @return std::vector<std::pair<std::shared_ptr<Crater>, double>> 匹配成功的陨石坑对象和概率
 */
void MatchingCrater::show_keys(const std::unique_ptr<CraterImage>& image1, const std::unique_ptr<CraterImage>& image2)
{
    std::sort(image2->craters.begin(), image2->craters.end(), CompareCratersByArea);

    std::atomic<int> matchedPoints{0};
    std::vector<keys> k1, k2;
    std::mutex k_mutex; // 保护k1/k2和输出

    const int NUM_THREADS = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;
    std::atomic<int> currentId{0};

    cout << "sumPoints: " << image1->sumIds << endl;

    for (int i = 0; i < NUM_THREADS; i++)
    {
        threads.emplace_back([&]()
        {
            while (true)
            {
                //动态获取任务
                int CraterId = currentId.fetch_add(1);
                if (CraterId >= image1->sumIds) break;
                auto& originCrater = image1->IdGetNeighborCraters[CraterId];
                double originx = originCrater[0]->crater->get_coordinates()[0];
                double originy = originCrater[0]->crater->get_coordinates()[1];
                double originDiameter = originCrater[0]->crater->get_diameter();

                //将image2中与originCrater中可能匹配的点存放到容器中
                auto matchedCraters = matching_pointProgram(originCrater[0]->crater, image2->imageId);
                if (!matchedCraters.empty())
                {
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
                //同步输出
                {
                    std::lock_guard<std::mutex> lock(k_mutex);
                    cout << originCrater[0]->crater->get_id()
                         << "匹配完成\n"
                         << "matched" << matchedPoints.load() << endl;
                }
            }
        });
    }
    
    for (auto& t: threads) t.join();

    cout << "匹配概率：" << static_cast<double>(matchedPoints.load()) / totalMatchingPoints << endl;
    writeKeys(k1, k2, image1->imageId, image2->imageId);
    show_matched_image(k1, k2, image1->imageId, image2->imageId);
}

//用于对陨石坑邻域数据排序的比较函数，按照距离从小到大排
bool MatchingCrater::CompareNeighborInformation(const std::shared_ptr<MatchingCrater::NeighborInformation> &a, const std::shared_ptr<MatchingCrater::NeighborInformation> &b)
{
    return a->distance < b->distance; // 按照 distance 从小到大排序
}

//对存储进的Crater数据按照面积大小排列
bool MatchingCrater::CompareCratersByArea(const std::shared_ptr<Crater> &a, const std::shared_ptr<Crater> &b)
{
    return a->get_area() < b->get_area();
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

void MatchingCrater::getTotalMatchingPoints()
{
    for (auto& crater: CraterImages[0]->craters)
    {
        double geoX1, geoY1;
        transformer1->pixelToGeo(crater->get_coordinates()[0], crater->get_coordinates()[1], geoX1, geoY1);
        double pixelX2, pixelY2;
        // 计算像素坐标
        transformer2->geoToPixel(geoX1, geoY1, pixelX2, pixelY2);

        if (pixelX2 >= 0 && pixelX2 < Src2Width && pixelY2 >= 0 && pixelY2 < Src2Height)
        {
            totalMatchingPoints++;
        }
    }
}

void MatchingCrater::build_dataStructure()
{
    for (auto& craterImage: CraterImages)
    {      
        craterImage->kdTree = std::unique_ptr<KDTree>(new KDTree(craterImage->craters));
    }    
    cout << "数据结构构建完成" << endl;
}

void MatchingCrater::get_NeighborInformation()
{
    //cout << "start to get neighborInformations" << endl;
    cout << "开始获取邻域信息" << endl;
    for (auto& craterImage: CraterImages)
    {
        for (auto& KDCrater: craterImage->craters)
        {
            std::vector<std::shared_ptr<Crater>> neighbors = craterImage->kdTree->findNeighbors(*KDCrater, DOMAIN_RANGE);        
            std::vector<std::shared_ptr<NeighborInformation>> neighborInformations;
            //获取邻域角度，距离信息
            for (auto neighbor: neighbors)
            {
                double distance = 0;
                double angle = 0;
                distance = Crater::euclideanDistance(*KDCrater, *neighbor);
                angle = Crater::getAngle(*KDCrater, *neighbor);

                std::shared_ptr<NeighborInformation> neighborInformation = std::make_shared<NeighborInformation>(distance, angle, neighbor);
                neighborInformations.push_back(neighborInformation);
            }
            std::sort(neighborInformations.begin(), neighborInformations.end(), CompareNeighborInformation);
            //neighborInformations.erase(neighborInformations.begin());
            craterImage->neighborCraters[KDCrater] = neighborInformations;
            craterImage->IdGetNeighborCraters[KDCrater->get_id()] = neighborInformations;
        }
    }
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
    if (CraterRatio < DOMAIN_FACTOR_RATIO)
        return {false, 0};

    //cout << "坑比例匹配成功" << endl;

    for (const auto& nowa: pa)
    {
        for (auto itb = pb.begin(); itb != pb.end(); itb++)
        {
            //距离判断
            if (!matchedByRatio && std::fabs(nowa->distance - (*itb)->distance) > DISTANCE_MAX_GAP)
            {
                if (nowa->distance > (*itb)->distance)
                {
                    //pb.erase(itb);
                    continue;
                }
                else break;
            }
            else if (matchedByRatio && (std::max(nowa->distance, (*itb)->distance) / std::min(nowa->distance, (*itb)->distance)) > DISTANCE_RATIO)
            {
                //@todo, 2025-02-16
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
    //cout << "匹配成功的邻域点:" << matchedNeighborPoints << endl;
    auto probability = matchedNeighborPoints / sumNeighborPoints;
    //cout << "概率:" << probability << endl;
    if (probability * CraterRatio < DOMAIN_FACTOR_RATIO)
        return {false, CraterRatio * probability};
    //cout << "True" << endl;
    return {true, CraterRatio * probability};
}

/**
 * @brief 点匹配程序
 *
 * @param originCrater 传入点
 * @param targetImageId 目标图像id
 *
 * @return std::vector<std::pair<std::shared_ptr<Crater>, double>> 匹配成功的点的集合
 */
std::vector<std::pair<std::shared_ptr<Crater>, double>> MatchingCrater::matching_pointProgram(const std::shared_ptr<Crater>& originCrater, const int targetImageId)
{

    const auto& targetImage = CraterImages[targetImageId];
    auto& targetCraters = CraterImages[targetImageId]->craters;
    const auto originArea = originCrater->get_area();
    const auto& originImage = CraterImages[originCrater->get_image_id()];

    double lowArea = originArea / AREA_TOLERANCE_RATIO;
    double highArea = originArea * AREA_TOLERANCE_RATIO;

    auto lower = std::lower_bound(targetCraters.begin(), targetCraters.end(), std::make_shared<Crater>(lowArea), CompareCratersByArea);
    auto higher = std::upper_bound(targetCraters.begin(), targetCraters.end(), std::make_shared<Crater>(highArea), CompareCratersByArea);

    std::vector<std::shared_ptr<Crater>> similarCraters;
    for (auto it = lower; it != higher; it++)
    {
        if (std::max(it->get()->get_aspectRatio(), originCrater->get_aspectRatio())/std::min(it->get()->get_aspectRatio(), originCrater->get_aspectRatio())
                > ASPECTRADIO_RADIO)
            continue;
        else
        {
            // 转换成地理坐标
            double geoX1, geoY1, geoX2, geoY2;
            transformer1->pixelToGeo(originCrater->get_coordinates()[0], originCrater->get_coordinates()[1], geoX1, geoY1);
            transformer2->pixelToGeo(it->get()->get_coordinates()[0], it->get()->get_coordinates()[1], geoX2, geoY2);

            if (std::sqrt(std::pow(geoX1 - geoX2, 2) + std::pow(geoY1 - geoY2, 2)) <= SEARCH_RANGE)
                similarCraters.push_back(*it);
        }
    }
    /*
    for (auto& i : similarCraters)
        printf("%lf %lf, area:%lf aspectRatio:%lf id:%d imgId:%d\n", i->get_coordinates()[0], i->get_coordinates()[1], i->get_area(), i->get_aspectRatio(), i->get_id(), i->get_image_id());
    */

    std::vector<std::pair<std::shared_ptr<Crater>, double>> results;
    for (auto it = similarCraters.begin(); it != similarCraters.end(); it++)
    {
        auto judgeProbability = judge_matchingPoint(originImage->neighborCraters[originCrater], targetImage->neighborCraters[*it]);
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
 * @brief 展示匹配的点
 *
 * @param key1 第一个点
 * @param key2 第二个点
 * @param imageId1 第一个点所在图像id
 * @param imageId2 第二个点所在图像id
 *
 * @return void
 */
void MatchingCrater::writeKeys(const std::vector<MatchingCrater::keys> &key1, const std::vector<MatchingCrater::keys> &key2, int imageId1, int imageId2)
{
    std::string ID = std::to_string(keyNums++);

    std::string name1 = get_imageName(CraterImages[imageId1]->imageName);
    std::string name2 = get_imageName(CraterImages[imageId2]->imageName);

    std::string folderName = savePath + name1 + "_and_" + name2 + "/";

    int status = mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

    if (status == 0) cout << "key文件夹创建成功" << endl;
    else        cout << "key文件夹已存在" << endl;

    tmp_savePath = folderName;

    // 定义面积范围和对应的标识
    std::vector<std::pair<double, double>> diameterRanges = {{0, 20}, {20, 50}, {50, 100}};
    std::vector<std::string> diameterLabels = {"20", "20_50", "50_100"};

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

    // 将内存中的数据写入文件
    for (size_t i = 0; i < diameterRanges.size(); ++i) 
    {
        int index = 1;
        auto [minDiameter, maxDiameter] = diameterRanges[i];
        std::string diameterLabel = diameterLabels[i];

        auto fileName1 = folderName + name1 + "_" + name2 + "_L_" + diameterLabel + ".key";
        auto fileName2 = folderName + name1 + "_" + name2 + "_R_" + diameterLabel + ".key";

        std::ofstream file1(fileName1);
        std::ofstream file2(fileName2);

        if (!file1.is_open() || !file2.is_open())
        {
            std::cerr << "open key file error" << std::endl;
            exit(1);
        }

        file1 << "IC\n" << "Crater Matched Points\n";
        file2 << "IC\n" << "Crater Matched Points\n";

        file1 << Src1Width << "\n" << Src1Height << std::endl;
        file2 << Src2Width << "\n" << Src2Height << std::endl;

        file1 << filteredKey1s[i].size() << std::endl;
        for (const auto& data: filteredKey1s[i]) 
        {
            file1 << index++ << " " << data.x << " " << data.y << std::endl;
        }
        file1.close();

        index = 1;
        file2 << filteredKey2s[i].size() << std::endl;
        for (const auto& data: filteredKey2s[i]) 
        {
            file2 << index++ << " " << data.x << " " << data.y << std::endl;
        }
        file2.close();
    }
} 

void MatchingCrater::show_matched_image(const std::vector<MatchingCrater::keys> &key1, const std::vector<MatchingCrater::keys> &key2, int imageId1, int imageId2)
{
    std::vector<cv::Point2f> points1, points2;
    for (auto& i: key1)
        points1.push_back({cv::Point2f(i.x, i.y)});
    for (auto& i: key2)
        points2.push_back({cv::Point2f(i.x, i.y)});

    GDALRasterBand* poSrcBand1 = dataset1->GetRasterBand(1);
    GDALRasterBand* poSrcBand2 = dataset2->GetRasterBand(1);

    cv::Mat img1 = GDALBlockToMat(poSrcBand1, 0, 0, Src1Width, Src1Height);
    cv::Mat img2 = GDALBlockToMat(poSrcBand2, 0, 0, Src2Width, Src2Height);
    cout << "Mat success" << endl;

    // TODO 防止内存不足提前释放
    GDALClose(dataset1);
    GDALClose(dataset2);

    cv::Size originalSize1 = img1.size();
    cv::Size originalSize2 = img2.size();

    cv::Size resizedSize1 = img1.size();
    cv::Size resizedSize2 = img2.size();

    while (img1.cols * img1.rows > 2000000)
    {
        cv::Mat resized_img;
        cv::resize(img1, resized_img, cv::Size(img1.cols / 2, img1.rows / 2));
        img1 = resized_img;
        resizedSize1 = img1.size();
    }
    while (img2.cols * img2.rows > 2000000)
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

    // !
    /*
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    for (const auto& point : points1) {
        cv::Point2f p = convertCoordinates(point, originalSize1, resizedSize1);
        keypoints1.emplace_back(p, 1.0);
    }

    for (const auto& point : points2) {
        cv::Point2f p = convertCoordinates(point, originalSize2, resizedSize2);
        keypoints2.emplace_back(p, 1.0);
    }

    // 创建 DMatch 向量
    std::vector<cv::DMatch> matches;
    for (size_t i = 0; i < points1.size(); ++i) {
        matches.emplace_back(static_cast<int>(i), static_cast<int>(i), 0.0);
    }

    // 绘制匹配结果
    cv::Mat img_matches;
    drawMatches(img1, keypoints1, img2, keypoints2,
                matches, img_matches, cv::Scalar::all(-1),
                cv::Scalar::all(-1), std::vector<char>(),
                //cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
                cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
                );

    // 显示匹配结果
    cv::imshow("Matches", img_matches);
    cv::waitKey(0);
    */
    // !

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
    double alpha = 0.2;

    // 将临时图像与原始图像进行混合
    cv::addWeighted(tempImage, alpha, mergedImg, 1 - alpha, 0, mergedImg);

    std::string saveName = get_imageName(CraterImages[imageId1]->imageName) + " and " + get_imageName(CraterImages[imageId2]->imageName) + ".png";
    // 显示合并后的图像
    cv::imshow(saveName, mergedImg);
    cv::imwrite(tmp_savePath + saveName, mergedImg);    
    cv::waitKey(10);
    cv::destroyWindow(saveName);
}


bool MatchingCrater::NeighborInformation::operator <(const MatchingCrater::NeighborInformation &other) const
{
    return distance < other.distance;
}


bool MatchingCrater::CompareSet::operator ()(const std::shared_ptr<Crater> &a, const std::shared_ptr<Crater> &b) const
{
    return a->get_id() < b->get_id();
}
