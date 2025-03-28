#include "getdata.hpp"
using std::cout;
using std::endl;

// 实时检查内存占用情况
void checkMemoryUsage() 
{
    while (!shouldExit && !normalExit) 
    {
        std::ifstream meminfo("/proc/meminfo");
        if (!meminfo.is_open()) 
        {
            std::cerr << "Failed to open /proc/meminfo" << std::endl;
            shouldExit = true;
            break;
        }

        size_t totalMemory = 0;
        size_t availableMemory = 0;
        std::string line;
        while (std::getline(meminfo, line)) 
        {
            if (line.find("MemTotal:") != std::string::npos) 
            {
                sscanf(line.c_str(), "MemTotal: %lu kB", &totalMemory);
            } else if (line.find("MemAvailable:") != std::string::npos) {
                sscanf(line.c_str(), "MemAvailable: %lu kB", &availableMemory);
            }
        }
        meminfo.close();

        if (totalMemory > 0) 
        {
            // 计算内存占用百分比
            double memoryUsage = 100.0 - (static_cast<double>(availableMemory) / totalMemory * 100);

            if (memoryUsage >= 92) 
            {
                std::cerr << "Memory usage reached 92%. Exiting program." << std::endl;
                exit(1);                
            }
        }

        // 每隔 0.1 秒检查一次
        //std::this_thread::sleep_for(std::chrono::seconds(1));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void CSVGet_crater_object(const std::string SrcName, const std::string DstName, MatchingCrater& matchingCrater)
{
    struct Data
    {
        int index;
        double x, y, w, h, conf, r;
    };
    std::ifstream SrcFile(SrcName);
    std::ifstream Dstfile(DstName);
    if (!SrcFile.is_open() || !Dstfile.is_open())
    {
        throw std::runtime_error("\n无法打开CSV文件");
        return;
    }

    matchingCrater.SrcCraterImage = std::make_unique<MatchingCrater::CraterImage>();
    matchingCrater.DstCraterImage = std::make_unique<MatchingCrater::CraterImage>();    

    std::string line;
    std::getline(SrcFile, line);
    std::getline(Dstfile, line);

    int SrcSumId = 0, DstSumId = 0;
    while (std::getline(SrcFile, line))
    {
        std::istringstream iss(line);
        std::string token;
        Data data;
        std::getline(iss, token, ',');
        data.index = std::stoi(token);
        std::getline(iss, token, ',');
        data.x = std::stod(token);
        std::getline(iss, token, ',');
        data.y = std::stod(token);
        std::getline(iss, token, ',');
        data.w = std::stod(token);
        std::getline(iss, token, ',');
        data.h = std::stod(token);
        std::getline(iss, token, ',');
        data.conf = std::stod(token);
        std::getline(iss, token, ',');
        data.r = std::stod(token);
        std::vector<double> coordinates = {data.x, data.y};
        double area = M_PI * data.r * data.r;
        double diameter = 2 * data.r;
        auto tmpCrater = std::make_shared<Crater>(coordinates, data.w, data.h, area, diameter, data.index, 0);        
        matchingCrater.SrcCraterImage->craters.push_back(tmpCrater);
        SrcSumId = data.index + 1;
    }

    while (std::getline(Dstfile, line))
    {
        std::istringstream iss(line);
        std::string token;
        Data data;
        std::getline(iss, token, ',');
        data.index = std::stoi(token);
        std::getline(iss, token, ',');
        data.x = std::stod(token);
        std::getline(iss, token, ',');
        data.y = std::stod(token);
        std::getline(iss, token, ',');
        data.w = std::stod(token);
        std::getline(iss, token, ',');
        data.h = std::stod(token);
        std::getline(iss, token, ',');
        data.conf = std::stod(token);
        std::getline(iss, token, ',');
        data.r = std::stod(token);
        std::vector<double> coordinates = {data.x, data.y};
        double area = M_PI * data.r * data.r;
        double diameter = 2 * data.r;
        auto tmpCrater = std::make_shared<Crater>(coordinates, data.w, data.h, area, diameter, data.index, 1);        
        matchingCrater.DstCraterImage->craters.push_back(tmpCrater);
        DstSumId = data.index + 1;
    }

    matchingCrater.SrcCraterImage->sumIds = SrcSumId;
    matchingCrater.DstCraterImage->sumIds = DstSumId;
    matchingCrater.SrcCraterImage->imageName = SrcName;
    matchingCrater.DstCraterImage->imageName = DstName;

    SrcFile.close();
    Dstfile.close();
}
