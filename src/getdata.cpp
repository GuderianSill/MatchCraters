#include "getdata.hpp"
using std::cout;
using std::endl;

void CSVGet_crater_object(const std::string name1, const std::string name2, MatchingCrater& matchingCrater)
{
    struct Data
    {
        int index;
        double x, y, w, h, conf, r;
    };
    std::ifstream file1(name1);
    std::ifstream file2(name2);
    if (!file1.is_open() || !file2.is_open())
    {
        throw std::runtime_error("无法打开文件");
        return;
    }
    matchingCrater.CraterImages.push_back(std::unique_ptr<MatchingCrater::CraterImage>(new MatchingCrater::CraterImage));
    matchingCrater.CraterImages.push_back(std::unique_ptr<MatchingCrater::CraterImage>(new MatchingCrater::CraterImage));
    matchingCrater.ImagesNum = 2;
    std::string line;
    std::getline(file1, line);
    std::getline(file2, line);

    int sumId1 = 0, sumId2 = 0;
    while (std::getline(file1, line))
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
        matchingCrater.CraterImages[0]->craters.push_back(tmpCrater);
        sumId1 = data.index + 1;
    }

    while (std::getline(file2, line))
    {
        cout << line << endl;
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
        matchingCrater.CraterImages[1]->craters.push_back(tmpCrater);
        sumId2 = data.index + 1;
    }

    matchingCrater.CraterImages[0]->sumIds = sumId1;
    matchingCrater.CraterImages[0]->imageName = name1;
    matchingCrater.CraterImages[1]->sumIds = sumId2;
    matchingCrater.CraterImages[1]->imageName = name2;
    file1.close();
    file2.close();
}
