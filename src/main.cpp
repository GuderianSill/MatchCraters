#include "MatchingCrater.hpp"
#include "getdata.hpp"
#include "debug.cxx"

using namespace std;

int main(int argc, char *argv[])
{
    if (argc == 3)
    {         
        // 启动内存检查线程        
        cout << "\n匹配目标：" << argv[1] << " " << argv[2] << endl;
        std::thread memoryThread(checkMemoryUsage);
        MatchingCrater* MC = new MatchingCrater(argv[1], argv[2]);
        MC->runMatching();
        MC->get_keys();
        delete MC;
        cout << "匹配完成" << endl;
        memoryThread.join();
    }
    return 0;
}
