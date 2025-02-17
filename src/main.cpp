#include "MatchingCrater.hpp"
#include "debug.cxx"

using namespace std;

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        cout << "login success" << endl;
        std::string csvPath = "csv/";
        std::string file1 = csvPath + argv[1];
        std::string file2 = csvPath + argv[2];
        cout << file1 << " " << file2 << endl;
        //bool flag = std::string(argv[3]) == "true";
        bool flag = true;
        MatchingCrater* MC = new MatchingCrater(file1, file2, flag);
        MC->runMatching();
        MC->test_keys();
    }
    return 0;
}
