#include "MatchingCrater.hpp"
#include "debug.cxx"

using namespace std;

int main(int argc, char *argv[])
{
    if (argc == 3)
    {         
        MatchingCrater* MC = new MatchingCrater(argv[1], argv[2]);
        MC->runMatching();
        MC->get_keys();
    }
    return 0;
}
