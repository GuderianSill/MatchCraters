#ifndef GETDATA_H
#define GETDATA_H

#include "crater.hpp"
#include "kdtree.hpp"
#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include "MatchingCrater.hpp"
class MatchingCrater;

static std::atomic<bool> shouldExit;
extern bool normalExit;

void checkMemoryUsage();
void CSVGet_crater_object(const std::string name1, const std::string name2, MatchingCrater& matchingCrater);


#endif
