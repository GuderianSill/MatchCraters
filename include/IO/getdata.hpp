#ifndef GETDATA_H
#define GETDATA_H

#include "crater.hpp"
#include "kdtree.hpp"
#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include <memory>
#include "MatchingCrater.hpp"
class MatchingCrater;

static std::atomic<bool> shouldExit;
extern bool normalExit;

void checkMemoryUsage();
void CSVGet_crater_object(const std::string SrcName, const std::string DstName, MatchingCrater& matchingCrater);


#endif
