#ifndef PYFUCUSE_H
#define PYFUCUSE_H
#include "crater.hpp"
#include "kdtree.hpp"
#include <string>
#include <iostream>
#include "MatchingCrater.hpp"
class MatchingCrater;

void ExcelGet_crater_object(const std::string name1, const std::string name2, MatchingCrater& matchingCrater);


#endif // PYFUCUSE_H
