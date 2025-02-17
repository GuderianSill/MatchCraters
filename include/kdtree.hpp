#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <memory>
#include "crater.hpp"
#include <algorithm>
#include <iostream>


class Crater;


class KDTree
{
private:
    // K-D 树节点结构体
    struct KDNode
    {
        std::shared_ptr<Crater> point;
        std::unique_ptr<KDNode> left;
        std::unique_ptr<KDNode> right;
        int depth;


        // 构造函数，初始化 KDNode 对象
        KDNode(std::shared_ptr<Crater> p, int d);
    };


    std::unique_ptr<KDNode> root;
    int k;


    // 递归构建 K-D 树的私有函数
    std::unique_ptr<KDNode> buildKDTree(std::vector<std::shared_ptr<Crater>>& points, int depth = 0);


    // 范围搜索的私有函数，查找范围内的邻居
    void rangeSearch(const std::unique_ptr<KDNode>& node, const Crater& target, double range, std::vector<std::shared_ptr<Crater>>& result) const;

public:
    KDTree(std::vector<std::shared_ptr<Crater>>& craters);

    // 查找目标点周围一定范围内的邻居
    std::vector<std::shared_ptr<Crater>> findNeighbors(const Crater& target, double range) const;


};


#endif
