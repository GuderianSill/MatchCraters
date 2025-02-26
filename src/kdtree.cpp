#include "kdtree.hpp"
#include <thread>
#include <mutex>

using std::cout;
using std::endl;

// KDNode 构造函数的实现
KDTree::KDNode::KDNode(std::shared_ptr<Crater> p, int d) : point(std::move(p)), left(nullptr), right(nullptr), depth(d) {}


// KDTree 构造函数的实现
KDTree::KDTree(std::vector<std::shared_ptr<Crater>>& craters)
{
    k = craters[0]->get_coordinates().size();
    root = buildKDTree(craters);
}


// 查找邻居的函数实现
std::vector<std::shared_ptr<Crater>> KDTree::findNeighbors(const Crater& target, double range) const
{
    std::vector<std::shared_ptr<Crater>> neighbors;
    rangeSearch(root, target, range, neighbors);
    return neighbors;
}


// 递归构建 K-D 树的函数实现
std::unique_ptr<KDTree::KDNode> KDTree::buildKDTree(std::vector<std::shared_ptr<Crater>>& points, int depth)
{
    if (points.empty()) return nullptr;
    int axis = depth % k;
    auto medianIter = points.begin() + points.size() / 2;
    std::nth_element(points.begin(), medianIter, points.end(),
           [axis](const std::shared_ptr<Crater>& p1, const std::shared_ptr<Crater>& p2)
    { return p1->get_coordinates()[axis] < p2->get_coordinates()[axis]; });
    //std::unique_ptr<KDNode> node = std::make_unique<KDNode>(*medianIter, depth);
    std::unique_ptr<KDNode> node = std::unique_ptr<KDNode>(new KDNode(*medianIter, depth));
    std::vector<std::shared_ptr<Crater>> leftPoints(points.begin(), medianIter);
    std::vector<std::shared_ptr<Crater>> rightPoints(medianIter + 1, points.end());
    node->left = buildKDTree(leftPoints, depth + 1);
    node->right = buildKDTree(rightPoints, depth + 1);
    return node;
    // std::thread leftThread;
    // std::unique_ptr<KDTree::KDNode> leftSubtree;
    // if (!leftPoints.empty()) 
    // {
    //     leftThread = std::thread([this, &leftSubtree, &leftPoints, depth]() 
    //     {
    //         leftSubtree = this->buildKDTree(leftPoints, depth + 1);
    //     });
    // }
    // std::unique_ptr<KDNode> rightSubtree = buildKDTree(rightPoints, depth + 1);

    // if (leftThread.joinable()) 
    // {
    //     leftThread.join();
    // }

    // node->left = std::move(leftSubtree);
    // node->right = std::move(rightSubtree);
    // return node;
}


// 范围搜索的函数实现
void KDTree::rangeSearch(const std::unique_ptr<KDNode>& node, const Crater& target,
                         double range, std::vector<std::shared_ptr<Crater>>& result) const
{
    if (node == nullptr) return;
    double dist = Crater::euclideanDistance(*(node->point), target);
    if (dist <= range)
    {
        result.push_back(node->point);
    }
    int axis = node->depth % k;
    if ((target.get_coordinates()[axis] - range) <= node->point->get_coordinates()[axis])
    {
        rangeSearch(node->left, target, range, result);
    }
    if ((target.get_coordinates()[axis] + range) >= node->point->get_coordinates()[axis])
    {
        rangeSearch(node->right, target, range, result);
    }
}

// 范围搜索的函数实现
// void KDTree::rangeSearch(const std::unique_ptr<KDNode>& node, const Crater& target,
//                          double range, std::vector<std::shared_ptr<Crater>>& result) const
// {
//     if (node == nullptr) return;
//     double dist = Crater::euclideanDistance(*(node->point), target);
//     if (dist <= range)
//     {
//         std::lock_guard<std::mutex> lock(resultMutex);
//         result.push_back(node->point);
//     }
//     int axis = node->depth % k;
//     if ((target.get_coordinates()[axis] - range) <= node->point->get_coordinates()[axis])
//     {
//         rangeSearch(node->left, target, range, result);
//     }
//     if ((target.get_coordinates()[axis] + range) >= node->point->get_coordinates()[axis])
//     {
//         rangeSearch(node->right, target, range, result);
//     }
// }

// // 并行范围搜索的函数实现
// void KDTree::parallelRangeSearch(const std::unique_ptr<KDNode>& node, const Crater& target,
//                                  double range, std::vector<std::shared_ptr<Crater>>& result) const
// {
//     if (node == nullptr) return;

//     std::thread leftThread;
//     std::vector<std::shared_ptr<Crater>> leftResult;
//     int axis = node->depth % k;
//     if ((target.get_coordinates()[axis] - range) <= node->point->get_coordinates()[axis]) {
//         leftThread = std::thread([this, &node, &target, range, &leftResult]() {
//             rangeSearch(node->left, target, range, leftResult);
//         });
//     }

//     std::vector<std::shared_ptr<Crater>> rightResult;
//     if ((target.get_coordinates()[axis] + range) >= node->point->get_coordinates()[axis]) {
//         rangeSearch(node->right, target, range, rightResult);
//     }

//     if (leftThread.joinable()) {
//         leftThread.join();
//     }

//     {
//         std::lock_guard<std::mutex> lock(resultMutex);
//         result.insert(result.end(), leftResult.begin(), leftResult.end());
//         result.insert(result.end(), rightResult.begin(), rightResult.end());
//     }
// }