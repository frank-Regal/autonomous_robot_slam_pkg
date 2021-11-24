
#include <vector>

#include "eigen3/Eigen/Dense"
#include "shared/util/random.h"
#include <deque>
#include <utility>

#ifndef RRT_GRAPH_H
#define RRT_GRAPH_H

class RRTGraph
{
private:
    std::vector<Eigen::Vector2f> tree_vertex;
    std::vector<std::vector<Eigen::Vector2f>> tree_edge;

    Eigen::Vector2f init_node;
    std::deque<Eigen::Vector2f> path_to_goal;
    util_random::Random rng_;
    Eigen::Vector2f random;

public:

RRTGraph();

void SetInitNode(const Eigen::Vector2f& node);

Eigen::Vector2f GetInitNode();

void AddVertex(Eigen::Vector2f q_new);

void AddEdge(Eigen::Vector2f q_near, Eigen::Vector2f q_new);

Eigen::Vector2f GetRandq(const float Cx, const float Cy);

Eigen::Vector2f GetClosestq(const Eigen::Vector2f q_rand);

Eigen::Vector2f GetNewq(const Eigen::Vector2f q_near, const Eigen::Vector2f q_rand, const float Max_delta_q);

bool IsNearGoal(const Eigen::Vector2f q, const Eigen::Vector2f q_goal, const float threshold);

void FindPathBack(const Eigen::Vector2f q_near, const Eigen::Vector2f q_new);

void ClearTree();

void TestFunc();

std::vector<Eigen::Vector2f> GetVertices();

std::vector<std::vector<Eigen::Vector2f>> GetEdges();

std::deque<Eigen::Vector2f> GetPathBack();

void ClearPath();

};

#endif // RRT_GRAPH_H
