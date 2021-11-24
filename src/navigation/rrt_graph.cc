#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/util/random.h"
#include "shared/math/geometry.h"
#include "rrt_graph.h"
#include <iostream>
#include <utility>
#include <deque>

using Eigen::Vector2f;
using std::string;
using std::vector;

RRTGraph::RRTGraph():
    init_node(0,0), random(0,0)
{

}

void RRTGraph::ClearTree()
{
    tree_vertex.clear();
    tree_edge.clear();
}

void RRTGraph::ClearPath()
{
    path_to_goal.clear();
}

void RRTGraph::SetInitNode(const Eigen::Vector2f& node)
{
    init_node << node;
    tree_vertex.push_back(init_node);
}

Eigen::Vector2f RRTGraph::GetInitNode()
{
    return init_node;
}

void RRTGraph::AddVertex(Eigen::Vector2f q_new)
{
    tree_vertex.push_back(q_new);
}

std::vector<Eigen::Vector2f> RRTGraph::GetVertices()
{
    return tree_vertex;
}

void RRTGraph::AddEdge(Eigen::Vector2f q_near, Eigen::Vector2f q_new)
{
    std::vector<Eigen::Vector2f> tree_pair;
    tree_pair.push_back(q_near);
    tree_pair.push_back(q_new);
    tree_edge.push_back(tree_pair);
}

std::vector<std::vector<Eigen::Vector2f>> RRTGraph::GetEdges()
{
    return tree_edge;
}

Eigen::Vector2f RRTGraph::GetRandq(float Cx, float Cy)
{
    random.x() = rng_.UniformRandom(-Cx, Cx);
    random.y() = rng_.UniformRandom(-Cy, Cy);
    return random;
}

Eigen::Vector2f RRTGraph::GetClosestq(const Eigen::Vector2f q_rand)
{
    Eigen::Vector2f delta_q (0,0);
    Eigen::Vector2f closest_q (0,0);
    float magnitude {0};
    float min_dist {500000};  //potential bug

    for (auto& q:tree_vertex){
        delta_q = q_rand - q;
        magnitude = delta_q.norm();
        if (magnitude < min_dist){
            closest_q = q;
            min_dist = magnitude;
        }
    }

    return closest_q;
}

Eigen::Vector2f RRTGraph::GetNewq(const Eigen::Vector2f q_near, const Eigen::Vector2f q_rand, const float max_delta_q)
{
    Eigen::Vector2f q = q_rand;    // output vector
    float angle {0};               // angle of magnitudes

    Eigen::Vector2f delta_q = q_rand - q_near; // delta between nodes
    float magnitude = delta_q.norm();          // magnitude of nodes

    if (magnitude > max_delta_q){   
        angle = geometry::Angle(delta_q);
        q.x() = q_near.x() + max_delta_q*cos(angle);
        q.y() = q_near.y() + max_delta_q*sin(angle);
    }

    return q;
}

bool RRTGraph::IsNearGoal(const Eigen::Vector2f q, const Eigen::Vector2f q_goal, const float threshold)
{
    bool is_close = false;
    Eigen::Vector2f delta_q = q_goal - q;
    float magnitude = delta_q.norm();
    if (magnitude < threshold)
        is_close = true;

    return is_close; 
}

void RRTGraph::FindPathBack(const Eigen::Vector2f q_near, const Eigen::Vector2f q_new)
{   
    path_to_goal.push_front(q_new);
    int vec_size = tree_edge.size()-1;
    Eigen::Vector2f closest = q_near;
    while(closest != init_node){
        for (int i = vec_size; i >= 0; i--){
            if (tree_edge[i][1] == closest){
                closest = tree_edge[i][0];
                path_to_goal.push_front(closest);
            }
        }
    }
}

std::deque<Eigen::Vector2f> RRTGraph::GetPathBack()
{
    return path_to_goal;
}

void RRTGraph::TestFunc()
{
    std::cout <<"testing 1, 2" << std::endl;
} 