#ifndef _ULITYS_H
#define _ULITYS_H

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <unordered_map>
#include <queue>
#include "esdf_map.h"

// #define IN_CLOSE_SET 'a'
// #define IN_OPEN_SET 'b'
// #define NOT_EXPAND 'c'

enum{IN_CLOSE_SET = 3,IN_OPEN_SET,NOT_EXPAND};

class Node {
public:
  /* -------------------- */
  Eigen::Vector2i index;
  Eigen::Vector2i position;
  double g_score, f_score;
  int escape_step;
  Node* parent;
  char node_state;

  /* -------------------- */
  Node() {
    parent = NULL;
    escape_step = 0;
    node_state = NOT_EXPAND;
  }
  ~Node(){};
};
typedef Node* NodePtr;

class NodeComparator0 {
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash0 : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (Eigen::Index i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable0 {
private:
  /* data */
  std::unordered_map<Eigen::Vector2i, NodePtr, matrix_hash0<Eigen::Vector2i>> data_2d_;

public:
  NodeHashTable0(/* args */) {
  }
  ~NodeHashTable0() {
  }
  void insert(Eigen::Vector2i idx, NodePtr node) {
    data_2d_.insert(std::make_pair(idx, node));
  }

  NodePtr find(Eigen::Vector2i idx) {
    auto iter = data_2d_.find(idx);
    return iter == data_2d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_2d_.clear();
  }
};

// class Astar {
// private:
//   /* ---------- main data structure ---------- */
//   std::vector<NodePtr> path_node_pool_;
//   int use_node_num_, iter_num_;
//   NodeHashTable0 expanded_nodes_;
//   std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
//   std::vector<NodePtr> path_nodes_;

//   /* ---------- record data ---------- */
//   ESDF_enviroment::Ptr esdf_environment_;
//   bool has_path_ = false;

//   /* ---------- parameter ---------- */
//   /* search */
//   double lambda_heu_;
//   double margin_;
//   int allocate_num_;
//   double tie_breaker_;
//   /* map */
//   double resolution_, inv_resolution_;
//   Eigen::Vector3d origin_, map_size_2d_;
//   double time_origin_;

//   /* helper */
//   Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
//   void retrievePath(NodePtr end_node);

//   /* heuristic function */
//   double getEuclHeu(Eigen::Vector2i x1, Eigen::Vector2i x2);

// public:
//   Astar(){};
//   ~Astar() {
//     for (int i = 0; i < allocate_num_; i++) {
//         delete path_node_pool_[i];
//     }
//   }

//   enum { REACH_END = 1, NO_PATH = 2 };

//   /* main API */
//   void setParam(ros::NodeHandle& nh);
//   void init();
//   void reset();
//   int search(Eigen::Vector2i start_pt, Eigen::Vector2i end_pt);

//   //void setEnvironment(const EDTEnvironment::Ptr& env);
//   std::vector<Eigen::Vector2i> getPath();
//   std::vector<NodePtr> getVisitedNodes();

//   typedef std::shared_ptr<Astar> Ptr;
// };



#endif
