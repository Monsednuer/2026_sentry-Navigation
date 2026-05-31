#ifndef ASTAR_H_
#define ASTAR_H_
#include "ulitys.h"
#include <string>
namespace navi_planner
{
class Astar {
private:

  std::vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable0 expanded_nodes_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::vector<NodePtr> path_nodes_;


  ESDF_enviroment::Ptr esdf_environment_;
  bool has_path_ = false;


  double lambda_heu_;
  double dynamic_penalty_weight_;
  double margin_;
  int allocate_num_;
  double tie_breaker_;
  double obstacle_weight_;
  double min_clearance_cells_;
  bool enable_start_escape_mode_;
  int start_escape_steps_;

  double resolution_, inv_resolution_;
  Eigen::Vector2i origin_, map_size_2d_;
  double time_origin_;

  enum SearchExitReason {
    SEARCH_EXIT_NONE = 0,
    SEARCH_EXIT_REACH_END = 1,
    SEARCH_EXIT_NODE_POOL_EXHAUSTED = 2,
    SEARCH_EXIT_OPEN_SET_EMPTY = 3
  };

  struct SearchDebugInfo {
    bool valid = false;
    bool path_found = false;
    int exit_reason = SEARCH_EXIT_NONE;

    Eigen::Vector2i start_idx = Eigen::Vector2i::Zero();
    Eigen::Vector2i goal_idx = Eigen::Vector2i::Zero();
    bool start_in_collision = false;
    bool goal_in_collision = false;
    double start_dist_cells = 0.0;
    double goal_dist_cells = 0.0;
    bool escape_mode_used = false;
    int escape_budget_steps = 0;

    int expanded_nodes = 0;
    int discovered_nodes = 0;
    int relaxed_nodes = 0;
    int max_open_set_size = 0;

    int neighbor_candidates = 0;
    int reject_out_of_map = 0;
    int reject_in_close_set = 0;
    int reject_collision = 0;
    int reject_clearance = 0;
    int bypass_escape_constraints = 0;
  };

  SearchDebugInfo last_search_debug_;

  Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
  void retrievePath(NodePtr end_node);

  double getDistCost(double dist_);
  double getEuclHeu(Eigen::Vector2i x1, Eigen::Vector2i x2);
  static const char* searchExitReasonToString(int reason);

public:
  Astar();
  ~Astar();

  enum { REACH_END = 1, NO_PATH = 2 };

  void setParam(
    double obstacle_weight = 10.0,
    double dynamic_penalty_weight_ = 0.0,
    double min_clearance_cells = 0.0,
    bool enable_start_escape_mode = true,
    int start_escape_steps = 16);
  void init();
  void reset();
  int search(Eigen::Vector2i start_pt, Eigen::Vector2i end_pt);
  void setEnvironment(const ESDF_enviroment::Ptr& env);

  std::vector<Eigen::Vector2i> getPath();
  std::vector<NodePtr> getVisitedNodes();
  std::string formatLastSearchDebugInfo() const;

  typedef std::shared_ptr<Astar> Ptr;
};
}
#endif
