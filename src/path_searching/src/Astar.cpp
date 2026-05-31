#include "Astar.h"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

namespace navi_planner
{
    const char* Astar::searchExitReasonToString(int reason)
    {
        switch (reason) {
        case SEARCH_EXIT_REACH_END:
            return "reach_end";
        case SEARCH_EXIT_NODE_POOL_EXHAUSTED:
            return "node_pool_exhausted";
        case SEARCH_EXIT_OPEN_SET_EMPTY:
            return "open_set_exhausted";
        default:
            return "unknown";
        }
    }

    std::string Astar::formatLastSearchDebugInfo() const
    {
        if (!last_search_debug_.valid) {
            return "search_debug=unavailable";
        }

        const SearchDebugInfo& s = last_search_debug_;
        std::ostringstream ss;
        ss
            << "exit=" << searchExitReasonToString(s.exit_reason)
            << " start_idx=(" << s.start_idx[0] << "," << s.start_idx[1] << ")"
            << " goal_idx=(" << s.goal_idx[0] << "," << s.goal_idx[1] << ")"
            << " start_collision=" << (s.start_in_collision ? "true" : "false")
            << " goal_collision=" << (s.goal_in_collision ? "true" : "false")
            << " start_dist_cells=" << s.start_dist_cells
            << " goal_dist_cells=" << s.goal_dist_cells
            << " escape_mode=" << (s.escape_mode_used ? "true" : "false")
            << " escape_budget_steps=" << s.escape_budget_steps
            << " expanded_nodes=" << s.expanded_nodes
            << " discovered_nodes=" << s.discovered_nodes
            << " relaxed_nodes=" << s.relaxed_nodes
            << " max_open_set=" << s.max_open_set_size
            << " neighbor_candidates=" << s.neighbor_candidates
            << " reject_out_of_map=" << s.reject_out_of_map
            << " reject_in_close_set=" << s.reject_in_close_set
            << " reject_collision=" << s.reject_collision
            << " reject_clearance=" << s.reject_clearance
            << " bypass_escape_constraints=" << s.bypass_escape_constraints;
        return ss.str();
    }

    int Astar::search(Eigen::Vector2i start_pt, Eigen::Vector2i end_pt) {
        last_search_debug_ = SearchDebugInfo();
        last_search_debug_.valid = true;
        last_search_debug_.start_idx = start_pt;
        last_search_debug_.goal_idx = end_pt;
        has_path_ = false;

        NodePtr cur_node = path_node_pool_[0];
        cur_node->parent = NULL;
        cur_node->position = start_pt;
        cur_node->index = start_pt;
        cur_node->g_score = 0.0;
        cur_node->escape_step = 0;

        //Eigen::Vector2d end_state(6);
        Eigen::Vector2i end_index;
        end_index = end_pt;

        cur_node->f_score = lambda_heu_ * getEuclHeu(start_pt, end_pt);
        cur_node->node_state = IN_OPEN_SET;

        const bool start_in_collision = esdf_environment_->checkCollision(start_pt);
        const bool goal_in_collision = esdf_environment_->checkCollision(end_pt);
        last_search_debug_.start_in_collision = start_in_collision;
        last_search_debug_.goal_in_collision = goal_in_collision;
        last_search_debug_.start_dist_cells = esdf_environment_->getDist(start_pt);
        last_search_debug_.goal_dist_cells = esdf_environment_->getDist(end_pt);
        const bool use_start_escape =
            start_in_collision && enable_start_escape_mode_ && start_escape_steps_ > 0;
        last_search_debug_.escape_mode_used = use_start_escape;
        last_search_debug_.escape_budget_steps = start_escape_steps_;
        if (use_start_escape) {
            RCLCPP_WARN(
                rclcpp::get_logger("Astar"),
                "Start cell is occupied. Enabling escape mode for first %d expansion steps.",
                start_escape_steps_);
        }

        open_set_.push(cur_node);
        use_node_num_ += 1;
        last_search_debug_.discovered_nodes = use_node_num_;
        last_search_debug_.max_open_set_size = static_cast<int>(open_set_.size());

        expanded_nodes_.insert(cur_node->index, cur_node);

        // NodePtr neighbor = NULL;
        NodePtr terminate_node = NULL;

        while (!open_set_.empty()) 
        {
            cur_node = open_set_.top();

            bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                abs(cur_node->index(1) - end_index(1)) <= 1;

            if (reach_end) {

                terminate_node = cur_node;
                retrievePath(terminate_node);
                has_path_ = true;
                last_search_debug_.path_found = true;
                last_search_debug_.exit_reason = SEARCH_EXIT_REACH_END;
                RCLCPP_INFO(rclcpp::get_logger("Astar"),"after %d iters , reached end",iter_num_);
                return REACH_END;
            }

            open_set_.pop();
            cur_node->node_state = IN_CLOSE_SET;
            iter_num_ += 1;
            last_search_debug_.expanded_nodes = iter_num_;


            Eigen::Vector2i cur_pos = cur_node->position;
            const double cur_dist = esdf_environment_->getDist(cur_pos);
            const bool cur_collision = esdf_environment_->checkCollision(cur_pos);
            const bool cur_below_clearance =
                min_clearance_cells_ > 0.0 && cur_dist < min_clearance_cells_;
            Eigen::Vector2i pro_pos;
            // double pro_t;    未使用变量

            Eigen::Vector2i d_pos;

            // 20-neighborhood kept here for reference.
            // for (int dx = -2; dx <= 2; dx += 1){
            //     for (int dy = -2; dy <= 2; dy += 1){
            //
            //         if(dx*dx + dy*dy >= 8){continue;}
            //         if(dx == 0 && dy == 0){continue;}
            //
            //         d_pos << dx, dy;
            //
            //         pro_pos = cur_pos + d_pos;
            //
            //         if (pro_pos(0) < 0 || pro_pos(0) >= map_size_2d_(0) || pro_pos(1) < 0 ||
            //             pro_pos(1) >= map_size_2d_(1) ) {
            //         std::cout << "outside map" << std::endl;
            //         continue;
            //         }
            //
            //         Eigen::Vector2i pro_id = pro_pos;
            //
            //         NodePtr pro_node = expanded_nodes_.find(pro_id);
            //
            //         if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
            //         continue;
            //         }
            //
            //         bool Collsion_flg= esdf_environment_->checkCollision(pro_pos);
            //         if (Collsion_flg) {
            //         continue;
            //         }
            //
            //         double tmp_g_score, tmp_f_score;
            //         double dist = esdf_environment_->getDist(pro_pos);
            //         tmp_g_score = d_pos.norm() + getDistCost(dist) + cur_node->g_score;
            //         tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt);
            //
            //         if (pro_node == NULL) {
            //             pro_node = path_node_pool_[use_node_num_];
            //             pro_node->index = pro_id;
            //             pro_node->position = pro_pos;
            //             pro_node->f_score = tmp_f_score;
            //             pro_node->g_score = tmp_g_score;
            //             pro_node->parent = cur_node;
            //             pro_node->node_state = IN_OPEN_SET;
            //             open_set_.push(pro_node);
            //             expanded_nodes_.insert(pro_id, pro_node);
            //
            //             use_node_num_ += 1;
            //             if (use_node_num_ == allocate_num_) {
            //                 std::cout << "run out of memory." << std::endl;
            //                 return NO_PATH;
            //             }
            //             else if (pro_node->node_state == IN_OPEN_SET) {
            //                 if (tmp_g_score < pro_node->g_score) {
            //                     pro_node->position = pro_pos;
            //                     pro_node->f_score = tmp_f_score;
            //                     pro_node->g_score = tmp_g_score;
            //                     pro_node->parent = cur_node;
            //                 }
            //                 else {
            //                 }
            //             }
            //         }
            //     }
            // }

            for (int dx = -1; dx <= 1; dx += 1){
                for (int dy = -1; dy <= 1; dy += 1){

                    if(dx == 0 && dy == 0){continue;}

                    last_search_debug_.neighbor_candidates += 1;
                    d_pos << dx, dy;

                    pro_pos = cur_pos + d_pos;

                    if (pro_pos(0) < 0 || pro_pos(0) >= map_size_2d_(0) || pro_pos(1) < 0 ||
                        pro_pos(1) >= map_size_2d_(1) ) {
                        last_search_debug_.reject_out_of_map += 1;
                        continue;
                    }

                    Eigen::Vector2i pro_id = pro_pos;

                    NodePtr pro_node = expanded_nodes_.find(pro_id);

                    if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
                        last_search_debug_.reject_in_close_set += 1;
                        continue;
                    }

                    const int next_escape_step = cur_node->escape_step + 1;
                    double dist = esdf_environment_->getDist(pro_pos);
                    const bool within_escape_budget =
                        use_start_escape && (next_escape_step <= start_escape_steps_);
                    const bool bypass_collision_and_clearance =
                        within_escape_budget && (cur_collision || cur_below_clearance);
                    bool collision_flg = esdf_environment_->checkCollision(pro_pos);
                    const bool below_clearance =
                        min_clearance_cells_ > 0.0 && dist < min_clearance_cells_;
                    if (bypass_collision_and_clearance && (collision_flg || below_clearance)) {
                        last_search_debug_.bypass_escape_constraints += 1;
                    }
                    if (!bypass_collision_and_clearance && collision_flg) {
                        last_search_debug_.reject_collision += 1;
                        continue;
                    }
                    if (!bypass_collision_and_clearance && below_clearance) {
                        last_search_debug_.reject_clearance += 1;
                        continue;
                    }

                    // bool UpstairFlg = esdf_environment_->checkUpStairs(cur_pos,pro_pos);
                    // if(UpstairFlg)
                    // {
                    //     continue;
                    // }

                    double tmp_g_score, tmp_f_score;
                    tmp_g_score = d_pos.norm() + getDistCost(dist) + cur_node->g_score;
                    tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt);

                    if (pro_node == NULL) {
                        pro_node = path_node_pool_[use_node_num_];
                        pro_node->index = pro_id;
                        pro_node->position = pro_pos;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->parent = cur_node;
                        pro_node->escape_step = next_escape_step;
                        pro_node->node_state = IN_OPEN_SET;
                        open_set_.push(pro_node);
                        if (static_cast<int>(open_set_.size()) > last_search_debug_.max_open_set_size) {
                            last_search_debug_.max_open_set_size = static_cast<int>(open_set_.size());
                        }
                        expanded_nodes_.insert(pro_id, pro_node);

                        use_node_num_ += 1;
                        last_search_debug_.discovered_nodes = use_node_num_;
                        if (use_node_num_ == allocate_num_) {
                            last_search_debug_.exit_reason = SEARCH_EXIT_NODE_POOL_EXHAUSTED;
                            RCLCPP_ERROR(
                                rclcpp::get_logger("Astar"),
                                "A* NO_PATH because node pool exhausted (%d). %s",
                                allocate_num_,
                                formatLastSearchDebugInfo().c_str());
                            return NO_PATH;
                        }
                    }
                    else if (pro_node->node_state == IN_OPEN_SET) {
                        if (tmp_g_score < pro_node->g_score) {
                            // pro_node->index = pro_id;
                            pro_node->position = pro_pos;
                            pro_node->f_score = tmp_f_score;
                            pro_node->g_score = tmp_g_score;
                            pro_node->parent = cur_node;
                            pro_node->escape_step = next_escape_step;
                            last_search_debug_.relaxed_nodes += 1;
                        }
                    }
                }
            }
        }
        last_search_debug_.exit_reason = SEARCH_EXIT_OPEN_SET_EMPTY;
        RCLCPP_WARN(
            rclcpp::get_logger("Astar"),
            "A* NO_PATH because open set is exhausted. %s",
            formatLastSearchDebugInfo().c_str());
        return NO_PATH;
    }

    void Astar::setParam(
        double obstacle_weight,
        double dynamic_penalty_weight,
        double min_clearance_cells,
        bool enable_start_escape_mode,
        int start_escape_steps) {
        lambda_heu_ = 2.0;
        allocate_num_ = 500000;
        obstacle_weight_ = obstacle_weight;
        dynamic_penalty_weight_ = dynamic_penalty_weight;
        min_clearance_cells_ = std::max(0.0, min_clearance_cells);
        enable_start_escape_mode_ = enable_start_escape_mode;
        start_escape_steps_ = std::max(0, start_escape_steps);
        // nh.param("astar/lambda_heu", lambda_heu_, 2.0);
        // nh.param("astar/allocate_num", allocate_num_, 500000);
        RCLCPP_INFO(rclcpp::get_logger("Astar"),"lambda_heu:%f",lambda_heu_);
        RCLCPP_INFO(rclcpp::get_logger("Astar"),"allocate_num_:%d",allocate_num_);
        RCLCPP_INFO(rclcpp::get_logger("Astar"),"obstacle_weight_:%f",obstacle_weight_);
        RCLCPP_INFO(rclcpp::get_logger("Astar"),"dynamic_penalty_weight_:%f",dynamic_penalty_weight_);
        RCLCPP_INFO(rclcpp::get_logger("Astar"),"min_clearance_cells_:%f",min_clearance_cells_);
        RCLCPP_INFO(rclcpp::get_logger("Astar"),"enable_start_escape_mode_:%s",enable_start_escape_mode_ ? "true" : "false");
        RCLCPP_INFO(rclcpp::get_logger("Astar"),"start_escape_steps_:%d",start_escape_steps_);
    }

    void Astar::retrievePath(NodePtr end_node) {
        NodePtr cur_node = end_node;
        path_nodes_.push_back(cur_node);

        while (cur_node->parent != NULL) {
            cur_node = cur_node->parent;
            path_nodes_.push_back(cur_node);
        }

        reverse(path_nodes_.begin(), path_nodes_.end());
    }

    std::vector<Eigen::Vector2i> Astar::getPath() {
        std::vector<Eigen::Vector2i> path;
        for (size_t i = 0; i < path_nodes_.size(); ++i) {
            path.push_back(path_nodes_[i]->position);
        }
        return path;
    }

    double Astar::getEuclHeu(Eigen::Vector2i x1, Eigen::Vector2i x2) {
        return (x2 - x1).norm();
    }

    double Astar::getDistCost(double dist_)
    {
        return obstacle_weight_ / (dist_ + 0.1f);
    }

    void Astar::init() {
        /* ---------- map params ---------- */
        esdf_environment_->getMapRegion(map_size_2d_);
        // cout << "origin_: " << origin_.transpose() << endl;
        std::cout << "map size: " << map_size_2d_.transpose() << std::endl;

        /* ---------- pre-allocated node ---------- */
        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++) {
            path_node_pool_[i] = new Node;
        }

        use_node_num_ = 0;
        iter_num_ = 0;
    }

    void Astar::setEnvironment(const ESDF_enviroment::Ptr& env) {
        this->esdf_environment_ = env;
    }

    void Astar::reset() {
        expanded_nodes_.clear();
        path_nodes_.clear();

        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
        open_set_.swap(empty_queue);

        for (int i = 0; i < use_node_num_; i++) {
            NodePtr node = path_node_pool_[i];
            node->parent = NULL;
            node->escape_step = 0;
            node->node_state = NOT_EXPAND;
        }

        use_node_num_ = 0;
        iter_num_ = 0;
    }

    std::vector<NodePtr> Astar::getVisitedNodes() {
        std::vector<NodePtr> visited;
        if (use_node_num_ <= 0) {
            return visited;
        }
        visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_);
        return visited;
    }

    Astar::Astar()
    {
        lambda_heu_ = 2.0;
        allocate_num_ = 500000;
        obstacle_weight_ = 10.0;
        dynamic_penalty_weight_ = 0.0;
        min_clearance_cells_ = 0.0;
        enable_start_escape_mode_ = true;
        start_escape_steps_ = 16;
    }

    Astar::~Astar()
    {
        for (NodePtr node : path_node_pool_) {
            delete node;
        }
    }

// Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt) {
//   Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

//   // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
//   // origin_(1)) * inv_resolution_),
//   //     floor((pt(2) - origin_(2)) * inv_resolution_);

//   return idx;
// }
}
