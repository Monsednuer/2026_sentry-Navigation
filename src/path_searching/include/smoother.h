#ifndef SMOOTHER_H_
#define SMOOTHER_H_

#include <Eigen/Core>
#include <vector>
#include "esdf_map.h"

namespace navi_planner {
class smoother{
public:
    typedef std::vector<Eigen::Vector2d> VV2D;
    smoother(){;}
    ~smoother(){;}
    VV2D smooth(VV2D path_input, 
    double lambda_1 = 0.1f, double lambda_2 = 0.1f);
    void smoother_setEnvironment(ESDF_enviroment::Ptr esdf_ptr);
    VV2D pathResampling(VV2D path_input);
    bool isCollision(Eigen::Vector2d node1, Eigen::Vector2d node2);
private:
    ESDF_enviroment::Ptr esdf_environment_;
    
};
}

#endif