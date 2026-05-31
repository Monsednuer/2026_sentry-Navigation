#include "smoother.h"
#include "cubic_spline/cubic_spline_ros.h"

namespace navi_planner {

    smoother::VV2D smoother::smooth(smoother::VV2D path_input, 
    double lambda_1 , double lambda_2)
    {
        smoother::VV2D newPath = path_input;
        smoother::VV2D Path_d;

        // double lambda1 = 0.1;
        // double lambda2 = 3;
        double Obs_thre = 10;

        for(int j = 0 ; j < newPath.size();j+=1)
        {
            Eigen::Vector2d Node_ = Eigen::Vector2d(newPath[j][0],newPath[j][1]);
            Path_d.push_back(Node_);
        }
        Path_d.push_back(Eigen::Vector2d(newPath.back()[0],newPath.back()[1]));
        std::vector<Eigen::Vector2d> KeyPoints = pathResampling(Path_d);
        Path_d = KeyPoints;

        for(int i = 0; i < 10 ; i++)
        {
            std::vector<Eigen::Vector2d> Path_d_temp = Path_d;
            // for(int j = 1 ; j < Path_d.size()-1;j+=1)
            for(int j = Path_d.size()-2 ; j > 0;j-=1)
            {
                
                double& currunt_x = Path_d_temp[j][0] ;
                double& currunt_y = Path_d_temp[j][1] ;
                
                double& next_x = Path_d_temp[j-1][0] ;
                double& next_y = Path_d_temp[j-1][1] ;
                
                double& last_x = Path_d_temp[j+1][0] ;
                double& last_y = Path_d_temp[j+1][1] ;
                
                Eigen::Vector2d &currunt_node = Path_d_temp[j];
                Eigen::Vector2d &next_node = Path_d_temp[j-1];
                Eigen::Vector2d &last_node = Path_d_temp[j+1];
                
                // double map_x = (currunt_x - x_offset);
                // double map_y = (currunt_y - y_offset);
                Eigen::Vector2i index_ = esdf_environment_->Pos2index(currunt_node);
                if(1)
                {
                    // if current cell is occupied, apply smoothing regularization only
                    if(esdf_environment_->bin_map[index_[0]][index_[1]])
                    {
                        currunt_x -= lambda_1*(2*currunt_x - last_x - next_x);
                        currunt_y -= lambda_1*(2*currunt_y - last_y - next_y);
                        continue;
                    }
                    
                    Eigen::Vector2i neighborObsIndex = esdf_environment_->getNearestObstacleIndex(index_);
                    Eigen::Vector2d neighborObs = Eigen::Vector2d::Zero();
                    if(neighborObsIndex[0] >= 0) neighborObs = esdf_environment_->Index2pos(neighborObsIndex);
                    Eigen::Vector2d delta_vec = neighborObs-currunt_node;

                    int cnt_1 = 0;
                    for(int di = -2; di < 3; di++)
                    {
                        for(int dj = -2; dj < 3 ; dj++)
                        {
                            Eigen::Vector2i check_idx = index_ + Eigen::Vector2i(di,dj);
                            if(check_idx[0] < 0 || check_idx[1] < 0 || check_idx[0] >= esdf_environment_->Size[0] || check_idx[1] >= esdf_environment_->Size[1]) continue;
                            if(esdf_environment_->checkCollision(check_idx))
                                continue;
                            cnt_1 ++;
                            Eigen::Vector2i nIdx = esdf_environment_->getNearestObstacleIndex(check_idx);
                            if(nIdx[0] < 0) continue;
                            Eigen::Vector2d nObs = esdf_environment_->Index2pos(nIdx);
                            delta_vec += (nObs-currunt_node);
                        }
                    }
                    if(cnt_1 > 0) delta_vec /= cnt_1;
                    // delta_y /= cnt_1;
                    double R_2 = (delta_vec).norm();
                    double R_3 = pow(R_2,1.5);
                    
                    double temp_x = currunt_x - lambda_2 * ((delta_vec[0]) / R_2);
                    double temp_y = currunt_y - lambda_2 * ((delta_vec[1]) / R_2);
                    Eigen::Vector2d temp_pos(temp_x,temp_y);
                    Eigen::Vector2i temp_index = esdf_environment_->Pos2index(temp_pos);
                    if(esdf_environment_->checkCollision(temp_index))
                    {
                        currunt_x -= lambda_1*(2*currunt_x - last_x - next_x);
                        currunt_y -= lambda_1*(2*currunt_y - last_y - next_y);
                        continue;
                    }
                        
                    currunt_x -= lambda_2 * ((delta_vec[0]) / R_2);
                    currunt_y -= lambda_2 * ((delta_vec[1]) / R_2);

                    currunt_x -=  lambda_1* ( 2* currunt_x -  last_x -  next_x );
                    currunt_y -= lambda_1 *(2* currunt_y -  last_y -  next_y );
                    }
            }
            Path_d = Path_d_temp;
        }
        std::vector<Eigen::Vector3d> CubicPath;
        std::vector<Eigen::Vector2d> CubicPath_2d;
        GenTraj(Path_d,CubicPath,0.10f);
        for(auto &p: CubicPath)
        {
            CubicPath_2d.push_back(Eigen::Vector2d(p[0],p[1]));
        }

        Path_d = CubicPath_2d;
        return Path_d;
    //path_2d_smoothed = Path_d;
        
    }
    smoother::VV2D smoother::pathResampling(smoother::VV2D path_input)
    {
        int j = 1;
        std::vector<Eigen::Vector2d> KeyPoints;
        KeyPoints.push_back(path_input[0]);
        for(int i = 2 ; i < path_input.size()-1; i++)
        {
            if(isCollision(path_input[j],path_input[i]) == false)
            {
                // int map_x = Path_d[i][0] - x_offset;
                // int map_y = Path_d[i][1] - y_offset;
                Eigen::Vector2i index_ = esdf_environment_->Pos2index(path_input[i]);
                if(((path_input[i] - path_input[j]).norm()*2 < 
                esdf_environment_->getDist(index_) * 0.05f))
                    continue;
                else
                {
                    KeyPoints.push_back(path_input[i]);
                    j = i;
                }
            }
            else
            {
                KeyPoints.push_back(path_input[i]);
                j = i;
            }
        }
        KeyPoints.push_back(path_input[path_input.size()-1]);
        return KeyPoints;
    }
    bool smoother::isCollision(Eigen::Vector2d node1, Eigen::Vector2d node2)
    {
        double checknum = (node1 - node2).norm()*20.0f;
        for(int i = 1 ; i < checknum; i++)
        {
            Eigen::Vector2d midPoint = (i/checknum)*node1 + (1-i/checknum)*node2;
            Eigen::Vector2i index_ = esdf_environment_->Pos2index(midPoint);
            if(esdf_environment_->checkCollision(index_))
            {return true;}
        }
        return false;
    }
    void smoother::smoother_setEnvironment(ESDF_enviroment::Ptr esdf_ptr)
    {
        esdf_environment_ = esdf_ptr;
    }
    
}