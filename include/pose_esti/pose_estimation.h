/**************************************************************************
 * Copyright 2023 Youibot Robotics Co., Ltd. All Rights Reserved.
 * Contact: wenzhaochen (robotmanciu@gmail.com)
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 *************************************************************************/

#ifndef _COPPER_TAG_POSE_ESTIMATION_H_
#define _COPPER_TAG_POSE_ESTIMATION_H_

#include <string>

#include "ippe.h"
#include "levmarq.h"

namespace CopperTag
{

struct IntrinsicMatrix
{
    float fx;
    float fy;
    float cx;
    float cy;
}; // struct IntrinsicMatrix

struct Distortion
{
    float k1;
    float k2;
    float p1;
    float p2;
    float k3;
}; // struct Distortion

struct CameraParameters
{
    int width;
    int height;
    IntrinsicMatrix intrinsic_matrix;
    Distortion distortion;
}; // struct CameraParameters

struct PoseResult
{
    bool valid;
    float repj_err;
    float x;
    float y;
    float z;
    float rx;
    float ry;
    float rz;
}; // struct PoseResult

class PoseEstimation
{
public:
    /**
     * @brief construct function
    */
    PoseEstimation();

    /**
     * @brief deconstruct function
    */
    ~PoseEstimation();

    /**
     * @brief load camera intrinsic matrix and distortion value for PnP problem
     * 
     * @param input_params input camera parameters
    */
    void load_camera_parameters(CameraParameters& input_params);

    /**
     * @brief load camera parameters for PnP problem from .yaml fime
     * 
     * @param file_name camera parameters yaml file name
    */
    void load_camera_parameters(std::string filename);

    /**
     * @brief calculate the tag pose according to the camera coordinate (using IPPE method)
     * 
     * @param need_polish whether the pose result from IPPE method will be polished by LM method
     * @param objPoints four corner world coordinate of the tag
     * @param imgPoints four corner pixel coordinate of the tag
     * 
     * @return PoseResult tag pose result
    */
    PoseResult calculate_pose(bool need_polish,
                              const std::vector<cv::Point3f>& objPoints,
                              const std::vector<cv::Point2f>& imgPoints);
private:
    // whether the camera parameters was initialized
    bool isInit_;

    // current camera parameters
    CameraParameters curCameraParams_;

    // current camera intrinsic matrix
    cv::Mat cameraMatrix_;

    // current camera distortion matrix
    cv::Mat distCoeffs_;

    // pointer of IPPE pose solver
    std::unique_ptr<IPPE::PoseSolver> ippeSolver_;

    /**
     * @brief template function,The Levenberg-Marquardt iterative method is 
     * implemented to optimize the obtained initial pose values
     * 
     * @tparam T optimized numeric data types
     * @param p3d The 3D position of corner points in the world coordinate system
     * @param p2d The 2D position of corner points in the image coordinate system
     * @param cam_matrix camera intrinscic matrix
     * @param dist camera distortion coefficient
     * @param r_io rotation vector output
     * @param t_io translation vector output
     * @return double error value at the end of the iteration
     */
    template <typename T>
    double solvePNP_LM_IMPL(const std::vector<cv::Point3f>& p3d, const std::vector<cv::Point2f>& p2d,
                            const cv::InputArray& cam_matrix, const cv::InputArray& dist, cv::Mat& r_io, cv::Mat& t_io) {
        
        assert(r_io.type() == CV_32F);
        assert(t_io.type() == CV_32F);
        assert(t_io.total() == r_io.total());
        assert(t_io.total() == 3);

        auto toSol = [](const cv::Mat& r, const cv::Mat& t) {                
            typename LevMarq<T>::eVector sol(6);
            for (int i = 0; i < 3; i++)
            {
                sol(i) = r.ptr<float>(0)[i];
                sol(i + 3) = t.ptr<float>(0)[i];
            }
            return sol;
        };
        auto fromSol = [](const typename LevMarq<T>::eVector& sol, cv::Mat& r, cv::Mat& t) {
            r.create(1, 3, CV_32F);
            t.create(1, 3, CV_32F);
            for (int i = 0; i < 3; i++)
            {
                r.ptr<float>(0)[i] = sol(i);
                t.ptr<float>(0)[i] = sol(i + 3);
            }
        };

        cv::Mat Jacb;
        auto err_f = [&](const typename LevMarq<T>::eVector& sol, typename LevMarq<T>::eVector& err) {
            std::vector<cv::Point2f> p2d_rej;
            cv::Mat r, t;
            fromSol(sol, r, t);
            cv::projectPoints(p3d, r, t, cam_matrix, dist, p2d_rej, Jacb);
            err.resize(p3d.size() * 2);
            int err_idx = 0;
            for (size_t i = 0; i < p3d.size(); i++)
            {
                cv::Point2f  errP=p2d_rej[i] -p2d[i];

                double SqErr=(errP.x*errP.x+ errP.y*errP.y);

                float robuse_weight= getHuberMonoWeight(SqErr,1);
                err(err_idx++) = robuse_weight* errP.x;//p2d_rej[i].x - p2d[i].x;
                err(err_idx++) = robuse_weight* errP.y;//p2d_rej[i].y - p2d[i].y;
            }
        };
        auto jac_f = [&](const typename LevMarq<T>::eVector& sol, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& J) {
            (void)(sol);
            J.resize(p3d.size() * 2, 6);
            for (size_t i = 0; i < p3d.size() * 2; i++)
            {
                double* jacb = Jacb.ptr<double>(i);
                for (int j = 0; j < 6; j++)
                    J(i, j) = jacb[j];
            }
        };
        
        LevMarq<T> solver;
        solver.setParams(100, 0.01, 0.001);
        //solver.verbose()=true;
        typename LevMarq<T>::eVector sol = toSol(r_io, t_io);     
        auto err = solver.solve(sol, err_f, jac_f);
        fromSol(sol, r_io, t_io);  
        return err;
    }

    inline double huberLost(double e, double delta) {
        double dsqr = delta * delta;
        if (e <= dsqr) {
            return e;
        } else {
            double sqrte = sqrt(e);
            return 2 * sqrte * delta - dsqr;
        }
    }

    inline double huberMono(double e) {
        if (e <= 5.991) {
            return e;
        } else {
            double sqrte = sqrt(e);
            return sqrte * 4.895303872 - 5.991;
        }
    }

    inline double getHuberMonoWeight(double sq_err, double info) {
        return sqrt(huberMono(info *sq_err) / sq_err);
    }
}; // class PoseEstimation

} // namespace CopperTag

#endif // _COPPER_TAG_POSE_ESTIMATION_H_