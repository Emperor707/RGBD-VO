/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/camera.h"
#include <myslam/config.h>

namespace myslam
{
    
Camera::Camera()
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}

Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3& T_c_w )
{
    return T_c_w*p_w;
}

Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3& T_c_w )
{
    return T_c_w.inverse() *p_c;
}

Vector2d Camera::camera2pixel ( const Vector3d& p_c )
{
    return Vector2d (
               fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
               fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
           );
}

Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )
{
    return Vector3d (
               ( p_p ( 0,0 )-cx_ ) *depth/fx_,
               ( p_p ( 1,0 )-cy_ ) *depth/fy_,
               depth
           );
}

Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3& T_c_w )
{
    return camera2pixel ( world2camera(p_w, T_c_w) );
}

Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth )
{
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}


//将cv::Mat类型的四元数转换成为std::vector型
std::vector<float> Camera::toQuaternion(const Eigen::Matrix3d &M)
{
	//首先将cv::Mat格式的旋转矩阵转换成为Eigen::Matrix格式
    // Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
	//然后利用这个矩阵转换成为四元数格式
	Eigen::Quaterniond q(M);
	//最后声明一个这样的向量
    std::vector<float> v(4);
	//将四元数的四个元素分别保存到这个vector中
    ///@note 注意,使用std::vector存储的四元数的顺序是x y z w
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();
	//返回转换结果
    return v;
}


}
