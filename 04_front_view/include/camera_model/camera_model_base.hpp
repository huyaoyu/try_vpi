#ifndef __CAMERA_MODEL_CAMERA_MODEL_BASE_HPP__
#define __CAMERA_MODEL_CAMERA_MODEL_BASE_HPP__

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <opencv2/imgproc.hpp>

namespace mvs
{

typedef Eigen::Matrix<float, 1, Eigen::Dynamic> PointMat1;
typedef Eigen::Matrix<float, 2, Eigen::Dynamic> PointMat2;
typedef Eigen::Matrix<float, 3, Eigen::Dynamic> PointMat3;
typedef Eigen::Matrix<std::uint8_t, 1, Eigen::Dynamic> PointMask;

typedef Eigen::Matrix<float, 4, 4> TransformMat;

TransformMat inverse_transform( const TransformMat& T );

struct Shape_t {    
    Shape_t(int H, int W)
    : h(H), w(W), mat{w, h}
    {}

    Shape_t(const Shape_t& other)
    : h(other.h), w(other.w), mat{w, h}
    {}

    const Eigen::Matrix<int, 2, 1>& as_mat() const {
        return mat;
    }

    int size() const { return h * w; }

    std::string string() const;
    friend std::ostream& operator << ( std::ostream& os, const Shape_t& shape );
    
    int h;
    int w;
    Eigen::Matrix<int, 2, 1> mat; // The matrix representation.
};

class CameraModel {

public:
    enum InterpolationType {
        LINEAR,
        NEAREST
    };

    CameraModel(const std::string& name, float fx, float fy, float cx, float cy, const Shape_t& shape);
    virtual ~CameraModel();

    virtual PointMat2 project_3d_2_image_plane( const PointMat3& points, PointMask* p_mask=nullptr, bool dimensionless=false) const = 0;
    virtual std::pair< PointMat1, PointMat1 >
        project_3d_2_image_plane_separated(const PointMat3& points, PointMask* p_mask=nullptr, bool dimensionless=false) const = 0;

    std::string string() const;

public:
    std::string name;
    float fx;
    float fy;
    float cx;
    float cy;
    Shape_t shape;

    TransformMat extrinsics;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}

#endif // __CAMERA_MODEL_CAMERA_MODEL_BASE_HPP__