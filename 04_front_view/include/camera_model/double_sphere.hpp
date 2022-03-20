
#ifndef __CAMERA_MODEL_DOUBLE_SPHERE_HPP__
#define __CAMERA_MODEL_DOUBLE_SPHERE_HPP__

#include "camera_model/camera_model_base.hpp"

namespace mvs
{

class DoubleSphere : public CameraModel {

public:
    DoubleSphere(const std::string& name, float xi, float alpha, float fx, float fy, float cx, float cy, const Shape_t& shape);
    virtual ~DoubleSphere();

    PointMat2 project_3d_2_image_plane( const PointMat3& points, PointMask* p_mask=nullptr, bool dimensionless=false) const;
    std::pair< PointMat1, PointMat1 >
        project_3d_2_image_plane_separated(const PointMat3& points, PointMask* p_mask=nullptr, bool dimensionless=false) const;

    std::string string() const;

    friend std::ostream& operator << ( std::ostream& os, const DoubleSphere& ds );

protected:
    PointMat1 compute_t_and_mask( const PointMat3& points, PointMask* p_mask ) const;

public:
    float xi;
    float alpha;

protected:
    float w1;
    float w2;

};

}

#endif
