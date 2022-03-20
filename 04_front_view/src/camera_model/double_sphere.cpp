
#include <cmath>
// #include <tuple>

#include "camera_model/double_sphere.hpp"

using namespace mvs;

DoubleSphere::DoubleSphere(const std::string& name, float xi, float alpha, float fx, float fy, float cx, float cy, const Shape_t& shape)
: CameraModel(name, fx, fy, cx, cy, shape), xi(xi), alpha(alpha)
{
    if ( alpha <= 0.5 ) {
        w1 = alpha / ( 1 - alpha );
    } else {
        w1 = ( 1 - alpha ) / alpha;
    }

    w2 = ( w1 + xi ) / std::sqrt( 2 * w1 * xi + xi * xi + 1 );
}

DoubleSphere::~DoubleSphere()
{}

PointMat1 DoubleSphere::compute_t_and_mask( const PointMat3& points, PointMask* p_mask ) const {
    // Make a copy of points.
    PointMat3 points_d2 = points;

    // Cmopute d1.
    const auto d1 = points.colwise().norm();

    // Compute d2.
    points_d2.row(2) += d1 * xi;
    const auto d2 = points_d2.colwise().norm();

    // Coordinate base.
    auto t = alpha * d2 + ( 1 - alpha ) * ( xi * d1 + points.row(2) );

    // Mask.
    if ( p_mask ) {
        *p_mask = ( points.row(2).array() > -w2 * d1.array() ).matrix().cast<std::uint8_t>();
    }

    return t;
}

PointMat2 DoubleSphere::project_3d_2_image_plane( 
    const PointMat3& points, PointMask* p_mask, bool dimensionless) const {
    // Compute t and mask.
    const auto t = compute_t_and_mask( points, p_mask );

    // Dimensionless.
    PointMat2 uxuy;
    uxuy.resize(2, points.cols());
    if ( dimensionless ) {
        // Directly expressed as the math equations.
        // uxuy.row(0) = ( ( fx / t.array() * points.row(0).array() + cx ) / ( shape.w - 1 ) * 2 - 1 ).matrix();
        // uxuy.row(1) = ( ( fy / t.array() * points.row(1).array() + cy ) / ( shape.h - 1 ) * 2 - 1 ).matrix();
        
        // Better performance with less element-wise operations.
        uxuy.row(0) = ( 
            ( fx / ( shape.w - 1 ) * 2 ) / t.array() * points.row(0).array() 
            + ( cx / ( shape.w - 1 ) * 2 - 1 ) 
        ).matrix();
        uxuy.row(1) = (
            ( fy / ( shape.h - 1 ) * 2 ) / t.array() * points.row(1).array() 
            + ( cy / ( shape.h - 1 ) * 2 - 1 ) 
        ).matrix();
    } else {
        // ux, uy with dimension.
        uxuy.row(0) = ( fx / t.array() * points.row(0).array() + cx ).matrix();
        uxuy.row(1) = ( fy / t.array() * points.row(1).array() + cy ).matrix();
    }

    return uxuy;
}

std::pair< PointMat1, PointMat1 >
DoubleSphere::project_3d_2_image_plane_separated(
    const PointMat3& points, PointMask* p_mask, bool dimensionless) const {
    // Compute t and mask.
    const auto t = compute_t_and_mask( points, p_mask );

    // Dimensionless.
    PointMat1 ux, uy;

    if ( dimensionless ) {
        // Directly expressed as the math equations.
        // uxuy.row(0) = ( ( fx / t.array() * points.row(0).array() + cx ) / ( shape.w - 1 ) * 2 - 1 ).matrix();
        // uxuy.row(1) = ( ( fy / t.array() * points.row(1).array() + cy ) / ( shape.h - 1 ) * 2 - 1 ).matrix();
        
        // Better performance with less element-wise operations.
        ux = ( 
            ( fx / ( shape.w - 1 ) * 2 ) / t.array() * points.row(0).array() 
            + ( cx / ( shape.w - 1 ) * 2 - 1 ) 
        ).matrix();
        uy = (
            ( fy / ( shape.h - 1 ) * 2 ) / t.array() * points.row(1).array() 
            + ( cy / ( shape.h - 1 ) * 2 - 1 ) 
        ).matrix();
    } else {
        // ux, uy with dimension.
        ux = ( fx / t.array() * points.row(0).array() + cx ).matrix();
        uy = ( fy / t.array() * points.row(1).array() + cy ).matrix();
    }

    return { ux, uy };
}

std::string DoubleSphere::string() const {
    std::stringstream ss;
    ss << "xi = " << xi << "\n"
       << "alpha = " << alpha << "\n"
       << CameraModel::string() << "\n";
    return ss.str();
}

namespace mvs
{

std::ostream& operator << ( std::ostream& os, const DoubleSphere& ds ) {
    os << ds.string();
    return os;
}

} // namespace mvs