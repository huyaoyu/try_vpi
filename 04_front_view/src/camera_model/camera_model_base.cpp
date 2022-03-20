
#include <sstream>

#include "camera_model/camera_model_base.hpp"

using namespace mvs;

namespace mvs
{

TransformMat inverse_transform( const TransformMat& T ) {
    TransformMat inversed = TransformMat::Identity();

    const auto RT = T.block(0, 0, 3, 3).transpose();

    inversed.block(0, 0, 3, 3) = RT;
    inversed.block(0, 3, 3, 1) = -RT * T.block(0, 3, 3, 1);

    return inversed;
}

} // namespace mvs

std::string Shape_t::string() const {
    std::stringstream ss;
    ss << "(h, w) " << h << ", " << w;
    return ss.str();
}

namespace mvs
{

std::ostream& operator << ( std::ostream& os, const Shape_t& shape ) {
    os << shape.string();
    return os;
}

} // namespace mvs

CameraModel::CameraModel(const std::string& name, float fx, float fy, float cx, float cy, const Shape_t& shape)
: name(name), fx(fx), fy(fy), cx(cx), cy(cy), shape(shape) 
{
    extrinsics = TransformMat::Identity();
}

CameraModel::~CameraModel()
{}

std::string CameraModel::string() const {
    std::stringstream ss;

    ss << "name = " << name << "\n"
       << "fx = " << fx << "\n"
       << "fy = " << fy << "\n"
       << "cx = " << cx << "\n"
       << "cy = " << cy << "\n"
       << "shape = " << shape << "\n"
       << "extrinsics = \n" << extrinsics;

    return ss.str();
}