
#include "camera_model/camera_model_base.hpp"

using namespace mvs;

CameraModel::CameraModel(float fx, float fy, float cx, float cy, const Shape_t& shape)
: fx(fx), fy(fy), cx(cx), cy(cy), shape(shape) 
{}

CameraModel::~CameraModel()
{}
