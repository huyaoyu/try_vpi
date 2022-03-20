
#ifndef __KALIBR_PARSER_HPP__
#define __KALIBR_PARSER_HPP__

#include <iostream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "camera_model/double_sphere.hpp"

namespace mvs
{

class KalibrParser {
public:
    KalibrParser() = default;
    ~KalibrParser() = default;

    DoubleSphere parser_camera_node(
        YAML::Node&& node,
        const TransformMat& previous_chain_pose = TransformMat::Identity()
    ) const ;

public:
    static const std::string KALIBR_INTRINSICS;
    static const std::string KALIBR_EXTRINSICS;
    static const std::string KALIBR_RESOLUTION;
    static const std::string KALIBR_ROSTOPIC;
};

} // namespace mvs

#endif // __KALIBR_PARSER_HPP__
