
#include "calib/kalibr_parser.hpp"

namespace mvs
{

const std::string KalibrParser::KALIBR_INTRINSICS = "intrinsics";
const std::string KalibrParser::KALIBR_EXTRINSICS = "T_cn_cnm1";
const std::string KalibrParser::KALIBR_RESOLUTION = "resolution";
const std::string KalibrParser::KALIBR_ROSTOPIC   = "rostopic";

}

using namespace mvs;

DoubleSphere KalibrParser::parser_camera_node(
    YAML::Node&& node,
    const TransformMat& previous_chain_pose ) const {
    
    typedef TransformMat::Scalar Scalar_T;

    const auto& intrinsics = node[KALIBR_INTRINSICS];
    auto xi    = intrinsics[0].as<Scalar_T>();
    auto alpha = intrinsics[1].as<Scalar_T>();
    auto fx    = intrinsics[2].as<Scalar_T>();
    auto fy    = intrinsics[3].as<Scalar_T>();
    auto cx    = intrinsics[4].as<Scalar_T>();
    auto cy    = intrinsics[5].as<Scalar_T>();
    auto h     = node[KALIBR_RESOLUTION][1].as<int>();
    auto w     = node[KALIBR_RESOLUTION][0].as<int>();
    auto name  = node[KALIBR_ROSTOPIC].as<std::string>();
    DoubleSphere camera(name, xi, alpha, fx, fy, cx, cy, {h, w});

    if ( node[KALIBR_EXTRINSICS] ) {
        for ( int i = 0; i < 4; ++i ) {
            const auto& row = node[KALIBR_EXTRINSICS][i];
            for ( int j = 0; j < 4; ++j ) {
                camera.extrinsics( i, j ) = row[j].as<Scalar_T>();
            }
        }
    }

    camera.extrinsics = inverse_transform( camera.extrinsics );

    camera.extrinsics = previous_chain_pose * camera.extrinsics.eval();

    return camera;
}