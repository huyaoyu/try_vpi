
// #include <array>
#include <iostream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

template < typename Scalar_T >
using ExtrinsicMat = Eigen::Matrix<Scalar_T, 4, 4>;

template < typename Scalar_T >
static ExtrinsicMat<Scalar_T> inverse_transform( const ExtrinsicMat<Scalar_T>& T ) {
    ExtrinsicMat<Scalar_T> inversed = ExtrinsicMat<Scalar_T>::Identity();

    const auto RT = T.block(0, 0, 3, 3).transpose();

    inversed.block(0, 0, 3, 3) = RT;
    inversed.block(0, 3, 3, 1) = -RT * T.block(0, 3, 3, 1);

    return inversed;
}

template < typename Scalar_T >
struct DSCameraModel {

    DSCameraModel()
    {
        extrinsics = ExtrinsicMat<Scalar_T>::Identity();
    }

    template < typename Scalar_TF >
    friend std::ostream& operator << ( std::ostream& os, const DSCameraModel<Scalar_TF>& camera );

    Scalar_T xi;
    Scalar_T alpha;
    Scalar_T fx;
    Scalar_T fy;
    Scalar_T cx;
    Scalar_T cy;
    Eigen::Matrix<int, 2, 1> shape;
    ExtrinsicMat<Scalar_T> extrinsics;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template < typename Scalar_T >
std::ostream& operator << ( std::ostream& os, const DSCameraModel<Scalar_T>& camera ) {
    os << "xi    = " << camera.xi                << "\n";
    os << "alpha = " << camera.alpha             << "\n";
    os << "fx    = " << camera.fx                << "\n";
    os << "fy    = " << camera.fy                << "\n";
    os << "cx    = " << camera.cx                << "\n";
    os << "cy    = " << camera.cy                << "\n";
    os << "shape = " << camera.shape.transpose() << "\n";
    os << "extrinsics = \n" << camera.extrinsics << "\n";

    return os;
}

class KalibrParser {
public:
    KalibrParser() = default;
    ~KalibrParser() = default;

    template < typename Scalar_T >
    DSCameraModel<Scalar_T> parser_camera_node(
        YAML::Node&& node,
        const ExtrinsicMat<Scalar_T>& previous_chain_pose = ExtrinsicMat<Scalar_T>::Identity()
    ) const ;

public:
    static const std::string KALIBR_INTRINSICS;
    static const std::string KALIBR_EXTRINSICS;
    static const std::string KALIBR_RESOLUTION;
};

const std::string KalibrParser::KALIBR_INTRINSICS = "intrinsics";
const std::string KalibrParser::KALIBR_EXTRINSICS = "T_cn_cnm1";
const std::string KalibrParser::KALIBR_RESOLUTION = "resolution";

template < typename Scalar_T >
DSCameraModel<Scalar_T>
KalibrParser::parser_camera_node(
    YAML::Node&& node,
    const ExtrinsicMat<Scalar_T>& previous_chain_pose ) const {
        
    DSCameraModel<Scalar_T> camera;

    const auto& intrinsics = node[KALIBR_INTRINSICS];
    camera.xi    = intrinsics[0].as<Scalar_T>();
    camera.alpha = intrinsics[1].as<Scalar_T>();
    camera.fx    = intrinsics[2].as<Scalar_T>();
    camera.fy    = intrinsics[3].as<Scalar_T>();
    camera.cx    = intrinsics[4].as<Scalar_T>();
    camera.cy    = intrinsics[5].as<Scalar_T>();
    camera.shape(0, 0) = node[KALIBR_RESOLUTION][1].as<Scalar_T>();
    camera.shape(1, 0) = node[KALIBR_RESOLUTION][0].as<Scalar_T>();

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

int main(int argc, char** argv) {
    std::cout << "Hello, read_kalibr! \n";

    const std::string kalibr_result_fn = "../data/kalibr.yaml";

    YAML::Node calib = YAML::LoadFile(kalibr_result_fn);

    std::cout << calib["cam0"][KalibrParser::KALIBR_INTRINSICS] << "\n";
    std::cout << calib["cam1"][KalibrParser::KALIBR_EXTRINSICS][0] << "\n";

    KalibrParser parser;
    
    DSCameraModel<float> cam0 = parser.parser_camera_node<float>(calib["cam0"]);
    DSCameraModel<float> cam1 = parser.parser_camera_node<float>(calib["cam1"]);
    DSCameraModel<float> cam2 = parser.parser_camera_node<float>(calib["cam2"], cam1.extrinsics);

    std::cout << "cam0: \n" << cam0 << "\n";
    std::cout << "cam1: \n" << cam1 << "\n";
    std::cout << "cam2: \n" << cam2 << "\n";

    return 0;
}
