
// C++ std.
// #include <filesystem> // Not supported by GCC 7.
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// System.
#include <boost/filesystem.hpp> // For GCC version under 8.
#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>

// OpenCV.
#include <opencv2/opencv.hpp>

// VPI.
#include <vpi/Context.h>
#include <vpi/Image.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Stream.h>
#include <vpi/WarpMap.h>
#include <vpi/algo/Remap.h>

// Local.
#include "calib/kalibr_parser.hpp"
#include "camera_model/double_sphere.hpp"

// Namespace.
// namespace fs = std::filesystem;
namespace fs = boost::filesystem;

constexpr const auto PI = boost::math::constants::pi<double>();

static
bool test_file(const std::string& fn) {
    const fs::path path {fn};
    return fs::exists(path);
}

static
cv::Mat read_image(const std::string& fn) {
    if ( !test_file(fn) ) {
        std::stringstream ss;
        ss << fn << " does not exit. ";
        std::cout << ss.str() << "\n";
        throw std::runtime_error(ss.str());
    }

    return cv::imread(fn, cv::IMREAD_UNCHANGED);
}

static
mvs::PointMat3 get_xyz(int fov_x, int fov_y, const mvs::Shape_t& shape) {
    typedef mvs::PointMat3::Scalar Scalar_t;

    const int N = shape.size();
    
    // Angle values associated with the x and y pixel coordinates in the final pinhole camera.
    mvs::PointMat1 ax = 
        mvs::PointMat1::LinSpaced(shape.w, -1, 1) * static_cast<Scalar_t>( fov_x / 2.0 / 180 * PI );
    mvs::PointMat1 ay = 
        mvs::PointMat1::LinSpaced(shape.h, -1, 1) * static_cast<Scalar_t>( fov_y / 2.0 / 180 * PI );

    // The xyz coordinates of pixels in the pinhole camera.
    mvs::PointMat3 xyz;
    xyz.resize( 3, N );

    xyz.row(0) = 
        ax.array().tan().matrix().replicate( 1, shape.h );
    // Eigen 3.4.
    // xyz.row(1) = 
    //     ay.array().tan().matrix().replicate( shape.w, 1 ).reshaped(1, N);

    // Eigen 3.3.
    Eigen::Matrix<Scalar_t, Eigen::Dynamic, Eigen::Dynamic> temp_row = 
        ay.array().tan().matrix().replicate( shape.w, 1 );
    xyz.row(1) = Eigen::Map<Eigen::Matrix<Scalar_t, 1, Eigen::Dynamic>>( 
        temp_row.data(), 1, N
    );
    xyz.row(2) = mvs::PointMat1::Ones(1, N);

    return xyz;
}

static
std::vector< std::pair< cv::Mat, cv::Mat > >
get_remap_coordinates(
    const mvs::DoubleSphere& cam0,
    const mvs::DoubleSphere& cam1,
    const Eigen::Matrix3f& R0,
    const Eigen::Matrix3f& R1) {

    // Get the xyz in the pinhole camera.
    const mvs::Shape_t out_shape { 200, 300 };
    mvs::PointMat3 xyz = get_xyz( 90, 40, out_shape );

    // Apply the rotation to the xyz in the pinhole to get the new xyz in the fisheye camera frame.
    auto xyz0 = R0 * xyz.eval();

    auto [ ux0, uy0 ] = cam0.project_3d_2_image_plane_separated( xyz0 );

    // Convert uxuy to cv::Mat.
    auto xx0 = cv::Mat( out_shape.h, out_shape.w, CV_32FC1, ux0.data() );
    auto yy0 = cv::Mat( out_shape.h, out_shape.w, CV_32FC1, uy0.data() );

    auto xyz1 = R1 * xyz.eval();
    auto [ ux1, uy1 ] = cam1.project_3d_2_image_plane_separated( xyz1 );
    // Convert uxuy to cv::Mat.
    auto xx1 = cv::Mat( out_shape.h, out_shape.w, CV_32FC1, ux1.data() );
    auto yy1 = cv::Mat( out_shape.h, out_shape.w, CV_32FC1, uy1.data() );

    // Copy the results as output.
    return { 
        { xx0.clone(), yy0.clone() }, 
        { xx1.clone(), yy1.clone() }
    };
}

static
void create_single_vpi_image(int w, int h, const VPIImageFormat& type) {
    // Empty function.
}

template< typename VPIImage_t, typename... VPIImage_vt >
void create_single_vpi_image(int w, int h, const VPIImageFormat& type, VPIImage_t& vpi_image, VPIImage_vt& ... out) {
    vpiImageCreate(w, h, type, 0, &vpi_image);
    create_single_vpi_image(w, h, type, out...);
}

template< typename ...VPIImage_vt >
void create_vpi_image_by_copy_type(int w, int h, const VPIImage& copy_type_from, VPIImage_vt& ... out) {
    VPIImageFormat vpi_type;
    vpiImageGetFormat( copy_type_from, &vpi_type );

    // Unpack the parameter pack by recursive calls.
    create_single_vpi_image(w, h, vpi_type, out...);
}

static
void populate_vpi_warp_map( const cv::Mat& xx, const cv::Mat& yy, VPIWarpMap& vpi_warp_map ) {
    const int W = xx.cols;
    const int H = xx.rows;
    
    std::memset( &vpi_warp_map, 0, sizeof(vpi_warp_map) );
    vpi_warp_map.grid.numHorizRegions  = 1;
    vpi_warp_map.grid.numVertRegions   = 1;
    vpi_warp_map.grid.regionWidth[0]   = W;
    vpi_warp_map.grid.regionHeight[0]  = H;
    vpi_warp_map.grid.horizInterval[0] = 1;
    vpi_warp_map.grid.vertInterval[0]  = 1;
    vpiWarpMapAllocData(&vpi_warp_map);

    vpiWarpMapGenerateIdentity(&vpi_warp_map);
    for ( int i = 0; i < vpi_warp_map.numVertPoints; ++i ) {
        VPIKeypoint* row = ( VPIKeypoint* ) ( ( std::uint8_t* ) vpi_warp_map.keypoints + vpi_warp_map.pitchBytes * i );
        for ( int j = 0; j < vpi_warp_map.numHorizPoints; ++j ) {
            row[j].x = xx.at<float>( i, j );
            row[j].y = yy.at<float>( i, j );
        }
    }
}

std::vector<mvs::DoubleSphere> read_cameras(
    const std::string& yaml_fn ) {
    
    YAML::Node calib = YAML::LoadFile(yaml_fn);

    mvs::KalibrParser parser;
    
    mvs::DoubleSphere cam0 = parser.parser_camera_node(calib["cam0"]);
    mvs::DoubleSphere cam1 = parser.parser_camera_node(calib["cam1"]);
    mvs::DoubleSphere cam2 = parser.parser_camera_node(calib["cam2"], cam1.extrinsics);

    return { cam0, cam1, cam2 }; // Did I just make a copy?
}

int main(int argc, char** argv) {
    std::cout << "Hello, front_view! \n";

    const std::string kalibr_result_fn = "../data/kalibr.yaml";

    std::vector<mvs::DoubleSphere> cameras = read_cameras( kalibr_result_fn );

    for ( const auto& camera : cameras )
        std::cout << camera << "\n";

    const std::string in_img_0_fn  = "../data/cam0.png";
    const std::string in_img_1_fn = "../data/cam1.png";

    // Read the input images.
    cv::Mat in_img_0  = read_image( in_img_0_fn  );
    cv::Mat in_img_1 = read_image( in_img_1_fn );

    Eigen::Matrix3f rot_mat = Eigen::Matrix3f::Zero();
    // rot_mat(0, 0) = -1;
    // rot_mat(1, 2) = -1;
    // rot_mat(2, 1) = -1;
    rot_mat(0, 0) = -1;
    rot_mat(1, 1) = -1;
    rot_mat(2, 2) =  1;

    std::vector< std::pair< cv::Mat, cv::Mat > > vec_xxyy = get_remap_coordinates(
        cameras[0], cameras[1],
        rot_mat, rot_mat
    );

    auto& xx0 = vec_xxyy[0].first;
    auto& yy0 = vec_xxyy[0].second;
    auto& xx1 = vec_xxyy[1].first;
    auto& yy1 = vec_xxyy[1].second;

    // Debug.
    std::cout << "xx0: [ " << xx0.rows << ", " << xx0.cols << " ] \n";
    std::cout << "yy0: [ " << yy0.rows << ", " << yy0.cols << " ] \n";

    // VPI context.
    VPIContext ctx;
    vpiContextCreate(0, &ctx);
    vpiContextSetCurrent(ctx);

    // VPI streams.
    VPIStream stream_0, stream_1;
    vpiStreamCreate(0, &stream_0);
    vpiStreamCreate(0, &stream_1);

    // VPI image buffer wrappers.
    VPIImage img_fisheye_0, img_fisheye_1;
    vpiImageCreateOpenCVMatWrapper(in_img_0, 0, &img_fisheye_0);
    vpiImageCreateOpenCVMatWrapper(in_img_1, 0, &img_fisheye_1);

    // VPI output image.
    VPIImage out_0, out_1;
    create_vpi_image_by_copy_type( xx0.cols, xx0.rows, img_fisheye_0, out_0, out_1 );

    // VPI dense warp map.
    VPIWarpMap vpi_warp_map_0, vpi_warp_map_1;
    populate_vpi_warp_map(xx0, yy0, vpi_warp_map_0);
    populate_vpi_warp_map(xx1, yy1, vpi_warp_map_1);

    // VPI payload.
    VPIPayload warp_0, warp_1;
    vpiCreateRemap( VPI_BACKEND_CUDA, &vpi_warp_map_0, &warp_0 );
    vpiCreateRemap( VPI_BACKEND_CUDA, &vpi_warp_map_1, &warp_1 );

    // Extract.
    vpiSubmitRemap( stream_0, VPI_BACKEND_CUDA, warp_0, img_fisheye_0, out_0, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0 );
    vpiSubmitRemap( stream_1, VPI_BACKEND_CUDA, warp_1, img_fisheye_1, out_1, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0 );

    // Sync.
    vpiStreamSync(stream_0);
    vpiStreamSync(stream_1);

    // Copy the output.
    VPIImageData out_data_0, out_data_1;
    vpiImageLock( out_0, VPI_LOCK_READ, &out_data_0 );
    vpiImageLock( out_1, VPI_LOCK_READ, &out_data_1 );

    cv::Mat out_ocv_0, out_ocv_1;
    vpiImageDataExportOpenCVMat(out_data_0, &out_ocv_0);
    vpiImageDataExportOpenCVMat(out_data_1, &out_ocv_1);

    cv::imwrite("pinhole_0.png", out_ocv_0);
    cv::imwrite("pinhole_1.png", out_ocv_1);

    // Clean-up.
    vpiImageDestroy(out_1);
    vpiImageDestroy(out_0);

    vpiPayloadDestroy(warp_1);
    vpiPayloadDestroy(warp_0);
    vpiWarpMapFreeData(&vpi_warp_map_1);
    vpiWarpMapFreeData(&vpi_warp_map_0);

    vpiImageDestroy(img_fisheye_1);
    vpiImageDestroy(img_fisheye_0);

    vpiStreamDestroy(stream_1);
    vpiStreamDestroy(stream_0);

    vpiContextDestroy(ctx);

    return 0;
}
