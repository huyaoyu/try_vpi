
// C++ std.
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>

// System.
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
#include "camera_model/double_sphere.hpp"

constexpr const auto PI = boost::math::constants::pi<double>();

static
bool test_file(const std::string& fn) {
    const std::filesystem::path path {fn};
    return std::filesystem::exists(path);
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
    xyz.row(1) = 
        ay.array().tan().matrix().replicate( shape.w, 1 ).reshaped(1, N);
    xyz.row(2) = mvs::PointMat1::Ones(1, N);

    return xyz;
}

static
std::pair< cv::Mat, cv::Mat >
get_remap_coordinates() {
    typedef mvs::PointMat3::Scalar Scalar_t;

    // Get the xyz in the pinhole camera.
    const mvs::Shape_t out_shape { 200, 300 };
    mvs::PointMat3 xyz = get_xyz( 60, 40, out_shape );

    // Rotation between the final pinhole camera and the fisheye camera.
    Eigen::Matrix<Scalar_t, 3, 3> rot_mat = Eigen::Matrix<Scalar_t, 3, 3>::Zero();
    rot_mat(0, 0) =  1;
    rot_mat(1, 2) =  1;
    rot_mat(2, 1) = -1;

    // Apply the rotation to the xyz in the pinhole to get the new xyz in the fisheye camera frame.
    xyz = rot_mat * xyz.eval();

    const mvs::Shape_t shape { 687, 687 };
    mvs::DoubleSphere camera_model { -0.17023409, 0.59679147, 156.96507623, 157.72873153, 343, 343, shape };
    auto [ ux, uy ] = camera_model.project_3d_2_image_plane_separated( xyz );

    // Convert uxuy to cv::Mat.
    auto xx = cv::Mat( out_shape.h, out_shape.w, CV_32FC1, ux.data() );
    auto yy = cv::Mat( out_shape.h, out_shape.w, CV_32FC1, uy.data() );

    // Copy the results as output.
    return { xx.clone(), yy.clone() };
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

int main(int argc, char** argv) {
    std::cout << "Hello, CameraModel! \n";

    const std::string in_img_left_fn  = argv[1];
    const std::string in_img_right_fn = argv[2];

    // Read the input images.
    cv::Mat in_img_left  = read_image( in_img_left_fn  );
    cv::Mat in_img_right = read_image( in_img_right_fn );

    auto [ xx, yy ] = get_remap_coordinates();

    // Debug.
    std::cout << "xx: [ " << xx.rows << ", " << xx.cols << " ] \n";
    std::cout << "yy: [ " << yy.rows << ", " << yy.cols << " ] \n";

    // VPI context.
    VPIContext ctx;
    vpiContextCreate(0, &ctx);
    vpiContextSetCurrent(ctx);

    // VPI streams.
    VPIStream stream_left, stream_right;
    vpiStreamCreate(0, &stream_left);
    vpiStreamCreate(0, &stream_right);

    // VPI image buffer wrappers.
    VPIImage img_fisheye_left, img_fisheye_right;
    vpiImageCreateOpenCVMatWrapper(in_img_left,  0, &img_fisheye_left);
    vpiImageCreateOpenCVMatWrapper(in_img_right, 0, &img_fisheye_right);

    // VPI output image.
    VPIImage out_left, out_right;
    create_vpi_image_by_copy_type( xx.cols, xx.rows, img_fisheye_left, out_left, out_right );

    // VPI dense warp map.
    VPIWarpMap vpi_warp_map;
    populate_vpi_warp_map(xx, yy, vpi_warp_map);

    // VPI payload.
    VPIPayload warp_left, warp_right;
    vpiCreateRemap( VPI_BACKEND_CUDA, &vpi_warp_map, &warp_left );

    // Extract.
    vpiSubmitRemap( stream_left, VPI_BACKEND_CUDA, warp_left, img_fisheye_left, out_left, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0);

    // Sync.
    vpiStreamSync(stream_left);
    vpiStreamSync(stream_right);

    // Copy the output.
    VPIImageData out_data_left;
    vpiImageLock( out_left, VPI_LOCK_READ, &out_data_left );

    cv::Mat out_ocv_left, out_ocv_right;
    vpiImageDataExportOpenCVMat(out_data_left, &out_ocv_left);

    imwrite("pinhole.png", out_ocv_left);

    // Clean-up.
    vpiImageDestroy(out_right);
    vpiImageDestroy(out_left);

    vpiPayloadDestroy(warp_left);
    vpiWarpMapFreeData(&vpi_warp_map);

    vpiImageDestroy(img_fisheye_right);
    vpiImageDestroy(img_fisheye_left);

    vpiStreamDestroy(stream_left);
    vpiStreamDestroy(stream_right);

    vpiContextDestroy(ctx);

    return 0;
}