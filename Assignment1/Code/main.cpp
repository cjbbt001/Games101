#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1,-eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // Edit begin

    float theta = rotation_angle / 180.0f * MY_PI;

    model << std::cos(theta), -std::sin(theta), 0, 0,
             std::sin(theta),  std::cos(theta), 0, 0,
             0,                0,               1, 0,
             0,                0,               0, 1;

    return model;

    // Edit end
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // Edit begin

    Eigen::Matrix4f projection;

    float angle_rad = eye_fov / 180.0f * MY_PI;
    float t = zNear * std::tan(angle_rad / 2);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;

    Eigen::Matrix4f M_ortho_scale(4, 4);
    M_ortho_scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zFar - zNear), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f M_ortho_translate(4, 4);
    M_ortho_translate << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;

    Eigen::Matrix4f M_persp_to_ortho(4, 4);
    M_persp_to_ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    Eigen::Matrix4f M_flip(4, 4);
    M_flip << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;

    M_persp_to_ortho = M_persp_to_ortho * M_flip;

    projection = M_ortho_scale * M_ortho_translate * M_persp_to_ortho;

    return projection;

    // Edit end
}


// Edit begin
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

    float theta = angle / 180.0f * MY_PI;

    axis.normalize();

    float x = axis.x();
    float y = axis.y();
    float z = axis.z();

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f N;
    N << 0, -z,  y,
         z,  0, -x,
        -y,  x,  0;

    Eigen::Matrix3f outer_product = axis * axis.transpose();

    Eigen::Matrix3f R =
        std::cos(theta) * I +
        (1 - std::cos(theta)) * outer_product +
        std::sin(theta) * N;

    rotation << R(0, 0), R(0, 1), R(0, 2), 0,
                R(1, 0), R(1, 1), R(1, 2), 0,
                R(2, 0), R(2, 1), R(2, 2), 0,
                0,       0,       0,       1;

    return rotation;
}
// Edit end

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    // Edit begin
    Eigen::Vector3f rotate_axis = {1,1,0};
    // Edit end

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // Edit begin
        //围绕z轴旋转
        //r.set_model(get_model_matrix(angle));
        //围绕任意轴旋转
        r.set_model(get_rotation(rotate_axis,angle));
        // Edit end

        r.set_view(get_view_matrix(eye_pos));
        //注意这里写入的zNear和zFar是正数，代表着距离，但课程上推导的透视矩阵是坐标，且假定是朝向z负半轴的，所以透视矩阵是需要取反的
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // Edit begin
        //围绕z轴旋转
        //r.set_model(get_model_matrix(angle));
        //围绕任意轴旋转
        r.set_model(get_rotation(rotate_axis,angle));
        // Edit end

        r.set_view(get_view_matrix(eye_pos));
        //注意这里写入的zNear和zFar是正数，代表着距离，但课程上推导的透视矩阵是坐标，且假定是朝向z负半轴的，所以透视矩阵是需要取反的
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
