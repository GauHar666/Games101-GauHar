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
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

//实现三维中绕Z轴旋转的变换矩阵，不用平移和缩放
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotation;
    double angle = rotation_angle/180 * MY_PI；
    rotation << cos(angle),-1*sin(angle),0,0,
                sin(angle),cos(angle),0,0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    model = rotation * model;

    return model;
}

//构建透视投影矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) //垂直可视角，宽高比，近面距，远面距
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f proj, ortho;//proj是透视投影矩阵，

    proj<< zNear,0,0,0,
            0,zNear,0,0,
            0,0,zNear+zFar,-zNear*zFar
            0,0,1,0;

    double w,h,z;
    h = zNear* tan(eye_fov/2) *2;
    w = h*aspect_ratio;
    z=zFar-zNear;

    ortho << 2/w ,0,0,0
            0,h/2,0,0
            0,0,z/2,-(zFar+zNear)/2,//正交投影矩阵，因为在观测投影时x0y平面视角默认是中心，所以这里的正交投影就不用平移x和y了
            0,0,0,1;

    projection = ortho*proj*projection;

    return projection;
}

//提高部分：构造一个函数，该函数的作用是得到绕任意过原点的轴的旋转变换矩阵，本质就是罗德里格斯旋转公式
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {//任意轴旋转矩阵（罗德里格斯旋转公式，默认轴过原点）
    double fangle = angle /180 * MY_PI;
    Eigen::Matrix4f I,N,Rod;
    Eigen::Vector4f axi;
    Eigen::RowVector4f taxi;//表示行向量，是横过来的，用来表示axi的转置

    axi<<axis.x(),axis.y(),axis.z(),0;
    taxi<<axis.x(),axis.y(),axis.z(),0;

    I<<1,0,0,0
       0,1,0,0
       0,0,1,0
       0,0,0,1;
    //罗德里格斯的N矩阵公式
    N<<0, -axis.z(), axis.y(), 0,
        axis.z(), 0, -axis.x(), 0,
        -axis.y(), axis.x(), 0, 0,
        0, 0, 0, 1;

    Rod=cos(fangle)*I + (1-cos(fangle))*axi*taxi+sin(fangle)*N;
    Rod(3, 3) = 1;//这里要注意，非齐次坐标的公式应用在齐次坐标上时记得运算完成后把矩阵的右下角改为1，否则会导致图形比例错误
    return Rod;
}


int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {//接收到的参数大于三个，即检测到通过命令行传入参数时
        command_line = true;//设命令行开关标志为开
        angle = std::stof(argv[2]); // -r by default,获取角度
        if (argc == 4) {//如果输入四个那么说明输入了文件名
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);//设定700*700像素的光栅器视口

    Eigen::Vector3f eye_pos = {0, 0, 5}; //相机位置

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}}; //三个顶点

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};//顶点序号

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;//键盘输入
    int frame_count = 0; //帧的序号

    if (command_line) { //如果有命令行输入
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));//作用是将内部的投影矩阵设为给定矩阵 p，并传递给光栅化器.相当于你设置好了，要传给光栅

        //画图的步骤
        r.draw(pos_id, ind_id, rst::Primitive::Triangle); 
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        //写入文件
        cv::imwrite(filename, image);

        return 0;
    }

    //按任意轴旋转：
    bool rflag = false;
    std::cout << "Please enter the axis and angle:" << std::endl;
    std::cin >> raxis.x() >> raxis.y() >> raxis.z() >> ra;//定义罗德里格斯旋转轴和角


    while (key != 27) {//只要没有检测到按下ESC就循环(ESC的ASCII码是27)
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        if (rflag) //如果按下r了，就开始绕给定任意轴旋转
            r.set_rodrigues(get_rotation(raxis, rangle));
        else
            r.set_rodrigues(get_rotation({ 0,0,1 }, 0));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        //按a逆时针旋转10°，按d顺时针旋转10°
        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'r') {//按下r，绕给定任意轴旋转
            rflag = true;
            rangle += ra;
        }
    }

    return 0;
}
