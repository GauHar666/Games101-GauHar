#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    //套公式：`b01（t）= （1-t）b0+tb1` `b11(t)=(1-t)b1+tb2`
    if(control_points.size() == 2){
        return control_points[0]+ t*(control_points[1]-control_points[0]);
    }
    //重新用一个vector存放少了一个点的结果：
    std::vector<cv::Point2f> control_points_temp;
    for(int i=0;i<control_points.size()-1;i++){
        auto temp = control_points[i]+t*(control_points[i+1]-control_points[i]);
        control_points_temp.push_back(temp);
    }
    return recursive_bezier(control_points_temp,t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    //t在0-1之间迭代
    for(double t = 0; t <= 1; t+=0.001){
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;//使用绿色显示。
        //以下是提高部分，防走样操作：
        const float x[4] = {0.0,0.0,0.5,-0.5};
        const float y[4] = {0.5,-0.5,0.0,0.0};
        int xnow = round(point.x),ynow = round(point.y);//四舍五入
        float d = std::sqrt(std::pow(point.x-xnow,2)+std::pow(point.y-ynow,2));
        for(int i=0;i<4;i++){
            float x_neibor = floor(point.x+x[i]);//floor表示向下取整
            float y_neibor = floor(point.y+y[i]);
            if(x_neibor>=0 && x_neibor<700 && y_neibor>=0 && y_neibor<700){
                float w = d/std::sqrt((std::pow(x_neibor-point.x,2)+std::pow(y_neibor-point.y,2)));
                //当你处理下一个像素时，可能得到其最近邻域有一个是上一个已经处理的像素，如果直接对其赋值，会导致曲线有些像素出现不正常的暗，看起来断断续续的。
                window.at<cv::Vec3b>(y_neibor, x_neibor)[1] = std::max(float(window.at<cv::Vec3b>(y_neibor, x_neibor)[1]), 255 * w);
            }
        }
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //原始框架的看看对不对久打开对naive_bezier的注释，想要看贝塞尔曲线的实现的时候就打开bezier
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
