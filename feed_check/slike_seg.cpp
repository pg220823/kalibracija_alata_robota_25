#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<iostream>
#include<cmath>
#include<mutex>
#include<sys/socket.h>
#include<netinet/in.h>
#include<unistd.h>
#include<thread>
#include<sstream>
#include<algorithm>
#include<cctype>
#include<eigen3/Eigen/Dense>
#include<vector>

int gzh , gzs, gzv, dzh, dzs ,dzv;

int mx0,my0,mx1,my1;
cv::Point gl0(405,365); //gornje lijevo za rez/box 0
cv::Point gl1(405,365); //gornje lijevo za rez/box 1
// matrice kamera i matrice distorzijskih koeficjenata
cv::Mat cam_mat_0 = (cv::Mat_<double>(3,3) << 815.4701998334627, 0.0, 816.3746024757398,
                     0.0, 814.9427154247614, 616.7998924543018,
                     0.0, 0.0, 1.0);
cv::Mat dist_coef_0 = (cv::Mat_<double>(1,5) << 0.001351728590951338, 0.07330065566520007, -0.002845433292251144, 0.002334882552224456, -0.09921688660118233);

cv::Mat cam_mat_1 = (cv::Mat_<double>(3,3) << 814.3097531833109, 0.0, 810.6401441916403,
        0.0, 813.3822440603992, 626.6564417158986,
        0.0, 0.0, 1.0);
cv::Mat dist_coef_1 = (cv::Mat_<double>(1,5) << 0.0110281691408063, 0.06642369117113196, 0.001098443458176403, -0.0006956327194863606, -0.09539373780159867);

bool running = true;

float dx,dy,dz,rx_r,ry_r,rz_r;

std::vector<Eigen::Matrix4d> kalibracijske_mat_trans;

void onMouse0(int event,int x, int y, int flags, void* userdata)
{
    if(event == cv::EVENT_MOUSEMOVE){
        mx0 = x;
        my0 = y;
    }
}
void onMouse1(int event,int x, int y, int flags, void* userdata)
{
    if(event == cv::EVENT_MOUSEMOVE){
        mx1 = x;
        my1 = y;
    }
}
void snimanje_slike()
{
   std::string pipeline0 = "nvarguscamerasrc sensor-id = 0   ! video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
   std::string pipeline1 = "nvarguscamerasrc sensor-id = 1   ! video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
   cv::VideoCapture cap0(pipeline0, cv::CAP_GSTREAMER);
   cv::VideoCapture cap1(pipeline1, cv::CAP_GSTREAMER);

    if (!cap1.isOpened() | !cap0.isOpened() ){
        std::cerr << "Error : Could not open cameras" << std::endl;
        cap0.release();
        cap1.release();
    }


    cv::Size vel_frame(1640,1232);

    cv::Mat map01,map02,map11,map12;
    cv::initUndistortRectifyMap(cam_mat_0,dist_coef_0,cv::Mat(),cam_mat_0,vel_frame,CV_32FC1,map01,map02);
    cv::initUndistortRectifyMap(cam_mat_1,dist_coef_1,cv::Mat(),cam_mat_1,vel_frame,CV_32FC1,map11,map12);




    cv::Mat dframe0,frame0;
    cv::Mat dframe1,frame1;
    while (true) {
        cap0 >> dframe0;
        cap1 >> dframe1;

        cv::remap(dframe0,frame0,map01,map02,cv::INTER_LINEAR);
        cv::remap(dframe1,frame1,map11,map12,cv::INTER_LINEAR);


        if(frame0.empty() | frame1.empty() ){
            break;
        }

        //cv::namedWindow("Camera 0");
        cv::setMouseCallback("Camera 0",onMouse0,nullptr);
        std::string posm0 = "(" + std::to_string(mx0) + "),(" + std::to_string(my0) + ")";
        cv::putText(frame0,posm0,cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255),1);//za određivanje interesa
        //cv::Point gl0(400,260); //gornje lijevo
        cv::Point dd0(1230,840); //donje desno
        cv::Rect vp0(gl0,dd0);
        cv::Mat rezano0 = frame0(vp0).clone();

        //cv::namedWindow("Camera 1");
        cv::setMouseCallback("Camera 1",onMouse1,nullptr);
        std::string posm1 = "(" + std::to_string(mx1) + "),(" + std::to_string(my1) + ")";
        cv::putText(frame1,posm1,cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255),1);//za određivanje interesa
        //cv::Point gl1(380,250); //gornje lijevo
        cv::Point dd1(1230,840); //donje desno
        cv::Rect vp1(gl1,dd1);
        cv::Mat rezano1 = frame1(vp1).clone();

        //cv::imshow("Camera 0", frame0);
        cv::imshow("Camera 0- interest", rezano0);


        //cv::imshow("Camera 1", frame1);
        cv::imshow("Camera 1- interest", rezano1);

        if (cv::waitKey(1) == 27) {  // esc
            std::cout<<"proslo esc"<<std::endl;
                    break;
                }
        else if (cv::waitKey(30) == 115){ //s
            std::cout<<"proslo s"<<std::endl;
            cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/slika_cam0.png",frame0);
            cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/slika_rez0.png",rezano0);
            cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/slika_cam1.png",frame1);
            cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/slika_rez1.png",rezano1);
            break;
            }
    }

    cap0.release();
    cap1.release();
    cv::destroyAllWindows();


}


void segmentacija() {

//-------------------kamera0-------------------------------------------------------------------
cv::Mat rezano0 = cv::imread("/home/pg220823/Desktop/dipl/slike_seg/slika_rez0.png");
cv::cvtColor(rezano0,rezano0,cv::COLOR_BGR2Lab);
std::vector<cv::Mat> lab0;
cv::split(rezano0, lab0);//podjela na kanale
//cv::imshow("orig0",rezano0);
//cv::imshow("l0",lab0[0]);
//cv::imshow("a0",lab0[1]);//kanal od interesa (zelena-crvena)(0-128 zelena)
//cv::imshow("b0",lab0[2]);

cv::Mat laba0, rja_o0 , rja_a0;
cv::bilateralFilter(lab0[1],laba0,5,150,150);
//cv::imshow("blura0", laba0);
cv::threshold(laba0,rja_o0,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU);//segmentacija otsu
//cv::imshow("rja_o0",rja_o0);
cv::adaptiveThreshold(laba0, rja_a0, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 391, 1);//segmentacija adaptivni prag
//cv::imshow("rja_a0",rja_a0);
//-------------------kamera1--------------------------------------------------------------------
cv::Mat rezano1 = cv::imread("/home/pg220823/Desktop/dipl/slike_seg/slika_rez1.png");
cv::cvtColor(rezano1,rezano1,cv::COLOR_BGR2Lab);
std::vector<cv::Mat> lab1;
cv::split(rezano1, lab1);//podjela na kanale
//cv::imshow("orig1",rezano1);
//cv::imshow("l1",lab1[0]);
//cv::imshow("a1",lab1[1]);//kanal od interesa (zelena-crvena)(0-128 zelena)
//cv::imshow("b1",lab1[2]);

cv::Mat laba1, rja_o1 , rja_a1;
cv::bilateralFilter(lab1[1],laba1,5,150,150);
cv::imwrite("/home/pg220823/Desktop/slike/bilateral.png",laba1);
//cv::imshow("blura1", laba1);
cv::threshold(laba1,rja_o1,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU);//segmentacija otsu
//cv::imshow("rja_o1",rja_o1);
cv::adaptiveThreshold(laba1, rja_a1, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 391, 1);//segmentacija adaptivni prag
//cv::imshow("rja_a1",rja_a1);

cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/slika_seg_a0.png",rja_a0);
cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/slika_seg_a1.png",rja_a1);
cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/slika_seg_o0.png",rja_o0);
cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/slika_seg_o1.png",rja_o1);


}

void tocke_osi_alata (const std::vector<cv::Point2f>& vrhovi,cv::Point2f& tocka_osi, cv::Point2f& tocka_vrh){
    float dist1 = cv::norm(vrhovi[0]-vrhovi[1]);
    float dist2 = cv::norm(vrhovi[1]-vrhovi[2]);
    if(dist1 < dist2){
    tocka_osi = (vrhovi[0]+vrhovi[1])/2.0f;
    tocka_vrh = (vrhovi[2]+vrhovi[3])/2.0f;
    }
    else{
        tocka_osi = (vrhovi[1]+vrhovi[2])/2.0f;
        tocka_vrh = (vrhovi[3]+vrhovi[0])/2.0f;
    }

    if (tocka_osi.y > tocka_vrh.y){
       std::swap(tocka_osi,tocka_vrh);
    }

}

std::vector<cv::Point2f> vrhoviboxa (const cv::Mat& slika){
    cv::Mat gray0;
    cv::cvtColor(slika,gray0,cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point>> konture;
    std::vector<cv::Vec4i> hijerarhija;
    cv::findContours(gray0, konture, hijerarhija, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double min_povrsina = 200.0;
    std::vector<std::vector<cv::Point>> filtrirane;
    for(const auto& konture : konture){
        if(cv::contourArea(konture) >= min_povrsina){
            filtrirane.push_back(konture);
        }
    }

    int index_1 =-1;
    double max_povrsina_1 = 0.0;
    int index_2 = -1;
    double max_povrsina_2 = 0.0;

    for(size_t i = 0; i < filtrirane.size(); ++i ){

        double povrsina = cv::contourArea(filtrirane[i]);

        if(povrsina > max_povrsina_1){
            max_povrsina_2 = max_povrsina_1;
            index_2 = index_1;

            max_povrsina_1 = povrsina;
            index_1 = static_cast<int>(i);
        }
        else if (povrsina > max_povrsina_2){
            max_povrsina_2 = povrsina;
            index_2 = static_cast<int>(i);
        }
    }

    std::vector<cv::Point> kontura_2 = filtrirane[index_1];// index_1 za otsu, index_2 za adaptivno

    cv::RotatedRect minbox = cv::minAreaRect(kontura_2);


    cv::Point2f vrhovi[4];
    minbox.points(vrhovi);
    return std::vector<cv::Point2f>(std::begin(vrhovi),std::end(vrhovi));

}

void boxovi_tocke (float &dx_m, float &dy_m, float &dz_m ){
    cv::Mat objekti0 = cv::imread("/home/pg220823/Desktop/dipl/slike_seg/slika_seg_o0.png"); // a0(adaptivno) , o0(otsu)
    std::vector<cv::Point2f> vrhovi0 = vrhoviboxa(objekti0);
    cv::Mat orig0 = cv::imread("/home/pg220823/Desktop/dipl/slike_seg/slika_cam0.png");
    for (auto& tocka : vrhovi0){
        tocka.x += static_cast<float>(gl0.x);
        tocka.y += static_cast<float>(gl0.y);
    }
    for(int i = 0; i < 4; i++){
        cv::line(orig0,vrhovi0[i],vrhovi0[(i+1) % 4], cv::Scalar(255,0,0),1);
    }

    cv::Mat objekti1 = cv::imread("/home/pg220823/Desktop/dipl/slike_seg/slika_seg_o1.png");// a1(adaptivno) , o1(otsu)
    std::vector<cv::Point2f> vrhovi1 = vrhoviboxa(objekti1);
    cv::Mat orig1 = cv::imread("/home/pg220823/Desktop/dipl/slike_seg/slika_cam1.png");
    for (auto& tocka : vrhovi1){
        tocka.x += static_cast<float>(gl1.x);
        tocka.y += static_cast<float>(gl1.y);
    }
    for(int i = 0; i < 4; i++){
        cv::line(orig1,vrhovi1[i],vrhovi1[(i+1) % 4], cv::Scalar(255,0,0),1);
    }

    cv::Point2f tocka_osi0,tocka_vrh0,tocka_osi1,tocka_vrh1;
    tocke_osi_alata(vrhovi0,tocka_osi0,tocka_vrh0);
    cv::circle(orig0,tocka_osi0,5,cv::Scalar(255,0,0),-1);
    cv::circle(orig0,tocka_vrh0,5,cv::Scalar(0,0,255),-1);

    tocke_osi_alata(vrhovi1,tocka_osi1,tocka_vrh1);
    cv::circle(orig1,tocka_osi1,5,cv::Scalar(255,0,0),-1);
    cv::circle(orig1,tocka_vrh1,5,cv::Scalar(0,0,255),-1);

    int cx0 = orig0.cols /2;
    int cy0 = orig0.rows /2;
    int cx1 = orig1.cols /2;
    int cy1 = orig1.rows /2;
    cv::Point centar0 = cv::Point(cx0,cy0);
    cv::Point centar1 = cv::Point(cx1,cy1);

    cv::circle(orig0,centar0,5,cv::Scalar(0,255,255),-1);
    cv::circle(orig1,centar1,5,cv::Scalar(0,255,255),-1);

    //ofsset u pixelima

    float dx0 = cv::norm(tocka_vrh0.x - centar0.x);// dx za kameru 0 pikseli
    float dy0 = cv::norm(tocka_vrh0.y - centar0.y);// dy za kameru 0 pikseli
    float dx1 = cv::norm(tocka_vrh1.x - centar1.x);// dx za kameru 1 pikseli
    float dy1 = cv::norm(tocka_vrh1.y - centar1.y);// dy za kameru 1 pikseli

    // definiranje gibanja

    if(tocka_vrh0.x > centar0.x){
        dx0 = dx0;
    }
    else{
        dx0 = -dx0;
    }
    if(tocka_vrh0.y > centar0.y){
        dy0 = dy0;
    }
    else{
        dy0 = -dy0;
    }
    if(tocka_vrh1.x > centar1.x){
        dx1 = -dx1;
    }
    else{
        dx1 = dx1;
    }
    if(tocka_vrh1.y > centar1.y){
        dy1 = dy1;
    }
    else{
        dy1 = -dy1;
    }

    cv::line(orig0,centar0, cv::Point(tocka_vrh0.x, centar0.y),cv::Scalar(255,255,255),1);
    cv::line(orig0,cv::Point(tocka_vrh0.x, centar0.y), tocka_vrh0,cv::Scalar(255,255,255),1);

    cv::line(orig1,centar1, cv::Point(tocka_vrh1.x, centar1.y),cv::Scalar(255,255,255),1);
    cv::line(orig1,cv::Point(tocka_vrh1.x, centar1.y), tocka_vrh1,cv::Scalar(255,255,255),1);

    //konverzija ofsseta u mm

    float fok_duljina_x0 = cam_mat_0.at<double>(0,0);
    float fok_duljina_y0 = cam_mat_0.at<double>(1,1);
    float fok_duljina_x1 = cam_mat_1.at<double>(0,0);
    float fok_duljina_y1 = cam_mat_1.at<double>(1,1);
    float dubina = 82.0;// udaljenost od sjecista osi kamera

    float dx0_mm = (dx0 * dubina) / fok_duljina_x0;
    float dy0_mm = (dy0 * dubina) / fok_duljina_y0;
    float dx1_mm = (dx1 * dubina) / fok_duljina_x1;
    float dy1_mm = (dy1 * dubina) / fok_duljina_y1;

    std::cout << "y os udaljenost od sredine(cam0 - x):" << dx0 <<"pikesela" << ", " << dx0_mm << "mm" << std::endl;
    std::cout << "z os udaljenost od sredine(cam0 - y):" << dy0 <<"pikesela" << ", " << dy0_mm << "mm" << std::endl;
    std::cout << "x os udaljenost od sredine(cam1 - x):" << dx1 <<"pikesela" << ", " << dx1_mm << "mm" << std::endl;
    std::cout << "z os udaljenost od sredine(cam0 - y):" << dy1 <<"pikesela" << ", " << dy1_mm << "mm" << std::endl;

    //float dz_mm = dy0_mm > dy1_mm ? dy0_mm : dy1_mm;
    float dz_mm = dy0_mm;
    // konvertiranje u metre za slanje robotu
    dx_m = std::round(dx1_mm*1000)/1000/1000;
    dy_m = std::round(dx0_mm*1000)/1000/1000;
    dz_m = std::round(dz_mm*1000)/1000/1000;

    //std::cout << dx_m << "," << dy_m << "," << dz_m << std::endl;

    cv::imshow("obj0",orig0);
    cv::imshow("obj1",orig1);

    cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/rj_tocke0.png",orig0);
    cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/rj_tocke1.png",orig1);

    cv::waitKey(0);
    cv::destroyAllWindows();


}

void boxovi_tocke_orient (float &rx_r, float &ry_r, float &rz_r ){
    cv::Mat objekti0 = cv::imread("/home/pg220823/Desktop/dipl/slike_seg/slika_seg_o0.png"); // a0(adaptivno) , o0(otsu)
    std::vector<cv::Point2f> vrhovi0 = vrhoviboxa(objekti0);
    cv::Mat orig0 = cv::imread("/home/pg220823/Desktop/dipl/slike_seg/slika_cam0.png");
    for (auto& tocka : vrhovi0){
        tocka.x += static_cast<float>(gl0.x);
        tocka.y += static_cast<float>(gl0.y);
    }
    for(int i = 0; i < 4; i++){
        cv::line(orig0,vrhovi0[i],vrhovi0[(i+1) % 4], cv::Scalar(255,0,0),1);
    }

    cv::Mat objekti1 = cv::imread("/home/pg220823/Desktop/dipl/slike_seg/slika_seg_o1.png");// a1(adaptivno) , o1(otsu)
    std::vector<cv::Point2f> vrhovi1 = vrhoviboxa(objekti1);
    cv::Mat orig1 = cv::imread("/home/pg220823/Desktop/dipl/slike_seg/slika_cam1.png");
    for (auto& tocka : vrhovi1){
        tocka.x += static_cast<float>(gl1.x);
        tocka.y += static_cast<float>(gl1.y);
    }
    for(int i = 0; i < 4; i++){
        cv::line(orig1,vrhovi1[i],vrhovi1[(i+1) % 4], cv::Scalar(255,0,0),1);
    }

    cv::Point2f tocka_osi0,tocka_vrh0,tocka_osi1,tocka_vrh1;
    tocke_osi_alata(vrhovi0,tocka_osi0,tocka_vrh0);
    cv::circle(orig0,tocka_osi0,5,cv::Scalar(255,0,0),-1);
    cv::circle(orig0,tocka_vrh0,5,cv::Scalar(0,0,255),-1);

    tocke_osi_alata(vrhovi1,tocka_osi1,tocka_vrh1);
    cv::circle(orig1,tocka_osi1,5,cv::Scalar(255,0,0),-1);
    cv::circle(orig1,tocka_vrh1,5,cv::Scalar(0,0,255),-1);


    //trokut u pixelima

    float x0 = cv::norm(tocka_vrh0.x - tocka_osi0.x);// x0 dio trokuta
    float y0 = cv::norm(tocka_vrh0.y - tocka_osi0.y);// y0 dio trokuta
    float x1 = cv::norm(tocka_vrh1.x - tocka_osi1.x);// x1 dio trokuta
    float y1 = cv::norm(tocka_vrh1.y - tocka_osi1.y);// y1 dio trokuta

    // definiranje gibanja



    cv::line(orig0,tocka_vrh0, cv::Point(tocka_osi0.x, tocka_vrh0.y),cv::Scalar(255,255,255),1);
    cv::line(orig0,cv::Point(tocka_osi0.x, tocka_vrh0.y), tocka_osi0,cv::Scalar(255,255,255),1);

    cv::line(orig1,tocka_vrh1, cv::Point(tocka_osi1.x, tocka_vrh1.y),cv::Scalar(255,255,255),1);
    cv::line(orig1,cv::Point(tocka_osi1.x, tocka_vrh1.y), tocka_osi1,cv::Scalar(255,255,255),1);

    //konverzija ofsseta u mm

    rx_r = std::atan(x0/y0);
    ry_r = std::atan(x1/y1);
    rz_r = 0.0;

    if(tocka_vrh0.x > tocka_osi0.x){
        rx_r = rx_r;
    }
    else{
        rx_r = -rx_r;
    }
    if(tocka_vrh1.x > tocka_osi1.x){
        ry_r = ry_r;
    }
    else{
        ry_r = -ry_r;
    }

    cv::imshow("obj0",orig0);
    cv::imshow("obj1",orig1);

    cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/rj_tocke0_or.png",orig0);
    cv::imwrite("/home/pg220823/Desktop/dipl/slike_seg/rj_tocke1_or.png",orig1);

    cv::waitKey(0);
    cv::destroyAllWindows();


}

void izvrsavanje(){
    snimanje_slike();
    segmentacija();
    boxovi_tocke(dx,dy,dz);
}

void izvrsavanje_orient(){
    snimanje_slike();
    segmentacija();
    boxovi_tocke_orient(rx_r,ry_r,rz_r);
}

std::vector<double> ciscenje_poze(const std::string& messagep){
    std::vector<double> rezultat;

    size_t start = messagep.find('[');
    size_t end = messagep.find(']');
    if(start == std::string::npos || end == std::string::npos || start >= end){
        throw std::invalid_argument("krivi format poruke");
    }

    std::string brojevi = messagep.substr(start + 1, end - start - 1);
    std::istringstream stream(brojevi);
    std::string dio;
    while(std::getline(stream,dio,',')){
        dio.erase(std::remove_if(dio.begin(),dio.end(),::isspace),dio.end());

        rezultat.push_back(std::stod(dio));
    }

    return rezultat;
}

Eigen::Matrix4d poza_matrica_transformacije (const std::vector<double>& poza){
    Eigen::Matrix4d mat_transformacije = Eigen::Matrix4d::Identity();

    mat_transformacije(0,3) = poza[0];
    mat_transformacije(1,3) = poza[1];
    mat_transformacije(2,3) = poza[2];

    double theta = std::sqrt(poza[3]*poza[3]+poza[4]*poza[4]+poza[5]*poza[5]);
    if(theta > 1e-6){
        Eigen::Vector3d os(poza[3]/theta, poza[4]/theta, poza[5]/theta);
        Eigen::Matrix3d rot;
        rot << 0,-os.z(),os.y(),
                os.z(),0,-os.x(),
                -os.y(),os.x(),0;
        Eigen::Matrix3d rotacija = Eigen::Matrix3d::Identity()+
                                    std::sin(theta)*rot +
                                    (1.0-std::cos(theta))*rot*rot;
        mat_transformacije.block<3,3>(0,0) = rotacija;
    }

    return mat_transformacije;
}

Eigen::Vector3d TCP_kalibracija(const std::vector<Eigen::Matrix4d>& mat_transformacija){
    size_t n = mat_transformacija.size();

    if(n < 3){
        std::cerr << "Za kalibraciju potrebne barem 3 poze" << std::endl;
    }

    Eigen::MatrixXd R_mtx;
    Eigen::VectorXd t_vec;

    for(size_t i = 0; i < n; ++i){
        for(size_t j = 0; j < n; ++j){
            if(i != j){
                Eigen::Matrix3d delta_rot = mat_transformacija[i].block<3,3>(0,0) - mat_transformacija[j].block<3,3>(0,0);
                R_mtx.conservativeResize(R_mtx.rows() + 3, 3);
                R_mtx.bottomRows(3) = delta_rot;

                Eigen::Vector3d delta_trans = mat_transformacija[j].block<3,1>(0,3) - mat_transformacija[i].block<3,1>(0,3);
                t_vec.conservativeResize(t_vec.size() + 3);
                t_vec.tail(3) = -delta_trans;
            }
        }
    }

    Eigen::Vector3d TCP = R_mtx.colPivHouseholderQr().solve(t_vec);

    std::cout << "kalibrirani TCP: " << TCP.transpose() <<std::endl;

    //devijacije

    std::vector<Eigen::Vector3d> tcp_pozicije;
    Eigen::Matrix4d transformacija_alat = Eigen::Matrix4d::Identity();
    transformacija_alat.block<3,1>(0,3) = TCP;

    for(const auto& transformacija : mat_transformacija){
        Eigen::Matrix4d tcp_transformacija = transformacija * transformacija_alat;
        tcp_pozicije.push_back(tcp_transformacija.block<3,1>(0,3));
    }

    Eigen::Vector3d tcp_avg = Eigen::Vector3d::Zero();
    for(const auto& tcp : tcp_pozicije){
        tcp_avg += tcp;
    }

    tcp_avg /= tcp_pozicije.size();

    std::cout << "Srednji TCP: " << tcp_avg.transpose() << std::endl;

    double max_devijacija = 0.0;

    for(const auto& tcp : tcp_pozicije){
        double devijacija = (tcp - tcp_avg).norm();
        max_devijacija = std::max(max_devijacija, devijacija);
    }

    std::cout << "maksimalna devijacija: " << max_devijacija << std::endl;

    return TCP;
}


void socket_server_kom(){
    int server_fd, new_socket;
    struct sockaddr_in adresa;
    int opt = 1;
    int addrlen = sizeof(adresa);
    const int PORT = 8080;

    if((server_fd = socket(AF_INET,SOCK_STREAM,0)) == 0 ){
        perror("izrada socketa nije uspjela");
        exit(EXIT_FAILURE);
    }

    if(setsockopt(server_fd,SOL_SOCKET,SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))){
        perror("socketopt greska");
        exit(EXIT_FAILURE);
    }

    adresa.sin_family = AF_INET;
    adresa.sin_addr.s_addr = INADDR_ANY;
    adresa.sin_port = htons(PORT);

    if(bind(server_fd,(struct sockaddr *)&adresa, sizeof(adresa)) < 0 ){
        perror("bind nije uspio");
        exit(EXIT_FAILURE);
    }

    if(listen(server_fd,3) < 0){
        perror("slusanje neuspjesno");
        exit(EXIT_FAILURE);
    }

    std::cout << "Socket server ceka poruke..." << std::endl;


    while(running){
        if((new_socket = accept(server_fd,(struct sockaddr *)&adresa, (socklen_t*)&addrlen)) < 0 ){
            perror("prihvacanje neuspjesno");
            exit(EXIT_FAILURE);
        }

        while(true){
            char buffer[1024] = {0};
            read(new_socket,buffer,1024);

            std::string message(buffer);
            std::cout << "primljena poruka: " << message << std::endl;

            if(message == "robot_ready"){
                std::cout << "robot na poziciji. pokretanje sustava..." << std::endl;

                std::string response_formats = "string\n";
                send(new_socket,response_formats.c_str(),response_formats.size(),0);
                std::cout<<"šalje se string"<<std::endl;

                std::string response_kreni = "kreni\n";
                send(new_socket,response_kreni.c_str(),response_kreni.size(),0);
                izvrsavanje();

                std::string response_formata = "ascii\n";
                send(new_socket,response_formata.c_str(),response_formata.size(),0);
                std::cout<<"šalje se ascii"<<std::endl;

                std::string response = "("+std::to_string(dx)+","+std::to_string(dy)+","+std::to_string(dz)+",0.0,0.0,0.0)\n";
                send(new_socket, response.c_str(), response.size(), 0);
                std::cout << "poslana greska: " + response << std::endl;

            }
            if(message == "robot_finished_movement"){

                if(fabs(dx) < 0.0005 & fabs(dy) < 0.0005 & fabs(dz) < 0.0005){
                    std::cout<< "greske zadovoljavajuce"<<std::endl;

                    std::string response_formats = "string\n";
                    send(new_socket,response_formats.c_str(),response_formats.size(),0);
                    std::cout<<"šalje se string"<<std::endl;

                    std::string response_kal_tocka = "spremi\n";
                    send(new_socket,response_kal_tocka.c_str(),response_kal_tocka.size(),0);
                    std::cout<<"poslano spremanje i promjena orijentacije"<<std::endl;
                }
                else{
                    std::cout << "robot na poziciji. pokretanje sustava..." << std::endl;

                    std::string response_formats = "string\n";
                    send(new_socket,response_formats.c_str(),response_formats.size(),0);
                    std::cout<<"šalje se string"<<std::endl;

                    std::string response_kreni = "kreni\n";
                    send(new_socket,response_kreni.c_str(),response_kreni.size(),0);
                    izvrsavanje();

                    std::string response_formata = "ascii\n";
                    send(new_socket,response_formata.c_str(),response_formata.size(),0);
                    std::cout<<"šalje se ascii"<<std::endl;

                    std::string response = "("+std::to_string(dx)+","+std::to_string(dy)+","+std::to_string(dz)+",0.0,0.0,0.0)\n";
                    send(new_socket, response.c_str(), response.size(), 0);
                    std::cout << "poslana greska: " + response << std::endl;
                }
            }

            if(message == "robot_finished"){
                running = false;
                break;
            }

            if(message == "kalibracijska_poza"){
                char bufferp[1024] = {0};
                read(new_socket,bufferp,1024);

                std::string messagep(bufferp);
                std::cout << "primljena poza: " << messagep << std::endl;
                std::vector<double> poza = ciscenje_poze(messagep);
                Eigen::Matrix4d poza_m = poza_matrica_transformacije(poza);
                kalibracijske_mat_trans.push_back(poza_m);
                std::cout << "poza dodana i pretvorena u matricu transformacije..." << std::endl;

            }

            if(message == "zapocni_kalibraciju"){
                Eigen::Vector3d TCP = TCP_kalibracija(kalibracijske_mat_trans);
                //std::cout << "TCP:" << TCP.x() << "," << TCP.y() << "," << TCP.z() << std::endl;

                std::string response_formata = "ascii\n";
                send(new_socket,response_formata.c_str(),response_formata.size(),0);
                std::cout<<"šalje se ascii"<<std::endl;

                std::string response_mm = "("+std::to_string(std::round((-TCP.x()*1000*1000))/1000)+","
                                        +std::to_string(std::round((-TCP.y()*1000*1000))/1000)+","
                                        +std::to_string(std::round((-TCP.z()*1000*1000))/1000)+
                                        ",0.0,0.0,0.0)\n";
                std::string response = "("+ std::to_string(-TCP.x()) + "," + std::to_string(-TCP.y()) +
                                            "," + std::to_string(-TCP.z()) +  ",0.0,0.0,0.0)\n";
                send(new_socket, response.c_str(), response.size(), 0);
                std::cout << "poslani kalibrirani TCP (u milimetrima): " + response_mm << std::endl;
            }
            if(message == "robot_ready_orient"){
                std::cout << "robot spreman za promjenu orijentacije. pokretanje sustava..." << std::endl;
                izvrsavanje_orient();

                std::string response_formata = "ascii\n";
                send(new_socket,response_formata.c_str(),response_formata.size(),0);
                std::cout<<"šalje se ascii"<<std::endl;

                std::string response = "(0.0,0.0,0.0,"+std::to_string(rx_r)+","+std::to_string(ry_r)+","+std::to_string(rz_r)+")\n";
                send(new_socket, response.c_str(), response.size(), 0);
                std::cout << "poslana promjena orijnetacije: " + response << std::endl;

                std::string response_formats = "string\n";
                send(new_socket,response_formats.c_str(),response_formats.size(),0);
                std::cout<<"šalje se string"<<std::endl;

                std::string response_kreni = "kraj\n";
                send(new_socket,response_kreni.c_str(),response_kreni.size(),0);

            }
        }


        close(new_socket);
    }

    close(server_fd);
    std::cout << "kraj" << std::endl;
}


int main(){

    std::thread serverThread(socket_server_kom);
    serverThread.join();
    //izvrsavanje();
    return 0;
}
