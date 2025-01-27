#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/core.hpp>
#include <iostream>
int mx,my;
std::string boja = "zuta";
std::string alat = "2";
void onMouse(int event,int x, int y, int flags, void* userdata)
{
    if(event == cv::EVENT_MOUSEMOVE){
        mx =x;
        my=y;
    }

}

void onMousehsv(int event,int x, int y, int flags, void* userdata)
{
    if(event == cv::EVENT_LBUTTONDOWN){
        cv::Mat* rj = reinterpret_cast<cv::Mat*>(userdata);
        if (x >= 0 && y >= 0 && x < rj->cols && y <rj->rows){
            cv::Vec3b hsvpiksel = rj->at<cv::Vec3b>(x,y);
            int h = hsvpiksel[0];
            int s = hsvpiksel[1];
            int v = hsvpiksel[2];
            std::cout << "h: " << h << ", s: " << s << ", v: " << v << "\r";
            std::cout.flush();
        }
    }

}

int main1()
{
   std::string pipeline1 = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
   cv::VideoCapture cap1(pipeline1, cv::CAP_GSTREAMER);

    if (!cap1.isOpened() ){
        std::cerr << "Error : Could not open cameras" << std::endl;
        return -1;
    }

    cv::Mat frame1 ;
    while (true) {
        cap1 >> frame1;

        if(frame1.empty() ){
            break;
        }

        cv::namedWindow("Camera 1");
        cv::setMouseCallback("Camera 1",onMouse,nullptr);
        std::string posm = "(" + std::to_string(mx) + "),(" + std::to_string(my) + ")";
        cv::putText(frame1,posm,cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255),1);

        cv::Point gl(390,255); //gornje lijevo
        cv::Point dd(1245,985); //donje desno
        cv::Rect vp(gl,dd);
        cv::Mat rezano = frame1(vp).clone();

        cv::medianBlur(rezano,rezano,5);
        cv::Mat hsv , maska , rj;
        //cv::Mat maska2 , maska1; //za crvenu
        cv::cvtColor(rezano,hsv,cv::COLOR_BGR2HSV);

        /*//CRVENA----------------------------------
        cv::Scalar dc1(0,100,100);
        cv::Scalar gc1(10,255,255);
        cv::Scalar dc2(170,100,100);
        cv::Scalar gc2(180,255,255);
        cv::inRange(hsv,dc1,gc1,maska1);
        cv::inRange(hsv,dc2,gc2,maska2);
        cv::bitwise_or(maska1,maska2,maska);
        cv::bitwise_and(rezano,rezano,rj,maska);
        *///---------------------------------------

        /*//ZUTA----------------------------------
        cv::Scalar dzu(20,100,100);
        cv::Scalar gzu(30,255,255);
        cv::inRange(hsv,dzu,gzu,maska);
        cv::bitwise_and(rezano,rezano,rj,maska);
        *///-----------------------------------

        /*//PLAVA--------------------------------
        cv::Scalar dp(100,100,80);
        cv::Scalar gp(120,255,255);
        cv::inRange(hsv,dp,gp,maska);
        cv::bitwise_and(rezano,rezano,rj,maska);
        *///------------------------------------

        //ZELENA------------------------------
        cv::Scalar dze(70,30,30);
        cv::Scalar gze(85,255,255);
        cv::inRange(hsv,dze,gze,maska);
        cv::bitwise_and(rezano,rezano,rj,maska);
        //-----------------------------------

        /*//BIJELA-----------------------------
        cv::Scalar db(0,0,100);
        cv::Scalar gb(180,40,255);
        cv::inRange(hsv,db,gb,maska);
        cv::bitwise_and(rezano,rezano,rj,maska);
        //-----------------------------------*/


        cv::imshow("Camera 1", frame1);
        cv::imshow("Camera 1- interest", rezano);
        cv::setMouseCallback("Camera 1- interest",onMousehsv,&rezano);
        cv::imshow("segmentacija_boje",rj);

        if (cv::waitKey(1) == 27) {  // esc
            std::cout<<"proslo esc"<<std::endl;
                    break;
                }
        else if (cv::waitKey(30) == 115){ //s
            std::cout<<"proslo s"<<std::endl;
            cv::imwrite("/home/pg220823/Desktop/dipl/slike_pozadine/slika_"+boja+alat+".jpg",frame1);
            cv::imwrite("/home/pg220823/Desktop/dipl/slike_pozadine/slikai_"+boja+alat+".jpg",rezano);
            cv::imwrite("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_"+boja+alat+".jpg",rj);
            return 0;
            }
    }

    cap1.release();
    cv::destroyAllWindows();

    /*
    cv::Mat zu1= cv::imread("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_zuta1.jpg",cv::IMREAD_COLOR);
    cv::Mat ze1= cv::imread("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_zelena1.jpg",cv::IMREAD_COLOR);
    cv::Mat c1= cv::imread("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_crvena1.jpg",cv::IMREAD_COLOR);
    cv::Mat p1= cv::imread("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_plava1.jpg",cv::IMREAD_COLOR);
    cv::Mat b1= cv::imread("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_bijela1.jpg",cv::IMREAD_COLOR);
    std::vector<cv::Mat> im1 = {zu1,ze1,c1,p1,b1};
    cv::Mat alat1;
    cv::hconcat(im1,alat1);
    //cv::imshow("alat 1", alat1);

    cv::Mat zu2= cv::imread("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_zuta2.jpg",cv::IMREAD_COLOR);
    cv::Mat ze2= cv::imread("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_zelena2.jpg",cv::IMREAD_COLOR);
    cv::Mat c2= cv::imread("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_crvena2.jpg",cv::IMREAD_COLOR);
    cv::Mat p2= cv::imread("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_plava2.jpg",cv::IMREAD_COLOR);
    cv::Mat b2= cv::imread("/home/pg220823/Desktop/dipl/slike_pozadine/slikas_bijela2.jpg",cv::IMREAD_COLOR);
    std::vector<cv::Mat> im2 = {zu2,ze2,c2,p2,b2};
    cv::Mat alat2;
    cv::hconcat(im2,alat2);
    cv::imshow("alat 2", alat2);
    cv::Mat alat_1 , alat_2;
    cv::resize(alat1,alat_1, cv::Size(), 0.5,0.5,cv::INTER_LINEAR_EXACT);
    cv::resize(alat2,alat_2, cv::Size(), 0.5,0.5,cv::INTER_LINEAR_EXACT);
    cv::imshow("alat_1",alat_1);
    cv::imshow("alat_2",alat_2);
    cv::imwrite("/home/pg220823/Desktop/dipl/slike_pozadine/alat1.jpg",alat_1);
    cv::imwrite("/home/pg220823/Desktop/dipl/slike_pozadine/alat2.jpg",alat_2)
    */

    cv::waitKey(0) == 97; //a
    std::cout<<"proslo a"<<std::endl;
    return 0;

}
