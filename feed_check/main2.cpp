#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;
int kamera = 0 ;


int main() {
    std::string ime_mape = "kalibracijske_slike";

    std::string pipeline1 = "nvarguscamerasrc sensor-id = " + to_string(kamera) + " ! video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
    cv::VideoCapture camera(pipeline1, cv::CAP_GSTREAMER);
    int brojac = 0;
    if(!camera.isOpened()){
        std::cerr << "error : nemožem" << std::endl;
        return -1;
    }

    cv::Mat frame;

    while(true){
        camera >> frame;
        if (frame.empty()){
            cerr<<"empty frame"<<endl;
        break;
        }


    cv::imshow("camfeed", frame);
    char key = cv::waitKey(1);
    if(key == 'q'){
        break;
    } else if (key == ' '){
        std::ostringstream datoteka;
        datoteka << ime_mape << "/kalib_test_kamera" << kamera << brojac << ".jpg";
        cv::imwrite(datoteka.str(), frame);
        brojac++;
        std::cout << "slika snimljena" << std::endl;
    }

   }

    camera.release();
    destroyAllWindows();
    return 0;
}
