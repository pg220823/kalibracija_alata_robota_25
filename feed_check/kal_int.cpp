#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

int brojac = 0;
int kamera = 1;

double kalibracija_func (const std::vector<std::string>& dir_slika, cv::Size dim_kal_ploce, float vel_kvadrata, cv::Mat& mat_kamere , cv::Mat& koef_distorzije) {

    std::vector<std::vector<cv::Point2f>> tocke_slika;
    std::vector<std::vector<cv::Point3f>> tocke_objekta;
    //ploca u 3d
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < dim_kal_ploce.height; i++){
        for(int j = 0; j < dim_kal_ploce.width; j++ ){
            obj.emplace_back(j * vel_kvadrata, i * vel_kvadrata, 0);
        }
    }
    //kutovi na slici
    for (const auto& dir : dir_slika){
        cv::Mat kal_slika = cv::imread(dir);
        cv::Mat gray;
        cv::cvtColor(kal_slika,gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> kutovi;
        bool nadjeni = cv::findChessboardCorners(gray, dim_kal_ploce, kutovi);
        cv::drawChessboardCorners(kal_slika, dim_kal_ploce, kutovi, nadjeni);
        cv::imshow("detected corners", kal_slika);
        cv::waitKey(0);

        if (nadjeni){
            cv::cornerSubPix(gray, kutovi, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 30, 0.001));
            tocke_slika.push_back(kutovi);
            tocke_objekta.push_back(obj);
        }

    }
    cv::Mat prva_slika = cv::imread(dir_slika[0]);
    cv::Size vel_slike = prva_slika.size();
    std::vector<cv::Mat> rvecs, tvecs;

    double err = cv::calibrateCamera(tocke_objekta, tocke_slika, vel_slike, mat_kamere, koef_distorzije,rvecs, tvecs);
    return err;
}

void spremanje_parametara (const std::string ime_txt , const cv::Mat& mat_kamere, const cv::Mat& koef_distorzije){
    std::ofstream file(ime_txt);
    if (file.is_open()){
        file << "matrica kamere: \n" << mat_kamere << "\n\n";
        file << "koeficjenti distorzije: \n" << koef_distorzije << "\n";
        file.close();
        std::cout << "parametri spremljeni na:" << ime_txt << std::endl;
    }
    else {
        std::cerr << "greška pri otvaranju txt-a" << std::endl;
    }
}

int main() {

    cv::Size dim_kal_ploce(8,6);
    float vel_kvadrata = 25.0f;
    std::vector<std::string> dir_slika;
    while(true){
        std::string put_slika = "/home/pg220823/Desktop/build-feed_check-Desktop-Default/kalibracijske_slike/kalib_kamera"+to_string(kamera)+to_string(brojac)+".jpg";
        cv::Mat ks = cv::imread(put_slika);
        if(!ks.empty()){
            dir_slika.push_back(put_slika);
            std::cout << "dodana slika: " << put_slika << std::endl;
        }
        else {
            break;
        }
        brojac++;
    }
    cv::Mat mat_kamere , koef_distorzije;

    double rerr = kalibracija_func(dir_slika, dim_kal_ploce, vel_kvadrata, mat_kamere, koef_distorzije);
    std::cout << "final reprojection error:" << rerr << std::endl;

    spremanje_parametara("intrinzicni_parametri"+to_string(kamera)+".txt", mat_kamere, koef_distorzije);
    cv::Mat usl1;
    cv::Mat sl1;
    if (kamera == 0){
        cv::Mat sl1 = cv::imread("/home/pg220823/Desktop/build-feed_check-Desktop-Default/kalibracijske_slike/kalib_kamera013.jpg");
        cv::undistort(sl1, usl1, mat_kamere, koef_distorzije);
        cv::imshow("nekalibrirana",sl1);
        cv::imshow("kalibrirana",usl1);
        cv::waitKey(0);
    }
    else if (kamera == 1){
        cv::Mat sl1 = cv::imread("/home/pg220823/Desktop/build-feed_check-Desktop-Default/kalibracijske_slike/kalib_kamera115.jpg");
        cv::undistort(sl1, usl1, mat_kamere, koef_distorzije);
        cv::imshow("nekalibrirana",sl1);
        cv::imshow("kalibrirana",usl1);
        cv::waitKey(0);
    }


    return 0;

}
