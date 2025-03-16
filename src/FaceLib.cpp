#include <jni.h>
#include <opencv2/opencv.hpp>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>
#include <vector>
#include <iostream>
#include <dlib/image_processing.h>
#include <dlib/geometry.h>

using namespace cv;
using namespace dlib;
using namespace std;

extern "C" {

// JNI 方法: 人脸检测与特征点识别
    JNIEXPORT jobjectArray JNICALL Java_FaceProcessor_detectLandmarks(JNIEnv* env, jobject obj, jstring imagePath) {
        const char* path = env->GetStringUTFChars(imagePath, nullptr);
        Mat img = imread(path);
        env->ReleaseStringUTFChars(imagePath, path);

        if (img.empty()) {
            cerr << "无法加载图像: " << path << endl;
            return nullptr;
        }

        cvtColor(img, img, COLOR_BGR2GRAY);

        frontal_face_detector detector = get_frontal_face_detector();
        shape_predictor predictor;
        deserialize("data/shape_predictor_68_face_landmarks.dat") >> predictor;

        array2d<unsigned char> dlibImg;
        assign_image(dlibImg, cv_image<unsigned char>(img));

        // 使用 dlib::rectangle 代替 rectangle
        std::vector<dlib::rectangle> faces = detector(dlibImg);
        if (faces.empty()) {
            return nullptr;
        }

        jobjectArray result = env->NewObjectArray(faces.size(), env->FindClass("[I"), nullptr);

        for (size_t i = 0; i < faces.size(); ++i) {
            full_object_detection shape = predictor(dlibImg, faces[i]);

            jintArray faceLandmarks = env->NewIntArray(68 * 2);
            jint landmarks[68 * 2];

            for (int j = 0; j < 68; ++j) {
                landmarks[2 * j] = shape.part(j).x();
                landmarks[2 * j + 1] = shape.part(j).y();
                circle(img,Point(landmarks[2*j],landmarks[2*j+1]),5,(0,0,138),-1);
            }
            int cx =(int )((landmarks[2*19]+landmarks[2*24]+landmarks[2*66])/3);
            int cy =(int )((landmarks[2*19+1]+landmarks[2*24+1]+landmarks[2*66+1])/3);
            circle(img,Point(cx,cy),5,(0,0,138),-1);
            env->SetIntArrayRegion(faceLandmarks, 0, 68 * 2, landmarks);
            env->SetObjectArrayElement(result, i, faceLandmarks);
            env->DeleteLocalRef(faceLandmarks);
        }
        imwrite("tagc.jpg",img);
        return result;
    }

// 夸张变形
    void exaggerate(Mat& img1, Mat& img, Point center, int radius) {
        int r = radius;
        double k = 0.8;
        double pk = 0.5;

        for (int i = max(center.x - r, 0); i < min(center.x + r, img.cols); i++) {
            for (int j = max(center.y - r, 0); j < min(center.y + r, img.rows); j++) {
                double d_square = (i - center.x) * (i - center.x) + (j - center.y) * (j - center.y);
                if (d_square > r * r)
                    continue;

                if (d_square <= (k * r) * (k * r)) {
                    int ux1 = int(float(center.x) + float(i - center.x) / k * pk);
                    if (i-center.x!=0) int ux2=ux1+(i-center.x)/ abs(i-center.x);
                    else int ux2 = ux1;
                    int uy1 = int(float(center.y) + float(j - center.y) / k * pk);
                    if (j-center.y!=0) int uy2=uy1+(j-center.y)/ abs(j-center.y);
                    else int uy2 = uy1;
                    double dx = (float )(ux1-center.x)/k;
                    img1.at<Vec3b>(j, i) = img.at<Vec3b>(uy1, ux1);
                } else {
                    double dx = float(i - center.x) / sqrt(d_square) * float(r);
                    double dy = float(j - center.y) / sqrt(d_square) * float(r);
                    int ux = int(float(center.x) + (float(i - center.x) - dx * k) / (1 - k) * pk + dx * pk);
                    int uy = int(float(center.y) + (float(j - center.y) - dy * k) / (1 - k) * pk + dy * pk);
                    img1.at<Vec3b>(j, i) = img.at<Vec3b>(uy, ux);
                }
            }
        }
    }

    // JNI 方法: 夸张变形处理
    JNIEXPORT void JNICALL Java_FaceProcessor_applyExaggeration(JNIEnv* env, jobject obj, jstring imagePath, jstring outputPath,jint n) {
        const char* path = env->GetStringUTFChars(imagePath, nullptr);
        Mat img = imread(path);
        Mat img1 = img.clone();
        Mat img2 = img.clone();
        env->ReleaseStringUTFChars(imagePath, path);
        cvtColor(img, img, COLOR_BGR2GRAY);
        frontal_face_detector detector = get_frontal_face_detector();
        shape_predictor predictor;
        deserialize("data/shape_predictor_68_face_landmarks.dat") >> predictor;

        array2d<unsigned char> dlibImg;
        assign_image(dlibImg, cv_image<unsigned char>(img));

        // 使用 dlib::rectangle 代替 rectangle
        std::vector<dlib::rectangle> faces = detector(dlibImg);

        for (size_t i = 0; i < faces.size(); i++) {
            full_object_detection shape = predictor(dlibImg, faces[i]);
            jintArray faceLandmarks = env->NewIntArray(68 * 2);
            jint landmarks[68 * 2];

            for (int j = 0; j < 68; ++j) {
                landmarks[2 * j] = shape.part(j).x();
                landmarks[2 * j + 1] = shape.part(j).y();
            }
            int x1 = landmarks[2*17], y1 = landmarks[2*17+1];
            int x2 = landmarks[2*26], y2 = landmarks[2*26+1];
            int x3 = landmarks[2*62], y3 = landmarks[2*62+1];
            Point center((x1 + x2 + x3) / 3, (y1 + y2 + y3) / 3);
            int radius = int(sqrt((x1 - center.x) * (x1 - center.x) + (y1 - center.y) * (y1 - center.y)));
            exaggerate(img1, img2, center, radius * 0.8);
        }
    const char* output = env->GetStringUTFChars(outputPath, nullptr);
    imwrite(output, img1);
    env->ReleaseStringUTFChars(outputPath, output);
    }

}
