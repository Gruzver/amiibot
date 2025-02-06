#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace dnn;
using namespace std;

class FacePublisher : public rclcpp::Node
{
public:
    FacePublisher() : Node("face_publisher")
    {
      // 1. Inicialización de la cámara
      video.open(2);
      video.set(cv::CAP_PROP_FRAME_WIDTH, 640);
      video.set(cv::CAP_PROP_FRAME_HEIGHT, 480);  
      if (!video.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Error: No se pudo abrir la cámara.");
        return;
      }

      string faceModelPath = "/home/gr/amii_ws/src/face_detection/models/opencv_face_detector_uint8.pb";
      string faceConfigPath = "/home/gr/amii_ws/src/face_detection/models/opencv_face_detector.pbtxt";
      faceNet = readNet(faceModelPath, faceConfigPath);

      if (faceNet.empty()) {
          RCLCPP_ERROR(this->get_logger(), "Error: No se pudo cargar el modelo.");
          return;
      }

      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("face_coordinates", 10);

      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&FacePublisher::detect_faces_periodically, this)
      );
    }

private:


    void detect_faces_periodically() {
        Mat frame;
        video >> frame;  // Captura un fotograma

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error: No se pudo capturar el fotograma.");
            return;
        }

        // Realizar la detección de rostros
        vector<int> coordinates = detect_faces_in_frame(frame, faceNet);
        
        // Publicar las coordenadas si se detecta algún rostro
        if (!coordinates.empty()) {
            std_msgs::msg::Float32MultiArray msg;
            msg.data = std::vector<float>(coordinates.begin(), coordinates.end());
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published coordinates: [%f, %f]", msg.data[0], msg.data[1]);
        }
    }

    // Función que realiza la detección de rostro en un fotograma
    std::vector<int> detect_faces_in_frame(const cv::Mat &frame, cv::dnn::dnn4_v20211004::Net &faceNet)
 {
        vector<int> coordinates;

        // Crear un blob y pasar al modelo de detección de rostros
        Mat resized_frame;
        resize(frame, resized_frame, Size(640, 480));  // Reducción a 640x480
        Mat blob = blobFromImage(resized_frame, 1.0, Size(300, 300), Scalar(104.0, 177.0, 123.0), false, false);
        faceNet.setInput(blob);
        Mat detections = faceNet.forward();
        
        float* data = (float*)detections.ptr<float>(0);  // Tensor de salida
        for (int i = 0; i < detections.size[2]; i++) {
            float confidence = data[i * 7 + 2];
            if (confidence > 0.5) {  // Si la confianza es mayor que 0.5
                int x1 = static_cast<int>(data[i * 7 + 3] * frame.cols);
                int y1 = static_cast<int>(data[i * 7 + 4] * frame.rows);
                int x2 = static_cast<int>(data[i * 7 + 5] * frame.cols);
                int y2 = static_cast<int>(data[i * 7 + 6] * frame.rows);
                
                // Agregar las coordenadas de la cara detectada
                coordinates.push_back(x1);
                coordinates.push_back(y1);
                coordinates.push_back(x2);
                coordinates.push_back(y2);
            }
        }

        return coordinates;
    }

    cv::VideoCapture video;
    cv::dnn::Net faceNet;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FacePublisher>());
    rclcpp::shutdown();
    return 0;
}