#include <deque>
#include <mutex>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <cstring>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h> // Include this header for getaddrinfo and gai_strerror
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket close
#endif

//I know this is stupid. lol
#define DEBUG
#define NO_DETECT

const size_t TARGET_FPS = 5;        // Target frame rate for capture
const size_t BUFFER_DURATION_SECONDS = 5; // Duration of the buffer in seconds


class UdpSocket {
public:
    UdpSocket();
    ~UdpSocket();

    bool openSocket();
    bool sendTo(const std::string& address, unsigned short port, const char* buffer, int len);
    void closeSocket();

private:
    SOCKET sockfd;
    struct sockaddr_in serverAddr;
};

UdpSocket::UdpSocket() : sockfd(INVALID_SOCKET) {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

UdpSocket::~UdpSocket() {
    closeSocket();
#ifdef _WIN32
    WSACleanup();
#endif
}

bool UdpSocket::openSocket() {
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    return sockfd != INVALID_SOCKET;
}

bool UdpSocket::sendTo(const std::string& address, unsigned short port, const char* buffer, int len) {
    if (sockfd == INVALID_SOCKET) return false;

    struct addrinfo hints, *res;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    std::string portStr = std::to_string(port);

    int status = getaddrinfo(address.c_str(), portStr.c_str(), &hints, &res);
    if (status != 0) {
        std::cerr << "getaddrinfo error: " << gai_strerror(status) << std::endl;
        return false;
    }

    // Note: Cast len to int to avoid warning
    bool sendSuccess = sendto(sockfd, buffer, static_cast<int>(len), 0, res->ai_addr, res->ai_addrlen) != SOCKET_ERROR;

    freeaddrinfo(res);

    return sendSuccess;
}

void UdpSocket::closeSocket() {
    if (sockfd != INVALID_SOCKET) {
        closesocket(sockfd);
        sockfd = INVALID_SOCKET;
    }
}

class FrameBuffer {
public:
    FrameBuffer(size_t maxFrames, UdpSocket& udpSocket) 
        : maxFrames(maxFrames), udpSocket(udpSocket) {}

    void addFrame(const cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(mutex);
        if (buffer.size() >= maxFrames) {
            // Send the oldest frame
            sendFrontFrame();
        }
        buffer.push_back(frame.clone());

        //std::cout << "Frame Buffer Size: " << buffer.size() << " of " << maxFrames << std::endl;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex);
        buffer.clear();
    }

private:
    void sendFrontFrame() {
        if (!buffer.empty()) {
            cv::Mat frontFrame = buffer.front();
            buffer.pop_front();

            std::vector<uchar> encodedBuffer;
            cv::imencode(".jpg", frontFrame, encodedBuffer);
            udpSocket.sendTo("127.0.0.1", 12345, 
                reinterpret_cast<const char*>(encodedBuffer.data()), 
                static_cast<int>(encodedBuffer.size()));

            #ifdef DEBUG
                cv::imshow("UDP Feed", frontFrame);
                cv::waitKey(1); // For display refresh
            #endif
        }
    }

    std::deque<cv::Mat> buffer;
    size_t maxFrames;
    UdpSocket& udpSocket;
    std::mutex mutex;
};

int main(int argc, char** argv)
{
    cv::VideoCapture cap(0);
    
    cv::CascadeClassifier frontal_face_cascade;
    cv::CascadeClassifier profile_face_cascade;

    frontal_face_cascade.load("C:/opencv/sources/data/haarcascades/haarcascade_frontalface_default.xml");
    profile_face_cascade.load("C:/opencv/sources/data/haarcascades/haarcascade_profileface.xml");

    UdpSocket udpSocket;
    if (!udpSocket.openSocket()) {
        std::cerr << "Failed to open UDP socket." << std::endl;
        return -1;
    }

    FrameBuffer frameBuffer(TARGET_FPS * BUFFER_DURATION_SECONDS, udpSocket);

    if (!cap.isOpened()) {
        std::cerr << "Error opening video capture\n";
        return -1;
    }

    cap.set(cv::CAP_PROP_FPS, TARGET_FPS);
    double actualFPS = cap.get(cv::CAP_PROP_FPS);
    std::cout << "Actual FPS: " << actualFPS << std::endl;

    // Calculate the frame period in milliseconds based on the actual FPS
    const double frameDurationMs = 1000.0 / actualFPS;
    const std::chrono::milliseconds framePeriod(static_cast<int>(frameDurationMs));

    std::vector<cv::Rect> faces;
    cv::Mat frame;
    bool faceDetected = false;

    // Variables for FPS calculation
    double fps = 0.0;
    int frameCount = 0;
    auto lastTime = std::chrono::steady_clock::now();

    // Variables for elapsed time measurement
    auto loopStartTime = std::chrono::steady_clock::now();
    auto lastLoopTime = loopStartTime;

    while (true) {
        // Mark the start of the loop
        loopStartTime = std::chrono::steady_clock::now();

        cap >> frame;
        
        frontal_face_cascade.detectMultiScale(frame, faces);
        if (faces.empty()) {
            // If no frontal faces detected, try to detect profile faces
            profile_face_cascade.detectMultiScale(frame, faces);
        }

#ifndef NO_DETECT
        if (!faces.empty()) {
            faceDetected = true;
            std::cout << "Face detected, stopping stream.\n";
            frameBuffer.clear();
        } else {
            if (faceDetected) {
                std::cout << "No face detected, resuming stream.\n";
                faceDetected = false;
            }
#endif
            frameBuffer.addFrame(frame);

            #ifdef DEBUG
                // Display the frame for debugging
                cv::imshow("Add Frame Feed", frame);
            #endif
#ifndef NO_DETECT
        }
#endif

        if (cv::waitKey(30) >= 0) break;

        // FPS calculation
        frameCount++;
        auto currentTime = std::chrono::steady_clock::now();
        auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime);
        if (timeDiff.count() >= 1000) { // Update FPS every second
            fps = frameCount / (timeDiff.count() / 1000.0);
            frameCount = 0;
            lastTime = currentTime;
            std::cout << "FPS: " << fps << std::endl;
        }

        // Elapsed time for each loop
        auto loopEndTime = std::chrono::steady_clock::now();
        auto loopElapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(loopEndTime - lastLoopTime);
        lastLoopTime = loopEndTime;
        //std::cout << "Elapsed Time per Loop: " << loopElapsedTime.count() << " ms" << std::endl;
    }

    return 0;
}
