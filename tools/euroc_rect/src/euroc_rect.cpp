#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <csignal>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

rosbag::Bag bagr, bagw;

void signal_handler(int signal) {
    bagr.close();
}

int main(int argc, char** argv) {
    if (argc <= 3) {
        printf("need more params!\n");
        return -1;
    }

    std::signal(SIGINT, signal_handler);
    cv::FileStorage fs(argv[3], cv::FileStorage::READ);
    cv::Mat D1, D2, P1, P2, R1, R2, K1, K2, Q;
    fs["LEFT.D"] >> D1;
    fs["LEFT.P"] >> P1;
    fs["LEFT.R"] >> R1;
    fs["LEFT.K"] >> K1;
    fs["RIGHT.D"] >> D2;
    fs["RIGHT.P"] >> P2;
    fs["RIGHT.R"] >> R2;
    fs["RIGHT.K"] >> K2;
    fs.release();
    if (!boost::filesystem::exists(argv[1])) {
        printf("origin bag doesn't exist.\n");
        return -1;
    }

    boost::filesystem::copy(argv[1], argv[2]);
    bagr.open(argv[1], rosbag::BagMode::Read);
    bagw.open(argv[2], rosbag::BagMode::Append);

    std::vector<std::string> topics;
    topics.push_back("/cam0/image_raw");
    topics.push_back("/cam1/image_raw");

    cv_bridge::CvImage cvimg;

    cv::Mat ml1, ml2, mr1, mr2;
    cv::initUndistortRectifyMap(K1, D1, R1, P1.rowRange(0, 3).colRange(0, 3), cv::Size(752, 480), CV_32FC1, ml1, ml2);
    cv::initUndistortRectifyMap(K2, D2, R2, P2.rowRange(0, 3).colRange(0, 3), cv::Size(752, 480), CV_32FC1, mr1, mr2);

    rosbag::View view(bagr, rosbag::TopicQuery(topics));

    int i = 0;
    BOOST_FOREACH (rosbag::MessageInstance const m, view) {
        std::string topic;
        sensor_msgs::ImageConstPtr p = m.instantiate<sensor_msgs::Image>();
        auto imgp = cv_bridge::toCvCopy(p);
        cv::Mat img_rect;
        cv::Mat *m1, *m2;

        if (m.getTopic() == "/cam0/image_raw") {
            topic = "/cam0/image_rect";
            m1 = &ml1;
            m2 = &ml2;
        }
        else {
            topic = "/cam1/image_rect";
            m1 = &mr1;
            m2 = &mr2;
        }
        if (i++ % 100 == 0)
            printf("idx: %d\n", i);
        cv::remap(imgp->image, img_rect, *m1, *m2, CV_INTER_LINEAR);
        cvimg.header = p->header;
        cvimg.encoding = p->encoding;
        cvimg.image = img_rect;
        sensor_msgs::Image outmsg;
        cvimg.toImageMsg(outmsg);
        bagw.write(topic, m.getTime(), outmsg);
    }
    bagw.close();
    bagr.close();

    return 0;
}
