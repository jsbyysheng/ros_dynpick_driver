#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Trigger.h>

#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <csignal>
#include <regex>

#define true                1
#define false               0

// TODO: Find out why and fix the multiple try approach
#define RESET_COMMAND_TRY   3       // It only works when send several times.

#define DATA_LENGTH 27
#define CALIB_DATA_LENGTH 46
#define HARDWARE_DATA_LENGTH 68

std::mutex m_;
std::condition_variable cv_;
int offset_reset_ = 0;
int downsample = 20;
sig_atomic_t volatile g_request_shutdown = 0;

int SetComAttr(int fdc)
    {
    int n;
    struct termios term{};
    // Set baud rate
    n = tcgetattr(fdc, &term);
    if (n < 0)
        goto over;

    bzero(&term, sizeof(term));

    term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
    term.c_iflag = IGNPAR;
    term.c_oflag = 0;
    term.c_lflag = 0;/*ICANON;*/

    term.c_cc[VINTR]    = 0;     /* Ctrl-c */
    term.c_cc[VQUIT]    = 0;     /* Ctrl-? */
    term.c_cc[VERASE]   = 0;     /* del */
    term.c_cc[VKILL]    = 0;     /* @ */
    term.c_cc[VEOF]     = 4;     /* Ctrl-d */
    term.c_cc[VTIME]    = 0;
    term.c_cc[VMIN]     = 0;
    term.c_cc[VSWTC]    = 0;     /* '?0' */
    term.c_cc[VSTART]   = 0;     /* Ctrl-q */
    term.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    term.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    term.c_cc[VEOL]     = 0;     /* '?0' */
    term.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    term.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    term.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    term.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    term.c_cc[VEOL2]    = 0;     /* '?0' */

//  tcflush(fdc, TCIFLUSH);
    n = tcsetattr(fdc, TCSANOW, &term);
over :
    return (n);
    }

bool offsetRequest(std_srvs::Trigger::Request  &req,
         std_srvs::Trigger::Response &res) {
    std::unique_lock<std::mutex> lock(m_);
    offset_reset_ = RESET_COMMAND_TRY;
    cv_.wait(lock, []{ return offset_reset_ <= 0; });
    lock.unlock();
    res.message = "Reset offset command was send " + std::to_string(RESET_COMMAND_TRY) + " times to the sensor.";
    res.success = true;
    return true;
}

bool clearSocket(const int& fdc, char* leftover){
    int len = 0;
    int c;
    int length = 255;
    while ( len < length ) {
        c = read(fdc, leftover+len, length-len);
        if (c > 0) {
            len += c;
            ROS_DEBUG("More data to clean up; n = %d (%d) ===", c, len);
        } else {
            ROS_DEBUG("No more data on socket");
            break;
        }
    }
    // This could actually check if data was received and may return on timeout with false
    return true;
}

bool readCharFromSocket(const int& fdc, const int& length, char* reply){
    int len = 0;
    int c;
    while ( len < length ) {
        c = read(fdc, reply + len, length - len);
        if (c > 0) {
            len += c;
        } else if (c == 0) {
            ROS_DEBUG("=== need to read more data ... n = %d (%d) ===", c, len);
            usleep(100); // polling @ 0.1 ms
        } else {
            ROS_WARN("read() error detected");
        }
    }
    // This could actually check if data was received and may return on timeout with false
    return true;
}

// Ref[0]: https://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
// Ref[1]: http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown

void SigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    // All the default sigint handler does is call shutdown()
    g_request_shutdown = 1;
}

int main(int argc, char **argv) {
    int clock = 0;
    double rate;
    std::string devname, frame_id;
    bool auto_adjust = true;
    int frq_div = 1;
    char trash[255];
    int fdc = -1;

    ros::init(argc, argv, "dynpick_driver", ros::init_options::NoSigintHandler);
    ros::NodeHandle n, nh("~");
    nh.param<std::string>("device", devname, "/dev/ttyUSB0");
    nh.param<std::string>("frame_id", frame_id, "/sensor");
    nh.param<double>("rate", rate, 1000);
    nh.param<bool>("acquire_calibration", auto_adjust, true);
    nh.param<int>("frequency_div", frq_div, 1);

    ros::ServiceServer service = n.advertiseService("tare", offsetRequest);
    ros::Publisher pub = n.advertise<geometry_msgs::WrenchStamped>("force", 2000);
    ros::Publisher pub_vis = n.advertise<geometry_msgs::WrenchStamped>("force_vis", 100);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Open COM port
    ROS_INFO("Open %s", devname.c_str());

    fdc = open(devname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fdc < 0) {
        ROS_ERROR("could not open %s\n", devname.c_str());
        return -1;
    }

    // Obtain sampling rate
    ROS_INFO("Sampling Frequency = %f Hz", 2000.0);

    // Set baud rate of COM port
    SetComAttr(fdc);

    // Clean up
    clearSocket(fdc, trash);

    // Stop Sensor Output
    if (fdc > 0 && write(fdc, "E", 1) < 0) {
        ROS_WARN("Cannot Stop Sensor Output!");
    }
    clearSocket(fdc, trash);

    // Sensor Information
    if (fdc > 0 && write(fdc, "V", 1) >= 0) {
        char hardware_version[HARDWARE_DATA_LENGTH];
        readCharFromSocket(fdc, HARDWARE_DATA_LENGTH, hardware_version);
        std::string hardware_version_str(hardware_version);
        std::regex ws_re("\n");
        std::vector<std::string> v(
                std::sregex_token_iterator(
                        hardware_version_str.begin(),
                        hardware_version_str.end(),
                        ws_re,
                        -1),
                std::sregex_token_iterator());
        int skip_first_char = 0;
        for (auto&& s: v) {
            if (!skip_first_char) {
                skip_first_char = 1;
                continue;
            }
            ROS_INFO("%s", s.c_str());
        }
        clearSocket(fdc, trash);
    } else {
        ROS_WARN("Cannot Get Sensor Information!");
    }

    // Autoadjust
    float calib[6] = {1, 1, 1, 1, 1, 1};
    if (auto_adjust){
        if (fdc > 0 && write(fdc, "p", 1) >= 0) {
            char reply[CALIB_DATA_LENGTH];
            readCharFromSocket(fdc, CALIB_DATA_LENGTH, reply);
            sscanf(reply, "%f,%f,%f,%f,%f,%f", &calib[0], &calib[1], &calib[2], &calib[3], &calib[4], &calib[5]);
            ROS_INFO("Calibration from sensor: %.3f LSB/N, %.3f LSB/N, %.3f LSB/N, %.3f LSB/Nm, %.3f LSB/Nm, %.3f LSB/Nm", calib[0], calib[1], calib[2], calib[3], calib[4], calib[5]);
            clearSocket(fdc, trash);
        } else {
            ROS_WARN("Cannot Set Auto Adjust!");
        }
    }

    // Set frequency divider filter
    if (frq_div == 1 || frq_div == 2 || frq_div == 4 || frq_div == 8) {
        char cmd[50];
        sprintf(cmd, "%dF", frq_div);
        if (fdc > 0 && write(fdc, cmd, 2) >= 0) {
            ROS_INFO("Set the frequency divider to %s", cmd);
        } else {
            ROS_WARN("Cannot Set Frequency Divider Filter!");
        }
        // check if successful
        if (fdc > 0 && write(fdc, "0F", 2) >= 0) {
            char repl[3];
            readCharFromSocket(fdc, 3, repl);
            ROS_ERROR_COND(repl[0]-'0' != frq_div, "Response by sensor is not as expected! Current Filter: %dF", repl[0]-'0');
            clearSocket(fdc, trash);
        } else {
            ROS_WARN("Cannot Check Frequency Divider Filter!");
        }
    } else {
        ROS_WARN("Not setting frequency divider. Parameter out of acceptable values {1,2,4,8}: %d", frq_div);
    }

    // Request for initial single data
    // ret = write(fdc, "R", 1);

    char str[256];
    int tick;
    unsigned short data[6];
    geometry_msgs::WrenchStamped msg;

    // ros::Rate loop_rate(rate);
    // Continuous Mode
    if (fdc > 0 && write(fdc, "S", 1) < 0) {
        ROS_ERROR("Cannot Start Continuous Sample Mode!");
        SigintHandler(0);
    }

    signal(SIGINT, SigintHandler);
    while (!g_request_shutdown) {
        std::unique_lock<std::mutex> lock(m_);
        if (offset_reset_ <= 0){
            // Request for initial data (2nd round)
            // ret = write(fdc, "R", 1);

            // Obtain single data
            readCharFromSocket(fdc, DATA_LENGTH, str);
            sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx", &tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

            msg.header.frame_id = frame_id;
            msg.header.stamp = ros::Time::now();
            msg.header.seq = clock++;

            msg.wrench.force.x = (data[0] - 8192.0)/calib[0];
            msg.wrench.force.y = (data[1] - 8192.0)/calib[1];
            msg.wrench.force.z = (data[2] - 8192.0)/calib[2];
            msg.wrench.torque.x = (data[3] - 8192.0)/calib[3];
            msg.wrench.torque.y = (data[4] - 8192.0)/calib[4];
            msg.wrench.torque.z = (data[5] - 8192.0)/calib[5];

            pub.publish(msg);

            if (clock % downsample == 0) {
                msg.header.seq = clock / downsample;
                pub_vis.publish(msg);
            }

            lock.unlock();
        } else {
            // Request for offset reset
            if (fdc > 0 && write(fdc, "O", 1) < 0) {
                ROS_WARN("Cannot Check Frequency Divider Filter!");
            }
            offset_reset_ --;
            lock.unlock();
            cv_.notify_all();
        }
        // loop_rate.sleep();
    }

    if (fdc > 0 && write(fdc, "E", 1) >=0) {
        ROS_INFO("Continuous Output Stopped.");
    } else {
        ROS_WARN("Cannot Stop Continuous Output.");
    }
    clearSocket(fdc, trash);
    if (fdc > 0 && close(fdc) >= 0) {
        ROS_INFO("Dynpick Force Sensor Closed.");
    } else {
        ROS_WARN("Cannot Close Dynpick Force Sensor.");
    }
    ros::shutdown();

    return 0;
}
