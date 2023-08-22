// Copyright 2021, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#include <csignal>
#include <cstdio>
#include <cstdlib>

#include <rocos_app/drive.h>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <string>
#include <gflags/gflags.h>

DEFINE_string(urdf, "config/robot.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");

bool isRuning = true;

#pragma region //*测试9  完整上电保护程序

namespace rocos
{
    /**
     * @brief 字符串切割函数
     *
     * @param str 待切割字符串
     * @param tokens 结果存储
     * @param delim 切割符
     */

    void Robot::test()
    {

        //**变量初始化 **//
        std::string str{""};
        std::ifstream csv_null_motion;

        char tem[2048];
        std::vector<std::string> tokens;
        std::vector<KDL::JntArray> servo_data;
        KDL::JntArray joints(_joint_num);
        KDL::JntArray last_joints(_joint_num);

        int row_index = 1;

        JC_helper::TCP_server my_server;

        my_server.init();
        boost::thread(&JC_helper::TCP_server::RunServer, &my_server).detach(); // 开启服务器

        //**-------------------------------**//

#pragma region //*电机使能检查

        for (int i{0}; i < jnt_num_; i++)
        {
            if (joints_[i]->getDriveState() != DriveState::OperationEnabled)
            {
                for (int j{0}; j < 1; j++)
                {
                    PLOG_ERROR << "电机[" << i << "] 未使能，确定主站已初始化完成了？,输入y确认";
                    std::cin >> str;
                    if (str != std::string_view{"y"})
                    {
                        PLOG_ERROR << "未输入y, 判断主站 {未} 初始化完成,程序关闭";

                        exit(0);
                    }
                }
            }
        }

        setEnabled();
#pragma endregion

        str.clear();

        while (isRuning)
        {

            PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
            std::cin >> str;
            if (str == std::string_view{"run"})
            {
                int i = 0;

                using namespace KDL;
                double pose1[7] = {90, 63.823, 3.596, -105.917, 88.828, -89.121, 0};
                double pose2[7] = {59.691, 67.837, -1.395, 43.771, 78.769, -59.241, 0};
                double pose5[7] = {119.879, -59.942, -0.334, -45.327, 90.743, -68.856, 0};
                // pose2->pose5 middle pose
                double pose_mid[7] = {119.879, -67.837, -1.395, 43.771, 78.624, -85.528, 0};
                double pose3[7] = {90, 33.435, 3.596, 20, 97.833, -89.155, 0};
                double pose4[7] = {98.107, -42.702, 0, 4.48, 86.667, -92.953, 0};
                //                getJointTorque();
                KDL::JntArray q_target1(_joint_num);
                KDL::JntArray q_target2(_joint_num);
                KDL::JntArray q_target3(_joint_num);
                KDL::JntArray q_target4(_joint_num);
                KDL::JntArray q_target5(_joint_num);
                KDL::JntArray q_target_mid(_joint_num);
                double v_demo = 0.3;
                double acc_demo = 0.6;
                // for (unsigned int i = 0; i < _joint_num; ++i)
                // {
                //     q_target1(i) = pose1[i] * M_PI / 180;
                //     q_target2(i) = pose2[i] * M_PI / 180;
                //     q_target3(i) = pose3[i] * M_PI / 180;
                //     q_target4(i) = pose4[i] * M_PI / 180;
                //     q_target5(i) = pose5[i] * M_PI / 180;
                //     q_target_mid(i) = pose_mid[i] * M_PI / 180;
                // }
                double joint_tor[_joint_num];
                for (unsigned int i = 0; i < _joint_num; ++i)
                {
                    joint_tor[i] = getJointTorque(i);
                    std::cout << "关节 " << i << ":" << joint_tor[i]<<std::endl;
                }

                //**pose3->pose2**//
                // MoveJ(q_target2, v_demo, acc_demo, 0, 0, false);
                // sleep(2);
                // std::cout << "运行到位 " << "5" << std::endl;
            }
            else
            {
                PLOG_ERROR << "不安全环境,电机抱闸";
                setDisabled();

                return;
            }
        }
    }

    // PLOG_INFO << "全部测试结束,goodbye!";
}
// namespace rocos
#pragma endregion

/// \brief 处理终端的Ctrl-C信号
/// \param signo
void signalHandler(int signo)
{
    if (signo == SIGINT)
    {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRuning = false;
        exit(0);
    }
}

int main(int argc, char *argv[])
{
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //**-------------------------------**//

    // boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(_joint_num); // 仿真
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>(); // 真实机械臂

    Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    auto robotService = RobotServiceImpl::getInstance(&robot);

    //------------------------wait----------------------------------
    std::thread thread_test{&rocos::Robot::test, &robot};

    //------------------------wait----------------------------------
    robotService->runServer();

    thread_test.join();

    return 0;
}
