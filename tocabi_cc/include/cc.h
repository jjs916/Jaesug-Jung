#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"


class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    //void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    RobotData &rd_;
    RobotData rd_cc_;

    const std::string FILE_NAMES[6]={
        "../jjs_data/RH.txt",
        "../jjs_data/LH.txt",
        "../jjs_data/Joint.txt",
        "../jjs_data/walking.txt",
        "../jjs_data/walking_vel.txt",
        "../jjs_data/common.txt"
    };
    ofstream file[6];

    //WholebodyController &wbc_;
    //TaskCommand tc;

private:
//control variables
    bool task_init;
    Eigen::VectorQd ControlVal_;
    Eigen::VectorQd Task_torque;
    Eigen::VectorQd Gravity_torque;
    Eigen::VectorQd Extra_torque;
    Eigen::VectorQd Contact_torque;
    Eigen::VectorQd Total_torque;
    Eigen::VectorQd Test_torque;
//task variables
    Eigen::VectorXd f_star;
    Eigen::Vector3d COM_init;
    Eigen::Vector3d RH_init;
    Eigen::Vector3d LH_init;
    Eigen::Vector3d RF_init;
    Eigen::Vector3d LF_init;
    Eigen::VectorXd F_contact;
    double fc_ratio;
    Eigen::Vector3d COM_pos_local;
    Eigen::Vector3d RH_pos_local;
    Eigen::Vector3d LH_pos_local;
    Eigen::Vector3d COM_vel_local;
    Eigen::Vector3d RH_vel_local;
    Eigen::Vector3d LH_vel_local;
//joint variables //for IK
    Eigen::VectorQd q_init;
    Eigen::VectorQd q_desired;
    Eigen::VectorQd q_desired_pre;
    Eigen::VectorQd qdot_desired;
    Eigen::VectorXd R_xdot;
    Eigen::VectorXd L_xdot;

    Eigen::Isometry3d pelv_yaw_rot_current_from_global_;
    Eigen::Isometry3d pelv_trajectory_float_; //pelvis frame
    Eigen::Isometry3d rfoot_trajectory_float_;
    Eigen::Isometry3d lfoot_trajectory_float_;
    Eigen::Isometry3d pelv_vel_float_;
    Eigen::Isometry3d rfoot_vel_float_;
    Eigen::Isometry3d lfoot_vel_float_;

    Eigen::Isometry3d pelv_trajectory_support_; //local frame
    Eigen::Isometry3d rfoot_trajectory_support_;  //local frame
    Eigen::Isometry3d lfoot_trajectory_support_;
    Eigen::Isometry3d pelv_vel_support_; 
    Eigen::Isometry3d rfoot_vel_support_;  //local frame
    Eigen::Isometry3d lfoot_vel_support_;

    Eigen::Isometry3d rfoot_float_init_;
    Eigen::Isometry3d lfoot_float_init_;
    Eigen::Isometry3d pelv_float_init_;
    Eigen::Isometry3d pelv_support_init_;
    Eigen::Isometry3d rfoot_support_init_;
    Eigen::Isometry3d lfoot_support_init_;

    Eigen::Vector3d pelv_support_euler_init_;
    Eigen::Vector3d lfoot_support_euler_init_;
    Eigen::Vector3d rfoot_support_euler_init_;
    Eigen::Vector3d pelv_traj_euler;
    Eigen::Vector3d lfoot_traj_euler;
    Eigen::Vector3d rfoot_traj_euler;

    Eigen::Vector12d q_d;//desired from ik
    Eigen::MatrixXd Jr;//leg jacobian for velocity
    Eigen::MatrixXd Jl;

    double zmp_x;
    double zmp_y;

    //control gains
    Eigen::VectorQd Kp;
    Eigen::VectorQd Kd;
    Eigen::Vector3d Kp_com;
    Eigen::Vector3d Kd_com;
    Eigen::Vector3d Kp_com_rot;
    Eigen::Vector3d Kd_com_rot;
    Eigen::Vector3d Kp_ub;
    Eigen::Vector3d Kd_ub;
    Eigen::Vector3d Kp_hand;
    Eigen::Vector3d Kd_hand;
    Eigen::Vector3d Kp_hand_rot;
    Eigen::Vector3d Kd_hand_rot;
    Eigen::Vector3d Kp_foot;
    Eigen::Vector3d Kd_foot;
    Eigen::Vector3d Kp_foot_rot;
    Eigen::Vector3d Kd_foot_rot;

    //task parameters
    bool task_state_init = true;
    bool D2S = true;
    bool first_step = true;
    bool prefoot = true; // true : left contact - false : right contact
    int state;
    int prestate;
    double presupport;// 1 = left 0 = right
    double cursupport;
    int task_state_n;
    int task_number;
    double task_time1;
    double task_time2;
    double task_time3;

    //motor variable
    Eigen::MatrixVVd motor_inertia;
    Eigen::MatrixVVd motor_inertia_inv;
};
