#include <tocabi_controller/data_container.h>
#include <tocabi_controller/link.h>
#include "math_type_define.h"
#include <mutex>

class CustomController
{
public:
    CustomController(DataContainer &dc,RobotData &rd);
    ~CustomController();
    Eigen::VectorQd getControl();
    Eigen::MatrixXd getCmatrix(Eigen::VectorQVQd qq, Eigen::VectorVQd qdot);
    Eigen::MatrixXd getCmatrix(Eigen::VectorQd q, Eigen::VectorQd qdot);
    Eigen::VectorVQd Quar2Eul(Eigen::VectorQVQd q);

    void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    
    DataContainer &dc_;
    RobotData &rd_;
    WholebodyController &wbc_;
    Walking_controller &wkc_;
    TaskCommand tc;

    Eigen::MatrixXd foot_stepc;
    int contactModec = 1;
    int walking_tickc = 1;
    int walking_tickc_pre = 1;
    bool phaseChangec = false;
    bool phaseChangec1 = false;
    int current_step_numc = 0;
    int double2Single_prec = 0;
    int double2Singlec = 0;
    int single2Doublec = 0;
    int single2Double_prec = 0;
    int total_step_numc = 0;
    int t_start_realc = 0;
    int t_double1c = 0;
    int t_rest_tempc = 0;
    int t_totalc = 0;
    int t_rest_initc = 0;
    int t_rest_lastc = 0;
    int t_double2c = 0;
    int t_startc = 0;
    double t_impc = 0.0;
    double rate;
    int walkingHz = 1000;
    double t_tempc;
    double lipm_wc;
    Eigen::VectorXd zmp_refxc;
    Eigen::VectorXd zmp_refyc;

    std::mutex mtx_wlk;

    const std::string FILE_NAMES[6]={
        "../jjs_data/RH.txt",
        "../jjs_data/LH.txt",
        "../jjs_data/Joint.txt",
        "../jjs_data/walking.txt",
        "../jjs_data/walking_vel.txt",
        "../jjs_data/common.txt"
    };

    ofstream file[6];

private:
//////////////////////////////////////////////////////////task control
    //control variables
    Eigen::VectorQd ControlVal_;
    Eigen::VectorQd task_torque;
    Eigen::VectorQd gravity_torque;
    Eigen::VectorQd total_torque;
    Eigen::VectorQd Contact_torque;
    Eigen::VectorQd Null_torque;
    Eigen::VectorQd extra_torque;

    Eigen::VectorQd FF_total_torque;
    Eigen::VectorQd FF_torque;
    Eigen::VectorQd FF_contact_torque;
    
    Eigen::MatrixXd N_matrix;
    Eigen::MatrixXd I33;
    Eigen::MatrixXd JkT;
    Eigen::MatrixXd JkT_sub;
    Eigen::MatrixXd JkT_sub_pinv;
    Eigen::MatrixXd Cmat;
    Eigen::VectorXd x_dot_desired;
    Eigen::VectorXd x_dot_desired_pre;
    Eigen::VectorXd x_ddot_desired;
    Eigen::VectorXd x_ddot_desired_pre;
    Eigen::VectorQd q_dot_pre;
    Eigen::VectorQd q_ddot_est;
    Eigen::VectorQd q_ext_est;
    Eigen::VectorQd q_ext_est_pre;
    Eigen::VectorQd q_ext_dot_est;
    Eigen::VectorQd q_ext_dot_est_pre;
    Eigen::VectorQd q_ext_ddot_est;

    double zmp_x;
    double zmp_y;

    //initializiong points
    Eigen::Vector3d RH_x_init_local;
    Eigen::Vector3d LH_x_init_local;
    Eigen::Vector3d RH_x_target_local;
    Eigen::Vector3d LH_x_target_local;
    Eigen::Vector3d x_traj_local;
    Eigen::Matrix3d RH_R_init_local;
    Eigen::Matrix3d LH_R_init_local;
    Eigen::Matrix3d RH_R_target_local;
    Eigen::Matrix3d LH_R_target_local;
    Eigen::Matrix3d gui2hand_rot;
    Eigen::Vector3d pelvis_gap;
    Eigen::Vector3d estimate_rgap;
    Eigen::Vector3d estimate_lgap;
    Eigen::Vector3d initial_rfoot;
    Eigen::Vector3d initial_lfoot;
    Eigen::Vector3d rfoot_loop_init;
    Eigen::Vector3d lfoot_loop_init;
    Eigen::Vector3d pelvis_loop_init;
    Eigen::Vector3d rfoot2pel_init;
    Eigen::Vector3d lfoot2pel_init;
    Eigen::Vector3d COM_init;
    Eigen::Vector3d Foot_init;
    Eigen::Vector3d Foot_init2;
    Eigen::Vector3d COM_Target;
    double step_cnt;

    //control gains
    Eigen::VectorQd Kp;
    Eigen::VectorQd Kd;
    Eigen::Vector3d Kp_com;
    Eigen::Vector3d Kd_com;
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

    Eigen::Vector12d contact_force_re;
    double fc_ratio;
    double eta_data;

    bool task_state_init = true;
    bool D2S = true;
    bool first_step = true;
    bool prefoot = true; // true : left contact - false : right contact
    int state;
    int prestate;
    int task_state_n;
    int task_number;
    double task_time1;
    double task_time2;
    double task_time3;
    /////////////////////////////////////////////task control
    int n = 1;
};