#include "custom_controller.h"
#define PI 3.141592653589793238462643
#define controltype 1 //1:original 2:mosf

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc.wbc_), wkc_(dc.wkc_)
{
    ControlVal_.setZero();
    Cmat.resize(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    Cmat.setZero();
    I33.resize(MODEL_DOF, MODEL_DOF);
    I33.setIdentity(MODEL_DOF, MODEL_DOF);
    extra_torque.setZero();
    Foot_init.setZero();
    Foot_init2.setZero();
    step_cnt = 1.0;
    presupport = 1.0;
    cursupport = 1.0;

    phaseChangec = false;
    for (int i = 0; i < 3; ++i)
    {
        //walking gain
        // Kp_com(i) = 300.0;
        // Kd_com(i) = 10.0;
        // Kp_ub(i) = 900.0;
        // Kd_ub(i) = 60.0;
        // Kp_hand(i) = 1600.0;
        // Kd_hand(i) = 40.0;
        // Kp_hand_rot(i) = 400.0;
        // Kd_hand_rot(i) = 40.0;
        // Kp_foot(i) = 1600.0;
        // Kd_foot(i) = 80.0;
        // Kp_foot_rot(i) = 2500.0;
        // Kd_foot_rot(i) = 60.0;
        if(controltype == 1){
            Kp_com(i) = 55.0;//60.0;
            Kd_com(i) = 5.0;//5.0;
            Kp_com_rot(i) = 100.0;
            Kd_com_rot(i) = 10.0;
            Kp_ub(i) = 100.0;
            Kd_ub(i) = 10.0;
            Kp_hand(i) = 10.0;
            Kd_hand(i) = 1.0;
            Kp_hand_rot(i) = 10.0;
            Kd_hand_rot(i) = 1.0;
            Kp_foot(i) = 100.0;
            Kd_foot(i) = 5.0;
            Kp_foot_rot(i) = 100.0;
            Kd_foot_rot(i) = 5.0;
        }
        else if (controltype == 2)
        {
            Kp_com(i) = 150.0;
            Kd_com(i) = 10.0;//40 60 100
            Kp_com_rot(i) = 100.0;
            Kd_com_rot(i) = 1.0;
            Kp_ub(i) = 50.0;
            Kd_ub(i) = 1.0;
            Kp_hand(i) = 100.0;
            Kd_hand(i) = 5.0;
            Kp_hand_rot(i) = 100.0;
            Kd_hand_rot(i) = 5.0;
            Kp_foot(i) = 400.0;
            Kd_foot(i) = 40.0;
            Kp_foot_rot(i) = 100.0;
            Kd_foot_rot(i) = 5.0;
        }
    }

    Kp << 400.0, 400.0, 400.0, 400.0, 400.0, 400.0,           //left foot
        400.0, 400.0, 400.0, 400.0, 400.0, 400.0,             //right foot
        400.0, 400.0, 400.0,                                  //waist
        200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 10.0, 10.0, //left hand
        10.0, 10.0,                                           //neck
        200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 10.0, 10.0; //right hand

    Kd << 15.0, 24.0, 15.0, 25.0, 24.0, 24.0,
        15.0, 24.0, 15.0, 25.0, 24.0, 24.0,
        15.0, 24.0, 15.0,
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 1.0, 1.0,
        1.0, 1.0,
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 1.0, 1.0;

    gui2hand_rot.setZero();
    gui2hand_rot(0, 1) = 1.0;
    gui2hand_rot(1, 0) = 1.0;
    gui2hand_rot(2, 2) = -1.0;
    task_number = 0;
    pelvis_gap.setZero();
    estimate_lgap.setZero();
    estimate_rgap.setZero();

    total_torque.setZero();
    FF_torque.setZero();
    FF_total_torque.setZero();
    FF_contact_torque.setZero();

    file[0].open(FILE_NAMES[0].c_str());
    file[0] << "RH_Xd" << "\t" << "RH_Yd" << "\t" << "RH_Zd"
            << "\t" << "RH_X" << "\t" << "RH_Y" << "\t" << "RH_Z"
            << endl;
    file[1].open(FILE_NAMES[1].c_str());
    file[1] << "LH_Xd" << "\t" << "LH_Yd" << "\t" << "LH_Zd"
            << "\t" << "LH_X" << "\t" << "LH_Y" << "\t" << "LH_Z" 
            << endl;
    file[2].open(FILE_NAMES[2].c_str());
    file[2] << "L_HipYaw" << "\t" << "L_HipRoll" << "\t" << "L_HipPitch"
            << "\t" << "L_Knee" << "\t" << "L_AnklePitch" << "\t" << "L_AnkleRoll"
            << "\t" << "R_HipYaw" << "\t" << "R_HipRoll" << "\t" << "R_HipPitch"
            << "\t" << "R_Knee" << "\t" << "R_AnklePitch" << "\t" << "R_AnkleRoll"
            << "\t" << "WaistYaw" << "\t" << "WaistPitch" << "\t" << "WaistRoll"
            << "\t" << "L_Shoulder1" << "\t" << "L_Shoulder2" << "\t" << "L_Shoulder3"
            << "\t" << "L_Shoulder4" << "\t" << "L_Elbow" << "\t" << "L_Forearm"
            << "\t" << "L_Wrist1" << "\t" << "L_Wrist2" << "\t" << "Neck"
            << "\t" << "Head" << "\t" << "R_Shoulder1" << "\t" << "R_Shoulder2"
            << "\t" << "R_Shoulder3" << "\t" << "R_Shoulder4" << "\t" << "R_Elbow"
            << "\t" << "R_Forearm" << "\t" << "R_Wrist1" << "\t" << "R_Wrist2"
            << endl;
    file[3].open(FILE_NAMES[3].c_str());
    file[3] << "pelv_x_d" << "\t" << "pelv_y_d" << "\t" << "pelv_z_d"
            << "\t" << "pelv_x" << "\t" << "pelv_y" << "\t" << "pelv_z"
            << "\t" << "RF_x_d" << "\t" << "RF_y_d" << "\t" << "RF_z_d"
            << "\t" << "RF_x" << "\t" << "RF_y" << "\t" << "RF_z"
            << "\t" << "LF_x_d" << "\t" << "LF_y_d" << "\t" << "LF_z_d" 
            << "\t" << "LF_x" << "\t" << "LF_y" << "\t" << "LF_z"
            << endl;
    file[4].open(FILE_NAMES[4].c_str());
    file[4] << "pelv_xdot_d" << "\t" << "pelv_ydot_d" << "\t" << "pelv_zdot_d"
            << "\t" << "pelv_xdot" << "\t" << "pelv_ydot" << "\t" << "pelv_zdot"
            << "\t" << "RF_xdot_d" << "\t" << "RF_ydot_d" << "\t" << "RF_zdot_d"
            << "\t" << "RF_xdot" << "\t" << "RF_ydot" << "\t" << "RF_zdot"
            << "\t" << "LF_xdot_d" << "\t" << "LF_ydot_d" << "\t" << "LF_zdot_d"
            << "\t" << "LF_xdot" << "\t" << "LF_ydot" << "\t" << "LF_zdot"
            << endl;
    file[5].open(FILE_NAMES[5].c_str());
    
    state = 1;
    prestate = 1;
}

CustomController::~CustomController()
{
    file[0].close();
    file[1].close();
    file[2].close();
    file[3].close();
    file[4].close();
    file[5].close();
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}
void CustomController::taskCommandToCC(TaskCommand tc_)
{
    tc = tc_;
    task_state_init = true;
}

void CustomController::computeSlow()
{
    if (tc.mode == 10)
    { task_torque.setZero();
        wbc_.set_contact(rd_, 1, 1);

        if (tc.task_init)
        {
            rd_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
            tc.task_init = false;
            task_state_n = 1;
        }
        rd_.link_[Upper_Body].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

        if (task_state_n == 1)
        {
            if (task_state_init == true)
            {
                cout << "DSP1" << endl;
                task_number = 9 + 12;
                task_time1 = tc.traj_time;//5.0;

                rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                rd_.f_star.setZero(task_number);

                COM_init = rd_.link_[Pelvis].xpos - rd_.link_[Right_Foot].xpos; //rd_.link_[Pelvis].xpos;//

                rd_.link_[Pelvis].x_desired = rd_.link_[Pelvis].xpos - rd_.link_[Right_Foot].xpos; //COM_init;//
                rd_.link_[Pelvis].x_desired(0) = COM_init(0) + tc.l_x;                             //rd_.link_[Pelvis].xpos(0) + tc.l_x;//
                rd_.link_[Pelvis].x_desired(1) = COM_init(1) + tc.l_y;                             //rd_.link_[Pelvis].xpos(1) + tc.l_y;//
                rd_.link_[Pelvis].x_desired(2) = COM_init(2) + tc.l_z;                             //rd_.link_[Pelvis].xpos(2) + tc.l_z;//
                                                                                                   //rd_.link_[Pelvis].rot_desired = Matrix3d::Identity();
                rd_.link_[Pelvis].rot_desired = DyrosMath::rotateWithX(tc.roll * 3.1415 / 180.0);
                RH_x_init_local = rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos; //rd_.link_[Upper_Body].rot_init.transpose() * (rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos);
                LH_x_init_local = rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos;  //rd_.link_[Upper_Body].rot_init.transpose() * (rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos);
                RH_R_init_local = rd_.link_[Upper_Body].rot_init.transpose() * rd_.link_[Right_Hand].rot_init;
                LH_R_init_local = rd_.link_[Upper_Body].rot_init.transpose() * rd_.link_[Left_Hand].rot_init;

                RH_R_target_local = Matrix3d::Identity();
                LH_R_target_local = Matrix3d::Identity();

                q_ddot_est.setZero();
                for (int i = 0; i < MODEL_DOF; ++i)
                {
                    q_ext_est(i) = rd_.q_(i) + (1.0 / 20000.0) * (-dc_.torque_elmo_(i));
                }

                task_state_init = false;
            }
            rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac;
            rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;
            rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;// - rd_.link_[Upper_Body].Jac;
            rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;// - rd_.link_[Upper_Body].Jac;

            q_ddot_est = (rd_.q_dot_ - q_dot_pre) * 1000.0;
            for (int i = 0; i < MODEL_DOF; ++i)
            {
                q_ext_est(i) = rd_.q_(i) + (rd_.Motor_inertia(i + 6, i + 6) * q_ddot_est(i) - dc_.torque_elmo_(i));
            }
            q_ext_dot_est = (q_ext_est - q_ext_est_pre) * 1000.0;
            q_ext_ddot_est = (q_ext_dot_est - q_ext_dot_est_pre) * 1000.0;

            //rd_.link_[Pelvis].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + task_time1);
            rd_.link_[Pelvis].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + task_time1, false);

            for (int i = 0; i < 3; ++i)
            {
                rd_.link_[Pelvis].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, COM_init(i), 0.0, 0.0, rd_.link_[Pelvis].x_desired(i), 0.0, 0.0)(0);
                rd_.link_[Pelvis].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, COM_init(i), 0.0, 0.0, rd_.link_[Pelvis].x_desired(i), 0.0, 0.0)(1);

                rd_.link_[Right_Hand].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Right_Hand].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(1);

                rd_.link_[Left_Hand].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Left_Hand].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(1);
            }

            //rd_.link_[Right_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].x_traj_local;
            //rd_.link_[Right_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].v_traj_local;
            rd_.link_[Right_Hand].r_traj = rd_.link_[Upper_Body].rot_init * RH_R_init_local * rd_.link_[Right_Hand].r_traj_local;
            rd_.link_[Right_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Right_Hand].w_traj_local;

            //rd_.link_[Left_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].x_traj_local;
            //rd_.link_[Left_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].v_traj_local;
            rd_.link_[Left_Hand].r_traj = rd_.link_[Upper_Body].rot_init * LH_R_init_local * rd_.link_[Left_Hand].r_traj_local;
            rd_.link_[Left_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Left_Hand].w_traj_local;

            //d_.f_star.segment(0, 6) = wbc_.getfstar6d(rd_, Pelvis, Kp_com, Kd_com, Kp_com_rot, Kd_com_rot);

            rd_.f_star(0) = Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - rd_.link_[Right_Foot].xpos(0))) + Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0));
            rd_.f_star(1) = Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - rd_.link_[Right_Foot].xpos(1))) + Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
            rd_.f_star(2) = Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - rd_.link_[Right_Foot].xpos(2))) + Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
            rd_.f_star.segment(3, 3) = wbc_.getfstar_rot(rd_, Pelvis, Kp_com_rot, Kd_com_rot);
            
            rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body, Kp_ub, Kd_ub);
            //rd_.f_star.segment(9, 6) = wbc_.getfstar6d(rd_, Right_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
            for (int i = 0; i < 3; i++){
                rd_.f_star(i+9) = Kp_com(i) * (rd_.link_[Right_Hand].x_traj(i) - (rd_.link_[Right_Hand].xpos(i) - rd_.link_[Upper_Body].xpos(i))) + Kd_com(i) * (rd_.link_[Right_Hand].v_traj(i) - (rd_.link_[Right_Hand].v(i) - rd_.link_[Upper_Body].v(i)));
            }
            rd_.f_star.segment(12, 3) = wbc_.getfstar_rot(rd_, Right_Hand, Kp_hand_rot, Kd_hand_rot);
            //rd_.f_star.segment(15, 6) = wbc_.getfstar6d(rd_, Left_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
            for (int i = 0; i < 3; i++)
            {
                rd_.f_star(i+15) = Kp_com(i) * (rd_.link_[Left_Hand].x_traj(i) - (rd_.link_[Left_Hand].xpos(i) - rd_.link_[Upper_Body].xpos(i))) + Kd_com(i) * (rd_.link_[Left_Hand].v_traj(i) - (rd_.link_[Left_Hand].v(i) - rd_.link_[Upper_Body].v(i)));
            }
            rd_.f_star.segment(18, 3) = wbc_.getfstar_rot(rd_, Left_Hand, Kp_hand_rot, Kd_hand_rot);

            gravity_torque = wbc_.gravity_compensation_torque(rd_, false, false);

            fc_ratio = 1.0;

            if (controltype == 1)
            {
                task_torque = wbc_.task_control_torque_with_gravity(rd_, rd_.J_task, rd_.f_star, false);
                //task_torque.setZero();
                Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque, contact_force_re, eta_data, fc_ratio, 0);
                //Contact_torque.setZero();
                total_torque = gravity_torque + task_torque + Contact_torque;
                ControlVal_ = gravity_torque + task_torque + Contact_torque;
            }
            else if (controltype == 2)
            {
                task_torque = wbc_.task_control_torque_motor(rd_, rd_.J_task, rd_.f_star);
                //task_torque.setZero();
                Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, 0);
                total_torque = gravity_torque + task_torque + Contact_torque;
                ControlVal_ = gravity_torque + task_torque + Contact_torque;
            }
        }
        q_dot_pre = rd_.q_dot_;
        q_ext_est_pre = q_ext_est;
        q_ext_dot_est_pre = q_ext_dot_est;

		    pel_ori_desired = rd_.link_[Pelvis].r_traj.eulerAngles(0, 1, 2);
        pel_ori = rd_.link_[Pelvis].Rotm.eulerAngles(0, 1, 2);

        file[3] << rd_.control_time_
                << "\t" << rd_.link_[Pelvis].x_traj(0) << "\t" << rd_.link_[Pelvis].x_traj(1) << "\t" << rd_.link_[Pelvis].x_traj(2)
                << "\t" << rd_.link_[Pelvis].xpos(0) - rd_.link_[Right_Foot].xpos(0) << "\t" << rd_.link_[Pelvis].xpos(1) - rd_.link_[Right_Foot].xpos(1) << "\t" << rd_.link_[Pelvis].xpos(2) - rd_.link_[Right_Foot].xpos(2)
                // << "\t" << rd_.link_[Pelvis].v_traj(0) << "\t" << rd_.link_[Pelvis].v_traj(1) << "\t" << rd_.link_[Pelvis].v_traj(2)
                // << "\t" << rd_.link_[Pelvis].v(0) << "\t" << rd_.link_[Pelvis].v(1) << "\t" << rd_.link_[Pelvis].v(2)
				        << "\t" << pel_ori_desired(0) << "\t" << pel_ori_desired(1) << "\t" << pel_ori_desired(2)
                << "\t" << pel_ori(0) << "\t" << pel_ori(1) << "\t" << pel_ori(2)
                << endl;
        // file[2] << rd_.control_time_
        //         << "\t" << total_torque(0) << "\t" << total_torque(1) << "\t" << total_torque(2)
				// << "\t" << total_torque(3) << "\t" << total_torque(4) << "\t" << total_torque(5)
				// << "\t" << total_torque(6) << "\t" << total_torque(7) << "\t" << total_torque(8)
				// << "\t" << total_torque(9) << "\t" << total_torque(10) << "\t" << total_torque(11)
				// << "\t" << dc_.torque_elmo_(0) << "\t" << dc_.torque_elmo_(1) << "\t" << dc_.torque_elmo_(2) 
				// << "\t" << dc_.torque_elmo_(3) << "\t" << dc_.torque_elmo_(4) << "\t" << dc_.torque_elmo_(5) 
				// << "\t" << dc_.torque_elmo_(6) << "\t" << dc_.torque_elmo_(7) << "\t" << dc_.torque_elmo_(8) 
				// << "\t" << dc_.torque_elmo_(9) << "\t" << dc_.torque_elmo_(10) << "\t" << dc_.torque_elmo_(11) 
				// << endl;
    }
    else if (tc.mode == 11)
    { //before single
        task_torque.setZero();
        wbc_.set_contact(rd_, 1, 1);

        if (tc.task_init)
        {
            rd_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
            tc.task_init = false;
            task_state_n = 1;
        }
        rd_.link_[Upper_Body].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

        if (task_state_n == 1)
        {
            if (task_state_init == true)
            {
              cout << "DSP1" << endl;
              task_number = 9 + 12;
              task_time1 = 10.0;

              for (int i = 0; i < 3; ++i)
              {
                Kp_com(i) = 100.0;
                Kd_com(i) = 10.0; //40 60 100
                Kp_com_rot(i) = 100.0;
                Kd_com_rot(i) = 1.0;
                Kp_ub(i) = 50.0;
                Kd_ub(i) = 1.0;
                Kp_hand(i) = 100.0;
                Kd_hand(i) = 5.0;
                Kp_hand_rot(i) = 100.0;
                Kd_hand_rot(i) = 5.0;
                Kp_foot(i) = 400.0;
                Kd_foot(i) = 40.0;
                Kp_foot_rot(i) = 100.0;
                Kd_foot_rot(i) = 5.0;
              }

              rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
              rd_.f_star.setZero(task_number);

              COM_init = rd_.link_[Pelvis].xpos - rd_.link_[Right_Foot].xpos; //rd_.link_[Pelvis].xpos;//

                //rd_.link_[Pelvis].x_desired = rd_.link_[Pelvis].xpos - rd_.link_[Right_Foot].xpos; //COM_init;//
                rd_.link_[Pelvis].x_desired(0) = COM_init(0);                             //rd_.link_[Pelvis].xpos(0) + tc.l_x;//
                rd_.link_[Pelvis].x_desired(1) = -0.03;                             //rd_.link_[Pelvis].xpos(1) + tc.l_y;//
                rd_.link_[Pelvis].x_desired(2) = COM_init(2);                             //rd_.link_[Pelvis].xpos(2) + tc.l_z;//

              rd_.link_[Pelvis].rot_desired = Matrix3d::Identity();
              RH_x_init_local = rd_.link_[Upper_Body].rot_init.transpose() * (rd_.link_[Right_Hand].x_init - rd_.link_[Upper_Body].x_init);
              LH_x_init_local = rd_.link_[Upper_Body].rot_init.transpose() * (rd_.link_[Left_Hand].x_init - rd_.link_[Upper_Body].x_init);
              RH_R_init_local = rd_.link_[Upper_Body].rot_init.transpose() * rd_.link_[Right_Hand].rot_init;
              LH_R_init_local = rd_.link_[Upper_Body].rot_init.transpose() * rd_.link_[Left_Hand].rot_init;

              RH_R_target_local = Matrix3d::Identity();
              LH_R_target_local = Matrix3d::Identity();

              q_ddot_est.setZero();
              for (int i = 0; i < MODEL_DOF; ++i)
              {
                q_ext_est(i) = rd_.q_(i) + (1.0 / 20000.0) * (-dc_.torque_elmo_(i));
              }

              task_state_init = false;
            }

            rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac;
            rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;
            rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;
            rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;

            q_ddot_est = (rd_.q_dot_ - q_dot_pre) * 1000.0;
            for (int i = 0; i < MODEL_DOF; ++i)
            {
                q_ext_est(i) = rd_.q_(i) + (rd_.Motor_inertia(i + 6, i + 6) * q_ddot_est(i) - dc_.torque_elmo_(i));
            }
            q_ext_dot_est = (q_ext_est - q_ext_est_pre) * 1000.0;
            q_ext_ddot_est = (q_ext_dot_est - q_ext_dot_est_pre) * 1000.0;

            //rd_.link_[Pelvis].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + task_time1);
            rd_.link_[Pelvis].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + task_time1, false);

            for (int i = 0; i < 3; ++i)
            {
                rd_.link_[Pelvis].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, COM_init(i), 0.0, 0.0, rd_.link_[Pelvis].x_desired(i), 0.0, 0.0)(0);
                rd_.link_[Pelvis].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, COM_init(i), 0.0, 0.0, rd_.link_[Pelvis].x_desired(i), 0.0, 0.0)(1);

                rd_.link_[Right_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Right_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(1);

                rd_.link_[Left_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Left_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(1);
            }

            rd_.link_[Right_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].x_traj_local;
            rd_.link_[Right_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].v_traj_local;
            rd_.link_[Right_Hand].r_traj = rd_.link_[Upper_Body].rot_init * RH_R_init_local * rd_.link_[Right_Hand].r_traj_local;
            rd_.link_[Right_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Right_Hand].w_traj_local;

            rd_.link_[Left_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].x_traj_local;
            rd_.link_[Left_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].v_traj_local;
            rd_.link_[Left_Hand].r_traj = rd_.link_[Upper_Body].rot_init * LH_R_init_local * rd_.link_[Left_Hand].r_traj_local;
            rd_.link_[Left_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Left_Hand].w_traj_local;

            //rd_.f_star.segment(0, 6) = wbc_.getfstar6d(rd_, Pelvis, Kp_com, Kd_com, Kp_com_rot, Kd_com_rot);
            rd_.f_star(0) = Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - rd_.link_[Right_Foot].xpos(0))) + Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0));
            rd_.f_star(1) = Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - rd_.link_[Right_Foot].xpos(1))) + Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
            rd_.f_star(2) = Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - rd_.link_[Right_Foot].xpos(2))) + Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
            rd_.f_star.segment(3, 3) = wbc_.getfstar_rot(rd_, Pelvis, Kp_com_rot, Kd_com_rot);
            rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body, Kp_ub, Kd_ub);
            rd_.f_star.segment(9, 6) = wbc_.getfstar6d(rd_, Right_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
            rd_.f_star.segment(15, 6) = wbc_.getfstar6d(rd_, Left_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);

            gravity_torque = wbc_.gravity_compensation_torque(rd_, false, false);

            if (rd_.control_time_ >= tc.command_time + task_time1 - 0.01)//only for going to SSP
            {
                fc_ratio = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1 - 0.01, tc.command_time + task_time1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)(0);
            }
            else
            {
                fc_ratio = 1.0;
            }

                task_torque = wbc_.task_control_torque_motor(rd_, rd_.J_task, rd_.f_star);
                Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, 0);
                total_torque = gravity_torque + task_torque + Contact_torque;
                ControlVal_ = gravity_torque + task_torque + Contact_torque;
        }
        q_dot_pre = rd_.q_dot_;
        q_ext_est_pre = q_ext_est;
        q_ext_dot_est_pre = q_ext_dot_est;

        file[3] << rd_.control_time_
                << "\t" << rd_.link_[Pelvis].x_traj(0) << "\t" << rd_.link_[Pelvis].x_traj(1) << "\t" << rd_.link_[Pelvis].x_traj(2)
                << "\t" << rd_.link_[Pelvis].xpos(0) - rd_.link_[Right_Foot].xpos(0) << "\t" << rd_.link_[Pelvis].xpos(1) - rd_.link_[Right_Foot].xpos(1) << "\t" << rd_.link_[Pelvis].xpos(2) - rd_.link_[Right_Foot].xpos(2)
                << endl;
    }
    else if (tc.mode == 12)
    { //after single
        if (task_state_n == 1)
        {
            wbc_.set_contact(rd_, 0, 1);

            if (task_state_init == true)
            {
                cout << "SSP1" << endl;
                task_number = 9 + 6 + 12;
                task_time1 = tc.traj_time;//10.0;

                rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                rd_.f_star.setZero(task_number);
                rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac;
                rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;
                rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Foot].Jac;
                rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;
                rd_.J_task.block(21, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;

                rd_.link_[Pelvis].Set_initpos();
                rd_.link_[Left_Foot].Set_initpos();
                rd_.link_[Right_Hand].Set_initpos();
                rd_.link_[Left_Hand].Set_initpos();

                rd_.link_[Pelvis].x_desired = rd_.link_[Pelvis].xpos;
				rd_.link_[Pelvis].x_desired(0) = rd_.link_[Pelvis].xpos(0)  + tc.l_x;//- 0.2;
				rd_.link_[Pelvis].x_desired(1) = rd_.link_[Pelvis].xpos(1)  + tc.l_y;//- 0.2;
                rd_.link_[Pelvis].x_desired(2) = rd_.link_[Pelvis].xpos(2)  + tc.l_z;//- 0.2;
                //rd_.link_[Pelvis].rot_desired = Matrix3d::Identity();
				rd_.link_[Pelvis].rot_desired = DyrosMath::rotateWithX(tc.roll * 3.1415 / 180.0);

                rd_.link_[Left_Foot].x_desired = rd_.link_[Left_Foot].xpos;
                //rd_.link_[Left_Foot].x_desired(2) = rd_.link_[Left_Foot].xpos(2);
                rd_.link_[Left_Foot].rot_desired = Matrix3d::Identity();

                RH_x_init_local = rd_.link_[Upper_Body].Rotm.transpose() * (rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos);
                LH_x_init_local = rd_.link_[Upper_Body].Rotm.transpose() * (rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos);
                RH_R_init_local = rd_.link_[Upper_Body].Rotm.transpose() * rd_.link_[Right_Hand].Rotm;
                LH_R_init_local = rd_.link_[Upper_Body].Rotm.transpose() * rd_.link_[Left_Hand].Rotm;

                RH_R_target_local = Matrix3d::Identity();
                LH_R_target_local = Matrix3d::Identity();

                task_state_init = false;
            }
            gravity_torque = wbc_.gravity_compensation_torque(rd_, false, false);

            rd_.link_[Pelvis].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + task_time1);
            rd_.link_[Pelvis].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + task_time1, false);

            rd_.link_[Left_Foot].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + task_time1);
            rd_.link_[Left_Foot].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + task_time1, false);

            for (int i = 0; i < 3; ++i)
            {
                rd_.link_[Right_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Right_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(1);

                rd_.link_[Left_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Left_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(1);
            }

            rd_.link_[Right_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].x_traj_local;
            rd_.link_[Right_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].v_traj_local;
            rd_.link_[Right_Hand].r_traj = rd_.link_[Upper_Body].rot_init * RH_R_init_local * rd_.link_[Right_Hand].r_traj_local;
            rd_.link_[Right_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Right_Hand].w_traj_local;

            rd_.link_[Left_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].x_traj_local;
            rd_.link_[Left_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].v_traj_local;
            rd_.link_[Left_Hand].r_traj = rd_.link_[Upper_Body].rot_init * LH_R_init_local * rd_.link_[Left_Hand].r_traj_local;
            rd_.link_[Left_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Left_Hand].w_traj_local;

            rd_.f_star.segment(0, 6) = wbc_.getfstar6d(rd_, Pelvis, 0.5*Kp_com, 0.5*Kd_com, 0.5*Kp_com_rot, 0.5*Kd_com_rot);
            rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body, 0.5*Kp_ub, 0.5*Kd_ub);
            rd_.f_star.segment(9, 6) = wbc_.getfstar6d(rd_, Left_Foot, 0.5*Kp_hand, 0.5*Kd_hand, 0.5*Kp_hand_rot, 0.5*Kd_hand_rot);
            rd_.f_star.segment(15, 6) = wbc_.getfstar6d(rd_, Right_Hand, 0.5*Kp_hand, 0.5*Kd_hand, 0.5*Kp_hand_rot, 0.5*Kd_hand_rot);
            rd_.f_star.segment(21, 6) = wbc_.getfstar6d(rd_, Left_Hand, 0.5*Kp_hand, 0.5*Kd_hand, 0.5*Kp_hand_rot, 0.5*Kd_hand_rot);

            //task_torque = wbc_.task_control_torque_motor(rd_, rd_.J_task, rd_.f_star);
            if (controltype == 1)
            {
                task_torque = wbc_.task_control_torque_with_gravity(rd_, rd_.J_task, rd_.f_star, false);
                //Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque, contact_force_re, eta_data, fc_ratio, 0);
                
                total_torque = gravity_torque + task_torque;// + Contact_torque;
                ControlVal_ = gravity_torque + task_torque;// + Contact_torque;
            }
            else if (controltype == 2)
            {
                task_torque = wbc_.task_control_torque_motor(rd_, rd_.J_task, rd_.f_star);
                //Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, 0);
                total_torque = gravity_torque + task_torque;// + Contact_torque;
                ControlVal_ = gravity_torque + task_torque;// + Contact_torque;
            }
/*
            // JkT = rd_.W_inv * rd_.Q_motor_T_ * rd_.Q_motor_temp_inv;
            // JkT_sub = JkT.block(0, 0, MODEL_DOF, 3);
            // JkT_sub_pinv = (JkT_sub.transpose()*JkT_sub).inverse()*JkT_sub.transpose();
            // N_matrix = I33 - JkT_sub*JkT_sub_pinv;

            // extra_torque(5) = (task_torque(5) + gravity_torque(5));
            // extra_torque(11) = (task_torque(11) + gravity_torque(11));
            // double th = 65.0;
            // if(extra_torque(5) < -th){
            //     extra_torque(5) = -(extra_torque(5) + th);
            // }
            // else if(extra_torque(5) > th){
            //     extra_torque(5) = -(extra_torque(5) - th);
            // }
            // else
            // {
            //     extra_torque(5) = 0.0;
            // }

            // if(extra_torque(11) < -th){
            //     extra_torque(11) = -(extra_torque(11) + th);
            // }
            // else if(extra_torque(11) > th){
            //     extra_torque(11) = -(extra_torque(11) - th);
            // }
            // else
            // {
            //     extra_torque(11) = 0.0;
            // }

            // Null_torque = N_matrix*extra_torque;

            //total_torque = gravity_torque + task_torque; // + Null_torque;
            //ControlVal_ = gravity_torque + task_torque;  // + Null_torque;*/
        }
        file[3] << rd_.control_time_
                << "\t" << rd_.link_[Pelvis].x_traj(0) << "\t" << rd_.link_[Pelvis].x_traj(1) << "\t" << rd_.link_[Pelvis].x_traj(2)
                << "\t" << rd_.link_[Pelvis].xpos(0) << "\t" << rd_.link_[Pelvis].xpos(1) << "\t" << rd_.link_[Pelvis].xpos(2)
                << endl;
    }
    else if (tc.mode == 13)
    { //IROS rebuild
        task_torque.setZero();

        if (tc.task_init)
        {
            rd_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
            tc.task_init = false;
            task_state_n = 1;
        }
        rd_.link_[Upper_Body].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

        if (task_state_n == 1)
        {
            wbc_.set_contact(rd_, 1, 1);

            if (task_state_init == true)
            {
                cout << "DSP1" << endl;
                task_number = 9 + 12;
                task_time1 = 10.0;

                rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                rd_.f_star.setZero(task_number);
                rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac;
                rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;
                rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;
                rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;

                COM_init = rd_.link_[COM_id].xpos;
                rd_.link_[COM_id].x_desired = rd_.link_[Right_Foot].xpos;
                rd_.link_[COM_id].x_desired(2) = rd_.link_[COM_id].xpos(2);
                rd_.link_[COM_id].rot_desired = Matrix3d::Identity();
                RH_x_init_local = rd_.link_[Upper_Body].rot_init.transpose() * (rd_.link_[Right_Hand].x_init - rd_.link_[Upper_Body].x_init);
                LH_x_init_local = rd_.link_[Upper_Body].rot_init.transpose() * (rd_.link_[Left_Hand].x_init - rd_.link_[Upper_Body].x_init);
                RH_R_init_local = rd_.link_[Upper_Body].rot_init.transpose() * rd_.link_[Right_Hand].rot_init;
                LH_R_init_local = rd_.link_[Upper_Body].rot_init.transpose() * rd_.link_[Left_Hand].rot_init;

                RH_R_target_local = Matrix3d::Identity();
                LH_R_target_local = Matrix3d::Identity();

                task_state_init = false;
            }
            gravity_torque = wbc_.gravity_compensation_torque(rd_, false, false);

            rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + task_time1);
            rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + task_time1, false);

            for (int i = 0; i < 3; ++i)
            {
                rd_.link_[Right_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Right_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(1);

                rd_.link_[Left_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Left_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time, tc.command_time + task_time1, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(1);
            }

            rd_.link_[Right_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].x_traj_local;
            rd_.link_[Right_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].v_traj_local;
            rd_.link_[Right_Hand].r_traj = rd_.link_[Upper_Body].rot_init * RH_R_init_local * rd_.link_[Right_Hand].r_traj_local;
            rd_.link_[Right_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Right_Hand].w_traj_local;

            rd_.link_[Left_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].x_traj_local;
            rd_.link_[Left_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].v_traj_local;
            rd_.link_[Left_Hand].r_traj = rd_.link_[Upper_Body].rot_init * LH_R_init_local * rd_.link_[Left_Hand].r_traj_local;
            rd_.link_[Left_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Left_Hand].w_traj_local;

            rd_.f_star.segment(0, 6) = wbc_.getfstar6d(rd_, COM_id, Kp_com, Kd_com, Kp_com, Kd_com);
            rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body, Kp_ub, Kd_ub);
            rd_.f_star.segment(9, 6) = wbc_.getfstar6d(rd_, Right_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
            rd_.f_star.segment(15, 6) = wbc_.getfstar6d(rd_, Left_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);

            task_torque = wbc_.task_control_torque_motor(rd_, rd_.J_task, rd_.f_star);

            if (rd_.control_time_ >= tc.command_time + task_time1 - 0.1)
            {
                fc_ratio = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1 - 0.1, tc.command_time + task_time1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)(0);
            }
            else
            {
                fc_ratio = 1.0;
            }

            Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, 0);

            ControlVal_ = gravity_torque + task_torque + Contact_torque;

            if (rd_.control_time_ >= tc.command_time + task_time1)
            {
                task_state_init = true;
                task_state_n = 2;
            }
        }
        else if (task_state_n == 2)
        {
            wbc_.set_contact(rd_, 0, 1);

            if (task_state_init == true)
            {
                cout << "SSP1" << endl;
                task_number = 9 + 6 + 12;
                task_time2 = 10.0;

                rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                rd_.f_star.setZero(task_number);
                rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac;
                rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;
                rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Foot].Jac;
                rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;
                rd_.J_task.block(21, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;

                rd_.link_[COM_id].x_init = rd_.link_[COM_id].x_desired;
                rd_.link_[Left_Foot].Set_initpos();
                rd_.link_[Right_Hand].Set_initpos();
                rd_.link_[Left_Hand].Set_initpos();

                rd_.link_[COM_id].x_desired = rd_.link_[COM_id].xpos;
                rd_.link_[COM_id].rot_desired = Matrix3d::Identity();

                rd_.link_[Left_Foot].x_desired = rd_.link_[Left_Foot].xpos;
                rd_.link_[Left_Foot].x_desired(2) = rd_.link_[Left_Foot].xpos(2) + 0.03;
                rd_.link_[Left_Foot].rot_desired = Matrix3d::Identity();

                RH_x_init_local = rd_.link_[Upper_Body].Rotm.transpose() * (rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos);
                LH_x_init_local = rd_.link_[Upper_Body].Rotm.transpose() * (rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos);
                RH_R_init_local = rd_.link_[Upper_Body].Rotm.transpose() * rd_.link_[Right_Hand].Rotm;
                LH_R_init_local = rd_.link_[Upper_Body].Rotm.transpose() * rd_.link_[Left_Hand].Rotm;

                RH_R_target_local = Matrix3d::Identity();
                LH_R_target_local = Matrix3d::Identity();

                task_state_init = false;
            }
            gravity_torque = wbc_.gravity_compensation_torque(rd_, false, false);

            rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time + task_time1, tc.command_time + task_time1 + task_time2);
            rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time + task_time1, tc.command_time + task_time1 + task_time2, false);

            rd_.link_[Left_Foot].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time + task_time1, tc.command_time + task_time1 + task_time2);
            rd_.link_[Left_Foot].Set_Trajectory_rotation(rd_.control_time_, tc.command_time + task_time1, tc.command_time + task_time1 + task_time2, false);

            for (int i = 0; i < 3; ++i)
            {
                rd_.link_[Right_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1, tc.command_time + task_time1 + task_time2, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Right_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1, tc.command_time + task_time1 + task_time2, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(1);

                rd_.link_[Left_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1, tc.command_time + task_time1 + task_time2, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Left_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1, tc.command_time + task_time1 + task_time2, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(1);
            }

            rd_.link_[Right_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].x_traj_local;
            rd_.link_[Right_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].v_traj_local;
            rd_.link_[Right_Hand].r_traj = rd_.link_[Upper_Body].rot_init * RH_R_init_local * rd_.link_[Right_Hand].r_traj_local;
            rd_.link_[Right_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Right_Hand].w_traj_local;

            rd_.link_[Left_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].x_traj_local;
            rd_.link_[Left_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].v_traj_local;
            rd_.link_[Left_Hand].r_traj = rd_.link_[Upper_Body].rot_init * LH_R_init_local * rd_.link_[Left_Hand].r_traj_local;
            rd_.link_[Left_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Left_Hand].w_traj_local;

            rd_.f_star.segment(0, 6) = wbc_.getfstar6d(rd_, COM_id, Kp_com, Kd_com, Kp_com, Kd_com);
            rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body, Kp_ub, Kd_ub);
            rd_.f_star.segment(9, 6) = wbc_.getfstar6d(rd_, Left_Foot, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
            rd_.f_star.segment(15, 6) = wbc_.getfstar6d(rd_, Right_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
            rd_.f_star.segment(21, 6) = wbc_.getfstar6d(rd_, Left_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);

            task_torque = wbc_.task_control_torque_motor(rd_, rd_.J_task, rd_.f_star);
            ControlVal_ = gravity_torque + task_torque;

            if (rd_.control_time_ >= tc.command_time + task_time1 + task_time2)
            {
                task_state_init = true;
                task_state_n = 3;
            }
        }
        else if (task_state_n == 3)
        {
            wbc_.set_contact(rd_, 0, 1);

            if (task_state_init == true)
            {
                cout << "SSP2" << endl;
                task_number = 9 + 6 + 12;
                task_time2 = 10.0;

                rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                rd_.f_star.setZero(task_number);
                rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac;
                rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;
                rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Foot].Jac;
                rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;
                rd_.J_task.block(21, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;

                rd_.link_[COM_id].x_init = rd_.link_[COM_id].x_desired;
                rd_.link_[Left_Foot].Set_initpos();
                rd_.link_[Right_Hand].Set_initpos();
                rd_.link_[Left_Hand].Set_initpos();

                rd_.link_[COM_id].x_desired = rd_.link_[COM_id].xpos;
                rd_.link_[COM_id].rot_desired = Matrix3d::Identity();

                //rd_.link_[Left_Foot].x_desired = rd_.link_[Left_Foot].xpos;
                rd_.link_[Left_Foot].x_desired(2) = rd_.link_[Left_Foot].x_desired(2) - 0.04;
                rd_.link_[Left_Foot].rot_desired = Matrix3d::Identity();

                RH_x_init_local = rd_.link_[Upper_Body].Rotm.transpose() * (rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos);
                LH_x_init_local = rd_.link_[Upper_Body].Rotm.transpose() * (rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos);
                RH_R_init_local = rd_.link_[Upper_Body].Rotm.transpose() * rd_.link_[Right_Hand].Rotm;
                LH_R_init_local = rd_.link_[Upper_Body].Rotm.transpose() * rd_.link_[Left_Hand].Rotm;

                RH_R_target_local = Matrix3d::Identity();
                LH_R_target_local = Matrix3d::Identity();

                task_state_init = false;
            }
            gravity_torque = wbc_.gravity_compensation_torque(rd_, false, false);

            rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time + task_time1 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2);
            rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time + task_time1 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2, false);

            rd_.link_[Left_Foot].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time + task_time1 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2);
            rd_.link_[Left_Foot].Set_Trajectory_rotation(rd_.control_time_, tc.command_time + task_time1 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2, false);

            for (int i = 0; i < 3; ++i)
            {
                rd_.link_[Right_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Right_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(1);

                rd_.link_[Left_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Left_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(1);
            }

            rd_.link_[Right_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].x_traj_local;
            rd_.link_[Right_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].v_traj_local;
            rd_.link_[Right_Hand].r_traj = rd_.link_[Upper_Body].rot_init * RH_R_init_local * rd_.link_[Right_Hand].r_traj_local;
            rd_.link_[Right_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Right_Hand].w_traj_local;

            rd_.link_[Left_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].x_traj_local;
            rd_.link_[Left_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].v_traj_local;
            rd_.link_[Left_Hand].r_traj = rd_.link_[Upper_Body].rot_init * LH_R_init_local * rd_.link_[Left_Hand].r_traj_local;
            rd_.link_[Left_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Left_Hand].w_traj_local;

            rd_.f_star.segment(0, 6) = wbc_.getfstar6d(rd_, COM_id, Kp_com, Kd_com, Kp_com, Kd_com);
            rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body, Kp_ub, Kd_ub);
            rd_.f_star.segment(9, 6) = wbc_.getfstar6d(rd_, Left_Foot, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
            rd_.f_star.segment(15, 6) = wbc_.getfstar6d(rd_, Right_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
            rd_.f_star.segment(21, 6) = wbc_.getfstar6d(rd_, Left_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);

            task_torque = wbc_.task_control_torque_motor(rd_, rd_.J_task, rd_.f_star);
            ControlVal_ = gravity_torque + task_torque;

            if (rd_.control_time_ >= tc.command_time + task_time1 + task_time2 + task_time2)
            {
                task_state_init = true;
                task_state_n = 4;
            }
        }
        if (task_state_n == 4)
        {
            wbc_.set_contact(rd_, 1, 1);

            if (task_state_init == true)
            {
                cout << "DSP2" << endl;
                task_number = 9 + 12;
                task_time3 = 10.0;

                rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                rd_.f_star.setZero(task_number);
                rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac;
                rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;
                rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;
                rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;

                rd_.link_[COM_id].x_init = rd_.link_[COM_id].x_desired;
                rd_.link_[COM_id].x_desired = 0.5 * rd_.link_[Right_Foot].xpos + 0.5 * rd_.link_[Left_Foot].xpos;
                rd_.link_[COM_id].x_desired(2) = rd_.link_[COM_id].x_init(2);
                rd_.link_[COM_id].rot_desired = Matrix3d::Identity();
                RH_x_init_local = rd_.link_[Upper_Body].rot_init.transpose() * (rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos);
                LH_x_init_local = rd_.link_[Upper_Body].rot_init.transpose() * (rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos);
                RH_R_init_local = rd_.link_[Upper_Body].rot_init.transpose() * rd_.link_[Right_Hand].Rotm;
                LH_R_init_local = rd_.link_[Upper_Body].rot_init.transpose() * rd_.link_[Left_Hand].Rotm;

                RH_R_target_local = Matrix3d::Identity();
                LH_R_target_local = Matrix3d::Identity();

                task_state_init = false;
            }
            gravity_torque = wbc_.gravity_compensation_torque(rd_, false, false);

            rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time + task_time1 + task_time2 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2 + task_time3);
            rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + task_time1, false);

            for (int i = 0; i < 3; ++i)
            {
                rd_.link_[Right_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1 + task_time2 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2 + task_time3, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Right_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1 + task_time2 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2 + task_time3, RH_x_init_local(i), 0.0, 0.0, RH_x_init_local(i), 0.0, 0.0)(1);

                rd_.link_[Left_Hand].x_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + +task_time1 + task_time2 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2 + task_time3, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(0);
                rd_.link_[Left_Hand].v_traj_local(i) = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + +task_time1 + task_time2 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2 + task_time3, LH_x_init_local(i), 0.0, 0.0, LH_x_init_local(i), 0.0, 0.0)(1);
            }

            rd_.link_[Right_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].x_traj_local;
            rd_.link_[Right_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].v_traj_local;
            rd_.link_[Right_Hand].r_traj = rd_.link_[Upper_Body].rot_init * RH_R_init_local * rd_.link_[Right_Hand].r_traj_local;
            rd_.link_[Right_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Right_Hand].w_traj_local;

            rd_.link_[Left_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].x_traj_local;
            rd_.link_[Left_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].v_traj_local;
            rd_.link_[Left_Hand].r_traj = rd_.link_[Upper_Body].rot_init * LH_R_init_local * rd_.link_[Left_Hand].r_traj_local;
            rd_.link_[Left_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Left_Hand].w_traj_local;

            rd_.f_star.segment(0, 6) = wbc_.getfstar6d(rd_, COM_id, Kp_com, Kd_com, Kp_com, Kd_com);
            rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body, Kp_ub, Kd_ub);
            rd_.f_star.segment(9, 6) = wbc_.getfstar6d(rd_, Right_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
            rd_.f_star.segment(15, 6) = wbc_.getfstar6d(rd_, Left_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);

            task_torque = wbc_.task_control_torque_motor(rd_, rd_.J_task, rd_.f_star);

            if (rd_.control_time_ >= (tc.command_time + task_time1 + task_time2 + task_time2) && rd_.control_time_ < (tc.command_time + task_time1 + task_time2 + task_time2 + 0.1))
            {
                fc_ratio = DyrosMath::QuinticSpline(rd_.control_time_, tc.command_time + task_time1 + task_time2 + task_time2, tc.command_time + task_time1 + task_time2 + task_time2 + 0.1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)(0);
            }
            else
            {
                fc_ratio = 1.0;
            }

            Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, 0);

            ControlVal_ = gravity_torque + task_torque + Contact_torque;

            if (rd_.control_time_ >= tc.command_time + task_time1 + task_time2 + task_time2 + task_time3)
            {
                task_state_init = true;
                task_state_n = 4; //지금은 4가 끝
            }
        }

        file[3] << rd_.control_time_
                << "\t" << rd_.link_[COM_id].x_traj(0) << "\t" << rd_.link_[COM_id].x_traj(1) << "\t" << rd_.link_[COM_id].x_traj(2)
                << "\t" << rd_.link_[COM_id].xpos(0) << "\t" << rd_.link_[COM_id].xpos(1) << "\t" << rd_.link_[COM_id].xpos(2)
                << "\t" << rd_.link_[COM_id].v_traj(0) << "\t" << rd_.link_[COM_id].v_traj(1) << "\t" << rd_.link_[COM_id].v_traj(2)
                << "\t" << rd_.link_[COM_id].v(0) << "\t" << rd_.link_[COM_id].v(1) << "\t" << rd_.link_[COM_id].v(2)
                // << "\t" << rd_.link_[Right_Foot].x_traj(0) << "\t" << rd_.link_[Right_Foot].x_traj(1) << "\t" << rd_.link_[Right_Foot].x_traj(2)
                // << "\t" << rd_.link_[Right_Foot].xpos(0) << "\t" << rd_.link_[Right_Foot].xpos(1) << "\t" << rd_.link_[Right_Foot].xpos(2)
                // << "\t" << rd_.link_[Left_Foot].x_traj(0) << "\t" << rd_.link_[Left_Foot].x_traj(1) << "\t" << rd_.link_[Left_Foot].x_traj(2)
                // << "\t" << rd_.link_[Left_Foot].xpos(0) << "\t" << rd_.link_[Left_Foot].xpos(1) << "\t" << rd_.link_[Left_Foot].xpos(2)
                << endl;
    }
    else if (tc.mode == 14)
    { //real walking!!
        task_torque.setZero();
        Contact_torque.setZero();
        prestate = state;
        /////////////////////////////////////////initialize/////////////////////////////
        if (task_state_init == true)
        {
            wbc_.set_contact(rd_, 1, 1);
            rd_.link_[Pelvis].rot_desired = Matrix3d::Identity();
            rd_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
            rd_.link_[Right_Hand].x_traj_local = rd_.link_[Upper_Body].rot_init.transpose() * (rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos);
            rd_.link_[Left_Hand].x_traj_local = rd_.link_[Upper_Body].rot_init.transpose() * (rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos);
            RH_R_init_local = rd_.link_[Upper_Body].rot_init.transpose() * rd_.link_[Right_Hand].Rotm;
            LH_R_init_local = rd_.link_[Upper_Body].rot_init.transpose() * rd_.link_[Left_Hand].Rotm;

            rd_.link_[Right_Hand].v_traj_local.setZero();
            rd_.link_[Left_Hand].v_traj_local.setZero();

            initial_lfoot = rd_.link_[Left_Foot].xpos;
            initial_rfoot = rd_.link_[Right_Foot].xpos;
            rfoot_loop_init = rd_.link_[Right_Foot].xpos;
            lfoot_loop_init = rd_.link_[Left_Foot].xpos;
            pelvis_loop_init = rd_.link_[Pelvis].xpos;
            rfoot2pel_init = rd_.link_[Pelvis].xpos - rd_.link_[Right_Foot].xpos;
            lfoot2pel_init = rd_.link_[Pelvis].xpos - rd_.link_[Left_Foot].xpos;
            Foot_init = rd_.link_[Left_Foot].xpos;
            // x_dot_desired.resize(21);
            // x_dot_desired.setZero();
            // x_dot_desired_pre.resize(21);
            // x_dot_desired_pre.setZero();
            x_ddot_desired.resize(21);
            x_ddot_desired.setZero();
            x_ddot_desired_pre.resize(21);
            x_ddot_desired_pre.setZero();

            task_state_init = false;
        }
        /////////////////////////////////////////initialize/////////////////////////////
        /////////////////////////////////////////common traj//////////////////////////////
        rd_.link_[Pelvis].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + 1.0, false);
        rd_.link_[Upper_Body].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + 1.0, false);

        rd_.link_[Right_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].x_traj_local;
        rd_.link_[Right_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Right_Hand].v_traj_local;
        rd_.link_[Right_Hand].r_traj = rd_.link_[Upper_Body].rot_init * RH_R_init_local * rd_.link_[Right_Hand].r_traj_local;
        rd_.link_[Right_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Right_Hand].w_traj_local;

        rd_.link_[Left_Hand].x_traj = rd_.link_[Upper_Body].xpos + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].x_traj_local;
        rd_.link_[Left_Hand].v_traj = rd_.link_[Upper_Body].v + rd_.link_[Upper_Body].Rotm * rd_.link_[Left_Hand].v_traj_local;
        rd_.link_[Left_Hand].r_traj = rd_.link_[Upper_Body].rot_init * LH_R_init_local * rd_.link_[Left_Hand].r_traj_local;
        rd_.link_[Left_Hand].w_traj = rd_.link_[Upper_Body].rot_init * rd_.link_[Left_Hand].w_traj_local;
        /////////////////////////////////////////common traj//////////////////////////////
        if (walking_tickc <= 3)
        {
            for (int i = 0; i < 3; ++i)
            {
                pelvis_gap(i) = wkc_.PELV_trajectory_float.translation()(i) - rd_.link_[Pelvis].xpos(i);
            }
        }

        if (walking_tickc > 3)
        {
            if (contactModec == 1)
                state = 1;
            else if (contactModec == 2)
                state = 2;
            else if (contactModec == 3)
                state = 3;

            if(foot_stepc(current_step_numc, 6) < 0.5)
                cursupport = 0.0;
            else
                cursupport = 1.0;

            if(presupport == 1 && cursupport == 0){
                Foot_init = rd_.link_[Right_Foot].xpos;
                //cout << state << endl;
            }
            else if(presupport == 0 &&  cursupport == 1){
                Foot_init = rd_.link_[Left_Foot].xpos;
                //cout << state << endl;
            }

            if (prestate == 1 && state == 2)
            { //left foot contact
                rfoot_loop_init = rd_.link_[Right_Foot].xpos;
                pelvis_loop_init = rd_.link_[Pelvis].x_traj; //rd_.link_[Pelvis].xpos;
                //Foot_init = rd_.link_[Left_Foot].xpos;
                step_cnt += 1.0;
            }
            else if (prestate == 1 && state == 3)
            { //right foot contact
                lfoot_loop_init = rd_.link_[Left_Foot].xpos;
                pelvis_loop_init = rd_.link_[Pelvis].x_traj; //rd_.link_[Pelvis].xpos;
                //Foot_init = rd_.link_[Right_Foot].xpos;
                step_cnt += 1.0;
            }
            else if (prestate != 1 && state == 1)
            { //double support
            }
            ///////////////////////////////////Contact state//////////////////////////////////
            if (state == 1)
            { //DSP
                //state = 1;
                wbc_.set_contact(rd_, 1, 1);
                task_number = 9 + 12;

                rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                rd_.f_star.setZero(task_number);
                x_ddot_desired.resize(task_number);
                x_ddot_desired.setZero();
                rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac;
                rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;
                rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;
                rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;

                //rd_.link_[Pelvis].x_traj(2) = wkc_.PELV_trajectory_float.translation()(2) - pelvis_gap(2);
                //if (prefoot == true)
                if(cursupport == 1)
                { //leftfoot contact
                    rd_.link_[Pelvis].x_traj(0) = wkc_.PELV_trajectory_float.translation()(0) - pelvis_gap(0) - Foot_init(0);//initial_lfoot(0);
                    rd_.link_[Pelvis].x_traj(1) = wkc_.PELV_trajectory_float.translation()(1) - pelvis_gap(1) - Foot_init(1);//initial_lfoot(1);
                    rd_.link_[Pelvis].x_traj(2) = wkc_.PELV_trajectory_float.translation()(2) - pelvis_gap(2) - Foot_init(2);//initial_lfoot(2); // - pelvis_gap(2);
                }
                else
                {
                    rd_.link_[Pelvis].x_traj(0) = wkc_.PELV_trajectory_float.translation()(0) - pelvis_gap(0) - Foot_init(0);//initial_rfoot(0);
                    rd_.link_[Pelvis].x_traj(1) = wkc_.PELV_trajectory_float.translation()(1) - pelvis_gap(1) - Foot_init(1);//initial_rfoot(1);
                    rd_.link_[Pelvis].x_traj(2) = wkc_.PELV_trajectory_float.translation()(2) - pelvis_gap(2) - Foot_init(2);//initial_rfoot(2); // - pelvis_gap(2);
                }
                rd_.link_[Pelvis].v_traj = wkc_.PELVD_trajectory_float.translation();
                rd_.link_[Pelvis].v_traj(2) = 0.0;

                zmp_x = zmp_refxc(walking_tickc);
                zmp_y = zmp_refyc(walking_tickc);
                x_ddot_desired(0) = lipm_wc * lipm_wc * (rd_.link_[Pelvis].x_traj(0) - zmp_x);
                x_ddot_desired(1) = lipm_wc * lipm_wc * (rd_.link_[Pelvis].x_traj(1) - zmp_y);

                // rd_.f_star(0) = Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - Foot_init(0))) + Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0)); //.segment(0, 3) = wbc_.getfstar_tra(rd_, Pelvis, Kp_com, Kd_com, 0.3*Kp_com, 0.3*Kd_com);
                // rd_.f_star(1) = Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - Foot_init(1))) + Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
                // rd_.f_star(2) = Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - Foot_init(2))) + Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
                if(cursupport == 1){
                    rd_.f_star(0) = Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - rd_.link_[Left_Foot].xpos(0))) + Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0)); //.segment(0, 3) = wbc_.getfstar_tra(rd_, Pelvis, Kp_com, Kd_com, 0.3*Kp_com, 0.3*Kd_com);
                    rd_.f_star(1) = Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - rd_.link_[Left_Foot].xpos(1))) + Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
                    rd_.f_star(2) = Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - rd_.link_[Left_Foot].xpos(2))) + Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
                }
                else{
                    rd_.f_star(0) = Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - rd_.link_[Right_Foot].xpos(0))) + Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0)); //.segment(0, 3) = wbc_.getfstar_tra(rd_, Pelvis, Kp_com, Kd_com, 0.3*Kp_com, 0.3*Kd_com);
                    rd_.f_star(1) = Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - rd_.link_[Right_Foot].xpos(1))) + Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
                    rd_.f_star(2) = Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - rd_.link_[Right_Foot].xpos(2))) + Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
                }
                rd_.f_star.segment(3, 3) = wbc_.getfstar_rot(rd_, Pelvis, 0.3 * Kp_com, 0.3 * Kd_com);
                rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body, Kp_ub, Kd_ub);
                rd_.f_star.segment(9, 6) = wbc_.getfstar6d(rd_, Right_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
                rd_.f_star.segment(15, 6) = wbc_.getfstar6d(rd_, Left_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
            }
            else if (state == 2)
            { //Leftfoot_contact && Rightfoot_swing
                //state = 2;
                wbc_.set_contact(rd_, 1, 0);
                task_number = 9 + 6 + 12;
                //prefoot = true;
                initial_lfoot = rd_.link_[Left_Foot].xpos;

                rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                rd_.f_star.setZero(task_number);
                x_ddot_desired.resize(task_number);
                x_ddot_desired.setZero();
                rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac;
                rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;
                rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;
                rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;
                rd_.J_task.block(21, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Foot].Jac;

                rd_.link_[Pelvis].x_traj(0) = wkc_.PELV_trajectory_float.translation()(0) - pelvis_gap(0) - Foot_init(0);//initial_lfoot(0);
                rd_.link_[Pelvis].x_traj(1) = wkc_.PELV_trajectory_float.translation()(1) - pelvis_gap(1) - Foot_init(1);//initial_lfoot(1);
                rd_.link_[Pelvis].x_traj(2) = wkc_.PELV_trajectory_float.translation()(2) - pelvis_gap(2) - Foot_init(2);//initial_lfoot(2);
                rd_.link_[Pelvis].v_traj = wkc_.PELVD_trajectory_float.translation();
                //rd_.link_[Pelvis].v_traj(2) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + t_rest_tempc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2, pelvis_loop_init(2), 0.0, 0.0, lfoot2pel_init(2) + initial_lfoot(2), 0.0, 0.0)(1) * walkingHz;

                rd_.link_[Right_Foot].x_traj = wkc_.RF_trajectory_float.translation(); // - estimate_rgap;
                rd_.link_[Right_Foot].v_traj = wkc_.RFD_trajectory_float.translation();
                if (walking_tickc < t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0)
                {
                    rd_.link_[Right_Foot].x_traj(2) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + t_rest_tempc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2, rfoot_loop_init(2), 0.0, 0.0, rfoot_loop_init(2) + 0.03, 0.0, 0.0)(0);
                    rd_.link_[Right_Foot].v_traj(2) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + t_rest_tempc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2, rfoot_loop_init(2), 0.0, 0.0, rfoot_loop_init(2) + 0.03, 0.0, 0.0)(1) * walkingHz;
                    x_ddot_desired(23) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + t_rest_tempc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2, rfoot_loop_init(2), 0.0, 0.0, rfoot_loop_init(2) + 0.03, 0.0, 0.0)(2) * walkingHz * walkingHz;
                } // the period for lifting the right foot
                else
                {
                    rd_.link_[Right_Foot].x_traj(2) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0, t_startc + t_totalc - t_rest_lastc - t_double2c - t_impc - t_rest_tempc, rfoot_loop_init(2) + 0.03, 0.0, 0.0, rfoot_loop_init(2), 0.0, 0.0)(0);
                    rd_.link_[Right_Foot].v_traj(2) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0, t_startc + t_totalc - t_rest_lastc - t_double2c - t_impc - t_rest_tempc, rfoot_loop_init(2) + 0.03, 0.0, 0.0, rfoot_loop_init(2), 0.0, 0.0)(1) * walkingHz;
                    x_ddot_desired(23) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0, t_startc + t_totalc - t_rest_lastc - t_double2c - t_impc - t_rest_tempc, rfoot_loop_init(2) + 0.03, 0.0, 0.0, rfoot_loop_init(2), 0.0, 0.0)(2) * walkingHz * walkingHz;
                } // the period for putting the right foot
                rd_.link_[Right_Foot].r_traj = Matrix3d::Identity();
                rd_.link_[Right_Foot].w_traj.setZero();

                zmp_x = zmp_refxc(walking_tickc);
                zmp_y = zmp_refyc(walking_tickc);
                x_ddot_desired(0) = lipm_wc * lipm_wc * (rd_.link_[Pelvis].x_traj(0) - zmp_x);
                x_ddot_desired(1) = lipm_wc * lipm_wc * (rd_.link_[Pelvis].x_traj(1) - zmp_y);

                // rd_.f_star(0) = 0.33*Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - Foot_init(0))) + 0.33*Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0)); //.segment(0, 3) = wbc_.getfstar_tra(rd_, Pelvis, Kp_com, Kd_com, 0.3*Kp_com, 0.3*Kd_com);
                // rd_.f_star(1) = 0.33*Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - Foot_init(1))) + 0.33*Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
                // rd_.f_star(2) = 0.33*Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - Foot_init(2))) + 0.33*Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
                rd_.f_star(0) = 0.33*Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - rd_.link_[Left_Foot].xpos(0))) + 0.33*Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0)); //.segment(0, 3) = wbc_.getfstar_tra(rd_, Pelvis, Kp_com, Kd_com, 0.3*Kp_com, 0.3*Kd_com);
                rd_.f_star(1) = 0.33*Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - rd_.link_[Left_Foot].xpos(1))) + 0.33*Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
                rd_.f_star(2) = 0.33*Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - rd_.link_[Left_Foot].xpos(2))) + 0.33*Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
                rd_.f_star.segment(3, 3) = wbc_.getfstar_rot(rd_, Pelvis, 0.3 * Kp_com, 0.3 * Kd_com);
                rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body, Kp_ub, Kd_ub);
                rd_.f_star.segment(9, 6) = wbc_.getfstar6d(rd_, Right_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
                rd_.f_star.segment(15, 6) = wbc_.getfstar6d(rd_, Left_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
                rd_.f_star.segment(21, 6) = wbc_.getfstar6d(rd_, Right_Foot, Kp_foot, Kd_foot, Kp_foot_rot, Kd_foot_rot);
            }
            else if (state == 3)
            { //Rightfoot_contact && Leftfoot_swing
                //state = 3;
                wbc_.set_contact(rd_, 0, 1);
                task_number = 9 + 6 + 12;
                //prefoot = false;
                initial_rfoot = rd_.link_[Right_Foot].xpos;

                rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                rd_.f_star.setZero(task_number);
                x_ddot_desired.resize(task_number);
                x_ddot_desired.setZero();
                rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac;
                rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;
                rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac;
                rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac;
                rd_.J_task.block(21, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Foot].Jac;

                rd_.link_[Pelvis].x_traj(0) = wkc_.PELV_trajectory_float.translation()(0) - pelvis_gap(0) - Foot_init(0);//initial_rfoot(0);
                rd_.link_[Pelvis].x_traj(1) = wkc_.PELV_trajectory_float.translation()(1) - pelvis_gap(1) - Foot_init(1);//initial_rfoot(1);
                rd_.link_[Pelvis].x_traj(2) = wkc_.PELV_trajectory_float.translation()(2) - pelvis_gap(2) - Foot_init(2);//initial_rfoot(2);
                rd_.link_[Pelvis].v_traj = wkc_.PELVD_trajectory_float.translation();
                rd_.link_[Pelvis].v_traj(2) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + t_rest_tempc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2, pelvis_loop_init(2), 0.0, 0.0, rfoot2pel_init(2) + initial_rfoot(2), 0.0, 0.0)(1);

                rd_.link_[Left_Foot].x_traj = wkc_.LF_trajectory_float.translation(); // - estimate_lgap;
                rd_.link_[Left_Foot].v_traj = wkc_.LFD_trajectory_float.translation();
                if (walking_tickc < t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0)
                {
                    rd_.link_[Left_Foot].x_traj(2) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + t_rest_tempc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0, lfoot_loop_init(2), 0.0, 0.0, lfoot_loop_init(2) + 0.03, 0.0, 0.0)(0);
                    rd_.link_[Left_Foot].v_traj(2) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + t_rest_tempc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0, lfoot_loop_init(2), 0.0, 0.0, lfoot_loop_init(2) + 0.03, 0.0, 0.0)(1) * walkingHz;
                    x_ddot_desired(23) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + t_rest_tempc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0, lfoot_loop_init(2), 0.0, 0.0, lfoot_loop_init(2) + 0.03, 0.0, 0.0)(2) * walkingHz * walkingHz;
                } // the period for lifting the left foot
                else
                {
                    rd_.link_[Left_Foot].x_traj(2) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0, t_startc + t_totalc - t_rest_lastc - t_double2c - t_impc - t_rest_tempc, lfoot_loop_init(2) + 0.03, 0.0, 0.0, lfoot_loop_init(2), 0.0, 0.0)(0);
                    rd_.link_[Left_Foot].v_traj(2) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0, t_startc + t_totalc - t_rest_lastc - t_double2c - t_impc - t_rest_tempc, lfoot_loop_init(2) + 0.03, 0.0, 0.0, lfoot_loop_init(2), 0.0, 0.0)(1) * walkingHz;
                    x_ddot_desired(23) = DyrosMath::QuinticSpline(walking_tickc, t_start_realc + t_double1c + (t_totalc - t_rest_initc - t_rest_lastc - t_double1c - t_double2c - t_impc) / 2.0, t_startc + t_totalc - t_rest_lastc - t_double2c - t_impc - t_rest_tempc, lfoot_loop_init(2) + 0.03, 0.0, 0.0, lfoot_loop_init(2), 0.0, 0.0)(1) * walkingHz * walkingHz;
                } // the period for putting the left foot
                rd_.link_[Left_Foot].r_traj = Matrix3d::Identity();
                rd_.link_[Left_Foot].w_traj.setZero();

                zmp_x = zmp_refxc(walking_tickc);
                zmp_y = zmp_refyc(walking_tickc);
                x_ddot_desired(0) = lipm_wc * lipm_wc * (rd_.link_[Pelvis].x_traj(0) - zmp_x);
                x_ddot_desired(1) = lipm_wc * lipm_wc * (rd_.link_[Pelvis].x_traj(1) - zmp_y);

                // rd_.f_star(0) = 0.33*Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - Foot_init(0))) + 0.33*Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0)); //.segment(0, 3) = wbc_.getfstar_tra(rd_, Pelvis, Kp_com, Kd_com, 0.3*Kp_com, 0.3*Kd_com);
                // rd_.f_star(1) = 0.33*Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - Foot_init(1))) + 0.33*Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
                // rd_.f_star(2) = 0.33*Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - Foot_init(2))) + 0.33*Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
                rd_.f_star(0) = 0.33*Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - rd_.link_[Right_Foot].xpos(0))) + 0.33*Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0)); //.segment(0, 3) = wbc_.getfstar_tra(rd_, Pelvis, Kp_com, Kd_com, 0.3*Kp_com, 0.3*Kd_com);
                rd_.f_star(1) = 0.33*Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - rd_.link_[Right_Foot].xpos(1))) + 0.33*Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
                rd_.f_star(2) = 0.33*Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - rd_.link_[Right_Foot].xpos(2))) + 0.33*Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
                rd_.f_star.segment(3, 3) = wbc_.getfstar_rot(rd_, Pelvis, 0.3 * Kp_com, 0.3 * Kd_com);
                rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body, Kp_ub, Kd_ub);
                rd_.f_star.segment(9, 6) = wbc_.getfstar6d(rd_, Right_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
                rd_.f_star.segment(15, 6) = wbc_.getfstar6d(rd_, Left_Hand, Kp_hand, Kd_hand, Kp_hand_rot, Kd_hand_rot);
                rd_.f_star.segment(21, 6) = wbc_.getfstar6d(rd_, Left_Foot, Kp_foot, Kd_foot, Kp_foot_rot, Kd_foot_rot);
            }

            FF_torque = wbc_.task_control_torque_motor(rd_, rd_.J_task, x_ddot_desired);

            task_torque = wbc_.task_control_torque_motor(rd_, rd_.J_task, rd_.f_star);
            //task_torque = wbc_.task_control_torque_with_gravity(rd_, rd_.J_task, rd_.f_star);
        }
        ///////////////////////////////////Contact state//////////////////////////////////
        /////////////////////////gravity/////////////////////컨택 전환 뒤!
        gravity_torque = wbc_.gravity_compensation_torque(rd_, false, false);
        ///////////////////////////gravity/////////////////////////////
        ///////////////////////////////////////////////contact force/////////////////////////////
        if (walking_tickc > 3)
        {
            if (state == 1)
            {
                if (phaseChangec == true && phaseChangec1 == false)
                {
                    fc_ratio = DyrosMath::cubic(walking_tickc, double2Single_prec, double2Singlec, 1.0, 0.0, 0.0, 0.0);
                    Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, foot_stepc(current_step_numc, 6));
                    FF_contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, FF_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, foot_stepc(current_step_numc, 6));
                }
                else if (phaseChangec == false && phaseChangec1 == true)
                {
                    fc_ratio = DyrosMath::cubic(walking_tickc, single2Double_prec, single2Doublec, 0.0, 1.0, 0.0, 0.0);
                    if (current_step_numc < total_step_numc - 1)
                    {
                        Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, foot_stepc(current_step_numc, 6));
                        FF_contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, FF_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, foot_stepc(current_step_numc, 6));
                    }
                    else
                    {
                        Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, foot_stepc(total_step_numc - 1, 6));
                        FF_contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, FF_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, foot_stepc(total_step_numc - 1, 6));
                    }
                }
                else
                {
                    fc_ratio = 1.0;
                    Contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, task_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, foot_stepc(current_step_numc, 6));
                    FF_contact_torque = wbc_.contact_force_redistribution_torque_walking(rd_, FF_torque + gravity_torque, contact_force_re, eta_data, fc_ratio, foot_stepc(current_step_numc, 6));
                }
            }
            else
            {
                fc_ratio = 0.0;
                Contact_torque.setZero();
                FF_contact_torque.setZero();
            }
        }
        ///////////////////////////////////////////////contact force/////////////////////////////
        total_torque = task_torque + Contact_torque + gravity_torque;
        FF_total_torque = FF_torque + FF_contact_torque + gravity_torque;

        ControlVal_ = task_torque + Contact_torque + gravity_torque;
        //ControlVal_ = gravity_torque;

        //ControlVal_.setZero();
        if (walking_tickc > 3)
        {
            file[3] << rd_.link_[Pelvis].x_traj(0) << "\t" << rd_.link_[Pelvis].x_traj(1) << "\t" << rd_.link_[Pelvis].x_traj(2)
                    << "\t" << rd_.link_[Pelvis].xpos(0) - Foot_init(0) << "\t" << rd_.link_[Pelvis].xpos(1) - Foot_init(1) << "\t" << rd_.link_[Pelvis].xpos(2) - Foot_init(2)
                    << "\t" << rd_.f_star(0) << "\t" << rd_.f_star(1) << "\t" << rd_.f_star(2)
                    << "\t" << rd_.link_[Pelvis].v_traj(0) << "\t" << rd_.link_[Pelvis].v_traj(1) << "\t" << rd_.link_[Pelvis].v_traj(2)
                    << "\t" << rd_.link_[Pelvis].v(0) << "\t" << rd_.link_[Pelvis].v(1) << "\t" << rd_.link_[Pelvis].v(2)
                    << "\t" << foot_stepc(current_step_numc, 6) << "\t" << cursupport << "\t" << presupport
                    // << "\t" << rd_.link_[Right_Foot].x_traj(0) << "\t" << rd_.link_[Right_Foot].x_traj(1) << "\t" << rd_.link_[Right_Foot].x_traj(2)
                    // << "\t" << rd_.link_[Right_Foot].xpos(0) << "\t" << rd_.link_[Right_Foot].xpos(1) << "\t" << rd_.link_[Right_Foot].xpos(2)
                    // << "\t" << rd_.link_[Left_Foot].x_traj(0) << "\t" << rd_.link_[Left_Foot].x_traj(1) << "\t" << rd_.link_[Left_Foot].x_traj(2)
                    // << "\t" << rd_.link_[Left_Foot].xpos(0) << "\t" << rd_.link_[Left_Foot].xpos(1) << "\t" << rd_.link_[Left_Foot].xpos(2)
                    //<< "\t" << x_ddot_desired(0) << "\t" << x_ddot_desired(1) << "\t" << rd_.f_star(0) << "\t" << rd_.f_star(1)
                    << "\t" << fc_ratio << "\t" << state << "\t" << prestate << "\t" << contactModec
                    << endl;

            file[2] << total_torque(5) << "\t" << total_torque(11) << "\t" << task_torque(5)
                    << "\t" << task_torque(11) << "\t" << Contact_torque(5) << "\t" << Contact_torque(11)
                    << "\t" << gravity_torque(5) << "\t" << gravity_torque(11) << "\t" << FF_total_torque(5)
                    << "\t" << FF_total_torque(11) << "\t" << FF_torque(5) << "\t" << FF_torque(11)
                    // << "\t" << total_torque(9) << "\t" << total_torque(10) << "\t" << total_torque(11)
                    // << "\t" << task_torque(0) << "\t" << task_torque(1) << "\t" << task_torque(2)
                    // << "\t" << task_torque(3) << "\t" << task_torque(4) << "\t" << task_torque(5)
                    // << "\t" << Contact_torque(0) << "\t" << Contact_torque(1) << "\t" << Contact_torque(2)
                    // << "\t" << Contact_torque(3) << "\t" << Contact_torque(4) << "\t" << Contact_torque(5)
                    // << "\t" << gravity_torque(0) << "\t" << gravity_torque(1) << "\t" << gravity_torque(2)
                    // << "\t" << gravity_torque(3) << "\t" << gravity_torque(4) << "\t" << gravity_torque(5)
                    // << "\t" << total_torque(30) << "\t" << total_torque(31) << "\t" << total_torque(32)
                    << endl;
        }

        if(walking_tickc > 3){
            walking_tickc_pre = walking_tickc;
            presupport = cursupport;
        }
    }
}

void CustomController::computeFast()
{
    if (tc.mode == 10)
    {
    }
    else if (tc.mode == 11)
    {
    }
}

void CustomController::computePlanner()
{
    if (tc.mode == 14)
    {
        tc.walking_enable = 1;

        if (tc.task_init == true)
        {
            wkc_.setRobotStateInitialize(rd_);
            wkc_.getUiWalkingParameter(walkingHz, tc.walking_enable, tc.ik_mode, tc.walking_pattern, tc.walking_pattern2, tc.foot_step_dir, tc.target_x, tc.target_y, tc.target_z, tc.theta, tc.height, tc.step_length_x, tc.step_length_y, tc.dob, tc.imu_walk, false, 0, rd_);

            for (int i = 12; i < MODEL_DOF; i++)
            {
                wkc_.desired_init_leg_q(i) = rd_.q_(i);
            }

            tc.task_init = false;
        }

        wkc_.walkingCompute(rd_);

        mtx_wlk.lock();
        total_step_numc = wkc_.total_step_num;
        current_step_numc = wkc_.current_step_num;
        walking_tickc = wkc_.walking_tick;
        foot_stepc = wkc_.foot_step;
        contactModec = wkc_.contactMode;
        phaseChangec = wkc_.phaseChange;
        double2Single_prec = wkc_.double2Single_pre;
        double2Singlec = wkc_.double2Single;
        single2Doublec = wkc_.single2Double;
        single2Double_prec = wkc_.single2Double_pre;
        phaseChangec1 = wkc_.phaseChange1;
        t_start_realc = wkc_.t_start_real;
        t_double1c = wkc_.t_double1;
        t_rest_tempc = wkc_.t_rest_temp;
        t_totalc = wkc_.t_total;
        t_rest_initc = wkc_.t_rest_init;
        t_rest_lastc = wkc_.t_rest_last;
        t_double2c = wkc_.t_double2;
        t_startc = wkc_.t_start;
        t_impc = wkc_.t_imp;
        t_tempc = wkc_.t_temp;
        if (walking_tickc <= 3)
        {
            zmp_refxc.resize((t_totalc * (total_step_numc + 1) + t_tempc - 1));
            zmp_refyc.resize((t_totalc * (total_step_numc + 1) + t_tempc - 1));
            zmp_refxc = wkc_.zmp_refx;
            zmp_refyc = wkc_.zmp_refy;
            lipm_wc = wkc_.lipm_w;
        }

        mtx_wlk.unlock();
    }
}

Eigen::MatrixXd CustomController::getCmatrix(Eigen::VectorQVQd qq, Eigen::VectorVQd qdot)
{wbc_.set_contact(rd_, 0, 1);
    Eigen::VectorVQd q = Quar2Eul(qq);
    double h = 2e-11;
    Eigen::VectorVQd q_new = q;
    Eigen::MatrixXd C(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    Eigen::MatrixXd C1(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    C1.setZero();
    Eigen::MatrixXd C2(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    C2.setZero();
    Eigen::MatrixXd H_origin(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL), H_new(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    H_origin.setZero();
    H_new.setZero();
    Eigen::MatrixXd m[MODEL_DOF_VIRTUAL];
    double b[MODEL_DOF_VIRTUAL][MODEL_DOF_VIRTUAL][MODEL_DOF_VIRTUAL];

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(rd_.model_virtual, q, H_origin, true);
    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
    {
        q_new = q;
        q_new(i) += h;
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(rd_.model_virtual, q_new, H_new, true);
        m[i].resize(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
        m[i] = (H_new - H_origin) / h;
    }

    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
        for (int j = 0; j < MODEL_DOF_VIRTUAL; j++)
            for (int k = 0; k < MODEL_DOF_VIRTUAL; k++)
                b[i][j][k] = 0.5 * (m[k](i, j) + m[j](i, k) - m[i](j, k));

    C.setZero();
    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
        for (int j = 0; j < MODEL_DOF_VIRTUAL; j++)
            C1(i, j) = b[i][j][j] * qdot(j);

    for (int k = 0; k < MODEL_DOF_VIRTUAL; k++)
        for (int j = 0; j < MODEL_DOF_VIRTUAL; j++)
            for (int i = 1 + j; i < MODEL_DOF_VIRTUAL; i++)
                C2(k, j) += 2.0 * b[k][j][i] * qdot(i);

    C = C1 + C2;

    return C;
}

Eigen::MatrixXd CustomController::getCmatrix(Eigen::VectorQd q, Eigen::VectorQd qdot)
{
}

Eigen::VectorVQd CustomController::Quar2Eul(Eigen::VectorQVQd q)
{
    Eigen::VectorVQd q_e;
    q_e.segment(0, MODEL_DOF_VIRTUAL) = q.segment(0, MODEL_DOF_VIRTUAL);
    double sinr_cosp = 2 * (q(MODEL_DOF_VIRTUAL) * q(3) + q(4) * q(5));
    double cosr_cosp = 1 - 2 * (q(3) * q(3) + q(4) * q(4));
    q_e(3) = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q(MODEL_DOF_VIRTUAL) * q(4) - q(5) * q(3));
    if (abs(sinp) >= 1)
        q_e(4) = copysign(PI / 2, sinp);
    else
        q_e(4) = asin(sinp);

    double siny_cosp = 2 * (q(MODEL_DOF_VIRTUAL) * q(5) + q(3) * q(4));
    double cosy_cosp = 1 - 2 * (q(4) * q(4) + q(5) * q(5));
    q_e(5) = atan2(siny_cosp, cosy_cosp);

    return q_e;
}
