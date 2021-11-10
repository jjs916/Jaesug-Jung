#include "cc.h"
#define PI 3.141592653589793238462643
#define controltype 2 //1:original 2:mosf 3:ik

using namespace TOCABI;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    ControlVal_.setZero();
    Task_torque.setZero();
    Gravity_torque.setZero();
    Extra_torque.setZero();
    Contact_torque.setZero();
    Total_torque.setZero();
    task_init = true;

    //motor_inertia.resize(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    motor_inertia.setZero();

    for (int i = 0; i < 3; ++i)
    {
        motor_inertia(i, i) = 94.0;
        motor_inertia(i + 3, i + 3) = 20.0;
    }
    motor_inertia(5, 5) = 10.0;

    for (int i = 0; i < 2; ++i)
    {
        motor_inertia(6 + 6 * i, 6 + 6 * i) = 0.56;
        motor_inertia(7 + 6 * i, 7 + 6 * i) = 0.8;
        motor_inertia(8 + 6 * i, 8 + 6 * i) = 1.08;
        motor_inertia(9 + 6 * i, 9 + 6 * i) = 1.08;
        motor_inertia(10 + 6 * i, 10 + 6 * i) = 1.08;
        motor_inertia(11 + 6 * i, 11 + 6 * i) = 0.306;

        motor_inertia(21 + 10 * i, 21 + 10 * i) = 0.185;
        motor_inertia(22 + 10 * i, 22 + 10 * i) = 0.184;
        motor_inertia(23 + 10 * i, 23 + 10 * i) = 0.192;
        motor_inertia(24 + 10 * i, 24 + 10 * i) = 0.184;
        motor_inertia(25 + 10 * i, 25 + 10 * i) = 0.056;
        motor_inertia(26 + 10 * i, 26 + 10 * i) = 0.05;
        motor_inertia(27 + 10 * i, 27 + 10 * i) = 0.015;
        motor_inertia(28 + 10 * i, 28 + 10 * i) = 0.015;
    }
    motor_inertia(18, 18) = 1.01;
    motor_inertia(19, 19) = 1.01;
    motor_inertia(20, 20) = 1.27;
    motor_inertia(29, 29) = 0.015;
    motor_inertia(30, 30) = 0.015;

    motor_inertia_inv = motor_inertia.llt().solve(MatrixXd::Identity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL));

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
        //real robot gains
        if(controltype == 1){
            Kp_com(i) = 50.0;//60.0;
            Kd_com(i) = 5.0;//5.0;
            Kp_com_rot(i) = 550.0;
            Kd_com_rot(i) = 5.0;
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
            Kp_com(i) = 110.0;
            Kd_com(i) = 7.0;//40 60 100
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
    //////simulation
    // Kp << 1800.0, 2100.0, 2100.0, 2100.0, 1800.0, 1800.0,           //left foot
    //     1800.0, 2100.0, 2100.0, 2100.0, 1800.0, 1800.0,             //right foot
    //     2200.0, 2200.0, 2200.0,                                  //waist
    //     400.0, 800.0, 400.0, 400.0, 250.0, 250.0, 50.0, 50.0, //left hand
    //     50.0, 50.0,                                           //neck
    //     400.0, 800.0, 400.0, 400.0, 250.0, 250.0, 50.0, 50.0; //right hand

    // Kd << 70.0, 90.0, 90.0, 90.0, 80.0, 80.0,
    //     70.0, 90.0, 90.0, 90.0, 80.0, 80.0,
    //     90.0, 90.0, 1590.0,
    //     10.0, 10.0, 10.0, 10.0, 2.5, 2.0, 2.0, 2.0,
    //     2.0, 2.0,
    //     10.0, 10.0, 10.0, 10.0, 2.5, 2.0, 2.0, 2.0;
    /////real gain
    Kp << 2000.0, 5000.0, 4000.0, 3700.0, 5000.0, 5000.0,           //left foot
        2000.0, 5000.0, 4000.0, 3700.0, 5000.0, 5000.0,             //right foot
        6000.0, 10000.0, 10000.0,                                  //waist
        400.0, 800.0, 400.0, 400.0, 250.0, 250.0, 50.0, 50.0, //left hand
        50.0, 50.0,                                           //neck
        400.0, 800.0, 400.0, 400.0, 250.0, 250.0, 50.0, 50.0; //right hand

    Kd << 15.0, 50.0, 20.0, 25.0, 30.0, 30.0,
        15.0, 50.0, 20.0, 25.0, 30.0, 30.0,
        200.0, 100.0, 100.0,
        10.0, 10.0, 10.0, 10.0, 2.5, 2.0, 2.0, 2.0,
        2.0, 2.0,
        10.0, 10.0, 10.0, 10.0, 2.5, 2.0, 2.0, 2.0;

    file[0].open(FILE_NAMES[5].c_str());
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

// void CustomController::taskCommandToCC(TaskCommand tc_)
// {
//     tc = tc_;
// }

void CustomController::computeSlow()
{
    if(rd_.tc_.mode == 10)
    {
        WBC::SetContact(rd_, 1, 1);

        if (task_init)
        {
            rd_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
            
            cout << "DSP1" << endl;
            task_number = 6 + 3 + 6 + 6;
            f_star.resize(task_number);
            f_star.setZero();
            rd_.J_task.resize(task_number, MODEL_DOF_VIRTUAL);
            rd_.J_task.setZero();
            task_time1 = 3.0;

            for (int i = 0; i < 3; ++i)
            {
                //realrobot-mosf
                // Kp_com(i) = 90.0;
                // Kd_com(i) = 5.0; //40 60 100
                // Kp_com_rot(i) = 100.0;
                // Kd_com_rot(i) = 1.0;
                // Kp_ub(i) = 50.0;
                // Kd_ub(i) = 1.0;
                // Kp_hand(i) = 100.0;
                // Kd_hand(i) = 5.0;
                // Kp_hand_rot(i) = 100.0;
                // Kd_hand_rot(i) = 5.0;
                // Kp_foot(i) = 400.0;
                // Kd_foot(i) = 40.0;
                // Kp_foot_rot(i) = 100.0;
                // Kd_foot_rot(i) = 5.0;
                //realrobot-osf
                // Kp_com(i) = 700.0;
                // Kd_com(i) = 6.0; //40 60 100
                // Kp_com_rot(i) = 100.0;
                // Kd_com_rot(i) = 1.0;
                // Kp_ub(i) = 50.0;
                // Kd_ub(i) = 1.0;
                // Kp_hand(i) = 100.0;
                // Kd_hand(i) = 5.0;
                // Kp_hand_rot(i) = 100.0;
                // Kd_hand_rot(i) = 5.0;
                // Kp_foot(i) = 400.0;
                // Kd_foot(i) = 40.0;
                // Kp_foot_rot(i) = 100.0;
                // Kd_foot_rot(i) = 5.0;
                //simulation-mosf
                Kp_com(i) = 90.0;
                Kd_com(i) = 5.0; //40 60 100
                Kp_com_rot(i) = 200.0;
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

            rd_.link_[Right_Foot].SetGain(Kp_foot(0), Kd_foot(0), 0.0, Kp_foot_rot(0), Kd_foot_rot(0), 0.0);
            rd_.link_[Left_Foot].SetGain(Kp_foot(0), Kd_foot(0), 0.0, Kp_foot_rot(0), Kd_foot_rot(0), 0.0);
            rd_.link_[Right_Hand].SetGain(Kp_hand(0), Kd_hand(0), 0.0, Kp_hand_rot(0), Kd_hand_rot(0), 0.0);
            rd_.link_[Left_Hand].SetGain(Kp_hand(0), Kd_hand(0), 0.0, Kp_hand_rot(0), Kd_hand_rot(0), 0.0);
            rd_.link_[Pelvis].SetGain(Kp_com(0), Kd_com(0), 0.0, Kp_com_rot(0), Kd_com_rot(0), 0.0);
            rd_.link_[Upper_Body].SetGain(0.0, 0.0, 0.0, Kp_ub(0), Kd_ub(0), 0.0);
            rd_.link_[COM_id].SetGain(Kp_com(0), Kd_com(0), 0.0, Kp_com_rot(0), Kd_com_rot(0), 0.0);

            COM_init = rd_.link_[Pelvis].xpos - rd_.link_[Right_Foot].xpos; //rd_.link_[Pelvis].xpos;//
            RH_init = rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos;
            LH_init = rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos;

            rd_.link_[Pelvis].x_desired = COM_init;//
            rd_.link_[Pelvis].x_desired(0) = COM_init(0) + rd_.tc_.l_x;//
            rd_.link_[Pelvis].x_desired(1) = COM_init(1) + rd_.tc_.l_y;//               //rd_.link_[Pelvis].xpos(1) + tc.l_y;//
            rd_.link_[Pelvis].x_desired(2) = COM_init(2) + rd_.tc_.l_z;//
            rd_.link_[Pelvis].rot_desired = Matrix3d::Identity();

            rd_.link_[Right_Hand].x_desired = RH_init;
            rd_.link_[Right_Hand].rot_desired = rd_.link_[Upper_Body].rot_desired;
            rd_.link_[Left_Hand].x_desired = LH_init;
            rd_.link_[Left_Hand].rot_desired = rd_.link_[Upper_Body].rot_desired;

            task_init = false;
        }

        rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac();
        rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);
        rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac();
        rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac();

        COM_pos_local = rd_.link_[Pelvis].xpos - rd_.link_[Right_Foot].xpos;
        RH_pos_local = rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos;
        LH_pos_local = rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos;
        COM_vel_local = rd_.link_[Pelvis].v - rd_.link_[Right_Foot].v;
        RH_vel_local = rd_.link_[Right_Hand].v - rd_.link_[Upper_Body].v;
        LH_vel_local = rd_.link_[Left_Hand].v - rd_.link_[Upper_Body].v;

        //Eigen::Vector3d test = rd_.link_[Pelvis].xpos - rd_.link_[14].xpos;
        //double rr = test.transpose()*test;
        //cout << sqrt(17000/(rd_.total_mass_*rr)) << endl;
        //cout << test << endl << endl;


        for (int i = 0; i < 3; ++i)
        {
            rd_.link_[Pelvis].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, COM_init(i), 0.0, 0.0, rd_.link_[Pelvis].x_desired(i), 0.0, 0.0)(0);
            rd_.link_[Pelvis].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, COM_init(i), 0.0, 0.0, rd_.link_[Pelvis].x_desired(i), 0.0, 0.0)(1);

            rd_.link_[Right_Hand].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, RH_init(i), 0.0, 0.0, RH_init(i), 0.0, 0.0)(0);
            rd_.link_[Right_Hand].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, RH_init(i), 0.0, 0.0, RH_init(i), 0.0, 0.0)(1);

            rd_.link_[Left_Hand].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, LH_init(i), 0.0, 0.0, LH_init(i), 0.0, 0.0)(0);
            rd_.link_[Left_Hand].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, LH_init(i), 0.0, 0.0, LH_init(i), 0.0, 0.0)(1);
        }
        rd_.link_[Pelvis].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
        rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        // f_star(0) = Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - rd_.link_[Right_Foot].xpos(0))) + Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0));
        // f_star(1) = Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - rd_.link_[Right_Foot].xpos(1))) + Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
        // f_star(2) = Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - rd_.link_[Right_Foot].xpos(2))) + Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
        f_star.segment(0, 3) = WBC::GetFstarPosJS(rd_.link_[Pelvis], rd_.link_[Pelvis].x_traj, COM_pos_local, rd_.link_[Pelvis].v_traj, COM_vel_local);
        f_star.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Pelvis]);
        f_star.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);
        // f_star(9) = Kp_hand(0) * (rd_.link_[Right_Hand].x_traj(0) - (rd_.link_[Right_Hand].xpos(0) - rd_.link_[Upper_Body].xpos(0))) + Kd_hand(0) * (rd_.link_[Right_Hand].v_traj(0) - rd_.link_[Upper_Body].v(0));
        // f_star(10) = Kp_hand(1) * (rd_.link_[Right_Hand].x_traj(1) - (rd_.link_[Right_Hand].xpos(1) - rd_.link_[Upper_Body].xpos(1))) + Kd_hand(1) * (rd_.link_[Right_Hand].v_traj(1) - rd_.link_[Upper_Body].v(1));
        // f_star(11) = Kp_hand(2) * (rd_.link_[Right_Hand].x_traj(2) - (rd_.link_[Right_Hand].xpos(2) - rd_.link_[Upper_Body].xpos(2))) + Kd_hand(2) * (rd_.link_[Right_Hand].v_traj(2) - rd_.link_[Upper_Body].v(2));
        f_star.segment(9, 3) = WBC::GetFstarPosJS(rd_.link_[Right_Hand], rd_.link_[Right_Hand].x_traj, RH_pos_local, rd_.link_[Right_Hand].v_traj, RH_vel_local);
        f_star.segment(12, 3) = WBC::GetFstarRot(rd_.link_[Right_Hand]);
        // f_star(15) = Kp_hand(0) * (rd_.link_[Left_Hand].x_traj(0) - (rd_.link_[Left_Hand].xpos(0) - rd_.link_[Upper_Body].xpos(0))) + Kd_hand(0) * (rd_.link_[Left_Hand].v_traj(0) - rd_.link_[Upper_Body].v(0));
        // f_star(16) = Kp_hand(1) * (rd_.link_[Left_Hand].x_traj(1) - (rd_.link_[Left_Hand].xpos(1) - rd_.link_[Upper_Body].xpos(1))) + Kd_hand(1) * (rd_.link_[Left_Hand].v_traj(1) - rd_.link_[Upper_Body].v(1));
        // f_star(17) = Kp_hand(2) * (rd_.link_[Left_Hand].x_traj(2) - (rd_.link_[Left_Hand].xpos(2) - rd_.link_[Upper_Body].xpos(2))) + Kd_hand(2) * (rd_.link_[Left_Hand].v_traj(2) - rd_.link_[Upper_Body].v(2));
        f_star.segment(15, 3) = WBC::GetFstarPosJS(rd_.link_[Left_Hand], rd_.link_[Left_Hand].x_traj, LH_pos_local, rd_.link_[Left_Hand].v_traj, LH_vel_local);
        f_star.segment(18, 3) = WBC::GetFstarRot(rd_.link_[Left_Hand]);
// const clock_t begin_time = clock();
        fc_ratio = 1.0;
        
        Gravity_torque = WBC::GravityCompensationTorque(rd_);
        Task_torque = WBC::TaskControlTorqueMotor(rd_, f_star, motor_inertia, motor_inertia_inv);
        Extra_torque = WBC::TaskControlTorqueExtra(rd_, f_star, motor_inertia, motor_inertia_inv);
        Test_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Extra_torque, fc_ratio, 0);//0이 오른발

        Eigen::VectorXd test_force;
        test_force = WBC::Forcecompute(rd_, Extra_torque, motor_inertia, motor_inertia_inv);
        //cout << "Torque"<< endl;
        //cout << test_force << endl << endl;
        //Task_torque = WBC::TaskControlTorque(rd_, f_star);
        //  file[0] << rd_.control_time_
        //  << "\t" << Extra_torque(0) + Test_torque(0) << "\t" << Extra_torque(1) + Test_torque(1) << "\t" << Extra_torque(2) + Test_torque(2) << "\t" << Extra_torque(3) + Test_torque(3) 
        //  << "\t" << Extra_torque(6) + Test_torque(6) << "\t" << Extra_torque(7) + Test_torque(7) << "\t" << Extra_torque(8) + Test_torque(8) << "\t" << Extra_torque(9) + Test_torque(9)
        //  << endl;
        file[0] << rd_.control_time_
         << "\t" << test_force(0) << "\t" << test_force(1) << "\t" << test_force(2) << "\t" << test_force(3) 
         << "\t" << test_force(4) << "\t" << test_force(5) << "\t" << test_force(6) << "\t" << test_force(7) 
         << endl;
        // if (rd_.control_time_ >= rd_.tc_time_ + task_time1 - 0.01) //only for going to SSP
        // {
        //     fc_ratio = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_ + task_time1 - 0.01, rd_.tc_time_ + task_time1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)(0);
        // }
        // else
        // {
        //     fc_ratio = 1.0;
        // }
        
        Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Task_torque + Gravity_torque + Extra_torque, fc_ratio, 0);//0이 오른발
        
        Total_torque = Gravity_torque + Task_torque + Contact_torque + Extra_torque;
        rd_.torque_desired = Total_torque;
        //rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));
    }
    else if (rd_.tc_.mode == 11)
    {
        WBC::SetContact(rd_, 1, 1);
        F_contact.resize(rd_.J_C.rows());

        if (task_init)
        {
            rd_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
            
            cout << "NEW!" << endl;
            task_number = 6 + 6 + 3;
            f_star.resize(task_number);
            f_star.setZero();
            
            rd_.J_task.resize(task_number, MODEL_DOF_VIRTUAL);
            rd_.J_task.setZero();
            task_time1 = 3.0;

            for (int i = 0; i < 3; ++i)
            {
                //simulation-mosf
                Kp_com(i) = 90.0;
                Kd_com(i) = 5.0; //40 60 100
                Kp_com_rot(i) = 200.0;
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

            rd_.link_[Right_Foot].SetGain(Kp_foot(0), Kd_foot(0), 0.0, Kp_foot_rot(0), Kd_foot_rot(0), 0.0);
            rd_.link_[Left_Foot].SetGain(Kp_foot(0), Kd_foot(0), 0.0, Kp_foot_rot(0), Kd_foot_rot(0), 0.0);
            rd_.link_[Right_Hand].SetGain(Kp_hand(0), Kd_hand(0), 0.0, Kp_hand_rot(0), Kd_hand_rot(0), 0.0);
            rd_.link_[Left_Hand].SetGain(Kp_hand(0), Kd_hand(0), 0.0, Kp_hand_rot(0), Kd_hand_rot(0), 0.0);
            rd_.link_[Pelvis].SetGain(Kp_com(0), Kd_com(0), 0.0, Kp_com_rot(0), Kd_com_rot(0), 0.0);
            rd_.link_[Upper_Body].SetGain(0.0, 0.0, 0.0, Kp_ub(0), Kd_ub(0), 0.0);
            rd_.link_[COM_id].SetGain(Kp_com(0), Kd_com(0), 0.0, Kp_com_rot(0), Kd_com_rot(0), 0.0);

            RF_init = rd_.link_[Right_Foot].xpos - rd_.link_[Pelvis].xpos;
            LF_init = rd_.link_[Left_Foot].xpos - rd_.link_[Pelvis].xpos;

            rd_.link_[Right_Foot].x_desired = RF_init;//
            rd_.link_[Left_Foot].x_desired = LF_init;//
            rd_.link_[Right_Foot].x_desired(0) = RF_init(0) + rd_.tc_.l_x;//
            rd_.link_[Right_Foot].x_desired(1) = RF_init(1) + rd_.tc_.l_y;//               //rd_.link_[Pelvis].xpos(1) + tc.l_y;//
            rd_.link_[Right_Foot].x_desired(2) = RF_init(2) + rd_.tc_.l_z;//
            rd_.link_[Left_Foot].x_desired(0) = LF_init(0) + rd_.tc_.l_x;//
            rd_.link_[Left_Foot].x_desired(1) = LF_init(1) + rd_.tc_.l_y;//               //rd_.link_[Pelvis].xpos(1) + tc.l_y;//
            rd_.link_[Left_Foot].x_desired(2) = LF_init(2) + rd_.tc_.l_z;//
            rd_.link_[Right_Foot].rot_desired = Matrix3d::Identity();
            rd_.link_[Left_Foot].rot_desired = Matrix3d::Identity();

            task_init = false;
        }

        rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Foot].Jac();
        rd_.J_task.block(6, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Foot].Jac();
        rd_.J_task.block(12, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

        F_contact.setZero();
        F_contact(2) = -rd_.total_mass_*9.81/10.0;
        F_contact(8) = -rd_.total_mass_*9.81/10.0;

        Gravity_torque = WBC::Vgravitytoruqe(rd_);
        Task_torque = WBC::newtasktorque(rd_, f_star, motor_inertia, motor_inertia_inv, F_contact);

        Total_torque = Gravity_torque + Task_torque;
        rd_.torque_desired = Total_torque;
    }
    else if (rd_.tc_.mode == 12)
    {
        if (rd_.tc_init)
        {
            //Initialize settings for Task Control! 

            rd_.tc_init = false;
            std::cout<<"cc mode 11"<<std::endl;

            //rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
        }

        WBC::SetContact(rd_, 1, 1);

        rd_.J_task.setZero(9, MODEL_DOF_VIRTUAL);
        rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac();
        rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

        rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
        rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

        rd_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithX(rd_.tc_.roll) * DyrosMath::rotateWithY(rd_.tc_.pitch) * DyrosMath::rotateWithZ(rd_.tc_.yaw + rd_.link_[Pelvis].yaw_init);

        Eigen::VectorXd fstar;
        rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        fstar.setZero(9);
        fstar.segment(0, 6) = WBC::GetFstar6d(rd_.link_[COM_id]);
        fstar.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

        rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));
    }
}

void CustomController::computeFast()
{
    // if (tc.mode == 10)
    // {
    // }
    // else if (tc.mode == 11)
    // {
    // }
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}