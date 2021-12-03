/*! @file LegController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#include "Controllers/LegController.h"

/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void LegControllerCommand<T>::zero() {
  tauFeedForward = Vec3<T>::Zero();
  forceFeedForward = Vec3<T>::Zero();
  qDes = Vec3<T>::Zero();
  qdDes = Vec3<T>::Zero();
  pDes = Vec3<T>::Zero();
  vDes = Vec3<T>::Zero();
  kpCartesian = Mat3<T>::Zero();
  kdCartesian = Mat3<T>::Zero();
  kpJoint = Mat3<T>::Zero();
  kdJoint = Mat3<T>::Zero();

  tauFeedForwardBias = Vec3<T>::Zero();
  kpJointBias = Mat3<T>::Zero();
  kdJointBias = Mat3<T>::Zero();
  //tauCompleteHardCode = Vec3<T>::Zero();
}

/*!
 * Zero the leg data
 */
template <typename T>
void LegControllerData<T>::zero() {
  q = Vec3<T>::Zero();
  qd = Vec3<T>::Zero();
  p = Vec3<T>::Zero();
  v = Vec3<T>::Zero();
  J = Mat3<T>::Zero();
  tauEstimate = Vec3<T>::Zero();

  // Custom
  tauFeedback = Vec3<T>::Zero();
  tauFeedforward = Vec3<T>::Zero();
  tauEstimateFromActuatorModel = Vec3<T>::Zero();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template <typename T>
void LegController<T>::zeroCommand() {
  for (auto& cmd : commands) {
    cmd.zero();
  }
  _legsEnabled = false;
}

/*!
 * Set the leg to edamp.  This overwrites all command data and generates an
 * emergency damp command using the given gain. For the mini-cheetah, the edamp
 * gain is Nm/(rad/s), and for the Cheetah 3 it is N/m. You still must call
 * updateCommand for this command to end up in the low-level command data!
 */
template <typename T>
void LegController<T>::edampCommand(RobotType robot, T gain) {
  zeroCommand();
  if (robot == RobotType::CHEETAH_3) {
    for (int leg = 0; leg < 4; leg++) {
      for (int axis = 0; axis < 3; axis++) {
        commands[leg].kdCartesian(axis, axis) = gain;
      }
    }
  } else {  // mini-cheetah
    for (int leg = 0; leg < 4; leg++) {
      for (int axis = 0; axis < 3; axis++) {
        commands[leg].kdJoint(axis, axis) = gain;
      }
    }
  }
}

/*!
 * Update the "leg data" from a SPIne board message
 */
template <typename T>
void LegController<T>::updateData(const SpiData* spiData) {
  for (int leg = 0; leg < 4; leg++) {
    // q:
    datas[leg].q(0) = spiData->q_abad[leg];
    datas[leg].q(1) = spiData->q_hip[leg];
    datas[leg].q(2) = spiData->q_knee[leg];

    // qd
    datas[leg].qd(0) = spiData->qd_abad[leg];
    datas[leg].qd(1) = spiData->qd_hip[leg];
    datas[leg].qd(2) = spiData->qd_knee[leg];

    // J and p
    computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J),
                                     &(datas[leg].p), leg);

    // v
    datas[leg].v = datas[leg].J * datas[leg].qd;
  }
}

/*!
 * Update the "leg data" from a TI Board message
 */
template <typename T>
void LegController<T>::updateData(const TiBoardData* tiBoardData) {
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      datas[leg].q(joint) = tiBoardData[leg].q[joint];
      datas[leg].qd(joint) = tiBoardData[leg].dq[joint];
      datas[leg].p(joint) = tiBoardData[leg].position[joint];
      datas[leg].v(joint) = tiBoardData[leg].velocity[joint];

      // J and p
      computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &datas[leg].J,
                                       nullptr, leg);
      datas[leg].tauEstimate[joint] = tiBoardData[leg].tau[joint];
    }
    //printf("%d leg, position: %f, %f, %f\n", leg, datas[leg].p[0], datas[leg].p[1], datas[leg].p[2]);
    //printf("%d leg, velocity: %f, %f, %f\n", leg, datas[leg].v[0], datas[leg].v[1], datas[leg].v[2]);
  }
}

/*!
 * Update the "leg command" for the SPIne board message
 */
template <typename T>
void LegController<T>::updateCommand(SpiCommand* spiCommand) {
  for (int leg = 0; leg < 4; leg++) {
    // tauFF
    Vec3<T> legTorque = commands[leg].tauFeedForward;

    // forceFF
    Vec3<T> footForce = commands[leg].forceFeedForward;

    // cartesian PD
    footForce +=
        commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
    footForce +=
        commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);

    // Torque
    legTorque += datas[leg].J.transpose() * footForce;

    // Custom2
    if (leg == 1)
    {
      // Safety code temp
      if ((datas[leg].q(0) > 0.4) || (datas[leg].q(0) < -0.4))
      {
        commands[leg].tauFeedForwardBias(0) = 0;
      }
      if ((datas[leg].q(1) > -0.5) || (datas[leg].q(1) < -1.1))
      {
        commands[leg].tauFeedForwardBias(1) = 0;
      }
      if ((datas[leg].q(2) > 1.93) || (datas[leg].q(2) < 1.33))
      {
        commands[leg].tauFeedForwardBias(2) = 0;
      }

      if(commands[leg].flag_hardcode_torque == 0)
      {
        // Change legTorque:
        legTorque(0) += commands[leg].tauFeedForwardBias(0);
        legTorque(1) += commands[leg].tauFeedForwardBias(1);
        legTorque(2) += commands[leg].tauFeedForwardBias(2);
      } else if(commands[leg].flag_hardcode_torque == 1)
      {
        // AUTO ORIGINAL CODE
        // // Hardcode legTorque:
        // legTorque(0) += commands[leg].tauFeedForwardBias(0);
        // legTorque(1) = -0.015;
        // legTorque(2) = -3.159;

        // // Hardcode PD
        // commands[leg].kdJoint(0, 0) = 0;
        // commands[leg].kdJoint(1, 1) = 0;
        // commands[leg].kdJoint(2, 2) = 0;

        // commands[leg].kpJoint(0, 0) = 0;
        // commands[leg].kpJoint(1, 1) = 0;
        // commands[leg].kpJoint(2, 2) = 0;

        // 3 feet standing testing
        // Hardcode legTorque:
        legTorque(0) += commands[leg].tauFeedForwardBias(0);
        legTorque(1) = 0;
        legTorque(2) = -1;

        // Hardcode PD
        commands[leg].kdJoint(0, 0) = 0;
        commands[leg].kdJoint(1, 1) = 0;
        commands[leg].kdJoint(2, 2) = 0;

        commands[leg].kpJoint(0, 0) = 0;
        commands[leg].kpJoint(1, 1) = 0;
        commands[leg].kpJoint(2, 2) = 0;

        // printf("(Custom outputs18) Leg0 tau_ff: %f, %f, %f, %f, %f, %f\n", 
        //       commands[leg].tauFeedForward[0], 
        //       commands[leg].tauFeedForward[1], 
        //       commands[leg].tauFeedForward[2],
        //       commands[leg].forceFeedForward[0],
        //       commands[leg].forceFeedForward[1],
        //       commands[leg].forceFeedForward[2]
        //       );
      }
      
      // joint space pd
      // joint space PD
      // spiCommand->kd_abad[leg] = 0;
      // spiCommand->kd_hip[leg] = 0;
      // spiCommand->kd_knee[leg] = 0;

      // spiCommand->kp_abad[leg] = 0;
      // spiCommand->kp_hip[leg] = 0;
      // spiCommand->kp_knee[leg] = 0;

      // Custom printing info
      // std::cout<< "(Custom outputs4) q: "<< datas[leg].q
      //           << "\n";
      // std::cout<< "(Custom outputs14) DEBUG: tauEstimate: "<< datas[leg].tauEstimate
      //           << "\n";  // 0.796523, -0.0149865, -3.15868
    }

    // For lateral force calibration (**MUST COMMENT WHEN NOT DOING LATERAL GRF CALIBRATION**)
    // if(commands[1].flag_hardcode_torque == 1) 
    // {
    //   if(leg == 1)
    //   {
    //     // 3 feet standing testing
    //     // Hardcode legTorque:
    //     legTorque(0) = 0.8 + commands[leg].tauFeedForwardBias(0);
    //     legTorque(1) = 0;
    //     legTorque(2) = 0;

    //     // Hardcode PD
    //     commands[leg].kdJoint(0, 0) = 0;
    //     commands[leg].kdJoint(1, 1) = 0;
    //     commands[leg].kdJoint(2, 2) = 0;

    //     commands[leg].kpJoint(0, 0) = 0;
    //     // commands[leg].kpJoint(1, 1) = 0;
    //     // commands[leg].kpJoint(2, 2) = 0;

    //     // commands[leg].qDes(0) = 1.8;
    //     commands[leg].qDes(1) = -0.8;
    //     commands[leg].qDes(2) = 1.8;
    //   }
    //   else if(leg != 1)
    //   {
    //     // Disable RF, RH, LH legs for lateral force
    //     // Hardcode legTorque:
    //     legTorque(0) = 0;
    //     legTorque(1) = 0;
    //     legTorque(2) = 0;

    //     // Hardcode PD
    //     commands[leg].kdJoint(0, 0) = 0;
    //     commands[leg].kdJoint(1, 1) = 0;
    //     commands[leg].kdJoint(2, 2) = 0;

    //     commands[leg].kpJoint(0, 0) = 0;
    //     commands[leg].kpJoint(1, 1) = 0;
    //     commands[leg].kpJoint(2, 2) = 0;
    //   }
    // }

    // set command:
    spiCommand->tau_abad_ff[leg] = legTorque(0);
    spiCommand->tau_hip_ff[leg] = legTorque(1);
    spiCommand->tau_knee_ff[leg] = legTorque(2);

    // joint space pd
    // joint space PD
    spiCommand->kd_abad[leg] = commands[leg].kdJoint(0, 0);
    spiCommand->kd_hip[leg] = commands[leg].kdJoint(1, 1);
    spiCommand->kd_knee[leg] = commands[leg].kdJoint(2, 2);

    spiCommand->kp_abad[leg] = commands[leg].kpJoint(0, 0);
    spiCommand->kp_hip[leg] = commands[leg].kpJoint(1, 1);
    spiCommand->kp_knee[leg] = commands[leg].kpJoint(2, 2);

    spiCommand->q_des_abad[leg] = commands[leg].qDes(0);
    spiCommand->q_des_hip[leg] = commands[leg].qDes(1);
    spiCommand->q_des_knee[leg] = commands[leg].qDes(2);

    spiCommand->qd_des_abad[leg] = commands[leg].qdDes(0);
    spiCommand->qd_des_hip[leg] = commands[leg].qdDes(1);
    spiCommand->qd_des_knee[leg] = commands[leg].qdDes(2);

    // estimate torque
    datas[leg].tauEstimate =
        legTorque +
        commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
        commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);

    datas[leg].tauFeedforward = legTorque;
    datas[leg].tauFeedback = commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
      commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);

    spiCommand->flags[leg] = _legsEnabled ? 1 : 0;

    // Custom
    // if (leg == 0)
    // {
    //   datas[leg].tauEstimate[0] = 0;
    //   datas[leg].tauEstimate[1] = 0;
    //   datas[leg].tauEstimate[2] = 0;
    // }

    // if (leg == 0)
    // {
    //   spiCommand->q_des_abad[leg] = 0;
    //   spiCommand->q_des_hip[leg] = 0;
    //   spiCommand->q_des_knee[leg] = 0;

    //   spiCommand->qd_des_abad[leg] = 0;
    //   spiCommand->qd_des_hip[leg] = 0;
    //   spiCommand->qd_des_knee[leg] = 0;
    // }

    // Abandoned because not recording changed torque, but just change outputs
    // if (leg == 0)
    // {
    //   // Safety code temp
    //   if ((datas[leg].q(0) > 0.4) || (datas[leg].q(0) < -0.4))
    //   {
    //     commands[leg].tauFeedForwardBias(0) = 0;
    //   }
    //   if ((datas[leg].q(1) > -0.5) || (datas[leg].q(1) < -1.1))
    //   {
    //     commands[leg].tauFeedForwardBias(1) = 0;
    //   }
    //   if ((datas[leg].q(2) > 1.93) || (datas[leg].q(2) < 1.33))
    //   {
    //     commands[leg].tauFeedForwardBias(2) = 0;
    //   }

    //   // set command:
    //   spiCommand->tau_abad_ff[leg] += commands[leg].tauFeedForwardBias(0);
    //   spiCommand->tau_hip_ff[leg] += commands[leg].tauFeedForwardBias(1);
    //   spiCommand->tau_knee_ff[leg] += commands[leg].tauFeedForwardBias(2);

    //   // joint space pd
    //   // joint space PD
    //   // spiCommand->kd_abad[leg] = 0;
    //   // spiCommand->kd_hip[leg] = 0;
    //   // spiCommand->kd_knee[leg] = 0;

    //   // spiCommand->kp_abad[leg] = 0;
    //   // spiCommand->kp_hip[leg] = 0;
    //   // spiCommand->kp_knee[leg] = 0;

    //   // Custom printing info
    //   // std::cout<< "(Custom outputs4) q: "<< datas[leg].q
    //   //           << "\n";
    //   // std::cout<< "(Custom outputs14) DEBUG: tauEstimate: "<< datas[leg].tauEstimate
    //   //           << "\n";  // 0.796523, -0.0149865, -3.15868
    // }
  }
}

constexpr float CHEETAH_3_ZERO_OFFSET[4][3] = {{1.f, 4.f, 7.f},
                                               {2.f, 5.f, 8.f},
                                               {3.f, 6.f, 9.f}};
/*!
 * Update the "leg command" for the TI Board
 */
template <typename T>
void LegController<T>::updateCommand(TiBoardCommand* tiBoardCommand) {
  for (int leg = 0; leg < 4; leg++) {
    Vec3<T> tauFF = commands[leg].tauFeedForward.template cast<T>();


    for (int joint = 0; joint < 3; joint++) {
      tiBoardCommand[leg].kp[joint] = commands[leg].kpCartesian(joint, joint);
      tiBoardCommand[leg].kd[joint] = commands[leg].kdCartesian(joint, joint);
      tiBoardCommand[leg].tau_ff[joint] = tauFF[joint];
      tiBoardCommand[leg].position_des[joint] = commands[leg].pDes[joint];
      tiBoardCommand[leg].velocity_des[joint] = commands[leg].vDes[joint];
      tiBoardCommand[leg].force_ff[joint] =
          commands[leg].forceFeedForward[joint];
      tiBoardCommand[leg].q_des[joint] = commands[leg].qDes[joint];
      tiBoardCommand[leg].qd_des[joint] = commands[leg].qdDes[joint];
      tiBoardCommand[leg].kp_joint[joint] = commands[leg].kpJoint(joint, joint);
      tiBoardCommand[leg].kd_joint[joint] = commands[leg].kdJoint(joint, joint);
      tiBoardCommand[leg].zero_offset[joint] = CHEETAH_3_ZERO_OFFSET[leg][joint];
    }

    // please only send 1 or 0 here or the robot will explode.
    tiBoardCommand[leg].enable = _legsEnabled ? 1 : 0;
    tiBoardCommand[leg].max_torque = _maxTorque;
    tiBoardCommand[leg].zero = _zeroEncoders ? 1 : 0;
    if(_calibrateEncoders) {
      tiBoardCommand[leg].enable = _calibrateEncoders + 1;
    }

    if(_zeroEncoders) {
      tiBoardCommand[leg].enable = 0;
    }

  }
}

/*  added for contact estimation */
template<typename T>
void LegController<T>::setLcm(contact_data_lcmt* lcmData) {
  for (int leg = 0; leg < 4; leg++) {
    for (int axis = 0; axis < 3; axis++) {
      int idx = leg*3 + axis;
      lcmData->tau_feed_forward[idx] = datas[leg].tauFeedforward[axis];
      lcmData->tau_feed_back[idx] = datas[leg].tauFeedback[axis];
      lcmData->tau_estimate[idx] = datas[leg].tauEstimate[axis];
      lcmData->tau_estimate_from_actuator_model[idx] = datas[leg].tauEstimateFromActuatorModel[axis];
    }    
  }
}

/*!
 * Set LCM debug data from leg commands and data
 */
template<typename T>
void LegController<T>::setLcm(leg_control_data_lcmt *lcmData, leg_control_command_lcmt *lcmCommand) {
    for(int leg = 0; leg < 4; leg++) {
        for(int axis = 0; axis < 3; axis++) {
            int idx = leg*3 + axis;
            lcmData->q[idx] = datas[leg].q[axis];
            lcmData->qd[idx] = datas[leg].qd[axis];
            lcmData->p[idx] = datas[leg].p[axis];
            lcmData->v[idx] = datas[leg].v[axis];
            lcmData->tau_est[idx] = datas[leg].tauEstimate[axis];

            lcmCommand->tau_ff[idx] = commands[leg].tauFeedForward[axis];
            lcmCommand->f_ff[idx] = commands[leg].forceFeedForward[axis];
            lcmCommand->q_des[idx] = commands[leg].qDes[axis];
            lcmCommand->qd_des[idx] = commands[leg].qdDes[axis];
            lcmCommand->p_des[idx] = commands[leg].pDes[axis];
            lcmCommand->v_des[idx] = commands[leg].vDes[axis];
            lcmCommand->kp_cartesian[idx] = commands[leg].kpCartesian(axis, axis);
            lcmCommand->kd_cartesian[idx] = commands[leg].kdCartesian(axis, axis);
            lcmCommand->kp_joint[idx] = commands[leg].kpJoint(axis, axis);
            lcmCommand->kd_joint[idx] = commands[leg].kdJoint(axis, axis);
        }
    }
}

// Custom added for estimate tau_est more accurately (with Actuator Model)
template <typename T>
void LegController<T>::getTauEstimateFromActuatorModel(SpiCommand* spiCommand, SpiData* spiData)
{
  // double temp_abadGearRatio = 6;
  // double temp_hipGearRatio = 6;
  // double temp_kneeGearRatio = 9.33;
  double temp_GearRatio[3] = {6, 6, 9.33};
  double temp_motorTauMax = 3.f;
  double temp_batteryV = 24;
  double temp_motorKT = .05;  // this is flux linkage * pole pairs
  double temp_motorR = 0.173;
  double temp_jointDamping = .01;
  double temp_jointDryFriction = .2;
  //double temp_jointDamping = .0;
  //double temp_jointDryFriction = .0;

  // Init spine calculation
  for(int leg = 0; leg < 4; leg++)
  {
    _spineBoards[leg].init(Quadruped<float>::getSideSign(leg), leg);
    _spineBoards[leg].data_cal = spiData;
    _spineBoards[leg].cmd_cal = spiCommand;
  }

  // Run spine board control
  for (auto& spineBoard : _spineBoards) {
    spineBoard.run_cal();
  }

  // Compute refined torques
  for (int leg = 0; leg < 4; leg++)
  {
    for (int joint = 0; joint < 3; joint++)
    {
      // Get computed des torque
      double temp_tauDes = _spineBoards[leg].torque_out_cal[joint];
      double temp_qd = datas[leg].qd[joint];

      // compute motor torque
      double temp_tauDesMotor = temp_tauDes / temp_GearRatio[joint];  // motor torque
      double temp_iDes = temp_tauDesMotor / (temp_motorKT * 1.5);  // i = tau / KT
      // double bemf =  qd * _gr * _kt * 1.732;     // back emf
      double temp_bemf = temp_qd * temp_GearRatio[joint] * temp_motorKT * 2.;       // back emf
      double temp_vDes = temp_iDes * temp_motorR + temp_bemf;          // v = I*R + emf
      double temp_vActual = coerce(temp_vDes, -temp_batteryV, temp_batteryV);  // limit to battery voltage
      double temp_tauActMotor =
          1.5 * temp_motorKT * (temp_vActual - temp_bemf) / temp_motorR;  // tau = Kt * I = Kt * V / R
      double temp_tauAct = temp_GearRatio[joint] * coerce(temp_tauActMotor, -temp_motorTauMax, temp_motorTauMax);

      // add damping and dry friction (set 1 as default)
      if (1)
        temp_tauAct = temp_tauAct - temp_jointDamping * temp_qd - temp_jointDryFriction * sgn(temp_qd);

      // Final output to data for lcm (prepare)
      datas[leg].tauEstimateFromActuatorModel[joint] = temp_tauAct;
    }
  }
  // std::cout<< "(Custom outputs15) Tau_Est_Motor: "<< datas[0].tauEstimateFromActuatorModel[0] << ","
  //           << datas[0].tauEstimateFromActuatorModel[1] << ","
  //           << datas[0].tauEstimateFromActuatorModel[2]
  //           << "\n";
}

template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;

/*!
 * Compute the position of the foot and its Jacobian.  This is done in the local
 * leg coordinate system. If J/p are NULL, the calculation will be skipped.
 */
template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg) {
  T l1 = quad._abadLinkLength;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T l4 = quad._kneeLinkY_offset;
  T sideSign = quad.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  if (J) {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p) {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

template void computeLegJacobianAndPosition<double>(Quadruped<double>& quad,
                                                    Vec3<double>& q,
                                                    Mat3<double>* J,
                                                    Vec3<double>* p, int leg);
template void computeLegJacobianAndPosition<float>(Quadruped<float>& quad,
                                                   Vec3<float>& q,
                                                   Mat3<float>* J,
                                                   Vec3<float>* p, int leg);
