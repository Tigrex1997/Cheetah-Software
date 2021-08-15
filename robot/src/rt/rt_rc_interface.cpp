#include <pthread.h>
#include <rt/rt_rc_interface.h>
#include "Utilities/EdgeTrigger.h"
#include <string.h> // memcpy
#include <stdio.h>
#include <rt/rt_sbus.h>
static pthread_mutex_t lcm_get_set_mutex =
PTHREAD_MUTEX_INITIALIZER; /**< mutex to protect gui settings coming over
                             LCM */


void rc_control_settings::zeroRCcontrol()
{
  // mode = 0;
  for(int i = 0; i < 2; i++)
  {
    p_des[i] = 0; // (x, y) -1 ~ 1
  }
  height_variation = 0; // -1 ~ 1
  for(int i = 0; i < 3; i++)
  {
    v_des[i] = 0; // -1 ~ 1 * (scale 0.5 ~ 1.5)
  }
  for(int i = 0; i < 3; i++)
  {
    rpy_des[i] = 0; // -1 ~ 1
  }
  for(int i = 0; i < 3; i++)
  {
    omega_des[i] = 0; // -1 ~ 1
  }
  // for(int i = 0; i < 3; i++)
  // {
  //   variable[i] = 0;
  // }

  for(int i = 0; i < 3; i++)
  {
    tauFeedForwardBias_des[i] = 0;
  }
  // for(int i = 0; i < 3; i++)
  // {
  //   tauFeedForwardBiasCounter_des[i] = 0;
  // }
}

// Controller Settings
rc_control_settings rc_control;

/* ------------------------- HANDLERS ------------------------- */

// Controller Settings
void get_rc_control_settings(void *settings) {
  pthread_mutex_lock(&lcm_get_set_mutex);
  v_memcpy(settings, &rc_control, sizeof(rc_control_settings));
  pthread_mutex_unlock(&lcm_get_set_mutex);
}

//void get_rc_channels(void *settings) {
//pthread_mutex_lock(&lcm_get_set_mutex);
//v_memcpy(settings, &rc_channels, sizeof(rc_channels));
//pthread_mutex_unlock(&lcm_get_set_mutex);
//}

EdgeTrigger<int> mode_edge_trigger(0);
EdgeTrigger<TaranisSwitchState> backflip_prep_edge_trigger(SWITCH_UP);
EdgeTrigger<TaranisSwitchState> experiment_prep_edge_trigger(SWITCH_UP);
TaranisSwitchState initial_mode_go_switch = SWITCH_DOWN;

void sbus_packet_complete() {
  Taranis_X7_data data;
  update_taranis_x7(&data);

  // Custom (Set RC input as 0s every iteration)
  //rc_control.zeroRCcontrol();

  float v_scale = data.knobs[0]*1.5f + 2.0f; // from 0.5 to 3.5
  float w_scale = 2.*v_scale; // from 1.0 to 7.0
  //printf("v scale: %f\n", v_scale);

  auto estop_switch = data.right_lower_right_switch;
  auto mode_selection_switch = data.left_lower_left_switch;
  auto mode_go_switch = data.left_upper_switch;
  // Custom
  // printf("(Custom outputs9) DEBUG: Value of mode_go_switch: %d\n", mode_go_switch);
  // std::cout<< "(Custom outputs9) DEBUG: Actual commanded torque in Sim: "
  //                 << mode_go_switch
  //                 << "\n";

  auto left_select = data.left_lower_right_switch;
  auto right_select = data.right_lower_left_switch;

  int selected_mode = 0;

  switch(estop_switch) {

    case SWITCH_UP: // ESTOP
      selected_mode = RC_mode::OFF;
      break;

    case SWITCH_MIDDLE: // recover
      selected_mode = RC_mode::RECOVERY_STAND;
      break;

    case SWITCH_DOWN: // run 
      selected_mode = RC_mode::LOCOMOTION; // locomotion by default

      // stand mode
      if(left_select == SWITCH_UP && right_select == SWITCH_UP) {
        selected_mode = RC_mode::QP_STAND;
      }

      if(backflip_prep_edge_trigger.trigger(mode_selection_switch) 
          && mode_selection_switch == SWITCH_MIDDLE) {
        initial_mode_go_switch = mode_go_switch;
      }

      // Experiment mode (two leg stance, vision, ...)
      if(experiment_prep_edge_trigger.trigger(mode_selection_switch) 
          && mode_selection_switch == SWITCH_DOWN) {
        initial_mode_go_switch = mode_go_switch;
      }


      // backflip
      if(mode_selection_switch == SWITCH_MIDDLE) {
        // Origin
        /*
        selected_mode = RC_mode::BACKFLIP_PRE;

        if(mode_go_switch == SWITCH_DOWN && initial_mode_go_switch != SWITCH_DOWN) {
          selected_mode = RC_mode::BACKFLIP;
        } else if(mode_go_switch == SWITCH_UP) {
          initial_mode_go_switch = SWITCH_UP;
        }
        */

        // Custom
        if (mode_go_switch == SWITCH_UP)
        {
          selected_mode = RC_mode::QP_STAND_FRICTION_EST;
        }
        else if(mode_go_switch == SWITCH_DOWN)
        {
          selected_mode = RC_mode::QP_STAND_FRICTION_EST_AUTO;
        }
      } // Experiment Mode
      else if(mode_selection_switch == SWITCH_DOWN){
        // Origin
        // int mode_id = left_select * 3 + right_select;

        // if(mode_id == 0){ // Two leg stance
        //   selected_mode = RC_mode::TWO_LEG_STANCE_PRE;
        //   if(mode_go_switch == SWITCH_DOWN && initial_mode_go_switch != SWITCH_DOWN) {
        //     selected_mode = RC_mode::TWO_LEG_STANCE;
        //   } else if(mode_go_switch == SWITCH_UP) {
        //     initial_mode_go_switch = SWITCH_UP;
        //   }
        // }
        // else if(mode_id == 1){ // Vision 
        //   selected_mode = RC_mode::VISION;
        // }

        //Custom for vertical GRF calibration
        selected_mode = RC_mode::QP_STAND_GRF_CALIB;
      }

      // gait selection
      int mode_id = left_select * 3 + right_select;

      constexpr int gait_table[9] = {0, //stand
        0, // trot
        1, // bounding
        2, // pronking
        3, // gallop
        5, // trot run
        6, // walk};
        7, // walk2?
        8, // pace
  };


  // Deadband
  for(int i(0); i<2; ++i){
    data.left_stick[i] = deadband(data.left_stick[i], 0.1, -1., 1.);
    data.right_stick[i] = deadband(data.right_stick[i], 0.1, -1., 1.);
  }

  // Custom: Clear auto bias if NOT in QP_STAND_FRICTION_EST_AUTO mode
  // to enable the auto mode to start from 0s
  if(selected_mode != RC_mode::QP_STAND_FRICTION_EST_AUTO)
  {
    rc_control.tauFeedForwardBiasCounter_des[0] = 0;
    rc_control.tauFeedForwardBiasCounter_des[1] = 0;
    rc_control.tauFeedForwardBiasCounter_des[2] = 0;
    rc_control.bias_counter = 0;
    rc_control.flag_auto_bias_limit_reached = 0;
  }

  if(selected_mode == RC_mode::LOCOMOTION 
      || selected_mode == RC_mode::VISION) {
    rc_control.variable[0] = gait_table[mode_id];
    //rc_control.v_des[0] = v_scale * data.left_stick[1] * 0.5;
    //rc_control.v_des[1] = v_scale * data.left_stick[0] * -1.;
    rc_control.v_des[0] = v_scale * data.left_stick[1];
    rc_control.v_des[1] = -v_scale * data.left_stick[0];
    rc_control.v_des[2] = 0;

    rc_control.height_variation = data.knobs[1];
    //rc_control.p_des[2] = 0.27 + 0.08 * data.knobs[1]; // todo?

    rc_control.omega_des[0] = 0;
    rc_control.omega_des[1] = 0;
    rc_control.omega_des[2] = w_scale * data.right_stick[0];
    //rc_control.omega_des[2] = -v_scale * data.right_stick[0];

  } else if(selected_mode == RC_mode::QP_STAND || 
      selected_mode == RC_mode::TWO_LEG_STANCE) {
    //rc_control.rpy_des[0] = data.left_stick[0] * 1.4;
    //rc_control.rpy_des[1] = data.right_stick[1] * 0.46;
    
    // Origin
    rc_control.rpy_des[0] = data.left_stick[0];
    rc_control.rpy_des[1] = data.right_stick[1];
    rc_control.rpy_des[2] = data.right_stick[0];

    rc_control.height_variation = data.left_stick[1];
    

    rc_control.omega_des[0] = 0;
    rc_control.omega_des[1] = 0;
    rc_control.omega_des[2] = 0;
    //rc_control.p_des[1] = -0.667 * rc_control.rpy_des[0];
    //rc_control.p_des[2] = data.left_stick[1] * .12;

    // Custom
    // rc_control.rpy_des[0] = 0;
    // rc_control.rpy_des[1] = 0;
    // rc_control.rpy_des[2] = 0;

    // rc_control.height_variation = 0;

    // rc_control.tauFeedForwardBias_des[0] = data.left_stick[0];
    // rc_control.tauFeedForwardBias_des[1] = data.right_stick[0];
    // rc_control.tauFeedForwardBias_des[2] = data.right_stick[1];
  } else if(selected_mode == RC_mode::QP_STAND_FRICTION_EST) {
    //rc_control.rpy_des[0] = data.left_stick[0] * 1.4;
    //rc_control.rpy_des[1] = data.right_stick[1] * 0.46;
    
    // Origin
    /*
    rc_control.rpy_des[0] = data.left_stick[0];
    rc_control.rpy_des[1] = data.right_stick[1];
    rc_control.rpy_des[2] = data.right_stick[0];

    rc_control.height_variation = data.left_stick[1];
    */

    rc_control.omega_des[0] = 0;
    rc_control.omega_des[1] = 0;
    rc_control.omega_des[2] = 0;
    //rc_control.p_des[1] = -0.667 * rc_control.rpy_des[0];
    //rc_control.p_des[2] = data.left_stick[1] * .12;

    // Custom
    rc_control.rpy_des[0] = 0;
    rc_control.rpy_des[1] = 0;
    rc_control.rpy_des[2] = 0;

    rc_control.height_variation = 0;

    //????????
    // double manual_adjustment_gain = data.knobs[1];
    // printf("(Custom outputs10) DEBUG: Value of the left knob: %f\n", data.knobs[0]);

    rc_control.tauFeedForwardBias_des[0] = data.left_stick[0]*10;
    rc_control.tauFeedForwardBias_des[1] = data.right_stick[0]*10;
    rc_control.tauFeedForwardBias_des[2] = data.right_stick[1]*10;
  } else if(selected_mode == RC_mode::QP_STAND_FRICTION_EST_AUTO) {
    //rc_control.rpy_des[0] = data.left_stick[0] * 1.4;
    //rc_control.rpy_des[1] = data.right_stick[1] * 0.46;
    
    // Origin
    /*
    rc_control.rpy_des[0] = data.left_stick[0];
    rc_control.rpy_des[1] = data.right_stick[1];
    rc_control.rpy_des[2] = data.right_stick[0];

    rc_control.height_variation = data.left_stick[1];
    */

    rc_control.omega_des[0] = 0;
    rc_control.omega_des[1] = 0;
    rc_control.omega_des[2] = 0;
    //rc_control.p_des[1] = -0.667 * rc_control.rpy_des[0];
    //rc_control.p_des[2] = data.left_stick[1] * .12;

    // Custom
    rc_control.rpy_des[0] = 0;
    rc_control.rpy_des[1] = 0;
    rc_control.rpy_des[2] = 0;

    rc_control.height_variation = 0;

    // rc_control.tauFeedForwardBias_des[0] = data.left_stick[0];
    // rc_control.tauFeedForwardBias_des[1] = data.right_stick[0];
    // rc_control.tauFeedForwardBias_des[2] = data.right_stick[1];

    if(rc_control.flag_auto_bias_limit_reached == 0)
    {
      rc_control.bias_counter = rc_control.bias_counter + 1;
    } else if(rc_control.flag_auto_bias_limit_reached == 1)
    {
      rc_control.bias_counter = 0;  // Prevent limit reached but counter still growing
    }
    // printf("(Custom outputs11) DEBUG: bias_counter: %d\n", rc_control.bias_counter);  // Should change under 10000 (10N)

    // Adding joint biases (?????? Consider 0 if joint changes)
    // if(???? == SWITCH_UP)
    // {
    if(rc_control.flag_auto_bias_limit_reached == 0)
    {
      rc_control.tauFeedForwardBiasCounter_des[0] = rc_control.bias_counter*rc_control.gain_time;
    } else if(rc_control.flag_auto_bias_limit_reached == 1)  // Safety limit reached
    {
      rc_control.tauFeedForwardBiasCounter_des[0] = rc_control.auto_bias_limit;
    }
    // printf("(Custom outputs12) DEBUG: tauFeedForwardBiasCounter_des[0]: %f\n", rc_control.tauFeedForwardBiasCounter_des[0]);
    
    // } else if(mode_auto_friction_switch == SWITCH_MIDDLE)
    // {
    //   rc_control.tauFeedForwardBiasCounter_des[1]++;
    // } else if(mode_auto_friction_switch == SWITCH_DOWN)
    // {
    //   rc_control.tauFeedForwardBiasCounter_des[2]++;
    // }

    // Safety (15N limit)
    for(int temp_i = 0; temp_i < 3; temp_i++)
    {
      if(rc_control.tauFeedForwardBiasCounter_des[temp_i] > rc_control.auto_bias_limit)
      {
        rc_control.tauFeedForwardBiasCounter_des[temp_i] = rc_control.auto_bias_limit;
        rc_control.bias_counter = 0;
        rc_control.flag_auto_bias_limit_reached = 1;
      }
    }
  } else if (selected_mode == RC_mode::QP_STAND_GRF_CALIB){
    //rc_control.rpy_des[0] = data.left_stick[0] * 1.4;
    //rc_control.rpy_des[1] = data.right_stick[1] * 0.46;
    double temp_static_roll = data.knobs[0];
    double temp_static_pitch = data.knobs[1];
    //printf("(Custom outputs13) DEBUG: Knobs0, 1: %f, %f.\n", data.knobs[0], data.knobs[1]);
    double temp_static_yaw = 0;
    if(mode_go_switch == SWITCH_UP)
    {
      temp_static_yaw = 0;
    } else if(mode_go_switch == SWITCH_DOWN)
    {
      temp_static_yaw = -0.6;
    }
    
    // Origin
    rc_control.rpy_des[0] = temp_static_roll;
    rc_control.rpy_des[1] = temp_static_pitch;
    rc_control.rpy_des[2] = temp_static_yaw;

    rc_control.height_variation = data.left_stick[1];
    

    rc_control.omega_des[0] = 0;
    rc_control.omega_des[1] = 0;
    rc_control.omega_des[2] = 0;
    //rc_control.p_des[1] = -0.667 * rc_control.rpy_des[0];
    //rc_control.p_des[2] = data.left_stick[1] * .12;

    // Safety
    if(rc_control.rpy_des[0] < -0.4)
    {
      rc_control.rpy_des[0] = -0.4;
    } else if(rc_control.rpy_des[0] > 0.4)
    {
      rc_control.rpy_des[0] = 0.4;
    }
  }
  break;
}

bool trigger = mode_edge_trigger.trigger(selected_mode);
if(trigger || selected_mode == RC_mode::OFF || selected_mode == RC_mode::RECOVERY_STAND) {
  if(trigger) {
    printf("MODE TRIGGER!\n");
  }
  rc_control.mode = selected_mode;
}

}

void *v_memcpy(void *dest, volatile void *src, size_t n) {
  void *src_2 = (void *)src;
  return memcpy(dest, src_2, n);
}

float deadband(float command, float deadbandRegion, float minVal, float maxVal){
  if (command < deadbandRegion && command > -deadbandRegion) {
    return 0.0;
  } else {
    return (command / (2)) * (maxVal - minVal);
  }
}


