#ifndef _SHOVEL_H_
#define _SHOVEL_H_

void stop_ball();
void go_to_origin(double v, double a);
void move(double cm, int direction, double v, double a);
void launch_ball();
void prepare_launch(double cm);
void stop_shovel();
int shovel_is_at_origin();
int shovel_is_at_photodiode();
void move_to_photodiode(double v, double a);
int convert_cm_to_steps(double cm);
double convert_cm_to_turns(double cm);
double convert_v_to_omega(double v);
double convert_a_to_alpha(double a);

#endif
