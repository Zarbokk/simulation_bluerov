this->height -= (_manual_control_setpoint.z-0.5f);
if(abs(this->height-vlocal_pos.z)>1){
this->height+=(_manual_control_setpoint.z-0.5f);
}
this->yawAngle += 0.1f*_manual_control_setpoint.r;
if(this->yawAngle>(float)M_PI){
this->yawAngle = this->yawAngle-(float)(2*M_PI);
}
if(this->yawAngle<(float)-M_PI){
this->yawAngle = this->yawAngle+(float)(2*M_PI);
}
//printf("yaw angle = %f z positionDes = %f current z Position : %f \n",(double)this->yawAngle,(double)this->height,(double)vlocal_pos.z);
vehicle_rates_setpoint_s _rates_setpointdesired {};
vehicle_attitude_setpoint_s attitudeDesired;
attitudeDesired.roll_body = 0;
attitudeDesired.pitch_body = 0;
attitudeDesired.yaw_body = this->yawAngle;
attitudeDesired.thrust_body[0] = _manual_control_setpoint.x;
attitudeDesired.thrust_body[1] = _manual_control_setpoint.y;
attitudeDesired.thrust_body[2] = 10.0f*(this->height-vlocal_pos.z);

control_attitude_geo(attitude, attitudeDesired, angular_velocity, _rates_setpointdesired);








uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};





vehicle_local_position_s vlocal_pos;
_vehicle_local_position_sub.copy(&vlocal_pos);
vehicle_angular_velocity_s angular_velocity {};
_angular_velocity_sub.copy(&angular_velocity);