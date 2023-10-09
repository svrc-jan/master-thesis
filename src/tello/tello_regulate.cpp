void CTello::regulate(){
    
    ///regulator constants
    const float d_alfa = 0.3;
    // const float kp = 1;// vicon
    const float kp = 1;
    float kd = 0.0;
    // float ki = 0.3; // vicon
    float ki = 0;
    const float max_i = 0.05;
    
    double ok_dist = 0.1;
    
    if(!localize_dwloc && !localize_vicon){
        fprintf(stderr, "regulator: set localization method first\n");
        return;
    }
    if(!navigate_dwloc && ! navigate_position && !navigate_vicon && !navigate_trajectory){
        fprintf(stderr, "regulator: set navigation method first\n");
        return;
    }
   
    mypoint_t my_pos;
    mypoint_t target_pos;
    
    /// get my position
    if(localize_dwloc){
        my_pos = target_own->get_smooth_position();
    } else if (localize_vicon){
        vicon_master->get_position(&my_pos, vicon_id_own);
    } else {
        fprintf(stderr, "regulator: no position source active\n");
        return;
    }
    
    /// get target position
    if (navigate_dwloc){
        target_pos = target_goal->get_smooth_position();
    } else if (navigate_position){
        goal_mtx.lock();
        target_pos.x = goal.x;
        target_pos.y = goal.y;
        target_pos.z = goal.z;
        goal_mtx.unlock();
    } else if (navigate_vicon){
        vicon_master->get_position(&target_pos, vicon_id_target);
    } else if (navigate_trajectory){
        uint64_t t = dzony::timestamp_ms();
        target_pos = get_trajectory_point((uint16_t)(t-trajectory_timestamp));
    } else {
        fprintf(stderr, "regulator: no navigation enabled\n");
        return;
    }
    
    /// regulator 
    double dx = (target_pos.x + goal_offset.x - my_pos.x);
    double dy = (target_pos.y + goal_offset.y - my_pos.y);
    
    //printf("%2.2f\n", sqrt(dx*dx+dy*dy));
  
    
    //printf("der %f %f\n", x_d, y_d);
    if(abs(dx) > ok_dist){
        x_i += dx*(STICKS_LOOP*1e-6);
    }else{
       ;
    }
    if(abs(dy) > ok_dist){
        y_i += dy*(STICKS_LOOP*1e-6);
    }else{
        ;
    }
   
    limit(x_i, -max_i, max_i);
    limit(y_i, -max_i, max_i);
    
    double u_x = dx*kp+x_i*ki+x_d*kd;
    double u_y = dy*kp+y_i*ki+y_d*kd;
    
    normalize_sticks_input(u_x, u_y);
    
    log_file << (dzony::timestamp_ms() - trajectory_timestamp) << " " << my_pos.x << " " 
            << my_pos.y << " " << target_pos.x << " " << target_pos.y << " " << u_x << " " << u_y << " "
             << x_i << " " << y_i << "\n";
    
    this->setStickData(false, u_x, u_y, 0, 0);
}

mypoint_t const trajectory_circle(uint16_t t){
    mypoint_t point;
    float T = 15e3;
    float r = 0.5;
    point.x = sin(2*M_PI*t/T)*r;
    point.y = -cos(2*M_PI*t/T)*r+r;
    point.z = 1;
     
    return point;
}

mypoint_t const trajectory_step(uint16_t t){
    
    mypoint_t point;
    
    uint16_t T = 500;
    
    if(t<T){
        point.x = 0;
        point.y = 0;
    }else if (t < 2*T){
        point.x = 0;
        point.y = -2;
    } else {
        point.x = 0;
        point.y = 0;
    }
    
    
    return point;
}

mypoint_t const trajectory_square(uint16_t t){
    mypoint_t point;
    float T =20e3;
    float r = 1;
    
    while(t>T)
        t-=T;
    
    if(t < T/4){
        point.x = 0;
        point.y = t*4/T*r;
    }else if(t < T/2){
        t -= T/4;
        point.x = t*4/T*r;
        point.y = r;
    }else if(t < T/4*3){
        t -= T/2;
        point.x = r;
        point.y = r-t*4/T*r;
    }else{
        t -= T/4*3;
        point.x = r-t*4/T*r;
        point.y = 0;
    }
    return point;
}

mypoint_t CTello::get_trajectory_point(uint16_t t){
    
    mypoint_t point = trajectory_circle(t);
    
    point.z = 1;   
    point.x += trajectory_origin.x;
    point.y += trajectory_origin.y;   
    
    return point;
}

void CTello::set_goal_offset(float x, float y, float z){
    goal_offset.x = x;
    goal_offset.y = y;
    goal_offset.z = z;
}

void CTello::set_goal(float x, float y, float z){   
    goal_mtx.lock();
    goal.x = x;
    goal.y = y;
    goal.z = z;
    goal_mtx.unlock();
}

void CTello::set_goal(Target* target){   
    this->target_goal = target;
    printf("Goal target was set\n");
}

void CTello::assign_own_target(Target* target){
    printf("Own antenna target was assign\n");
    this->target_own = target;
}

void CTello::stop_navigating(){
    navigate_dwloc = false;
    navigate_position = false;
    setStickData(false,0,0,0,0);
}

void CTello::start_target_navigating(){
    if (!target_own){
        fprintf(stderr, "set own target first\n");
        return;
    }
    if (!target_goal){
        fprintf(stderr, "set goal target first\n");
        return;
    }
    
    setStickData(false,0,0,0,0);
    navigate_dwloc = true;
    
}


void CTello::start_position_navigating(){
    if (!target_own){
        fprintf(stderr, "set own target first\n");
        return;
    }
    
    setStickData(false,0,0,0,0);
    navigate_position = true;
}

void CTello::start_trajectory_navigating(){
    trajectory_timestamp = dzony::timestamp_ms();
    get_position(&trajectory_origin);
    
    setStickData(false,0,0,0,0);
    navigate_trajectory = true;
}

void CTello::get_position(mypoint_t* position){
    if(localize_vicon){
        vicon_master->get_position(&trajectory_origin, vicon_id_own);
    } else if (localize_dwloc){
        *position = target_own->get_smooth_position();
    } else {
        fprintf(stderr, "CTello::get_position: no localization method set\n");
    }
}

void CTello::start_vicon_navigating(){
    
    if(!vicon_master){
        fprintf(stderr, "link vicon master first\n");
        return;
    }
    
    if (!target_own){
        fprintf(stderr, "set own target first\n");
        return;
    }
    
    if(vicon_id_target < 0){
        fprintf(stderr, "set target vicon ID\n");
        return;
    }
    
    setStickData(false,0,0,0,0);
    navigate_vicon = true;
}

void CTello::start_localize_vicon(){
    if(vicon_id_own < 0){
        fprintf(stderr, "localize_vicon: set ID first\n");
    }
    if(!vicon_master){
        fprintf(stderr, "localize_vicon: link vicn master first\n");
    }
    mypoint_t temp;
    uint64_t start = dzony::timestamp_ms();
    while(!vicon_master->get_position(&temp, vicon_id_own)){
        uint64_t now = dzony::timestamp_ms();
        if((now - start) > 2000){
            fprintf(stderr, "start_localize_vicon: no valid location\n");
            return;
        }
    }
    //printf("start_localize_vicon: got valid location\n");

    localize_vicon = true;
}

void CTello::start_localize_dwloc(){
    if(target_own == 0){
        fprintf(stderr, "localize_dwloc: set target pointer first\n");
    }
    localize_dwloc = true;
}

void CTello::set_own_vicon(int ID){
    vicon_id_own = ID;
}

void CTello::link_vicon_master(Vicon_master* master){
    vicon_master = master;
}

void CTello::set_goal_vicon(int ID){
    vicon_id_target = ID;
}