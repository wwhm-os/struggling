#include "system_config.h"
//#include "com_prop.h"
//#include "RobotCAN_proplist.h"
#include "task_init.h"
#include "motor_control.h"

uint16_t angle = 45;

#ifdef DEBUG
uint8_t flag_on_road = false;
uint8_t count_stop_route_state = 0;
uint8_t flag_turn1 = false, flag_turn2 = false;
uint8_t flag_test =false;
#endif

void TaskRegulate(void* p_arg)
{	
    OS_ERR err;
    uint8_t sensor_bit_info = 0x00; //sensor_route
    
#ifdef DEBUG
    uint32_t temp;
    
    Motor1.State=IDLE;
    Motor2.State=IDLE;
    Motor3.State=IDLE;
    set_servo_angle(LOOSE);
 
    while(1)
    {
        sensor_read();
        sensor_bit_info = (sensor.sensor_route[SENSOR_ROUTE1]<<SENSOR_ROUTE1) + \
                            (sensor.sensor_route[SENSOR_ROUTE2]<<SENSOR_ROUTE2) + \
                            (sensor.sensor_route[SENSOR_ROUTE3]<<SENSOR_ROUTE3) + \
                            (sensor.sensor_route[SENSOR_ROUTE4]<<SENSOR_ROUTE4) + \
                            (sensor.sensor_route[SENSOR_ROUTE5]<<SENSOR_ROUTE5) + \
                            (sensor.sensor_route[SENSOR_ROUTE6]<<SENSOR_ROUTE6) + \
                            (sensor.sensor_route[SENSOR_ROUTE7]<<SENSOR_ROUTE7);
         OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
    } 

#ifdef TEST_MOVE
    
//    float trace_speed = 20.0f;
//    
//    flag_step = STEP1;
//    
//    Motor1.State=PIDSPEED;
//    Motor2.State=PIDSPEED;
//    Motor3.State=PIDSPEED;
//    while(1)
//    {
//        sensor_read();
//        sensor_bit_info = (sensor.sensor_route[SENSOR_ROUTE1]<<SENSOR_ROUTE1) + \
//                            (sensor.sensor_route[SENSOR_ROUTE2]<<SENSOR_ROUTE2) + \
//                            (sensor.sensor_route[SENSOR_ROUTE3]<<SENSOR_ROUTE3) + \
//                            (sensor.sensor_route[SENSOR_ROUTE4]<<SENSOR_ROUTE4) + \
//                            (sensor.sensor_route[SENSOR_ROUTE5]<<SENSOR_ROUTE5) + \
//                            (sensor.sensor_route[SENSOR_ROUTE6]<<SENSOR_ROUTE6) + \
//                            (sensor.sensor_route[SENSOR_ROUTE7]<<SENSOR_ROUTE7);
//        
//        switch(flag_step)
//        {
//            case STEP1: // turn left
//                        if(sensor_bit_info == 0x00)
//                        {
//                            if(flag_turn2)
//                            {
//                                flag_turn1 = true;
//                            }
//                            if(flag_turn1&&flag_turn2)
//                            {
//                                my_forward(20.0f);
//                                flag_step = STEP2;
//                            }
//                            else
//                            {
//                                my_turn_left(10.0f);                    
//                            }
//                        }
//                        else
//                        {
//                            flag_turn2 = true;                    
//                        }
//                        break;

//            case STEP2: // trace forward
//                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == ON_ROAD)
//                        {
//                            flag_on_road = true;
//                        }
//                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP && flag_on_road)
//                        {
//                            flag_on_road = false;
//                            trace_speed = 5.0f;
//                        }
//                        if(!sensor.sensor_pos1)
//                        {
//                            flag_step = STEP3;
//                            my_allstop();
//                        }
//                        break;
//                        
//            case STEP3: // get the weight
//                        my_allstop();
//                        trace_speed = 20.0f;
//                        break;
//            
//            case STEP4: // backward to the dest.
////                        flag_stop = trace_straight_route(trace_speed, &sensor_bit_info);
////                        if( last_flag_stop == ON_ROAD && (flag_stop == CHANGE_SPEED_OR_STOP || flag_stop == OUT_ROAD_BUT_UNCERTAIN) )
////                        {
////                            
////                        }
////                        last_flag_stop = flag_stop;
//                        if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == ON_ROAD)
//                        {
//                            flag_on_road = true;
//                        }
//                        if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == CHANGE_SPEED_OR_STOP && flag_on_road)
//                        {
//                            flag_on_road = false;
//                            count_stop_route_state++; 
//                        }
//                        if(count_stop_route_state == 2)
//                        {
//                            flag_step = STEP5;
//                            my_allstop();
//                        }
//                        break;
//            default:
//                        my_allstop();
//                        break;       
//        }  //end of switch_case        
//        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
//        
//    } // end of while(1)    
//   
    Motor1.State=PIDSPEED;
    Motor2.State=PIDSPEED;
    Motor3.State=PIDSPEED;
    
    temp = Motor1.PositionMeasure;
    while(1)
    {
        if( FLAG_CAR_ANGLE_ARRIVAL(temp, 360, 5.0f) )
        {
            my_allstop();
        }
        else
        {
            my_turn_right(20.0f);
        }
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);

    }

#endif 

#ifdef TEST_TRACE
    Motor1.State=PIDSPEED;
    Motor2.State=PIDSPEED;
    Motor3.State=PIDSPEED;
    Motor5.State=IDLE;    
    Motor6.State=IDLE;
    
    flag_step = STEP1;
    while(1)
    {
        sensor_read();
        sensor_bit_info = (sensor.sensor_route[SENSOR_ROUTE1]<<SENSOR_ROUTE1) + \
                            (sensor.sensor_route[SENSOR_ROUTE2]<<SENSOR_ROUTE2) + \
                            (sensor.sensor_route[SENSOR_ROUTE3]<<SENSOR_ROUTE3) + \
                            (sensor.sensor_route[SENSOR_ROUTE4]<<SENSOR_ROUTE4) + \
                            (sensor.sensor_route[SENSOR_ROUTE5]<<SENSOR_ROUTE5) + \
                            (sensor.sensor_route[SENSOR_ROUTE6]<<SENSOR_ROUTE6) + \
                            (sensor.sensor_route[SENSOR_ROUTE7]<<SENSOR_ROUTE7);
        
        switch(flag_step)
        {
            case STEP1:// turn right all_lited to only_D4
                        if(sensor_bit_info == 0x00)
                        {
                            my_turn_right(10.0f);                   
                        }
                        else
                        {
                            flag_turn2 = true;                    
                        }
                        if(flag_turn2 && sensor.sensor_route[SENSOR_ROUTE1] \
                            && sensor.sensor_route[SENSOR_ROUTE2] \
                            && sensor.sensor_route[SENSOR_ROUTE6] \
                            && sensor.sensor_route[SENSOR_ROUTE7] \
                            && (sensor.sensor_route[SENSOR_ROUTE3] == 0 | sensor.sensor_route[SENSOR_ROUTE4] == 0 | sensor.sensor_route[SENSOR_ROUTE5] == 0 ))// D3/4/5 lited, others all not
                        {
                            flag_turn1 = true;
                        }
                        if(flag_turn1&&flag_turn2)
                        {
                            my_forward(10.0f);
                            flag_step = STEP2;
                        }
                        break;
            case STEP2:
                        if(trace_round_route(20.0f, &sensor_bit_info) == CHANGE_SPEED_OR_STOP)
                        {
                            flag_step = STEP3;
                        }
                        break;
            case STEP3:
                        if(trace_straight_route(20.0f, &sensor_bit_info, FORWARD) == ON_ROAD)
                        {
                            flag_on_road = true;
                        }
                        if(trace_straight_route(20.0f, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP && flag_on_road)
                        {
                            flag_on_road = false;
                            flag_step = STEP4;
                        }
                        break;
            case STEP4:
                        trace_round_route(20.0f, &sensor_bit_info);
                        if(sensor.sensor_pos1 == SENSOR_YES)
                        {
                            my_allstop();
                            flag_step = STEP5;
                        }
                        break;                        
            case STEP5:
                        my_allstop();
                        break;
        } //end of switch

        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
    } // end of while(1)

#endif

    
#else  



    
#ifdef TEST_TASK1
    float trace_speed = 20.0f;
    uint8_t flag_on_road = false;
    uint8_t flag_stop = false;
    uint8_t count_stop_route_state = 0;
    
    flag_step = STEP1;
    
    Motor1.State=PIDSPEED;
    Motor2.State=PIDSPEED;
    Motor3.State=PIDSPEED;
    while(1)
    {
        sensor_read();
        sensor_bit_info = (sensor.sensor_route[SENSOR_ROUTE1]<<SENSOR_ROUTE1) + \
                            (sensor.sensor_route[SENSOR_ROUTE2]<<SENSOR_ROUTE2) + \
                            (sensor.sensor_route[SENSOR_ROUTE3]<<SENSOR_ROUTE3) + \
                            (sensor.sensor_route[SENSOR_ROUTE4]<<SENSOR_ROUTE4) + \
                            (sensor.sensor_route[SENSOR_ROUTE5]<<SENSOR_ROUTE5) + \
                            (sensor.sensor_route[SENSOR_ROUTE6]<<SENSOR_ROUTE6) + \
                            (sensor.sensor_route[SENSOR_ROUTE7]<<SENSOR_ROUTE7);
        
        switch(flag_step)
        {
            case STEP1:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            trace_speed = 10.0f;
                        }
                        if(!sensor.sensor_pos1)
                        {
                            flag_step = STEP2;
                            my_allstop();
                        }
                        break;
                        
            case STEP2:
                        trace_speed = 20.0f;
                        break;
            
            case STEP3:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            trace_speed = 5.0f;
                        }
                        if(!sensor.sensor_pos1)
                        {
                            flag_step = STEP4;
                            my_allstop();
                        }
                        break;
                        
            case STEP4: // put the weight
                        break;
            
            case STEP5: // loose the weight
                        trace_speed = 20.0f;
                        break;
            
            case STEP6: // backward to the dest.(part1)
//                        flag_stop = trace_straight_route(trace_speed, &sensor_bit_info);
//                        if( last_flag_stop == ON_ROAD && (flag_stop == CHANGE_SPEED_OR_STOP || flag_stop == OUT_ROAD_BUT_UNCERTAIN) )
//                        {
//                            
//                        }
//                        last_flag_stop = flag_stop;
                        if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == ON_ROAD)
                        {
                            flag_on_road = true;
                        }
                        if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == CHANGE_SPEED_OR_STOP && flag_on_road)
                        {
                            flag_on_road = false;
                            count_stop_route_state++; 
                        }
                        if(count_stop_route_state == 2)
                        {
                            my_allstop();
                      /********     end of task1           ****************/
                        }
                        break;
            default:
                        my_allstop();
                        break;       
        }  //end of switch_case        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
        
    } // end of while(1)    
#endif
    
#ifdef TEST_TASK2
    float trace_speed = 20.0f;
    uint8_t flag_on_road = false;
    uint8_t flag_stop = false;
    uint8_t flag_turn1 = false, flag_turn2 = false;
    uint8_t count_stop_route_state = 0;
    
    flag_step = STEP1;
    
    Motor1.State=PIDSPEED;
    Motor2.State=PIDSPEED;
    Motor3.State=PIDSPEED;
    while(1)
    {
        sensor_read();
        sensor_bit_info = (sensor.sensor_route[SENSOR_ROUTE1]<<SENSOR_ROUTE1) + \
                            (sensor.sensor_route[SENSOR_ROUTE2]<<SENSOR_ROUTE2) + \
                            (sensor.sensor_route[SENSOR_ROUTE3]<<SENSOR_ROUTE3) + \
                            (sensor.sensor_route[SENSOR_ROUTE4]<<SENSOR_ROUTE4) + \
                            (sensor.sensor_route[SENSOR_ROUTE5]<<SENSOR_ROUTE5) + \
                            (sensor.sensor_route[SENSOR_ROUTE6]<<SENSOR_ROUTE6) + \
                            (sensor.sensor_route[SENSOR_ROUTE7]<<SENSOR_ROUTE7);
        
        switch(flag_step)
        {
            case STEP1: // turn left
                        if(sensor_bit_info == 0x00)
                        {
                            if(flag_turn2)
                            {
                                flag_turn1 = true;
                            }
                            if(flag_turn1&&flag_turn2)
                            {
                                my_forward(20.0f);
                                flag_step = STEP2;
                            }
                            else
                            {
                                my_turn_left(10.0f);                    
                            }
                        }
                        else
                        {
                            flag_turn2 = true;                    
                        }
                        break;

            case STEP2: // trace forward
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == ON_ROAD)
                        {
                            flag_on_road = true;
                        }
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP && flag_on_road)
                        {
                            flag_on_road = false;
                            trace_speed = 5.0f;
                        }
                        if(!sensor.sensor_pos1)
                        {
                            flag_step = STEP3;
                            my_allstop();
                        }
                        break;
                        
            case STEP3: // get the weight
                        my_allstop();
                        trace_speed = 20.0f;
                        break;
            
            case STEP4: // backward to the dest.
//                        flag_stop = trace_straight_route(trace_speed, &sensor_bit_info);
//                        if( last_flag_stop == ON_ROAD && (flag_stop == CHANGE_SPEED_OR_STOP || flag_stop == OUT_ROAD_BUT_UNCERTAIN) )
//                        {
//                            
//                        }
//                        last_flag_stop = flag_stop;
                        if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == ON_ROAD)
                        {
                            flag_on_road = true;
                        }
                        if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == CHANGE_SPEED_OR_STOP && flag_on_road)
                        {
                            flag_on_road = false;
                            count_stop_route_state++; 
                        }
                        if(count_stop_route_state == 2)
                        {
                            flag_step = STEP5;
                            my_allstop();
                        }
                        break;
          case STEP5 :
                        
                        break;       
        }  //end of switch_case        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
        
    } // end of while(1)    
    
#endif
    

#ifdef TEST_TASK3
    float trace_speed = 20.0f;
    uint8_t flag_on_road = false;
    uint8_t flag_stop = false;
    uint8_t flag_turn1 = false, flag_turn2 = false;
    uint8_t count_stop_route_state = 0;
    float Motor_pos = 0.0f;
    
    flag_step = STEP1;
    
    Motor1.State=PIDSPEED;
    Motor2.State=PIDSPEED;
    Motor3.State=PIDSPEED;
    while(1)
    {
        sensor_read();
        sensor_bit_info = (sensor.sensor_route[SENSOR_ROUTE1]<<SENSOR_ROUTE1) + \
                            (sensor.sensor_route[SENSOR_ROUTE2]<<SENSOR_ROUTE2) + \
                            (sensor.sensor_route[SENSOR_ROUTE3]<<SENSOR_ROUTE3) + \
                            (sensor.sensor_route[SENSOR_ROUTE4]<<SENSOR_ROUTE4) + \
                            (sensor.sensor_route[SENSOR_ROUTE5]<<SENSOR_ROUTE5) + \
                            (sensor.sensor_route[SENSOR_ROUTE6]<<SENSOR_ROUTE6) + \
                            (sensor.sensor_route[SENSOR_ROUTE7]<<SENSOR_ROUTE7);
        
        switch(flag_step)
        {
            case STEP1:// turn right all_lited to only_D4
                        if(sensor_bit_info == 0x00)
                        {
                            my_turn_right(10.0f);                   
                        }
                        else
                        {
                            flag_turn2 = true;                    
                        }
                        if(flag_turn2 && sensor.sensor_route[SENSOR_ROUTE1] \
                            && sensor.sensor_route[SENSOR_ROUTE2] \
                            && sensor.sensor_route[SENSOR_ROUTE6] \
                            && sensor.sensor_route[SENSOR_ROUTE7] \
                            && sensor.sensor_route[SENSOR_ROUTE3] &&( sensor.sensor_route[SENSOR_ROUTE4] == 0 | sensor.sensor_route[SENSOR_ROUTE5] == 0 ))// D3/4/5 lited, others all not
                        {
                            flag_turn1 = true;
                        }
                        if(flag_turn1&&flag_turn2)
                        {
                            my_allstop();
                            flag_step = STEP2;
                        }
                        break;
            case STEP2:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            my_allstop();
                            Motor_pos = Motor1.PositionMeasure;
                            flag_step = STEP3;
                        }
                        break;
            case STEP3:
                        if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, -80.0f, 2.0f) )
                        {
                            my_allstop();
                            trace_speed = 20.0f;
                            flag_step = STEP4;
                        }
                        else
                        {
                            my_turn_left(20.0f);
                        }
                        break;
            case STEP4:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            trace_speed = 5.0f;
                        }
                        if(sensor.sensor_pos1 == SENSOR_YES)
                        {
                            my_allstop();
                            flag_step = STEP5;
                        }
                        break;
            case STEP5:
                        trace_speed = 20.0f;
                        my_allstop();
                        break;
            case STEP6:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            trace_speed = 5.0f;
                        }
                        if(sensor.sensor_pos1 == SENSOR_YES)
                        {
                            my_allstop();
                            flag_step = STEP7;
                        }
                        break;
            case STEP7:
                        my_allstop();
                        Motor_pos = Motor1.PositionMeasure;
                        break;                        
            case STEP8:
                        if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, -54.0f, 2.0f) )
                        {
                            my_allstop();
                            trace_speed = 20.0f;
                            flag_step = STEP9;
                        }
                        else
                        {
                            my_turn_left(20.0f);
                        }
                        break;
            case STEP9:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            my_allstop();
                            Motor_pos = Motor1.PositionMeasure;
                            flag_step = STEP10;
                        }
                        break;
            case STEP10:
                        if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, 100.0f, 2.0f) )
                        {
                            my_allstop();
                            trace_speed = 20.0f;
                            flag_step = STEP11;
                        }
                        else
                        {
                            my_turn_right(20.0f);
                        }
                        break;
             case STEP11:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            trace_speed = 5.0f;
                        }
                        if(sensor.sensor_pos1 == SENSOR_YES)
                        {
                            my_allstop();
                            flag_step = STEP12;
                        }
                        break;
            case STEP12:
                        trace_speed = 20.0f;
                        my_allstop();
                        break;
            case STEP13:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            my_allstop();
                        }
                        break;
        }  //end of switch_case        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
        
    } // end of while(1)    

    
#endif

#ifdef TEST_TASK4
    float trace_speed = 10.0f;
    uint8_t flag_on_road = false;
    uint8_t flag_stop = false;
    uint8_t flag_turn1 = false, flag_turn2 = false;
    uint8_t count_stop_route_state = 0;
    float Motor_pos = 0.0f;
    
    flag_step = STEP1;
    
    Motor1.State=PIDSPEED;
    Motor2.State=PIDSPEED;
    Motor3.State=PIDSPEED;
    while(1)
    {
        sensor_read();
        sensor_bit_info = (sensor.sensor_route[SENSOR_ROUTE1]<<SENSOR_ROUTE1) + \
                            (sensor.sensor_route[SENSOR_ROUTE2]<<SENSOR_ROUTE2) + \
                            (sensor.sensor_route[SENSOR_ROUTE3]<<SENSOR_ROUTE3) + \
                            (sensor.sensor_route[SENSOR_ROUTE4]<<SENSOR_ROUTE4) + \
                            (sensor.sensor_route[SENSOR_ROUTE5]<<SENSOR_ROUTE5) + \
                            (sensor.sensor_route[SENSOR_ROUTE6]<<SENSOR_ROUTE6) + \
                            (sensor.sensor_route[SENSOR_ROUTE7]<<SENSOR_ROUTE7);
        
        switch(flag_step)
        {
            case STEP1:// turn right all_lited to only_D4
                        if(sensor_bit_info == 0x00)
                        {
                            my_turn_left(10.0f);                   
                        }
                        else
                        {
                            flag_turn2 = true;                    
                        }
                        if(flag_turn2 && sensor.sensor_route[SENSOR_ROUTE1] \
                            && sensor.sensor_route[SENSOR_ROUTE2] \
                            && sensor.sensor_route[SENSOR_ROUTE6] \
                            && sensor.sensor_route[SENSOR_ROUTE7] \
                            && sensor.sensor_route[SENSOR_ROUTE5]&& ( sensor.sensor_route[SENSOR_ROUTE4] == 0 | sensor.sensor_route[SENSOR_ROUTE3] == 0 ))// D3/4/5 lited, others all not
                        {
                            flag_turn1 = true;
                        }
                        if(flag_turn1&&flag_turn2)
                        {
                            my_allstop();
                            flag_step = STEP2;
                        }
                        break;
            case STEP2:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            trace_speed = 5.0f;
                        }
                        if(sensor.sensor_pos1 == SENSOR_YES)
                        {
                            my_allstop();
                            flag_step = STEP3;
                        }
                        break;
            case STEP3:
                        my_allstop();
                        Motor_pos = Motor1.PositionMeasure;
                        break;                        
            case STEP4:
                        if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, -32.0f, 2.0f) )
                        {
                            my_allstop();
                            trace_speed = 20.0f;
                            flag_step = STEP5;
                        }
                        else
                        {
                            my_turn_left(20.0f);
                        }
                        break;
            case STEP5:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            my_allstop();
                            Motor_pos = Motor1.PositionMeasure;
                            flag_step = STEP6;
                        }
                        break;
            case STEP6:
                        if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, 74.0f, 2.0f) )
                        {
                            my_allstop();
                            trace_speed = 20.0f;
                            flag_step = STEP7;
                        }
                        else
                        {
                            my_turn_right(20.0f);
                        }
                        break;
             case STEP7:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            trace_speed = 5.0f;
                        }
                        if(sensor.sensor_pos1 == SENSOR_YES)
                        {
                            my_allstop();
                            flag_step = STEP8;
                        }
                        break;
            case STEP8:
                        break;
            case STEP9:
                        trace_speed = 20.0f;
                        my_allstop();
                        break;
            case STEP10:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            my_allstop();
                        }
                        break;
        }  //end of switch_case        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
        
    } // end of while(1)    
    
#endif

#ifdef TEST_TASK5
    float trace_speed = 20.0f;
    uint8_t flag_on_road = false;
    uint8_t flag_stop = false;
    uint8_t flag_turn1 = false, flag_turn2 = false;
    uint8_t count_stop_route_state = 0;
    float Motor_pos = 0.0f;
    
    flag_step = STEP1;
    
    Motor1.State=PIDSPEED;
    Motor2.State=PIDSPEED;
    Motor3.State=PIDSPEED;
    while(1)
    {
        sensor_read();
        sensor_bit_info = (sensor.sensor_route[SENSOR_ROUTE1]<<SENSOR_ROUTE1) + \
                            (sensor.sensor_route[SENSOR_ROUTE2]<<SENSOR_ROUTE2) + \
                            (sensor.sensor_route[SENSOR_ROUTE3]<<SENSOR_ROUTE3) + \
                            (sensor.sensor_route[SENSOR_ROUTE4]<<SENSOR_ROUTE4) + \
                            (sensor.sensor_route[SENSOR_ROUTE5]<<SENSOR_ROUTE5) + \
                            (sensor.sensor_route[SENSOR_ROUTE6]<<SENSOR_ROUTE6) + \
                            (sensor.sensor_route[SENSOR_ROUTE7]<<SENSOR_ROUTE7);
        
        switch(flag_step)
        {
            case STEP1:// turn right all_lited to only_D4
                        if(sensor_bit_info == 0x00)
                        {
                            my_turn_left(20.0f);                   
                        }
                        else
                        {
                            flag_turn2 = true;                    
                        }
                        if(flag_turn2 && sensor.sensor_route[SENSOR_ROUTE1] \
                            && sensor.sensor_route[SENSOR_ROUTE2] \
                            && sensor.sensor_route[SENSOR_ROUTE6] \
                            && sensor.sensor_route[SENSOR_ROUTE7] \
                            && (sensor.sensor_route[SENSOR_ROUTE3] == 0 | sensor.sensor_route[SENSOR_ROUTE4] == 0 | sensor.sensor_route[SENSOR_ROUTE5] == 0 ))// D3/4/5 lited, others all not
                        {
                            flag_turn1 = true;
                        }
                        if(flag_turn1&&flag_turn2)
                        {
                            my_allstop();
                            flag_step = STEP2;
                        }
                        break;
            case STEP2:
                        if(trace_round_route(trace_speed, &sensor_bit_info) == CHANGE_SPEED_OR_STOP)
                        {
                            trace_speed = 5.0f;
                        }
                        if(sensor.sensor_pos1 == SENSOR_YES)
                        {
                            my_allstop();
                            flag_step = STEP3;
                        }
                        break;
            case STEP3:
                        my_allstop();
                        Motor_pos = Motor1.PositionMeasure;
                        break;                        
            case STEP4:
                        if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, -25.0f, 2.0f) )
                        {
                            my_allstop();
                            trace_speed = 20.0f;
                            flag_step = STEP5;
                        }
                        else
                        {
                            my_turn_left(20.0f);
                        }
                        break;
            case STEP5:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            my_allstop();
                            Motor_pos = Motor1.PositionMeasure;
                            flag_step = STEP6;
                        }
                        break;
            case STEP6:
                        if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, 70.0f, 2.0f) )
                        {
                            my_allstop();
                            trace_speed = 20.0f;
                            flag_step = STEP7;
                        }
                        else
                        {
                            my_turn_right(20.0f);
                        }
                        break;
             case STEP7:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            trace_speed = 5.0f;
                        }
                        if(sensor.sensor_pos1 == SENSOR_YES)
                        {
                            my_allstop();
                            flag_step = STEP8;
                        }
                        break;
            case STEP8:
                        break;
            case STEP9:
                        trace_speed = 20.0f;
                        my_allstop();
                        break;
            case STEP10:
                        if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == CHANGE_SPEED_OR_STOP)
                        {
                            my_allstop();
                        }
                        break;
        }  //end of switch_case        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
        
    } // end of while(1)    
    
#endif  //end of TASK5

#ifdef TEST_TASK6
    float trace_speed = 20.0f;
    uint8_t flag_on_road = false;
    uint8_t flag_stop = false;
    uint8_t flag_turn1 = false, flag_turn2 = false;
    uint8_t count_stop_route_state = 0;
    float Motor_pos = 0.0f;
    
    flag_step = STEP1;
    
    Motor1.State=PIDSPEED;
    Motor2.State=PIDSPEED;
    Motor3.State=PIDSPEED;
    while(1)
    {
        sensor_read();
        sensor_bit_info = (sensor.sensor_route[SENSOR_ROUTE1]<<SENSOR_ROUTE1) + \
                            (sensor.sensor_route[SENSOR_ROUTE2]<<SENSOR_ROUTE2) + \
                            (sensor.sensor_route[SENSOR_ROUTE3]<<SENSOR_ROUTE3) + \
                            (sensor.sensor_route[SENSOR_ROUTE4]<<SENSOR_ROUTE4) + \
                            (sensor.sensor_route[SENSOR_ROUTE5]<<SENSOR_ROUTE5) + \
                            (sensor.sensor_route[SENSOR_ROUTE6]<<SENSOR_ROUTE6) + \
                            (sensor.sensor_route[SENSOR_ROUTE7]<<SENSOR_ROUTE7);
        
        switch(flag_step)
        {
            case STEP1:// turn right all_lited to only_D4
                        if(sensor_bit_info == 0x00)
                        {
                            my_turn_left(20.0f);                   
                        }
                        else
                        {
                            flag_turn2 = true;                    
                        }
                        if(flag_turn2 && sensor.sensor_route[SENSOR_ROUTE1] \
                            && sensor.sensor_route[SENSOR_ROUTE2] \
                            && sensor.sensor_route[SENSOR_ROUTE6] \
                            && sensor.sensor_route[SENSOR_ROUTE7] \
                            && sensor.sensor_route[SENSOR_ROUTE5]  && (sensor.sensor_route[SENSOR_ROUTE4] == 0 | sensor.sensor_route[SENSOR_ROUTE3] == 0 ))// D3/4/5 lited, others all not
                        {
                            flag_turn1 = true;
                        }
                        if(flag_turn1&&flag_turn2)
                        {
                            my_allstop();
                            flag_step = STEP2;
                        }
                        break;
            case STEP2:
                        if(trace_round_route(trace_speed, &sensor_bit_info) == CHANGE_SPEED_OR_STOP)
                        {
                            trace_speed = 5.0f;
                        }
                        if(sensor.sensor_pos1 == SENSOR_YES)
                        {
                            my_allstop();
                            flag_step = STEP3;
                        }
                        break;
            case STEP3:
                        my_allstop();
                        break;
        }  //end of switch_case        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
        
    } // end of while(1)    
    
#endif  //end of TASK6



#ifdef FINAL
    float trace_speed = 20.0f;
    uint8_t flag_on_road = false;
    uint8_t flag_stop = false;
    uint8_t flag_turn1 = false, flag_turn2 = false;
    uint8_t count_stop_route_state = 0;
    float Motor_pos = 0.0f;
    
    flag_step = STEP1;
    flag_task = TASK1;
    
    Motor1.State=PIDSPEED;
    Motor2.State=PIDSPEED;
    Motor3.State=PIDSPEED;
    while(1)
    {
        sensor_read();
        sensor_bit_info = (sensor.sensor_route[SENSOR_ROUTE1]<<SENSOR_ROUTE1) + \
                            (sensor.sensor_route[SENSOR_ROUTE2]<<SENSOR_ROUTE2) + \
                            (sensor.sensor_route[SENSOR_ROUTE3]<<SENSOR_ROUTE3) + \
                            (sensor.sensor_route[SENSOR_ROUTE4]<<SENSOR_ROUTE4) + \
                            (sensor.sensor_route[SENSOR_ROUTE5]<<SENSOR_ROUTE5) + \
                            (sensor.sensor_route[SENSOR_ROUTE6]<<SENSOR_ROUTE6) + \
                            (sensor.sensor_route[SENSOR_ROUTE7]<<SENSOR_ROUTE7);
        
        switch(flag_task)
        {
            case TASK1:
                    switch(flag_step)
                    {
                        case STEP1:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        trace_speed = 10.0f;
                                    }
                                    if(!sensor.sensor_pos1)
                                    {
                                        flag_step = STEP2;
                                        my_allstop();
                                    }
                                    break;
                                    
                        case STEP2:
                                    trace_speed = 20.0f;
                                    break;
                        
                        case STEP3:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        trace_speed = 5.0f;
                                    }
                                    if(!sensor.sensor_pos1)
                                    {
                                        flag_step = STEP4;
                                        my_allstop();
                                    }
                                    break;
                                    
                        case STEP4: // put the weight
                                    break;
                        
                        case STEP5: // loose the weight
                                    trace_speed = 20.0f;
                                    break;
                        
                        case STEP6: // backward to the dest.(part1)
                //                        flag_stop = trace_straight_route(trace_speed, &sensor_bit_info);
                //                        if( last_flag_stop == ON_ROAD && (flag_stop == CHANGE_SPEED_OR_STOP || flag_stop == OUT_ROAD_BUT_UNCERTAIN) )
                //                        {
                //                            
                //                        }
                //                        last_flag_stop = flag_stop;
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == ON_ROAD)
                                    {
                                        flag_on_road = true;
                                    }
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == CHANGE_SPEED_OR_STOP && flag_on_road)
                                    {
                                        flag_on_road = false;
                                        count_stop_route_state++; 
                                    }
                                    if(count_stop_route_state == 2)
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_turn1 = false,flag_turn2 = false;
                                        flag_step = STEP1;
                                        flag_task = TASK3;

                                  /********     end of task1           ****************/
                                    } // end of switch task1_step
                                    break;
                    } // end of switch task1
                                break;
            case TASK3:
                    switch(flag_step)
                    {
                        case STEP1:// turn right all_lited to only_D4
                                    if(sensor_bit_info == 0x00)
                                    {
                                        my_turn_right(10.0f);                   
                                    }
                                    else
                                    {
                                        flag_turn2 = true;                    
                                    }
                                    if(flag_turn2 && sensor.sensor_route[SENSOR_ROUTE1] \
                                        && sensor.sensor_route[SENSOR_ROUTE2] \
                                        && sensor.sensor_route[SENSOR_ROUTE6] \
                                        && sensor.sensor_route[SENSOR_ROUTE7] \
                                        && sensor.sensor_route[SENSOR_ROUTE3] &&( sensor.sensor_route[SENSOR_ROUTE4] == 0 | sensor.sensor_route[SENSOR_ROUTE5] == 0 ))// D3/4/5 lited, others all not
                                    {
                                        flag_turn1 = true;
                                    }
                                    if(flag_turn1&&flag_turn2)
                                    {
                                        my_allstop();
                                        flag_step = STEP2;
                                    }
                                    break;
                        case STEP2:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        my_allstop();
                                        Motor_pos = Motor1.PositionMeasure;
                                        flag_step = STEP3;
                                    }
                                    break;
                        case STEP3:
                                    if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, -80.0f, 2.0f) )
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_step = STEP4;
                                    }
                                    else
                                    {
                                        my_turn_left(20.0f);
                                    }
                                    break;
                        case STEP4:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        trace_speed = 5.0f;
                                    }
                                    if(sensor.sensor_pos1 == SENSOR_YES)
                                    {
                                        my_allstop();
                                        flag_step = STEP5;
                                    }
                                    break;
                        case STEP5:
                                    trace_speed = 20.0f;
                                    my_allstop();
                                    break;
                        case STEP6:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        trace_speed = 5.0f;
                                    }
                                    if(sensor.sensor_pos1 == SENSOR_YES)
                                    {
                                        my_allstop();
                                        flag_step = STEP7;
                                    }
                                    break;
                        case STEP7:
                                    my_allstop();
                                    Motor_pos = Motor1.PositionMeasure;
                                    break;                        
                        case STEP8:
                                    if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, -55.0f, 2.0f) )
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_step = STEP9;
                                    }
                                    else
                                    {
                                        my_turn_left(20.0f);
                                    }
                                    break;
                        case STEP9:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        my_allstop();
                                        Motor_pos = Motor1.PositionMeasure;
                                        flag_step = STEP10;
                                    }
                                    break;
                        case STEP10:
                                    if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, 95.0f, 2.0f) )
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_step = STEP11;
                                    }
                                    else
                                    {
                                        my_turn_right(20.0f);
                                    }
                                    break;
                         case STEP11:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        trace_speed = 5.0f;
                                    }
                                    if(sensor.sensor_pos1 == SENSOR_YES)
                                    {
                                        my_allstop();
                                        flag_step = STEP12;
                                    }
                                    break;
                        case STEP12:
                                    trace_speed = 20.0f;
                                    my_allstop();
                                    break;
                        case STEP13:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_turn1 = false,flag_turn2 = false;
                                        flag_step = STEP1;
                                        Motor_pos = Motor1.PositionMeasure;
                                        flag_task = TASK4;
                                    }
                                    break;
                    }  //end of switch_task3_STEP        
                  
                            break;
            
            
            case TASK4: 
                    switch(flag_step)
                    {
//                        case STEP1:// turn right all_lited to only_D4
//                                    if(sensor_bit_info == 0x00)
//                                    {
//                                        my_turn_left(10.0f);                   
//                                    }
//                                    else
//                                    {
//                                        flag_turn2 = true;                    
//                                    }
//                                    if(flag_turn2 && sensor.sensor_route[SENSOR_ROUTE1] \
//                                        && sensor.sensor_route[SENSOR_ROUTE2] \
//                                        && sensor.sensor_route[SENSOR_ROUTE6] \
//                                        && sensor.sensor_route[SENSOR_ROUTE7] \
//                                        && sensor.sensor_route[SENSOR_ROUTE5]&& ( sensor.sensor_route[SENSOR_ROUTE4] == 0 | sensor.sensor_route[SENSOR_ROUTE3] == 0 ))// D3/4/5 lited, others all not
//                                    {
//                                        flag_turn1 = true;
//                                    }
//                                    if(flag_turn1&&flag_turn2)
//                                    {
//                                        my_allstop();
//                                        flag_step = STEP2;
//                                    }
//                                    break;
                        case STEP1:
                                    if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, -115.0f, 2.0f) )
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_step = STEP2;
                                    }
                                    else
                                    {
                                        my_turn_left(20.0f);
                                    }
                                    break;
                        case STEP2:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        trace_speed = 5.0f;
                                    }
                                    if(sensor.sensor_pos1 == SENSOR_YES)
                                    {
                                        my_allstop();
                                        flag_step = STEP3;
                                    }
                                    break;
                        case STEP3:
                                    my_allstop();
                                    Motor_pos = Motor1.PositionMeasure;
                                    break;                        
                        case STEP4:
                                    if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, -30.0f, 2.0f) )
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_step = STEP5;
                                    }
                                    else
                                    {
                                        my_turn_left(20.0f);
                                    }
                                    break;
                        case STEP5:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        my_allstop();
                                        Motor_pos = Motor1.PositionMeasure;
                                        flag_step = STEP6;
                                    }
                                    break;
                        case STEP6:
                                    if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, 74.0f, 2.0f) )
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_step = STEP7;
                                    }
                                    else
                                    {
                                        my_turn_right(20.0f);
                                    }
                                    break;
                         case STEP7:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        trace_speed = 5.0f;
                                    }
                                    if(sensor.sensor_pos1 == SENSOR_YES)
                                    {
                                        my_allstop();
                                        flag_step = STEP8;
                                    }
                                    break;
                        case STEP8:
                                    break;
                        case STEP9:
                                    trace_speed = 20.0f;
                                    my_allstop();
                                    break;
                        case STEP10:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_turn1 = false,flag_turn2 = false;
                                        flag_step = STEP1;
                                        flag_task = TASK5;
                                    }
                                    break;
                    }  //end of switch_case        
                
                                    break;
            
            
            
            case TASK5: 
                    switch(flag_step)
                    {
                        case STEP1:// turn right all_lited to only_D4
                                    if(sensor_bit_info == 0x00)
                                    {
                                        my_turn_left(20.0f);                   
                                    }
                                    else
                                    {
                                        flag_turn2 = true;                    
                                    }
                                    if(flag_turn2 && sensor.sensor_route[SENSOR_ROUTE1] \
                                        && sensor.sensor_route[SENSOR_ROUTE2] \
                                        && sensor.sensor_route[SENSOR_ROUTE6] \
                                        && sensor.sensor_route[SENSOR_ROUTE7] \
                                        && (sensor.sensor_route[SENSOR_ROUTE3] == 0 | sensor.sensor_route[SENSOR_ROUTE4] == 0 | sensor.sensor_route[SENSOR_ROUTE5] == 0 ))// D3/4/5 lited, others all not
                                    {
                                        flag_turn1 = true;
                                    }
                                    if(flag_turn1&&flag_turn2)
                                    {
                                        my_allstop();
                                        flag_step = STEP2;
                                    }
                                    break;
                        case STEP2:
                                    if(trace_round_route(trace_speed, &sensor_bit_info) == CHANGE_SPEED_OR_STOP)
                                    {
                                        trace_speed = 5.0f;
                                    }
                                    if(sensor.sensor_pos1 == SENSOR_YES)
                                    {
                                        my_allstop();
                                        flag_step = STEP3;
                                    }
                                    break;
                        case STEP3:
                                    my_allstop();
                                    Motor_pos = Motor1.PositionMeasure;
                                    break;                        
                        case STEP4:
                                    if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, -25.0f, 2.0f) )
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_step = STEP5;
                                    }
                                    else
                                    {
                                        my_turn_left(20.0f);
                                    }
                                    break;
                        case STEP5:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        my_allstop();
                                        Motor_pos = Motor1.PositionMeasure;
                                        flag_step = STEP6;
                                    }
                                    break;
                        case STEP6:
                                    if( FLAG_CAR_ANGLE_ARRIVAL(Motor_pos, 70.0f, 2.0f) )
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_step = STEP7;
                                    }
                                    else
                                    {
                                        my_turn_right(20.0f);
                                    }
                                    break;
                         case STEP7:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, FORWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        trace_speed = 5.0f;
                                    }
                                    if(sensor.sensor_pos1 == SENSOR_YES)
                                    {
                                        my_allstop();
                                        flag_step = STEP8;
                                    }
                                    break;
                        case STEP8:
                                    break;
                        case STEP9:
                                    trace_speed = 20.0f;
                                    my_allstop();
                                    break;
                        case STEP10:
                                    if(trace_straight_route(trace_speed, &sensor_bit_info, BACKWARD) == CHANGE_SPEED_OR_STOP)
                                    {
                                        my_allstop();
                                        trace_speed = 20.0f;
                                        flag_turn1 = false,flag_turn2 = false;
                                        flag_step = STEP1;
                                        flag_task = TASK6;
                                    }
                                    break;
                    }  //end of switch_case        
                   
                                    break;
            case TASK6:
                    switch(flag_step)
                    {
                        case STEP1:// turn right all_lited to only_D4
                                    if(sensor_bit_info == 0x00)
                                    {
                                        my_turn_left(20.0f);                   
                                    }
                                    else
                                    {
                                        flag_turn2 = true;                    
                                    }
                                    if(flag_turn2 && sensor.sensor_route[SENSOR_ROUTE1] \
                                        && sensor.sensor_route[SENSOR_ROUTE2] \
                                        && sensor.sensor_route[SENSOR_ROUTE6] \
                                        && sensor.sensor_route[SENSOR_ROUTE7] \
                                        && sensor.sensor_route[SENSOR_ROUTE5]  && (sensor.sensor_route[SENSOR_ROUTE4] == 0 | sensor.sensor_route[SENSOR_ROUTE3] == 0 ))// D3/4/5 lited, others all not
                                    {
                                        flag_turn1 = true;
                                    }
                                    if(flag_turn1&&flag_turn2)
                                    {
                                        my_allstop();
                                        flag_step = STEP2;
                                    }
                                    break;
                        case STEP2:
                                    if(trace_round_route(trace_speed, &sensor_bit_info) == CHANGE_SPEED_OR_STOP)
                                    {
                                        trace_speed = 5.0f;
                                    }
                                    if(sensor.sensor_pos1 == SENSOR_YES)
                                    {
                                        my_allstop();
                                        flag_step = STEP3;
                                    }
                                    break;
                        case STEP3:
                                    my_allstop();
                                    break;
                    }  //end of switch_case        
                    
                                break;
            case TASK_OVER:
                    my_allstop();
                    break;
        }  //end of switch_case        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
        
    } // end of while(1)    
    
 //end of FINAL

#endif
#endif
        
} // end of task
