#include "system_config.h"
#include "task_init.h"
#include "motor_control.h"



void TaskMonitor(void *p_arg)
{
    OS_ERR err;

#ifdef DEBUG
    uint8_t flag = false;
    
    Motor5.State=IDLE;    
    Motor6.State=IDLE;
    
    
#ifdef MOTOR_RESET

    
    Motor1.State=IDLE;
    Motor2.State=IDLE;
    Motor3.State=IDLE;
    Motor5.State=IDLE;    
    Motor6.State=PIDSPEED;
    
    while (1)
    {
        sensor_read();
        if(sensor.sensor_reset&&(!flag))
        {
            Motor6.SpeedExpected = -100.0f;
        }
        else
        {
            Motor6.SpeedExpected = 0.0f;
            flag = true;
        }
        OSTimeDlyHMSM(0, 0, 0, 5, OS_OPT_TIME_HMSM_STRICT, &err);
    }


#endif
    
#else
    
#ifdef TEST_TASK1

    uint8_t flag_once = false;
    Motor5.State=PIDPOSITION;    
    Motor6.State=PIDPOSITION;
    
    flag_step = STEP1;
    
    while(1)
    {
        switch(flag_step)
        {
            case STEP1:
                        Motor6.PositionExpected = POS_GET;
                        break;
            case STEP2:
                        if( FLAG_MOTOR6_ARRIVAL(POS_GET) && (!flag_once) ) // only execute once, for getting the weight
                        {
                            set_servo_angle(CLOSE);
                            OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                            flag_get_pid = true;
                            Motor6.PositionExpected = POS_PRE_PUT_LOW;
                            flag_once = true; // for  only execute once
                        }
                        if(FLAG_MOTOR6_ARRIVAL(POS_PRE_PUT_LOW))
                        {
                            flag_step = STEP3;
                        }
                        break;
            case STEP3:
                        break;
            case STEP4: // put the weight
                        Motor6.PositionExpected = POS_PUT_LOW;
                        if(FLAG_MOTOR6_ARRIVAL1(POS_PUT_LOW, 2.0f))
                        {                        
                            flag_step = STEP5;
                            flag_once = false ; // for  only execute once
                        }
                        break;
            case STEP5: // loose the weight
                        // only execute once, for reseting the servo

                        set_servo_angle(LOOSE);
                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                        flag_get_pid = false;
                        flag_step = STEP6; // only execute once

                        break;
            case STEP6: // backward to the dest.
                        motor_reset();
                        break;
        }// end of switch_case
        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
    }//end of while
    
#endif

#ifdef TEST_TASK2

    uint8_t flag_once = false;
    Motor5.State=PIDPOSITION;    
    Motor6.State=PIDPOSITION;
    
    flag_step = STEP1;
    
    while(1)
    {
        switch(flag_step)
        {
            case STEP1:
                        Motor6.PositionExpected = POS_GET;
                        break;
            
            case STEP2:
                        break;
            
            case STEP3: // just get the weight
                        if( FLAG_MOTOR6_ARRIVAL(POS_GET) && (!flag_once) ) // only execute once, for getting the weight
                        {
                            set_servo_angle(CLOSE);
                            OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                            flag_get_pid = true;
                            Motor6.PositionExpected = POS_GET - 800.0f;
                            flag_once = true; // for  only execute once
                        }
                        if(FLAG_MOTOR6_ARRIVAL(POS_GET - 800.0f))
                        {
                            flag_step = STEP4;
                        }
                        break;
            case STEP4: // save the weight
                        if(store_weight(POS_STORE2))
                        {
                            flag_step = STEP5;
                        }
                        break;
            case STEP5: // loose the weight
                        motor_reset();
                        break;
        }// end of switch_case
        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
    }//end of while
    
#endif    

#ifdef TEST_TASK3
    uint8_t flag_once = false;
    Motor5.State=PIDPOSITION;    
    Motor6.State=PIDPOSITION;
    
    flag_step = STEP1;
    
    while(1)
    {
        switch(flag_step)
        {
            case STEP1:
                        Motor6.PositionExpected = POS_GET;
                        break;
            
            case STEP2:
                        break;
            case STEP3:
                        break;
            case STEP4:
                        break;
            
            case STEP5: //  get and store the weight1
                        if(get_and_store(POS_STORE1))
                        {
                            flag_step = STEP6;
                        }
                        break;
            case STEP6: // prepare to get the weight2 and trace
                        Motor6.PositionExpected = POS_GET;
                        break;
            case STEP7: // get the weight
                        if( FLAG_MOTOR6_ARRIVAL(POS_GET) && (!flag_once) ) // only execute once, for getting the weight
                        {
                            set_servo_angle(CLOSE);
                            OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                            flag_get_pid = true;
                            Motor6.PositionExpected = POS_GET - 800.0f;
                            flag_once = true; // for  only execute once
                        }
                        if(FLAG_MOTOR6_ARRIVAL(POS_GET - 800.0f))
                        {
                            flag_step = STEP8;
                        }
                        break;
            case STEP8: // prepare to put the weight2
            case STEP9: 
            case STEP10: 
            case STEP11: 
                        Motor6.PositionExpected = POS_PRE_PUT_HIGH;
                        break;
            case STEP12:
                        Motor6.PositionExpected = POS_PUT_HIGH;
                        if(FLAG_MOTOR6_ARRIVAL1(POS_PUT_HIGH, 2.0f))
                        {
                            set_servo_angle(LOOSE);
                            OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                            flag_step = STEP13;
                        }
                        break;
            case STEP13: 
                        break;
        }// end of switch_case
        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
    }//end of while


#endif // TEST_TASK3

#ifdef TEST_TASK4
    uint8_t flag_once = false;
    Motor5.State=PIDPOSITION;    
    Motor6.State=PIDPOSITION;
    
    flag_step = STEP1;
    
    while(1)
    {
        switch(flag_step)
        {
            case STEP1:
                        Motor6.PositionExpected = POS_GET;
                        break;
            
            case STEP2:
                        break;
            
            case STEP3: // get the weight
                        if( FLAG_MOTOR6_ARRIVAL(POS_GET) && (!flag_once) ) // only execute once, for getting the weight
                        {
                            set_servo_angle(CLOSE);
                            OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                            flag_get_pid = true;
                            Motor6.PositionExpected = POS_GET - 800.0f;
                            flag_once = true; // for  only execute once
                        }
                        if(FLAG_MOTOR6_ARRIVAL(POS_GET - 800.0f))
                        {
                            flag_step = STEP4;
                        }
                        break;
            case STEP4: // prepare to put the weight
            case STEP5: 
            case STEP6:
            case STEP7:
                        Motor6.PositionExpected = POS_PRE_PUT_HIGH ;
                        break;
            case STEP8:
                        if(FLAG_MOTOR6_ARRIVAL1(POS_PRE_PUT_HIGH, 2.0f))
                        {
                            flag_step = STEP9;
                        }
            case STEP9:
                        Motor6.PositionExpected = POS_PUT_HIGH;
                        if(FLAG_MOTOR6_ARRIVAL1(POS_PUT_HIGH, 2.0f))
                        {
                            set_servo_angle(LOOSE);
                            OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                            flag_step = STEP10;
                        }
                        break;
            case STEP10: 
                        break;
        }// end of switch_case
        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
    }//end of while

#endif
    
#ifdef TEST_TASK5
    uint8_t flag_once = false;
    Motor5.State=PIDPOSITION;    
    Motor6.State=PIDPOSITION;
    
    flag_step = STEP1;
    
    while(1)
    {
        switch(flag_step)
        {
            case STEP1:
                        Motor6.PositionExpected = POS_GET;
                        break;
            
            case STEP2:
                        break;
            
            case STEP3: // get the weight
                        if( FLAG_MOTOR6_ARRIVAL(POS_GET) && (!flag_once) ) // only execute once, for getting the weight
                        {
                            set_servo_angle(CLOSE);
                            OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                            flag_get_pid = true;
                            Motor6.PositionExpected = POS_GET - 800.0f;
                            flag_once = true; // for  only execute once
                        }
                        if(FLAG_MOTOR6_ARRIVAL(POS_GET - 800.0f))
                        {
                            flag_step = STEP4;
                        }
                        break;
            case STEP4: // prepare to put the weight
            case STEP5: 
            case STEP6:
            case STEP7:
                        Motor6.PositionExpected = POS_PRE_PUT_HIGH ;
                        break;
            case STEP8:
                        if(FLAG_MOTOR6_ARRIVAL1(POS_PRE_PUT_HIGH, 2.0f))
                        {
                            flag_step = STEP9;
                        }
            case STEP9:
                        Motor6.PositionExpected = POS_PUT_HIGH;
                        if(FLAG_MOTOR6_ARRIVAL1(POS_PUT_HIGH, 2.0f))
                        {
                            set_servo_angle(LOOSE);
                            OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                            flag_step = STEP10;
                        }
                        break;
            case STEP10: 
                        break;
        }// end of switch_case
        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
    }//end of while

#endif  //end of TASK5
    

#ifdef TEST_TASK6
    uint8_t flag_once = false;
    Motor5.State=PIDPOSITION;    
    Motor6.State=PIDPOSITION;
    
    flag_step = STEP1;
    
    while(1)
    {
        switch(flag_step)
        {
            case STEP1:
            case STEP2:
                        if(!flag_once)  // only execute once,
                        {
                            get_from_the_store(POS_STORE1);
                            flag_once = true;
                        }
                        break;
            case STEP3: 
                        Motor6.PositionExpected = POS_PUT_HIGH;
                        if(FLAG_MOTOR6_ARRIVAL1(POS_PUT_HIGH, 2.0f))
                        {
                            set_servo_angle(LOOSE);
                            OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                            flag_step = STEP10;
                        }
                        break;
        }// end of switch_case
        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
    }//end of while

#endif  //end of TASK6
    
#ifdef FINAL
    uint8_t flag_once = false;
    Motor5.State=PIDPOSITION;    
    Motor6.State=PIDPOSITION;
    
    flag_step = STEP1;
    flag_task = TASK1;
    while(1)
    {
        switch(flag_task)
        {
            case TASK1:
                    switch(flag_step)
                    {
                        case STEP1:
                                    Motor6.PositionExpected = POS_GET;
                                    break;
                        case STEP2:
                                    if( FLAG_MOTOR6_ARRIVAL(POS_GET) && (!flag_once) ) // only execute once, for getting the weight
                                    {
                                        set_servo_angle(CLOSE);
                                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                                        flag_get_pid = true;
                                        Motor6.PositionExpected = POS_PRE_PUT_LOW;
                                        flag_once = true; // for  only execute once
                                    }
                                    if(FLAG_MOTOR6_ARRIVAL(POS_PRE_PUT_LOW))
                                    {
                                        flag_step = STEP3;
                                    }
                                    break;
                        case STEP3:
                                    break;
                        case STEP4: // put the weight
                                    Motor6.PositionExpected = POS_PUT_LOW;
                                    if(FLAG_MOTOR6_ARRIVAL1(POS_PUT_LOW, 2.0f))
                                    {                        
                                        flag_step = STEP5;
                                        flag_once = false ; // for  only execute once
                                    }
                                    break;
                        case STEP5: // loose the weight
                                    // only execute once, for reseting the servo

                                    set_servo_angle(LOOSE);
                                    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                                    flag_get_pid = false;
                                    flag_step = STEP6; // only execute once

                                    break;
                        case STEP6: // backward to the dest.
                                    flag_once = false;
                                    break;
                    }// end of switch_case
                    
                   
                                    break;
            
            
            
            case TASK3:
                    switch(flag_step)
                    {
                        case STEP1:
                                    Motor6.PositionExpected = POS_GET;
                                    break;
                        
                        case STEP2:
                                    break;
                        case STEP3:
                                    break;
                        case STEP4:
                                    break;
                        
                        case STEP5: //  get and store the weight1
                                    if(get_and_store(POS_STORE1))
                                    {
                                        flag_step = STEP6;
                                    }
                                    break;
                        case STEP6: // prepare to get the weight2 and trace
                                    Motor6.PositionExpected = POS_GET;
                                    break;
                        case STEP7: // get the weight
                                    if( FLAG_MOTOR6_ARRIVAL(POS_GET) && (!flag_once) ) // only execute once, for getting the weight
                                    {
                                        set_servo_angle(CLOSE);
                                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                                        flag_get_pid = true;
                                        Motor6.PositionExpected = POS_GET - 800.0f;
                                        flag_once = true; // for  only execute once
                                    }
                                    if(FLAG_MOTOR6_ARRIVAL(POS_GET - 800.0f))
                                    {
                                        flag_step = STEP8;
                                    }
                                    break;
                        case STEP8: // prepare to put the weight2
                        case STEP9: 
                        case STEP10: 
                        case STEP11: 
                                    Motor6.PositionExpected = POS_PRE_PUT_HIGH;
                                    break;
                        case STEP12:
                                    Motor6.PositionExpected = POS_PUT_HIGH;
                                    if(FLAG_MOTOR6_ARRIVAL1(POS_PUT_HIGH, 2.0f))
                                    {
                                        set_servo_angle(LOOSE);
                                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                                        flag_step = STEP13;
                                    }
                                    break;
                        case STEP13: 
                                    flag_once = false;
                                    break;
                    }// end of switch_case

                                    break;
            

            case TASK4: 
                    switch(flag_step)
                    {
                        case STEP1:
                                    Motor6.PositionExpected = POS_GET;
                                    break;
                        
                        case STEP2:
                                    break;
                        
                        case STEP3: // get the weight
                                    if( FLAG_MOTOR6_ARRIVAL(POS_GET) && (!flag_once) ) // only execute once, for getting the weight
                                    {
                                        set_servo_angle(CLOSE);
                                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                                        flag_get_pid = true;
                                        Motor6.PositionExpected = POS_GET - 800.0f;
                                        flag_once = true; // for  only execute once
                                    }
                                    if(FLAG_MOTOR6_ARRIVAL(POS_GET - 800.0f) && flag_once)
                                    {
                                        flag_step = STEP4;
                                    }
                                    break;
                        case STEP4: // prepare to put the weight
                        case STEP5: 
                        case STEP6:
                        case STEP7:
                                    Motor6.PositionExpected = POS_PRE_PUT_HIGH ;
                                    break;
                        case STEP8:
                                    if(FLAG_MOTOR6_ARRIVAL1(POS_PRE_PUT_HIGH, 2.0f))
                                    {
                                        flag_step = STEP9;
                                    }
                                    break;
                        case STEP9:
                                    Motor6.PositionExpected = POS_PUT_HIGH;
                                    if(FLAG_MOTOR6_ARRIVAL1(POS_PUT_HIGH, 2.0f))
                                    {
                                        set_servo_angle(LOOSE);
                                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                                        flag_step = STEP10;
                                    }
                                    break;
                        case STEP10: 
                                    flag_once = false;
                                    break;
                    }// end of switch_case
                    
                                    break;
            
            
            
            case TASK5: 
                    switch(flag_step)
                    {
                        case STEP1:
                                    Motor6.PositionExpected = POS_GET;
                                    break;
                        
                        case STEP2:
                                    break;
                        
                        case STEP3: // get the weight
                                    if( FLAG_MOTOR6_ARRIVAL(POS_GET) && (!flag_once) ) // only execute once, for getting the weight
                                    {
                                        set_servo_angle(CLOSE);
                                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                                        flag_get_pid = true;
                                        Motor6.PositionExpected = POS_GET - 800.0f;
                                        flag_once = true; // for  only execute once
                                    }
                                    if(FLAG_MOTOR6_ARRIVAL(POS_GET - 800.0f))
                                    {
                                        flag_step = STEP4;
                                    }
                                    break;
                        case STEP4: // prepare to put the weight
                        case STEP5: 
                        case STEP6:
                        case STEP7:
                                    Motor6.PositionExpected = POS_PRE_PUT_HIGH ;
                                    break;
                        case STEP8:
                                    if(FLAG_MOTOR6_ARRIVAL1(POS_PRE_PUT_HIGH, 2.0f))
                                    {
                                        flag_step = STEP9;
                                    }
                        case STEP9:
                                    Motor6.PositionExpected = POS_PUT_HIGH;
                                    if(FLAG_MOTOR6_ARRIVAL1(POS_PUT_HIGH, 2.0f))
                                    {
                                        set_servo_angle(LOOSE);
                                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                                        flag_step = STEP10;
                                    }
                                    break;
                        case STEP10:
                                    flag_once = false;
                                    break;
                    }// end of switch_case
                    
                   
                                    break;
                    
            case TASK6:
                    switch(flag_step)
                    {
                        case STEP1:
                        case STEP2:
                                    if(!flag_once)  // only execute once,
                                    {
                                        get_from_the_store(POS_STORE1);
                                        flag_once = true;
                                    }
                                    break;
                        case STEP3: 
                                    Motor6.PositionExpected = POS_PUT_HIGH;
                                    if(FLAG_MOTOR6_ARRIVAL1(POS_PUT_HIGH, 2.0f))
                                    {
                                        set_servo_angle(LOOSE);
                                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); 
                                        flag_task = TASK_OVER;
                                    }
                                    break;
                    }// end of switch_case
                    
                  
                                break;
            case TASK_OVER: 
                        break;
        }// end of switch_case_TASK
        
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
    }//end of while
#endif

#endif  //end   of  DEUG
}

//    uint8_t flag1 = false, flag2 = false;
//    
//    Motor5.State=PIDPOSITION;    
//    Motor6.State=PIDPOSITION;
//    
   
/**** 拾取砝码,并放置于STORE1(right) ****/
//        /*** test1 ***/
//    flag1 = 1;
//    while(1)
//    {
//        switch(flag1)
//        {
//            case 0:
//                        Motor5.PositionExpected = 0;
//                        if(FLAG_MOTOR5_ARRIVAL)
//                        {
//                            flag1 = 1;
//                        }
//                        break;
//            case 1:
//                        Motor5.PositionExpected = 100;
//                        if(FLAG_MOTOR5_ARRIVAL)
//                        {
//                            flag1 = 0;
//                        }
//                        break;
//         }
//        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);

//    }

        
        /*** test2 ***/
//        get_and_store(POS_STORE1);

/**** translation movement test ****/
//        my_left_translation(10.0f);
//        OSTimeDlyHMSM(0, 0, 3, 0, OS_OPT_TIME_HMSM_STRICT, &err);
//        my_right_translation(10.0f);
//        OSTimeDlyHMSM(0, 0, 3, 0, OS_OPT_TIME_HMSM_STRICT, &err);


/**** sensor.route test_turn ****/
//         switch(sensor_bit_info)
//         {
//            case 0x7F:// all_unlited  0111 1111
//                my_allstop();
//                break;
//            case 0x00:// all_lited  0000 0000
//                if(flag2)
//                {
//                    flag1 = true;
//                }
//                if(flag1&&flag2)
//                {
//                    my_allstop();
//                    /*  next step  */
//                }
//                else
//                {
//                    my_turn_right(20.0f);                    
//                }
//                break; 
//            default:
//                flag2 = true;
//                break;
//          }


/* snesor.pos1(行程开关) test */
//        if(sensor_bit_info == 0x00)
//        {
//            flag2 = true;
//        }
//        if(sensor.sensor_pos1&&(!flag1))
//        {
//            if(flag2)
//            {
//                my_forward(5);            
//            }
//            else
//            {
//                my_forward(20);
//            }

//        }
//        else
//        {
//            flag1 = true;
//            my_allstop();
//        }

