from robodyno import Robot
from robodyno.nodes import MotorFactory,NanoBoardFactory,StepperBoardFactory
import numpy as np
import math
import time

arm_AB = 30.0
arm_BC = 40.0
arm_AO = 7.0
arm_OE = 7.0
arm_CD = 40.0
arm_DE = 30.0
step_odom_all = 0
global theta
theta = 3.14
global keng
keng = 0
global step_now
step_now = 0
global motor_now
motor_now = 0

class DoubleArm(Robot):
    global step_now,motor_now
    def __init__(self):
        super(DoubleArm, self).__init__('can0')
        self.add_device('M1', MotorFactory(), 0x12, reduction = 12.45)
        self.add_device('M2', MotorFactory(), 0x13, reduction = -12.45)
        self.add_device('M3', MotorFactory(), 0x1A, reduction = -100 )
        self.add_device('M4', StepperBoardFactory(), 0x22, reduction = 10)
        self.motors = [
            self.device('M1'),
            self.device('M2'),
            self.device('M3'),
            self.device('M4'),
        ]
        #for motor in self.motors:
        #    motor.position_filter_mode(8)
        self._zeros = [0 for i in range(4)]
        self._poses = [0 for i in range(4)]
        #self._steppers_poses = 0
        #self._steppers_zeros = 0
        #self.stepper = self.add_device('peanutstep', StepperBoardFactory(), 0x22, reduction = 10)
        
    def init(self):
        for i in range(3):
            pos = None
            if(i ==2):
                pos = 0.0
            else:
                while not pos:
                    pos = self.motors[i].get_pos(0.3)
                    #motors[3].get_pos(0.3)
            self._zeros[i] = pos
            self._poses[i] = 0
            print("i:",i)
        #self._steppers_zeros = self.stepper.get_pos(0.3)
        #self._steppers_poses = 0
        #print("steppers_zeros_init:",self._steppers_position," _steppers_poses:",self._steppers_poses)

    #get motor position information
    def get_motor_position_message(self):
        for i in range(4):
            print("motor","i",self.motors[i].get_pos(0.3) )

    #set motor position
    def set_motor_pos(self, i, pos):
        self.motors[i].set_pos(pos + self._zeros[i])
        self._poses[i] = pos


    def move_to_axis(self, target, dur = 0):
        if dur > 0:
            t0 = self.time()
            start = self._poses.copy()
            while self.time() - t0 < dur :
                ratio = (self.time() - t0) / dur
                for i in range(3):
                    pos = (target[i]-start[i])*ratio+start[i]
                    self.set_motor_pos(i, pos)
                self.delay(0.02)
        for i in range(3):
            self.set_motor_pos(i, target[i])
      
    
    def home(self, dur = 5):
        self.move_to_axis((0,0,0), dur)

    #-------------------------------------------------------------------------电机进入滤波模式
    def set_position_fliter_mode(self, i, flash_hz):
        self.motors[i].position_filter_mode(flash_hz)

    #-------------------------------------------------------------------------获取电机控制模式信息（直接位置、匀加/减速模式......）
    def get_motor_mode(self,i):
        print(self.motors[i].get_controller_modes(1))
    
    #-------------------------------------------------------------------------设置电机速度及电流上限
    def set_motor_vel_inter(self, _input_max_vel, _input_max_i):
        for i in range(2):
            self.motors[i].set_limits(_input_max_vel, _input_max_i)

    #-------------------------------------------------------------------------读取电机速度及电流上限
    def read_motor_get_limits(self):
        self.motors[0].get_limits(1)
        self.motors[1].get_limits(1)
        time.sleep(1)
        print("motor0:", self.motors[0].get_limits(1))
        time.sleep(1)
        print("motor1:", self.motors[1].get_limits(1))
    #-------------------------------------------------------------------------读取电机（速度、加速度大小、减速度大小）信息
    def set_motor_get_traj_mode_params(self):
        self.motors[0].get_traj_mode_params()
        self.motors[1].get_traj_mode_params()
        time.sleep(1)
        print("motor0:", self.motors[0].get_traj_mode_params())
        time.sleep(1)
        print("motor1:", self.motors[1].get_traj_mode_params())
    
    #-------------------------------------------------------------------------读取电机控制模式信息
    def read_get_motor_mode(self):
        self.motors[0].get_controller_modes(1)
        self.motors[1].get_controller_modes(1)
        time.sleep(1)
        print("motor0_mode:", self.motors[0].get_controller_modes(1))
        time.sleep(1)
        print("motor1_mode:", self.motors[1].get_controller_modes(1)) 
    
    #-------------------------------------------------------------------------设置电机匀速、加速度大小、减速度大小
    def write_position_traj_mode(self, whichmotor, vel, acc_up, acc_down):
        self.motors[whichmotor].position_traj_mode(vel, acc_up, acc_down)
        print("motor",whichmotor," set sucessfully!")
        time.sleep(0.5)
        
    def peanut_jump(self, jump_numbers):
        if jump_numbers == 0:
            #self.set_motor_pos(self._zeros[2])
            self.motors[2].set_pos(self._zeros[2])
        else:
            for current_number in range(jump_numbers):
                self._poses[2] = self._poses[2] + 3.1415*2
                self.set_motor_pos(2,self._poses[2])
    
    #-------------------------------------------------------------------------设置步进电机细分、最大速度、加速度大小
    def set_stepper_vel_acc_limit(self, _input_xifen, max_stepperspeed, acc_stepper):
        self.motors[3].set_subdivision(_input_xifen) 
        self.motors[3].set_vel_acc_limit( max_stepperspeed , acc_stepper )

    #-------------------------------------------------------------------------设置步进电机位置
    def stepper_position(self, _input_pos):
        self.motors[3].set_pos(_input_pos + self._zeros[3])
        self._poses[3] = _input_pos    
    #-------------------------------------------------------------------------步进电机复位
    def stepper_position_reset(self):
        self.motors[3].set_pos(self._zeros[3])
        
    def input_position_message(self,point_x, point_y):
        ########################################################################calculate motor 0x12 degree
        if (point_x < -37) or (point_x > 37) or (point_y < 30) or (point_y > 70):
            print( "Sorry, data is wrong!" )
            print(" Check your data of input,please ")
        else:
            arm_AF=0.0
            arm_AC=0.0
            arm_EF=0.0
            arm_CE=0.0
            if ( point_x < (-arm_AO) ):
                arm_AF=math.fabs( math.fabs(point_x) -arm_AO )
                arm_AC=math.sqrt(point_y*point_y + arm_AF*arm_AF)
                arm_EF=math.fabs(point_x) + arm_OE
                arm_CE=math.sqrt( point_y*point_y + arm_EF*arm_EF )
            elif ( (point_x > -arm_AO) and (point_x < 0) ):
                arm_AC=math.sqrt(point_y*point_y + (arm_AO-math.fabs(point_x))*(arm_AO-math.fabs(point_x)) )
                arm_CE=math.sqrt( point_y*point_y + ( math.fabs(point_x) + arm_AO )*( math.fabs(point_x) + arm_AO ) )
            elif (point_x == (-arm_AO)):
                arm_AC = point_y
                arm_CE = math.sqrt( point_y*point_y + (2*arm_AO)*(2*arm_AO) )
            elif(point_x == 0):
                arm_AC = math.sqrt( point_y*point_y + arm_AO*arm_AO )
                arm_CE = math.sqrt( point_y*point_y + arm_AO*arm_AO )            
            elif( (point_x  >0) and (point_x < arm_OE) ):
                arm_AC = math.sqrt( point_y*point_y + (point_x+arm_AO)*(point_x+arm_AO) )
                arm_CE = math.sqrt( point_y*point_y + (arm_AO-point_x)*(arm_AO-point_x) )
            elif( point_x == arm_OE ):
                arm_AC = math.sqrt( point_y*point_y + (2*arm_AO)*(2*arm_AO) )
                arm_CE = point_y
            elif( point_x>arm_OE ):
                arm_AC = math.sqrt( point_y*point_y + (point_x+arm_AO)*(point_x+arm_AO) )
                arm_CE = math.sqrt( point_y*point_y + (point_x-arm_AO)*(point_x-arm_AO) )                
            else:
                print("warning:data is not exit!")
                print("warning:data is not exit!!")
                print("warning:data is not exit!!!")
                print("Now, you need check your data,please!")
            
            cos_CAB= (arm_AB*arm_AB + arm_AC*arm_AC - arm_BC*arm_BC) / (2*arm_AB*arm_AC)
            angle_CAB= math.acos(cos_CAB)


            cos_CAE= ((arm_AO+arm_OE)*(arm_AO+arm_OE) + (arm_AC*arm_AC) - (arm_CE*arm_CE)) / (2*arm_AO*2*arm_AC)
            angle_CAE=math.acos(cos_CAE)
            angle_BAF=math.pi-angle_CAE-angle_CAB
            ########################################################################calculate motor 0x13 degree
            cos_CEA= ( (2*arm_AO)*(2*arm_AO) + arm_CE*arm_CE - arm_AC*arm_AC) / (2* 2*arm_AO * arm_CE)
            angle_CEA=math.acos(cos_CEA)
            cos_CED= (arm_CE*arm_CE + arm_DE*arm_DE - arm_CD*arm_CD) / (2*arm_CE*arm_DE)
            angle_CED=math.acos(cos_CED)
            angle_DEG=math.pi-angle_CED-angle_CEA
            #print("angle_BAF:", angle_BAF, "angle_DEG:", angle_DEG)
            rad_to_angle_baf= (180/math.pi)*angle_BAF
            rad_to_angle_deg= (180/math.pi)*angle_DEG
            #print("-----------------------------------")
            #print("motor_12_angle:",rad_to_angle_baf)
            #print("motor_13_angle:",rad_to_angle_deg)
            self.motors[0].set_pos(angle_BAF + self._zeros[0])
            self.motors[1].set_pos(angle_DEG + self._zeros[1])
            #print("step_now:",step_now)

    # 圆 方 法     
    def peanum_arm_move(self, left_point, right_point, delta_point, high_point, stepper_ups, stepper_downs, stepper_delay):
        global keng,theta,step_now
        list = np.arange( left_point, right_point, delta_point )
        keng = 0
        print("step_now:",step_now)
        self.set_position_fliter_mode(0,4)
        self.set_position_fliter_mode(1,4)
        for point in list:
            especially_data = 0
            keng = keng + 1
            do_once = 1
            data_param = [0, 0, 2]
            self.stepper_position(step_now + stepper_ups)
            self.delay(stepper_delay)
            for i in range(40):
                theta += 6.28/10
                if theta > 6.28: 
                    theta -= 6.28
                x = data_param[0] + data_param[2] * math.cos(theta)
                y = data_param[1] + data_param[2] * math.sin(theta)
                if (( keng <= 8) and (do_once == 1) ):
                    do_once = 2
                    self.input_position_message(point-0.5,high_point)
                    self.delay(0.5)
                    self.input_position_message(point-0.5,high_point -0.5)
                    self.delay(0.5)
                    self.input_position_message(point-0.5,high_point -1)
                    self.delay(0.5)
                    self.input_position_message(point-0.5,high_point -1.5)
                    self.delay(0.5)
                    self.input_position_message(point-0.5,high_point -2.0)
                    self.delay(0.5)            
                    heiheihei = 2
                    #print("i am here")
                    self.input_position_message(point,high_point)
                    self.delay(1.8)
                elif (keng >=9) and (keng<=24):
                    especially_data = 1
                    #print("i am check myself")
                    self.input_position_message(point + round(x,2),high_point + round(y,2)+0.22)
                    self.delay(0.0234)
                else:
                    self.input_position_message(point + round(x,2),high_point + round(y,2))
                    self.delay(0.0234)
                    wahaha = 1
                    especially_data = 2

            if especially_data == 1:
                #print("teshu")
                self.stepper_position(step_now + stepper_downs)
                self.delay(0.2)
                self.input_position_message(point,high_point+0.92)
                self.delay(1.8)
            else:
                self.stepper_position(step_now + stepper_downs)
                self.delay(0.2)                
                self.input_position_message(point,high_point)
                self.delay(1.8)
                #print("hello")
            self.peanut_jump(2)
            self.delay(3.5)
            print("keng:",keng)
        self.stepper_position(step_now + stepper_ups)
        self.write_position_traj_mode(0, 0.2+0.1, 2+0.5, 2+0.5)
        self.write_position_traj_mode(1, 0.2+0.1, 2+0.5, 2+0.5)

            
            
    # 圆 方 法     
    def peanum_arm_move_first(self, left_point, right_point, delta_point, high_point, stepper_ups, stepper_downs, stepper_delay):
        global keng,theta,step_now
        list = np.arange( left_point, right_point, delta_point )
        keng = 0
        print("step_now:",step_now)
        for point in list:
            especially_data = 0
            keng = keng + 1
            do_once = 1
            data_param = [0, 0, 2]
            self.stepper_position(step_now + stepper_ups)
            self.delay(stepper_delay)
            for i in range(40):
                theta += 6.28/10
                if theta > 6.28: 
                    theta -= 6.28
                x = data_param[0] + data_param[2] * math.cos(theta)
                y = data_param[1] + data_param[2] * math.sin(theta)
                if (keng >=9) and (keng<=24):
                    especially_data = 1
                    #print("i am check myself")
                    self.input_position_message(point + round(x,2),high_point + round(y,2)+0.22)
                    self.delay(0.0234)
                else:
                    self.input_position_message(point + round(x,2),high_point + round(y,2))
                    self.delay(0.0234)
                    wahaha = 1
                    especially_data = 2

            if especially_data == 1:
                #print("teshu")
                self.stepper_position(step_now + stepper_downs)
                self.delay(0.2)
                self.input_position_message(point,high_point+0.92)
                self.delay(1.8)
            else:
                self.stepper_position(step_now + stepper_downs)
                self.delay(0.2)                
                self.input_position_message(point,high_point)
                self.delay(1.8)
                #print("hello")
            self.peanut_jump(2)
            self.delay(3.5)
            #print("keng:",keng)
        self.stepper_position(step_now + stepper_ups)
        print("end")
            
            
    # 左 右 摆 动 方 法
    def peanum_arm_move_second(self, left_point, right_point, delta_point, high_point, stepper_ups, stepper_downs, stepper_delay, arm_delay,delta_point_one, counts):
        global keng,theta,step_now
        self.set_position_fliter_mode(0,4)
        self.set_position_fliter_mode(1,4)
        list = np.arange( left_point, right_point, delta_point )
        keng = 0
        for point in list:
            keng = keng +1
            if keng == 1:
                print("first")
                self.input_position_message(point,high_point-0.5)
                self.delay(arm_delay+1.9)
                #self.stepper_position(step_now + stepper_ups)
                #self.delay(stepper_delay)
            elif keng == 2:
                print("second")
                self.input_position_message(point,high_point-0.8)
                self.delay(arm_delay+1.9)
                #self.stepper_position(step_now + stepper_ups)
                #self.delay(stepper_delay)
                #self.input_position_message(point,high_point-delta_point_one)
                #self.delay(3)
            elif keng == 3:
                print("third")
                self.input_position_message(point,high_point-1)
                self.delay(arm_delay+1.9)
            elif keng == 11:
                print("notice:",keng)
                self.stepper_position(step_now + stepper_ups)
                self.delay(stepper_delay)
                self.input_position_message(point,high_point+2)
                self.delay(arm_delay+1.9) 
            elif keng == 12:
                print("notice:",keng)
                self.stepper_position(step_now + stepper_ups)
                self.delay(stepper_delay)
                self.input_position_message(point,high_point+2)
                self.delay(arm_delay+1.9)
            else:
                self.stepper_position(step_now + stepper_ups)
                self.delay(stepper_delay)
                self.input_position_message(point-2,high_point+2)
                self.delay(arm_delay)
                self.input_position_message(point-2,high_point)
                self.delay(arm_delay)
            
            if(keng <=3):
                print("keng:",keng)
            else:
                self.input_position_message(point,high_point)
                self.delay(arm_delay+1.5)
                self.stepper_position(step_now + stepper_downs)
                self.delay(stepper_delay)            
                self.peanut_jump(counts)
                self.delay(3.5)
        print("end")
        self.stepper_position(step_now + stepper_ups)
        self.write_position_traj_mode(0, 0.2+0.1, 2+0.5, 2+0.5)
        self.write_position_traj_mode(1, 0.2+0.1, 2+0.5, 2+0.5)
