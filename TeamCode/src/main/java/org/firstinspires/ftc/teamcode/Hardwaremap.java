package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Hardwaremap {

    DcMotorEx Leftfront=null; //1
    DcMotorEx Rightfront=null; //0
    DcMotorEx Leftback=null; //2
    DcMotorEx Rightback=null; //3

    DcMotorEx Lift_pulleys =null;
    Servo   Catch=null;

    HardwareMap hwp=null;
    //DcMotor arm_motor =null;

    public Hardwaremap(){}

    public void init(HardwareMap HwMap){
        hwp=HwMap;
        /*Rightback = hwp.get(DcMotorEx.class, "RightBack");
        Leftback = hwp.get(DcMotorEx.class, "LeftBack");
        Rightfront = hwp.get(DcMotorEx.class, "RightFront");*/
        Leftfront = hwp.get(DcMotorEx.class, "Leftfront");
        Rightfront = hwp.get(DcMotorEx.class, "Rightfront");
        Leftback = hwp.get(DcMotorEx.class, "Leftback");
        Rightback = hwp.get(DcMotorEx.class, "Rightback");

        Lift_pulleys = hwp.get(DcMotorEx.class,"Liftpulleys");
        Catch=hwp.get(Servo.class,"Catch");



        Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       Lift_pulleys.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Leftfront1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Leftfront2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift_pulleys.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //反转
        Leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
        Rightfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Leftback.setDirection(DcMotorSimple.Direction.REVERSE);
        Rightback.setDirection(DcMotorSimple.Direction.FORWARD);

        Lift_pulleys.setDirection(DcMotorSimple.Direction.REVERSE);
        Catch.setDirection(Servo.Direction.FORWARD);


        /*arm_motor = hwp.dcMotor.get("arm");

        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);


         */
    }
}
