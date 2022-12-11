package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="player1 and player2")
public class Opmode_player1_2 extends LinearOpMode {
    Hardwaremap telehwp = new Hardwaremap();
    double MinS =0.5;
    int p1 = 0;
    int p2 = 0;
    int a = 10;
    double motorspeed = 0.2;
    double MS=3;

    double M1,M2,M3,M4;
    double MotorMaxspeed=1;//0.7  0.4


    @Override
    public void runOpMode() throws InterruptedException {

        telehwp.init(hardwareMap);
        //telehwp.Leftfront1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //telehwp.Leftfront2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telehwp.Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telehwp.Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telehwp.Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telehwp.Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       // telehwp.Lift_pulleys.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telehwp.Lift_pulleys.setTargetPosition(0);
        telehwp.Lift_pulleys.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*telehwp.Leftfront1.setTargetPosition(0);
        telehwp.Leftfront2.setTargetPosition(0);
        telehwp.Leftfront1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telehwp.Leftfront2.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //急停BRAKE 漂浮FLOAT
        telehwp.Leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telehwp.Rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telehwp.Leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telehwp.Rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while (opModeIsActive()){


            /*telehwp.Leftfront.setPower(gamepad1.left_stick_y);
            telehwp.Rightfront.setPower(gamepad1.left_stick_y);
            telehwp.Leftback.setPower(gamepad1.left_stick_y);
            telehwp.Rightback.setPower(gamepad1.left_stick_y);*/

            int position1 = telehwp.Leftfront.getCurrentPosition();
            int position2 = telehwp.Rightfront.getCurrentPosition();
            int position3 = telehwp.Leftback.getCurrentPosition();
            int position4 = telehwp.Rightback.getCurrentPosition();

            //double Lift_pulleys = telehwp.Lift_pulleys.getCurrentPosition();

            telemetry.addData("Leftfront Position", position1);
            telemetry.addData("Rightfront Position", position2);
            telemetry.addData("Leftback Position", position3);
            telemetry.addData("Rightback Position", position4);
           // telemetry.addData("Lift_pulleys",Lift_pulleys);
            telemetry.update();

            /*    p1 += gamepad2.left_stick_x * a;
                telehwp.Leftfront1.setTargetPosition(p1);


                p2 += gamepad2.left_stick_y * a;
                telehwp.Leftfront1.setTargetPosition(p2);
            telehwp.Leftfront1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telehwp.Leftfront2.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/


            //直连
            /*M1= (-gamepad1.left_stick_y-(gamepad1.left_stick_x*0.9)-(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M2= (-gamepad1.left_stick_y+(gamepad1.left_stick_x*0.9)+(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M3= (-gamepad1.left_stick_y+(gamepad1.left_stick_x*0.9)-(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M4= (-gamepad1.left_stick_y-(gamepad1.left_stick_x*0.9)+(gamepad1.right_stick_x/2))*MotorMaxspeed;

            telehwp.Leftfront.setPower(M1);
            telehwp.Rightfront.setPower(M2);
            telehwp.Leftback.setPower(M3);
            telehwp.Rightback.setPower(M4);*/

            /*if (gamepad1.x){
                telehwp.Lift_pulleys.setTargetPosition(10);
                telehwp.Lift_pulleys.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }*/
            //telehwp.Lift_pulleys.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            telehwp.Lift_pulleys.setPower(gamepad2.right_stick_y);

            M1= (gamepad1.left_stick_y-(gamepad1.left_stick_x*0.9)-(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M2= (gamepad1.left_stick_y+(gamepad1.left_stick_x*0.9)+(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M3= (gamepad1.left_stick_y+(gamepad1.left_stick_x*0.9)-(gamepad1.right_stick_x/2))*MotorMaxspeed;
            M4= (gamepad1.left_stick_y-(gamepad1.left_stick_x*0.9)+(gamepad1.right_stick_x/2))*MotorMaxspeed;

            telehwp.Leftfront.setPower(M1);
            telehwp.Rightfront.setPower(M2);
            telehwp.Leftback.setPower(M3);
            telehwp.Rightback.setPower(M4);


            /*
            if (gamepad1.left_trigger!=0){
                telehwp.Leftfront.setPower(motorspeed);
                telehwp.Rightfront.setPower(motorspeed);
                telehwp.Leftback.setPower(motorspeed);
                telehwp.Rightback.setPower(motorspeed);
            }

            if (gamepad1.right_trigger!=0){
                telehwp.Leftfront.setPower(-motorspeed);
                telehwp.Rightfront.setPower(-motorspeed);
                telehwp.Leftback.setPower(-motorspeed);
                telehwp.Rightback.setPower(-motorspeed);
            }
            if (gamepad1.left_bumper){
                telehwp.Leftfront.setPower(MS*motorspeed);
                telehwp.Rightfront.setPower(-MS*motorspeed);
                telehwp.Leftback.setPower(-MS*motorspeed);
                telehwp.Rightback.setPower(MS*motorspeed);
            }
            if (gamepad1.right_bumper){
                telehwp.Leftfront.setPower(-MS*motorspeed);
                telehwp.Rightfront.setPower(MS*motorspeed);
                telehwp.Leftback.setPower(MS*motorspeed);
                telehwp.Rightback.setPower(-MS*motorspeed);
            }
            */

            if(gamepad1.x){
                telehwp.Leftfront.setPower(-MinS*motorspeed);
                telehwp.Rightfront.setPower(MinS*motorspeed);
                telehwp.Leftback.setPower(-MinS*motorspeed);
                telehwp.Rightback.setPower(MinS*motorspeed);
            }

            if (gamepad1.b){
                telehwp.Leftfront.setPower(MinS*motorspeed);
                telehwp.Rightfront.setPower(-MinS*motorspeed);
                telehwp.Leftback.setPower(MinS*motorspeed);
                telehwp.Rightback.setPower(-MinS*motorspeed);
            }

            if (gamepad2.y){
                telehwp.Catch.setPosition(0);
            }
            if (gamepad2.x){
                telehwp.Catch.setPosition(1);
            }


            //telehwp.arm_motor.setPower(gamepad1.right_stick_x);

        }

    }
}
