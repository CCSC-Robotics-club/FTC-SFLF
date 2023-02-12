package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Middle_Auto")


public class AutoMiddle extends LinearOpMode {
    Hardwaremap autohwp = new Hardwaremap();
    EncoderDrive eg =new EncoderDrive();


    static final double ANDYMARK_PER_MOTOR_REV = 560;    // eg: ANDYMARK Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (ANDYMARK_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    ElapsedTime runtime = new ElapsedTime();


    static final double DRIVE_SPEED = 0.4;



    @Override
    public void runOpMode() throws InterruptedException {
        autohwp.init(hardwareMap);


        autohwp.Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        autohwp.Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        autohwp.Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        autohwp.Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autohwp.Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autohwp.Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autohwp.Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        autohwp.Leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autohwp.Leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autohwp.Rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autohwp.Rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        autohwp.Leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
        autohwp.Rightfront.setDirection(DcMotorSimple.Direction.FORWARD);
        autohwp.Leftback.setDirection(DcMotorSimple.Direction.REVERSE);
        autohwp.Rightback.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();





        encoderDrive(DRIVE_SPEED,5,5,5,5,10);



    }


    public void encoderDrive(double speed, double leftfrontInches, double rightfrontInches,double rightbackInches,double leftbackInches, double timeoutS) {
        int newLeftfrontTarget;
        int newLeftbackTarget;
        int newRightfrontTarget;
        int newRightbackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftfrontTarget = autohwp.Leftfront.getCurrentPosition() + (int) (leftfrontInches * COUNTS_PER_INCH);
            newLeftbackTarget = autohwp.Leftback.getCurrentPosition() + (int) (leftbackInches * COUNTS_PER_INCH);
            newRightfrontTarget = autohwp.Rightfront.getCurrentPosition() + (int) (rightfrontInches * COUNTS_PER_INCH);
            newRightbackTarget = autohwp.Rightback.getCurrentPosition() + (int) (rightbackInches * COUNTS_PER_INCH);
            autohwp.Leftfront.setTargetPosition(newLeftfrontTarget);
            autohwp.Leftback.setTargetPosition(newLeftbackTarget);
            autohwp.Rightfront.setTargetPosition(newRightfrontTarget);
            autohwp.Rightback.setTargetPosition(newRightbackTarget);

            // Turn On RUN_TO_POSITION
            autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autohwp.Rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            autohwp.Leftfront.setPower(-speed);
            autohwp.Rightfront.setPower(-speed);
            autohwp.Leftback.setPower(-speed);
            autohwp.Rightback.setPower(-speed);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && runtime.seconds() < timeoutS &&
                    Math.abs(-autohwp.Leftfront.getCurrentPosition() - autohwp.Leftfront.getTargetPosition()) > 50 &&
                    Math.abs(-autohwp.Leftback.getCurrentPosition() - autohwp.Leftback.getTargetPosition()) > 50 &&
                    Math.abs(-autohwp.Rightfront.getCurrentPosition() - autohwp.Rightfront.getTargetPosition()) > 50 &&
                    Math.abs(-autohwp.Rightback.getCurrentPosition() - autohwp.Rightback.getTargetPosition()) > 50) {


                // Display it for the driver.
                telemetry.addData("Path1", autohwp.Leftfront.getTargetPosition());
                telemetry.addData("Path2", autohwp.Leftfront.getCurrentPosition());

                telemetry.addData("Leftfront busy",autohwp.Leftfront.isBusy());
                telemetry.addData("Leftback busy",autohwp.Leftback.isBusy());
                telemetry.addData("Rightback busy",autohwp.Rightback.isBusy());
                telemetry.addData("Rightfront busy",autohwp.Rightfront.isBusy());

                telemetry.update();
            }
            // Stop all motion;

            autohwp.Leftfront.setPower(0);
            autohwp.Leftback.setPower(0);
            autohwp.Rightfront.setPower(0);
            autohwp.Rightback.setPower(0);
            // Turn off RUN_TO_POSITION
            autohwp.Leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autohwp.Rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(50);   // optional pause after each move
        }
    }
}
