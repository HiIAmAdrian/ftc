package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by adrian on 16/03/2018.
 */

@Autonomous(name = "AutoMovement")
public class AutoMovement extends LinearOpMode
{
    private DcMotor     motorRightFront;
    private DcMotor     motorRightBack;
    private DcMotor     motorLeftFront;
    private DcMotor     motorLeftBack;
    private static final double COUNTS_PER_MOTOR_REV    = 1440 ;
    private static final double WHEEL_DIAMETER          = 10;
    private static final double COUNTS_PER_CM           = ((COUNTS_PER_MOTOR_REV * 1.5) / (WHEEL_DIAMETER * 3.1415));

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");

        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                motorLeftFront.getCurrentPosition(),
                motorLeftBack.getCurrentPosition(),
                motorRightBack.getCurrentPosition(),
                motorRightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();

        /*DriveRight(1,1,1,1,3);
        sleep(250);
        DriveForward(1, 1);
        sleep(250);
        DriveBackwards(1, 1, 1, 1, 1);
        sleep(250);*/
        DriveLeft(0.3, 0.3, 0.3, 0.3, 100);
        sleep(250);
    }
        public void DriveForward(double power, int distance)
        {
            int range = getDistance(distance);
            resetEncoders();
            SetDistance(range);
            runToPositionEncoders();
            encodersSetPower(power, -power, power, -power);
            while (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() && motorRightFront.isBusy()) {
                telemetry.addData("Path",  "Running at %7d :%7d :%7d, :%7d",
                        motorRightFront.getCurrentPosition(),
                        motorLeftBack.getCurrentPosition(),
                        motorRightFront.getCurrentPosition(),
                        motorRightBack.getCurrentPosition());
                telemetry.update();
            }
            encodersSetPower(0, 0, 0, 0);
            runEncoders();
        }

        public void DriveBackwards(double power, double power1, double power2, double power3, int distance)
        {
            int range = getDistance(distance);
            resetEncoders();
            SetDistance(range);
            runToPositionEncoders();
            encodersSetPower(-power, power1, -power2, power3);
            while (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() && motorRightFront.isBusy()) {
                telemetry.addData("Path",  "Running at %7d :%7d :%7d, :%7d",
                        motorRightFront.getCurrentPosition(),
                        motorLeftBack.getCurrentPosition(),
                        motorRightFront.getCurrentPosition(),
                        motorRightBack.getCurrentPosition());
                telemetry.update();
            }
            encodersSetPower(0, 0,0, 0);
            runEncoders();
        }

        public void DriveRight(double power, double power1, double power2, double power3, int distance)
        {
            int range = getDistance(distance);
            resetEncoders();
            SetDistance(range);
            runToPositionEncoders();
            encodersSetPower(-power, -power1, power2, power3);
            while (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() && motorRightFront.isBusy()) {
                telemetry.addData("Path", "Running at %7d :%7d :%7d, :%7d",
                        motorRightFront.getCurrentPosition(),
                        motorLeftBack.getCurrentPosition(),
                        motorRightFront.getCurrentPosition(),
                        motorRightBack.getCurrentPosition());
                telemetry.update();
            }
            encodersSetPower(0, 0, 0, 0);
            runEncoders();
        }

        private void DriveLeft(double power, double power1, double power2, double power3, int distance)
        {
            int range = getDistance(distance);
            resetEncoders();
            runEncoders();
            SetDistance(range);
            runToPositionEncoders();
            encodersSetPower(power, -power1, power2, -power3);
            telemetry.update();
            while (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() && motorRightFront.isBusy()) {
                telemetry.addData("Path", "Running at %7d :%7d :%7d, :%7d",
                        motorRightFront.getCurrentPosition(),
                        motorLeftBack.getCurrentPosition(),
                        motorRightFront.getCurrentPosition(),
                        motorRightBack.getCurrentPosition());
                telemetry.update();
            }

            encodersSetPower(0, 0, 0, 0);
            runEncoders();
        }

        private void SetDistance(int distance)
        {
            telemetry.addData("Distance: ", distance);
            motorLeftBack.setTargetPosition(distance);
            motorLeftFront.setTargetPosition(distance);
            motorRightFront.setTargetPosition(distance);
            motorRightBack.setTargetPosition(distance);
        }

        private void resetEncoders()
        {
            telemetry.addData("StopAndResetEncoders", ": ok");
            motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        private void runToPositionEncoders()
        {
            telemetry.addData("RunToPosition", ": ok");
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        private void encodersSetPower(double power, double power2, double power3, double power4)
        {
            motorLeftFront.setPower(power);
            telemetry.addData("motorLeftFront: ", power);
            motorLeftBack.setPower(power2);
            telemetry.addData("motorLeftFront: ", power2);

            motorRightFront.setPower(power3);
            telemetry.addData("motorLeftFront: ", power3);

            motorRightBack.setPower(power4);
            telemetry.addData("motorLeftFront: ", power4);

        }

        private void runEncoders()
        {
            telemetry.addData("Running encoders", ": ok");
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        private int getDistance(int distance)
        {
            return (motorLeftFront.getCurrentPosition() + (int)(COUNTS_PER_CM * distance));
        }
}