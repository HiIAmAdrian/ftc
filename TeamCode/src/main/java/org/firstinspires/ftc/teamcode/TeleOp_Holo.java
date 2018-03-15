package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by cristi & adrian on 14/03/2018.
 */

@TeleOp (name = "TeleOp Holo")

public class TeleOp_Holo extends LinearOpMode
{
    double xValue, yValue,U1 = 45, U2 = 135, U3 = 225,U4 = 315, r, alpha;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;

    @Override
    public void runOpMode() throws InterruptedException {
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");

        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode: ","waiting");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            xValue = gamepad1.left_stick_x;
            yValue = gamepad1.left_stick_y;
            if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                motorRightFront.setPower(-gamepad1.right_stick_x);
                motorRightBack.setPower(-gamepad1.right_stick_x);
                motorLeftFront.setPower(gamepad1.right_stick_x);
                motorLeftBack.setPower(gamepad1.right_stick_x);
            }
            if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                if (xValue == 0) {
                    telemetry.addData("xfront = ", gamepad1.left_stick_x);
                    telemetry.addData("yfront = ", gamepad1.left_stick_y);
                    telemetry.update();
                    motorRightFront.setPower(gamepad1.left_stick_y);
                    motorRightBack.setPower(gamepad1.left_stick_y);
                    motorLeftFront.setPower(gamepad1.left_stick_y);
                    motorLeftBack.setPower(gamepad1.left_stick_y);
                }

                if (xValue != 0) {
                    telemetry.addData("Mode: ", "running");
                    telemetry.addData("Joystick: ", "x = " + xValue + " y = " + yValue);

                    r = Math.sqrt((xValue * xValue) + (yValue * yValue));
                    telemetry.addData("r(inainte de clip): ", r);

                    alpha = Math.atan(yValue / xValue) * 180;
                    telemetry.addData("alpha= ", alpha);

                    r = Range.clip(r, -1.0, 1.0);
                    telemetry.addData("r(dupa clip)= ", r);

                    motorRightFront.setPower(speedMotor(r, alpha, U1));
                    telemetry.addData("MotorRightFront:", speedMotor(r, alpha, U1));

                    motorRightBack.setPower(speedMotor(r, alpha, U2));
                    telemetry.addData("MotorRightBack: ", speedMotor(r, alpha, U2));

                    motorLeftFront.setPower(speedMotor(r, alpha, U3));
                    telemetry.addData("MotorLeftFront: ", speedMotor(r, alpha, U3));

                    motorLeftBack.setPower(speedMotor(r, alpha, U4));
                    telemetry.addData("MotorLeftBack: ", speedMotor(r, alpha, U4));
                    telemetry.update();
                }
            }
        }
    }
    public double speedMotor(double r, double alpha, double unghi) {
        // double number1 =
        return (r * Math.sin(unghi - alpha));

        /*if (number1 > 1)
            return 1;
        else if (number1 < -1)
            return -1;
        return number1;*/
    }
}