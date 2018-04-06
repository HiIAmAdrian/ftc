package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by cristi & adrian on 14/03/2018.
 */

@TeleOp (name = "TeleOp_HoloTest")

public class TeleOp_HoloTest extends LinearOpMode {
    double xValue, yValue, r;
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

        telemetry.addData("Mode: ", "waiting");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            xValue = gamepad1.left_stick_x;
            yValue = -gamepad1.left_stick_y;
            r = Math.atan(yValue / xValue);
            r = Range.clip(r, -1.0, 1.0);
            if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                motorRightFront.setPower(-gamepad1.right_stick_x);
                motorRightBack.setPower(-gamepad1.right_stick_x);
                motorLeftFront.setPower(gamepad1.right_stick_x);
                motorLeftBack.setPower(gamepad1.right_stick_x);
            }
            if (xValue > 0 && yValue > 0)
                setSpeed(0, r, r, 0);
            if (xValue < 0 && yValue > 0)
                setSpeed(r, 0, 0, r);
            if (xValue < 0 && yValue < 0)
                setSpeed(0, -r, -r, 0);
            if (xValue > 0 && yValue < 0)
                setSpeed(-r, 0, 0, -r);
            if (yValue > 0 && xValue == 0)
                setSpeed(r, r, r, r);
            if (yValue < 0 && xValue == 0)
                setSpeed(-r, -r, -r, -r);
            if (yValue == 0 && xValue < 0)
                setSpeed(r, -r, -r, r);
            if (yValue == 0 && xValue > 0)
                setSpeed(-r, r, r, -r);
        }
    }

    public void setSpeed(double power1, double power2, double power3, double power4)
    {
        motorRightFront.setPower(power1);
        motorRightBack.setPower(power2);
        motorLeftFront.setPower(power3);
        motorLeftBack.setPower(power4);
    }

    //public double speedMotor(double r, double alpha, double unghi) {
  //      return (r * Math.sin(alpha - unghi));
    //}
}