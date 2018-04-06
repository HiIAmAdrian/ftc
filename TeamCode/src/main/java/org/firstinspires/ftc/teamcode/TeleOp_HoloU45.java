package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by cristi & adrian on 14/03/2018.
 */

@TeleOp (name = "TeleOp_HoloU45")

public class TeleOp_HoloU45 extends LinearOpMode
{
    double  xValue, yValue, U1 = Math.PI/4, U2 = 3*Math.PI/4, U3 = 5*Math.PI/4, U4 = 7*Math.PI/4, r, teta, u, l=22.25, L=25.25, t, tM;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private Servo   servoDownLeft;
    private Servo   servoDownRight;
    private Servo   servoUpLeft;
    private Servo   servoUpRight;

    @Override
    public void runOpMode() throws InterruptedException {
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        servoDownRight = hardwareMap.servo.get("servoDownRight");
        servoDownLeft = hardwareMap.servo.get("servoDownLeft");
        servoUpRight = hardwareMap.servo.get("servoUpRight");
        servoUpLeft = hardwareMap.servo.get("servoUpLeft");

        //motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        //motorLeftFront.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode: ","waiting");
        telemetry.update();

        u = (l/2) / (L/2);
        //uM = (L/2) / (l/2);
        t = Math.atan2(l/2,L/2);//era atan
        tM= 2*Math.atan2(L/2,l/2);
        U1 = t;
        U2 = t + tM;
        U3= U2 + 2*t;
        U4 = U3 + tM;


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("tM = ", Math.toDegrees(tM));
            telemetry.addData("U1 = ", Math.toDegrees(t));
            telemetry.addData("U2 = ", Math.toDegrees(U2));
            telemetry.addData("U3 = ", Math.toDegrees(U3));
            telemetry.addData("U4 = ", Math.toDegrees(U4));
            telemetry.addData("Mode: ", "running");
            telemetry.addData("Joystick: ", "x = " + gamepad1.left_stick_x + " y = " + gamepad1.left_stick_y);
            xValue = gamepad1.left_stick_x;
            yValue = -gamepad1.left_stick_y;//fara minus
            telemetry.addData("x buton = ", gamepad1.x);
            if (gamepad1.x) {
                telemetry.addData("dsdasds", "  intra");
                servoDownLeft.setPosition(0.5);
                servoDownRight.setPosition(0.5);
                servoUpLeft.setPosition(0.5);
                servoUpRight.setPosition(0.5);
            }
            if (gamepad1.a){
                servoUpLeft.setPosition(0.4);
                servoUpRight.setPosition(0.4);
                servoDownLeft.setPosition(0.4);
                servoDownRight.setPosition(0.4);
            }
            if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                motorRightFront.setPower(gamepad1.right_stick_x);
                motorRightBack.setPower(gamepad1.right_stick_x);
                motorLeftFront.setPower(gamepad1.right_stick_x);
                motorLeftBack.setPower(gamepad1.right_stick_x);
            }
            if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                if (xValue == 0) {
                    telemetry.addData("xfront = ", gamepad1.left_stick_x);
                    telemetry.addData("yfront = ", gamepad1.left_stick_y);
                    telemetry.update();
                    motorRightFront.setPower(-gamepad1.left_stick_y);
                    motorRightBack.setPower(-gamepad1.left_stick_y);
                    motorLeftFront.setPower(gamepad1.left_stick_y);
                    motorLeftBack.setPower(gamepad1.left_stick_y);
                }

                if (xValue != 0) {
                    r = Math.sqrt((xValue * xValue) + (yValue * yValue));

                    teta = Math.atan(yValue / xValue);
                    if (xValue == -1 && yValue == 0)
                        teta = Math.PI;
            //   if (yValue / xValue > -1 && yValue / xValue < 1)
             //      teta = teta +
                   /*if (xValue > 0 && yValue > 0)
                        teta = Math.atan(yValue / xValue);
                    else if (xValue < 0 && yValue > 0)
                        teta = teta + Math.PI;
                    else if (xValue < 0 && yValue < 0)
                        teta = teta + Math.PI;
                    else if (xValue > 0 && yValue < 0)
                        teta = teta + 2 * Math.PI;*/

                    telemetry.addData("teta= ", Math.toDegrees(teta));

                    r = Range.clip(r, -1.0, 1.0);
                    telemetry.addData("r(dupa clip)= ", r);


                    if (xValue > 0 && yValue > 0)
                        setSpeed(speedMotor(r, teta, U1),-speedMotor(r, teta, U2), speedMotor(r, teta, U3), -speedMotor(r, teta, U4));
                    if (xValue > 0 && yValue < 0)
                        setSpeed(-speedMotor(r, teta, U1),speedMotor(r, teta, U2), -speedMotor(r, teta, U3), speedMotor(r, teta, U4));
                    if (yValue == 0 && xValue > 0) {
                        setSpeed(-speedMotor(r, teta, U1), -speedMotor(r, teta, U2), -speedMotor(r, teta, U3), -speedMotor(r, teta, U4));}
                    if (yValue == 0 && xValue < 0) {
                        setSpeed(-speedMotor(r, teta, U1), -speedMotor(r, teta, U2), -speedMotor(r, teta, U3), -speedMotor(r, teta, U4));
                    }
                    if ((xValue < 0 && yValue > 0) || (xValue < 0 && yValue < 0))
                        setSpeed(speedMotor(r, teta, U1), speedMotor(r, teta, U2), speedMotor(r, teta, U3), speedMotor(r, teta, U4));
                }
            }
        }
    }//inverseaza cadraneleeeee
    public double speedMotor(double r, double teta, double unghi) {
        double teta_mare = unghi - teta;//tele teta1
        teta_mare = Math.sin(teta_mare);//tele teta2
        teta_mare *= r;//tele teta3
        return (teta_mare);
    }
    public void setSpeed(double s1, double s2, double s3, double s4)
    {
        motorRightFront.setPower(s1);
        telemetry.addData("MotorRightFront:", s1);

        motorLeftFront.setPower(s2);
        telemetry.addData("MotorLeftFront: ", s2);

        motorLeftBack.setPower(s3);
        telemetry.addData("MotorLeftBack: ", s3);

        motorRightBack.setPower(s4);
        telemetry.addData("MotorRightBack: ", s4);
        telemetry.update();
    }
}