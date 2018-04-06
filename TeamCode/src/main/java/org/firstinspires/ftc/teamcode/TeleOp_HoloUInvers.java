package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by cristi & adrian on 14/03/2018.
 */

@TeleOp (name = "TeleOp_HoloUInvers")

public class TeleOp_HoloUInvers extends LinearOpMode
{

    private DcMotor motorRightFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private DcMotor motorGlisiera;
    private DcMotor motorPapusa;
    private double POWER_REDUCTION = 3;



    @Override
    public void runOpMode() throws InterruptedException {
        //xValue, yValue = valorile citite de la left joystick
        //U1, 2, 3, 4 = unghiurile dintre roti
        //l, L = latimea si Lungimea dreptunghiului format de roti
        //t = unghiul format de ox - ul joystick-ului cu prima roata(RightFront)

        //22.5 25.5
        double xValue, yValue, U1, U2, U3, U4, r, teta, l=35, L=37, t, tM;//37/35 de fapt l si L
        Servo   servoDownLeft;
        Servo   servoDownRight;
        Servo   servoUpLeft;
        Servo   servoUpRight;
        Servo   servoPapusa;
        Servo   servoSenzor;

        //getters
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        servoDownRight = hardwareMap.servo.get("servoDownRight");
        servoDownLeft = hardwareMap.servo.get("servoDownLeft");
        servoUpRight = hardwareMap.servo.get("servoUpRight");
        servoUpLeft = hardwareMap.servo.get("servoUpLeft");
        motorGlisiera = hardwareMap.dcMotor.get("motorGlisiera");
        motorPapusa = hardwareMap.dcMotor.get("motorPapusa");
        servoSenzor = hardwareMap.servo.get("servoSensor");
        servoPapusa = hardwareMap.servo.get("servoPapusa");

        telemetry.addData("Mode: ","waiting");
        telemetry.update();


        t = Math.atan2(l / 2, L / 2);
        tM= 2 * Math.atan2(L / 2, l / 2);
        //unghiurile dintre ox-ul joystick-ului si roatile(RightFront,LeftFront,LeftBack,RightBack)
        U1 = t;
        U2 = t + tM;
        U3 = U2 + 2 * t;
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
            yValue = -gamepad1.left_stick_y;
            //TURBO ON/OFF
            if (gamepad1.x)
                POWER_REDUCTION = 1;
            if (gamepad1.b)
                POWER_REDUCTION = 2.5;

            servoSenzor.setPosition(0.0);
            //motorul de la glisiera
            if (gamepad2.right_stick_y == 0)
                motorGlisiera.setPower(0);
            if (gamepad2.right_stick_y != 0)
                motorGlisiera.setPower(gamepad2.right_stick_y);

            motorPapusa.setPower(gamepad2.left_stick_y / 3);//misca brat de papudsa
            if (gamepad2.left_stick_y == 0)
                motorPapusa.setPower(0);
            //deschis pt manuta
            if (gamepad2.left_bumper)
                servoPapusa.setPosition(0.65);
            //inchis pt manuta
            if (gamepad2.right_bumper)
                servoPapusa.setPosition(0.0);
            //deschide bratele de la glisiera
            if (gamepad2.a){
                servoUpLeft.setPosition(0.2);
                servoUpRight.setPosition(0.6);
                servoDownLeft.setPosition(0.1);
                servoDownRight.setPosition(0.9);
            }
            //inchide bratele de la glisiera
            if(gamepad2.b)
            {
                servoDownLeft.setPosition(0.85);
                sleep(120);
                servoDownRight.setPosition(0.15);
                servoUpRight.setPosition(0.0);
                sleep(120);
                servoUpLeft.setPosition(0.8);
            }

            //rotatie
            if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                motorRightFront.setPower(gamepad1.right_stick_x / 3);
                motorRightBack.setPower(gamepad1.right_stick_x / 3);
                motorLeftFront.setPower(gamepad1.right_stick_x / 3);
                motorLeftBack.setPower(gamepad1.right_stick_x / 3);
            }
            //pentru miscare fata/spate
            if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                if (xValue == 0) {
                    telemetry.addData("xfront = ", gamepad1.left_stick_x);
                    telemetry.addData("yfront = ", gamepad1.left_stick_y);
                    telemetry.update();
                    motorRightFront.setPower(-gamepad1.left_stick_y / POWER_REDUCTION);
                    motorRightBack.setPower(-gamepad1.left_stick_y / POWER_REDUCTION);
                    motorLeftFront.setPower(gamepad1.left_stick_y / POWER_REDUCTION);
                    motorLeftBack.setPower(gamepad1.left_stick_y / POWER_REDUCTION);
                }

                //calculele efective pentru restul miscarii(transformare coord cardinale in coord polare
                if (xValue != 0) {
                    //rezultanta dintre x-ul si y-ul joystickului
                    r = Math.sqrt((xValue * xValue) + (yValue * yValue));

                    //unghiul dintre rezultanta si ox-ul joystick-ului
                    teta = Math.atan(yValue / xValue);
                    //if in caz ca x-ul < 0 si y == 0 => teta e pi
                    if (xValue < 0 && yValue == 0)
                        teta = Math.PI;
                    telemetry.addData("teta= ", Math.toDegrees(teta));

                    //rezultanta poate fi mai mare de 1 in caz ca nu e pe ox sau oy, deci ii punem cap
                    r = Range.clip(r, -1.0, 1.0);
                    telemetry.addData("r(dupa clip)= ", r);

                    //if-uri pentru cadrane(dupa cadrane ne ajustam directiile)
                    if (xValue > 0 && yValue > 0) {
                        telemetry.addData("Intra in C1 cu :" ,"x = " + xValue + "y = " + yValue);
                        setSpeed(speedMotor(r, teta, U1), -speedMotor(r, teta, U2), speedMotor(r, teta, U3), -speedMotor(r, teta, U4));
                    }
                    if (xValue > 0 && yValue < 0) {
                        telemetry.addData("Intra in C4 cu :", "x = " + xValue + "y = " + yValue);
                        setSpeed(-speedMotor(r, teta, U1), speedMotor(r, teta, U2), -speedMotor(r, teta, U3), speedMotor(r, teta, U4));
                    }
                    if (yValue == 0 && xValue > 0) {
                        telemetry.addData("Intra in dreapta cu :" ,"x = " + xValue + "y = " + yValue);
                        setSpeed(-speedMotor(r, teta, U1), -speedMotor(r, teta, U2), -speedMotor(r, teta, U3), -speedMotor(r, teta, U4));
                    }
                    if (yValue == 0 && xValue < 0) {
                        telemetry.addData("Intra in stanga cu :" ,"x = " + xValue + "y = " + yValue);
                        setSpeed(-speedMotor(r, teta, U1), -speedMotor(r, teta, U2), -speedMotor(r, teta, U3), -speedMotor(r, teta, U4));
                    }
                    if ((xValue < 0 && yValue > 0) || (xValue < 0 && yValue < 0)) {
                        telemetry.addData("Intra in C2 sau 3 cu :" ,"x = " + xValue + "y = " + yValue);
                        setSpeed(speedMotor(r, teta, U1), speedMotor(r, teta, U2), speedMotor(r, teta, U3), speedMotor(r, teta, U4));
                    }
                }
            }
            idle();
        }
    }

    private double speedMotor(double r, double teta, double unghi) {
        //aici calculeaza power-ul pentru fiecare roata in functie de rezultanta, unghiul dintre rezultanta si ox si in functie de unghiul dintre roata si ox
        double teta_mare = unghi - teta;//tele teta1
        teta_mare = Math.sin(teta_mare);//tele teta2
        teta_mare *= r;//tele teta3
        return (teta_mare);
    }

    private void setSpeed(double s1, double s2, double s3, double s4)
    {
        //obvious
        motorRightFront.setPower(s1 / POWER_REDUCTION);
        telemetry.addData("MotorRightFront:", s1 / POWER_REDUCTION);

        motorLeftFront.setPower(s2 / POWER_REDUCTION);
        telemetry.addData("MotorLeftFront: ", s2 / POWER_REDUCTION);

        motorLeftBack.setPower(s3 / POWER_REDUCTION);
        telemetry.addData("MotorLeftBack: ", s3 / POWER_REDUCTION);

        motorRightBack.setPower(s4 / POWER_REDUCTION);
        telemetry.addData("MotorRightBack: ", s4 / POWER_REDUCTION);
        telemetry.update();
    }
}