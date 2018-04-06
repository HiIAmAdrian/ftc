package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoBlueMovement")
public class AutoBlueMovement extends LinearOpMode {

    //declarari
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo   servoDownLeft;
    private Servo   servoDownRight;


    @Override
    public void runOpMode() {


        //getters
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        servoDownRight = hardwareMap.servo.get("servoDownRight");
        servoDownLeft = hardwareMap.servo.get("servoDownLeft");

        waitForStart();

        //urm 2 servo-uri apuca cubul
        servoDownLeft.setPosition(0.85);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.12) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        servoDownRight.setPosition(0.15);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.12) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }
        setSpeed(0.25, 0.25, -0.25, -0.25);


        //merge in fata 2.17 sec
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2.17) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //stop
        setSpeed(0, 0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.05) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //merge in spate
        setSpeed(-0.25, -0.25, 0.25, 0.25);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //stop
        setSpeed(0, 0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.05) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //rotatie counter-clockwise
        setSpeed(0.25, 0.25, 0.25, 0.25);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.65) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //stop
        setSpeed(0, 0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.05) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //??
        setSpeed(-0.25, 0.25, 0.25, -0.25);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.15) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //stop
        setSpeed(0, 0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.12) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //da drumul la cub
        servoDownLeft.setPosition(0.1);
        servoDownRight.setPosition(0.9);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.12) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //??fata
        setSpeed(0.25, 0.25, -0.25, -0.25);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.4) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //????
        setSpeed(-0.25, 0.25, 0.25, -0.25);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.4) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }

        //??spate
        setSpeed(-0.25, -0.25, 0.25, 0.25);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.4) {
            telemetry.addData("intra", "iese");
            telemetry.update();
        }
    }








  /*  private void pushBila() {

        colorSensor.enableLed(true);
        if (colorSensor.red() > 0) {
            telemetry.addData("Am vazut rosu:", "RedSensor: " + colorSensor.red());
            while (runtime.seconds() < 2) {
                motorRightFront.setPower(-0.4);
                motorRightBack.setPower(0.4);
                motorLeftFront.setPower(-0.4);
                motorLeftBack.setPower(0.4);
            }
            servoSensor.setPosition(0.0);
            runtime.reset();
            while (runtime.seconds() < 5) {
                motorRightFront.setPower(0.4);
                motorRightBack.setPower(-0.4);
                motorLeftFront.setPower(0.4);
                motorLeftBack.setPower(-0.4);
            }
            runtime.reset();
        } else if (colorSensor.blue() > 0) {
            telemetry.addData("Am vazut albastru", "Blue sensor: " + colorSensor.blue());
            while (runtime.seconds() < 2) {
                motorRightFront.setPower(-0.4);
                motorRightBack.setPower(0.4);
                motorLeftFront.setPower(-0.4);
                motorLeftBack.setPower(0.4);
            }
            servoSensor.setPosition(0.0);
            runtime.reset();
            while (runtime.seconds() < 1) {
                motorRightFront.setPower(0.4);
                motorRightBack.setPower(-0.4);
                motorLeftFront.setPower(0.4);
                motorLeftBack.setPower(-0.4);
            }
            runtime.reset();
        }
        while (opModeIsActive())
            sleep(10);

    }*/

    private void setSpeed(double s, double s2, double s3, double s4) {
        telemetry.addData("RF: ", s);
        telemetry.addData("RB", s2);
        telemetry.addData("LF: ",s3);
        telemetry.addData("LB :", s4);
        telemetry.update();

        motorRightFront.setPower(s);
        motorRightBack.setPower(s2);
        motorLeftFront.setPower(s3);
        motorLeftBack.setPower(s4);
    }
}
