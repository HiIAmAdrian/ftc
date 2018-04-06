package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

@Autonomous(name="AutoRedMovement")
public class AutoRedMovement extends LinearOpMode {

    /* Declare OpMode members. */
    //HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private DcMotor         motorRightFront;
    private DcMotor         motorRightBack;
    private DcMotor         motorLeftFront;
    private DcMotor         motorLeftBack;
    private ColorSensor colorSensor;
    private Servo           servoDownRight;
    private Servo           servoSensor;
    private ElapsedTime     runtime = new ElapsedTime();

    /*private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    private static final double     DRIVE_GEAR_REDUCTION    = 1.5 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_CM   = 10.0 ;
    private static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    private static final double     DRIVE_SPEED             = 0.1;
    //private static final double     TURN_SPEED              = 0.5;
    private static final int     DRIVE_FRONT             = 1;
    private static final int     DRIVE_RIGHT             = 2;
    private static final int     DRIVE_BACK             = 3;
    private static final int     DRIVE_LEFT             = 4;*/

    @Override
    public void runOpMode() {

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //getters
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        servoSensor = hardwareMap.servo.get("servoSensor");
        servoDownRight = hardwareMap.servo.get("servoDownLeft");
        colorSensor = hardwareMap.colorSensor.get("colorSensors");


        //for initing encoders
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at RF:%7d :RB%7d LF:%7d LB:%7d",
                motorRightFront.getCurrentPosition(),
                motorRightBack.getCurrentPosition(),
                motorLeftFront.getCurrentPosition(),
                motorLeftBack.getCurrentPosition());
        telemetry.update();

        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a NEGATIVE DISTANCE (NOT SPEED)

        /*servoDownLeft.setPosition(0.38);
        servoDownRight.setPosition(0.62);*/

        pushBila();
        runtime.reset();

        //encoderDrive(DRIVE_SPEED,  400,  400, 10.0, DRIVE_FRONT);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    private void pushBila()
    {

        servoSensor.setPosition(0.80);
        sleep(200);
        colorSensor.enableLed(true);

           /* while (colorSensor.blue() < 20 && colorSensor.red() < 20) {
                telemetry.addData("mergem la bile", "BlueSensor: " + colorSensor.blue() + " RedSensor: " + colorSensor.red());
                motorRightFront.setPower(0.4);
                motorRightBack.setPower(0.4);
                motorLeftFront.setPower(-0.4);
                motorLeftBack.setPower(-0.4);
            }*/
        if (colorSensor.red() > 0) {
            telemetry.addData("Am vazut rosu:", "RedSensor: " + colorSensor.red());
            while (runtime.seconds() < 0.5) {
                motorRightFront.setPower(0.4);
                motorRightBack.setPower(0.4);
                motorLeftFront.setPower(0.4);
                motorLeftBack.setPower(0.4);
            }
            servoSensor.setPosition(0.0);
            runtime.reset();
            while (runtime.seconds() < 0.5)
            {
                motorRightFront.setPower(-0.4);
                motorRightBack.setPower(-0.4);
                motorLeftFront.setPower(-0.4);
                motorLeftBack.setPower(-0.4);
            }
            runtime.reset();
            while (runtime.seconds() < 5)
            {
                motorRightFront.setPower(-0.4);
                motorRightBack.setPower(0.4);
                motorLeftFront.setPower(-0.4);
                motorLeftBack.setPower(0.4);
            }
            runtime.reset();
        }
        else if (colorSensor.blue() > 0) {
            telemetry.addData("Am vazut albastru", "Blue sensor: " + colorSensor.blue());
            while (runtime.seconds() < 0.5) {
                motorRightFront.setPower(-0.4);
                motorRightBack.setPower(-0.4);
                motorLeftFront.setPower(-0.4);
                motorLeftBack.setPower(-0.4);
            }
            servoSensor.setPosition(0.0);
            runtime.reset();
            while (runtime.seconds() < 0.5)
            {
                motorRightFront.setPower(0.4);
                motorRightBack.setPower(0.4);
                motorLeftFront.setPower(0.4);
                motorLeftBack.setPower(0.4);
            }
            runtime.reset();
            while (runtime.seconds() < 2)
            {
                motorRightFront.setPower(-0.4);
                motorRightBack.setPower(0.4);
                motorLeftFront.setPower(-0.4);
                motorLeftBack.setPower(0.4);
            }
            runtime.reset();
        }
        while(opModeIsActive())
            sleep(10);

    }
    /*private void encoderDrive(double speed, double leftCm, double rightCm, double timeoutS, int direction) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = motorLeftFront.getCurrentPosition() + (int)(leftCm * COUNTS_PER_CM);
            newRightTarget = motorRightFront.getCurrentPosition() + (int)(rightCm * COUNTS_PER_CM);
            telemetry.addData("left: ",newLeftTarget);
            telemetry.addData("right: ", newRightTarget);

            //newLeftTarget = motorLeftFront.getCurrentPosition() + (int)(getDistance(leftCm) * COUNTS_PER_CM);
            //newRightTarget = motorRightFront.getCurrentPosition() + (int)(getDistance(rightCm) * COUNTS_PER_CM);
            if (direction == DRIVE_FRONT) {
                motorLeftFront.setTargetPosition(-newLeftTarget);
                motorLeftBack.setTargetPosition(-newLeftTarget);
                motorRightFront.setTargetPosition(newRightTarget);
                motorRightBack.setTargetPosition(newRightTarget);
            }
            else if (direction == DRIVE_RIGHT){
                motorLeftFront.setTargetPosition(-newLeftTarget);
                motorLeftBack.setTargetPosition(newLeftTarget);
                motorRightFront.setTargetPosition(-newRightTarget);
                motorRightBack.setTargetPosition(newRightTarget);
            }
            else if (direction == DRIVE_BACK){
                motorLeftFront.setTargetPosition(newLeftTarget);
                motorLeftBack.setTargetPosition(newLeftTarget);
                motorRightFront.setTargetPosition(-newRightTarget);
                motorRightBack.setTargetPosition(-newRightTarget);
            }
            else if (direction == DRIVE_LEFT){
                motorLeftFront.setTargetPosition(newLeftTarget);
                motorLeftBack.setTargetPosition(-newLeftTarget);
                motorRightFront.setTargetPosition(newRightTarget);
                motorRightBack.setTargetPosition(-newRightTarget);
            }

            // Turn On RUN_TO_POSITION
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorRightFront.setPower(Math.abs(speed));
            motorRightBack.setPower(Math.abs(speed));
            motorLeftFront.setPower(Math.abs(speed));
            motorLeftBack.setPower(Math.abs(speed));
            telemetry.addData("Speed: ", speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorRightFront.isBusy() && motorRightBack.isBusy() && motorLeftFront.isBusy() && motorLeftBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("PathTo",  "Running to Left:%7d Right:%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Curenttttt:",  "Running at RF:%7d RB:%7d LF:%7d LB:%7d",
                        motorRightFront.getCurrentPosition(),
                        motorRightBack.getCurrentPosition(),
                        motorLeftFront.getCurrentPosition(),
                        motorLeftBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);

            // Turn off RUN_TO_POSITION
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    private double getDistance(double distance)
    {
        return (distance / (Math.cos(Math.toRadians(41))));
    }*/
}