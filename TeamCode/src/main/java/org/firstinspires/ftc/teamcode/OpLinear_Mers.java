package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by cristi on 09/03/2018.
 */



@TeleOp(name = "TeleOp Mers")
public class OpLinear_Mers extends LinearOpMode
{
    private DcMotor motorLeft1;
    private DcMotor motorLeft2;
    private DcMotor motorRight1;
    private DcMotor motorRight2;
    private DcMotor motorLift;
    //private DcMotor motorPra;
    private Servo pra;
    private Servo servor;
    private Servo servol;
    //private Servo brat;


    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        //motorPra = hardwareMap.dcMotor.get("motorPra");
        motorLift = hardwareMap.dcMotor.get("motorLift");
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");

        pra = hardwareMap.servo.get("pra");
        servor = hardwareMap.servo.get("servor");
        servol = hardwareMap.servo.get("servol");
        //brat = hardwareMap.servo.get("brat");


        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())
        {
            //brat.setPosition(0.5);

            if(gamepad1.a)
            {
                servor.setPosition(0);
                servol.setPosition(1);
            }

            if(gamepad1.dpad_down)
            {
                pra.setPosition(0.05);
            }

            if(gamepad1.dpad_up)
            {
                pra.setPosition(0.45);
            }
            /*
            if(gamepad1.dpad_right)
            {
                motorPra.setPower(0);
            }

            if(gamepad1.x)
            {
                motorPra.setPower(0.3);
            }

            if(gamepad1.y)
            {
                motorPra.setPower(-0.3);

            }
        */
            if(gamepad1.b)
            {
                 servor.setPosition(0.42);
                 servol.setPosition(0.64);
            }

            if(gamepad1.right_bumper)
            {
                motorLift.setPower(0.05);
            }

            if(gamepad1.dpad_left)
            {
                motorLift.setPower(-0.1);
            }

            if(gamepad1.left_bumper)
            {
                motorLift.setPower(-0.5);
            }

            motorLeft1.setPower(-gamepad1.left_stick_y);
            motorLeft2.setPower(-gamepad1.left_stick_y);
            motorRight1.setPower(-gamepad1.right_stick_y);
            motorRight2.setPower(-gamepad1.right_stick_y);



            idle();
        }
    }

}
