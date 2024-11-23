package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DriveMotorDirectionTest",  group = "MecanumDrive")
public class DriveMotorDirectionTest extends LinearOpMode {

    //Motors
    static DcMotor leftBack;
    static DcMotor rightBack;
    static DcMotor leftFront;
    static DcMotor rightFront;

    //Elapsed Timer
    ElapsedTime ET = new ElapsedTime();

    public void runOpMode() {

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.y) {
                leftBack.setPower(1);
            }
            else if (!gamepad2.y) {
                leftBack.setPower(0);
            }

            if (gamepad2.b) {
                leftFront.setPower(1);
            }
            else if (!gamepad2.b) {
                leftFront.setPower(0);
            }

            if (gamepad2.x) {
                rightBack.setPower(1);
            }
            else if (!gamepad2.x) {
                rightBack.setPower(0);
            }

            if (gamepad2.a) {
                rightFront.setPower(1);
            }
            else if (!gamepad2.a) {
                rightFront.setPower(0);
            }

            if (gamepad2.left_bumper) {
                leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
                rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if (gamepad2.right_bumper) {
                leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
                rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
                leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            telemetry.addData("leftFront direction:", leftFront.getDirection());
            telemetry.addData("leftBack direction:", leftBack.getDirection());
            telemetry.addData("rightFront direction:", rightFront.getDirection());
            telemetry.addData("rightBack direction:", rightBack.getDirection());
            telemetry.addData("leftFront power:", leftFront.getPower());
            telemetry.addData("leftBack power:", leftBack.getPower());
            telemetry.addData("rightFront power:", rightFront.getPower());
            telemetry.addData("rightBack power:", rightBack.getPower());

            telemetry.update();

        }

    }

}