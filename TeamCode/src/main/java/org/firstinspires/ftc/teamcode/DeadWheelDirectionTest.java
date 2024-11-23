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

@TeleOp(name = "DeadWheelDirectionTest",  group = "MecanumDrive")
public class DeadWheelDirectionTest extends LinearOpMode {

    //Motors
    static DcMotor leftBack;
    static DcMotor rightBack;
    static DcMotor leftFront;
    static DcMotor rightFront;

    static Encoder perp;
    static Encoder par0;
    static Encoder par1;

    //Elapsed Timer
    ElapsedTime ET = new ElapsedTime();

    public void runOpMode() {

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront")));

        perp.setDirection(DcMotorSimple.Direction.FORWARD);
        par0.setDirection(DcMotorSimple.Direction.REVERSE);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("perp encoder:", leftFront.getCurrentPosition());
            telemetry.addData("par0 encoder:", leftBack.getCurrentPosition());
            telemetry.addData("par1 encoder:", rightFront.getCurrentPosition());

            telemetry.addData("perp", perp.getDirection());
            telemetry.addData("par0", par0.getDirection());
            telemetry.addData("par1", par1.getDirection());

            telemetry.update();

        }

    }

}