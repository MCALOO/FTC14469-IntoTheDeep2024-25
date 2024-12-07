package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "DeadWheelTest",  group = "MecanumDrive")
public class MechDriveDeadWheel_Test extends LinearOpMode {

    //Motors
    static DcMotor leftBack;
    static DcMotor rightBack;
    static DcMotor leftFront;
    static DcMotor rightFront;

    static Encoder perp;
    static Encoder par0;
    static Encoder par1;

    Mech_Drive_DEADWHEEL DeadWheel;
    Mech_Drive_FAST MechDrive;
//    Direction_Control DirectionControl;

    //IMU
    BNO055IMU IMU;

    //IMU Orientation
    byte AXIS_MAP_CONFIG_BYTE = 0x24;
    byte AXIS_MAP_SIGN_BYTE = 0x06;

    //Variables For IMU Gyro
    double globalangle;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;
    //Gyrocontinuity Variables
    double current_value;
    double prev_value = 0;
    double final_value;

    int programorder = 0;

    //Elapsed Timer
    ElapsedTime ET = new ElapsedTime();

    public void runOpMode() {

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
//        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack")));
//        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront")));

//        perp.setDirection(DcMotorSimple.Direction.FORWARD);
//        par0.setDirection(DcMotorSimple.Direction.REVERSE);
//        par1.setDirection(DcMotorSimple.Direction.REVERSE);

        MechDrive = new Mech_Drive_FAST(rightFront, leftFront, rightBack, leftBack, MoveDirection.FORWARD, telemetry);

//        Direction = new Direction_Control(IMU, leftFront, rightFront, leftBack, rightBack);

//        DeadWheel = new Mech_Drive_DEADWHEEL(rightFront, leftFront, rightBack, leftBack, rightFront, leftFront,
//                MoveDirection.FORWARD, MoveDirection.FORWARD, MoveDirection.FORWARD, telemetry);

        //imu initialization
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        //Configure IMU for GyroTurning
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        globalangle = 0;

        //Configure the control hub orientation
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100);

        waitForStart();

        while (opModeIsActive()) {

            switch (programorder) {

                case 0:
//                    DeadWheel.SetTargets(0,0,10000,0.5);
                    MechDrive.SetTargets(0,90,15000,0.5,0);
//                    rightFront.setPower(0.3);
//                    rightBack.setPower(0.3);
//                    leftFront.setPower(0.3);
//                    leftBack.setPower(0.3);
                    programorder++;
                    break;

                case 1:
                    if (MechDrive.GetTaskState() == Task_State.DONE || MechDrive.GetTaskState() == Task_State.READY) {
//                        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        ET.reset();
                         programorder++;
                    }
                    break;

                case 3:
                    if (ET.milliseconds() > 3000) {
                        MechDrive.SetTargets(0,-90,15000,0.5,0);
                        programorder++;
                    }
                    break;

                default:
                    break;
            }

            MechDrive.Task(GyroContinuity());
//            DeadWheel.Task(GyroContinuity());
//            DirectionControl.GyroContinuity();

            telemetry.addData("perp encoder:", leftFront.getCurrentPosition());
            telemetry.addData("par0 encoder:", leftBack.getCurrentPosition());
            telemetry.addData("par1 encoder:", rightFront.getCurrentPosition());
            telemetry.addData("Angle: ", GyroContinuity());

            telemetry.addLine();

            telemetry.addData("leftFront Mode (perp):", leftFront.getMode());
            telemetry.addData("leftBack Mode:", leftBack.getMode());
            telemetry.addData("rightFront Mode (par):", rightFront.getMode());
            telemetry.addData("rightBack Mode:", rightBack.getMode());

            telemetry.addData("leftFront Mode (perp):", leftFront.getPower());
            telemetry.addData("leftBack Mode:", leftBack.getPower());
            telemetry.addData("rightFront Mode (par):", rightFront.getPower());
            telemetry.addData("rightBack Mode:", rightBack.getPower());

            telemetry.addLine();

//            telemetry.addData("perp", perp.getDirection());
//            telemetry.addData("par0", par0.getDirection());
//            telemetry.addData("par1", par1.getDirection());

//            telemetry.update();

        }

    }

    public void resetDriveEncoders() {
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private double GyroContinuity() {

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        current_value = orientation.firstAngle;

        final_value = current_value - prev_value;

        if (final_value < -180)
            final_value += 360;
        else if (final_value > 180)
            final_value -= 360;

        globalangle += final_value;

        prev_value = current_value;

        return -globalangle;
    }

}