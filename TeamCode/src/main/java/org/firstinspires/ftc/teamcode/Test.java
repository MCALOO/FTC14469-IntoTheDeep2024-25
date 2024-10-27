package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@TeleOp(name = "Test",  group = "MecanumDrive")
public class Test extends LinearOpMode {

    //Motors
    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor Slide;
    static DcMotor Base;

    //Servos
    static Servo rightClaw; // Servo Mode
    static Servo leftClaw; // Servo Mode
    static Servo clawPivot; // Servo Mode

    //Vars for White color detection
    boolean WhiteJerry;
    boolean WhiteBackLeft;
    boolean WhiteBackRight;
    boolean UnknownJerry;
    boolean UnknownBackLeft;
    boolean UnknownBackRight;

    //Vars for Touch Sensor
    boolean leftTouchIsPressed = false;
    boolean rightTouchIsPressed = false;
    double leftTouchValue;
    double rightTouchValue;

    //IMU
    BNO055IMU IMU;
    //Variables For IMU Gyro
    double globalangle;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;

    //Button Pressing Variables
    boolean button_a_already_pressed = false;
    boolean button_b_already_pressed = false;
    boolean button_x_already_pressed = false;
    boolean button_y_already_pressed = false;
    boolean button_a_already_pressed2 = false;
    boolean button_b_already_pressed2 = false;
    boolean button_x_already_pressed2 = false;
    boolean button_y_already_pressed2 = false;
    boolean button_bumper_left_already_pressed = false;
    boolean button_bumper_right_already_pressed = false;
    boolean button_bumper_left_already_pressed2 = false;
    boolean button_bumper_right_already_pressed2 = false;
    boolean button_dpad_right_already_pressed = false;
    boolean button_dpad_left_already_pressed = false;
    boolean button_dpad_up_already_pressed = false;
    boolean button_dpad_down_already_pressed = false;
    boolean button_dpad_right_already_pressed2 = false;
    boolean button_dpad_left_already_pressed2 = false;
    boolean button_dpad_up_already_pressed2 = false;
    boolean button_dpad_down_already_pressed2 = false;
    boolean button_left_trigger_already_pressed = false;
    boolean button_right_trigger_already_pressed = false;
    boolean button_left_trigger_already_pressed2 = false;
    boolean button_right_trigger_already_pressed2 = false;
    boolean button_back_already_pressed = false;
    boolean button_back_already_pressed2 = false;
    boolean button_start_already_pressed = false;
    boolean button_start_already_pressed2 = false;
    boolean double_trigger_already_pressed = false;
    boolean button_guide_already_pressed = false;
    boolean button_guide_already_pressed2 = false;

    //movement
    boolean lowPowerSetting; // toggle low power setting
    boolean highPowerSetting; // toggle high power setting
    double movement;

    //direction control
    double l;

    //toggle vars
    boolean clawSetting;


    //separate boolean vars
    boolean stopControls = false; //stop all movement controls
    boolean lowMovement = false; //set movement to 20% right before reaching the backdrop

    //Elapsed Timer
    ElapsedTime ET = new ElapsedTime();

    public void runOpMode() {

        //Motor Initalization
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Base = hardwareMap.get(DcMotor.class, "Base");

        //Servo Initalization
        rightClaw = hardwareMap.get(Servo.class,"rightClaw");
        leftClaw = hardwareMap.get(Servo.class,"leftClaw");
        clawPivot = hardwareMap.get(Servo.class, "clawPivot");

        //Set Drive Motor Directions
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set Servo and Motor Presets
        AttachmentPresets();


        //imu initialization
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        //Configrue IMU for GyroTurning
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        globalangle = 0;

        //when you let go of movement control, robot will stop, not drift
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            /*****************************************************************
             * Bumper Right/Left (G1) - Set Low/High Power Mode for driving
             *****************************************************************/

            if (gamepad1.right_bumper) {
                lowPowerSetting = true;
            } else {
                lowPowerSetting = false;
            }

            if (gamepad1.left_bumper) {
                highPowerSetting = true;
            } else {
                highPowerSetting = false;
            }

            if (lowPowerSetting || highPowerSetting) {
                if (lowPowerSetting) {
                    movement = 0.4;
                }
                if (highPowerSetting) {
                    movement = 1;
                }
            } else {
                if (lowMovement) {
                    movement = 0.2;
                } else {
                    movement = 0.85;
                }
            }

            /******************************************************************
             * Left/Right Analog Sticks (G1) - Movement
             *****************************************************************/

            if (!stopControls) {
                double y = -gamepad1.left_stick_y * movement;
                double x = gamepad1.left_stick_x * movement;
                double rx = gamepad1.right_stick_x * movement;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                double FLPower = (y + x + rx + l) / denominator;
                double BLPower = (y - x + rx + l) / denominator;
                double FRPower = (y - x - rx - l) / denominator;
                double BRPower = (y + x - rx - l) / denominator;

                FrontLeft.setPower(FLPower);
                BackLeft.setPower(BLPower);
                FrontRight.setPower(FRPower);
                BackRight.setPower(BRPower);

            }

            /*****************************************************************
             * Button Guide (G2) :
             *****************************************************************/

            if (!button_guide_already_pressed2) {
                if (gamepad2.guide) {
                    if (!clawSetting) {
                        clawOpen();
                        clawSetting = true;
                    } else {
                        clawClose();
                        clawSetting = false;
                    }
                    button_guide_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.guide) {
                    button_guide_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button Back (G2) :
             *****************************************************************/

            if (!button_back_already_pressed2) {
                if (gamepad2.back) {
                    setSlidePosition(0,1);
                    setBasePosition(3200,1);
                    button_back_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.back) {
                    button_back_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button Start (G2) :
             *****************************************************************/
            if (!button_start_already_pressed2) {
                if (gamepad2.start) {
                    setSlidePosition(1500,1);
                    setBasePosition(3500,1);
                    clawOpen();
                    button_start_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.start) {
                    button_start_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button Y (G2) : Set Slide for High Bucket
             **************************************************************z***/

            if (!button_y_already_pressed2) {
                if (gamepad2.y) {
                    setSlidePosition(4100,1);
                    setBasePosition(700,1);
                    clawPivot.setPosition(0.3);
                    button_y_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.y) {
                    button_y_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button B (G2) :
             *****************************************************************/

            if (!button_b_already_pressed2) {
                if (gamepad2.b) {
                    setSlidePosition(2400,1);
                    setBasePosition(0,1);
                    clawPivot.setPosition(0.35);
                    button_b_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.b) {
                    button_b_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button A (G2) :
             *****************************************************************/

            if (!button_a_already_pressed2) {
                if (gamepad2.a) {
                    setBasePosition(3200,1);
                    clawPivot.setPosition(0.5);
                    button_a_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.a) {
                    button_a_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button X (G2) : Reset All Attachments (Slides, Base, Claw)
             *****************************************************************/

            if (!button_x_already_pressed2) {
                if (gamepad2.x) {
                    setSlidePosition(0,0.7);
                    setBasePosition(0,1);
                    clawClose();
                    clawPivot.setPosition(1);
                    button_x_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Dpad Up (G2) :
             *****************************************************************/

            if (!button_dpad_up_already_pressed2) {
                if (gamepad2.dpad_up) {

                    button_dpad_up_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_up) {
                    button_dpad_up_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Dpad Down (G2) :
             *****************************************************************/

            if (!button_dpad_down_already_pressed2) {
                if (gamepad2.dpad_down) {

                    button_dpad_down_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_down) {
                    button_dpad_down_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Dpad Right (G2) :
             *****************************************************************/

            if (!button_dpad_right_already_pressed2) {
                if (gamepad2.dpad_right) {

                    button_dpad_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_right) {
                    button_dpad_right_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Dpad Left (G2) :
             *****************************************************************/

            if (!button_dpad_left_already_pressed2) {
                if (gamepad2.dpad_left) {

                    button_dpad_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_left) {
                    button_dpad_left_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Right Bumper (G2) :
             *****************************************************************/

            if (!button_bumper_right_already_pressed2) {
                if (gamepad2.right_bumper) {

                    button_bumper_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Left Bumper (G2) :
             *****************************************************************/

            if (!button_bumper_left_already_pressed2) {
                if (gamepad2.left_bumper) {

                    button_bumper_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                }
            }

            telemetry.addData("Base Encoder:", Base.getCurrentPosition());
            telemetry.addData("Slide Encoder:", Base.getCurrentPosition());
            telemetry.update();

        }
    }

    public void AttachmentPresets() {
        //Intake & Lin Actuator Presets
        Slide.setDirection(DcMotorSimple.Direction.FORWARD);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setTargetPosition(0);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Base.setDirection(DcMotorSimple.Direction.REVERSE);
        Base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Base.setTargetPosition(0);
        Base.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Servo Presets
        rightClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setPosition(0.5);
        leftClaw.setDirection(Servo.Direction.REVERSE);
        leftClaw.setPosition(0.5);
        leftClaw.setDirection(Servo.Direction.FORWARD);
        clawPivot.setPosition(1);
    }

    public void clawOpen() {

        rightClaw.setPosition(-1);
        leftClaw.setPosition(1);

    }

    public void clawClose() {

        rightClaw.setPosition(0.55);
        leftClaw.setPosition(0.45);

    }

    public void setBasePosition (int position, double power){

        Base.setTargetPosition(position);
        Base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Base.setPower(power);

    }

    public void setSlidePosition(int position, double power) {

        Slide.setTargetPosition(position);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(power);

    }

}