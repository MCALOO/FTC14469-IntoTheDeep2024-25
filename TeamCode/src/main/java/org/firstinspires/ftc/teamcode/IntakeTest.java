package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@TeleOp(name = "IntakeTest",  group = "Intake")
public class IntakeTest extends LinearOpMode {

//    static NormalizedColorSensor Intake_Back;
    static NormalizedColorSensor CI;
    static DistanceSensor DI;

    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;

    static Servo IntakeBucket; // Servo Mode
    static CRServo IntakeWheels; // Continuous Rotation Mode

    boolean UnknownIntakeBack;
    boolean BlueIntakeBack;
    boolean RedIntakeBack;
    boolean YellowIntakeBack;
    boolean UnknownIntakeSide;
    boolean BlueIntakeSide;
    boolean RedIntakeSide;
    boolean YellowIntakeSide;

    //Button Pressing Variables
    boolean button_a_already_pressed = false;
    boolean button_b_already_pressed = false;
    boolean button_x_already_pressed = false;
    boolean button_y_already_pressed = false;
    boolean button_bumper_left_already_pressed = false;
    boolean button_bumper_right_already_pressed = false;
    boolean button_dpad_right_already_pressed = false;
    boolean button_dpad_left_already_pressed = false;
    boolean button_dpad_up_already_pressed = false;
    boolean button_dpad_down_already_pressed = false;
    boolean button_left_trigger_already_pressed = false;
    boolean button_right_trigger_already_pressed = false;
    boolean button_back_already_pressed = false;
    boolean button_start_already_pressed = false;
    boolean double_trigger_already_pressed = false;
    boolean button_guide_already_pressed = false;

    boolean lowPowerSetting; // toggle low power setting
    boolean highPowerSetting; // toggle high power setting
    double movement;
    boolean lowMovement = false;
    double l;

    public void runOpMode() {

//        Intake_Back = hardwareMap.get(NormalizedColorSensor.class, "IntakeColorBack");
        CI = hardwareMap.get(NormalizedColorSensor.class, "CI");
        DI = hardwareMap.get(DistanceSensor.class,"DI");

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeBucket = hardwareMap.get(Servo.class,"IntakeBucket");
        IntakeWheels = hardwareMap.get(CRServo.class, "IntakeWheels");

        AttachmentPresets();

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

            /*****************************************************************
             * Button Guide (G2) :
             *****************************************************************/

            if (!button_guide_already_pressed) {
                if (gamepad1.guide) {
                    wheelOff();
                    button_guide_already_pressed = true;
                }
            } else {
                if (!gamepad1.guide) {
                    button_guide_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button Back (G2) :
             *****************************************************************/


            if (!button_back_already_pressed) {
                if (gamepad1.back) {
                    wheelReverse();
                    button_back_already_pressed = true;
                }
            } else {
                if (!gamepad1.back) {
                    button_back_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button Start (G2) :
             *****************************************************************/

            if (!button_start_already_pressed) {
                if (gamepad1.start) {
                    wheelOn();
                    button_start_already_pressed = true;
                }
            } else {
                if (!gamepad1.start) {
                    button_start_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button Y (G2) :
             **************************************************************z***/

            if (!button_y_already_pressed) {
                if (gamepad1.y) {

                    button_y_already_pressed = true;
                }
            } else {
                if (!gamepad1.y) {
                    button_y_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button B (G2) :
             *****************************************************************/

            if (!button_b_already_pressed) {
                if (gamepad1.b) {
                    setEntry();
                    button_b_already_pressed = true;
                }
            } else {
                if (!gamepad1.b) {
                    button_b_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button A (G2) :
             *****************************************************************/

            if (!button_a_already_pressed) {
                if (gamepad1.a) {
                    setIntake();
                    button_a_already_pressed = true;
                }
            } else {
                if (!gamepad1.a) {
                    button_a_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button X (G2) :
             *****************************************************************/

            if (!button_x_already_pressed) {
                if (gamepad1.x) {
                    setFolded();
                    button_x_already_pressed = true;
                }
            } else {
                if (!gamepad1.x) {
                    button_x_already_pressed = false;
                }
            }

//            ColorDetector_IntakeBack();
//            telemetry.addData("Is Blue in IntakeBack", BlueIntakeBack);
//            telemetry.addData("Is Red in IntakeBack", RedIntakeBack);
//            telemetry.addData("Is Yellow in IntakeBack", YellowIntakeBack);
//            telemetry.addData("Is Unknown in IntakeBack", UnknownIntakeBack);

//            ColorDetector_IntakeSide();
//            telemetry.addData("Is Blue in IntakeSide", BlueIntakeSide);
//            telemetry.addData("Is Red in IntakeSide", RedIntakeSide);
//            telemetry.addData("Is Yellow in IntakeSide", YellowIntakeSide);
//            telemetry.addData("Is Unknown in IntakeSide", UnknownIntakeSide);

            Color_Distance_Detector();

            telemetry.addLine();

            telemetry.addData("Distance Sensor Value", DI.getDistance(DistanceUnit.CM));

            telemetry.update();

        }

    }

    public void AttachmentPresets() {

        IntakeBucket.setDirection(Servo.Direction.FORWARD);
        setFolded();

        IntakeWheels.setDirection(CRServo.Direction.FORWARD);

    }

//    private int ColorDetector_IntakeBack() {
//
//        float[] HSV = new float[3];
//        NormalizedRGBA RGBA = Intake_Back.getNormalizedColors();
//        Intake_Back.setGain(30);
//
//        Color.colorToHSV(RGBA.toColor(), HSV);
//        telemetry.addData("H:", HSV[0]);
//        telemetry.addData("S:", HSV[1]);
//        telemetry.addData("V:", HSV[2]);
//
//        int known = 1;
//        int unkwown = 0;
//
//        if (HSV[0] <= 225 && HSV[0] >= 180){
//            telemetry.addData("IntakeBack Color:", "Blue");
//            UnknownIntakeBack = false;
//            BlueIntakeBack = true;
//            RedIntakeBack = false;
//            YellowIntakeBack = false;
//            return known;
//        } else if (HSV[0] <= 40 && HSV[0] >= 20) {
//            telemetry.addData("IntakeBack Color:", "Red");
//            UnknownIntakeBack = false;
//            BlueIntakeBack = false;
//            RedIntakeBack = true;
//            YellowIntakeBack = false;
//            return known;
//        } else if (HSV[0] <= 70 && HSV[0] >= 50) {
//            telemetry.addData("IntakeBack Color:", "Yellow");
//            UnknownIntakeBack = false;
//            BlueIntakeBack = false;
//            RedIntakeBack = false;
//            YellowIntakeBack = true;
//            return known;
//        } else {
//            telemetry.addData("IntakeBack Color:", "Unknown");
//            UnknownIntakeBack = true;
//            BlueIntakeBack = false;
//            RedIntakeBack = false;
//            YellowIntakeBack = false;
//            return unkwown;
//        }
//    }

    private int ColorDetector_IntakeSide() {

        float[] HSV = new float[3];
        NormalizedRGBA RGBA = CI.getNormalizedColors();
        CI.setGain(30);

        Color.colorToHSV(RGBA.toColor(), HSV);
        telemetry.addData("H:", HSV[0]);
        telemetry.addData("S:", HSV[1]);
        telemetry.addData("V:", HSV[2]);

        int known = 1;
        int unkwown = 0;

        if (HSV[0] <= 225 && HSV[0] >= 175){
            telemetry.addData("IntakeSide Color:", "Blue");
            UnknownIntakeSide = false;
            BlueIntakeSide = true;
            RedIntakeSide = false;
            YellowIntakeSide = false;
            return known;
        } else if (HSV[0] <= 40 && HSV[0] >= 20) {
            telemetry.addData("IntakeSide Color:", "Red");
            UnknownIntakeSide = false;
            BlueIntakeSide = false;
            RedIntakeSide = true;
            YellowIntakeSide = false;
            return known;
        } else if (HSV[0] <= 70 && HSV[0] >= 50) {
            telemetry.addData("IntakeSide Color:", "Yellow");
            UnknownIntakeSide = false;
            BlueIntakeSide = false;
            RedIntakeSide = false;
            YellowIntakeSide = true;
            return known;
        } else {
            telemetry.addData("IntakeSide Color:", "Unknown");
            UnknownIntakeSide = true;
            BlueIntakeSide = false;
            RedIntakeSide = false;
            YellowIntakeSide = false;
            return unkwown;
        }
    }

    private void Color_Distance_Detector() {

        if (DI.getDistance(DistanceUnit.CM) <= 4){

            float[] HSV = new float[3];
            NormalizedRGBA RGBA = CI.getNormalizedColors();
            CI.setGain(30);

            Color.colorToHSV(RGBA.toColor(), HSV);
            telemetry.addData("H:", HSV[0]);
            telemetry.addData("S:", HSV[1]);
            telemetry.addData("V:", HSV[2]);
            telemetry.addData("Sample Position:","Vertical");

            if (HSV[0] <= 225 && HSV[0] >= 175){
                telemetry.addData("Color:", "Blue");
                UnknownIntakeSide = false;
                BlueIntakeSide = true;
                RedIntakeSide = false;
                YellowIntakeSide = false;
            } else if (HSV[0] <= 65 && HSV[0] >= 15) {
                telemetry.addData("Color:", "Red");
                UnknownIntakeSide = false;
                BlueIntakeSide = false;
                RedIntakeSide = true;
                YellowIntakeSide = false;
            } else if (HSV[0] <= 100 && HSV[0] >= 70) {
                telemetry.addData("Color:", "Yellow");
                UnknownIntakeSide = false;
                BlueIntakeSide = false;
                RedIntakeSide = false;
                YellowIntakeSide = true;
            } else {
                telemetry.addData("Color:", "Unknown");
                UnknownIntakeSide = true;
                BlueIntakeSide = false;
                RedIntakeSide = false;
                YellowIntakeSide = false;
            }

        } else if (DI.getDistance(DistanceUnit.CM) <= 9 && DI.getDistance(DistanceUnit.CM) >= 5) {

            float[] HSV = new float[3];
            NormalizedRGBA RGBA = CI.getNormalizedColors();
            CI.setGain(30);

            Color.colorToHSV(RGBA.toColor(), HSV);
            telemetry.addData("H:", HSV[0]);
            telemetry.addData("S:", HSV[1]);
            telemetry.addData("V:", HSV[2]);
            telemetry.addData("Sample Position:","Horizontal");

            if (HSV[0] <= 225 && HSV[0] >= 180){
                telemetry.addData("Color:", "Blue");
                UnknownIntakeSide = false;
                BlueIntakeSide = true;
                RedIntakeSide = false;
                YellowIntakeSide = false;
            } else if (HSV[0] <= 69 && HSV[0] >= 40) {
                telemetry.addData("Color:", "Red");
                UnknownIntakeSide = false;
                BlueIntakeSide = false;
                RedIntakeSide = true;
                YellowIntakeSide = false;
            } else if (HSV[0] <= 100 && HSV[0] >= 71) {
                telemetry.addData("Color:", "Yellow");
                UnknownIntakeSide = false;
                BlueIntakeSide = false;
                RedIntakeSide = false;
                YellowIntakeSide = true;
            } else {
                telemetry.addData("Color:", "Unknown");
                UnknownIntakeSide = true;
                BlueIntakeSide = false;
                RedIntakeSide = false;
                YellowIntakeSide = false;
            }

        } else {

            telemetry.addData("Sample Position:","None Found");

        }

    }

    public void setIntake() {
        IntakeBucket.setPosition(0.35);
    }

    public void setEntry() {
        IntakeBucket.setPosition(0.6);
    }

    public void setFolded() {
        IntakeBucket.setPosition(0.87);
    }

    public void wheelOn() {
        IntakeWheels.setPower(1);
    }

    public  void wheelOff() {
        IntakeWheels.setPower(0);
    }

    public void wheelReverse() {
        IntakeWheels.setPower(-1);
    }

}