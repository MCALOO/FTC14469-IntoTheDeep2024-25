package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "IntakeTest",  group = "Intake")
public class IntakeTest extends LinearOpMode {

    static NormalizedColorSensor Intake_Back;
    static NormalizedColorSensor Intake_Side;

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

    public void runOpMode() {

        Intake_Back = hardwareMap.get(NormalizedColorSensor.class, "IntakeColorBack");
        Intake_Side = hardwareMap.get(NormalizedColorSensor.class, "IntakeColorSide");

        IntakeBucket = hardwareMap.get(Servo.class,"IntakeBucket");
        IntakeWheels = hardwareMap.get(CRServo.class, "IntakeWheels");

        waitForStart();

        while (opModeIsActive()) {

            ColorDetector_IntakeBack();
            telemetry.addData("Is Blue in IntakeBack", BlueIntakeBack);
            telemetry.addData("Is Red in IntakeBack", RedIntakeBack);
            telemetry.addData("Is Yellow in IntakeBack", YellowIntakeBack);
            telemetry.addData("Is Unknown in IntakeBack", UnknownIntakeBack);

            telemetry.addLine();

            ColorDetector_IntakeSide();
            telemetry.addData("Is Blue in IntakeSide", BlueIntakeSide);
            telemetry.addData("Is Red in IntakeSide", RedIntakeSide);
            telemetry.addData("Is Yellow in IntakeSide", YellowIntakeSide);
            telemetry.addData("Is Unknown in IntakeSide", UnknownIntakeSide);

            telemetry.update();

        }

    }

    public void AttachmentPresets() {

        IntakeBucket.setDirection(Servo.Direction.FORWARD);
        setFolded();

        IntakeWheels.setDirection(CRServo.Direction.FORWARD);

    }

    private int ColorDetector_IntakeBack() {

        float[] HSV = new float[3];
        NormalizedRGBA RGBA = Intake_Back.getNormalizedColors();
        Intake_Back.setGain(30);

        Color.colorToHSV(RGBA.toColor(), HSV);
        telemetry.addData("H:", HSV[0]);
        telemetry.addData("S:", HSV[1]);
        telemetry.addData("V:", HSV[2]);

        int known = 1;
        int unkwown = 0;

        if (HSV[0] <= 225 && HSV[0] >= 180){
            telemetry.addData("IntakeBack Color:", "Blue");
            UnknownIntakeBack = false;
            BlueIntakeBack = true;
            RedIntakeBack = false;
            YellowIntakeBack = false;
            return known;
        } else if (HSV[0] <= 40 && HSV[0] >= 20) {
            telemetry.addData("IntakeBack Color:", "Red");
            UnknownIntakeBack = false;
            BlueIntakeBack = false;
            RedIntakeBack = true;
            YellowIntakeBack = false;
            return known;
        } else if (HSV[0] <= 70 && HSV[0] >= 50) {
            telemetry.addData("IntakeBack Color:", "Yellow");
            UnknownIntakeBack = false;
            BlueIntakeBack = false;
            RedIntakeBack = false;
            YellowIntakeBack = true;
            return known;
        } else {
            telemetry.addData("IntakeBack Color:", "Unknown");
            UnknownIntakeBack = true;
            BlueIntakeBack = false;
            RedIntakeBack = false;
            YellowIntakeBack = false;
            return unkwown;
        }
    }

    private int ColorDetector_IntakeSide() {

        float[] HSV = new float[3];
        NormalizedRGBA RGBA = Intake_Side.getNormalizedColors();
        Intake_Side.setGain(30);

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