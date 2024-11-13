package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "RedObservation",  group = "MecanumDrive")
public class RedObservation extends LinearOpMode {

    //Motors
    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor Intake_Rail;
    static DcMotor Outtake_Rail;
    static DcMotor Swivel;

    Rail_ControlV3 RailControl_Intake;
    Rail_ControlV3 RailControl_Outtake;
    Mech_Drive_FAST MechDrive;
    Direction_Control DirectionControl;

    //Servos
    static Servo OuttakeBucket; // Servo Mode
    static Servo IntakeBucket; // Servo Mode
    static CRServo IntakeWheels; // Continuous Rotation Mode
    static Servo OuttakeWrist; // Servo Mode
    static CRServo Claw; // Continuous Rotation Mode

    //Sensors
    static NormalizedColorSensor CI;
    static DistanceSensor DI;
    static DistanceSensor DO;

    //IMU
    BNO055IMU IMU;

    //IMU Orientation
    byte AXIS_MAP_CONFIG_BYTE = 0x18; //rotates control hub 90 degrees around y axis by swapping x and z axis
    byte AXIS_MAP_SIGN_BYTE = 0x02; //Negates the remapped z-axis

    //Variables For IMU Gyro
    double globalangle;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;
    //Gyrocontinuity Variables
    double current_value;
    double prev_value = 0;
    double final_value;

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

    //booleans for intake color detection
    boolean UnknownIntakeSide;
    boolean BlueIntakeSide;
    boolean RedIntakeSide;
    boolean YellowIntakeSide;

    //movement
    boolean lowPowerSetting; // toggle low power setting
    boolean highPowerSetting; // toggle high power setting
    double movement;

    //direction control
    double l;

    //separate boolean vars
    boolean lowMovement = false; //set movement to 20% right before reaching the backdrop
    boolean blueTeam = false;
    boolean redTeam = false;

    //sequence managers
    int programOrder = 0;
    int intakeSequence = 1000;
    int outtakeBucketSequence = 1000;
    int outtakeTargetPosition;
    int resetAttachments = 1000;
    int intakeSpecimen = 1000;
    int setObservation = 1000;
    int outtakeObservation = 1000;

    //Elapsed Timer
    ElapsedTime ET = new ElapsedTime();
    ElapsedTime ET_Delay = new ElapsedTime();

    public void runOpMode() {

        //Motor Initialization
        BackLeft = hardwareMap.get(DcMotor.class, "leftBack");
        BackRight = hardwareMap.get(DcMotor.class, "rightBack");
        FrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        FrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        Intake_Rail = hardwareMap.get(DcMotor.class, "IntakeRail");
        Outtake_Rail = hardwareMap.get(DcMotor.class, "OuttakeRail");
        Swivel = hardwareMap.get(DcMotor.class,"Swivel");

        //Servo Initialization
        IntakeBucket = hardwareMap.get(Servo.class,"IntakeBucket");
        OuttakeBucket = hardwareMap.get(Servo.class,"OuttakeBucket");
        IntakeWheels = hardwareMap.get(CRServo.class, "IntakeWheels");
        OuttakeWrist = hardwareMap.get(Servo.class,"OuttakeWrist");
        Claw = hardwareMap.get(CRServo.class,"Claw");

        //Sensor Initialization
        CI = hardwareMap.get(NormalizedColorSensor.class,"CI");
        DI = hardwareMap.get(DistanceSensor.class,"DI");
        DO = hardwareMap.get(DistanceSensor.class,"DO");

        //imu initialization
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        //Set Drive Motor Directions
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set Servo and Motor Presets
        AttachmentPresets();

        //MechDrive Object
        MechDrive = new Mech_Drive_FAST(FrontRight, FrontLeft, BackRight, BackLeft, MoveDirection.FORWARD, telemetry);

        //direction control object
        DirectionControl = new Direction_Control(IMU, FrontLeft, FrontRight, BackLeft, BackRight);

        //rail control objects
        RailControl_Intake = new Rail_ControlV3(Intake_Rail);
        RailControl_Outtake = new Rail_ControlV3(Outtake_Rail);

        //when you let go of movement control, robot will stop, not drift
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        ET.reset();
        ET_Delay.reset();

        waitForStart();

        while (opModeIsActive()) {

            switch (programOrder) {

                case 0:
                    MechDrive.SetTargets(0,90,1100,0.5,1);
                    RailControl_Outtake.SetTargetPosition(600,-0.7,0.7);
                    setSwivelPosition(0);
                    programOrder++;
                    break;

                case 1:
                    if (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE) {
                        MechDrive.SetTargets(0,0,600,0.5,1);
                        setIntakeEntry();
                        programOrder++;
                    }
                    break;

                case 2:
                    if (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE) {
                        setSwivelPosition(100);
                        RailControl_Outtake.SetTargetPosition(1500,-0.8,0.8);
                        programOrder++;
                    }
                    break;

                case 3:
                    if (Outtake_Rail.getCurrentPosition() < -1400) {
                        MechDrive.SetTargets(0,90,450,0.5,1);
                        programOrder++;
                    }
                    break;

                case 4:
                    if (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE) {
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 5:
                    if (ET.milliseconds() > 1000) {
                        RailControl_Outtake.SetTargetPosition(1100, -0.8, 0.8);
                        programOrder++;
                    }
                    break;

                case 6:
                    if (Outtake_Rail.getCurrentPosition() > -1150) {
                        Claw.setPower(1);
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 7:
                    MechDrive.SetTargets(0,-90,500,0.5,1);
                    if (ET.milliseconds() > 500) {
                        Claw.setPower(0);
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 8:
                    if (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE) {
                        RailControl_Outtake.SetTargetPosition(0,-0.8,0.8);
                        MechDrive.SetTargets(0,180,1800,0.5,1);
                        programOrder++;
                    }
                    break;

                case 9:
                    if (Outtake_Rail.getCurrentPosition() > -200) {
                        setSwivelPosition(0);
                        programOrder++;
                    }
                    break;

                case 10:
                    if (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE) {
                        DirectionControl.SetTargetDirection(75, 0.5);
                        if (GyroContinuity() > 74) {
                            DirectionControl.Override();
                            programOrder++;
                        }
                    }
                    break;

                case 11:
                    DirectionControl.SetTargetDirection(90, 0.2);
                    if (GyroContinuity() > 89) {
                        DirectionControl.Override();
                        ET.reset();
                        resetDriveEncoders();
                        programOrder++;
                    }
                    break;

                case 12:
                    if (ET.milliseconds() > 2000) {
                        MechDrive.SetTargets(90,180,900,0.2,1);
                        programOrder++;
                    }
                    break;

                case 13:
                    if (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE) {
                        Claw.setPower(-1);
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 14:
                    if (ET.milliseconds() > 500) {
                        MechDrive.SetTargets(90,-90,2000,0.5,1);
                        RailControl_Outtake.SetTargetPosition(500,-0.7,0.7);
                        setSwivelPosition(200);
                        programOrder++;
                    }
                    break;

                case 15:
                    if (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE) {
                        MechDrive.SetTargets(90,0,1000,0.5,1);
                        RailControl_Outtake.SetTargetPosition(1500,-0.7,0.7);
                        programOrder++;
                    }
                    break;

                case 16:
                    if (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE) {
                        MechDrive.SetTargets(90,0,400,0.2,1);
                        programOrder++;
                    }
                    break;

                case 17:
                    if (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE) {
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 18:
                    if (ET.milliseconds() > 500) {
                        RailControl_Outtake.SetTargetPosition(1100, -0.8, 0.8);
                        programOrder++;
                    }
                    break;

                case 19:
                    if (Outtake_Rail.getCurrentPosition() > -1150) {
                        Claw.setPower(1);
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 20:
                    MechDrive.SetTargets(90,180,1000,0.5,1);
                    if (ET.milliseconds() > 500) {
                        Claw.setPower(0);
                        setSwivelPosition(100);
                        RailControl_Outtake.SetTargetPosition(0, -0.8, 0.8);
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 21:
                    if (Outtake_Rail.getCurrentPosition() > -150) {
                        setSwivelPosition(0);
                        programOrder++;
                    }
                    break;

                case 22:
                    if (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE) {
                        MechDrive.SetTargets(90,90,2100,0.5,1);
                        programOrder++;
                    }
                    break;

                default:
                    break;

            }


            RailControl_Intake.RailTask();
            RailControl_Outtake.RailTask();
            MechDrive.Task(GyroContinuity());
            DirectionControl.GyroTask();
            telemetry.addData("Program Order", programOrder);
            telemetry.addData("Angle", GyroContinuity());
            telemetry.addData("Elapsed Time", ET.milliseconds());
            telemetry.addLine();
            Color_Distance_Detector();
            telemetry.addData("Blue Sample Intake?", BlueIntakeSide);
            telemetry.addData("Red Sample Intake?", RedIntakeSide);
            telemetry.addData("Yellow Sample Intake?", YellowIntakeSide);
            telemetry.addData("Unknown Sample Intake?", UnknownIntakeSide);
            telemetry.addLine();
            telemetry.addData("Intake Distance Sensor Value:", DI.getDistance(DistanceUnit.CM));
            telemetry.addData("Outtake Distance Sensor Value:", DO.getDistance(DistanceUnit.CM));
            telemetry.addData("Intake Rail Encoder Value:", Intake_Rail.getCurrentPosition());
            telemetry.addData("Outtake Rail Encoder Value:", Outtake_Rail.getCurrentPosition());
            telemetry.addData("Swivel Encoder Value:", Swivel.getCurrentPosition());
            telemetry.update();

        }
    }

    public void AttachmentPresets() {

        //Attachment Motor Presets
        Outtake_Rail.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake_Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake_Rail.setTargetPosition(0);
        Outtake_Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Outtake_Rail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake_Rail.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake_Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake_Rail.setTargetPosition(0);
        Intake_Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake_Rail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Swivel.setDirection(DcMotorSimple.Direction.REVERSE);
        Swivel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Swivel.setTargetPosition(0);
        Swivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Swivel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo Presets
        IntakeWheels.setDirection(CRServo.Direction.FORWARD);
        IntakeBucket.setDirection(Servo.Direction.FORWARD);
        setIntakeFolded();

        OuttakeBucket.setDirection(Servo.Direction.FORWARD);
        OuttakeWrist.setDirection(Servo.Direction.REVERSE);
        setOuttakeBase();

        Claw.setPower(-1);
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
            UnknownIntakeSide = false;
            BlueIntakeSide = false;
            RedIntakeSide = false;
            YellowIntakeSide = false;

        }

    }

    public void setSwivelPosition (int position){

        Swivel.setTargetPosition(position);
        Swivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Swivel.setPower(0.15);

    }

    public void setIntake() {
        IntakeBucket.setPosition(0.3);
    }

    public void setIntakeEntry() {
        IntakeBucket.setPosition(0.5);
    }

    public void setIntakeFolded() {
        IntakeBucket.setPosition(0.8);
    }

    public void setOuttake() {
        OuttakeBucket.setPosition(0.2);
        OuttakeWrist.setPosition(0.8);
    }

    public void setOuttakeTransfer() {
        OuttakeBucket.setPosition(0.5);
        OuttakeWrist.setPosition(0.2);
    }

    public void setOuttakeBase() {
        OuttakeBucket.setPosition(0.5);
        OuttakeWrist.setPosition(0.78);
    }

    public void wheelOn() {
        IntakeWheels.setPower(1);
    }

    public  void wheelOff() {
        IntakeWheels.setPower(0);
    }

    public void wheelReverse() {
        IntakeWheels.setPower(-0.8);
    }

    public void resetDriveEncoders() {
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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