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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sprint4Teleop",  group = "MecanumDrive")
public class Sprint4Teleop extends LinearOpMode {

    //Motors
    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor Intake_Rail;
    static DcMotor Outtake_Rail;

    Rail_ControlV3 RailControl_Intake;
    Rail_ControlV3 RailControl_Outtake;

    //Servos
    static Servo Outtake; // Servo Mode
    static Servo Intake; // Servo Mode
    static CRServo Wheels; // Continuous Rotation Mode
    static Servo Arm; // Servo Mode
    static Servo Swivel; // Servo Mode
    static CRServo Claw; // Continuous Rotation Mode

    //Sensors
    static NormalizedColorSensor CI;
    static DistanceSensor DI;
    static DistanceSensor DO;

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
    boolean button_right_stick_button = false;
    boolean button_left_stick_button = false;
    boolean button_right_stick_button2 = false;
    boolean button_left_stick_button2 = false;

    //sensor
    boolean UnknownIntake;
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
    int intakeSequence = 1000;
    int outtakeBucketSequence = 1000;
    int outtakeTargetPosition = 0;
    boolean extendIntakeRail;
    int resetAttachments = 1000;
    int intakeSpecimen = 1000;
    int outtakeSpecimen = 1000;
    int setSpecimen = 1000;
    int setObservation = 1000;
    int outtakeObservation = 1000;

    //Elapsed Timer
    ElapsedTime ET = new ElapsedTime();

    public void runOpMode() {

        //Motor Initalization
        BackLeft = hardwareMap.get(DcMotor.class, "leftBack");
        BackRight = hardwareMap.get(DcMotor.class, "rightBack");
        FrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        FrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        Intake_Rail = hardwareMap.get(DcMotor.class, "IntakeRail");
        Outtake_Rail = hardwareMap.get(DcMotor.class, "OuttakeRail");

        //Servo Initalization
        Intake = hardwareMap.get(Servo.class,"Intake");
        Outtake = hardwareMap.get(Servo.class,"Outtake");
        Wheels = hardwareMap.get(CRServo.class, "Wheels");
        Arm = hardwareMap.get(Servo.class,"Arm");
        Swivel = hardwareMap.get(Servo.class,"Swivel");
        Claw = hardwareMap.get(CRServo.class,"Claw");

        //Sensor Initalization
        CI = hardwareMap.get(NormalizedColorSensor.class,"CI");
        DI = hardwareMap.get(DistanceSensor.class,"DI");
        DO = hardwareMap.get(DistanceSensor.class,"DO");

        //Set Drive Motor Directions
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set Servo and Motor Presets
        AttachmentPresets();

        RailControl_Intake = new Rail_ControlV3(Intake_Rail);
        RailControl_Outtake = new Rail_ControlV3(Outtake_Rail);

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
                    movement = 0.1;
                } else {
                    movement = 0.85;
                }
            }

            /******************************************************************
             * Left/Right Analog Sticks (G1) - Movement
             *****************************************************************/

            double y = -gamepad1.left_stick_y * movement;
            double x = -gamepad1.left_stick_x * movement;
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

            /******************************************************************
             * Automatic Sequence Managers
             *****************************************************************/

            //Intake automatic sequencing
            switch (intakeSequence) {

                case 1:
                    if (extendIntakeRail == false) {
                        setIntakeEntry();
                        ET.reset();
                        RailControl_Intake.SetTargetPosition(0, -0.7, 0.7);
                        intakeSequence++;
                    } else {
                        setIntakeEntry();
                        ET.reset();
                        RailControl_Intake.SetTargetPosition(1000, -0.7, 0.7);
                        intakeSequence++;
                    }
                    break;

                case 2:
                    if (extendIntakeRail == false) {
                        wheelOn();
                        setIntakeActive();
                        intakeSequence++;
                    } else {
                        if (Intake_Rail.getCurrentPosition() < -850) {
                            wheelOn();
                            setIntakeActive();
                            intakeSequence++;
                        }
                    }
                    break;

                case 3:
                    if (redTeam) {
                        if ((RedIntakeSide || YellowIntakeSide || UnknownIntake)) {
                            setIntakeEntry();
                            RailControl_Intake.SetTargetPosition(0, -0.7, 0.7);
                            ET.reset();
                            intakeSequence++;
                        } else if (BlueIntakeSide) {
                            wheelReverse();
                        } else {
                            wheelOn();
                        }
                    } else if (blueTeam) {
                        if ((BlueIntakeSide || YellowIntakeSide || UnknownIntake)) {
                            setIntakeEntry();
                            RailControl_Intake.SetTargetPosition(0, -0.7, 0.7);
                            ET.reset();
                            intakeSequence++;
                        } else if (RedIntakeSide) {
                            wheelReverse();
                        } else {
                            wheelOn();
                        }
                    } else if (!blueTeam && !redTeam) {
                        if (RedIntakeSide || YellowIntakeSide || BlueIntakeSide) {
                            setIntakeEntry();
                            RailControl_Intake.SetTargetPosition(0, -0.7, 0.7);
                            ET.reset();
                            intakeSequence++;
                        }
                    }
                    break;

                case 4:
                    if (ET.milliseconds() > 250) {
                        wheelOff();
                        if (Intake_Rail.getCurrentPosition() > -150) {
                            setIntakeBase();
                            ET.reset();
                            intakeSequence++;
                        }
                    }
                    break;

                case 5:
                    if (ET.milliseconds() > 500) {
                        wheelReverse();
                        intakeSequence++;
                    }
                    break;

                case 6:
                    if (DO.getDistance(DistanceUnit.CM) < 10) {
                        wheelOff();
                        setIntakeEntry();
                        intakeSequence++;
                    }
                    break;

                default:
                    break;
            }

            switch (outtakeBucketSequence) {

                case 1:
                    setIntakeEntry();
                    setArmDrop();
                    setOuttakeRaised();
                    RailControl_Outtake.SetTargetPosition(outtakeTargetPosition - 500,-1,1);
                    ET.reset();
                    outtakeBucketSequence++;
                    break;

                case 2:
                    if (Outtake_Rail.getCurrentPosition() < -1000) {
                        RailControl_Outtake.SetTargetPosition(outtakeTargetPosition,-0.9,0.9);
                        setSwivelBack();
                        outtakeBucketSequence++;
                    }
                    break;

                default:
                    break;

            }

            switch (resetAttachments) {

                case 1:
                    setOuttakeDrop();
                    ET.reset();
                    intakeSpecimen = 0;
                    outtakeBucketSequence = 0;
                    resetAttachments++;
                    break;

                case 2:
                    if (ET.milliseconds() > 2500) {
                        setSwivelFront();
                        setIntakeEntry();
                        setArmBase();
                        setOuttakeBase();
                        wheelOff();
                        RailControl_Intake.SetTargetPosition(0, -0.7, 0.7);
                        RailControl_Outtake.SetTargetPosition(0, -0.6, 0.6);
                        intakeSequence = 1000;
                        outtakeBucketSequence = 1000;
                        outtakeTargetPosition = 0;
                        Claw.setPower(1);
                        ET.reset();
                        resetAttachments++;
                    }
                    break;

                case 3:
                    if (ET.milliseconds() > 300) {
                        Claw.setPower(0);
                        ET.reset();
                        resetAttachments++;
                    }
                    break;

                default:
                    break;
            }

            switch (setSpecimen) {

                case 1:
                    setIntakeEntry();
                    setArmDrop();
                    setSwivelBack();
                    ET.reset();
                    clawOpen();
                    setSpecimen++;
                    break;

                case 2:
                    if (ET.milliseconds() > 1000) {
                        setOuttakeDrop();
                        clawOff();
                        ET.reset();
                        setSpecimen++;
                    }
                    break;

                case 3:
                    if (ET.milliseconds() > 1000) {
                        setArmBase();
                        setSpecimen++;
                    }
                    break;

                default:
                    break;
            }

            switch (intakeSpecimen) {

                case 1:
                    clawClose();
                    ET.reset();
                    intakeSpecimen++;
                    break;

                case 2:
                    if (ET.milliseconds() > 1000) {
                        RailControl_Outtake.SetTargetPosition(1350,-0.8,0.8);
                        ET.reset();
                        intakeSpecimen++;
                    }
                    break;

                case 3:
                    if (ET.milliseconds() > 500) {
                        Swivel.setPosition(0.75);
                        setOuttakeBase();
                        setArmBase();
                        setIntakeEntry();
                        intakeSpecimen++;
                    }
                    break;

                default:
                    break;
            }

            switch (outtakeSpecimen) {

                case 1:
                    RailControl_Outtake.SetTargetPosition(950,-0.5,0.5);
                    outtakeSpecimen++;
                    break;

                case 2:
                    if (Outtake_Rail.getCurrentPosition() > -1000) {
                        clawOpen();
                        ET.reset();
                        outtakeSpecimen++;
                    }
                    break;

                case 3:
                    if (ET.milliseconds() > 200) {
                        clawOff();
                        ET.reset();
                        RailControl_Outtake.SetTargetPosition(0,-0.8,0.8);
                        outtakeSpecimen++;
                    }
                    break;

                case 4:
                    if (ET.milliseconds() > 500) {
                        clawClose();
                        ET.reset();
                        outtakeSpecimen++;
                    }
                    break;

                case 5:
                    if (ET.milliseconds() > 200) {
                        clawOff();
                        setSwivelFront();
                        outtakeSpecimen++;
                    }
                    break;

                default:
                    break;
            }

            /*****************************************************************
             * Button Guide (G1) :
             *****************************************************************/

            if (!button_guide_already_pressed) {
                if (gamepad1.guide) {
                    redTeam = false;
                    blueTeam = false;
                    button_guide_already_pressed = true;
                }
            } else {
                if (!gamepad1.guide) {
                    button_guide_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button Back (G1) :
             *****************************************************************/

            if (!button_back_already_pressed) {
                if (gamepad1.back) {
                    redTeam = false;
                    blueTeam = true;
                    button_back_already_pressed = true;
                }
            } else {
                if (!gamepad1.back) {
                    button_back_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button Start (G1) :
             *****************************************************************/

            if (!button_start_already_pressed) {
                if (gamepad1.start) {
                    redTeam = true;
                    blueTeam = false;
                    button_start_already_pressed = true;
                }
            } else {
                if (!gamepad1.start) {
                    button_start_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button A (G1) :
             *****************************************************************/

            if (!button_a_already_pressed) {
                if (gamepad1.a) {

                    setSpecimen = 1;

                    button_a_already_pressed = true;
                }
            } else {
                if (!gamepad1.a) {
                    button_a_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button B (G1) :
             *****************************************************************/

            if (!button_b_already_pressed) {
                if (gamepad1.b) {

                    intakeSpecimen = 1;

                    button_b_already_pressed = true;
                }
            } else {
                if (!gamepad1.b) {
                    button_b_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button Y (G1) :
             *****************************************************************/

            if (!button_y_already_pressed) {
                if (gamepad1.y) {

                    outtakeSpecimen = 1;

                    button_y_already_pressed = true;
                }
            } else {
                if (!gamepad1.y) {
                    button_y_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button Guide (G2) : Turn off Intake Wheels
             *****************************************************************/

            if (!button_guide_already_pressed2) {
                if (gamepad2.guide) {

                    wheelOff();
                    clawOff();

                    button_guide_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.guide) {
                    button_guide_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button Back (G2) : Reverse Intake Wheels
             *****************************************************************/

            if (!button_back_already_pressed2) {
                if (gamepad2.back) {

                    wheelReverse();

                    button_back_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.back) {
                    button_back_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button Start (G2) : Turn on Intake Wheels
             *****************************************************************/

            if (!button_start_already_pressed2) {
                if (gamepad2.start) {

                    wheelOn();

                    button_start_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.start) {
                    button_start_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button Y (G2) : Start the Outtake Sequence for High Bucket
             **************************************************************z***/

            if (!button_y_already_pressed2) {
                if (gamepad2.y) {

                    outtakeTargetPosition = 2800;
                    outtakeBucketSequence = 1;

                    button_y_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.y) {
                    button_y_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button B (G2) : Start the Outtake Sequence for Low Bucket
             *****************************************************************/

            if (!button_b_already_pressed2) {
                if (gamepad2.b) {

                    outtakeTargetPosition = 1500;
                    outtakeBucketSequence = 1;

                    button_b_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.b) {
                    button_b_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button A (G2) : Start the Intake Sequence
             *****************************************************************/

            if (!button_a_already_pressed2) {
                if (gamepad2.a) {



                    button_a_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.a) {
                    button_a_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button X (G2) : Reset All Attachments to Start Position
             *****************************************************************/

            if (!button_x_already_pressed2) {
                if (gamepad2.x) {

                    if (outtakeTargetPosition == 0) {
                        resetAttachments = 2;
                    } else {
                        resetAttachments = 1;
                    }

                    button_x_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed2 = false;
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
             * Right Bumper (G2) :
             *****************************************************************/

            if (!button_bumper_right_already_pressed2) {
                if (gamepad2.right_bumper) {

                    extendIntakeRail = false;
                    intakeSequence = 1;

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

                    extendIntakeRail = true;
                    intakeSequence = 1;

                    button_bumper_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Right Stick Button (G2) :
             *****************************************************************/

            if (!button_right_stick_button2) {
                if (gamepad2.right_stick_button) {



                    button_right_stick_button2 = true;
                }
            } else {
                if (!gamepad2.right_stick_button) {
                    button_right_stick_button2 = false;
                }
            }

            /*****************************************************************
             * Left Stick Button (G2) :
             *****************************************************************/

            if (!button_left_stick_button2) {
                if (gamepad2.left_stick_button) {



                    button_left_stick_button2 = true;
                }
            } else {
                if (!gamepad2.left_stick_button) {
                    button_left_stick_button2 = false;
                }
            }

            RailControl_Intake.RailTask();
            RailControl_Outtake.RailTask();
            telemetry.addData("Intake Rail", Intake_Rail.getCurrentPosition());
            telemetry.addData("Outtake Rail", Outtake_Rail.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Red Team?", redTeam);
            telemetry.addData("Blue Team?", blueTeam);
            telemetry.addLine();
            Color_Distance_Detector();
            telemetry.addData("Blue Sample Intake?", BlueIntakeSide);
            telemetry.addData("Red Sample Intake?", RedIntakeSide);
            telemetry.addData("Yellow Sample Intake?", YellowIntakeSide);
            telemetry.addData("Unknown Sample Intake?", UnknownIntake);
            telemetry.addLine();
            telemetry.addData("Intake Distance Sensor Value:", DI.getDistance(DistanceUnit.CM));
//            telemetry.addData("Outtake Distance Sensor Value:", DO.getDistance(DistanceUnit.CM));
            telemetry.addData("Intake Rail Encoder Value:", Intake_Rail.getCurrentPosition());
            telemetry.addData("Outtake Rail Encoder Value:", Outtake_Rail.getCurrentPosition());
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

        //Servo Presets
        Outtake.setDirection(Servo.Direction.FORWARD);
        Intake.setDirection(Servo.Direction.REVERSE);
        Wheels.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm.setDirection(Servo.Direction.FORWARD);
        Swivel.setDirection(Servo.Direction.REVERSE);
        Claw.setDirection(DcMotorSimple.Direction.REVERSE);

        setOuttakeBase();
        setIntakeBase();
        setArmBase();
        setSwivelFront();

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
                UnknownIntake = false;
                BlueIntakeSide = true;
                RedIntakeSide = false;
                YellowIntakeSide = false;
            } else if (HSV[0] <= 65 && HSV[0] >= 15) {
                telemetry.addData("Color:", "Red");
                UnknownIntake = false;
                BlueIntakeSide = false;
                RedIntakeSide = true;
                YellowIntakeSide = false;
            } else if (HSV[0] <= 100 && HSV[0] >= 70) {
                telemetry.addData("Color:", "Yellow");
                UnknownIntake = false;
                BlueIntakeSide = false;
                RedIntakeSide = false;
                YellowIntakeSide = true;
            } else {
                telemetry.addData("Color:", "Unknown");
                UnknownIntake = true;
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
                UnknownIntake = false;
                BlueIntakeSide = true;
                RedIntakeSide = false;
                YellowIntakeSide = false;
            } else if (HSV[0] <= 69 && HSV[0] >= 40) {
                telemetry.addData("Color:", "Red");
                UnknownIntake = false;
                BlueIntakeSide = false;
                RedIntakeSide = true;
                YellowIntakeSide = false;
            } else if (HSV[0] <= 100 && HSV[0] >= 71) {
                telemetry.addData("Color:", "Yellow");
                UnknownIntake = false;
                BlueIntakeSide = false;
                RedIntakeSide = false;
                YellowIntakeSide = true;
            } else {
                telemetry.addData("Color:", "Unknown");
                UnknownIntake = true;
                BlueIntakeSide = false;
                RedIntakeSide = false;
                YellowIntakeSide = false;
            }

        } else {

            telemetry.addData("Sample Position:","None Found");
            UnknownIntake = false;
            BlueIntakeSide = false;
            RedIntakeSide = false;
            YellowIntakeSide = false;

        }

    }

    public void setIntakeActive() {
        Intake.setPosition(0.63);
    }

    public void setIntakeEntry() {
        Intake.setPosition(0.3);
    }

    public void setIntakeBase() {
        Intake.setPosition(0.1);
    }

    public void setArmBase() {
        Arm.setPosition(0.55);
    }

    public void setArmDrop() {
        Arm.setPosition(0.2);
    }

    public void setOuttakeBase() {
        Outtake.setPosition(0);
    }

    public void setOuttakeDrop() {
        Outtake.setPosition(0.8);
    }

    public void setOuttakeRaised() {
        Outtake.setPosition(0.4);
    }

    public void setSwivelFront() {
        Swivel.setPosition(0.84);
    }

    public void setSwivelBack() {
        Swivel.setPosition(0.12);
    }

    public void setSwivelMiddle() {
        Swivel.setPosition(0.5);
    }

    public void wheelOn() {
        Wheels.setPower(1);
    }

    public  void wheelOff() {
        Wheels.setPower(0);
    }

    public void wheelReverse() {
        Wheels.setPower(-0.8);
    }

    public void clawClose() {
        Claw.setPower(1);
    }

    public void clawOff() {
        Claw.setPower(0);
    }

    public void clawOpen() {
        Claw.setPower(-1);
    }

}