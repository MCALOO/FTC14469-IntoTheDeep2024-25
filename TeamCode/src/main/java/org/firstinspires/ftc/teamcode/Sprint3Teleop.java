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

@TeleOp(name = "Sprint3Teleop",  group = "MecanumDrive")
public class Sprint3Teleop extends LinearOpMode {

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
    Base_Control Swivel_Control;

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

    //sensor
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
    int intakeSequence = 1000;
    int outtakeBucketSequence = 1000;
    int outtakeTargetPosition;
    int resetAttachments = 1000;
    int intakeSpecimen = 1000;
    int outtakeSpecimen = 1000;
    int setObservation = 1000;
    int outtakeObservation = 1000;
    int swivelManualReset = 1000;
    int swivelPosition = 1000;

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
        Swivel = hardwareMap.get(DcMotor.class,"Swivel");

        //Servo Initalization
        IntakeBucket = hardwareMap.get(Servo.class,"IntakeBucket");
        OuttakeBucket = hardwareMap.get(Servo.class,"OuttakeBucket");
        IntakeWheels = hardwareMap.get(CRServo.class, "IntakeWheels");
        OuttakeWrist = hardwareMap.get(Servo.class,"OuttakeWrist");
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
        Swivel_Control = new Base_Control(Swivel);

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

            /******************************************************************
             * Automatic Sequence Managers
             *****************************************************************/

            //Intake automatic sequencing
            switch (intakeSequence) {

                case 1:
                    if (DO.getDistance(DistanceUnit.CM) > 10.5) {
                        setIntakeEntry();
                        ET.reset();
                        RailControl_Intake.SetTargetPosition(1000, -0.7, 0.7);
                        intakeSequence++;
                    } else {
                        intakeSequence = 1000;
                    }
                    break;

                case 2:
                    if (Intake_Rail.getCurrentPosition() < -850) {
                        wheelOn();
                        setIntake();
//                        setSwivelPosition(200);
                        Swivel_Control.SetTargetPosition(250,-0.5,0.5);
                        intakeSequence++;
                    }
                    break;

                case 3:
                    if (redTeam) {
                        if ((RedIntakeSide || YellowIntakeSide)) {
                            setOuttakeTransfer();
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
                        if ((BlueIntakeSide || YellowIntakeSide)) {
                            setOuttakeTransfer();
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
                            setOuttakeTransfer();
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
                            setIntakeFolded();
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

            switch (intakeSpecimen) {

                case 1:
                    Claw.setPower(-1);
//                    setSwivelPosition(0);
                    Swivel_Control.SetTargetPosition(-30,-0.3,0.3);
                    ET.reset();
                    intakeSpecimen++;
                    break;

                case 2:
                    if (ET.milliseconds() > 1000) {
                        RailControl_Outtake.SetTargetPosition(1450,-0.8,0.8);
                        ET.reset();
                        intakeSpecimen++;
                    }
                    break;

                case 3:
                    if (ET.milliseconds() > 500) {
//                        setSwivelPosition(200);
                        Swivel_Control.SetTargetPosition(220,-0.5,0.5);
                        setIntakeEntry();
                        intakeSpecimen++;
                    }
                    break;

                default:
                    break;
            }

            switch (outtakeBucketSequence) {

                case 1:
                    if (DO.getDistance(DistanceUnit.CM) < 10) {
//                        setSwivelPosition(100);
                        Swivel_Control.SetTargetPosition(100,-0.5,0.5);
                        OuttakeBucket.setPosition(0.3);
                        ET.reset();
                        outtakeBucketSequence++;
                    } else {
                        outtakeBucketSequence = 1000;
                        outtakeTargetPosition = 0;
                    }
                    break;

                case 2:
                    if (Swivel.getCurrentPosition() < 120 || ET.milliseconds() > 500) {
                        RailControl_Outtake.SetTargetPosition(outtakeTargetPosition - 500,-0.9,0.9);
//                        setSwivelPosition(0);
                        Swivel_Control.SetTargetPosition(0,-0.5,0.5);
                        outtakeBucketSequence++;
                    }
                    break;

                case 3:
                    if (Outtake_Rail.getCurrentPosition() < -outtakeTargetPosition + 700) {
                        OuttakeBucket.setPosition(0.2);
                        outtakeBucketSequence++;
                    }
                    break;

                default:
                    break;
            }

            switch (outtakeSpecimen) {

                case 1:
                    RailControl_Outtake.SetTargetPosition(1100,-0.5,0.5);
                    outtakeSpecimen++;
                    break;

                case 2:
                    if (Outtake_Rail.getCurrentPosition() > -1150) {
                        Claw.setPower(1);
                        ET.reset();
                        outtakeSpecimen++;
                    }
                    break;

                case 3:
                    if (ET.milliseconds() > 500) {
                        Claw.setPower(0);
//                        setSwivelPosition(100);
                        Swivel_Control.SetTargetPosition(100,-0.5,0.5);
                        ET.reset();
                        outtakeSpecimen++;
                    }
                    break;

                case 4:
                    if (Swivel.getCurrentPosition() < 140) {
                        RailControl_Outtake.SetTargetPosition(0,-0.8,0.8);
                        outtakeSpecimen++;
                    }
                    break;

                case 5:
                    if (Outtake_Rail.getCurrentPosition() > -150 || ET.milliseconds() > 2000) {
//                        setSwivelPosition(0);
                        Swivel_Control.SetTargetPosition(-30,-0.3,0.3);
                        outtakeSpecimen++;
                        ET.reset();
                    }
                    break;

                case 6:
                    if (ET.milliseconds() > 500) {
                        Outtake_Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        outtakeSpecimen++;
                    }
                    break;

                default:
                    break;

            }

            switch (setObservation) {

                case 1:
//                    setSwivelPosition(0);
                    Swivel_Control.SetTargetPosition(100,-0.5,0.5);
                    RailControl_Outtake.SetTargetPosition(500,-0.7,0.7);
                    OuttakeBucket.setPosition(0.2);
                    ET.reset();
                    setIntakeEntry();
                    setObservation++;
                    break;

                case 2:
                    if (ET.milliseconds() > 500) {
                        setOuttake();
                        ET.reset();
                    }
                    setObservation++;
                    break;

                case 3:
                    if (ET.milliseconds() > 500) {
                        ET.reset();
                    }
                    setObservation++;
                    break;

                default:
                    break;
            }


            switch (outtakeObservation) {

                case 1:
                    setOuttake();
                    outtakeObservation++;
                    ET.reset();
                    break;

                case 2:
                    if (ET.milliseconds() > 1000) {
                        Swivel_Control.SetTargetPosition(100,-0.3,0.3);
                        RailControl_Outtake.SetTargetPosition(0,-0.7,0.7);
                        setOuttakeBase();
                        ET.reset();
                        outtakeObservation++;
                    }
                    break;

                case 3:
                    if (ET.milliseconds() > 500) {
//                        setSwivelPosition(0);
                        Swivel_Control.SetTargetPosition(-30,-0.3,0.3);
                        outtakeObservation++;
                    }
                    break;

                default:
                    break;
            }

            switch (resetAttachments) {

                case 1:
                    setOuttake();
                    ET.reset();
                    resetAttachments++;
                    break;

                case 2:
                    if (ET.milliseconds() > 2500) {
                        setIntakeEntry();
                        setOuttakeBase();
                        wheelOff();
                        RailControl_Intake.SetTargetPosition(0, -0.7, 0.7);
                        RailControl_Outtake.SetTargetPosition(100, -0.6, 6);
//                        setSwivelPosition(100);
                        Swivel_Control.SetTargetPosition(100,-0.3,0.3);
                        intakeSequence = 1000;
                        outtakeBucketSequence = 1000;
                        outtakeTargetPosition = 0;
                        Claw.setPower(1);
                        ET.reset();
                        resetAttachments++;
                    }
                    break;

                case 3:
                    if (Outtake_Rail.getCurrentPosition() > -200 || ET.milliseconds() > 2000) {
                        RailControl_Outtake.SetTargetPosition(0,-0.7,0.7);
//                        setSwivelPosition(0);
                        Swivel_Control.SetTargetPosition(-30,-0.3,0.3);
                        if (ET.milliseconds() > 300) {
                            Claw.setPower(0);
                            if (ET.milliseconds() > 1000) {
                                ET.milliseconds();
                                resetAttachments++;
                            }
                        }
                    }
                    break;

                case 4:
                    Outtake_Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    if (ET.milliseconds() > 500) {
                        resetAttachments++;
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
//                    setSwivelPosition(0);
                    Swivel_Control.SetTargetPosition(-30,-0.5,0.5);
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
//                    setSwivelPosition(0);
                    Swivel_Control.SetTargetPosition(0,-0.5,0.5);
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

                    intakeSpecimen = 1;

                    button_a_already_pressed = true;
                }
            } else {
                if (!gamepad2.a) {
                    button_a_already_pressed = false;
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

                    outtakeTargetPosition = 3300;
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

                    outtakeTargetPosition = 2000;
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

                    intakeSequence = 1;

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

                    if (outtakeTargetPosition > 100) {
                        resetAttachments = 1;
                    } else if (outtakeTargetPosition < 100) {
                        resetAttachments = 2;
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

                    outtakeObservation = 1;

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

                    setObservation = 1;

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

//                    RailControl_Intake.SetTargetPosition(1000,-0.7,0.7);

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

//                    RailControl_Intake.SetTargetPosition(0,-0.7,0.7);

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

                    setIntake();

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

                    setIntakeEntry();

                    button_bumper_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                }
            }

//            /******************************************************************
//             * Trigger (G2) Manual Calibration For Base
//             *****************************************************************/
//
//            if (button_left_trigger_already_pressed2 == false) {
//                if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0) {
//
//                    basePosition = basePosition - 100;
//                    baseManualReset = 1;
//
//                    button_left_trigger_already_pressed2 = true;
//                }
//            } else {
//                if (gamepad2.left_trigger == 0) {
//                    baseManualReset = 0;
//                    button_left_trigger_already_pressed2 = false;
//                }
//            }
//
//            if (button_right_trigger_already_pressed2 == false) {
//                if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0) {
//
//                    basePosition = basePosition + 100;
//                    baseManualReset = 1;
//
//                    button_right_trigger_already_pressed2 = true;
//                }
//            } else {
//                if (gamepad2.right_trigger == 0) {
//                    baseManualReset = 0;
//                    button_right_trigger_already_pressed2 = false;
//                }
//            }



            RailControl_Intake.RailTask();
            RailControl_Outtake.RailTask();
            Swivel_Control.BaseTask();
            telemetry.addData("Red Team?", redTeam);
            telemetry.addData("Blue Team?", blueTeam);
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

//        Swivel.setDirection(DcMotorSimple.Direction.REVERSE);
//        Swivel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Swivel.setTargetPosition(0);
//        Swivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Swivel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo Presets
        IntakeWheels.setDirection(CRServo.Direction.FORWARD);
        IntakeBucket.setDirection(Servo.Direction.FORWARD);
        setIntakeFolded();

        OuttakeBucket.setDirection(Servo.Direction.FORWARD);
        OuttakeWrist.setDirection(Servo.Direction.REVERSE);
        setOuttakeBase();
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

//        Swivel.setTargetPosition(position);
//        Swivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Swivel.setPower(0.15);

    }

    public void setIntake() {
        IntakeBucket.setPosition(0.33);
    }

    public void setIntakeEntry() {
        IntakeBucket.setPosition(0.55);
    }

    public void setIntakeFolded() {
        IntakeBucket.setPosition(0.8);
    }

    public void setOuttake() {
        OuttakeBucket.setPosition(0.2);
        OuttakeWrist.setPosition(0.8);
    }

    public void setOuttakeTransfer() {
        OuttakeBucket.setPosition(0.45);
        OuttakeWrist.setPosition(0.15);
    }

    public void setOuttakeBase() {
        OuttakeBucket.setPosition(0.5);
        OuttakeWrist.setPosition(0.78);
    }

    public void wheelOn() {
        IntakeWheels.setPower(0.5);
    }

    public  void wheelOff() {
        IntakeWheels.setPower(0);
    }

    public void wheelReverse() {
        IntakeWheels.setPower(-1);
    }

}