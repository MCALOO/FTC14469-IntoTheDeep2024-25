package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RailTest",  group = "Rail")
public class RailTest extends LinearOpMode {

    //Motors
    static DcMotor Intake;
    static DcMotor Outtake;

    Rail_ControlV3 RailControl_Intake;
    Rail_ControlV3 RailControl_Outtake;

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

    public void runOpMode() {

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setTargetPosition(0);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Outtake = hardwareMap.get(DcMotor.class, "Outtake");
        Outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake.setTargetPosition(0);
        Outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RailControl_Intake = new Rail_ControlV3(Intake);
        RailControl_Outtake = new Rail_ControlV3(Outtake);

        waitForStart();

        while (opModeIsActive()) {

            if (!button_a_already_pressed2) {
                if (gamepad2.a) {
                    RailControl_Intake.SetTargetPosition(1500,-1,1);
                    button_a_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.a) {
                    button_a_already_pressed2 = false;
                }
            }

            if (!button_b_already_pressed2) {
                if (gamepad2.b) {
                    RailControl_Intake.SetTargetPosition(0,-1,1);
                    button_b_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.b) {
                    button_b_already_pressed2 = false;
                }
            }

            if (!button_y_already_pressed2) {
                if (gamepad2.y) {
                    RailControl_Outtake.SetTargetPosition(3000,-1,1);
                    button_y_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.y) {
                    button_y_already_pressed2 = false;
                }
            }

            if (!button_x_already_pressed2) {
                if (gamepad2.x) {
                    RailControl_Outtake.SetTargetPosition(0,-1,1);
                    button_x_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed2 = false;
                }
            }

            RailControl_Intake.RailTask();
            RailControl_Outtake.RailTask();

        }

    }

}