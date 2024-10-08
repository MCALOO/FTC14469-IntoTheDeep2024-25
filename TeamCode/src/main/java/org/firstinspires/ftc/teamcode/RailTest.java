package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "RailTest",  group = "Rail")
public class RailTest extends LinearOpMode {

    DcMotor Rail;

    Rail_ControlV3 RailControlV3;

    boolean button_a_already_pressed = false;
    boolean button_b_already_pressed = false;
    boolean button_x_already_pressed = false;
    boolean button_y_already_pressed = false;

    public void runOpMode() {

        Rail = hardwareMap.get(DcMotor.class, "rail");

        RailControlV3 = new Rail_ControlV3(Rail);

        waitForStart();

        while (opModeIsActive()) {

            if (!button_a_already_pressed) {
                if (gamepad2.a) {
                    RailControlV3.SetTargetPosition(1000, -0.85, 0.85);
                    button_a_already_pressed = true;
                }
            } else {
                if (!gamepad2.a) {
                    button_a_already_pressed = false;
                }
            }

            if (!button_b_already_pressed) {
                if (gamepad2.b) {
                    RailControlV3.SetTargetPosition(2000, -0.85, 0.85);
                    button_b_already_pressed = true;
                }
            } else {
                if (!gamepad2.b) {
                    button_b_already_pressed = false;
                }
            }

            if (!button_y_already_pressed) {
                if (gamepad2.y) {
                    RailControlV3.SetTargetPosition(3000, -0.85, 0.85);
                    button_y_already_pressed = true;
                }
            } else {
                if (!gamepad2.y) {
                    button_y_already_pressed = false;
                }
            }

            if (!button_x_already_pressed) {
                if (gamepad2.x) {
                    RailControlV3.SetTargetPosition(0, -0.85, 0.85);
                    button_x_already_pressed = true;
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed = false;
                }
            }

            RailControlV3.RailTask();
            telemetry.update();

        }

    }


}
