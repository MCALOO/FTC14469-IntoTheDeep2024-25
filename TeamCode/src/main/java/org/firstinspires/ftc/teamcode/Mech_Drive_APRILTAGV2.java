package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

public class Mech_Drive_APRILTAGV2 {

    DcMotor FrontLeft, FrontRight, BackLeft, BackRight;     // DC motors for each of the Mecanum wheels
    double flpower, frpower, blpower, brpower;              // Power command for each of the DC motors

    ElapsedTime ET = new ElapsedTime();
    Telemetry telemetry;
    PID pid;
    double targetcoordinateX;
    double targetcoordinateZ;
    double strafingangle;
    double headingangle = 0;
    double targetpower;
    double failsafedistance;
    double steeringoutput;
    Task_State state;
    int encoderselect; //0 is frontright, 1 is backright

    double finalpower;

    double poseX;
    double poseZ;
    boolean tagfound;
    int mechOrder = 0;
    double error;
    double kp_gain;
    double power;

    static final double FEET_PER_METER = 3.28084;

    // CONSTRUCTOR
    public Mech_Drive_APRILTAGV2(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, MoveDirection Direction, Telemetry Telemetry) {

        // Assign the motor connected to the bucket and initialize it
        FrontRight = fr;
        FrontLeft = fl;
        BackRight = br;
        BackLeft = bl;

        SetDirection(Direction);
        pid = new PID();
        telemetry = Telemetry;

        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        state = Task_State.INIT;

    }

    // METHOD THAT A STATE MACHINE OPMODE SHOULD CALL WHEN IT IS READY TO LAUNCH THE NEXT TASK IN ITS LIST
    public void SetTargets(int HeadingAngle,
                           double StrafingAngle,
                           double TargetCoordinateX,
                           double TargetCoordinateZ,
                           double TargetPower,
                           double FailSafeDistance) {

        headingangle = HeadingAngle;
        strafingangle = StrafingAngle;
        targetcoordinateX = TargetCoordinateX;
        targetcoordinateZ = TargetCoordinateZ;
        targetpower = TargetPower;
        failsafedistance = FailSafeDistance;

        mechOrder = 0;

        state = Task_State.RUN;
    }

    public void Override() {
        state = Task_State.OVERRIDE;
    }

    public void Done() {
        state = Task_State.DONE;
    }

    // THIS IS THE TASK THAT A STATE MACHINE OPMODE SHOULD CALL REPEATEDLY IN ITS LOOP
    public void Task (double gyro_Z_reading, AprilTagDetection detection, boolean tagfound) {

        double power_x_old, power_x_new;
        double power_y_old, power_y_new;
        double denominator;
        double radians = Math.toRadians(-strafingangle); // negate strafing angle for left hand rule
        double encoder;


        encoder = FrontRight.getCurrentPosition();

        if (encoder < 0) {
            encoder = -encoder;
        }

        // Always run this PID control when in RUN
        if (state == Task_State.RUN) {

            switch (mechOrder) {
                case 0:
                    finalpower = targetpower;
                    power_x_old = 0;                // make x_old 0 to make the degrees start at the front of the robot
                    power_y_old = finalpower;

                    power_x_new = power_x_old * Math.cos(radians) - power_y_old * Math.sin(radians); // equation for right hand rule
                    power_y_new = power_x_old * Math.sin(radians) + power_y_old * Math.cos(radians);
                    steeringoutput = pid.PID_Control(headingangle, 0.035, 0.00015, 0, gyro_Z_reading);

                    if (!tagfound && (encoder <= failsafedistance)) {

                        denominator = Math.max(Math.abs(power_y_new) + Math.abs(power_x_new), 1);
                        flpower = (power_y_new + 1.1 * power_x_new + steeringoutput) / denominator;
                        blpower = (power_y_new - 1.1 * power_x_new + steeringoutput) / denominator;
                        frpower = (power_y_new - 1.1 * power_x_new - steeringoutput) / denominator;
                        brpower = (power_y_new + 1.1 * power_x_new - steeringoutput) / denominator;

                        if (tagfound) {
                            FrontLeft.setPower(0);
                            FrontRight.setPower(0);
                            BackLeft.setPower(0);
                            BackRight.setPower(0);
                        } else {
                            FrontLeft.setPower(flpower);
                            FrontRight.setPower(frpower);
                            BackLeft.setPower(blpower);
                            BackRight.setPower(brpower);
                        }
                    } else {

                        if (encoder > failsafedistance) {
                            state = Task_State.FAIL;
                        }

                        FrontLeft.setPower(0);
                        FrontRight.setPower(0);
                        BackLeft.setPower(0);
                        BackRight.setPower(0);

                        mechOrder++;
                    }
                    break;

                case 1:

                    poseX = detection.pose.x * FEET_PER_METER;
                    if ((targetcoordinateX * FEET_PER_METER) > poseX) {
                        mechOrder++;
                    } else {
                        mechOrder = 3;
                    }
                    break;

                case 2:

                    if (encoder > failsafedistance) {
                        state = Task_State.FAIL;
                    }

                    poseX = detection.pose.x * FEET_PER_METER;
                    if ((targetcoordinateX * FEET_PER_METER) > poseX) {

                        strafingangle = -90;
                        radians = Math.toRadians(-strafingangle); // negate strafing angle for left hand rule

                        error = Math.abs((targetcoordinateX * FEET_PER_METER - poseX));
                        kp_gain = 0.2;

                        power = error * kp_gain;

                        finalpower = Range.clip(power, 0.25, targetpower);
                        power_x_old = 0;                // make x_old 0 to make the degrees start at the front of the robot
                        power_y_old = finalpower;

                        power_x_new = power_x_old * Math.cos(radians) - power_y_old * Math.sin(radians); // equation for right hand rule
                        power_y_new = power_x_old * Math.sin(radians) + power_y_old * Math.cos(radians);
                        steeringoutput = pid.PID_Control(headingangle, 0.035, 0.00015, 0, gyro_Z_reading);

                        if ((targetcoordinateX * FEET_PER_METER) > poseX) {

                            denominator = Math.max(Math.abs(power_y_new) + Math.abs(power_x_new), 1);
                            flpower = (power_y_new + 1.1 * power_x_new + steeringoutput) / denominator;
                            blpower = (power_y_new - 1.1 * power_x_new + steeringoutput) / denominator;
                            frpower = (power_y_new - 1.1 * power_x_new - steeringoutput) / denominator;
                            brpower = (power_y_new + 1.1 * power_x_new - steeringoutput) / denominator;


                            FrontLeft.setPower(flpower);
                            FrontRight.setPower(frpower);
                            BackLeft.setPower(blpower);
                            BackRight.setPower(brpower);

                        } else {

                            FrontLeft.setPower(0.25);
                            FrontRight.setPower(0.25);
                            BackLeft.setPower(0.25);
                            BackRight.setPower(0.25);

                            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            mechOrder = 4;
                        }
                    } else {

                        FrontLeft.setPower(0.25);
                        FrontRight.setPower(0.25);
                        BackLeft.setPower(0.25);
                        BackRight.setPower(0.25);

                        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        mechOrder = 4;
                    }
                    break;

                case 3:

                    if (encoder > failsafedistance) {
                        state = Task_State.FAIL;
                    }

                    poseX = detection.pose.x * FEET_PER_METER;
                    if ((targetcoordinateX * FEET_PER_METER) < poseX) {

                        strafingangle = 90;
                        radians = Math.toRadians(-strafingangle); // negate strafing angle for left hand rule

                        error = Math.abs((targetcoordinateX * FEET_PER_METER - poseX));
                        kp_gain = 0.2;

                        power = error * kp_gain;

                        finalpower = Range.clip(power, 0.25, targetpower);

                        power_x_old = 0;                // make x_old 0 to make the degrees start at the front of the robot
                        power_y_old = finalpower;

                        power_x_new = power_x_old * Math.cos(radians) - power_y_old * Math.sin(radians); // equation for right hand rule
                        power_y_new = power_x_old * Math.sin(radians) + power_y_old * Math.cos(radians);
                        steeringoutput = pid.PID_Control(headingangle, 0.035, 0.00015, 0, gyro_Z_reading);

                        if ((targetcoordinateX * FEET_PER_METER) < poseX) {

                            denominator = Math.max(Math.abs(power_y_new) + Math.abs(power_x_new), 1);
                            flpower = (power_y_new + 1.1 * power_x_new + steeringoutput) / denominator;
                            blpower = (power_y_new - 1.1 * power_x_new + steeringoutput) / denominator;
                            frpower = (power_y_new - 1.1 * power_x_new - steeringoutput) / denominator;
                            brpower = (power_y_new + 1.1 * power_x_new - steeringoutput) / denominator;


                            FrontLeft.setPower(flpower);
                            FrontRight.setPower(frpower);
                            BackLeft.setPower(blpower);
                            BackRight.setPower(brpower);

                        } else {

                            FrontLeft.setPower(0);
                            FrontRight.setPower(0);
                            BackLeft.setPower(0);
                            BackRight.setPower(0);

                            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            mechOrder++;
                        }
                    } else {

                        FrontLeft.setPower(0);
                        FrontRight.setPower(0);
                        BackLeft.setPower(0);
                        BackRight.setPower(0);

                        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        mechOrder++;
                    }
                    break;

                case 4:

                    poseZ = detection.pose.z * FEET_PER_METER;
                    if ((targetcoordinateZ * FEET_PER_METER) < poseZ) {

                        strafingangle = 0;
                        radians = Math.toRadians(-strafingangle); // negate strafing angle for left hand rule

                        error = Math.abs((targetcoordinateZ * FEET_PER_METER - poseZ));
                        kp_gain = 0.2;

                        power = error * kp_gain;

                        finalpower = Range.clip(power, 0.25, targetpower);

                        power_x_old = 0;                // make x_old 0 to make the degrees start at the front of the robot
                        power_y_old = finalpower;

                        power_x_new = power_x_old * Math.cos(radians) - power_y_old * Math.sin(radians); // equation for right hand rule
                        power_y_new = power_x_old * Math.sin(radians) + power_y_old * Math.cos(radians);
                        steeringoutput = pid.PID_Control(headingangle, 0.035, 0.00015, 0, gyro_Z_reading);

                        if ((targetcoordinateZ * FEET_PER_METER) < poseZ) {

                            denominator = Math.max(Math.abs(power_y_new) + Math.abs(power_x_new), 1);
                            flpower = (power_y_new + 1.1 * power_x_new + steeringoutput) / denominator;
                            blpower = (power_y_new - 1.1 * power_x_new + steeringoutput) / denominator;
                            frpower = (power_y_new - 1.1 * power_x_new - steeringoutput) / denominator;
                            brpower = (power_y_new + 1.1 * power_x_new - steeringoutput) / denominator;


                            FrontLeft.setPower(flpower);
                            FrontRight.setPower(frpower);
                            BackLeft.setPower(blpower);
                            BackRight.setPower(brpower);

                        } else {
                            FrontLeft.setPower(0);
                            FrontRight.setPower(0);
                            BackLeft.setPower(0);
                            BackRight.setPower(0);

                            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            mechOrder++;
                        }
                    } else {
                        FrontLeft.setPower(0);
                        FrontRight.setPower(0);
                        BackLeft.setPower(0);
                        BackRight.setPower(0);

                        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        mechOrder++;
                    }
                    break;

                case 5:
                    state = Task_State.DONE;
                    mechOrder++;
                    break;

                default:
                    break;
            }

        }
        else if (state == Task_State.DONE){
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            state = Task_State.READY;
        }
        else if (state == Task_State.OVERRIDE) {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            state = Task_State.READY;
        }
        else if (state == Task_State.FAIL) {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            state = Task_State.READY;
        }

        //telemetry.addData("ActualDistance", encoder);
        //telemetry.addData("steering output", steeringoutput);
        //telemetry.addData("Steering", steeringoutput);
        //telemetry.addData("DirectionZ", gyro_Z_reading);
        //telemetry.addData("Position", FrontRight.getCurrentPosition());
        telemetry.addData("Mechorder: ", mechOrder);
        telemetry.update();

    }

    // A STATE MACHINE OPMODE SHOULD CALL THIS METHOD TO DETERMINE WHETHER THE TASK IS DONE
    public Task_State GetTaskState() {

        return state;
    }

    public void GetDoneState() {
//        return Task_State.DONE;
        state = Task_State.DONE;
    }

    private void SetDirection (MoveDirection direction) {

        if (direction == MoveDirection.FORWARD) {
            FrontLeft.setDirection(DcMotor.Direction.REVERSE);
            FrontRight.setDirection(DcMotor.Direction.FORWARD);
            BackLeft.setDirection(DcMotor.Direction.REVERSE);
            BackRight.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == MoveDirection.REVERSE) {
            FrontLeft.setDirection(DcMotor.Direction.FORWARD);
            FrontRight.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.FORWARD);
            BackRight.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    private void StrafeLeft() {
        FrontLeft.setPower(-0.25);
        BackRight.setPower(-0.25);
        FrontRight.setPower(0.25);
        BackLeft.setPower(0.25);
    }

    private void StrafeRight() {
        FrontLeft.setPower(0.25);
        BackRight.setPower(0.25);
        FrontRight.setPower(-0.25);
        BackLeft.setPower(-0.25);
    }

    private void MoveForwards() {
        FrontLeft.setPower(0.25);
        BackRight.setPower(0.25);
        FrontRight.setPower(0.25);
        BackLeft.setPower(0.25);
    }

    private void MoveBackwards() {
        FrontLeft.setPower(-0.2);
        BackRight.setPower(-0.2);
        FrontRight.setPower(-0.2);
        BackLeft.setPower(-0.2);
    }

    private void Stop() {
        FrontLeft.setPower(0);
        BackRight.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
    }

}