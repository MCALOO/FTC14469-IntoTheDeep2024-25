package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Direction_ControlV2 {

    DcMotor frontRight, frontLeft, backRight, backLeft;
    IMU IMU;
    Orientation orientation;
    double globalangle = 0;
    double current_value;
    double prev_value = 0;
    double final_value;
    //    IMU.Parameters parameters = new IMU.Parameters();
    boolean turnright = false;
    boolean turnleft = false;
    double pwr;
    double Angle;
    double AngleTolerance;
    Task_State run_state;


    public Direction_ControlV2(IMU imu, DcMotor frontleft, DcMotor frontright, DcMotor backleft, DcMotor backright,
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection, RevHubOrientationOnRobot.UsbFacingDirection usbDirection) {

        IMU = imu;
        frontRight = frontright;
        frontLeft = frontleft;
        backRight = backright;
        backLeft = backleft;
        run_state = Task_State.INIT;

//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        RevHubOrientationOnRobot.UsbFacingDirection usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void SetTargetDirection(double angle, double power) {
        pwr = power;
        Angle = angle;
        run_state = Task_State.RUN;
    }

    public void Override() {
        run_state = Task_State.OVERRIDE;
    }

    public void GyroTask() {

        //Continuously turn to the direction chosen with the specified amount of power
        if (run_state == Task_State.RUN) {

            if (((Angle + 2) > GyroContinuity()) || ((Angle - 2) > GyroContinuity()) || turnright) {

                turnright = true;

                if (GyroContinuity() < Angle) {
                    MotorTurn(-pwr, pwr, -pwr, pwr);
                } else {
                    SetMotorPower(0);
                    turnright = false;

//                while (frontRight.getCurrentPosition() != 0) {
//                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                }
//                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

            } else if (((Angle + 2) < GyroContinuity()) || ((Angle - 2) < GyroContinuity()) || turnleft) {

                turnleft = true;

                if (GyroContinuity() > Angle) {
                    MotorTurn(pwr, -pwr, pwr, -pwr);
                } else {
                    SetMotorPower(0);
                    turnleft = false;

//                while (frontRight.getCurrentPosition() != 0) {
//                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                }
//                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }
        //Force Stop
        else if (run_state == Task_State.OVERRIDE) {
            SetMotorPower(0);
            pwr = 0;
            run_state = Task_State.DONE;
        }
    }

    public double GyroTask_TeleOp() {

        double command = 0;

        //Continuously turn to the direction chosen with the specified amount of power
        if (run_state == Task_State.RUN) {

            if (((Angle + 2) > GyroContinuity()) || ((Angle - 2) > GyroContinuity()) || turnright) {

                turnright = true;

                if (GyroContinuity() < Angle) {
                    command = pwr;
                } else {
                    command = 0;
                    turnright = false;

//                while (frontRight.getCurrentPosition() != 0) {
//                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                }
//                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

            } else if (((Angle + 2) < GyroContinuity()) || ((Angle - 2) < GyroContinuity()) || turnleft) {

                turnleft = true;

                if (GyroContinuity() > Angle) {
                    command = -pwr;
                } else {
                    command = 0;
                    turnleft = false;

//                while (frontRight.getCurrentPosition() != 0) {
//                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                }
//                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }
        //Force Stop
        else if (run_state == Task_State.OVERRIDE) {
            command = 0;
            pwr = 0;
            run_state = Task_State.DONE;
        }

        return command;
    }

    public Task_State GetTaskState() {
        return run_state;
    }

    public double GyroContinuity() {

        orientation = IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

    public void MotorTurn(double FR, double FL, double BR, double BL) {
        frontRight.setPower(FR);
        frontLeft.setPower(FL);
        backRight.setPower(BR);
        backLeft.setPower(BL);
    }

    public void SetMotorPower(double x) {
        frontLeft.setPower(x);
        frontRight.setPower(x);
        backLeft.setPower(x);
        backRight.setPower(x);
    }

}
