package org.firstinspires.ftc.teamcode.TeleOpModes;

import static java.lang.Math.abs;
import static java.lang.Math.min;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Common.RobotHardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Tools.PIDCONTROLLERTOOL;
import java.lang.Math;

@TeleOp
public class Tele extends LinearOpMode {

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    //Slide info State and target
    public int SlideTarget = 0;

    public enum SlideState{
        RETRACTED(0),
        EXTEND1(300),
        EXTEND2(600),
        EXTEND3(900),
        EXTEND4(1200),
        EXTEND5(1500),
        MAXEXTEND(1800);
        private final int ticks;
        private SlideState(final int ticks) { this.ticks = ticks; }


    }
    public int SlideStateCounter = 0;
    SlideState CurrentSlideState = SlideState.RETRACTED;




    //Mini Arm State and var
    public enum MiniArmState{
        Scoring(.7),
        HOVERING(.0),
        Intaking(.05);

        private final double miniarmangle;
        private MiniArmState(final double miniarmangle) { this.miniarmangle = miniarmangle; }


    }
    MiniArmState CurrentMiniArmState = MiniArmState.HOVERING;
    // public double MiniArmTarget = 0;


    //ClawPivot
    public enum ClawPivotState{
        RetractedPivot(.0),
        Extend1Pivot(.2),
        Extend2Pivot(.22),
        Extend3Pivot(.24),
        Extend4Pivot(.26),
        Extend5Pivot(.28),
        MaxHiehgtPivot(.3);
        private final double ClawAngle;
        private ClawPivotState(final double ClawAngle) { this.ClawAngle = ClawAngle; }
    }
    ClawPivotState CurrentClawPivot = ClawPivotState.RetractedPivot;
    //public double ClawPivotTarget = 0;


    //Claw Open Close
    public boolean ClawOpenOrClose = true; //Claw is closed if var is true
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware Robot = new RobotHardware(hardwareMap);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        PIDCONTROLLERTOOL SlideControllerLeft = new PIDCONTROLLERTOOL(.018,0,.00002,.0005,384.5/360,Robot.LeftSlide);//TODO tune these values in the test file
        PIDCONTROLLERTOOL SlideControllerRight = new PIDCONTROLLERTOOL(.018,0,.00002,.0005,384.5/360,Robot.RightSlide);//TODO tune these values in the test file

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Store gamepad
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            //Slides
            //SetPowers
            Robot.LeftSlide.setPower(SlideControllerLeft.calculatePid(SlideTarget));
            Robot.RightSlide.setPower(SlideControllerRight.calculatePid(SlideTarget));
            //Increment counter
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) SlideStateCounter++;

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) SlideStateCounter = 0;
            //Swtich Bettwen States
            if(gamepad1.right_stick_button) {
                CurrentSlideState = SlideState.values()[SlideStateCounter % SlideState.values().length];
                SlideTarget = CurrentSlideState.ticks;
            }

            //Mini Arm


            //Mini Arm Set Position
            Robot.MiniArmLeft.setPosition(CurrentMiniArmState.miniarmangle);
            Robot.MiniArmRight.setPosition(CurrentMiniArmState.miniarmangle);

            //MiniArmControlStatment
            if(currentGamepad1.y && !previousGamepad1.y && CurrentMiniArmState == MiniArmState.Scoring){
                CurrentMiniArmState = MiniArmState.HOVERING;
            }else if(currentGamepad1.y && !previousGamepad1.y && CurrentMiniArmState == MiniArmState.Intaking && CurrentSlideState != SlideState.RETRACTED){
                CurrentMiniArmState = MiniArmState.Scoring;
            } else if (currentGamepad1.y && !previousGamepad1.y && CurrentMiniArmState == MiniArmState.HOVERING) {
                CurrentMiniArmState = MiniArmState.Intaking;
            }else if(currentGamepad1.y && !previousGamepad1.y && CurrentMiniArmState == MiniArmState.Intaking && CurrentSlideState == SlideState.RETRACTED){
                CurrentMiniArmState = MiniArmState.HOVERING;
            }


            //ClawPivotControl
            if(CurrentMiniArmState == MiniArmState.Intaking || CurrentMiniArmState == MiniArmState.HOVERING){
                CurrentClawPivot = ClawPivotState.RetractedPivot;
            } else if (CurrentMiniArmState != MiniArmState.Intaking&& CurrentMiniArmState != MiniArmState.HOVERING && CurrentSlideState == SlideState.EXTEND1 ) {
                CurrentClawPivot = ClawPivotState.Extend1Pivot;
            } else if (CurrentMiniArmState != MiniArmState.Intaking&& CurrentMiniArmState != MiniArmState.HOVERING && CurrentSlideState == SlideState.EXTEND2 ) {
                CurrentClawPivot = ClawPivotState.Extend2Pivot;
            }else if (CurrentMiniArmState != MiniArmState.Intaking && CurrentMiniArmState != MiniArmState.HOVERING&& CurrentSlideState == SlideState.EXTEND3 ) {
                CurrentClawPivot = ClawPivotState.Extend3Pivot;
            }else if (CurrentMiniArmState != MiniArmState.Intaking && CurrentMiniArmState != MiniArmState.HOVERING&& CurrentSlideState == SlideState.EXTEND4 ) {
                CurrentClawPivot = ClawPivotState.Extend4Pivot;
            }else if (CurrentMiniArmState != MiniArmState.Intaking && CurrentMiniArmState != MiniArmState.HOVERING && CurrentSlideState == SlideState.EXTEND5 ) {
                CurrentClawPivot = ClawPivotState.Extend5Pivot;
            }else if (CurrentMiniArmState != MiniArmState.Intaking && CurrentMiniArmState != MiniArmState.HOVERING &&CurrentSlideState == SlideState.MAXEXTEND ) {
                CurrentClawPivot = ClawPivotState.MaxHiehgtPivot;
            }
            //ClawPivotSetPosition
            Robot.ClawPivotLeft.setPosition(CurrentClawPivot.ClawAngle);
            Robot.ClawPivotRight.setPosition(CurrentClawPivot.ClawAngle);

            //Intake Motor
            if(gamepad1.x){
                Robot.Intake.setPower(.5);
            }else if(gamepad1.b){
                Robot.Intake.setPower(-.5);
            }else{
                Robot.Intake.setPower(0);
            }

            if(currentGamepad1.a && !previousGamepad1.a && ClawOpenOrClose == false){
                Robot.Claw.setPosition(1);
                ClawOpenOrClose = true;
            }else if(currentGamepad1.a&& !previousGamepad1.a && ClawOpenOrClose == true){
                Robot.Claw.setPosition(0);
                ClawOpenOrClose = false;
            }


            //Drive
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x* 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            if(gamepad1.options){
                imu.resetYaw();
            }
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x *Math.cos(-botHeading)-y*Math.sin(-botHeading);
            double rotY = x *Math.sin(-botHeading)+y*Math.cos(-botHeading);
            rotX=rotX*1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            Robot.dtFrontLeftMotor.setPower(-frontLeftPower);
            Robot.dtBackLeftMotor.setPower(-backLeftPower);
            Robot.dtFrontRightMotor.setPower(-frontRightPower);
            Robot.dtBackRightMotor.setPower(-backRightPower);
            //Telemetry dat on the Robot
            telemetry.addData("CurrentSlideState",CurrentSlideState);
            telemetry.addData("counter", SlideStateCounter);
            telemetry.addData("current Claw Pivot state", CurrentClawPivot);
            telemetry.addData("current mini arm state", CurrentMiniArmState);
            telemetry.addData("claw State (True = claw is closed)", ClawOpenOrClose);
            telemetry.addData("claw State (True = claw is closed)", CurrentMiniArmState.miniarmangle);
            telemetry.addData("currentHeading", botHeading);
            telemetry.update();

        }
    }
}


