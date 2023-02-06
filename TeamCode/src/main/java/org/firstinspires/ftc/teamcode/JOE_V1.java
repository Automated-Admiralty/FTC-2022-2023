package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java and TrajectorySequenceRunnerCancelable.java
 * classes. Please ensure that these files are copied into your own project.
 */
@TeleOp(name = "JOE")
public class JOE_V1 extends LinearOpMode {
    // Define 2 states, drive control or automatic
    //control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    // make my new Vars

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(0, -52);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(0);


    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    @Override
    public void runOpMode() throws InterruptedException {
        Servo ArmRightServo = hardwareMap.servo.get("ArmRightServo");
        Servo ArmLeftServo = hardwareMap.servo.get("ArmLeftServo");
       // CRServo suckFront = hardwareMap.crservo.get("suckFront");
        //CRServo suckBack = hardwareMap.crservo.get("suckBack");
        DcMotor LeftSlide = hardwareMap.dcMotor.get("left slide");
        DcMotor RightSlide = hardwareMap.dcMotor.get("Right slide");
        LeftSlide.setDirection(DcMotor.Direction.REVERSE);
        RightSlide.setDirection(DcMotor.Direction.FORWARD);
        LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftSlide.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        RightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo Gripper = hardwareMap.servo.get("claw");
        ArmRightServo.setDirection(Servo.Direction.REVERSE);
        ArmLeftServo.setDirection(Servo.Direction.REVERSE);
        // Initialize custom cancelable SampleMecanumDrive class
        // Ensure that the contents are copied over from https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/SampleMecanumDriveCancelable.java
        // and https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TrajectorySequenceRunnerCancelable.java
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);
        double GripPos = 0;
        double s1pos =0.0;
        double s2pos = 0;
        boolean lbTriggred = false;
        boolean rbTriggerd = false;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Code above here


            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            if(gamepad1.a && -LeftSlide.getCurrentPosition() < 4000 && -
                    RightSlide.getCurrentPosition() < 4000){
                LeftSlide.setPower(1);
                RightSlide.setPower(1);
            } else if(gamepad1.b && -LeftSlide.getCurrentPosition() > 0 && -RightSlide.getCurrentPosition() > 0){
                LeftSlide.setPower(-1);
                RightSlide.setPower(-1);
            } else {
                LeftSlide.setPower(.00);
                RightSlide.setPower(.00);
            }

            ArmLeftServo.setPosition(s2pos);
            ArmRightServo.setPosition(s1pos);
            if(gamepad2.left_bumper ){
                s1pos =0.77;
                s2pos = 0.77;
            }else if(gamepad2.right_bumper){
                s1pos= 0.00;
                s2pos = 0.00;
            }else if (gamepad2.dpad_down){
                s1pos += 0.015;
                s2pos += 0.015;
            }else if (gamepad2.dpad_up){
                s1pos -=0.015;
                s2pos -=0.015;
            }else if(gamepad2.dpad_right){
            s1pos= 0.48;
            s2pos = 0.48;
        } else if(gamepad2.dpad_left){
            s1pos= 0.35;
            s2pos = 0.35;
        }

//lbTriggred = gamepad2.left_bumper;
//rbTriggerd = gamepad2.right_bumper;

            if(gamepad2.a){
                GripPos = 0.3;
            }
            if(gamepad2.b){
                GripPos = 0;
            }
            Gripper.setPosition(GripPos);
            /*if (gamepad2.a){
                suckFront.setPower(-1);
                suckBack.setPower(-1);
            }else if (gamepad2.b){
                suckFront.setPower(1);
                suckBack.setPower(1);
            }else {
                suckFront.setPower(0);
                suckBack.setPower(0);
            }
*/
            // Print pose to telemetry
            telemetry.addData("Left Slide Postion", LeftSlide.getCurrentPosition());
            telemetry.addData("Right Slide Postion", RightSlide.getCurrentPosition());
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Servo Target S1", s1pos);
            telemetry.addData("Servo Target S2", s2pos);
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (0 == 1) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (0 == 1) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineTo(targetBVector)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (0 == 1) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (0 == 1) {
                        drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}
