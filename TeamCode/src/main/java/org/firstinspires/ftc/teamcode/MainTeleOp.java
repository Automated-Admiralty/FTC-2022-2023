package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@TeleOp
public class MainTeleOp extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    private ProfiledServo servo;
    //Slide PIDF
    private PIDController SlideController;

    public static double pS = .006, iS = 0, dS = 0.0001;
    public static double fS = .01;


    public static int targetS = 0;



    public final double ticks_in_degreeS = 567.7/180;



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
       DcMotorEx arm_motor_Left = hardwareMap.get(DcMotorEx.class, "left slide");
        DcMotorEx arm_motor_Right = hardwareMap.get(DcMotorEx.class, "right slide");
        double s1pos = 0;
        servo = new ProfiledServo(hardwareMap, "ArmLeftServo", "ArmRightServo", .3, .3, .3, .3, s1pos);
        Servo Claw = hardwareMap.get(Servo.class, "claw");

        arm_motor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideController = new PIDController(pS,iS,dS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor_Left.setDirection(DcMotorSimple.Direction.REVERSE);


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




            if(gamepad2.y ){
                s1pos = 0;
            }else if(gamepad2.a){
                s1pos = 1;
            }       if(gamepad2.b ){
                s1pos = 0.5;
            }else if(gamepad2.x){
                s1pos = 0.3;
            }

            if(gamepad2.left_bumper ){
                Claw.setPosition(0);
            }else if(gamepad2.right_bumper){
                 Claw.setPosition(0.3);
            }

            servo.setPosition(s1pos);

            if(gamepad2.dpad_up && targetS <= 4000){
                targetS = 4000;
            }else if(gamepad2.dpad_down && targetS >= 0){
                targetS = 0;
            }else if(gamepad2.dpad_left && targetS != 3000){
                targetS = 3000;
            }else if(gamepad2.dpad_right && targetS != 1000){
                targetS = 1000;
            }
            if(gamepad1.dpad_up && targetS != 4000){
                targetS = 4000;
            }else if(gamepad1.dpad_down && targetS != 0){
                targetS = 0;
            }

            servo.periodic();

            SlideController.setPID(pS, iS , dS);
            int arm_pos_Left = -(arm_motor_Left.getCurrentPosition());
            int arm_pos_Right = -(arm_motor_Right.getCurrentPosition());
            double pidLeft = SlideController.calculate(arm_pos_Left, targetS);
           // double pidRight = SlideController.calculate(arm_pos_Right, targetS);
            double ff = Math.cos(Math.toRadians(targetS/ ticks_in_degreeS)) * fS; // might be the problem

            double powerLeft = pidLeft + ff;
            double powerRight = pidLeft + ff;

            arm_motor_Right.setPower(powerRight);
            arm_motor_Left.setPower(powerLeft);
            double servotarget = servo.getTarget();
            telemetry.addData("posLeft", arm_pos_Left);
            telemetry.addData("posRight", arm_pos_Right);
            telemetry.addData("target", targetS);
            telemetry.addData("powerleft", powerLeft);
            telemetry.addData("powerRight", powerRight);
           // telemetry.update();
            telemetry.addData("servo target", servotarget);



            // Print pose to telemetry
            telemetry.addData("Left Slide Postion", arm_motor_Left.getCurrentPosition());
            telemetry.addData("Right Slide Postion", arm_motor_Right.getCurrentPosition());
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
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
