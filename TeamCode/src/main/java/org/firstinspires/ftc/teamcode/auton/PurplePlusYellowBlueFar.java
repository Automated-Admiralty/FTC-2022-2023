
package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.RobotHardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.SubSyststem.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.Common.Tools.PIDCONTROLLERTOOL;
import org.firstinspires.ftc.teamcode.TeleOpModes.Tele;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class PurplePlusYellowBlueFar extends LinearOpMode
{
    static final String SPIKE_CENTER = "center";
    static final String SPIKE_LEFT = "left";
    static final String SPIKE_RIGHT = "right";

    static final String SIDE_FAR = "far";
    static final String SIDE_CLOSE = "close";

    static final String ALLIANCE_RED = "red";
    static final String ALLIANCE_BLUE = "blue";
    // OpenCvCamera camera;
    TeamElementSubsystem TeamElementSubsystem;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    private PIDController SlideController;
ElapsedTime timer = new ElapsedTime();
    public static double pS = .006, iS = 0, dS = 0.0001;
    public static double fS = .01;


    public static int targetS = 0;



    public final double ticks_in_degreeS = 567.7/180;


double time_bot_change =  .07;
    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    enum State {
        SPIKEPLACETRAJ,   // First, follow a splineTo() trajectory
        OUTAKESPIKE,   // Then, follow a lineTo() trajectory
        BACKBOARDTRAJ,         // Then we want to do a point turn
        ARMUP,   // Then, we follow another lineTo() trajectory
        MINIARMCLAWPIVOT,         // Then we're gonna wait a second
               // Finally, we're gonna turn again
        CLAWOPEN,
        MINIARMRESET,
        SlideRetract,
        Idle// Our bot will enter the IDLE state when done
    }
    PurplePlusYellowBlueFar.State currentState = PurplePlusYellowBlueFar.State.Idle;
    public enum ASlideState{
        ARETRACTED(0),
        AEXTEND1(300),
        AEXTEND2(600),
        AEXTEND3(900),
        AEXTEND4(1200),
        AEXTEND5(1500),
        AMAXEXTEND(1800);
        private final int Aticks;
        private ASlideState(final int Aticks) { this.Aticks = Aticks; }


    }
    ASlideState ACurrentSlideState = ASlideState.ARETRACTED;




    //Mini Arm State and var
    public enum AMiniArmState{
        AScoring(.7),
        AHOVERING(.0),
        AIntaking(.05);

        private final double Aminiarmangle;
        private AMiniArmState(final double Aminiarmangle) { this.Aminiarmangle = Aminiarmangle; }


    }
   AMiniArmState ACurrentMiniArmState = AMiniArmState.AHOVERING;
    // public double MiniArmTarget = 0;


    //ClawPivot
    public enum AClawPivotState{
        ARetractedPivot(.0),
        AExtend1Pivot(.2),
        AExtend2Pivot(.22),
        AExtend3Pivot(.24),
        AExtend4Pivot(.26),
        AExtend5Pivot(.28),
        AMaxHiehgtPivot(.3);
        private final double AClawAngle;
        private AClawPivotState(final double AClawAngle) { this.AClawAngle = AClawAngle; }
    }
   AClawPivotState ACurrentClawPivot = AClawPivotState.ARetractedPivot;
    @Override
    public void runOpMode()

    {
double count = 0;
double downPos = 0;
        //Arm
        RobotHardware robot = new RobotHardware(hardwareMap);
        //Claw//
      //  Servo Claw = hardwareMap.get(Servo.class, "claw");
        // Claw.setPosition(0.0);
        //traj setup
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-35 , 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        // servo arm setup

        //Traj Sequence Scoring

        //this is the steps of the driving
        TrajectorySequence MiddlePurplePixelPlace = drive.trajectorySequenceBuilder(startPose)
                .forward(28)
               // .turn(Math.toRadians(90))
                               .build();


        // Let's define our trajectories
        Trajectory PurplePixelPlaceRight = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-46.5,38))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        TrajectorySequence PurplePlacePixelLeft = drive.trajectorySequenceBuilder(startPose)
                .forward(28)
                .turn(Math.toRadians(90))
                .forward(2)
                .build();

        TrajectorySequence MoveToBoardCenter = drive.trajectorySequenceBuilder(MiddlePurplePixelPlace.end())
                .strafeRight(18)
                .splineToConstantHeading(new Vector2d(-28,11),Math.toRadians(0))
                .strafeLeft(75)
                .back(24)
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence MoveToBoardRight = drive.trajectorySequenceBuilder(PurplePixelPlaceRight.end())
                .strafeLeft(8)
                .splineToConstantHeading(new Vector2d(-28,11),Math.toRadians(0))
                .strafeLeft(73)
                .back(16)
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence MoveToBoardLeft = drive.trajectorySequenceBuilder(PurplePlacePixelLeft.end())
                .back(18)
                .splineToConstantHeading(new Vector2d(-28,11),Math.toRadians(0))
                .splineTo(new Vector2d(-28+75,11),Math.toRadians(0))
                .turn(Math.toRadians(180))
                // .turn(Math.toRadians(-90))
                //.strafeLeft(75)
                .strafeRight(30)
                .build();

        // Let's define our trajectories


        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one

        // Define the angle to turn at
        double turnAngle1 = Math.toRadians(-270);




        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        TeamElementSubsystem = new TeamElementSubsystem(hardwareMap);

        String alliance = ALLIANCE_BLUE;
        String spike = SPIKE_LEFT;
        String side = SIDE_CLOSE;




        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {

        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */


        PIDCONTROLLERTOOL ASlideControllerLeft = new PIDCONTROLLERTOOL(.018,0,.00002,.0005,384.5/360,robot.LeftSlide);//TODO tune these values in the test file
        PIDCONTROLLERTOOL ASlideControllerRight = new PIDCONTROLLERTOOL(.018,0,.00002,.0005,384.5/360,robot.RightSlide);//TODO tune these values in the test file
        robot.LeftSlide.setPower(ASlideControllerLeft.calculatePid(ACurrentSlideState.Aticks));
        robot.RightSlide.setPower(ASlideControllerRight.calculatePid(ACurrentSlideState.Aticks));
        robot.MiniArmLeft.setPosition(ACurrentMiniArmState.Aminiarmangle);
        robot.MiniArmRight.setPosition(ACurrentMiniArmState.Aminiarmangle);
        robot.ClawPivotLeft.setPosition(ACurrentClawPivot.AClawAngle);
        robot.ClawPivotRight.setPosition(ACurrentClawPivot.AClawAngle);
        robot.Claw.setPosition(1);

        /* Update the telemetry */



        /* Actually do something useful */

       // drive.update();
        int SelectedLocation = TeamElementSubsystem.elementDetection(telemetry);
        currentState = State.SPIKEPLACETRAJ;


        while(opModeIsActive() && !isStopRequested()) {
            switch (currentState) {

                case SPIKEPLACETRAJ:
                    if(SelectedLocation == 1) {
                        drive.followTrajectorySequenceAsync(PurplePlacePixelLeft);
                    } else if (SelectedLocation == 3) {
                        drive.followTrajectoryAsync(PurplePixelPlaceRight);
                    }else {
                        drive.followTrajectorySequenceAsync(MiddlePurplePixelPlace);
                    }
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    //timer.reset();

                    if (timer.seconds() >= .35 ) {
                        currentState = State.OUTAKESPIKE;
                        timer.reset();
                    }
                    break;
                case OUTAKESPIKE:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    //timer.reset();
                    robot.Intake.setPower(-.5);
                    if (timer.seconds() >= .35 ) {
                        currentState = State.BACKBOARDTRAJ;
                        timer.reset();
                    }
                    break;
                case BACKBOARDTRAJ:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if(SelectedLocation == 1) {
                        drive.followTrajectorySequenceAsync(MoveToBoardLeft);
                    } else if (SelectedLocation == 3) {
                        drive.followTrajectorySequenceAsync(MoveToBoardRight);
                    }else {
                        drive.followTrajectorySequenceAsync(MoveToBoardCenter);
                    }
                    robot.Intake.setPower(0);
                    if(!drive.isBusy()){
                        currentState = State.ARMUP;


                        timer.reset();
                    }
                    break;
                case ARMUP:
                    ACurrentSlideState = ASlideState.AEXTEND3;
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                //    timer.reset();
                    if(timer.seconds()>= .4) {

                        currentState = State.MINIARMCLAWPIVOT;

                    }
                     //   sleep(500);

                    break;
                case MINIARMCLAWPIVOT:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                   // timer.reset();
                    ACurrentMiniArmState = AMiniArmState.AScoring;
                    ACurrentClawPivot = AClawPivotState.AExtend3Pivot;
                    if(timer.seconds() >= .4) {
                        currentState = State.CLAWOPEN;


                     //   sleep(500);
    // Start the wait timer once we switch to the next state
    // This is so we can track how long we've been in the WAIT_1 state
                        timer.reset();
}

                    break;
                case CLAWOPEN:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    //timer.reset();
                    robot.Claw.setPosition(0);
                    if(timer.seconds() >= .3) {


                        currentState = State.MINIARMRESET;
                        timer.reset();
                    }
                    break;
                case MINIARMRESET:

                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    ACurrentMiniArmState = AMiniArmState.AHOVERING;
                    ACurrentClawPivot = AClawPivotState.ARetractedPivot;
                    if (timer.seconds() >= .3) {

                        currentState = State.SlideRetract;

                        //sleep(500);
                        timer.reset();
                    }
                    break;
                case SlideRetract:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                   // timer.reset();
                   ACurrentSlideState = ASlideState.ARETRACTED;
                   if(timer.seconds()>=.3){
                       currentState = State.Idle;
                       timer.reset();
                   }
                    break;
                case Idle:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    break;
            }
            robot.LeftSlide.setPower(ASlideControllerLeft.calculatePid(ACurrentSlideState.Aticks));
            robot.RightSlide.setPower(ASlideControllerRight.calculatePid(ACurrentSlideState.Aticks));
            robot.MiniArmLeft.setPosition(ACurrentMiniArmState.Aminiarmangle);
            robot.MiniArmRight.setPosition(ACurrentMiniArmState.Aminiarmangle);
            robot.ClawPivotLeft.setPosition(ACurrentClawPivot.AClawAngle);
            robot.ClawPivotRight.setPosition(ACurrentClawPivot.AClawAngle);
            drive.update();
            telemetry.addData("SlidePosition", ACurrentSlideState);
            telemetry.addData("Mini Arm Pos", ACurrentMiniArmState);
            telemetry.addData("Claw Pivot", ACurrentClawPivot );


            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;


            // Print pose to telemetry


            telemetry.update();

        }
        if(tagOfInterest == null){



        }else if(tagOfInterest.id == LEFT){

            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Pose2d startPosePost = new Pose2d(52.1,-.2, Math.toRadians(152.5));
            drive.setPoseEstimate(startPosePost);
            TrajectorySequence ParkLeft = drive.trajectorySequenceBuilder(startPosePost)
                    .lineToLinearHeading(new Pose2d(52.1,-15.8, Math.toRadians(160)))
                    .lineToLinearHeading(new Pose2d(-10,-15.8, Math.toRadians(160)))
                    .build();
            drive.followTrajectorySequence(ParkLeft);

        }else if(tagOfInterest.id == MIDDLE){
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Pose2d startPosePost = new Pose2d(52.1,-.2, Math.toRadians(152.5));
            drive.setPoseEstimate(startPosePost);
            TrajectorySequence ParkMiddle = drive.trajectorySequenceBuilder(startPosePost)
                    .lineToLinearHeading(new Pose2d(52.1,-15.8, Math.toRadians(160)))
                    .lineToLinearHeading(new Pose2d(37,-15.8, Math.toRadians(160)))
                    .build();
            drive.followTrajectorySequence(ParkMiddle);
        }else{
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Pose2d startPosePost = new Pose2d(52.1,-.2, Math.toRadians(152.5));
            drive.setPoseEstimate(startPosePost);
            TrajectorySequence ParkRight = drive.trajectorySequenceBuilder(startPosePost)
                    .lineToLinearHeading(new Pose2d(52.1,-15.8, Math.toRadians(160)))
                    .lineToLinearHeading(new Pose2d(60,-15.8, Math.toRadians(160)))
                    .build();
            drive.followTrajectorySequence(ParkRight);
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}