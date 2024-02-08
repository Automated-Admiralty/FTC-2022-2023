package org.firstinspires.ftc.teamcode.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Common.RobotHardware.ARobotHardware;
import org.firstinspires.ftc.teamcode.Common.SubSyststem.TeamElementSubsystem;
import org.firstinspires.ftc.teamcode.Common.Tools.PIDCONTROLLERTOOL;
import   org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(group = "advanced")
public class AsyncFollowingFSM extends LinearOpMode {
    double intakPower = 0;
    int firsttimerReset = 0;
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
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

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.Idle;
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
    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-35 , 60, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        TeamElementSubsystem TeamElementSubsystem = new TeamElementSubsystem(hardwareMap);
        TeamElementSubsystem.setAlliance("blue");
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ARobotHardware robot = new ARobotHardware(hardwareMap);
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
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
                .forward(1)
                .build();

        TrajectorySequence MoveToBoardCenter = drive.trajectorySequenceBuilder(MiddlePurplePixelPlace.end())
                .strafeRight(18)
                .splineToConstantHeading(new Vector2d(-28,11),Math.toRadians(0))
                .strafeLeft(75)
                .back(24)
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence MoveToBoardRight = drive.trajectorySequenceBuilder(PurplePixelPlaceRight.end())
                .strafeLeft(5)
                //.splineToConstantHeading(new Vector2d(-30,8),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-28,4),Math.toRadians(0))
                .strafeLeft(73.55)
                .back(24.5)
                .turn(Math.toRadians(-92))
                .build();
        TrajectorySequence MoveToBoardLeft = drive.trajectorySequenceBuilder(PurplePlacePixelLeft.end())
                .back(12)
                .splineToConstantHeading(new Vector2d(-28,-5),Math.toRadians(0))
                .splineTo(new Vector2d(-28+90,11),Math.toRadians(0))
                .turn(Math.toRadians(180))
                // .turn(Math.toRadians(-90))
                //.strafeLeft(75)
                .strafeRight(28)
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);
        int SelectedLocation= 0;
        PIDCONTROLLERTOOL ASlideControllerLeft = new PIDCONTROLLERTOOL(.018,0,.00002,.0005,384.5/360,robot.LeftSlide);
        PIDCONTROLLERTOOL ASlideControllerRight = new PIDCONTROLLERTOOL(.018,0,.00002,.0005,384.5/360,robot.RightSlide);

        while(opModeInInit()){
            robot.Claw.setPosition(0);
            robot.LeftSlide.setPower(ASlideControllerLeft.calculatePid(ACurrentSlideState.Aticks));
            robot.RightSlide.setPower(ASlideControllerRight.calculatePid(ACurrentSlideState.Aticks));
           // robot.MiniArmLeft.setPosition(ACurrentMiniArmState.Aminiarmangle);
           // robot.MiniArmRight.setPosition(ACurrentMiniArmState.Aminiarmangle);
            robot.ClawPivotLeft.setPosition(ACurrentClawPivot.AClawAngle);
            robot.ClawPivotRight.setPosition(ACurrentClawPivot.AClawAngle);
            robot.Intake.setPower(intakPower);

            SelectedLocation = TeamElementSubsystem.elementDetection(telemetry);
            telemetry.addData("selected",SelectedLocation);
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands

        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.SPIKEPLACETRAJ;

        //TeamElementSubsystem.elementDetection(telemetry);
        currentState = State.SPIKEPLACETRAJ;
        if(SelectedLocation == 1) {
            drive.followTrajectorySequenceAsync(PurplePlacePixelLeft);
        } else if (SelectedLocation == 3) {
            drive.followTrajectoryAsync(PurplePixelPlaceRight);
        }else {
            drive.followTrajectorySequenceAsync(MiddlePurplePixelPlace);
        }
      //  waitTimer1.reset();
        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.
            if(firsttimerReset == 0){
                waitTimer1.reset();

            }
            // We essentially define the flow of the state machine through this switch statement

            switch (currentState) {
                case SPIKEPLACETRAJ:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function

                    if (!drive.isBusy()) {
                        if(firsttimerReset == 0){
                            waitTimer1.reset();
                            firsttimerReset += 1;
                        }
                        intakPower = .45;
                        if(waitTimer1.seconds() >= .4){
                            intakPower = 0;
                            currentState = State.OUTAKESPIKE;
                            waitTimer1.reset();
                        }
                        //drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case OUTAKESPIKE:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.BACKBOARDTRAJ;
                        if(SelectedLocation == 1) {
                            drive.followTrajectorySequenceAsync(MoveToBoardLeft);
                        } else if (SelectedLocation == 3) {
                            drive.followTrajectorySequenceAsync(MoveToBoardRight);
                        }else {
                            drive.followTrajectorySequenceAsync(MoveToBoardCenter);
                        }
                        waitTimer1.reset();
                    }
                    break;
                case BACKBOARDTRAJ:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                     //   if(waitTimer1.seconds()>=1) {
                            ACurrentSlideState = ASlideState.AEXTEND3;
                            if(waitTimer1.seconds()>=.5){
                            ACurrentMiniArmState = AMiniArmState.AScoring;
                            ACurrentClawPivot = AClawPivotState.AExtend3Pivot;
                            currentState = State.ARMUP;
                            waitTimer1.reset();
                        }
                     //   drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case ARMUP:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        if(waitTimer1.seconds() >=.5){
                            robot.Claw.setPosition(1);
                            currentState = State.MINIARMCLAWPIVOT;

                            // Start the wait timer once we switch to the next state
                            // This is so we can track how long we've been in the WAIT_1 state
                            waitTimer1.reset();
                        }
                    }

                    break;
                case MINIARMCLAWPIVOT:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state

                    if (waitTimer1.seconds() >= .35) {
                        ACurrentSlideState = ASlideState.ARETRACTED;
                        ACurrentMiniArmState = AMiniArmState.AHOVERING;
                        ACurrentClawPivot = AClawPivotState.ARetractedPivot;
                        currentState = State.CLAWOPEN;
                      //  drive.turnAsync(turnAngle2);
                    }
                    break;
                case CLAWOPEN:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.MINIARMRESET;
                    }
                    break;
                case MINIARMRESET:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.SlideRetract;
                    }
                    break;
                case SlideRetract:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.Idle;
                    }
                    break;
                case Idle:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            robot.LeftSlide.setPower(ASlideControllerLeft.calculatePid(ACurrentSlideState.Aticks));
            robot.RightSlide.setPower(ASlideControllerRight.calculatePid(ACurrentSlideState.Aticks));
            robot.MiniArmLeft.setPosition(ACurrentMiniArmState.Aminiarmangle);
            robot.MiniArmRight.setPosition(ACurrentMiniArmState.Aminiarmangle);
            robot.ClawPivotLeft.setPosition(ACurrentClawPivot.AClawAngle);
            robot.ClawPivotRight.setPosition(ACurrentClawPivot.AClawAngle);
            robot.Intake.setPower(intakPower);
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop

}
