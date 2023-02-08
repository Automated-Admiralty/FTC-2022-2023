
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ProfiledServo;
import org.firstinspires.ftc.teamcode.advanced.AsyncFollowingFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class PLUS_IDK_MATE extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        ARMUP_1,   // Then, follow a lineTo() trajectory
        SERVO_Arm_Up,         // Then we want to do a point turn
        Claw_open,   // Then, we follow another lineTo() trajectory
        Arm_down,         // Then we're gonna wait a second
        Servo_Arm_Down,         // Finally, we're gonna turn again
        Claw_Close,
        Idle// Our bot will enter the IDLE state when done
    }
    PLUS_IDK_MATE.State currentState = PLUS_IDK_MATE.State.Idle;

    @Override
    public void runOpMode()

    {
double count = 0;
double downPos = 0;
        //Arm
        DcMotorEx arm_motor_Left = hardwareMap.get(DcMotorEx.class, "left slide");
        DcMotorEx arm_motor_Right = hardwareMap.get(DcMotorEx.class, "Right slide");
        SlideController = new PIDController(pS,iS,dS);
        arm_motor_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        //Claw//
        Servo Claw = hardwareMap.get(Servo.class, "claw");
        Claw.setPosition(0.0);
        //traj setup
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(46 , -60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        // servo arm setup
        Servo ArmRightServo = hardwareMap.servo.get("ArmRightServo");
        Servo ArmLeftServo = hardwareMap.servo.get("ArmLeftServo");
        ArmRightServo.setDirection(Servo.Direction.REVERSE);
        ArmLeftServo.setDirection(Servo.Direction.REVERSE);
        //Traj Sequence Scoring
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36,-60, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36,-16.1, Math.toRadians(270)))
                .turn(Math.toRadians(-110))
                .lineToLinearHeading(new Pose2d(51,-16.1,Math.toRadians(160)))
                //.splineToLinearHeading(new Pose2d(50,-9), Math.toRadians(160))
             .lineToLinearHeading(new Pose2d(50.3,1
                     , Math.toRadians(152)))
                               .build();
        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(45, -20), Math.toRadians(90))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineTo(new Vector2d(45, 0))
                .build();

        // Define the angle to turn at
        double turnAngle1 = Math.toRadians(-270);

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));
        Trajectory trajectory3 = drive.trajectoryBuilder(newLastPose)
                .lineToConstantHeading(new Vector2d(-15, 0))
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);
        // traj sequence park left

        //traj sequence park Right

        // traj sequence park center

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */



        /* Update the telemetry */

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */

        // drive.update();
        currentState = State.ARMUP_1;
        drive.followTrajectorySequenceAsync(trajSeq);
        while(opModeIsActive() && !isStopRequested()) {
            switch (currentState) {

                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    //timer.reset();
                    if (timer.seconds() >= .35 + (count * .07)) {
                        currentState = State.SERVO_Arm_Up;
                        targetS = 2760;
                        timer.reset();
                    }
                    break;
                case ARMUP_1:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if(!drive.isBusy()){
                        currentState = State.TRAJECTORY_1;
                        ArmLeftServo.setPosition(0.03);
                        ArmRightServo.setPosition(0.03);
                       // sleep(1000);
                        timer.reset();
                    }
                    break;
                case SERVO_Arm_Up:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                //    timer.reset();
                    if (arm_motor_Left.getCurrentPosition() > 2500 && arm_motor_Left.getCurrentPosition() < 3500 && arm_motor_Right.getCurrentPosition() > 2500 && arm_motor_Right.getCurrentPosition() < 3500 && timer.seconds()>= .4) {

                        currentState = State.Claw_open;
                        Claw.setPosition(0.3);
                        timer.reset();
                    }
                     //   sleep(500);

                    break;
                case Claw_open:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                   // timer.reset();
                    if(timer.seconds() >= .3) {
                        currentState = State.Arm_down;
                        downPos = .81 + count * .04;
                        ArmLeftServo.setPosition(downPos);
                        ArmRightServo.setPosition(downPos);

                     //   sleep(500);
    // Start the wait timer once we switch to the next state
    // This is so we can track how long we've been in the WAIT_1 state
                        timer.reset();
}

                    break;
                case Arm_down:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    //timer.reset();
                    if(timer.seconds() >= .3) {

                        targetS = 0;
                        currentState = State.Servo_Arm_Down;

                    }
                    break;
                case Servo_Arm_Down:

                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (arm_motor_Left.getCurrentPosition() > -50 && arm_motor_Left.getCurrentPosition() < 100 && arm_motor_Right.getCurrentPosition() > -50 && arm_motor_Right.getCurrentPosition() < 100) {

                        currentState = State.Claw_Close;
                        Claw.setPosition(0.0);
                        //sleep(500);
                        timer.reset();
                    }
                    break;
                case Claw_Close:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                   // timer.reset();
                    if(timer.seconds() >= 1) {
                        currentState = State.Idle;
                    }
                    break;
                case Idle:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        count++;
                        currentState = State.ARMUP_1;
                    }
                    break;
            }
            drive.update();
            SlideController.setPID(pS, iS , dS);
            int arm_pos_Left = (arm_motor_Left.getCurrentPosition());
            int arm_pos_Right = (arm_motor_Right.getCurrentPosition());
            double pidLeft = SlideController.calculate(arm_pos_Left, targetS);
            // double pidRight = SlideController.calculate(arm_pos_Right, targetS);
            double ff = Math.cos(Math.toRadians(targetS/ ticks_in_degreeS)) * fS; // might be the problem

            double powerLeft = pidLeft + ff;
            double powerRight = pidLeft + ff;

            arm_motor_Right.setPower(powerRight);
            arm_motor_Left.setPower(powerLeft);
            telemetry.addData("posLeft", arm_pos_Left);
            telemetry.addData("posRight", arm_pos_Right);
            telemetry.addData("target", targetS);
            telemetry.addData("powerleft", powerLeft);
            telemetry.addData("powerRight", powerRight);
            // telemetry.update();
            // telemetry.addData("servo target", servotarget);
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;


            // Print pose to telemetry
            telemetry.addData("Left Slide Postion", arm_motor_Left.getCurrentPosition());
            telemetry.addData("Right Slide Postion", arm_motor_Right.getCurrentPosition());

            telemetry.update();

        }
        if(tagOfInterest == null){



        }else if(tagOfInterest.id == LEFT){
           /* SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Pose2d startPose = new Pose2d(36, -60, Math.toRadians(270));
            drive.setPoseEstimate(startPose);
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(34,-8, Math.toRadians(270)))
                    .lineToLinearHeading(new Pose2d(46,-6.5, Math.toRadians(150)))
                    .build();
            drive.followTrajectorySequence(trajSeq);
            */

            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Pose2d startPosePost = new Pose2d(46, -10.5, Math.toRadians(167));
            drive.setPoseEstimate(startPosePost);
            TrajectorySequence ParkLeft = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(35,-34, Math.toRadians(270)))
                    .lineToLinearHeading(new Pose2d(-10,-34, Math.toRadians(270)))
                    .build();
            drive.followTrajectorySequence(ParkLeft);
        }else if(tagOfInterest.id == MIDDLE){
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Pose2d startPosePost = new Pose2d(46, -10.5, Math.toRadians(167));
            drive.setPoseEstimate(startPosePost);
            TrajectorySequence ParkMid = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(35,-33, Math.toRadians(270)))
                    .build();
            drive.followTrajectorySequence(ParkMid);
        }else{
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Pose2d startPosePost = new Pose2d(46, -10.5, Math.toRadians(167));
            drive.setPoseEstimate(startPosePost);
            TrajectorySequence ParkRight = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(35,-33, Math.toRadians(270)))
                    .lineToLinearHeading(new Pose2d(90,-33, Math.toRadians(270)))
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