/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ProfiledServo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

    public static double pS = .006, iS = 0, dS = 0.0001;
    public static double fS = .01;


    public static int targetS = 0;



    public final double ticks_in_degreeS = 567.7/180;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()

    {
        //Arm
        DcMotorEx arm_motor_Left = hardwareMap.get(DcMotorEx.class, "left slide");
        DcMotorEx arm_motor_Right = hardwareMap.get(DcMotorEx.class, "Right slide");
            arm_motor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_motor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_motor_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm_motor_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideController = new PIDController(pS,iS,dS);
        SlideController.setPID(pS,iS,dS);
        //Claw
        Servo Claw = hardwareMap.get(Servo.class, "claw");
        Claw.setPosition(0.3);
        //traj setup
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(36, -60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        // servo arm setup

        //Traj Sequence Scoring
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(34,-10, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(46,-10.5, Math.toRadians(167)))
                .addDisplacementMarker(() -> {

                    Claw.setPosition(1);
                })

                .build();
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
        drive.followTrajectorySequence(trajSeq);
       // drive.update();
        while(opModeIsActive()) {




           /* if (tagOfInterest == null) {

                drive.update();
                //drive.followTrajectorySequence(park3);

            } else if (tagOfInterest.id == LEFT) {

                drive.update();
            //    drive.followTrajectorySequence(park1);

                drive.followTrajectorySequence(trajSeq);
            } else if (tagOfInterest.id == MIDDLE) {

                drive.update();
              //  drive.followTrajectorySequence(park2);
            } else {

            }
*/
           targetS = 3000;
            sleep(2000);
          //drive.update();
             //drive.followTrajectorySequence(park3);
             //   SlideController.setPID(pS, iS , dS);
                int arm_pos_Left = -(arm_motor_Left.getCurrentPosition());
                int arm_pos_Right = -(arm_motor_Right.getCurrentPosition());
                double pidLeft = SlideController.calculate(arm_pos_Left, targetS);
                double pidRight = SlideController.calculate(arm_pos_Right, targetS);
                double ff = Math.cos(Math.toRadians(targetS/ ticks_in_degreeS)) * fS; // might be the problem

                double powerLeft = pidLeft + ff;
                double powerRight = pidLeft + ff;

                arm_motor_Right.setPower(powerRight);
                arm_motor_Left.setPower(powerLeft);
             //   double servotarget = servo.getTarget();
                telemetry.addData("posLeft", arm_pos_Left);
                telemetry.addData("posRight", arm_pos_Right);
                telemetry.addData("target", targetS);
                telemetry.addData("powerleft", powerLeft);
                telemetry.addData("powerRight", powerRight);
                // telemetry.update();
               // telemetry.addData("servo target", servotarget);



                // Print pose to telemetry
                telemetry.addData("Left Slide Postion", arm_motor_Left.getCurrentPosition());
                telemetry.addData("Right Slide Postion", arm_motor_Right.getCurrentPosition());

                telemetry.update();

        }
        if(tagOfInterest == null){

            drive.followTrajectorySequence(trajSeq);

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
