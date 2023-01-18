package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ProfiledServo implements Subsystem {
    public Servo servo_right;
    public Servo servo_left;
    protected double endPosition;
    protected double previousEndPosition;
    protected double currentPosition;
    protected String name;

    public MotionProfile profile_m;
    public MotionConstraint forwardConstraint;
    public MotionConstraint backwardContraint;
    public ElapsedTime timer = new ElapsedTime();

    public ProfiledServo(HardwareMap hwmap, String name1, String name2, double Forwardvelo, double Forwardaccel, double BackwardVelo, double BackwardAccel, double initialPosition) {
        servo_left = hwmap.get(Servo.class, name1);
        servo_right = hwmap.get(Servo.class,name2);
        servo_left.setDirection(Servo.Direction.REVERSE);
        servo_right.setDirection(Servo.Direction.REVERSE);
        this.name = name1 + " " + name2 + " ";
        this.endPosition = initialPosition;
        this.currentPosition = initialPosition;
        this.previousEndPosition = initialPosition + 100; // just guarantee that they are not equal
        this.forwardConstraint = new MotionConstraint(Forwardvelo,Forwardaccel,Forwardaccel);
        this.backwardContraint = new MotionConstraint(BackwardVelo,BackwardAccel, BackwardAccel);
        setPositionsSynced(initialPosition);
    }public double getTarget(){
       return this.endPosition;
    }
    protected void regenerate_profile() {
        if (endPosition > previousEndPosition) {
            profile_m = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(currentPosition, 0, 0),
                    new MotionState(endPosition, 0, 0),
                    forwardConstraint.max_velocity,
                    forwardConstraint.max_acceleration,
                    75
            );
        } else {
            profile_m = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(currentPosition, 0, 0),
                    new MotionState(endPosition, 0, 0),
                    backwardContraint.max_velocity,
                    backwardContraint.max_acceleration,
                    75
            );
        }

        timer.reset();
    }

    public void initAuto(HardwareMap hwMap) {

    }



    @Override
    public void periodic() {
        if (endPosition != previousEndPosition) {
            regenerate_profile();
        }
        previousEndPosition = endPosition;
        double current_target = profile_m.get(timer.seconds()).getX();
        setPositionsSynced(current_target);
     //   telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
       // telemetry.addData("current target", current_target);
      //  telemetry.update();
    }

    public boolean isBusy() {
        return timer.seconds() < profile_m.duration();
    }

    

    public void setPosition(double endPosition) {
        this.endPosition = endPosition;
    }

    public void shutdown() {

    }
    protected void setPositionsSynced(double pos) {
        servo_left.setPosition(1 - pos);
        servo_right.setPosition(pos);
    }
}