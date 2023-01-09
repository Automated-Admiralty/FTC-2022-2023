package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TeleOpMap {
    //Public opMode members
    public DcMotorEx motorFR = null;
    public DcMotorEx motorFL = null;
    public DcMotorEx motorBR = null;
    public DcMotorEx motorBL = null;
    public DcMotorEx motorArmLeft = null;
    public DcMotorEx motorArmRight = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public TeleOpMap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorFR  = hwMap.get(DcMotorEx.class, "frontRightMotor");
        motorFL = hwMap.get(DcMotorEx.class, "frontLeftMotor");
        motorBR  = hwMap.get(DcMotorEx.class, "backRightMotor");
        motorBL = hwMap.get(DcMotorEx.class, "backLeftMotor");
        motorArmRight = hwMap.get(DcMotorEx.class, "RightArm");
        motorArmLeft = hwMap.get(DcMotorEx.class, "LeftArm");
        //Set Motor Direction
        motorFR.setDirection(DcMotorEx.Direction.FORWARD);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBR.setDirection(DcMotorEx.Direction.FORWARD);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorArmLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorArmRight.setDirection(DcMotorEx.Direction.FORWARD);

        // Set all motors to zero power
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);

        //Set ZERO POWER BEHAVIOR
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorArmRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorArmLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.
        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorArmRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorArmLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }
}
