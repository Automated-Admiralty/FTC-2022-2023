package org.firstinspires.ftc.teamcode.Common.RobotHardware;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;




public class RobotHardware {

    public HardwareMap hardwareMap;
    //Drivetrain
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackRightMotor;
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtBackLeftMotor;

    //Lift
    public DcMotorEx LeftSlide;
    public DcMotorEx RightSlide;

    //Intake Motor
    public DcMotorEx Intake;

    //Outake + Claw
    public Servo Claw;
    public Servo ClawPivotLeft;
    public Servo ClawPivotRight;

    public Servo MiniArmLeft;
    public Servo MiniArmRight;

    public IMU imu;


    public RobotHardware(HardwareMap hardwareMap){


        dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "dtFrontRightMotor");
        dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "dtFrontLeftMotor");
        dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "dtBackRightMotor");
        dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "dtBackLeftMotor");

        dtFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftSlide = hardwareMap.get(DcMotorEx.class, "LeftSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RightSlide");
        LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        Claw = hardwareMap.get(Servo.class, "Claw");
        ClawPivotLeft = hardwareMap.get(Servo.class,"ClawPivotLeft");
        ClawPivotRight = hardwareMap.get(Servo.class,"ClawPivotRight");
        ClawPivotRight.setDirection(Servo.Direction.REVERSE);
        MiniArmLeft = hardwareMap.get(Servo.class,"MiniArmLeft");
        MiniArmRight = hardwareMap.get(Servo.class,"MiniArmRight");
        MiniArmRight.setDirection(Servo.Direction.REVERSE);
        MiniArmLeft.setDirection(Servo.Direction.REVERSE);


        // imu = hardwareMap.get(IMU.class, "imu");

    }
}
