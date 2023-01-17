package org.firstinspires.ftc.teamcode.ArmPIDFControler;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp
public class PIDF_Arm extends OpMode{

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    public final double ticks_in_degree = 567.7/180;

    private DcMotorEx arm_motor_Left;
    private DcMotorEx arm_motor_Right;

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor_Left = hardwareMap.get(DcMotorEx.class, "left slide");
        arm_motor_Right = hardwareMap.get(DcMotorEx.class, "right slide");
        arm_motor_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor_Left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i , d);
        int arm_pos_Left = -(arm_motor_Left.getCurrentPosition());
        int arm_pos_Right =-(arm_motor_Right.getCurrentPosition());
        double pidLeft = controller.calculate(arm_pos_Left, target);
       // double pidRight = controller.calculate(arm_pos_Right, target);
        double ff = Math.cos(Math.toRadians(target/ ticks_in_degree)) * f;

        double powerLeft = pidLeft + ff;
        double powerRight = pidLeft + ff;

        arm_motor_Right.setPower(powerRight);
        arm_motor_Left.setPower(powerLeft);

        telemetry.addData("posLeft", arm_pos_Left);
        telemetry.addData("posRight", arm_pos_Right);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
