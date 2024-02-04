package org.firstinspires.ftc.teamcode.Common.Tools;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDCONTROLLERTOOL {
    private PIDController controller;
    public double p, i , d,f;
    public int target;
    public DcMotorEx Motor;
    public double ticks_in_deg;
    public PIDCONTROLLERTOOL(double Pvalue, double Ivalue, double Dvalue, double Fvalue,double ticks_in_deg, DcMotorEx Motor){
        this.p = Pvalue;
        this.i = Ivalue;
        this.d = Dvalue;
        this.f = Fvalue;
        this.Motor = Motor;
        this.ticks_in_deg = ticks_in_deg;
        controller = new PIDController(p,i,d);

    }

    public double calculatePid(int target){
        controller.setPID(p,i,d);
        double pid = controller.calculate(Motor.getCurrentPosition(), target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_deg)) * f;
        double power = pid + ff;
        return power;

    }

}
