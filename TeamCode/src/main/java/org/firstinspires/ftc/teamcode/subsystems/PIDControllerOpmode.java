package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="linear slides pid tuning")
public class PIDControllerOpmode extends OpMode {
    ElapsedTime timer = new ElapsedTime();

    private PIDController controllerRight;
    private PIDController controllerLeft;
    public static double Lp = 0.015, Li=0, Ld = 0.0003;
    public static double Rp = 0.015, Ri=0, Rd = 0.0003;
    public static double f =0.1;
    public static int targetPos = 0;
    public int left_arm_pos;
    public int right_arm_pos;
    private DcMotorEx armmotor1;
    private DcMotorEx armmotor2;
    int count = 0;
    int count2 = 0;


    @Override
    public void init(){
        controllerLeft = new PIDController(Lp,Li,Ld);
        controllerRight = new PIDController(Rp,Ri,Rd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armmotor1 = hardwareMap.get(DcMotorEx.class, "leftSlides");
        armmotor2 = hardwareMap.get(DcMotorEx.class, "rightSlides");
        count = 1;
        count2 = 1;
        time = 0;
        timer.reset();






    }

    @Override
    public void loop(){
        if (count == 1){
            telemetry.addData("target", targetPos);
            telemetry.addData("currentLeft", left_arm_pos);
            telemetry.addData("currentRight", right_arm_pos);
            telemetry.update();
            sleep(5000);
            timer.startTime();
            count++;
        }
        else {
            controllerLeft.setPID(Lp, Li, Ld);
            controllerRight.setPID(Rp, Ri, Rd);
            left_arm_pos = (armmotor1.getCurrentPosition());
            right_arm_pos = (armmotor2.getCurrentPosition());
            double leftPID = controllerLeft.calculate(left_arm_pos, targetPos);
            double rightPID = controllerRight.calculate(right_arm_pos, targetPos);
            double leftPower = leftPID + f;
            double rightPower = rightPID + f;
            armmotor1.setPower(leftPower);
            armmotor2.setPower(rightPower);
            telemetry.addData("target", targetPos);
            telemetry.addData("currentLeft", left_arm_pos);
            telemetry.addData("currentRight", right_arm_pos);
            telemetry.addData("left power", leftPower);
            telemetry.addData("right power", rightPower);
            telemetry.addData("timer", timer.milliseconds());
            telemetry.update();
            if(Math.abs((int)(leftPower)) == 0){
                if(count2 == 1) {
                    double time = timer.milliseconds();
                    telemetry.addData("timeELAPSEDHJBSDBF", time);
                    telemetry.update();
                    count2++;
                }
            }
        }




    }
    public void sleep(long millisec){
        try {
            Thread.sleep( millisec);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}


