package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.PIDController;

@Autonomous
public class PIDTest extends OpMode{
    DcMotor motor;
    PIDController PID;
    int target = 5000;

    public void init(){
        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PID = new PIDController(target, 250, this);
        PID.setPIDParam(PIDController.PID_LIMITHIGH, 1);
        PID.setPIDParam(PIDController.PID_LIMITLOW, -1);
        PID.setPIDParam(PIDController.PID_KP, 0.0001F);
        PID.setPIDParam(PIDController.PID_KI, 0.000005F);
        PID.setPIDParam(PIDController.PID_KD, 0.0005F);
        PID.setTelemetry("motor", telemetry);
    }

    public void loop(){
        motor.setPower(PID.doPID(motor.getCurrentPosition()));
        telemetry.addData("Power", motor.getPower());
    }
}
