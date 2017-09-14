package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Kit Caldwell on 9/8/2017.
 */

public class TwoMotorTest extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    Gamepad pad1 = new Gamepad();

    public void init(){
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop(){

        leftMotor.setPower(pad1.left_stick_y);
        rightMotor.setPower(pad1.left_stick_y);
    }
}
