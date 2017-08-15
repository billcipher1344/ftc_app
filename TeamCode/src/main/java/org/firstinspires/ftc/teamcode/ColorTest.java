package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorTest extends OpMode {
    private HardwareVortex robot = new HardwareVortex();

    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){
        telemetry.addLine("L ")
                .addData("R", robot.colorLeft.red())
                .addData("G", robot.colorLeft.green())
                .addData("B", robot.colorLeft.blue())
                .addData("A", robot.colorLeft.alpha());
        telemetry.addData("L Prediction", isBlue(robot.colorLeft)?"Blue":"Red");

        telemetry.addLine("R ")
                .addData("R", robot.colorRight.red())
                .addData("G", robot.colorRight.green())
                .addData("B", robot.colorRight.blue())
                .addData("A", robot.colorRight.alpha());
        telemetry.addData("R Prediction", isBlue(robot.colorRight)?"Blue":"Red");
    }

    private boolean isBlue(ColorSensor colorSensor) {
        int red = colorSensor.red();
        int blue = colorSensor.blue();

        if (red >= 100 && blue >= red + 50) return true;
        if (Math.abs(red - blue) <= 10) return false;
        if (red - blue >= 100) return true;

        return false;
    }
}
