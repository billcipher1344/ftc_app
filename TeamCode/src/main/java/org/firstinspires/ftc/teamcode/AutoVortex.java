package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.modules.AccelerationIntegrator;
import org.firstinspires.ftc.teamcode.modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Precision;
import org.firstinspires.ftc.teamcode.modules.State;
import org.firstinspires.ftc.teamcode.modules.StateMachine;

import java.util.Locale;

@Autonomous
public class AutoVortex extends OpMode {
    private Precision precision;

    private HardwareVortex robot = new HardwareVortex();

    private DcMotor[] leftDrive, rightDrive, driveMotors;

    private ColorSensor color;

    private StateMachine main;

    //Parameters
    private double delay = 0;
    private boolean blue = true;
    private boolean cornerStart = false;
    private boolean beacon = true;
    private boolean park = false;


    private double heading = 0;

    //Globals
    private int times;
    private double intakeTime;

    private StateMachine shooter;

    private StateMachine pusher;

    @Override
    public void init() {
        precision = new Precision();

        robot.init(hardwareMap);
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new AccelerationIntegrator();
//
//        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
//        robot.imu.initialize(parameters);

        robot.limit.setPosition(HardwareVortex.LIMIT_OFF);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive = new DcMotor[] {
                robot.frontLeft,
                robot.backLeft
        };
        rightDrive = new DcMotor[] {
                robot.frontRight,
                robot.backRight
        };
        driveMotors = new DcMotor[] {
                robot.frontLeft,
                robot.frontRight,
                robot.backLeft,
                robot.backRight
        };

        shooter = new StateMachine(
                new State("off") {
                    @Override
                    public void run() {
                        robot.shooter.setPower(0);
                    }
                },
                new State("on") {
                    @Override
                    public void run() {
                        robot.shooter.setPower(HardwareVortex.SHOOTER_POWER);
                    }
                }
        ).start();

        pusher = new StateMachine(
                new State("stop") {
                    @Override
                    public void run() {
                        robot.pusherLeft.setPosition(0.5);
                        robot.pusherRight.setPosition(0.5);
                    }
                },
                new State("start") {
                    @Override
                    public void run() {
                        sendData("pushStart", time);
                        changeState("wait 1s");
                    }
                },
                new State("wait 1s") {
                    @Override
                    public void run() {
                        if (elapsedTime(getDouble("pushStart")) > 1) {
                            sendData("pushStart", time);
                            changeState("retract");
                        } else {
                            if (blue) {
                                robot.pusherLeft.setPosition(1);
                            } else {
                                robot.pusherRight.setPosition(1);
                            }
                        }
                    }
                },
                new State("retract") {
                    @Override
                    public void run() {
                        if (elapsedTime(getDouble("pushStart")) > 1) {
                            changeState("stop");
                        } else {
                            if (blue) {
                                robot.pusherLeft.setPosition(0);
                            } else {
                                robot.pusherRight.setPosition(0);
                            }
                        }
                    }
                }
        );

        main = new StateMachine(

                new State("stop") {
                    @Override
                    public void run() {
                        move(0);
                    }
                },

                new State("drive to vortex") {
                    @Override
                    public void run() {
                        if (elapsedTime(getDouble("start")) > delay) {
                            shooter.changeState("on");
                            sendData("shooter start", time);
                            if (cornerStart) {
                                changeState("drive to center line");
                            } else {
                                changeState("middle - reach vortex");
                            }
                        }
                    }
                },

                // MIDDLE SPECIFIC START
                new State("middle - reach vortex") {
                    @Override
                    public void run() {
                        if (reachedDestination(800, 2000, 0.4)) {
                            robot.resetEncoders();
                            changeState("turn a little left");
                        }
                    }
                },
                // MIDDLE SPECIFIC END

                // CORNER SPECIFIC START
                new State("drive to center line") {
                    @Override
                    public void run() {
                        if (reachedDestination(300, 2000, 0.4)) {
                            robot.resetEncoders();
                            changeState("turn to vortex");
                        }
                    }
                },

                new State("turn to vortex") {
                    @Override
                    public void run() {
                        if (turnedDegrees(-45 * boolToSign(blue), 3000, 0.8)) {
                            robot.resetEncoders();
                            changeState("corner - reach vortex");
                        }
                    }
                },

                new State("corner - reach vortex") {
                    @Override
                    public void run() {
                        if (reachedDestination(650, 3000, 0.5)) {
                            robot.resetEncoders();
                            changeState("turn a little left");
                        }
                    }
                },
                // CORNER SPECIFIC START END

                new State("turn a little left") {
                    @Override
                    public void run() {
                        //if (turnedDegrees(6, 1500, 0.45)) {
                            robot.resetEncoders();
                            robot.intake.setPower(1);
                            sendData("shooting time", time);
                            changeState("intake");
                        //}
                    }
                },

                new State("intake") {
                    @Override
                    public void run() {
                        if (elapsedTime(intakeTime) > 1.5) {
                            if (++times > 2) {
                                sendData("shooting time", time);
                                changeState("shoot the ball");
                            } else {
                                robot.intake.setPower(0);
                                intakeTime = time;
                                changeState("stop intaking");
                            }
                        }
                    }
                },

                new State("stop intaking") {
                    @Override
                    public void run() {
                        if (elapsedTime(intakeTime) > 0.5) {
                            robot.intake.setPower(1);
                            intakeTime = time;
                            changeState("intake");
                        }
                    }
                },

                new State("shoot the ball") {
                    @Override
                    public void run() {
                        if (elapsedTime(getDouble("shooting time")) > 1.0) {
                            move(0);
                            shooter.changeState("off");
                            robot.intake.setPower(0);
                            changeState("turn a little right");
                        }
                    }
                },

                new State("turn a little right") {
                    @Override
                    public void run() {
                        //if (turnedDegrees(-6, 1500, 0.45)) {
                            robot.resetEncoders();
                            if (beacon) {
                                changeState("turn to wall");
                            }
                            else {
                                changeState("stop");
                            }
                        //}
                    }
                },

                // STUFF UP TO THIS WORKS

                new State("turn to wall") {
                    @Override
                    public void run() {
                        if (turnedDegrees(-50 * boolToSign(blue), 2250, 0.8)) {
                            robot.resetEncoders();
                            changeState("drive to wall");
                        }
                    }
                },

                new State("drive to wall") {
                    @Override
                    public void run() {
                        if (reachedDestination(2460, 3500, 0.45)) {
                            robot.resetEncoders();
                            changeState("align with wall");
                        }
                    }
                },

                new State("align with wall") {
                    @Override
                    public void run() {
                        if (turnedDegrees(45 * boolToSign(blue), 2250, 0.6)) {
                            robot.resetEncoders();
                            sendData("white line", time);
                            changeState("white line");
                        }
                    }
                },

                new State("white line") {
                    @Override
                    public void run() {
                        if (robot.lightLeft.getLightDetected() >= 0.2 || robot.lightRight.getLightDetected() >= 0.3) {
                            robot.resetEncoders();
                            sendData("run time", time);
                            move(0);
                            changeState("wait 1 second");
                        } else if (elapsedTime(getDouble("white line")) < 1.5){
                            if (getBool("2nd time")) {
                                move(.25);
                            } else {
                                move(-.25);
                            }
                        }
                        else {
                            move(0);
                            if (getBool("2nd time")) {
                                changeState("drive to other beacon");
                            }
                            else {
                                changeState("stop");
                            }
                        }
                    }
                },

                new State("wait 1 second") {
                    @Override
                    public void run() {
                        if (elapsedTime(getDouble("run time")) > 1.0) {
                            double avgEncoder = 0;
                            for (DcMotor motor : driveMotors) {
                                avgEncoder += motor.getCurrentPosition();
                            }
                            avgEncoder /= 4;
                            sendData("encoder", avgEncoder);
                            robot.resetEncoders();
                            precision.reset();
                            changeState("drive back to line");
                        }
                    }
                },

                new State("drive back to line") {
                    @Override
                    public void run() {
                        if (reachedDestination((boolToSign(blue) * 150) - ((int) getDouble("encoder")), 1500, .25)) {
                            changeState("detect color");
                        }
                    }
                },

                new State("detect color") {
                    @Override
                    public void run() {
                        robot.resetEncoders();
                        precision.reset();
                        if (isBlue(color) == blue) { // Beacon match
                            pusher.changeState("start");
                            changeState("drive to other beacon");
                        } else {
                            changeState("drive to other side");
                        }
                    }
                },

                new State("drive to other side") {
                    @Override
                    public void run() {
                        if (reachedDestination(200, 1250, .225)) {
                            pusher.changeState("start");
                            changeState("drive to other beacon");
                        }
                    }
                },

                new State("drive to other beacon") {
                    @Override
                    public void run() {
                        if (pusher.getActiveState().equals("stop") && getBool("2nd time") && reachedDestination(250, 1000, .3)) {
                            if (getBool("2nd time")) {
                                changeState("stop");
                            }
                            else {
                                sendData("2nd time", true);
                                sendData("white line", time+1);
                                changeState("white line");
                            }
                        }
                    }
                }
        ).start();
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            delay = 0;
        }
        else if (gamepad1.b) {
            delay = 2.5;
        }
        else if (gamepad1.x) {
            delay = 5;
        }
        else if (gamepad1.y) {
            delay = 7.5;
        }

        if (gamepad1.left_bumper) {
            blue = true;
        }
        else if (gamepad1.right_bumper) {
            blue = false;
        }

        if (gamepad1.dpad_up){
            park = true;
        }

        if (gamepad1.dpad_down){
            park = false;
        }

        if (gamepad1.dpad_right){
            cornerStart = false;
        }

        else if (gamepad1.dpad_left){
            cornerStart = true;
        }

        telemetry.addData("Corner Start", cornerStart);
        telemetry.addData("Park", park);
        telemetry.addData("Beacon", beacon);
        telemetry.addData("Delay", delay);
        telemetry.addData("Side", blue?"Blue":"Red");
        color = blue?robot.colorRight:robot.colorLeft;
        updateSensors();
    }

    @Override
    public void start() {
        main.changeState("drive to vortex");
    }

    @Override
    public void loop() {
        telemetry.addData("avgEncoder", main.getDouble("encoder"));
        updateSensors();
        robot.generateTelemetry(telemetry, true);
    }

    @Override
    public void stop() {
        main.changeState("stop");
        main.stop();
        shooter.changeState("off");
        shooter.stop();
        pusher.changeState("stop");
        pusher.stop();
    }

    ///////////////
    // FUNCTIONS //
    ///////////////

    private void move(double power) {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
    }

    private double elapsedTime(double startTime) {
        return time - startTime;
    }

    private boolean reachedDestination(int target, int timeout, double power) {
        return precision.destinationReached(driveMotors, power, Math.signum(power)*0.12, target, 2.0, 10, timeout);
    }

    private boolean turnedDegrees(double degrees, int timeout, double power) {
        //return precision.angleTurned(leftDrive, rightDrive, heading, power, 0.4, degrees, 2.0, 0.75, timeout);
        return precision.distanceTurned(leftDrive, rightDrive, power, 0.42, (int)((-degrees/180.0)*1200), 2.0, 20, timeout);
    }


    private void updateSensors() {
        telemetry.addData("State", main.getActiveState());
        telemetry.addData("Color", "R: %d G: %d B: %d A: %d", color.red(), color.green(), color.blue(), color.alpha());
        telemetry.addData("Light", "Left: %f Right: %f", robot.lightLeft.getLightDetected(), robot.lightRight.getLightDetected());
//        Orientation angles = robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//        heading = angles.firstAngle;
//        telemetry.addData("IMU", "heading: %s", String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, heading))));
    }

    private boolean isBlue(ColorSensor colorSensor) {
        int red = colorSensor.red();
        int blue = colorSensor.blue();

        if (red >= 100 && blue >= red + 50) return true;
        if (Math.abs(red - blue) <= 10) return false;
        if (red - blue >= 100) return true;

        return false;
    }

    private int boolToSign(boolean bool) {
        return bool?1:-1;
    }
}
