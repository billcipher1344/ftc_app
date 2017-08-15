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
public class AutoVortexShoot extends OpMode {
    private Precision precision;

    private HardwareVortex robot = new HardwareVortex();

    private DcMotor[] leftDrive, rightDrive, driveMotors;

    private StateMachine main;

    //Parameters
    private double delay = 0;

    //Stuff
    private int times;
    private double intakeTime;
    private int[] startPos;
    private double startTime;

    private double errorzz = 0;
    private int timezz = 0;

    private StateMachine shooter;

    @Override
    public void init() {
        precision = new Precision(telemetry);

        robot.init(hardwareMap);
        robot.resetEncoders();
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

        times = 0;

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
                            startPos = new int[] {
                                    robot.frontLeft.getCurrentPosition(),
                                    robot.frontRight.getCurrentPosition(),
                                    robot.backLeft.getCurrentPosition(),
                                    robot.backRight.getCurrentPosition()
                            };
                            startTime = time;
                            changeState("reach vortex");
                        }
                    }
                },

                new State("reach vortex") {
                    @Override
                    public void run() {
                        if (reachedDestination(800, 2000, 0.3)) {
                            move(0);
                            robot.intake.setPower(1);
                            intakeTime = time;
                            changeState("intake");
                        }
                    }
                },

                new State("intake") {
                    @Override
                    public void run() {
                        if (elapsedTime(intakeTime) > 1.5) {
                            if (++times > 10) {
                                sendData("shooting time", time);
                                changeState("shoot the ball");
                            }
                            else {
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
                        if (elapsedTime(getDouble("shooting time")) > 2.0) {
                            move(0);
                            shooter.changeState("off");
                            robot.intake.setPower(0);
                            changeState("stop");
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
        telemetry.addData("Delay", delay);
    }

    @Override
    public void start() {
        main.changeState("drive to vortex");
    }

    @Override
    public void loop() {
        telemetry.addData("State", main.getActiveState());
        telemetry.addData("timezz", timezz);
        telemetry.addData("errorzz", errorzz);
        robot.generateTelemetry(telemetry, true);
    }

    @Override
    public void stop() {
        main.stop();
        shooter.changeState("off");
        shooter.stop();
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

    private boolean reachedDestination(int distance, int timeoutms, double maxPower) {
        double minPower = Math.signum(maxPower)*0.12;
        double marginOfError = 10;
        double flatness = 2.0;
        int distanceElapsed = 0;
        for (int i = 0; i < driveMotors.length; i++) {
            distanceElapsed += driveMotors[i].getCurrentPosition() - startPos[i];
        }

        distanceElapsed /= driveMotors.length;

        int error = distance - distanceElapsed;
        if (Math.abs(error) > Math.abs(distance)) {
            error = (int)Math.signum(error) * Math.abs(distance);
        }
        errorzz = error;

        int elapsedTime = (int) (elapsedTime(startTime)*1000);

        timezz = elapsedTime;

        if (elapsedTime > timeoutms) {
            for (DcMotor motor : driveMotors) {
                motor.setPower(0);
            }
            return true;
        }

        // y = a(x-h)^n + k
        // a = (y-k)/(x-h)^n
        // power = y
        // x = error
        // k = maxPower
        // h = distance
        // a = (minPower-maxPower)/(0-distance)^flatness
        distance -= Math.signum(distance)*marginOfError;
        double power = Math.pow(Math.abs(error - distance), flatness);
        power *= -Math.abs(minPower - maxPower) / Math.pow(-distance, flatness);
        power += Math.abs(maxPower);
        power *= Math.signum(error);

        if (Math.abs(power) > maxPower) {
            power = Math.signum(power)*maxPower;
        }
        else if (Math.abs(power) < minPower) {
            power = Math.signum(power)*minPower;
        }

        for (DcMotor motor : driveMotors) {
            if (Math.abs(error) < Math.abs(marginOfError)) {
                motor.setPower(0);
            }
            else {
                motor.setPower(power);
            }
        }

        return false;
    }
}
