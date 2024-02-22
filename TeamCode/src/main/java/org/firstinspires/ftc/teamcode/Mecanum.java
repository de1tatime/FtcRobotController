package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Mecanum teleop (with an optional arcade mode)
 * * Left stick controls x/y translation.
 * * Right stick controls rotation about the z axis
 * * When arcade mode is enabled (press "a"), translation direction
 * becomes relative to the field as opposed to the robot. You can
 * reset the forward heading by pressing "x".
 */
@TeleOp(name = "Mecanum")
public class Mecanum extends OpMode {
    private Robot robot;
    private Controller controller;
    private boolean arcadeMode = false;
    private int gyroCalibratedCount = 0;

    private double mulPower = 0.5;

    private IMU imu         = null;      // Control/Expansion Hub IMU

    private Singleton context = ContextSingleton.getContext();
    private double lastHeading = context.getHeading();
    private ElapsedTime runtime = new ElapsedTime();



    private double targetHeading = context.getHeading();

    private Servo dronelaunch = null;

    Arm arm = new Arm(this);
    Claw claw       = new Claw(this);
    Wrist wrist = new Wrist(this);
    public final Boolean opModeIsActive() {
        return true;
    }

    public void waitRuntime(double sec) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < sec)) {
            telemetry.update();
        }
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.runUsingEncoders();
        controller = new Controller(gamepad1);
    }

    @Override
    public void init_loop() {
        controller.update();
        if (controller.AOnce()) {
            arcadeMode = ! arcadeMode;
        }
        telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "YES" : "no.");
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.update();
    }


    double drone_launcher_pos = 0.6;

    @Override
    public void loop() {
        controller.update();
        robot.loop();

        if (controller.XOnce()) {
            robot.resetHeading();
        }
        if (controller.AOnce()) {
            arcadeMode = !arcadeMode;
        }

        if(gamepad1.dpad_right){
            mulPower+=0.1;
            waitRuntime(0.2);
        }
        // This part of this code is to increase the speed of the motor a little.
        if(gamepad1.dpad_left){
            mulPower-=0.1;
            waitRuntime(0.2);
        }

        if (mulPower > 1) {
            mulPower = 1;
        }

        if (mulPower < 0.2) {
            mulPower = 0.2;
        }

        if(gamepad2.left_bumper) {
            wrist.wristUp();
            arm.moveToDegree(130);

        }

        if (gamepad1.x) {
            imu.resetYaw();
            lastHeading = 0.0;
            targetHeading = 0.0;
            waitRuntime(0.3);
        }

        if (gamepad2.x) {
            drone_launcher_pos = 1;
        }
        if (gamepad2.b) {
            drone_launcher_pos = 0;
        }
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Heading (reset: x)", robot.getHeadingDegrees());
        telemetry.update();

        final double x = Math.pow(controller.left_stick_x, 3.0);
        final double y = Math.pow(controller.left_stick_y, 3.0);

        final double rotation = Math.pow(controller.right_stick_x, 3.0);
        final double direction = Math.atan2(x, y) + (arcadeMode ? robot.getHeading() : 0.0);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double lf = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double rf = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double lr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double rr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        robot.setMotors(lf, lr, rf, rr);
    }
}