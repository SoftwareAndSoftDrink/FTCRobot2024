package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDeviceGroup;
import org.firstinspires.ftc.teamcode.hardware.PIDFDcMotor;
import org.firstinspires.ftc.teamcode.modules.core.MotorPowerUpdater;
import org.firstinspires.ftc.teamcode.modules.core.Module;

/**
 * This module controls the arm, which rotates the intake mechanism around the robot
 */
@Config
public class Arm extends Module implements MotorPowerUpdater {
    /**
     * Contains both motors used to move the arm <br>
     * IMPORTANT: To ensure that the arm doesn't tear itself apart,
     * make sure to set both motors to the same power value <br>
     * ALSO IMPORTANT: All of the PID logic uses the left arm position as input, don't use the
     * right arm motor for comparisons
     */
    private final ConditionalHardwareDeviceGroup motors;
    /**
     * The name of the left arm motor
     */
    public static final String LEFT_ARM_MOTOR_NAME = "Left Arm Motor";
    public static final String RIGHT_ARM_MOTOR_NAME = "Right Arm Motor";

    /**
     * The position switch.  This is used to detect where the arm's 'zero position' is whenever the arm resets
     */
    private final ConditionalHardwareDevice<TouchSensor> positionSwitch;

    public static final String POSITION_SWITCH_NAME = "Position Switch";

    /**
     * The offset, in ticks, our intended 'zero position' is from the motor's actual 'zero position'
     * @deprecated this has been abandoned in favor of {@link #baseRotation} for use in resetting the arm
     */
    private int baseOffsetTicks;
    /**
     * The initial offset the arm should have when this module is initialized
     * @see #baseOffsetTicks
     * @deprecated {@link #baseOffsetTicks} has been abandoned in favor of {@link #baseRotation} for use in resetting the arm
     */
    private static final int DEFAULT_OFFSET_TICKS = 190;

    /**
     * Are the motors active?
     */
    private boolean active = false;

    /**
     * Encoder resolution for the 5203 117 RPM DC Motors used by the arm <br>
     * This is the amount of encoder ticks that represent one full revolution of the motor
     */
    private static final double ARM_ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28);
    /**
     * The factor used to convert motor ticks to degrees of rotation.
     * The encoder resolution maps to 360 degrees of rotation, and the arm uses a 5:1 gear ratio.
     */
    private static final double TICKS_TO_DEGREES = ARM_ENCODER_RESOLUTION * 5 / 360;

    /**
     * The PID controller used to move the arm to its target position
     */
    private final PIDController controller;

    /**
     * The coefficients for the arm's PIDF controller
     */
    @Config
    public static class ArmConfig {
        /**
         * The proportional coefficient
         */
        public static double P_COEF = 0.006;
        /**
         * The integral coefficient
         */
        public static double I_COEF = 0.0002;
        /**
         * The derivative coefficient
         */
        public static double D_COEF = 0.0003;
        /**
         * The feedforward coefficient <br>
         * @implNote Instead of using ftclib's PIDFController, we use their
         * PIDController and add a custom feedforward
         */
        public static double F_COEF = 0.15;
        /**
         * How many motor ticks the controller needs to be within the target to be considered
         * to have arrived
         */
        public static double TOLERANCE = 20; // the current arm PID usually gets around 10 ticks away from the target position
    }

    /*
     * Preset arm rotations for certain events during play
     */
    public static double ARM_ROTATION_INTAKE = -18.5;
    public static double ARM_ROTATION_MOVING = 0;
    public static double ARM_ROTATION_FRONT_SCORING = 65;
    public static double ARM_ROTATION_BACK_SCORING = 90;
    public static double ARM_ROTATION_HANG_LVL1_SETUP = 40;
    public static double ARM_ROTATION_HANG_LVL2_SETUP = 90;
    public static double ARM_ROTATION_HANG_LVL2_GRAB = 115;
    public static double ARM_ROTATION_HANG_LVL2_PULL = -20;

    /**
     * The internal 'base' rotation (in degrees) which all outside arm angles are relative to,
     * before the arm encoders are reset mid-match
     */
    public static double ARM_ROTATION_INTERNAL_BASE_INITIAL = 35;
    /**
     * The internal 'base' rotation (in degrees) which all outside arm angles are relative to,
     * after the arm encoders are reset mid-match
     */
    public static double ARM_ROTATION_INTERNAL_BASE_RESET = 40;
    private double baseRotation = ARM_ROTATION_INTERNAL_BASE_INITIAL;

    /**
     * Configures a motor to be used to control the arm
     * @param m either the left or right arm motor
     */
    private static void configureMotor(DcMotor m) {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Initializes the arm with the current rotation as '0 degrees'
     * @param registrar The op mode creating this arm
     */
    public Arm(OpMode registrar) {
        super(registrar);
        motors = new ConditionalHardwareDeviceGroup(registrar.hardwareMap);

        motors.tryLoadDevice(PIDFDcMotor.class, LEFT_ARM_MOTOR_NAME);
        motors.tryLoadDevice(PIDFDcMotor.class, RIGHT_ARM_MOTOR_NAME);

        motors.executeIfAllAreAvailable(() -> {
            final PIDFDcMotor leftMotor = motors.requireLoadedDevice(PIDFDcMotor.class, LEFT_ARM_MOTOR_NAME);
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            configureMotor(leftMotor);
            final PIDFDcMotor rightMotor = motors.requireLoadedDevice(PIDFDcMotor.class, RIGHT_ARM_MOTOR_NAME);
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            configureMotor(rightMotor);
        }, () -> getTelemetry().addLine("Failed to load arm motors!"));
        active = true;

        controller = new PIDController(ArmConfig.P_COEF, ArmConfig.I_COEF, ArmConfig.D_COEF);

        controller.setPID(ArmConfig.P_COEF, ArmConfig.I_COEF, ArmConfig.D_COEF);
        controller.setTolerance(ArmConfig.TOLERANCE);

        positionSwitch = ConditionalHardwareDevice.tryGetHardwareDevice(registrar.hardwareMap, TouchSensor.class, POSITION_SWITCH_NAME);
        baseOffsetTicks = 0;
    }

    /**
     * Checks if the motors are powered or in free-fall
     * @return true if the motors are active, false otherwise
     */
    public boolean isActive() {
        return active;
    }

    public void activate() {
        if (active || !motors.areAllDevicesAvailable()) { return; }
//        motors.requireLoadedDevice(DcMotor.class, RIGHT_ARM_MOTOR_NAME).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        active = true;
    }
    public void deactivate() {
        if (!active || !motors.areAllDevicesAvailable()) { return; }
        DcMotor left = motors.requireLoadedDevice(DcMotor.class, RIGHT_ARM_MOTOR_NAME);
        DcMotor right = motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME);
//        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left.setPower(0);
        right.setPower(0);
        active = false;
    }

    /**
     * Checks if this module is connected to the hardware it requires
     *
     * @return false if the module cannot change the state of the hardware, true otherwise
     */
    @Override
    public boolean isConnected() {
        return motors.areAllDevicesAvailable();
    }

    /**
     * Ensures that the module is in a safe state for other modules to operate.
     * Between calling this method and calling any other method on this module that modifies
     * hardware devices, the module is guaranteed to not damage itself or anything else when
     * other modules modify hardware state
     */
    @Override
    public void ensureSafety() {
        // TODO find a safer alternative to rotateArmTo
        this.rotateArmTo(ARM_ROTATION_MOVING);
    }

    /**
     * Sets the target position for the PIDF controller
     * @param targetPosition the target position, in ticks
     */
    private void setTargetPosition(int targetPosition) {
        controller.setSetPoint(targetPosition);
    }

    /**
     * Sets the target rotation for the arm
     * @param rotation The desired rotation in degrees
     */
    public void setTargetRotation(double rotation) {
        // convert given rotation (relative to a where the arm is parallel to the ground)
        // to the actual rotation (relative to the arm's starting position)
        setTargetRotationAbsolute(rotation + baseRotation);
    }

    public void setTargetRotationAbsolute(double rotation) {
        // 360 degrees maps to 1 rotation, with a 5:1 gear ratio
        setTargetPosition((int)(rotation * TICKS_TO_DEGREES) + baseOffsetTicks);
    }

    /**
     * @see #setTargetRotationAbsolute(double)
     */
    public double getCurrentRotationAbsolute() {
        return (motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME).getCurrentPosition() - baseOffsetTicks) / TICKS_TO_DEGREES;
    }

    public double getCurrentRotation() {
        return getCurrentRotationAbsolute() - baseRotation;
    }

    public boolean isMoving() {
        return Math.abs(motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME).getPower()) > calculateFeedForward();
    }

    private double calculateFeedForward() {
        return ArmConfig.F_COEF * Math.cos(Math.toRadians(getCurrentRotation()));
    }

    /**
     * Sets the motor powers to the current result of the PIDF algorithm
     */
    @Override
    public void updateMotorPowers() {
        if (!active) { return; }
        motors.executeIfAllAreAvailable(() -> {
            controller.setPID(ArmConfig.P_COEF, ArmConfig.I_COEF, ArmConfig.D_COEF);
            controller.setTolerance(ArmConfig.TOLERANCE);
            final DcMotor leftMotor = motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME);
            final DcMotor rightMotor = motors.requireLoadedDevice(DcMotor.class, RIGHT_ARM_MOTOR_NAME);
            // use one encoder for safety and apply the same power to both motors
            double power = controller.calculate(leftMotor.getCurrentPosition());
            final double feedForward = calculateFeedForward();
            // only apply feedforward if we arrived (so it does its job) it is in the direction the arm needs to move
            if (controller.atSetPoint() || Math.signum(controller.getPositionError()) == Math.signum(feedForward)) {
                power += feedForward;
            }
            leftMotor.setPower(power);
            rightMotor.setPower(power);
            getTelemetry().addData("Arm power", power);
        });
    }

    /**
     * Checks if the motors need to be updated
     * @return true if {@link #updateMotorPowers()} needs to be called, false otherwise
     */
    @Override
    public boolean isUpdateNecessary() {
        final DcMotor leftMotor = motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME);
        // check manually if we are within tolerance since the controller only gets updated with
        // the motor's current position when calculate() is called, which should only happen in
        // updateMotorPowers()
        if (Math.abs(leftMotor.getCurrentPosition() - controller.getSetPoint()) < ArmConfig.TOLERANCE) {
            // we are at our target position
            // since we will return false, the caller won't call updateMotorPowers(), so
            // we need to stop the motors ourselves
            final double feedForwardPower = calculateFeedForward(); // so the motors can still support themselves
            leftMotor.setPower(feedForwardPower);
            try {
                // if the right motor is connected, stop it as well
                final DcMotor rightMotor = motors.requireLoadedDevice(DcMotor.class, RIGHT_ARM_MOTOR_NAME);
                rightMotor.setPower(feedForwardPower);
            }
            catch (NullPointerException ignored) {
                // if the right motor is disconnected, we don't have to do anything
            }
            return false; // no update necessary
        }
        return true; // we are not at our target position -- an update is necessary
    }

    /**
     * Checks sensors to ensure that the arm is in the correct position
     */
    public boolean monitorPositionSwitch() {
        assert motors.areAllDevicesAvailable();
        if (!positionSwitch.isAvailable() || !positionSwitch.requireDevice().isPressed()) {
            return false;
        }
        DcMotor leftMotor = motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME);
        DcMotor rightMotor = motors.requireLoadedDevice(DcMotor.class, RIGHT_ARM_MOTOR_NAME);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        baseRotation = ARM_ROTATION_INTERNAL_BASE_RESET;
//      baseOffsetTicks = DEFAULT_OFFSET_TICKS;
        return true;
    }

    /**
     * Rotates the arm to the specified rotation, not returning until the arm has finished rotating
     * @param rotation the target rotation
     * @deprecated This method is unsafe, as it
     * a) has the potential to get stuck in an infinite loop if the arm never reaches its destination,
     * which could lead to a frozen opmode that cannot be gracefully stopped, and
     * b) does not update any other PID loops that could be running, which could potentially damage
     * the robot.
     * Currently, there is no way to resolve these problems without moving the method outside of
     * this class.
     * Do not use this method; instead, write your own loop that takes both of these factors into
     * consideration.
     */
    @Deprecated
    public void rotateArmTo(double rotation) {
        setTargetRotation(rotation);
        do {
            updateMotorPowers();
            monitorPositionSwitch();
        } while (!controller.atSetPoint());
    }

    @Override
    public void log() {
        Telemetry telemetry = getTelemetry();
        if (!motors.areAllDevicesAvailable()) { return; }
        final DcMotor leftMotor = motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME);
        final DcMotor rightMotor = motors.requireLoadedDevice(DcMotor.class, RIGHT_ARM_MOTOR_NAME);

        telemetry.addData("[Arm] Is Arm Active", isActive());
        telemetry.addData("[Arm] Current left motor position", leftMotor.getCurrentPosition());
        telemetry.addData("[Arm] Current right motor position", rightMotor.getCurrentPosition());
        telemetry.addData("[Arm] Target motor position", controller.getSetPoint());
        telemetry.addData("[Arm] Current arm rotation", getCurrentRotation());
    }
}
