package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.PIDFDcMotor;
import org.firstinspires.ftc.teamcode.modules.core.MotorPowerUpdater;
import org.firstinspires.ftc.teamcode.modules.core.Module;

/**
 * This module controls the linear slide, which moves the intake closer and farther
 * away from the robot.
 */

@Config
public class LinearSlide extends Module implements MotorPowerUpdater {
    private final ConditionalHardwareDevice<PIDFDcMotor> motor;
    public static final String SLIDE_MOTOR_NAME = "Slide Motor";
    /**
     * Encoder resolution for the 5203 312 RPM DC Motors used by the arm
     */
    private static final double SLIDE_ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);
    private static final int SLIDE_MAX_EXTENSION_TICKS = 2500;

    /**
     * The maximum absolute power the slide can have and still be considered 'idle.'
     * If the motor is being set to use more than this amount of power, the arm will be
     * considered to be moving.
     * @see #isMoving()
     */
    public static double IDLE_POWER_THRESHOLD = 0.05;

    @Config
    public static class SlideConfig {
        public static double P_COEF = 0.01;
        public static double I_COEF = 0;
        public static double D_COEF = 0.0003;
        public static double F_COEF = 0;
        public static double TOLERANCE = 25;
    }

    public static double SLIDE_HEIGHT_INTAKE = 0.7000;
    public static double SLIDE_HEIGHT_MOVING = 0.0120;
    public static double SLIDE_HEIGHT_SCORING_HIGH = 1;
    public static double SLIDE_HEIGHT_SCORING_LOW = 0.5;
    public static double SLIDE_HEIGHT_HANG_LVL1 = 0.5000;
    public static double SLIDE_HEIGHT_HANG_LVL2 = 0.0100;

    public LinearSlide(OpMode registrar) {
        this(registrar, true);
    }
    public LinearSlide(OpMode registrar, boolean resetPosition) {
        super(registrar);
        motor = ConditionalHardwareDevice.tryGetHardwareDevice(registrar.hardwareMap, PIDFDcMotor.class, SLIDE_MOTOR_NAME);

        motor.runIfAvailable(m -> {
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            if (resetPosition) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            m.setPIDF(SlideConfig.P_COEF, SlideConfig.I_COEF, SlideConfig.D_COEF, SlideConfig.F_COEF);
            m.setTolerance(SlideConfig.TOLERANCE);
        });
    }

    /**
     * Checks if this module is connected to the hardware it requires
     *
     * @return false if the module cannot change the state of the hardware, true otherwise
     */
    @Override
    public boolean isConnected() {
        return motor.isAvailable();
    }

    /**
     * Ensures that the module is in a safe state for other modules to operate.
     * Between calling this method and calling any other method on this module that modifies
     * hardware devices, the module is guaranteed to not damage itself or anything else when
     * other modules modify hardware state
     */
    @Override
    public void ensureSafety() {
        moveSlideTo(SLIDE_HEIGHT_MOVING);
    }

    /**
     * Resets the slide motor so that the position it is in right now
     * will be treated as its 'zero position'
     */
    public void resetZeroHeightPosition() {
        motor.runIfAvailable(m -> {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
    }

    private void setTargetPosition(int targetPosition) {
        motor.runIfAvailable(m -> { m.setSetPoint(targetPosition); });
    }

    /**
     * Sets the target height of the robot
     * @param height The desired height; values range from 0 for fully retracted to 1 for fully extended
     */
    public void setTargetHeight(double height) {
        if (height < 0 || height > 1) {
            return;
        }
        setTargetPosition((int)(height * SLIDE_MAX_EXTENSION_TICKS));
    }

    /**
     * Moves the slide to the specified height, not returning until that height has been reached
     * @param height the target height
     * @deprecated This method is unsafe, as it
     * a) has the potential to get stuck in an infinite loop if the slide never reaches its destination,
     * which could lead to a frozen opmode that cannot be gracefully stopped, and
     * b) does not update any other PID loops that could be running, which could potentially damage
     * the robot.
     * Currently, there is no way to resolve these problems without moving the method outside of
     * this class.
     * Do not use this method; instead, write your own loop that takes both of these factors into
     * consideration.
     */
    @Deprecated
    public void moveSlideTo(double height) {
        setTargetHeight(height);
        motor.runIfAvailable(PIDFDcMotor::waitUntilPointReached);
    }

    public boolean isMoving() {
        return Math.abs(motor.requireDevice().getPower()) > IDLE_POWER_THRESHOLD;
    }

    /**
     * Updates the motor power using the provided PIDF coefficients
     */
    @Override
    public void updateMotorPowers() {
        motor.runIfAvailable(slide -> {
            if (slide.isOverCurrent()) {
                slide.setPower(0);
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            slide.setPIDF(SlideConfig.P_COEF, SlideConfig.I_COEF, SlideConfig.D_COEF, SlideConfig.F_COEF);
            slide.setTolerance(SlideConfig.TOLERANCE);

            slide.applyMotorPIDF();
        });
    }

    /**
     * Checks if the motors need to be updated
     *
     * @return true if {@link #updateMotorPowers()} needs to be called, false otherwise
     */
    @Override
    public boolean isUpdateNecessary() {
        PIDFDcMotor slide = motor.requireDevice();
        slide.setTolerance(SlideConfig.TOLERANCE);
        if (slide.atSetPoint()) {
            // we are at our target position
            // since this method will return false, updateMotorPowers() won't be called, so we have
            // to stop the motors ourselves
            slide.setPower(0);
            return false; // no update necessary
        }
        return true; // we need to call the PID loop
    }

    @Override
    public void log() {
        Telemetry telemetry = getTelemetry();
        if (!motor.isAvailable()) { return; }
        PIDFDcMotor slide = motor.requireDevice();

        telemetry.addData("[slide] Current slide position", slide.getCurrentPosition());
        telemetry.addData("[slide] motor current", slide.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("[slide] motor current alert", slide.getCurrentAlert(CurrentUnit.MILLIAMPS));
        telemetry.addData("[slide] Target slide position", slide.getSetPoint());
    }
}
