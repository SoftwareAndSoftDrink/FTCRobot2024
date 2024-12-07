package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.core.Module;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;

/**
 * This module controls the intake, which collects specimens and samples
 */
@Config
public class Intake extends Module {

    private final ConditionalHardwareDevice<CRServo> intakeServo;

    private final ConditionalHardwareDevice<Servo> wristServo;

    private double currentWristPosition = WRIST_POSITION_MOVING;

    // Names of the servos on the robot configuration
    public static final String INTAKE_SERVO_NAME = "Intake Servo";

    public static final String WRIST_SERVO_NAME = "Wrist Servo";

    public static double SERVO_SPEED_GRAB = 0.5;
    public static double SERVO_SPEED_EJECT = 1;

    public static double WRIST_POSITION_INTAKE = 0.54;
    public static double WRIST_POSITION_MOVING = 0.50;
    public static double WRIST_POSITION_FRONT_SCORING_HIGH = 0.50;
    public static double WRIST_POSITION_FRONT_SCORING_LOW = 0.50;
    public static double WRIST_POSITION_BACK_SCORING = 0.75;
    public static double WRIST_POSITION_DEACTIVATED = 0.90;
    public static double WRIST_POSITION_START = 0.2;

    private double prevWristPosition;
    private boolean wristActive;

    /**
     * Gets the base wrist offset
     * @return the offset applied to any values passed to {@link #moveWristTo(double)}
     */
    public double getBaseWristOffset() {
        return baseWristOffset;
    }

    private static double clampToServoBounds(double val) {
        return Math.min(Math.max(val, 0.0), 1.0);
    }

    /**
     * Sets the offset applied to any values passed to {@link #moveWristTo(double)}
     */
    public void setBaseWristOffset(double baseWristOffset) {
        this.baseWristOffset = clampToServoBounds(baseWristOffset);
    }

    private double baseWristOffset = 0;

    /**
     * Initializes the module and registers it with the specified OpMode.  This is where references to any hardware
     * devices used by the module are loaded.
     *
     * @param registrar The OpMode initializing the module
     * @implNote In order to be used in {@link ModuleManager}, all modules should have a public constructor that takes
     * exactly the same parameters as this one
     * @see ModuleManager#getModule(Class)
     */
    public Intake(OpMode registrar) {
        super(registrar);

        intakeServo = ConditionalHardwareDevice.tryGetHardwareDevice(registrar.hardwareMap, CRServo.class, INTAKE_SERVO_NAME);
        wristServo = ConditionalHardwareDevice.tryGetHardwareDevice(registrar.hardwareMap, Servo.class, WRIST_SERVO_NAME);
        wristActive = true;
    }

    /**
     * Checks if this module is connected to the hardware it requires
     *
     * @return true if the module is connected, false otherwise
     */
    @Override
    public boolean isConnected() {
        return intakeServo.isAvailable() || wristServo.isAvailable();
    }

    /**
     * Ensures that the module is in a safe state for other modules to operate.
     * Between calling this method and calling any other method on this module that modifies
     * hardware devices, the module is guaranteed to not damage itself or anything else when
     * other modules modify hardware state
     */
    @Override
    public void ensureSafety() {
        settle();
        moveWristTo(WRIST_POSITION_INTAKE);
        holdWristRotation();
    }

    /**
     * Sets the intake to eject whatever it is holding.
     * The intake will stay in this mode until either {@link #grab()} or {@link #settle()} is called.
     */
    public void eject() {
        intakeServo.runIfAvailable(i -> {
            i.setPower(SERVO_SPEED_EJECT);
        });
    }

    /**
     * Sets the intake to collect whatever is in front of it.
     * The intake will stay in this mode until either {@link #eject()} or {@link #settle()} is called.
     */
    public void grab() {
        intakeServo.runIfAvailable(i -> {
            i.setPower(-SERVO_SPEED_GRAB);
        });
    }

    /**
     * Sets the intake to hold the object it has
     */
    public void settle() {
        intakeServo.runIfAvailable(i -> {
            i.setPower(0);
        });
    }

    /**
     * Toggles the rotation of the wrist
     * @deprecated use {@link #moveWristTo(double)} to set the position directly instead
     */
    public void turn() {
        wristServo.runIfAvailable(w -> {
            if (w.getPosition() == 0.0) {
                w.setPosition(0.5);
            } else if (w.getPosition() == 0.5) {
                w.setPosition(0.0);
            }
        });
    }

    /**
     * Sets the wrist to its current rotation
     */
    public void holdWristRotation() {
        moveWristTo(currentWristPosition);
    }

    /**
     * Moves the wrist to the specified position
     * @param position the desired position of the servo, within the range [0,1]
     */
    public void moveWristTo(double position) {
        // TODO I would move this assertion to after offset is applied, but it would cause normal code to throw
        assert position <= 1.0 && position >= 0.0;
        wristServo.runIfAvailable(w -> {
            currentWristPosition = clampToServoBounds(position + baseWristOffset);
            w.setPosition(currentWristPosition);
        });
    }

    public void setWristActive(boolean active) {
        if (wristActive == active || !wristServo.isAvailable()) { return; }
        if (wristActive) {
            deactivateWrist();
        }
        else {
            activateWrist();
        }
        wristActive = active;
    }
    private void activateWrist() {
        moveWristTo(prevWristPosition);
    }
    private void deactivateWrist() {
        prevWristPosition = wristServo.requireDevice().getPosition();
        moveWristTo(WRIST_POSITION_DEACTIVATED);
    }

    @Override
    public void log() {
        Telemetry telemetry = getTelemetry();
        if(!intakeServo.isAvailable()) {
            return;
        }
        CRServo intake = intakeServo.requireDevice();

        telemetry.addData("Current Wrist Servo Power", intake.getPower());
        telemetry.addData("Current wrist offset", baseWristOffset);
    }
}
