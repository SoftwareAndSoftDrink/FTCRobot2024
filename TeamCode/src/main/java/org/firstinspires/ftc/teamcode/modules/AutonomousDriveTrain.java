package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.modules.core.MotorPowerUpdater;

@Config
public class AutonomousDriveTrain extends FieldCentricDriveTrain implements MotorPowerUpdater {
    private MotorPowerUpdater[] updatableMechanisms = new MotorPowerUpdater[0];

    private Pose2D targetPose;
    public static final DistanceUnit TRANSLATE_UNIT = DistanceUnit.INCH;
    public static final AngleUnit ROTATE_UNIT = AngleUnit.DEGREES;

    //PIDs for control
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController hController;

    // Timer for information
    ElapsedTime timer = new ElapsedTime();

    // PID Coefficients for translation
    public static double TRANSLATE_P = 0.2;
    public static double TRANSLATE_I = 0.006;
    public static double TRANSLATE_D = 0.01;

    // PID Coefficients for rotation
    public static double ROTATE_P = 0.06;
    public static double ROTATE_I = 0;
    public static double ROTATE_D = 0.0;

    public static double TRANSLATE_TOLERANCE = 0.25;
    public static double ROTATE_TOLERANCE = 2;

    private boolean disabled = false;

    /**
     * Creates a AutonomousDriveTrain object using the provided opmode
     * Can be used for the entirety of an OpMode by changing the targetPosition
     * @param registrar the OpMode initializing this module
     */
    public AutonomousDriveTrain(OpMode registrar) {
        super(registrar);

        // initialize PID controllers
        xController = new PIDController(TRANSLATE_P, TRANSLATE_I,TRANSLATE_D);
        yController = new PIDController(TRANSLATE_P, TRANSLATE_I,TRANSLATE_D);
        hController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);

        xController.setTolerance(TRANSLATE_TOLERANCE);
        yController.setTolerance(TRANSLATE_TOLERANCE);
        hController.setTolerance(ROTATE_TOLERANCE);
    }

    /**
     * Disables the movement PID, allowing for manual control of the drive train
     */
    public void disableDrivePID() {
        disabled = true;
    }

    /**
     * Enables the movement PID, preventing manual control of the robot
     */
    public void enableDrivePID() {
        disabled = false;
    }

    /**
     * @param strafe   The right velocity
     * @param forward  The forward velocity
     * @param rotation The rotational velocity
     */
    @Override
    public void setVelocity(double strafe, double forward, double rotation) {
        if (strafe == 0 && forward == 0 && rotation == 0) {
            enableDrivePID();
        }
        else {
            disableDrivePID();
        }
        super.setVelocity(strafe, forward, rotation);
    }

    @Override
    public void updateMotorPowers() {
        if (disabled) {
            return; // If the movement PID is disabled, don't run the movement PID
        }

        final Pose2D currentRobotPose = getRobotPose();

        // updates PID controllers with current robot position (for the given axis)
        final double xPower = xController.calculate(currentRobotPose.getX(TRANSLATE_UNIT));
        final double yPower = yController.calculate(currentRobotPose.getY(TRANSLATE_UNIT));
        final double hPower = hController.calculate(currentRobotPose.getHeading(ROTATE_UNIT));

        // set driveTrain velocity based on PID controller output
        super.setVelocity(xPower, yPower, hPower);

        // update all updatableMechanisms
        for(MotorPowerUpdater mechanism : updatableMechanisms) {
            mechanism.updateMotorPowers();
        }
    }

    /**
     * Checks if the motors need to be updated
     *
     * @return true if {@link #updateMotorPowers()} needs to be called, false otherwise
     */
    @Override
    public boolean isUpdateNecessary() {
        if (disabled) {
            // if the drive train is being controlled manually, don't update the PID
            return false;
        }

        // use 'and' instead of 'or' since we are only at our target if x, y, and heading have been reached
        if (xController.atSetPoint() && yController.atSetPoint() && hController.atSetPoint()) {
            // we are at our target
            // since this method will return false, updateMotorPowers() won't be run, so we have to
            // stop the robot ourselves
            super.setVelocity(0, 0, 0);
            return false; // no update necessary
        }
        return true; // we haven't reached our target, so the PID loop should be called again
    }

    /**
     * moves the robot to the target position
     * @throws InterruptedException The opmode has stopped
     * @deprecated This method has been superseded by functionality in {@link org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousBase AutonomousBase}
     */
    @Deprecated
    public void move() throws InterruptedException {
        // used the end of the move to report the duration of the move
        timer.reset();

        // avoids a NullPointerException
        if(targetPose == null) {
            return;
        }

        // loop for driveTrain position PID, etc.
        while(!xController.atSetPoint() || !yController.atSetPoint() || !hController.atSetPoint()) {
            // When an opmode is stopped, its thread is interrupted, which means that we can check
            // for a thread interrupt to make sure we don't get smited by OpModeManager
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }

            updateMotorPowers();

            // update all telemetry
            getTelemetry().update();
        }

        // set the driveTrain velocity to 0 so that the robot doesn't move after the method ends
        super.setVelocity(0,0,0);

        // adds the time taken to move() to telemetry
        getTelemetry().addData("move() runtime: ", timer.time());
        getTelemetry().update();
    }

    /**
     * sets the target position and moves the robot to the target position
     * @param target the target robot position to be reached at the end of the move()
     * @throws InterruptedException The opmode has stopped
     * @deprecated This method has been superseded by functionality in {@link org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousBase AutonomousBase}
     */
    @Deprecated
    public void move(Pose2D target) throws InterruptedException {
        setTargetPose(target);
        move();
    }

    /**
     * a secondary method of changing the targetPosition without calling move()
     * @param target the Pose2D of the targetPosition of the robot for the next move()
     */
    public void setTargetPose(Pose2D target){
        targetPose = target;

        xController.setSetPoint(targetPose.getX(TRANSLATE_UNIT));
        yController.setSetPoint(targetPose.getY(TRANSLATE_UNIT));
        hController.setSetPoint(targetPose.getHeading(ROTATE_UNIT));
    }

    /**
     * sets mechanisms that need to be updated during the move()
     * @param mechanisms an array of mechanisms that need to have their control loops run during move()
     */
    public void setUpdatableMechanisms(MotorPowerUpdater[] mechanisms){
        updatableMechanisms = mechanisms;
    }
}
