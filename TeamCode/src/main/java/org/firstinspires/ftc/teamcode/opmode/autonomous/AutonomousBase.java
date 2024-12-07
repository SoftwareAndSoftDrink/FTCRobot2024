package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.AutonomousDriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;
import org.firstinspires.ftc.teamcode.modules.core.MotorPowerUpdater;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpMain;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

/**
 * A base class containing functionality common to all autonomous opmodes
 */
public abstract class AutonomousBase extends LinearOpMode {
    @Config
    public static class AutonomousConstants {
        public static long DRIVE_TRAIN_PID_DEFAULT_TIMEOUT_MS = 5 * 1000; // 5 s * 1000 ms/s
    }

    /**
     * The {@link ModuleManager} for this opmode.
     * All modules should be accessed through this to avoid creating multiple instances
     */
    protected final ModuleManager moduleManager = new ModuleManager(this);

    /**
     * Pauses until the play button has been pressed (or until the current thread
     * gets interrupted, which typically indicates that the OpMode has been stopped).
     */
    @Override
    public void waitForStart() {
        TeleOpMain.resetSlidePosition = false; // We have already reset the slide position in our init
        super.waitForStart();
    }

    /**
     * Blocks the current thread from progressing until a condition is met.
     * While waiting, updates all loaded {@linkplain MotorPowerUpdater MotorPowerUpdaters}.
     * @param conditionToStop When this returns true, the method will return
     * @throws InterruptedException This opmode has been stopped
     */
    private void waitUntil(BooleanSupplier conditionToStop) throws InterruptedException {
        RobotLog.ii("AutonomousBase", "Begin wait");
        while (!conditionToStop.getAsBoolean()) {
            if (isStopRequested()) {
                throw new InterruptedException();
            }
            moduleManager.updateMotorPowerLoops();
        }
        RobotLog.ii("AutonomousBase", "End wait");
    }

    /**
     * Blocks the current thread from progressing until the specified time has elapsed.
     * While waiting, this method will update all loaded {@linkplain MotorPowerUpdater MotorPowerUpdaters}
     * @param timeoutMs The amount of time this method will wait for, in milliseconds
     * @throws InterruptedException This opmode has been stopped
     */
    protected final void waitForTime(long timeoutMs) throws InterruptedException {
        final ElapsedTime run = new ElapsedTime();
        run.reset();
        waitUntil(() -> run.time(TimeUnit.MILLISECONDS) >= timeoutMs);
    }

    /**
     * Blocks the current thread from progressing until the provided mechanisms no longer need to update.
     * While waiting, updates all loaded {@linkplain MotorPowerUpdater MotorPowerUpdaters}.
     * @param timeoutMs The maximum time that can be spent waiting for the mechanisms, in milliseconds
     * @param mechanisms the mechanisms to update
     * @throws InterruptedException This opmode has been stopped
     */
    protected final void waitForMotorUpdaters(long timeoutMs, MotorPowerUpdater ...mechanisms) throws InterruptedException {
        final ElapsedTime run = new ElapsedTime();
        run.reset();
        // wait until mechanisms are done updating or timeout is reached, whichever comes first
        waitUntil(() -> run.time(TimeUnit.MILLISECONDS) >= timeoutMs ||
                Arrays.stream(mechanisms).reduce( // true == stop waiting, false == keep waiting
                true, // a && true == a
                (stop, mechanism) -> {
                    if (mechanism.isUpdateNecessary()) {
                        // more updates necessary, keep waiting
                        mechanism.updateMotorPowers();
                        return false;
                    }
                    return stop; // this mechanism has finished updating
                },
                (a, b) -> a && b // returns true only if both a and b are true -> only stops when all mechanisms are done
        ));
    }

    /**
     * Resets the arm's position using the arm's position switch
     * @throws InterruptedException This opmode has stopped
     */
    protected final void resetArmPosition() throws InterruptedException {
        final Arm arm = moduleManager.getModule(Arm.class);
        final LinearSlide slide = moduleManager.getModule(LinearSlide.class);
        final Intake intake = moduleManager.getModule(Intake.class);

        // get arm out of way
        slide.setTargetHeight(0);
        slide.updateMotorPowers();
        arm.setTargetRotationAbsolute(20);
        arm.updateMotorPowers();
        intake.moveWristTo(Intake.WRIST_POSITION_DEACTIVATED);
        Thread.sleep(TeleOpMain.INITIAL_JUMP_TIME_MILLIS);
        arm.deactivate();

        while (!arm.monitorPositionSwitch()) {
            slide.updateMotorPowers();
        }
        arm.activate();
    }

    protected final void waitForEnd() throws InterruptedException {
        try {
            waitUntil(this::isStopRequested);
        }
        finally {
            TeleOpMain.resetSlidePosition = false;
            moduleManager.unloadAll();
        }
    }

    /**
     * Waits until the robot has moved to the specified position.
     * While waiting, updates all {@linkplain MotorPowerUpdater MotorPowerUpdaters}
     * @param timeoutMs The maximum time that should be spent waiting for the robot to finish moving,
     *                  in milliseconds
     * @param destination The position for the robot to move to
     * @throws InterruptedException This opmode has been stopped
     */
    protected final void moveRobotTo(long timeoutMs, Pose2D destination) throws InterruptedException {
        final AutonomousDriveTrain driveTrain = moduleManager.getModule(AutonomousDriveTrain.class);
        driveTrain.setTargetPose(destination);
        RobotLog.ii("AutonomousBase", "Moving from " + driveTrain.getRobotPose() + " to " + destination);
        waitForMotorUpdaters(timeoutMs, driveTrain);
        RobotLog.ii("AutonomousBase", "Ended at " + driveTrain.getRobotPose());
    }

    /**
     * Waits until the robot has moved to the specified position.
     * While waiting, updates all {@linkplain MotorPowerUpdater MotorPowerUpdaters}
     * @param destination The position for the robot to move to
     * @throws InterruptedException This opmode has been stopped
     */
    protected final void moveRobotTo(Pose2D destination) throws InterruptedException {
        moveRobotTo(AutonomousConstants.DRIVE_TRAIN_PID_DEFAULT_TIMEOUT_MS, destination);
    }
}
