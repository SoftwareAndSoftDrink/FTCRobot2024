package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.modules.AutonomousDriveTrain;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpMain;

@Config
@Autonomous
public class BasketAutonomous extends AutonomousBase {
    //config variables for positions
    public static double SAFE_MOVE_DISTANCE_X_AND_Y = 12;

    public static long INTAKE_FORWARD_DURATION_MS = 450;
    public static double INTAKE_FORWARD_POWER = 0.37;

    public static long OUTTAKE_DURATION_MS = 500;

    public static double POST_INTAKE_HEADING = 90;

    public static long PRESCORE_SLIDE_MOVEMENT_MS = 250;
    public static long SCORING_ARM_SLIDE_MOVEMENT_TIMEOUT_MS = 1000;
    public static long INTAKE_ARM_SLIDE_MOVEMENT_TIMEOUT_MS = 1000;
    public static long HANG_MOVE_TO_FINAL_TIMEOUT_MS = 1000;

    public static double SCORING_X = -21;
    public static double SCORING_Y = 10;
    public static double SCORING_HEADING = 133;

    public static double PARKING_X = 12;
    public static double PARKING_Y = 3;
    public static double PARKING_HEADING = 0;

    @Deprecated
    public static double X2 = -26;
    @Deprecated
    public static double Y2 = 10;
    @Deprecated
    public static double H2 = 25;

    @Deprecated
    public static double X3 = -26;
    @Deprecated
    public static double Y3 = 16;
    @Deprecated
    public static double H3 = 25;

    public static double INTAKE1_X = 4.5;
    public static double INTAKE1_Y = 27;
    public static double INTAKE1_HEADING = 60;

    public static double INTAKE2_X = -2.5;
    public static double INTAKE2_Y = 27;
    public static double INTAKE2_HEADING = 61.5;

    public static double INTAKE3_X = -4;
    public static double INTAKE3_Y = 39;
    public static double INTAKE3_HEADING = 90;

    public static double HANG_SETUP_X = -12;
    public static double HANG_SETUP_Y = 60;
    public static double HANG_SETUP_HEADING = -100;

    public static double HANG_FINAL_X = 3;
    public static double HANG_FINAL_Y = 49;
    public static double HANG_FINAL_HEADING = -90;

    public static long INTAKE1_PAUSE_MS = 30;
    public static long INTAKE2_PAUSE_MS = 30;
    public static long INTAKE3_PAUSE_MS = 500;


    /*
     * ! IMPORTANT !
     * These poses are instance fields so that they get updated with the above values whenever
     * autonomous is rerun.  This way, the fields above can be updated in FTC Dashboard with the
     * correct positions during tuning.
     */

    public final Pose2D intake1 = new Pose2D(AutonomousDriveTrain.TRANSLATE_UNIT, INTAKE1_X, INTAKE1_Y, AutonomousDriveTrain.ROTATE_UNIT, INTAKE1_HEADING);

    public final Pose2D intake2 = new Pose2D(AutonomousDriveTrain.TRANSLATE_UNIT, INTAKE2_X, INTAKE2_Y, AutonomousDriveTrain.ROTATE_UNIT, INTAKE2_HEADING);

    public final Pose2D intake3 = new Pose2D(AutonomousDriveTrain.TRANSLATE_UNIT, INTAKE3_X, INTAKE3_Y, AutonomousDriveTrain.ROTATE_UNIT, INTAKE3_HEADING);

    public final Pose2D scoring = new Pose2D(AutonomousDriveTrain.TRANSLATE_UNIT, SCORING_X, SCORING_Y, AutonomousDriveTrain.ROTATE_UNIT, SCORING_HEADING);

    public final Pose2D postScore = new Pose2D(
            DistanceUnit.INCH,
            scoring.getX(DistanceUnit.INCH) + SAFE_MOVE_DISTANCE_X_AND_Y,
            scoring.getY(DistanceUnit.INCH) + SAFE_MOVE_DISTANCE_X_AND_Y,
            AngleUnit.DEGREES,
            scoring.getHeading(AngleUnit.DEGREES)
    );

    public final Pose2D parking = new Pose2D(AutonomousDriveTrain.TRANSLATE_UNIT, PARKING_X, PARKING_Y, AutonomousDriveTrain.ROTATE_UNIT, PARKING_HEADING);

    public final Pose2D hangSetup = new Pose2D(AutonomousDriveTrain.TRANSLATE_UNIT, HANG_SETUP_X, HANG_SETUP_Y, AutonomousDriveTrain.ROTATE_UNIT, HANG_SETUP_HEADING);

    public final Pose2D hangFinal = new Pose2D(AutonomousDriveTrain.TRANSLATE_UNIT, HANG_FINAL_X, HANG_FINAL_Y, AutonomousDriveTrain.ROTATE_UNIT, HANG_FINAL_HEADING);

    @Deprecated
    public final Pose2D move2 = new Pose2D(AutonomousDriveTrain.TRANSLATE_UNIT, X2, Y2, AutonomousDriveTrain.ROTATE_UNIT, H2);

    @Deprecated
    public final Pose2D move3 = new Pose2D(AutonomousDriveTrain.TRANSLATE_UNIT, X3, Y3, AutonomousDriveTrain.ROTATE_UNIT, H3);

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            final AutonomousDriveTrain driveTrain = moduleManager.getModule(AutonomousDriveTrain.class);
            final Arm arm = moduleManager.getModule(Arm.class);
            final LinearSlide slide = moduleManager.getModule(LinearSlide.class);
            final Intake intake = moduleManager.getModule(Intake.class);
            TeleOpMain.resetSlidePosition = false;

            waitForStart();
            TeleOpMain.resetSlidePosition = false;

            resetArmPosition();

            /* score preload */
            scoreHighBasket(arm, slide, intake);

            /* Intake & score the 1st sample */
            moveRobotTo(intake1);
            intakeSample(intake, arm, slide, driveTrain, INTAKE1_PAUSE_MS);
            scoreHighBasket(arm, slide, intake);

            /* Intake & score the 2nd sample */
            moveRobotTo(intake2);
            intakeSample(intake, arm, slide, driveTrain, INTAKE2_PAUSE_MS);
            scoreHighBasket(arm, slide, intake);

            /* Intake & score the 3rd sample */
            moveRobotTo(intake3);
            intakeSample(intake, arm, slide, driveTrain, INTAKE3_PAUSE_MS);
            scoreHighBasket(arm, slide, intake);

            /* hang */
            arm.setTargetRotation(Arm.ARM_ROTATION_HANG_LVL1_SETUP);
            slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_HANG_LVL1);
            intake.moveWristTo(Intake.WRIST_POSITION_DEACTIVATED);
            moveRobotTo(hangSetup);
            moveRobotTo(HANG_MOVE_TO_FINAL_TIMEOUT_MS, hangFinal);
            arm.deactivate();

            waitForEnd();
        }
        finally {
            TeleOpMain.resetSlidePosition = false;
        }
    }

    protected void postIntake(Arm arm, LinearSlide slide, Intake intake, AutonomousDriveTrain driveTrain) throws InterruptedException {
        arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        intake.moveWristTo(Intake.WRIST_POSITION_MOVING);
        final Pose2D curPose = driveTrain.getRobotPose();
        moveRobotTo(new Pose2D(
                AutonomousDriveTrain.TRANSLATE_UNIT,
                curPose.getX(AutonomousDriveTrain.TRANSLATE_UNIT),
                curPose.getY(AutonomousDriveTrain.TRANSLATE_UNIT),
                AngleUnit.DEGREES,
                POST_INTAKE_HEADING
        ));
    }

    protected void scoreHighBasket(Arm arm, LinearSlide slide, Intake intake) throws InterruptedException {
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        intake.moveWristTo(Intake.WRIST_POSITION_FRONT_SCORING_HIGH);
        waitForMotorUpdaters(PRESCORE_SLIDE_MOVEMENT_MS, slide);

        arm.setTargetRotation(Arm.ARM_ROTATION_FRONT_SCORING_HIGH);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_SCORING_HIGH);
        waitForMotorUpdaters(SCORING_ARM_SLIDE_MOVEMENT_TIMEOUT_MS, arm, slide);
        moveRobotTo(scoring);

        intake.eject();
        waitForTime(OUTTAKE_DURATION_MS);
        intake.settle();

        // move a bit back from the basket so that the arm can safely move down
        moveRobotTo(postScore);

        arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        intake.moveWristTo(Intake.WRIST_POSITION_MOVING);
        waitForMotorUpdaters(500, arm, slide);
    }

    protected void intakeSample(Intake intake, Arm arm, LinearSlide slide, AutonomousDriveTrain driveTrain, long intakeWaitMs) throws InterruptedException {
        intake.grab();
        arm.setTargetRotation(Arm.ARM_ROTATION_INTAKE);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_INTAKE);
        intake.moveWristTo(Intake.WRIST_POSITION_INTAKE);
        waitForMotorUpdaters(INTAKE_ARM_SLIDE_MOVEMENT_TIMEOUT_MS, arm, slide); // wait a bit for the arm & slide to move in place

        // move forward so that the sample is caught
        // We move for a time instead of using movePID to prevent the program from freezing when
        // the robot hits a wall
        driveTrain.disableDrivePID(); // so we can do manual control
        final Pose2D curPose = driveTrain.getRobotPose();
        final double curAngleTrig = curPose.getHeading(AngleUnit.RADIANS) + (Math.PI / 2);
        driveTrain.setVelocity(INTAKE_FORWARD_POWER * Math.cos(curAngleTrig), INTAKE_FORWARD_POWER * Math.sin(curAngleTrig), 0);
        waitForTime(INTAKE_FORWARD_DURATION_MS);
        driveTrain.enableDrivePID();

        waitForTime(intakeWaitMs);
        intake.settle();
        arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        intake.moveWristTo(Intake.WRIST_POSITION_MOVING);
    }
}