package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.AutonomousDriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpMain;

@Autonomous
public class BasketAuto3Sample extends BasketAutonomous {
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

}
