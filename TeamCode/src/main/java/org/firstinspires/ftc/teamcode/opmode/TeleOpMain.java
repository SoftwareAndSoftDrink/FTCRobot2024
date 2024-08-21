package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.*;

@TeleOp(name="TeleOpMain")
public class TeleOpMain extends OpMode {

    private DriveTrain driveTrain;

    private ConditionalHardwareDevice<DcMotor> slideMotor;
    private PIDFController slidePID;

    @Override
    public void init() {
        driveTrain = new DriveTrain(this);

        slideMotor = ConditionalHardwareDevice.tryGetHardwareDevice(hardwareMap, DcMotor.class, "Slide Motor");
        slideMotor.runIfAvailable(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
        slidePID = new PIDFController(0,0,0,0);
    }

    @Override
    public void loop() {
        driveTrain.setVelocity(-gamepad1.left_stick_x * 0.5, gamepad1.left_stick_y * 0.5, gamepad1.right_stick_x * 0.5);

        slideMotor.runIfAvailable(motor -> {
            slidePID.setSetPoint(-gamepad2.left_stick_y);
            motor.setPower(slidePID.calculate(motor.getCurrentPosition()));
        });
    }

}
