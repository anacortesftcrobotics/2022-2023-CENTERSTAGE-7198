package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TouchTest extends OpMode {

    DigitalChannel limitSwitchFront;
    DigitalChannel limitSwitchBack;
    DcMotor hookArm;

    @Override
    public void init() {
        limitSwitchFront = hardwareMap.get(DigitalChannel.class, "limitSwitchFront");
        limitSwitchBack = hardwareMap.get(DigitalChannel.class, "limitSwitchBack");
        hookArm = hardwareMap.get(DcMotor.class, "hookArm");

        limitSwitchFront.setMode(DigitalChannel.Mode.INPUT);
        limitSwitchBack.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        armControl();
        telemetry.addData("FrontTouch", limitSwitchFront.getState());
        telemetry.addData("BackTouch", limitSwitchBack.getState());
        telemetry.update();

    }
    private void armControl(){
        double arm = -gamepad2.right_stick_y;

        hookArm.setPower(arm);
    }
}
