package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class TouchTest2 extends OpMode {
    DigitalChannel limitSwitchFront;

    @Override
    public void init() {
        limitSwitchFront = hardwareMap.get(DigitalChannel.class, "limitSwitchFront");
        limitSwitchFront.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        telemetry.addData("State", limitSwitchFront.getState());
        telemetry.update();
    }
}
