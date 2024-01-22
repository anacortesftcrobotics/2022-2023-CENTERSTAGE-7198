package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoP extends OpMode {

    @Override
    public void init() {
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        wristServo.setPosition(0.8);
    }

    @Override
    public void loop() {

    }
}
