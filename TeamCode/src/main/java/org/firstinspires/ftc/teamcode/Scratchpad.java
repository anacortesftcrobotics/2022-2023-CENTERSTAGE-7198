package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;
import java.util.TimerTask;
@TeleOp
public class Scratchpad extends OpMode {
    Servo intakeElbow;
    Servo shoulderServo;
    boolean oldCircle;
    boolean intake;
    @Override
    public void init() {
        shoulderServo = hardwareMap.get(Servo.class, "shoulder");
        intakeElbow = hardwareMap.get(Servo.class, "intakeElbow");
    }
    public void loop(){
        intakeControl();
    }

    private void intakeControl() {
        /** intakeControl input: circle **/
        boolean intakeToggle = gamepad2.circle;
        boolean intakeToggle2 = gamepad2.a;
        if (intakeToggle && !oldCircle) {
            intake = !intake;
            if (intake) {
                //More negative target pos closer to output
                shoulderServo.setPosition(0.1);

                new Timer().schedule(new TimerTask() {
                    @Override
                    public void run() {
                        intakeElbow.setPosition(0.05);
                    }
                }, 700);
            } else {
                if (intakeToggle && !oldCircle) {
                    intake = !intake;
                    if (intake) {

                    } else {
                        shoulderServo.setPosition(0.1);
                        intakeElbow.setPosition(0.9);
                    }

                }
            }
            oldCircle = intakeToggle;
        }
    }
}
