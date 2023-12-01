package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RunArmMotor extends OpMode {
    DcMotorEx intakeShoulder;
    boolean oldCircle;
    boolean intake;


public void init() {
    intakeShoulder = hardwareMap.get(DcMotorEx.class, "grabElbow");
}

    @Override
    public void loop() {
        MoveMotor();
    }

    private void MoveMotor(){
        boolean intakeToggle = gamepad2.circle;
        if (intakeToggle && !oldCircle) {
            intake = !intake;
            if (intake) {
                intakeShoulder.setTargetPosition(-1300);
                intakeShoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                intakeShoulder.setTargetPosition(-1000);
                intakeShoulder.setPower(-0.4);
            }
            else {
                intakeShoulder.setPower(0.4);
                intakeShoulder.setTargetPosition(0);
            }
        }
        oldCircle = intakeToggle;

}
}