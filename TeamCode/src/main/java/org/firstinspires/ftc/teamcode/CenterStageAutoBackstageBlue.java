package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CS Auto Blue Backstage", group="CenterStage")
public class CenterStageAutoBackstageBlue extends CenterStageAutoBackstage {
    @Override
    public void initializeGameConfig() {
        THIS_ALLIANCE = ALLIANCE.BLUE;
    }
}
