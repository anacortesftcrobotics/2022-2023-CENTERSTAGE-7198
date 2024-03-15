package org.firstinspires.ftc.teamcode.CenterStage8934.OpModesRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CenterStage8934.OpModesRobot.CenterStageAutoBackstage;

@Autonomous(name="CS 8934 Auto Blue Backstage", group="CenterStage")
public class CenterStageAutoBackstageBlue extends CenterStageAutoBackstage {
    @Override
    public void initializeGameConfig() {
        THIS_ALLIANCE = ALLIANCE.BLUE;
        STAGE_LOCATION = STAGE_LOCATION.BACK;
    }
}
