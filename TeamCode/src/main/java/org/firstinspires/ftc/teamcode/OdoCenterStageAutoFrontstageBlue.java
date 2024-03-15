package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CS 8934 Auto Blue Frontstage ODO", group="CenterStage")
public class OdoCenterStageAutoFrontstageBlue extends CenterStageAutoBackstageOdoPowered {
    @Override
    public void initializeGameConfig() {
        STAGE_LOCATION = STAGE_LOCATION.FRONT;
        THIS_ALLIANCE = CenterStageAutoBackstage.ALLIANCE.BLUE;
    }
}
