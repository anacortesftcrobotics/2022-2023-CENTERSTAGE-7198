package org.firstinspires.ftc.teamcode.logancode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kinematics.PoseVelocity2D;

import java.io.*;
import java.util.ArrayList;

public class Odo6Path {

    private ArrayList<PoseVelocity2D> positions = new ArrayList<PoseVelocity2D>();

    public Odo6Path(InputStream inputStream, Telemetry telem)
    {
        loadPathFile(inputStream, telem);
        telem.addLine("Path Data Loaded from: " + inputStream.toString());
        telem.update();
    }

    public PoseVelocity2D getPosition(int index)
    {
        if(index >=0 && index < positions.size())
            return positions.get(index);
        return null;
    }

    private void loadPathFile(InputStream inputStream, Telemetry telem)
    {
        try (InputStreamReader read = new InputStreamReader(inputStream);
             BufferedReader br = new BufferedReader(read))
        {

            String line;
            while ((line = br.readLine()) != null) {
                String[] xyString;
                xyString = line.split(";", 6);

                double x,y,r,vx,vy,vr;
                x=0;y=0;r=0;vx=0;vy=0;vr=0;

                if(xyString.length == 6) {
                    for(int i = 0; i < 6; i++)
                    {
                        xyString[i] = xyString[i].replace(";","");
                    }

                    if (xyString[0].contains("x"))
                        x = Double.parseDouble(xyString[0].substring(1));
                    if (xyString[1].contains("y"))
                        y = Double.parseDouble(xyString[1].substring(2));
                    if (xyString[2].contains("r"))
                        r = Double.parseDouble(xyString[2].substring(2));
                    if (xyString[3].contains("vx"))
                        vx = Double.parseDouble(xyString[3].substring(3));
                    if (xyString[4].contains("vy"))
                        vy = Double.parseDouble(xyString[4].substring(3));
                    if (xyString[5].contains("vr")) {
                        vr = Double.parseDouble(xyString[5].substring(3));
                    }

                    positions.add(new PoseVelocity2D(x,y,r,vx,vy,vr));
                }
            }
        }
        catch (IOException ioException)
        {
            telem.addLine(ioException.toString());
        }
    }

    public Odo6Path(){

    }

    public void addPathMarker(PoseVelocity2D pm)
    {
        positions.add(pm);
    }

    public int length()
    {
        return positions.size();
    }

    public String serialize()
    {
        String output = new String();

        for (PoseVelocity2D pos : positions)
        {
            output += pos.serialize() + "\n";
        }

        return output;
    }
}
