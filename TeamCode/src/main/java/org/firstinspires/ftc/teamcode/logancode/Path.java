package org.firstinspires.ftc.teamcode.logancode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.*;
import java.util.ArrayList;

public class Path
{

    private ArrayList<PathMarker> positions = new ArrayList<PathMarker>();

    public Path(String pathDataFileLocation, Telemetry telem)
    {
        loadPathFile(pathDataFileLocation, telem);
        telem.addLine("Path Data Loaded from: " + pathDataFileLocation);
        telem.update();
    }

    public Path(InputStream inputStream, Telemetry telem)
    {
        loadPathFile(inputStream, telem);
        telem.addLine("Path Data Loaded from: " + inputStream.toString());
        telem.update();
    }

    public PathMarker getPosition(int index)
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

                //if(xyString[0].contains("x"))
                    //TODO: new path loader
            }
        }
        catch (IOException ioException)
        {
            telem.addLine(ioException.toString());
        }
    }

    private void loadPathFile(String pathDataFileLocation, Telemetry telem)
    {
        try (FileReader fr = new FileReader(pathDataFileLocation);
             BufferedReader br = new BufferedReader(fr))
        {

            String line;
            while ((line = br.readLine()) != null) {
                String[] xyString = new String[2];
                xyString = line.split(" ", 2);

                positions.add(new PathMarker(Double.parseDouble(xyString[0]), Double.parseDouble(xyString[1])));
            }
        }
        catch (IOException ioException)
        {
            telem.addLine(ioException.toString());
        }
    }
}
