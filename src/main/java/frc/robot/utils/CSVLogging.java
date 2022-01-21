package frc.robot.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.DriverStation;

public class CSVLogging {
    
    private ArrayList<String> header;
    private BufferedWriter writer;

    private final long startTime;

    public CSVLogging(String file, String[] header) {
        try {
            file = newFile(file);
        } catch(IOException e) {
            System.out.print(e.getLocalizedMessage());
            file = file+".csv";
        }

        this.header = new ArrayList<String>(Arrays.asList(header));
        this.header.add(0, "time");

        try {
            new File("logs\\"+file).createNewFile();
            writer = new BufferedWriter(new FileWriter("logs\\"+file));

            writer.append(String.join(",", header));
            writer.append("\n");
        } catch(IOException e) {
            e.printStackTrace();
            writer = null;
        }

        startTime = System.currentTimeMillis() / (long) 1000.0;
    }

    private static String newFile(String file) throws IOException{
        Set<String> files = Stream.of(new File("logs").listFiles())
            .filter(iFile -> !iFile.isDirectory())
            .map(File::getName)
            .collect(Collectors.toSet());
        
        if (files.contains(file)) {
            int fileNum = 0;
            if (!files.contains(file+"-"))
                file += "-";
            while (files.contains(file)) {
                file.replaceAll("-"+(fileNum-1), "-"+fileNum);
                fileNum++;
            }
        }

        return file+".csv";
    }

    public void logData(String[] data) {
        try {
            long time = System.currentTimeMillis() / (long) 1000.0;
            writer.append(String.valueOf(time - startTime));
            writer.append(",");
            writer.append(String.join(",", data));

            writer.flush();
        } catch(IOException e) {
            DriverStation.reportError("Logging Failure!", e.getStackTrace());
            e.printStackTrace();
        }
    }
}
