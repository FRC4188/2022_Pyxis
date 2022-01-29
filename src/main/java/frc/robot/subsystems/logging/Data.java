package frc.robot.subsystems.logging;

public class Data {

    private String label;
    private String value;

    public Data(String label, String value) {
        this.label = label;
        this.value = value;
    }

    public String getLabel() {
        return label;
    }

    public String getValue() {
        return value;
    }
}
