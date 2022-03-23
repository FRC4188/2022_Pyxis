package lib4188.data;

public class Data {

    public interface Key {}

    String label;
    Number datum;

    public Data(String label, Number datum) {
        this.label = label;
        this.datum = datum;
    }

    public String getLabel() {
        return label;
    }

    public Number getDatum() {
        return datum;
    }
}
