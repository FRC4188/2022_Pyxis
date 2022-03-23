package lib4188.data;

import java.util.HashMap;

import lib4188.data.Data.Key;

public class DataHandler {
    
    private static HashMap<Key, Data> map = new HashMap<>();

    public static Data getDatum(Key key) {
        return map.get(key);
    }

    public static void addDatum(Key key, Data value) {
        map.put(key, value);
    }
}
