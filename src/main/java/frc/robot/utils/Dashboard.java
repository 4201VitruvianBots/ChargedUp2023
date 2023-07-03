package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import java.util.HashMap;
import java.util.Map;

public class Dashboard {
  private Dashboard instance;
  private final Map<String, NetworkTable> ntMap = new HashMap<>();

  private Dashboard() {}

  public Dashboard getInstance() {
    if (instance == null) {
      instance = new Dashboard();
    }
    return instance;
  }

  //    public void set(String table, String entry, Object value, ) {
  //        if(!ntMap.containsKey(table)) {
  //            String[] tables = table.split("/");
  //
  //            NetworkTable ntEntry = NetworkTableInstance.getDefault().getTable("");
  //            for(int i = 0; i < tables.length; i++) {
  //                if(i == 0) {
  //                    ntEntry = NetworkTableInstance.getDefault().getTable(tables[i]);
  //                } else {
  //                    ntEntry = ntEntry.getSubTable(tables[i]);
  //                }
  //            }
  //            ntMap.put(table, ntEntry);
  //        }
  //
  //        var ntMapEntry = ntMap.get(table);
  //
  //        switch (value.getClass()) {
  //            case int.class:
  //                break;
  //            case double.class:
  //                break;
  //        }
  //
  //    }

  public void set(NetworkTable table, String entry, Object value) {}
}
