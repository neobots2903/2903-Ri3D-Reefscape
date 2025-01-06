package frc.robot.subsystems;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

import org.opencv.core.Rect2d;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionNetParse extends Thread {

    // Vision data fields
    public final int NUM_APRILTAGS = 22;
    public AprilTagData[] curAprilTagData = new AprilTagData[NUM_APRILTAGS];
    public AlgaeData curAlgaeData = new AlgaeData();

    private final int BUFFER = 256;
    private final int PORT = 5800;

    private DatagramSocket socket;
    private boolean running;
    private byte[] buf = new byte[BUFFER];

    public VisionNetParse() throws SocketException {
        socket = new DatagramSocket(PORT);
        // Initialize AprilTagData array
        for (int i = 0; i < curAprilTagData.length; i++) {
            AprilTagData tag = new AprilTagData();
            tag.id = i+1;
            curAprilTagData[i] = tag;
        }
    }

    public void run() {
        running = true;

        // Continuously attempt to receive packet data
        while(running) {
            DatagramPacket packet = new DatagramPacket(buf, buf.length);
            try {
                socket.receive(packet);
                byte[] data = packet.getData();

                // Format as ASCII string
                String stringData = new String(data, "UTF-8");
                // Remove brackets around packet data
                stringData = stringData.trim().substring(1, stringData.length()-2);
                
                // Determine what type of data is being received
                if (stringData.startsWith("APRIL")) {
                    curAprilTagData = parseAprilTagData(stringData.substring(6));
                } else if (stringData.startsWith("ALGAE")) {
                    curAlgaeData = parseAlgaeData(stringData.substring(6));
                }
            } catch (Exception e) {
                // Failed to receive packet; try again
            }
        }
    }

    private AprilTagData[] parseAprilTagData(String data) {
        AprilTagData[] result = new AprilTagData[16];
        
        //qx:1231|qy:1321|...,qx:1232|qy:1232,,
        String[] aprilData = data.split(",");
        // Check data for all 16 AprilTags
        for (int i = 0; i < aprilData.length; i++) {
            AprilTagData aprilTag = new AprilTagData();
            String[] kvs = aprilData[i].split("|");
            Double px = null, py = null, pz = null, qx = null, 
                    qy = null, qz = null, qw = null;
            // Set ID of current AprilTag we're checking
            aprilTag.id = i + 1;
            // Reach each transform value
            for (String kv : kvs) {
                String[] values = kv.split(":");
                try {
                    double num = Double.parseDouble(values[1]);
                    switch(values[0]) {
                        case "px":
                            px = num;
                            break;
                        case "py":
                            py = num;
                            break;
                        case "pz":
                            pz = num;
                            break;
                        case "qx":
                            qx = num;
                            break;
                        case "qy":
                            qy = num;
                            break;
                        case "qz":
                            qz = num;
                            break;
                        case "qw":
                            qw = num;
                            break;
                    }
                } catch (Exception e) {
                    // Error parsing data, transform is invalid
                    break;
                }
            }

            // Create transform if all data is valid
            if (px != null && py != null && pz != null &&
                qx != null && qy != null && qz != null && qw != null) {
                aprilTag.transform = new Transform3d(
                    new Translation3d(px, py, pz),
                    new Rotation3d(new Quaternion(qw, qx, qy, qz))
                    );
            } else {
                aprilTag.transform = null;
            }
            result[i] = aprilTag;
        }
        return result;
    }

    private AlgaeData parseAlgaeData(String data) {
        AlgaeData result = new AlgaeData();
        Integer x = null, y = null, w = null, h = null;
        // Reach bound values
        for (String kv : data.split("|")) {
            String[] values = kv.split(":");
            try {
                int num = Integer.parseInt(values[1]);
                switch(values[0]) {
                    case "x":
                        x = num;
                        break;
                    case "y":
                        y = num;
                        break;
                    case "w":
                        w = num;
                        break;
                    case "h":
                        h = num;
                        break;
                }
            } catch (Exception e) {
                // Error parsing data, bounds are invalid
                break;
            }
        }

        // Create bounds if all data is valid
        if (x != null && y != null && w != null && h != null) {
            result.bounds = new Rect2d(x, y, w, h);
        } else {
            result.bounds = null;
        }

        return result;
    }

    public class AprilTagData {
        /**
         * The ID of the AprilTag.
         */
        public int id;
        /**
         * The location of the AprilTag relative to the robot.
         */
        public Transform3d transform;
    }

    public class AlgaeData {
        /**
         * The bounding box of the closest algae in view.
         */
        public Rect2d bounds;
    }
}
