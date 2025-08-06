using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class RobotPoseReceiver : MonoBehaviour
{
    // --- Public Fields ---
    [Tooltip("The invisible 3D model of the robot that will act as the occluder.")]
    public Transform robotDigitalTwin;

    [Tooltip("The port to listen on. Must match the port in the Python script (e.g., 9051).")]
    public int listenPort = 9051;

    [Tooltip("Smoothes the movement of the digital twin to reduce network jitter.")]
    [Range(0.0f, 1.0f)]
    public float smoothingFactor = 0.25f;

    // --- Private Fields ---
    private Thread receiveThread;
    private UdpClient client;
    private IPEndPoint anyIP;

    // Latest pose data received from the network
    private Vector3 networkPosition;
    private Quaternion networkRotation;

    // Flag to indicate if new data has arrived
    private volatile bool newDataReceived = false;

    void Start()
    {
        if (robotDigitalTwin == null)
        {
            Debug.LogError("Robot Digital Twin transform is not assigned in the Inspector!");
            this.enabled = false;
            return;
        }

        // Initialize network position to the twin's starting position
        networkPosition = robotDigitalTwin.position;
        networkRotation = robotDigitalTwin.rotation;

        // Start the listener thread
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();

        Debug.Log($"Started listening for robot pose data on UDP port {listenPort}");
    }

    void Update()
    {
        // In the main thread, smoothly interpolate the digital twin's position and rotation
        // towards the latest data received from the network.
        if (newDataReceived)
        {
            robotDigitalTwin.position = Vector3.Lerp(robotDigitalTwin.position, networkPosition, smoothingFactor);
            robotDigitalTwin.rotation = Quaternion.Slerp(robotDigitalTwin.rotation, networkRotation, smoothingFactor);
        }
    }

    private void ReceiveData()
    {
        client = new UdpClient(listenPort);
        anyIP = new IPEndPoint(IPAddress.Any, listenPort);

        while (true)
        {
            try
            {
                // Blocks until a message returns on this socket from a remote host.
                byte[] data = client.Receive(ref anyIP);
                string text = Encoding.UTF8.GetString(data);

                // Parse the received string "x,y,z,yaw"
                string[] parts = text.Split(',');
                if (parts.Length == 4)
                {
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    float yaw = float.Parse(parts[3]);

                    // IMPORTANT: Coordinate System Conversion
                    // The Unitree SDK uses a right-handed coordinate system (X-forward, Y-left, Z-up).
                    // Unity uses a left-handed system (X-right, Y-up, Z-forward).
                    // We must convert from the robot's coordinate space to Unity's world space.
                    // Robot X (forward) -> Unity Z (forward)
                    // Robot Y (left)    -> Unity -X (right)
                    // Robot Z (up)      -> Unity Y (up)
                    networkPosition = new Vector3(-y, z, x);

                    // Yaw in the robot's system (around Z-up) corresponds to yaw around Unity's Y-up axis.
                    // The angle needs to be converted from radians to degrees.
                    networkRotation = Quaternion.Euler(0, yaw * Mathf.Rad2Deg, 0);
                    
                    newDataReceived = true;
                }
            }
            catch (Exception err)
            {
                Debug.LogError(err.ToString());
            }
        }
    }

    void OnDestroy()
    {
        // Clean up the thread and socket when the object is destroyed
        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Abort();
        }
        if (client != null)
        {
            client.Close();
        }
    }
} 