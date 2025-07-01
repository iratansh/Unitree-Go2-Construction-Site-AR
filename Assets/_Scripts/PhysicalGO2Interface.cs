using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Newtonsoft.Json;

[System.Serializable]
public class PhysicalGO2Status
{
    public Vector3 position;      // Real-world position in meters
    public float orientation;     // Heading in degrees
    public Vector3 velocity;      // Current velocity vector
    public bool isMoving;
    public string status;         // "idle", "moving", "error", etc.
    public float batteryLevel;
    public long timestamp;
}

[System.Serializable]
public class PhysicalGO2Command
{
    public Vector3 targetPosition;    // Target position in real-world coordinates
    public float targetOrientation;   // Target heading
    public float speed;              // Movement speed
    public string command;           // "move_to", "stop", "pause", "resume"
    public Vector3[] pathPoints;     // Full path for A* navigation
    public long timestamp;
}

public class PhysicalGO2Interface : MonoBehaviour
{
    [Header("Network Configuration")]
    public string go2IPAddress = "192.168.1.100";
    public int commandPort = 12346;
    public int statusPort = 12345;
    public float updateRate = 30f; // Hz
    
    [Header("Real-World Calibration")]
    public Vector3 realWorldOrigin = Vector3.zero;
    public float unityToRealWorldScale = 1.0f; // Unity units to meters
    public float coordinateRotationOffset = 0f; // Degrees to align coordinate systems
    
    [Header("GO2 Tracking")]
    public Transform physicalGO2Tracker;
    public bool enablePositionTracking = true;
    public bool enablePathSending = true;
    
    [Header("AR Integration")]
    public ARConstructionProjector arProjector;
    public bool updateARVisualization = true;
    
    [Header("Debug")]
    public bool showDebugInfo = true;
    public bool logNetworkTraffic = false;
    
    // Networking
    private UdpClient commandClient;
    private UdpClient statusClient;
    private Thread statusListener;
    private bool isListening = false;
    
    // Status tracking
    private PhysicalGO2Status lastStatus;
    private bool isConnected = false;
    private float lastUpdateTime = 0f;
    private float lastCommandTime = 0f;
    
    // Path planning integration
    private Vector3[] currentPath;
    private int currentWaypointIndex = 0;
    
    void Start()
    {
        InitializeNetworking();
        SetupTracking();
        
        if (arProjector == null)
        {
            arProjector = FindObjectOfType<ARConstructionProjector>();
        }
    }
    
    void InitializeNetworking()
    {
        try
        {
            Debug.Log($"Connecting to physical GO2 at {go2IPAddress}...");
            
            // Initialize UDP clients
            commandClient = new UdpClient();
            statusClient = new UdpClient(statusPort);
            statusClient.Client.ReceiveTimeout = 100; // 100ms timeout
            
            // Start status listener thread
            isListening = true;
            statusListener = new Thread(new ThreadStart(ListenForStatus));
            statusListener.Start();
            
            Debug.Log("Physical GO2 interface initialized");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to initialize GO2 interface: {e.Message}");
        }
    }
    
    void SetupTracking()
    {
        // Find or create tracker
        if (physicalGO2Tracker == null)
        {
            GameObject tracker = new GameObject("Physical GO2 Tracker");
            physicalGO2Tracker = tracker.transform;
        }
        
        Debug.Log("Physical GO2 tracking setup complete");
    }
    
    void ListenForStatus()
    {
        Debug.Log("Started listening for GO2 status updates...");
        
        while (isListening)
        {
            try
            {
                IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = statusClient.Receive(ref remoteEndPoint);
                string json = Encoding.UTF8.GetString(data);
                
                if (logNetworkTraffic)
                {
                    Debug.Log($"Received status: {json}");
                }
                
                PhysicalGO2Status status = JsonConvert.DeserializeObject<PhysicalGO2Status>(json);
                ProcessStatusUpdate(status);
                
                isConnected = true;
                lastUpdateTime = Time.time;
            }
            catch (SocketException e)
            {
                if (e.SocketErrorCode != SocketError.TimedOut)
                {
                    Debug.LogError($"Status listener error: {e.Message}");
                }
                
                // Check for connection timeout
                if (Time.time - lastUpdateTime > 5f)
                {
                    isConnected = false;
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Status processing error: {e.Message}");
            }
            
            Thread.Sleep(10); // Small delay to prevent CPU spinning
        }
    }
    
    void ProcessStatusUpdate(PhysicalGO2Status status)
    {
        lastStatus = status;
        
        // Update tracker position
        if (enablePositionTracking && physicalGO2Tracker != null)
        {
            Vector3 unityPosition = RealWorldToUnityPosition(status.position);
            physicalGO2Tracker.position = unityPosition;
            
            // Update rotation
            float unityRotation = status.orientation + coordinateRotationOffset;
            physicalGO2Tracker.rotation = Quaternion.Euler(0, unityRotation, 0);
        }
        
        // Update AR visualization
        if (updateARVisualization && arProjector != null)
        {
            // Update GO2 tracker position in AR
            // The AR projector will handle visual updates
        }
    }
    
    Vector3 RealWorldToUnityPosition(Vector3 realWorldPos)
    {
        // Convert real-world coordinates to Unity coordinates
        Vector3 offset = realWorldPos - realWorldOrigin;
        Vector3 scaled = offset / unityToRealWorldScale;
        
        // Apply coordinate rotation if needed
        if (coordinateRotationOffset != 0f)
        {
            float radians = coordinateRotationOffset * Mathf.Deg2Rad;
            float cos = Mathf.Cos(radians);
            float sin = Mathf.Sin(radians);
            
            Vector3 rotated = new Vector3(
                scaled.x * cos - scaled.z * sin,
                scaled.y,
                scaled.x * sin + scaled.z * cos
            );
            
            return rotated;
        }
        
        return scaled;
    }
    
    Vector3 UnityToRealWorldPosition(Vector3 unityPos)
    {
        // Convert Unity coordinates to real-world coordinates
        Vector3 scaled = unityPos * unityToRealWorldScale;
        
        // Apply inverse coordinate rotation if needed
        if (coordinateRotationOffset != 0f)
        {
            float radians = -coordinateRotationOffset * Mathf.Deg2Rad;
            float cos = Mathf.Cos(radians);
            float sin = Mathf.Sin(radians);
            
            Vector3 rotated = new Vector3(
                scaled.x * cos - scaled.z * sin,
                scaled.y,
                scaled.x * sin + scaled.z * cos
            );
            
            scaled = rotated;
        }
        
        return realWorldOrigin + scaled;
    }
    
    public void SendCommand(PhysicalGO2Command command)
    {
        if (commandClient == null)
        {
            Debug.LogWarning("Command client not initialized");
            return;
        }
        
        try
        {
            command.timestamp = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            string json = JsonConvert.SerializeObject(command);
            byte[] data = Encoding.UTF8.GetBytes(json);
            
            IPEndPoint endpoint = new IPEndPoint(IPAddress.Parse(go2IPAddress), commandPort);
            commandClient.Send(data, data.Length, endpoint);
            
            if (logNetworkTraffic)
            {
                Debug.Log($"Sent command: {json}");
            }
            
            lastCommandTime = Time.time;
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to send command: {e.Message}");
        }
    }
    
    public void MoveTo(Vector3 unityPosition, float speed = 0.5f)
    {
        Vector3 realWorldPos = UnityToRealWorldPosition(unityPosition);
        
        PhysicalGO2Command command = new PhysicalGO2Command
        {
            command = "move_to",
            targetPosition = realWorldPos,
            speed = speed
        };
        
        SendCommand(command);
        Debug.Log($"Sent move command to real-world position: {realWorldPos}");
    }
    
    public void SendPath(Vector3[] unityPath, float speed = 0.5f)
    {
        if (!enablePathSending)
        {
            Debug.LogWarning("Path sending is disabled");
            return;
        }
        
        // Convert Unity path to real-world coordinates
        Vector3[] realWorldPath = new Vector3[unityPath.Length];
        for (int i = 0; i < unityPath.Length; i++)
        {
            realWorldPath[i] = UnityToRealWorldPosition(unityPath[i]);
        }
        
        PhysicalGO2Command command = new PhysicalGO2Command
        {
            command = "follow_path",
            pathPoints = realWorldPath,
            speed = speed
        };
        
        SendCommand(command);
        currentPath = unityPath;
        currentWaypointIndex = 0;
        
        // Update AR visualization
        if (arProjector != null)
        {
            arProjector.UpdatePathVisualization(unityPath);
        }
        
        Debug.Log($"Sent path with {realWorldPath.Length} waypoints to physical GO2");
    }
    
    public void StopMovement()
    {
        PhysicalGO2Command command = new PhysicalGO2Command
        {
            command = "stop"
        };
        
        SendCommand(command);
        Debug.Log("Sent stop command to physical GO2");
    }
    
    public void PauseMovement()
    {
        PhysicalGO2Command command = new PhysicalGO2Command
        {
            command = "pause"
        };
        
        SendCommand(command);
        Debug.Log("Sent pause command to physical GO2");
    }
    
    public void ResumeMovement()
    {
        PhysicalGO2Command command = new PhysicalGO2Command
        {
            command = "resume"
        };
        
        SendCommand(command);
        Debug.Log("Sent resume command to physical GO2");
    }
    
    void Update()
    {
        // Handle manual controls for testing
        if (Input.GetKeyDown(KeyCode.T))
        {
            TestMovement();
        }
        
        if (Input.GetKeyDown(KeyCode.Y))
        {
            StopMovement();
        }
        
        // Check connection status
        if (Time.time - lastUpdateTime > 5f && isConnected)
        {
            isConnected = false;
            Debug.LogWarning("Lost connection to physical GO2");
        }
    }
    
    void TestMovement()
    {
        // Test movement to a position 2 meters forward
        Vector3 testPosition = transform.position + Vector3.forward * 2f;
        MoveTo(testPosition);
    }
    
    public bool IsConnected()
    {
        return isConnected;
    }
    
    public PhysicalGO2Status GetLastStatus()
    {
        return lastStatus;
    }
    
    public Vector3 GetPhysicalPosition()
    {
        if (lastStatus != null)
        {
            return RealWorldToUnityPosition(lastStatus.position);
        }
        return Vector3.zero;
    }
    
    void OnDestroy()
    {
        isListening = false;
        
        if (statusListener != null && statusListener.IsAlive)
        {
            statusListener.Join(1000);
            if (statusListener.IsAlive)
            {
                statusListener.Abort();
            }
        }
        
        if (commandClient != null)
        {
            commandClient.Close();
        }
        
        if (statusClient != null)
        {
            statusClient.Close();
        }
    }
    
    void OnGUI()
    {
        if (!showDebugInfo) return;
        
        GUILayout.BeginArea(new Rect(Screen.width - 300, 10, 280, 250));
        GUILayout.Label("Physical GO2 Interface:");
        GUILayout.Label($"Connected: {isConnected}");
        GUILayout.Label($"GO2 IP: {go2IPAddress}");
        
        if (lastStatus != null)
        {
            GUILayout.Label($"Position: {lastStatus.position}");
            GUILayout.Label($"Unity Pos: {GetPhysicalPosition()}");
            GUILayout.Label($"Orientation: {lastStatus.orientation:F1}°");
            GUILayout.Label($"Moving: {lastStatus.isMoving}");
            GUILayout.Label($"Battery: {lastStatus.batteryLevel:F1}%");
            GUILayout.Label($"Status: {lastStatus.status}");
        }
        
        GUILayout.Label("");
        GUILayout.Label("Test Controls:");
        GUILayout.Label("T - Test Movement");
        GUILayout.Label("Y - Stop Movement");
        
        if (GUILayout.Button("Send Test Path"))
        {
            Vector3[] testPath = new Vector3[]
            {
                transform.position,
                transform.position + Vector3.forward * 1f,
                transform.position + Vector3.forward * 2f + Vector3.right * 1f,
                transform.position + Vector3.forward * 3f
            };
            SendPath(testPath);
        }
        
        GUILayout.EndArea();
    }
} 