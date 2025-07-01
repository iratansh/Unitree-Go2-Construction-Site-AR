using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Newtonsoft.Json;

// Data structures for communication
[System.Serializable]
public class Go2Command
{
    public float speed;
    public string mode; // "forward" or "rightward"
    public float gazeAngle; // degrees relative to movement direction
    public bool start;
    public bool stop;
    public bool reset;
}

[System.Serializable]
public class Go2Status
{
    public Vector3 position;
    public float orientation;
    public float distanceTraveled;
    public bool isMoving;
    public string currentMode;
}

public class Go2PathController : MonoBehaviour
{
    [Header("Path Configuration")]
    public float pathLength = 10f; // 10 meters
    public Transform startPoint;
    public Transform endPoint;
    public LineRenderer pathVisualizer;
    
    [Header("Robot Configuration")]
    public GameObject go2Model;
    public float defaultSpeed = 0.5f; // m/s
    public float turnSpeed = 90f; // degrees/s
    
    [Header("Network Configuration")]
    public int receivePort = 12345;
    public int sendPort = 12346;
    public string pythonIP = "127.0.0.1";
    public bool enableNetworking = true; // Toggle for standalone mode
    
    [Header("Standalone Mode")]
    public bool standaloneMode = false;
    public KeyCode startKey = KeyCode.Space;
    public KeyCode stopKey = KeyCode.S;
    public KeyCode resetKey = KeyCode.R;
    public KeyCode forwardModeKey = KeyCode.Alpha1;
    public KeyCode rightwardModeKey = KeyCode.Alpha2;
    
    // Movement variables
    private Vector3 movementDirection;
    private float currentSpeed;
    private float targetGazeAngle;
    private bool isMoving = false;
    private float distanceTraveled = 0f;
    private Vector3 startPosition;
    private string movementMode = "forward";
    
    // Networking
    private Thread receiveThread;
    private UdpClient receiveClient;
    private UdpClient sendClient;
    private bool isListening = false;
    
    // Animation parameters (if using Animator)
    private Animator animator;
    
    // For realistic movement
    private Vector3 basePosition;
    
    void Start()
    {
        InitializePath();
        
        // Only initialize networking if enabled
        if (enableNetworking && !standaloneMode)
        {
            InitializeNetworking();
        }
        else
        {
            Debug.Log("Running in standalone mode - use keyboard controls");
        }
        
        animator = go2Model.GetComponent<Animator>();
        currentSpeed = defaultSpeed;
    }
    
    void InitializePath()
    {
        // Ensure we have a valid GO2 model first
        if (go2Model == null)
        {
            Debug.LogError("GO2 Model is not assigned!");
            return;
        }
        
        if (startPoint == null)
        {
            startPoint = new GameObject("StartPoint").transform;
            startPoint.position = transform.position;
        }
        
        if (endPoint == null)
        {
            endPoint = new GameObject("EndPoint").transform;
            endPoint.position = startPoint.position + Vector3.forward * pathLength;
        }
        
        startPosition = startPoint.position;
        
        // Validate start position
        if (!IsValidPosition(startPosition))
        {
            Debug.LogError($"Invalid start position: {startPosition}");
            startPosition = Vector3.zero;
        }
        
        go2Model.transform.position = startPosition;
        basePosition = startPosition;
        
        // Ensure GO2 has valid scale
        if (!IsValidVector(go2Model.transform.localScale) || go2Model.transform.localScale.magnitude < 0.01f)
        {
            Debug.LogWarning("GO2 scale was invalid, resetting to (1,1,1)");
            go2Model.transform.localScale = Vector3.one;
        }
        
        // Visualize path
        if (pathVisualizer != null)
        {
            pathVisualizer.SetPosition(0, startPoint.position);
            pathVisualizer.SetPosition(1, endPoint.position);
        }
    }
    
    void InitializeNetworking()
    {
        try
        {
            receiveClient = new UdpClient(receivePort);
            receiveClient.Client.ReceiveTimeout = 100; // 100ms timeout to prevent blocking
            sendClient = new UdpClient();
            
            isListening = true;
            receiveThread = new Thread(new ThreadStart(ReceiveData));
            receiveThread.Start();
            
            Debug.Log($"Go2 Controller listening on port {receivePort}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to initialize networking: {e.Message}");
            standaloneMode = true;
        }
    }
    
    void ReceiveData()
    {
        while (isListening)
        {
            try
            {
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = receiveClient.Receive(ref anyIP);
                string json = Encoding.UTF8.GetString(data);
                
                Go2Command command = JsonConvert.DeserializeObject<Go2Command>(json);
                ProcessCommand(command);
            }
            catch (SocketException e)
            {
                // Timeout is normal, just continue
                if (e.SocketErrorCode != SocketError.TimedOut)
                {
                    Debug.LogError($"Socket error: {e.Message}");
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Receive error: {e.Message}");
            }
            
            // Small delay to prevent CPU spinning
            Thread.Sleep(10);
        }
    }
    
    void ProcessCommand(Go2Command command)
    {
        if (command.start)
        {
            StartMovement();
        }
        else if (command.stop)
        {
            StopMovement();
        }
        else if (command.reset)
        {
            ResetPosition();
        }
        
        currentSpeed = command.speed;
        movementMode = command.mode;
        targetGazeAngle = command.gazeAngle;
    }
    
    void StartMovement()
    {
        isMoving = true;
        distanceTraveled = 0f;
        
        // Set movement direction based on mode
        if (movementMode == "rightward")
        {
            movementDirection = Vector3.right;
        }
        else
        {
            movementDirection = Vector3.forward;
        }
    }
    
    void StopMovement()
    {
        isMoving = false;
        if (animator != null)
        {
            animator.SetFloat("Speed", 0);
        }
    }
    
    void ResetPosition()
    {
        StopMovement();
        go2Model.transform.position = startPosition;
        basePosition = startPosition;
        go2Model.transform.rotation = Quaternion.identity;
        distanceTraveled = 0f;
    }
    
    void Update()
    {
        // Handle standalone mode input
        if (standaloneMode || !enableNetworking)
        {
            HandleStandaloneInput();
        }
        
        if (isMoving)
        {
            UpdateMovement();
            UpdateOrientation();
            
            if (enableNetworking && !standaloneMode)
            {
                SendStatus();
            }
        }
    }
    
    void HandleStandaloneInput()
    {
        if (Input.GetKeyDown(startKey))
        {
            StartMovement();
            Debug.Log("Started movement");
        }
        
        if (Input.GetKeyDown(stopKey))
        {
            StopMovement();
            Debug.Log("Stopped movement");
        }
        
        if (Input.GetKeyDown(resetKey))
        {
            ResetPosition();
            Debug.Log("Reset position");
        }
        
        if (Input.GetKeyDown(forwardModeKey))
        {
            movementMode = "forward";
            Debug.Log("Switched to forward mode");
        }
        
        if (Input.GetKeyDown(rightwardModeKey))
        {
            movementMode = "rightward";
            Debug.Log("Switched to rightward mode");
        }
        
        // Adjust gaze angle with arrow keys
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            targetGazeAngle -= turnSpeed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            targetGazeAngle += turnSpeed * Time.deltaTime;
        }
    }
    
    void UpdateMovement()
    {
        // Calculate movement
        float moveDistance = currentSpeed * Time.deltaTime;
        
        // Validate move distance
        if (float.IsNaN(moveDistance) || float.IsInfinity(moveDistance))
        {
            Debug.LogError("Invalid move distance detected!");
            StopMovement();
            return;
        }
        
        // Check if we've reached the path length
        if (distanceTraveled + moveDistance > pathLength)
        {
            moveDistance = pathLength - distanceTraveled;
            isMoving = false;
        }
        
        // Move the robot - update base position
        Vector3 movement = movementDirection * moveDistance;
        Vector3 newPosition = basePosition + movement;
        
        // Validate new position
        if (!IsValidPosition(newPosition))
        {
            Debug.LogError($"Invalid position detected: {newPosition}");
            StopMovement();
            return;
        }
        
        basePosition = newPosition;
        distanceTraveled += moveDistance;
        
        // Set the actual position (will be modified by realistic movement)
        go2Model.transform.position = basePosition;
        
        // Update animation
        if (animator != null)
        {
            animator.SetFloat("Speed", currentSpeed);
            animator.SetBool("IsWalking", isMoving);
            
            // Set strafe parameter for rightward movement
            if (movementMode == "rightward")
            {
                animator.SetFloat("Strafe", 1f);
            }
            else
            {
                animator.SetFloat("Strafe", 0f);
            }
        }
    }
    
    void UpdateOrientation()
    {
        // Calculate target rotation
        Quaternion targetRotation;
        
        if (movementMode == "rightward")
        {
            // When moving rightward, base orientation is looking forward
            // Add gaze angle adjustment
            targetRotation = Quaternion.Euler(0, targetGazeAngle, 0);
        }
        else
        {
            // When moving forward, base orientation is forward
            // Add gaze angle adjustment
            targetRotation = Quaternion.Euler(0, targetGazeAngle, 0);
        }
        
        // Smoothly rotate to target
        go2Model.transform.rotation = Quaternion.Slerp(
            go2Model.transform.rotation,
            targetRotation,
            turnSpeed * Time.deltaTime / 90f
        );
        
        // Add subtle body adjustments for realism
        AddRealisticMovement();
    }
    
    void AddRealisticMovement()
    {
        // Only apply realistic movement if we have a valid base position
        if (!IsValidPosition(basePosition))
        {
            Debug.LogError("Base position is invalid, skipping realistic movement");
            return;
        }
        
        // Add subtle swaying motion relative to base position
        float swayAmount = 0.02f; // 2cm sway
        float swaySpeed = 2f; // Hz
        
        Vector3 sway = new Vector3(
            Mathf.Sin(Time.time * swaySpeed) * swayAmount,
            0,
            Mathf.Cos(Time.time * swaySpeed * 0.7f) * swayAmount * 0.5f
        );
        
        Vector3 finalPosition = basePosition + sway;
        
        // Validate final position
        if (!IsValidPosition(finalPosition))
        {
            Debug.LogError($"Invalid final position after sway: {finalPosition}");
            go2Model.transform.position = basePosition; // Use base position without sway
            return;
        }
        
        // Apply sway as offset from base position
        go2Model.transform.position = finalPosition;
        
        // Add subtle body roll based on movement
        float rollAmount = currentSpeed * 2f; // degrees
        float roll = Mathf.Sin(Time.time * swaySpeed * 1.5f) * rollAmount;
        
        // Validate roll
        if (float.IsNaN(roll) || float.IsInfinity(roll))
        {
            Debug.LogError("Invalid roll value detected");
            return;
        }
        
        // Apply roll as a local rotation
        Vector3 currentEuler = go2Model.transform.eulerAngles;
        currentEuler.z = roll;
        
        // Validate euler angles
        if (!IsValidRotation(currentEuler))
        {
            Debug.LogError($"Invalid euler angles: {currentEuler}");
            return;
        }
        
        go2Model.transform.eulerAngles = currentEuler;
    }
    
    void SendStatus()
    {
        try
        {
            Go2Status status = new Go2Status
            {
                position = go2Model.transform.position,
                orientation = go2Model.transform.eulerAngles.y,
                distanceTraveled = distanceTraveled,
                isMoving = isMoving,
                currentMode = movementMode
            };
            
            string json = JsonConvert.SerializeObject(status);
            byte[] data = Encoding.UTF8.GetBytes(json);
            
            sendClient.Send(data, data.Length, pythonIP, sendPort);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to send status: {e.Message}");
        }
    }
    
    bool IsValidPosition(Vector3 position)
    {
        // Check for NaN
        if (float.IsNaN(position.x) || float.IsNaN(position.y) || float.IsNaN(position.z))
        {
            return false;
        }
        
        // Check for infinity
        if (float.IsInfinity(position.x) || float.IsInfinity(position.y) || float.IsInfinity(position.z))
        {
            return false;
        }
        
        // Check for reasonable bounds (within 1000 units from origin)
        float maxDistance = 1000f;
        if (position.magnitude > maxDistance)
        {
            Debug.LogWarning($"Position {position} is too far from origin (>{maxDistance} units)");
            return false;
        }
        
        return true;
    }
    
    bool IsValidRotation(Vector3 euler)
    {
        // Check for NaN
        if (float.IsNaN(euler.x) || float.IsNaN(euler.y) || float.IsNaN(euler.z))
        {
            return false;
        }
        
        // Check for infinity
        if (float.IsInfinity(euler.x) || float.IsInfinity(euler.y) || float.IsInfinity(euler.z))
        {
            return false;
        }
        
        return true;
    }
    
    bool IsValidVector(Vector3 v)
    {
        return !float.IsNaN(v.x) && !float.IsNaN(v.y) && !float.IsNaN(v.z) &&
               !float.IsInfinity(v.x) && !float.IsInfinity(v.y) && !float.IsInfinity(v.z);
    }
    
    void OnDestroy()
    {
        isListening = false;
        
        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Join(500); // Wait up to 500ms for thread to finish
            if (receiveThread.IsAlive)
            {
                receiveThread.Abort();
            }
        }
        
        if (receiveClient != null)
        {
            receiveClient.Close();
        }
        
        if (sendClient != null)
        {
            sendClient.Close();
        }
    }
    
    // Gizmos for visualization
    void OnDrawGizmos()
    {
        if (startPoint != null && endPoint != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(startPoint.position, 0.2f);
            
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(endPoint.position, 0.2f);
            
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(startPoint.position, endPoint.position);
        }
        
        // Show current base position
        if (Application.isPlaying && go2Model != null)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(basePosition, 0.1f);
        }
    }
}
