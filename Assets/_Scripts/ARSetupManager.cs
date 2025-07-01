using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Management;

public class ARSetupManager : MonoBehaviour
{
    [Header("AR Configuration")]
    public bool enablePassthrough = true;
    public bool showRealWorld = true;
    public Camera arCamera;
    public Transform trackingOrigin;
    
    [Header("Audio Settings")]
    public bool enableSpatialAudio = true;
    public AudioSource[] sceneSounds;
    
    [Header("GO2 Integration")]
    public bool enableGO2Tracking = true;
    public GameObject go2Controller;
    
    void Start()
    {
        SetupAR();
        ConfigureAudio();
        SetupGO2Integration();
    }
    
    void SetupAR()
    {
        Debug.Log("Setting up AR for Quest...");
        
        // Find or create AR Camera
        if (arCamera == null)
        {
            arCamera = Camera.main;
            if (arCamera == null)
            {
                GameObject cameraGO = new GameObject("AR Camera");
                arCamera = cameraGO.AddComponent<Camera>();
            }
        }
        
        // Configure camera for AR
        ConfigureARCamera();
        
        // Setup tracking origin
        SetupTrackingOrigin();
        
        // Enable passthrough if supported
        if (enablePassthrough)
        {
            EnablePassthrough();
        }
        
        Debug.Log("AR setup complete");
    }
    
    void ConfigureARCamera()
    {
        // Set camera clear flags for passthrough
        if (showRealWorld)
        {
            arCamera.clearFlags = CameraClearFlags.SolidColor;
            arCamera.backgroundColor = Color.clear; // Transparent for passthrough
        }
        else
        {
            arCamera.clearFlags = CameraClearFlags.Skybox;
        }
        
        // Set appropriate near/far clip planes for AR
        arCamera.nearClipPlane = 0.01f;
        arCamera.farClipPlane = 1000f;
        
        // Disable audio listener if there are multiple cameras
        AudioListener listener = arCamera.GetComponent<AudioListener>();
        if (listener == null)
        {
            arCamera.gameObject.AddComponent<AudioListener>();
        }
        
        Debug.Log("AR Camera configured");
    }
    
    void SetupTrackingOrigin()
    {
        // Create tracking origin if it doesn't exist
        if (trackingOrigin == null)
        {
            GameObject trackingGO = new GameObject("Tracking Origin");
            trackingOrigin = trackingGO.transform;
        }
        
        // Set camera as child of tracking origin
        if (arCamera.transform.parent != trackingOrigin)
        {
            arCamera.transform.SetParent(trackingOrigin);
        }
        
        // Reset local position for proper tracking
        arCamera.transform.localPosition = Vector3.zero;
        arCamera.transform.localRotation = Quaternion.identity;
        
        Debug.Log("Tracking origin setup complete");
    }
    
    void EnablePassthrough()
    {
        Debug.Log("Attempting to enable Quest passthrough...");
        
        // This will be handled by Quest's built-in passthrough when the app requests it
        // The actual passthrough enabling is done through Quest system settings
        // and the app needs to request it properly
        
        // Set environment blend mode for passthrough
        if (XRSettings.enabled)
        {
            Debug.Log("XR is enabled, passthrough should work");
        }
        else
        {
            Debug.LogWarning("XR is not enabled - passthrough may not work");
        }
    }
    
    void ConfigureAudio()
    {
        if (!enableSpatialAudio) return;
        
        Debug.Log("Configuring spatial audio for AR...");
        
        // Configure existing audio sources for spatial audio
        if (sceneSounds != null)
        {
            foreach (AudioSource audio in sceneSounds)
            {
                if (audio != null)
                {
                    audio.spatialBlend = 1.0f; // 3D spatial audio
                    audio.rolloffMode = AudioRolloffMode.Logarithmic;
                    audio.maxDistance = 50f;
                    audio.dopplerLevel = 1f;
                    
                    Debug.Log($"Configured spatial audio for: {audio.name}");
                }
            }
        }
        
        // Find all audio sources in scene if none specified
        if (sceneSounds == null || sceneSounds.Length == 0)
        {
            AudioSource[] allAudioSources = FindObjectsOfType<AudioSource>();
            foreach (AudioSource audio in allAudioSources)
            {
                audio.spatialBlend = 1.0f;
                audio.rolloffMode = AudioRolloffMode.Logarithmic;
                audio.maxDistance = 30f;
            }
            Debug.Log($"Auto-configured {allAudioSources.Length} audio sources for spatial audio");
        }
    }
    
    void SetupGO2Integration()
    {
        if (!enableGO2Tracking) return;
        
        Debug.Log("Setting up GO2 integration for AR...");
        
        // Find GO2 controller if not assigned
        if (go2Controller == null)
        {
            // Look for various GO2 controller types
            SimpleGo2Controller simpleController = FindObjectOfType<SimpleGo2Controller>();
            if (simpleController != null)
            {
                go2Controller = simpleController.gameObject;
            }
            else
            {
                Go2IsolationController isolationController = FindObjectOfType<Go2IsolationController>();
                if (isolationController != null)
                {
                    go2Controller = isolationController.gameObject;
                }
            }
        }
        
        if (go2Controller != null)
        {
            // Add AR-specific components to GO2 controller
            ARGo2Enhancer enhancer = go2Controller.GetComponent<ARGo2Enhancer>();
            if (enhancer == null)
            {
                enhancer = go2Controller.AddComponent<ARGo2Enhancer>();
            }
            
            Debug.Log("GO2 AR integration setup complete");
        }
        else
        {
            Debug.LogWarning("No GO2 controller found for AR integration");
        }
    }
    
    void Update()
    {
        // Monitor XR state
        if (Input.GetKeyDown(KeyCode.P))
        {
            TogglePassthrough();
        }
        
        // Update tracking origin if needed
        UpdateTrackingOrigin();
    }
    
    void TogglePassthrough()
    {
        showRealWorld = !showRealWorld;
        ConfigureARCamera();
        Debug.Log($"Passthrough toggled: {showRealWorld}");
    }
    
    void UpdateTrackingOrigin()
    {
        // This can be used to adjust the tracking origin based on real-world conditions
        // For now, we keep it simple
    }
    
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, Screen.height - 120, 300, 100));
        GUILayout.Label("AR Status:");
        GUILayout.Label($"XR Active: {XRSettings.enabled}");
        GUILayout.Label($"Passthrough: {showRealWorld}");
        GUILayout.Label($"Audio Sources: {(sceneSounds != null ? sceneSounds.Length : 0)}");
        GUILayout.Label("Press 'P' to toggle passthrough");
        GUILayout.EndArea();
    }
} 