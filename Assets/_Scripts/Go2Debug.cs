using UnityEngine;

[RequireComponent(typeof(Go2PathController))]
public class Go2Debug : MonoBehaviour
{
    private Go2PathController controller;
    private GameObject go2Model;
    private Vector3 lastValidPosition;
    private Quaternion lastValidRotation;
    private Vector3 lastValidScale;
    
    [Header("Debug Settings")]
    public bool enableDebugLogs = true;
    public bool autoFixInvalidTransform = true;
    public float maxAllowedDistance = 100f;
    
    void Start()
    {
        controller = GetComponent<Go2PathController>();
        if (controller != null && controller.go2Model != null)
        {
            go2Model = controller.go2Model;
            StoreValidTransform();
        }
    }
    
    void LateUpdate()
    {
        if (go2Model == null) return;
        
        // Check position
        if (!IsValidVector(go2Model.transform.position))
        {
            if (enableDebugLogs)
            {
                Debug.LogError($"GO2 position became invalid: {go2Model.transform.position}");
                Debug.LogError($"Last valid position was: {lastValidPosition}");
            }
            
            if (autoFixInvalidTransform)
            {
                go2Model.transform.position = lastValidPosition;
                Debug.LogWarning("Auto-fixed GO2 position");
            }
        }
        else if (go2Model.transform.position.magnitude > maxAllowedDistance)
        {
            if (enableDebugLogs)
            {
                Debug.LogWarning($"GO2 position too far from origin: {go2Model.transform.position.magnitude}");
            }
            
            if (autoFixInvalidTransform)
            {
                go2Model.transform.position = lastValidPosition;
                Debug.LogWarning("Auto-fixed GO2 position (too far)");
            }
        }
        else
        {
            lastValidPosition = go2Model.transform.position;
        }
        
        // Check rotation
        if (!IsValidQuaternion(go2Model.transform.rotation))
        {
            if (enableDebugLogs)
            {
                Debug.LogError($"GO2 rotation became invalid: {go2Model.transform.rotation}");
                Debug.LogError($"Euler angles: {go2Model.transform.eulerAngles}");
            }
            
            if (autoFixInvalidTransform)
            {
                go2Model.transform.rotation = lastValidRotation;
                Debug.LogWarning("Auto-fixed GO2 rotation");
            }
        }
        else
        {
            lastValidRotation = go2Model.transform.rotation;
        }
        
        // Check scale
        if (!IsValidVector(go2Model.transform.localScale) || go2Model.transform.localScale.magnitude < 0.01f)
        {
            if (enableDebugLogs)
            {
                Debug.LogError($"GO2 scale became invalid: {go2Model.transform.localScale}");
            }
            
            if (autoFixInvalidTransform)
            {
                go2Model.transform.localScale = lastValidScale;
                Debug.LogWarning("Auto-fixed GO2 scale");
            }
        }
        else
        {
            lastValidScale = go2Model.transform.localScale;
        }
    }
    
    void StoreValidTransform()
    {
        if (go2Model != null)
        {
            lastValidPosition = go2Model.transform.position;
            lastValidRotation = go2Model.transform.rotation;
            lastValidScale = go2Model.transform.localScale;
            
            // Ensure we have valid defaults
            if (!IsValidVector(lastValidScale) || lastValidScale.magnitude < 0.01f)
            {
                lastValidScale = Vector3.one;
            }
        }
    }
    
    bool IsValidVector(Vector3 v)
    {
        return !float.IsNaN(v.x) && !float.IsNaN(v.y) && !float.IsNaN(v.z) &&
               !float.IsInfinity(v.x) && !float.IsInfinity(v.y) && !float.IsInfinity(v.z);
    }
    
    bool IsValidQuaternion(Quaternion q)
    {
        return !float.IsNaN(q.x) && !float.IsNaN(q.y) && !float.IsNaN(q.z) && !float.IsNaN(q.w) &&
               !float.IsInfinity(q.x) && !float.IsInfinity(q.y) && !float.IsInfinity(q.z) && !float.IsInfinity(q.w);
    }
    
    void OnGUI()
    {
        if (!enableDebugLogs || go2Model == null) return;
        
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label($"GO2 Position: {go2Model.transform.position}");
        GUILayout.Label($"GO2 Rotation: {go2Model.transform.eulerAngles}");
        GUILayout.Label($"GO2 Scale: {go2Model.transform.localScale}");
        GUILayout.Label($"Distance from origin: {go2Model.transform.position.magnitude:F2}");
        GUILayout.EndArea();
    }
} 