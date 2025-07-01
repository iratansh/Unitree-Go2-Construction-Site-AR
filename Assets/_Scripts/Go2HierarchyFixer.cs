using UnityEngine;
using System.Collections.Generic;

public class Go2HierarchyFixer : MonoBehaviour
{
    [Header("Fix Settings")]
    public bool autoFixHierarchy = true;
    public bool disableAllColliders = true;
    public bool disableAllRigidbodies = true;
    public bool fixChildTransforms = true;
    public float maxAllowedScale = 1000f;
    public float minAllowedScale = 0.001f;
    
    private List<Collider> disabledColliders = new List<Collider>();
    private List<Rigidbody> disabledRigidbodies = new List<Rigidbody>();
    private Dictionary<Transform, TransformData> originalTransforms = new Dictionary<Transform, TransformData>();
    
    private class TransformData
    {
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 scale;
        
        public TransformData(Transform t)
        {
            position = t.localPosition;
            rotation = t.localRotation;
            scale = t.localScale;
        }
    }
    
    void Start()
    {
        if (autoFixHierarchy)
        {
            FixEntireHierarchy();
        }
    }
    
    [ContextMenu("Fix GO2 Hierarchy")]
    public void FixEntireHierarchy()
    {
        Debug.Log("Starting GO2 hierarchy fix...");
        
        // Store original transforms
        StoreOriginalTransforms(transform);
        
        // Fix all transforms in hierarchy
        if (fixChildTransforms)
        {
            FixTransformHierarchy(transform);
        }
        
        // Disable all colliders
        if (disableAllColliders)
        {
            DisableAllCollidersInHierarchy();
        }
        
        // Disable all rigidbodies
        if (disableAllRigidbodies)
        {
            DisableAllRigidbodiesInHierarchy();
        }
        
        Debug.Log($"GO2 hierarchy fix complete. Disabled {disabledColliders.Count} colliders and {disabledRigidbodies.Count} rigidbodies.");
    }
    
    void StoreOriginalTransforms(Transform root)
    {
        originalTransforms[root] = new TransformData(root);
        
        foreach (Transform child in root)
        {
            StoreOriginalTransforms(child);
        }
    }
    
    void FixTransformHierarchy(Transform root)
    {
        // Fix this transform
        if (!IsValidTransform(root))
        {
            Debug.LogWarning($"Fixing invalid transform: {root.name} at path: {GetPath(root)}");
            ResetTransform(root);
        }
        
        // Fix scale bounds
        if (root.localScale.x > maxAllowedScale || root.localScale.y > maxAllowedScale || root.localScale.z > maxAllowedScale ||
            root.localScale.x < minAllowedScale || root.localScale.y < minAllowedScale || root.localScale.z < minAllowedScale)
        {
            Debug.LogWarning($"Fixing out-of-bounds scale on: {root.name}. Was: {root.localScale}");
            root.localScale = Vector3.one;
        }
        
        // Recursively fix children
        foreach (Transform child in root)
        {
            FixTransformHierarchy(child);
        }
    }
    
    bool IsValidTransform(Transform t)
    {
        // Check position
        if (float.IsNaN(t.localPosition.x) || float.IsNaN(t.localPosition.y) || float.IsNaN(t.localPosition.z) ||
            float.IsInfinity(t.localPosition.x) || float.IsInfinity(t.localPosition.y) || float.IsInfinity(t.localPosition.z))
        {
            return false;
        }
        
        // Check rotation
        if (float.IsNaN(t.localRotation.x) || float.IsNaN(t.localRotation.y) || 
            float.IsNaN(t.localRotation.z) || float.IsNaN(t.localRotation.w) ||
            float.IsInfinity(t.localRotation.x) || float.IsInfinity(t.localRotation.y) || 
            float.IsInfinity(t.localRotation.z) || float.IsInfinity(t.localRotation.w))
        {
            return false;
        }
        
        // Check scale
        if (float.IsNaN(t.localScale.x) || float.IsNaN(t.localScale.y) || float.IsNaN(t.localScale.z) ||
            float.IsInfinity(t.localScale.x) || float.IsInfinity(t.localScale.y) || float.IsInfinity(t.localScale.z))
        {
            return false;
        }
        
        return true;
    }
    
    void ResetTransform(Transform t)
    {
        t.localPosition = Vector3.zero;
        t.localRotation = Quaternion.identity;
        t.localScale = Vector3.one;
    }
    
    void DisableAllCollidersInHierarchy()
    {
        Collider[] colliders = GetComponentsInChildren<Collider>(true);
        
        foreach (Collider col in colliders)
        {
            if (col.enabled)
            {
                col.enabled = false;
                disabledColliders.Add(col);
                Debug.Log($"Disabled collider on: {col.name} at path: {GetPath(col.transform)}");
            }
        }
    }
    
    void DisableAllRigidbodiesInHierarchy()
    {
        Rigidbody[] rigidbodies = GetComponentsInChildren<Rigidbody>(true);
        
        foreach (Rigidbody rb in rigidbodies)
        {
            if (!rb.isKinematic) // Only process non-kinematic rigidbodies
            {
                rb.isKinematic = true; // Make kinematic to disable physics
                disabledRigidbodies.Add(rb);
                Debug.Log($"Made rigidbody kinematic on: {rb.name}");
            }
        }
    }
    
    [ContextMenu("Restore Original")]
    public void RestoreOriginal()
    {
        // Restore transforms
        foreach (var kvp in originalTransforms)
        {
            if (kvp.Key != null)
            {
                kvp.Key.localPosition = kvp.Value.position;
                kvp.Key.localRotation = kvp.Value.rotation;
                kvp.Key.localScale = kvp.Value.scale;
            }
        }
        
        // Re-enable colliders
        foreach (Collider col in disabledColliders)
        {
            if (col != null)
            {
                col.enabled = true;
            }
        }
        
        // Re-enable rigidbodies
        foreach (Rigidbody rb in disabledRigidbodies)
        {
            if (rb != null)
            {
                rb.isKinematic = false; // Restore physics
            }
        }
        
        disabledColliders.Clear();
        disabledRigidbodies.Clear();
        
        Debug.Log("Restored original GO2 hierarchy");
    }
    
    string GetPath(Transform t)
    {
        string path = t.name;
        Transform parent = t.parent;
        
        while (parent != null)
        {
            path = parent.name + "/" + path;
            parent = parent.parent;
        }
        
        return path;
    }
    
    void OnDestroy()
    {
        if (disabledColliders.Count > 0 || disabledRigidbodies.Count > 0)
        {
            RestoreOriginal();
        }
    }
} 