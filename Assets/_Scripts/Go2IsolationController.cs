using UnityEngine;
using System.Collections.Generic;

public class Go2IsolationController : MonoBehaviour
{
    [Header("GO2 Isolation")]
    public GameObject originalGO2;
    public GameObject cleanGO2;
    public bool createCleanCopy = true;
    public bool hideOriginal = true;
    
    [Header("Path Configuration")]
    public float pathLength = 10f;
    public float speed = 0.5f;
    public Transform startPoint;
    public Transform endPoint;
    
    [Header("Controls")]
    public KeyCode startKey = KeyCode.Space;
    public KeyCode stopKey = KeyCode.S;
    public KeyCode resetKey = KeyCode.R;
    
    private Vector3 startPosition;
    private bool isMoving = false;
    private float progress = 0f;
    private LineRenderer pathVisualizer;
    
    void Start()
    {
        if (originalGO2 == null)
        {
            originalGO2 = GameObject.Find("go2_description");
        }
        
        if (createCleanCopy && originalGO2 != null)
        {
            CreateCleanGO2Copy();
        }
        
        SetupPath();
    }
    
    void CreateCleanGO2Copy()
    {
        Debug.Log("Creating clean GO2 copy...");
        
        // Create new clean GameObject
        cleanGO2 = new GameObject("Clean GO2");
        cleanGO2.transform.position = originalGO2.transform.position;
        cleanGO2.transform.rotation = originalGO2.transform.rotation;
        
        // Copy only the mesh renderers (visual components)
        CopyMeshesRecursively(originalGO2.transform, cleanGO2.transform);
        
        // Hide the original problematic GO2
        if (hideOriginal)
        {
            originalGO2.SetActive(false);
            Debug.Log("Hidden original GO2");
        }
        
        Debug.Log("Clean GO2 copy created successfully");
    }
    
    void CopyMeshesRecursively(Transform source, Transform destination)
    {
        // Copy mesh from current object
        MeshRenderer sourceMR = source.GetComponent<MeshRenderer>();
        MeshFilter sourceMF = source.GetComponent<MeshFilter>();
        
        if (sourceMR != null && sourceMF != null && sourceMF.mesh != null)
        {
            // Create a clean GameObject for this mesh
            GameObject meshObj = new GameObject(source.name + "_Mesh");
            meshObj.transform.parent = destination;
            meshObj.transform.localPosition = source.localPosition;
            meshObj.transform.localRotation = source.localRotation;
            meshObj.transform.localScale = source.localScale;
            
            // Validate the transform before applying
            if (IsValidTransform(meshObj.transform))
            {
                // Copy mesh renderer
                MeshRenderer newMR = meshObj.AddComponent<MeshRenderer>();
                newMR.materials = sourceMR.materials;
                
                // Copy mesh filter
                MeshFilter newMF = meshObj.AddComponent<MeshFilter>();
                newMF.mesh = sourceMF.mesh;
                
                Debug.Log($"Copied mesh: {source.name}");
            }
            else
            {
                Debug.LogWarning($"Skipped invalid mesh: {source.name}");
                DestroyImmediate(meshObj);
            }
        }
        
        // Recursively copy children meshes
        foreach (Transform child in source)
        {
            // Create child container
            GameObject childContainer = new GameObject(child.name);
            childContainer.transform.parent = destination;
            childContainer.transform.localPosition = Vector3.zero;
            childContainer.transform.localRotation = Quaternion.identity;
            childContainer.transform.localScale = Vector3.one;
            
            CopyMeshesRecursively(child, childContainer.transform);
            
            // Remove empty containers
            if (childContainer.transform.childCount == 0)
            {
                DestroyImmediate(childContainer);
            }
        }
    }
    
    bool IsValidTransform(Transform t)
    {
        Vector3 pos = t.localPosition;
        Vector3 scale = t.localScale;
        Quaternion rot = t.localRotation;
        
        // Check for NaN or Infinity
        if (float.IsNaN(pos.x) || float.IsNaN(pos.y) || float.IsNaN(pos.z) ||
            float.IsInfinity(pos.x) || float.IsInfinity(pos.y) || float.IsInfinity(pos.z) ||
            float.IsNaN(scale.x) || float.IsNaN(scale.y) || float.IsNaN(scale.z) ||
            float.IsInfinity(scale.x) || float.IsInfinity(scale.y) || float.IsInfinity(scale.z) ||
            float.IsNaN(rot.x) || float.IsNaN(rot.y) || float.IsNaN(rot.z) || float.IsNaN(rot.w) ||
            float.IsInfinity(rot.x) || float.IsInfinity(rot.y) || float.IsInfinity(rot.z) || float.IsInfinity(rot.w))
        {
            return false;
        }
        
        // Check for extreme values
        if (pos.magnitude > 1000f || scale.magnitude > 100f || scale.magnitude < 0.001f)
        {
            return false;
        }
        
        return true;
    }
    
    void SetupPath()
    {
        if (startPoint == null)
        {
            GameObject startGO = new GameObject("Start Point");
            startGO.transform.position = transform.position;
            startPoint = startGO.transform;
        }
        
        if (endPoint == null)
        {
            GameObject endGO = new GameObject("End Point");
            endGO.transform.position = startPoint.position + Vector3.forward * pathLength;
            endPoint = endGO.transform;
        }
        
        startPosition = startPoint.position;
        
        if (cleanGO2 != null)
        {
            cleanGO2.transform.position = startPosition;
        }
        
        // Create path visualizer
        GameObject pathLine = new GameObject("Path Line");
        pathVisualizer = pathLine.AddComponent<LineRenderer>();
        pathVisualizer.positionCount = 2;
        pathVisualizer.SetPosition(0, startPoint.position);
        pathVisualizer.SetPosition(1, endPoint.position);
        pathVisualizer.startWidth = 0.1f;
        pathVisualizer.endWidth = 0.1f;
        pathVisualizer.material = new Material(Shader.Find("Sprites/Default"));
        pathVisualizer.startColor = Color.green;
        pathVisualizer.endColor = Color.red;
        
        // Create markers
        CreateMarker(startPoint.position, Color.green, "Start");
        CreateMarker(endPoint.position, Color.red, "End");
    }
    
    void CreateMarker(Vector3 position, Color color, string label)
    {
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = label + " Marker";
        marker.transform.position = position;
        marker.transform.localScale = Vector3.one * 0.3f;
        
        Renderer renderer = marker.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material = new Material(Shader.Find("Standard"));
            renderer.material.color = color;
        }
        
        Destroy(marker.GetComponent<Collider>());
    }
    
    void Update()
    {
        HandleInput();
        
        if (isMoving && cleanGO2 != null)
        {
            UpdateMovement();
        }
    }
    
    void HandleInput()
    {
        if (Input.GetKeyDown(startKey))
        {
            StartMovement();
        }
        
        if (Input.GetKeyDown(stopKey))
        {
            StopMovement();
        }
        
        if (Input.GetKeyDown(resetKey))
        {
            ResetPosition();
        }
    }
    
    void StartMovement()
    {
        isMoving = true;
        Debug.Log("Started movement");
    }
    
    void StopMovement()
    {
        isMoving = false;
        Debug.Log("Stopped movement");
    }
    
    void ResetPosition()
    {
        isMoving = false;
        progress = 0f;
        
        if (cleanGO2 != null)
        {
            cleanGO2.transform.position = startPosition;
            cleanGO2.transform.rotation = Quaternion.identity;
        }
        
        Debug.Log("Reset position");
    }
    
    void UpdateMovement()
    {
        // Update progress
        progress += (speed / pathLength) * Time.deltaTime;
        
        if (progress >= 1f)
        {
            progress = 1f;
            isMoving = false;
            Debug.Log("Reached end of path");
        }
        
        // Update position
        Vector3 targetPos = Vector3.Lerp(startPosition, endPoint.position, progress);
        cleanGO2.transform.position = targetPos;
        
        // Add simple walking animation
        float walkBob = Mathf.Sin(Time.time * 8f) * 0.01f;
        cleanGO2.transform.position += Vector3.up * walkBob;
        
        // Simple body sway
        float sway = Mathf.Sin(Time.time * 3f) * 2f;
        cleanGO2.transform.rotation = Quaternion.Euler(0, 0, sway);
    }
    
    [ContextMenu("Recreate Clean GO2")]
    public void RecreateCleanGO2()
    {
        if (cleanGO2 != null)
        {
            DestroyImmediate(cleanGO2);
        }
        
        if (originalGO2 != null)
        {
            originalGO2.SetActive(true);
            CreateCleanGO2Copy();
        }
    }
    
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 150));
        GUILayout.Label("GO2 Isolation Controller");
        GUILayout.Label($"Progress: {(progress * 100f):F1}%");
        GUILayout.Label($"Clean GO2: {(cleanGO2 != null ? "Active" : "None")}");
        GUILayout.Label($"Original GO2: {(originalGO2 != null ? (originalGO2.activeInHierarchy ? "Active" : "Hidden") : "None")}");
        GUILayout.Label("Controls: Space-Start | S-Stop | R-Reset");
        GUILayout.EndArea();
    }
} 