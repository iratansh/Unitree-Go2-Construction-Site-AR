using UnityEngine;

public class SimpleGo2Controller : MonoBehaviour
{
    [Header("Simple GO2 Setup")]
    public GameObject simplifiedGO2;
    public bool createSimpleGO2 = true;
    
    [Header("Path Configuration")]
    public float pathLength = 10f;
    public float speed = 0.5f;
    
    [Header("Controls")]
    public KeyCode startKey = KeyCode.Space;
    public KeyCode stopKey = KeyCode.S;
    public KeyCode resetKey = KeyCode.R;
    
    private Vector3 startPosition;
    private Vector3 endPosition;
    private bool isMoving = false;
    private float progress = 0f;
    
    void Start()
    {
        if (createSimpleGO2 && simplifiedGO2 == null)
        {
            CreateSimpleGO2();
        }
        
        SetupPath();
    }
    
    void CreateSimpleGO2()
    {
        // Create a simple representation
        simplifiedGO2 = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        simplifiedGO2.name = "Simple GO2";
        simplifiedGO2.transform.localScale = new Vector3(0.4f, 0.5f, 0.8f);
        
        // Add visual distinction
        Renderer renderer = simplifiedGO2.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = new Color(0.3f, 0.3f, 0.8f);
        }
        
        // Add a "head" to show direction
        GameObject head = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        head.name = "Head";
        head.transform.parent = simplifiedGO2.transform;
        head.transform.localPosition = new Vector3(0, 0.5f, 0.5f);
        head.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
        
        Renderer headRenderer = head.GetComponent<Renderer>();
        if (headRenderer != null)
        {
            headRenderer.material.color = Color.red;
        }
        
        Debug.Log("Created simplified GO2 model");
    }
    
    void SetupPath()
    {
        startPosition = transform.position;
        endPosition = startPosition + Vector3.forward * pathLength;
        
        if (simplifiedGO2 != null)
        {
            simplifiedGO2.transform.position = startPosition;
        }
        
        // Create visual markers
        CreateMarker(startPosition, Color.green, "Start");
        CreateMarker(endPosition, Color.red, "End");
        
        // Create path line
        GameObject pathLine = new GameObject("Path Line");
        LineRenderer lr = pathLine.AddComponent<LineRenderer>();
        lr.positionCount = 2;
        lr.SetPosition(0, startPosition);
        lr.SetPosition(1, endPosition);
        lr.startWidth = 0.1f;
        lr.endWidth = 0.1f;
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.startColor = Color.yellow;
        lr.endColor = Color.yellow;
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
        
        // Remove collider
        Destroy(marker.GetComponent<Collider>());
    }
    
    void Update()
    {
        HandleInput();
        
        if (isMoving && simplifiedGO2 != null)
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
        
        if (simplifiedGO2 != null)
        {
            simplifiedGO2.transform.position = startPosition;
            simplifiedGO2.transform.rotation = Quaternion.identity;
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
        simplifiedGO2.transform.position = Vector3.Lerp(startPosition, endPosition, progress);
        
        // Add simple animation
        float bob = Mathf.Sin(Time.time * 4f) * 0.02f;
        simplifiedGO2.transform.position += Vector3.up * bob;
        
        // Simple rotation
        float sway = Mathf.Sin(Time.time * 2f) * 5f;
        simplifiedGO2.transform.rotation = Quaternion.Euler(0, 0, sway);
    }
    
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 150));
        GUILayout.Label("Simple GO2 Controller");
        GUILayout.Label($"Progress: {(progress * 100f):F1}%");
        GUILayout.Label($"Position: {(simplifiedGO2 != null ? simplifiedGO2.transform.position.ToString() : "N/A")}");
        GUILayout.Label("Controls:");
        GUILayout.Label("Space - Start | S - Stop | R - Reset");
        GUILayout.EndArea();
    }
} 