using UnityEngine;
using UnityEngine.UI;
using System.Runtime.InteropServices;
using System.Collections;
using System.IO;

public class TimeFrequencyVisualizer : MonoBehaviour
{
    [DllImport("geometry_processor")]
    private static extern void GetTimeFrequencyData(
        int timeSteps, 
        int frequencyBins, 
        [Out] float[] outputMagnitudes,
        [Out] float[] outputPhases,
        [Out] float[] outputTimeRange,
        [Out] float[] outputFreqRange
    );
    
    [Header("Visualization Settings")]
    [SerializeField] private int timeSteps = 100;
    [SerializeField] private int frequencyBins = 60;
    [SerializeField] private Vector2 plotSize = new Vector2(512, 256);
    [SerializeField] private float updateInterval = 0.1f; // Update frequency in seconds
    [SerializeField] private bool autoUpdate = true;
    [SerializeField] private KeyCode toggleKey = KeyCode.Tab;
    [SerializeField] private KeyCode updateKey = KeyCode.Space;
    [SerializeField] private KeyCode captureKey = KeyCode.P;
    [SerializeField] private bool invertFrequencyAxis = false; // Set to true if frequency axis appears inverted
    
    [Header("UI References")]
    [SerializeField] private RawImage displayImage;
    [SerializeField] private RectTransform displayRect;
    [SerializeField] private Text infoText;
    
    [Header("Color Settings")]
    [SerializeField] private Color backgroundColor = Color.black;
    [SerializeField] private Color gridColor = new Color(0.3f, 0.3f, 0.3f, 0.5f);
    [SerializeField] private Color textColor = Color.white;
    [SerializeField, Range(0f, 1f)] private float minBrightness = 0.1f;
    [SerializeField, Range(0f, 1f)] private float maxBrightness = 1.0f;
    [SerializeField, Range(0f, 1f)] private float phaseColorSaturation = 0.8f;
    
    // Runtime data
    private Texture2D plotTexture;
    private float[] magnitudes;
    private float[] phases;
    private float[] timeRange = new float[2];
    private float[] freqRange = new float[2];
    private float lastUpdateTime;
    private bool isVisible = true;
    private GUIStyle labelStyle;
    
    // Frequency division lines (for labeling)
    private float[] frequencyDivisions = { 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000 };
    
    void Start()
    {
        // Initialize texture
        plotTexture = new Texture2D((int)plotSize.x, (int)plotSize.y, TextureFormat.RGBA32, false);
        magnitudes = new float[timeSteps * frequencyBins];
        phases = new float[timeSteps * frequencyBins];
        
        // Setup UI elements if assigned
        if (displayImage != null)
        {
            displayImage.texture = plotTexture;
            if (displayRect == null)
                displayRect = displayImage.rectTransform;
        }
        
        // Setup GUI style for drawing directly
        labelStyle = new GUIStyle();
        labelStyle.normal.textColor = textColor;
        labelStyle.fontSize = 12;
        
        // Force initial update
        UpdateVisualization();
        
        // Start automatic updates if enabled
        if (autoUpdate)
            StartCoroutine(AutoUpdateRoutine());
    }
    
    void Update()
    {
        // Toggle visibility
        if (Input.GetKeyDown(toggleKey))
        {
            isVisible = !isVisible;
            if (displayImage != null)
                displayImage.enabled = isVisible;
        }
        
        // Manual update
        if (Input.GetKeyDown(updateKey))
        {
            UpdateVisualization();
        }
        
        // Capture screenshot
        if (Input.GetKeyDown(captureKey))
        {
            CaptureVisualization();
        }
    }
    
    IEnumerator AutoUpdateRoutine()
    {
        while (autoUpdate)
        {
            yield return new WaitForSeconds(updateInterval);
            UpdateVisualization();
        }
    }
    
    public void UpdateVisualization()
    {
        // Get the time-frequency data
        GetTimeFrequencyData(timeSteps, frequencyBins, magnitudes, phases, timeRange, freqRange);
        
        // Clear the texture with background color
        Color[] pixels = new Color[plotTexture.width * plotTexture.height];
        for (int i = 0; i < pixels.Length; i++)
            pixels[i] = backgroundColor;
        plotTexture.SetPixels(pixels);
        
        // Draw the grid
        DrawGrid();
        
        // Draw the data
        DrawData();
        
        // Apply the changes to the texture
        plotTexture.Apply();
        
        // Update info text if available
        if (infoText != null)
        {
            infoText.text = $"Time: {timeRange[0]:F2}s to {timeRange[1]:F2}s\nFreq: {freqRange[0]:F0}Hz to {freqRange[1]:F0}Hz";
        }
        
        // Update timestamp
        lastUpdateTime = Time.time;
    }
    
    private void DrawGrid()
    {
        // Draw time grid (vertical lines)
        int timeLines = 5;
        for (int i = 0; i < timeLines; i++)
        {
            int x = (int)(plotTexture.width * i / (float)(timeLines - 1));
            for (int y = 0; y < plotTexture.height; y++)
            {
                plotTexture.SetPixel(x, y, gridColor);
            }
        }
        
        // Draw frequency grid (horizontal lines)
        // Calculate pixel positions for specific frequencies
        float freqRange = this.freqRange[1] - this.freqRange[0];
        foreach (float freq in frequencyDivisions)
        {
            if (freq >= this.freqRange[0] && freq <= this.freqRange[1])
            {
                float normalizedFreq = (freq - this.freqRange[0]) / freqRange;
                // Invert if needed (0 = low freq, 1 = high freq)
                if (invertFrequencyAxis)
                    normalizedFreq = 1.0f - normalizedFreq;
                
                int y = Mathf.RoundToInt(normalizedFreq * plotTexture.height);
                
                // Draw line
                for (int x = 0; x < plotTexture.width; x++)
                {
                    if (y >= 0 && y < plotTexture.height)
                    {
                        plotTexture.SetPixel(x, y, gridColor);
                    }
                }
                
                // Add frequency label on the left
                string label = $"{freq:F0}Hz";
                // We'll draw this in OnGUI
            }
        }
    }
    
    // Convert phase (0-2π) to HSV color
    private Color PhaseToColor(float phase, float magnitude)
    {
        // Normalize phase to 0-1 range for hue
        float hue = phase / (2.0f * Mathf.PI);
        
        // Map magnitude to value (brightness), with a minimum brightness
        float value = Mathf.Lerp(minBrightness, maxBrightness, magnitude);
        
        // Create color with fixed saturation
        return Color.HSVToRGB(hue, phaseColorSaturation, value);
    }
    
    private void DrawData()
    {
        // Map frequency bin to y coordinate based on settings
        int GetYCoordinate(int freqBin)
        {
            float normalizedPos = (float)freqBin / (frequencyBins - 1);
            if (invertFrequencyAxis)
                normalizedPos = 1.0f - normalizedPos; // Invert if needed
            return Mathf.RoundToInt(normalizedPos * (plotTexture.height - 1));
        }
        
        // Draw the data
        for (int t = 0; t < timeSteps; t++)
        {
            // Map time step to x coordinate
            int x = Mathf.RoundToInt(t * (plotTexture.width - 1) / (timeSteps - 1));
            
            for (int f = 0; f < frequencyBins; f++)
            {
                // Get y coordinate based on frequency
                int y = GetYCoordinate(f);
                
                // Get data values
                int index = t * frequencyBins + f;
                float magnitude = magnitudes[index];
                float phase = phases[index];
                
                // Skip very low magnitudes
                if (magnitude < 0.01f) continue;
                
                // Calculate color based on phase and magnitude
                Color color = PhaseToColor(phase, magnitude);
                
                // Set pixel if it's within bounds
                if (x >= 0 && x < plotTexture.width && y >= 0 && y < plotTexture.height)
                {
                    // Set the main pixel
                    plotTexture.SetPixel(x, y, color);
                    
                    // For stronger signals, color neighboring pixels too (with reduced intensity)
                    if (magnitude > 0.3f)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            for (int dy = -1; dy <= 1; dy++)
                            {
                                // Skip the center pixel (already set)
                                if (dx == 0 && dy == 0) continue;
                                
                                int nx = x + dx;
                                int ny = y + dy;
                                
                                if (nx >= 0 && nx < plotTexture.width && ny >= 0 && ny < plotTexture.height)
                                {
                                    // Blend with existing color, using a reduced intensity
                                    Color existing = plotTexture.GetPixel(nx, ny);
                                    float blendFactor = magnitude * 0.3f; // Reduce intensity for neighbors
                                    plotTexture.SetPixel(nx, ny, Color.Lerp(existing, color, blendFactor));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    void OnGUI()
    {
        if (!isVisible)
            return;
            
        // Only draw GUI if not using UI components or they're not assigned
        if (displayImage != null && displayImage.enabled)
            return;
            
        // Fallback drawing directly to the screen
        if (displayImage == null)
        {
            // Draw in top-right corner
            float margin = 20;
            Rect rect = new Rect(Screen.width - plotSize.x - margin, margin, plotSize.x, plotSize.y);
            
            // Draw background and plot
            GUI.DrawTexture(rect, plotTexture);
            
            // Draw labels
            float labelHeight = 20;
            
            // Title
            GUI.Label(new Rect(rect.x, rect.y - labelHeight, 200, labelHeight), 
                    "Time-Frequency Analysis", labelStyle);
            
            // Time labels
            GUI.Label(new Rect(rect.x, rect.y + rect.height, 100, labelHeight), 
                    $"{timeRange[0]:F2}s", labelStyle);
            GUI.Label(new Rect(rect.x + rect.width - 50, rect.y + rect.height, 100, labelHeight), 
                    $"{timeRange[1]:F2}s", labelStyle);
            
            // Frequency labels
            float freqRange = this.freqRange[1] - this.freqRange[0];
            foreach (float freq in frequencyDivisions)
            {
                if (freq >= this.freqRange[0] && freq <= this.freqRange[1])
                {
                    float normalizedFreq = (freq - this.freqRange[0]) / freqRange;
                    if (invertFrequencyAxis)
                        normalizedFreq = 1.0f - normalizedFreq;
                    
                    float y = rect.y + rect.height * (1.0f - normalizedFreq);
                    GUI.Label(new Rect(rect.x - 50, y - 10, 50, 20), $"{freq:F0}Hz", labelStyle);
                }
            }
            
            // // Phase-color legend
            // float legendWidth = 100;
            // float legendHeight = 15;
            // Rect legendRect = new Rect(rect.x, rect.y + rect.height + 20, legendWidth, legendHeight);
            
            // // Draw legend background
            // GUI.DrawTexture(legendRect, Texture2D.whiteTexture, ScaleMode.StretchToFill, true, 0, Color.black, 0, 0);
            
            // // Draw phase colors
            // for (int i = 0; i < legendWidth; i++)
            // {
            //     float phase = (i / (float)legendWidth) * 2.0f * Mathf.PI;
            //     Color color = PhaseToColor(phase, 1.0f); // Full magnitude
                
            //     Rect colorRect = new Rect(legendRect.x + i, legendRect.y, 1, legendHeight);
            //     GUI.DrawTexture(colorRect, Texture2D.whiteTexture, ScaleMode.StretchToFill, true, 0, color, 0, 0);
            // }
            
            // // Legend labels
            // GUI.Label(new Rect(legendRect.x, legendRect.y + legendHeight, 50, 20), "0", labelStyle);
            // GUI.Label(new Rect(legendRect.x + legendWidth - 20, legendRect.y + legendHeight, 50, 20), "2π", labelStyle);
            // GUI.Label(new Rect(legendRect.x + 40, legendRect.y + legendHeight, 100, 20), "Phase", labelStyle);
        }
    }
    
    void CaptureVisualization()
    {
        // Create a temporary texture to render to
        RenderTexture rt = new RenderTexture((int)plotSize.x, (int)plotSize.y, 24);
        RenderTexture prev = RenderTexture.active;
        RenderTexture.active = rt;
        
        // Create texture for the capture
        Texture2D captureTexture = new Texture2D((int)plotSize.x, (int)plotSize.y, TextureFormat.RGB24, false);
        
        // Clear with black background
        GL.Clear(true, true, Color.black);
        
        // Draw the texture
        Graphics.Blit(plotTexture, rt);
        
        // Read pixels
        captureTexture.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        captureTexture.Apply();
        
        // Restore previous RenderTexture
        RenderTexture.active = prev;
        
        // Save as PNG
        byte[] bytes = captureTexture.EncodeToPNG();
        string timestamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string filename = $"TFPlot_{timestamp}.png";
        string path = Path.Combine(Application.persistentDataPath, filename);
        File.WriteAllBytes(path, bytes);
        
        Debug.Log($"Time-frequency plot saved to: {path}");
        
        // Clean up
        Destroy(captureTexture);
        rt.Release();
    }
}