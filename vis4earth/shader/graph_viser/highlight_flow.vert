#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in int vertexIndex;
layout (location = 2) in vec4 originalColor;
layout(location=3) in int edgeID;

uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform float time;
uniform float speed;
uniform float scale;

out vec4 fragColor;

void main() {
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    
    // 计算高光位置
    float highlightPos = mod(time * speed, 1.0) * 100.0;
    float dist = abs(float(vertexIndex) - highlightPos);
    float intensity = max(0.0, scale - dist / 9.0);
    
    // 颜色插值
    vec3 highlightColor = vec3(1.0, 1.0, 1.0);
    vec3 interpolatedRGB = highlightColor * intensity + originalColor.rgb * (1.0 - intensity);
    
    // 透明度计算
    float alpha = (dist == 0.0) ? 1.0 : (originalColor.a * (1.0 - intensity) + 1.0 * intensity);
    
    fragColor = vec4(interpolatedRGB, alpha);
}
