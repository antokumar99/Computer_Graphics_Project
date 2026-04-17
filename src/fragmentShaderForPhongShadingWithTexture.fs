#version 330 core
// Phong shading + texture + spot light + component toggles
// Theory: Illumination Model and Shading.pptx

out vec4 FragColor;

// ---- Material ----
struct Material {
    vec3  ambient;
    vec3  diffuse;
    vec3  specular;
    float shininess;
};

// ---- Point light with attenuation ----
struct PointLight {
    vec3  position;
    vec3  ambient;
    vec3  diffuse;
    vec3  specular;
    float constant;
    float linear;
    float quadratic;
};

// ---- Spot light (single cut-off cone) ----
struct SpotLight {
    vec3  position;
    vec3  direction;
    float cutOff;       // cos(inner angle)
    float outerCutOff;  // cos(outer angle - soft edge)
    vec3  ambient;
    vec3  diffuse;
    vec3  specular;
    float constant;
    float linear;
    float quadratic;
};

#define NR_POINT_LIGHTS 12

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

uniform vec3        viewPos;
uniform PointLight  pointLights[NR_POINT_LIGHTS];
uniform SpotLight   spotLight;

// ---- Section spotlights (paint booths, showroom, body shop, assembly) ----
#define NR_SECTION_SPOTS 10
uniform SpotLight   sectionSpots[NR_SECTION_SPOTS];
uniform bool        sectionSpotsOn;

uniform Material    material;
uniform sampler2D   texUnit;
uniform bool        useTexture;

// ---- Directional light ----
uniform bool dirLightOn;
uniform vec3 dirLightDir;
uniform vec3 dirLightAmb;
uniform vec3 dirLightDiff;
uniform vec3 dirLightSpec;

// ---- Spot light on/off (key 3) ----
uniform bool spotLightOn;

// ---- Per-component toggles (keys 5 / 6 / 7) ----
uniform bool ambientOn;
uniform bool diffuseOn;
uniform bool specularOn;

// ---- Emissive override ----
uniform bool emissive;
uniform vec3 emissiveColor;
uniform bool emissiveGlobalOn;   // key 8 – toggle all emissive glow

// ---- Declarations ----
vec3 CalcPointLight(PointLight light, vec3 N, vec3 fragPos, vec3 V,
                    vec3 matAmb, vec3 matDiff, vec3 matSpec);
vec3 CalcDirLight  (vec3 N, vec3 V,
                    vec3 matAmb, vec3 matDiff, vec3 matSpec);
vec3 CalcSpotLight (SpotLight light, vec3 N, vec3 fragPos, vec3 V,
                    vec3 matAmb, vec3 matDiff, vec3 matSpec);

void main()
{
    if (emissive) {
        if (emissiveGlobalOn)
            FragColor = vec4(emissiveColor, 1.0);           // full glow
        else
            FragColor = vec4(emissiveColor * 0.04, 1.0);    // lights off – dark shell
        return;
    }

    vec3 texColor = vec3(1.0);
    if (useTexture) texColor = vec3(texture(texUnit, TexCoords));

    vec3 matAmb  = material.ambient  * texColor;
    vec3 matDiff = material.diffuse  * texColor;
    vec3 matSpec = material.specular;

    // Component toggles
    if (!ambientOn)  matAmb  = vec3(0.0);
    if (!diffuseOn)  matDiff = vec3(0.0);
    if (!specularOn) matSpec = vec3(0.0);

    vec3 N = normalize(Normal);
    vec3 V = normalize(viewPos - FragPos);

    vec3 result = vec3(0.0);

    if (dirLightOn)
        result += CalcDirLight(N, V, matAmb, matDiff, matSpec);

    for (int i = 0; i < NR_POINT_LIGHTS; i++)
        result += CalcPointLight(pointLights[i], N, FragPos, V,
                                 matAmb, matDiff, matSpec);

    if (spotLightOn)
        result += CalcSpotLight(spotLight, N, FragPos, V,
                                matAmb, matDiff, matSpec);

    if (sectionSpotsOn)
        for (int i = 0; i < NR_SECTION_SPOTS; i++)
            result += CalcSpotLight(sectionSpots[i], N, FragPos, V,
                                    matAmb, matDiff, matSpec);

    FragColor = vec4(result, 1.0);
}

vec3 CalcPointLight(PointLight light, vec3 N, vec3 fragPos, vec3 V,
                    vec3 matAmb, vec3 matDiff, vec3 matSpec)
{
    if (dot(light.ambient + light.diffuse + light.specular, vec3(1.0)) < 0.001)
        return vec3(0.0);

    vec3 L = normalize(light.position - fragPos);
    vec3 R = reflect(-L, N);

    float dist  = length(light.position - fragPos);
    float atten = 1.0 / (light.constant
                       + light.linear    * dist
                       + light.quadratic * dist * dist);

    vec3 ambient  = matAmb  * light.ambient;
    vec3 diffuse  = matDiff * max(dot(N, L), 0.0) * light.diffuse;
    vec3 specular = matSpec * pow(max(dot(V, R), 0.0), material.shininess)
                            * light.specular;

    return (ambient + diffuse + specular) * atten;
}

vec3 CalcDirLight(vec3 N, vec3 V,
                  vec3 matAmb, vec3 matDiff, vec3 matSpec)
{
    vec3 L = normalize(-dirLightDir);
    vec3 R = reflect(-L, N);

    vec3 ambient  = matAmb  * dirLightAmb;
    vec3 diffuse  = matDiff * max(dot(N, L), 0.0) * dirLightDiff;
    vec3 specular = matSpec * pow(max(dot(V, R), 0.0), material.shininess)
                            * dirLightSpec;

    return ambient + diffuse + specular;
}

vec3 CalcSpotLight(SpotLight light, vec3 N, vec3 fragPos, vec3 V,
                   vec3 matAmb, vec3 matDiff, vec3 matSpec)
{
    vec3  L     = normalize(light.position - fragPos);
    float theta = dot(L, normalize(-light.direction));

    if (theta < light.outerCutOff)
        return vec3(0.0);

    float eps       = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / eps, 0.0, 1.0);

    vec3  R     = reflect(-L, N);
    float dist  = length(light.position - fragPos);
    float atten = 1.0 / (light.constant
                       + light.linear    * dist
                       + light.quadratic * dist * dist);

    vec3 ambient  = matAmb  * light.ambient;
    vec3 diffuse  = matDiff * max(dot(N, L), 0.0) * light.diffuse  * intensity;
    vec3 specular = matSpec * pow(max(dot(V, R), 0.0), material.shininess)
                            * light.specular * intensity;

    return (ambient + diffuse + specular) * atten;
}
